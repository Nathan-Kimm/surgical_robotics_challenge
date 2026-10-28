#!/usr/bin/env python3
"""Rigidly move a PSM (or any multi-body ADF file) in the world.

Every BODY's `location.position` in the file is shifted by the same vector.
Orientations, inertial offsets and joint pivots are untouched, so the arm's
internal geometry is preserved and nothing snaps at start-up. (Moving only
the base link would make the joints drag the other links to it on load.)

The direction is computed per file, from that file's anchor body (default
`base link`, i.e. the RCM) to the target point, so psm1 and psm2 each move
along their own line of sight to the phantom.

Examples (run from the repo root):

  # put both LND_420006 arms' RCM exactly 20 cm from the Simple phantom (safe to re-run)
  tools/move_psm.py ADF/PSMs/LND_420006/psm1.yaml ADF/PSMs/LND_420006/psm2.yaml \\
      --toward ADF/Phantoms/Simple/phantom.yaml Barrier --distance 0.20

  # or move 10 cm closer from wherever the arm is now (cumulative!)
  tools/move_psm.py ADF/PSMs/LND_420006/psm1.yaml ADF/PSMs/LND_420006/psm2.yaml \\
      --toward ADF/Phantoms/Simple/phantom.yaml Barrier --by 0.10

  # toward an explicit world point / by an explicit vector (metres, world frame)
  tools/move_psm.py ADF/PSMs/LND_400006_SIMPLE/psm1.yaml --to-point 0.03 0.26 0.74 --distance 0.22
  tools/move_psm.py ADF/PSMs/LND_420006/psm1.yaml --delta 0 -0.05 0

  # preview without writing
  tools/move_psm.py ... --dry-run

Edits are in place. `--distance` is idempotent (re-running lands in the same
place); `--by` and `--delta` are cumulative (running twice moves twice).
Use git to reset a file to its original pose.
"""
import argparse
import math
import re
import sys

POS_RE = re.compile(r'^(      )([xyz]):\s*(\S+)\s*$')


def walk(lines, on_coord):
    """Call on_coord(body, axis, value, line_idx) for every BODY location.position coordinate."""
    body = None
    in_loc = in_pos = False
    for i, line in enumerate(lines):
        if re.match(r'^\S', line):                      # top-level key (BODY ..., JOINT ..., etc.)
            body = line.strip().rstrip(':') if line.startswith('BODY ') else None
            in_loc = in_pos = False
        elif body is None:
            continue
        elif re.match(r'^  location:\s*$', line):
            in_loc, in_pos = True, False
        elif re.match(r'^  \S', line):                  # any other 2-space key (inertial offset, mass, ...)
            in_loc = in_pos = False
        elif in_loc and re.match(r'^    position:\s*$', line):
            in_pos = True
        elif in_loc and re.match(r'^    \S', line):     # orientation:
            in_pos = False
        elif in_pos:
            m = POS_RE.match(line)
            if m:
                on_coord(body, m.group(2), float(m.group(3)), i)


def body_position(lines, name):
    """World position of `BODY <name>` in an ADF file, or None."""
    want = name if name.startswith('BODY ') else 'BODY ' + name
    found = {}
    walk(lines, lambda b, ax, v, i: found.__setitem__(ax, v) if b == want else None)
    return (found['x'], found['y'], found['z']) if len(found) == 3 else None


def read_lines(path):
    with open(path) as f:
        return f.read().splitlines(keepends=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+', metavar='PSM_YAML', help='ADF file(s) to move, edited in place')
    how = ap.add_mutually_exclusive_group(required=True)
    how.add_argument('--toward', nargs=2, metavar=('ADF_YAML', 'BODY'),
                     help='move toward the world position of BODY in ADF_YAML (needs --by)')
    how.add_argument('--to-point', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                     help='move toward this world point (needs --by)')
    how.add_argument('--delta', nargs=3, type=float, metavar=('DX', 'DY', 'DZ'),
                     help='shift by this world-frame vector (metres)')
    amount = ap.add_mutually_exclusive_group()
    amount.add_argument('--distance', type=float, metavar='METRES',
                        help='end with the anchor exactly this far from the target (idempotent)')
    amount.add_argument('--by', type=float, metavar='METRES',
                        help='move this far along the direction from the current pose (cumulative; negative moves away)')
    ap.add_argument('--anchor', default='base link', metavar='BODY',
                    help='body whose position defines the direction (default: "base link")')
    ap.add_argument('--dry-run', action='store_true', help='report what would change, write nothing')
    args = ap.parse_args()

    if args.delta is None and args.by is None and args.distance is None:
        ap.error('--distance or --by is required with --toward / --to-point')
    if args.delta is not None and (args.by is not None or args.distance is not None):
        ap.error('--delta cannot be combined with --by / --distance')

    target = None
    if args.toward:
        tfile, tbody = args.toward
        target = body_position(read_lines(tfile), tbody)
        if target is None:
            sys.exit(f'error: BODY "{tbody}" with a location.position not found in {tfile}')
    elif args.to_point:
        target = tuple(args.to_point)

    for path in args.files:
        lines = read_lines(path)

        if args.delta is not None:
            delta = tuple(args.delta)
            note = ''
        else:
            anchor = body_position(lines, args.anchor)
            if anchor is None:
                sys.exit(f'error: anchor BODY "{args.anchor}" not found in {path}')
            d = tuple(target[k] - anchor[k] for k in range(3))
            dist = math.sqrt(sum(c * c for c in d))
            if dist == 0:
                sys.exit(f'error: anchor already at target in {path}')
            step = args.by if args.by is not None else dist - args.distance
            if dist - step <= 0:
                sys.exit(f'error: {path}: this would move the anchor to or past the target '
                         f'(currently {dist:.4f} m away); nothing written')
            delta = tuple(c / dist * step for c in d)
            note = f'  anchor->target {dist:.4f} m -> {dist - step:.4f} m'

        touched = {}

        def shift(body, ax, v, i):
            k = 'xyz'.index(ax)
            lines[i] = f'      {ax}: {v + delta[k]:.10f}\n'
            touched.setdefault(body, []).append(ax)

        walk(lines, shift)

        bad = [b for b, axes in touched.items() if sorted(axes) != ['x', 'y', 'z']]
        if bad:
            sys.exit(f'error: {path}: incomplete x/y/z under location.position for {bad}; nothing written')
        if not touched:
            sys.exit(f'error: {path}: no BODY with a location.position found; nothing written')

        print(f'{"[dry-run] " if args.dry_run else ""}{path}')
        if note:
            print(note)
        print(f'  delta = ({delta[0]:+.6f}, {delta[1]:+.6f}, {delta[2]:+.6f})')
        print(f'  {len(touched)} bodies: ' + ', '.join(b[len("BODY "):] for b in touched))

        if not args.dry_run:
            with open(path, 'w') as f:
                f.writelines(lines)


if __name__ == '__main__':
    main()
