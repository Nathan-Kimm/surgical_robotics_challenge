# tools

Offline helpers that edit ADF files. They don't need the simulator or the
`surgical_robotics_challenge` Python package.

## move_psm.py — move a PSM closer to / further from the phantom

Shifts **every** body in a PSM yaml by the same vector. Moving only the base
link makes the joints drag the rest of the arm to it on load (the arm visibly
jerks and rotates at start-up); shifting all links together keeps the arm's
geometry identical and nothing moves at init.

The direction is computed per file from the arm's `base link` (the RCM) to the
target point, so psm1 and psm2 each move along their own line toward the
phantom. Orientations are never changed.

```bash
# from the repo root
tools/move_psm.py <psm yaml...> --toward <phantom yaml> <BODY name> --distance <metres>   # preferred
tools/move_psm.py <psm yaml...> --toward <phantom yaml> <BODY name> --by <metres>
tools/move_psm.py <psm yaml...> --to-point X Y Z --distance|--by <metres>
tools/move_psm.py <psm yaml...> --delta DX DY DZ
tools/move_psm.py ... --dry-run        # preview only
```

- `--distance` sets how far the arm's `base link` (RCM) ends up from the target.
  It is **idempotent**: re-running with the same number lands in the same place,
  so it's the safe option. The stock LND_420006 arms sit ~0.31 m from the Simple
  phantom's `Barrier`.
- `--by` moves relative to wherever the arm is *now* and is **cumulative** —
  running it twice moves twice. A negative value moves the arm away.
- Edits are in place; reset with `git checkout -- <file>`.

Running `--by 0.10` twice puts the gripper tips *inside* the Simple phantom's
collision hull, which shows up as the arms jerking/rotating at start-up.

### Which files to edit per environment

| run script | phantom yaml / target body | PSM yamls |
|---|---|---|
| `run_env_SIMPLE_LND_420006.sh` | `ADF/Phantoms/Simple/phantom.yaml` · `Barrier` | `ADF/PSMs/LND_420006/psm{1,2}.yaml` |
| `run_env_3D_MED_COMPLEX_LND_420006.sh` | `ADF/Phantoms/3D_MED/phantom.yaml` · `phantom` | `ADF/PSMs/LND_420006/psm{1,2}.yaml` |
| `run_env_3D_MED_STRAIGHT_LND_420006.sh` | `ADF/Phantoms/3D_MED_STRAIGHT/phantom.yaml` · `phantom` | `ADF/PSMs/LND_420006/psm{1,2}.yaml` |
| `run_env_pegboard_symmetric_with_wall.sh` | `ADF/Phantoms/Pegboards/pegboard_symmetric_with_wall.yaml` · `peg_board_s` | `ADF/PSMs/LND_420006/psm{1,2}.yaml` |
| `run_env_pegboard_asymmetric.sh` | `ADF/Phantoms/Pegboards/pegboard_asymmetric.yaml` · `peg_board` | `ADF/PSMs/LND_420006/psm{1,2}.yaml` |
| `run_env_LND_400006.sh` | `ADF/Phantoms/3D_MED/phantom.yaml` · `phantom` | `ADF/PSMs/LND_400006_SIMPLE/psm{1,2}.yaml` |

Example — 10 cm closer to the Simple phantom:

```bash
tools/move_psm.py ADF/PSMs/LND_420006/psm1.yaml ADF/PSMs/LND_420006/psm2.yaml \
    --toward ADF/Phantoms/Simple/phantom.yaml Barrier --by 0.10
```

### Caveat: the PSM yamls are shared between environments

`LND_420006/psm{1,2}.yaml` are loaded by five of the six run scripts, so moving
them for one environment moves them for all of them. If you want a different
arm pose per environment, copy the PSM yamls into a new folder (e.g.
`ADF/PSMs/LND_420006_SIMPLE_ENV/`), add them to `launch.yaml` with new indices,
and point that environment's run script at those indices with `-l`.

The `ghosts_actuators_psm*.yaml` files are parented to arm links and follow
automatically; they never need editing.
