#!/usr/bin/env python3
"""
plot_omni_log.py

Plot a recording made by omni_control.py (omni_log.py): for each arm, the
three stages of the teleoperation over time, one row per degree of freedom
(x, y, z, roll, pitch, yaw, grasper).

    left column    the console command: the raw Touch stylus reading, in
                   the device frame, on its own scale
    right column   the transformed command sent to the PSM and the PSM
                   measured in the simulation, both in the PSM base frame,
                   on one scale, so any lag or offset between them is the
                   simulator's own tracking

Grey bands mark the clutch held: the stylus moves while the command stands
still. Console rows are blank until the Touch has calibrated. Angles are
unwrapped so a turn through +-180 deg does not draw a jump.

For each arm the lag that best aligns the simulated tip with the command
and the RMS position error left after it are printed.

Run (no ROS needed):
    python3 plot_omni_log.py                                       # newest recording in logs/
    python3 plot_omni_log.py logs/omni_teleop_20261105_143000.csv
    python3 plot_omni_log.py logs/omni_teleop_20261105_143000.csv 10 40   # only 10 s to 40 s
The figures are shown and saved as PNG files beside the CSV.
"""

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

from omni_log import DOFS, FILE_PREFIX, LOG_DIR

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
COLORS = {"console": "#2a78d6", "cmd": "#eb6834", "sim": "#1baf7a"}
LABELS = {"console": "console command (stylus)", "cmd": "transformed command (to PSM)", "sim": "PSM in simulation"}
UNITS = {"console": ("mm", "deg", "button"), "cmd": ("m", "deg", "rad")}   # position, angle, grasper
ANGLES = ("roll", "pitch", "yaw")
LINE_WIDTH = 1.2
CLUTCH_COLOR = "#000000"
CLUTCH_ALPHA = 0.08
GRID_COLOR = "#dddddd"
MAX_LAG = 1.0                    # s, longest simulation-behind-command lag searched for
FIGURE_SIZE = (12, 15)           # inches


# ----------------------------------------------------------------------------
# Reading
# ----------------------------------------------------------------------------
def newest_log():
    files = sorted(LOG_DIR.glob(f"{FILE_PREFIX}*.csv"))
    if not files:
        sys.exit(f"No recordings in {LOG_DIR}; run omni_control.py first")
    return files[-1]


def read_log(path, start, end):
    """{device: {column: array}} for the rows between start and end seconds: float arrays plus the psm name."""
    with open(path, newline="") as f:
        rows = [row for row in csv.DictReader(f) if start <= float(row["t"]) <= end]
    if not rows:
        sys.exit(f"{path} holds no samples between {start} s and {end} s")
    logs = {}
    for device in dict.fromkeys(row["device"] for row in rows):
        device_rows = [row for row in rows if row["device"] == device]
        data = {"psm": device_rows[0]["psm"]}
        for column in rows[0]:
            if column not in ("device", "psm"):
                data[column] = np.array([float(row[column]) if row[column] else np.nan for row in device_rows])
        logs[device] = data
    return logs


def signal(data, stage, dof):
    """One column, plot-ready: angles unwrapped, the console blank until calibrated."""
    values = data[f"{stage}_{dof}"].copy()
    if stage == "console":
        values[data["calibrated"] < 0.5] = np.nan
    if dof in ANGLES:
        valid = ~np.isnan(values)
        values[valid] = np.unwrap(values[valid], period=360.0)
    return values


def unit(stage, dof):
    position, angle, grasper = UNITS[stage]
    return grasper if dof == "grasper" else angle if dof in ANGLES else position


# ----------------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------------
def positions(data, stage):
    return np.column_stack([data[f"{stage}_{axis}"] for axis in ("x", "y", "z")])


def tracking(data):
    """(lag [s], RMS position error [m]): the shift of the simulated tip behind the command that fits it best."""
    t, cmd, sim = data["t"], positions(data, "cmd"), positions(data, "sim")
    dt = np.median(np.diff(t)) if len(t) > 1 else 1.0
    best = None
    for lag in range(min(int(MAX_LAG / dt), len(t) - 1) + 1):
        error = cmd[:len(cmd) - lag] - sim[lag:]
        rms = np.sqrt(np.mean(np.sum(error ** 2, axis=1)))
        if best is None or rms < best[1]:
            best = (lag * dt, rms)
    return best


def clutch_spans(t, clutch):
    """(start, end) times during which the clutch was held."""
    spans, start = [], None
    for ti, held in zip(t, clutch > 0.5):
        if held and start is None:
            start = ti
        if not held and start is not None:
            spans.append((start, ti))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


# ----------------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------------
def style(ax, spans):
    for start, end in spans:
        ax.axvspan(start, end, color=CLUTCH_COLOR, alpha=CLUTCH_ALPHA, lw=0)
    ax.grid(color=GRID_COLOR, lw=0.5)
    ax.spines[["top", "right"]].set_visible(False)


def plot_arm(device, data, path):
    """One figure: a row per DOF, the console command on the left, command and simulation on the right."""
    t = data["t"]
    spans = clutch_spans(t, data["clutch"])
    fig, axes = plt.subplots(len(DOFS), 2, figsize=FIGURE_SIZE, sharex=True)
    fig.suptitle(f"{device} -> {data['psm']}   ({path.name})")
    axes[0, 0].set_title("Console command: Touch stylus, device frame")
    axes[0, 1].set_title("Transformed command and PSM in simulation, PSM base frame")
    for row, dof in enumerate(DOFS):
        left, right = axes[row]
        left.plot(t, signal(data, "console", dof), color=COLORS["console"], lw=LINE_WIDTH, label=LABELS["console"])
        left.set_ylabel(f"{dof} [{unit('console', dof)}]")
        for stage in ("cmd", "sim"):
            right.plot(t, signal(data, stage, dof), color=COLORS[stage], lw=LINE_WIDTH, label=LABELS[stage])
        right.set_ylabel(f"{dof} [{unit('cmd', dof)}]")
        style(left, spans)
        style(right, spans)
    for ax in axes[-1]:
        ax.set_xlabel("time [s]")
    handles = [*axes[0, 0].get_legend_handles_labels()[0], *axes[0, 1].get_legend_handles_labels()[0],
               Patch(facecolor=CLUTCH_COLOR, alpha=CLUTCH_ALPHA)]
    labels = [LABELS["console"], LABELS["cmd"], LABELS["sim"], "clutch held"]
    fig.legend(handles, labels, loc="lower center", ncol=4, frameon=False)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    png = path.with_name(f"{path.stem}_{data['psm']}.png")
    fig.savefig(png, dpi=120)
    return png


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else newest_log()
    start = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    end = float(sys.argv[3]) if len(sys.argv) > 3 else np.inf
    for device, data in read_log(path, start, end).items():
        t = data["t"]
        rate = (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0
        lag, rms = tracking(data)
        png = plot_arm(device, data, path)
        print(f"{device} -> {data['psm']}: {len(t)} samples, {t[0]:.1f} s to {t[-1]:.1f} s ({rate:.0f} Hz);"
              f" the simulated tip lags the command by {lag * 1000:.0f} ms,"
              f" RMS position error after that lag {rms * 1000:.2f} mm; saved {png}")
    plt.show()


if __name__ == "__main__":
    main()
