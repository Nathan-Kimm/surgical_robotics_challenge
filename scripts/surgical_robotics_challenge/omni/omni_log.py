#!/usr/bin/env python3
"""
omni_log.py

CSV recording of an omni_control.py session: one row per arm per control
cycle, holding the three stages of the teleoperation side by side so that
they can be plotted against each other (plot_omni_log.py). Each stage is the
same 7 degrees of freedom: x, y, z, roll, pitch, yaw, grasper.

    console   the raw Touch reading: stylus tip in the device frame [mm],
              roll/pitch/yaw [deg], grasper = 1 while the jaw button is held
    cmd       the transformed command sent to the simulator: tool tip in
              the PSM base frame [m], roll/pitch/yaw [deg], grasper = the
              commanded jaw angle [rad]
    sim       the PSM in the simulation: tool tip from the forward
              kinematics of the measured joints, same frame and units as
              cmd, grasper = the measured jaw angle [rad]

cmd and sim share a frame, so they compare directly; console is the input
before scaling and mapping, so its shape is what to compare. Roll, pitch,
yaw follow PyKDL's GetRPY.

Other columns
    t                  seconds since the recording started
    device, psm        the stylus and the arm it drives
    calibrated         1 once the Touch has calibrated in its inkwell; the
                       console pose is not meaningful before that
    clutch             1 while the clutch button is held (the tool is frozen)
    pos_scale          simulation units per stylus mm at that moment
Files go to LOG_DIR as omni_teleop_<start time>.csv; omni_control.py opens
one per run and prints its path.
"""

import csv
import math
import time
from datetime import datetime
from pathlib import Path

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
LOG_DIR = Path(__file__).resolve().parent / "logs"
FILE_PREFIX = "omni_teleop_"

STAGES = ("console", "cmd", "sim")
DOFS = ("x", "y", "z", "roll", "pitch", "yaw", "grasper")
COLUMNS = (["t", "device", "psm", "calibrated", "clutch", "pos_scale"]
           + [f"{stage}_{dof}" for stage in STAGES for dof in DOFS])


# ----------------------------------------------------------------------------
# Recording
# ----------------------------------------------------------------------------
def pose_values(T):
    """x, y, z and roll, pitch, yaw in degrees of a Frame."""
    return [T.p[i] for i in range(3)] + [math.degrees(a) for a in T.M.GetRPY()]


def formatted(values):
    """Numbers as short text, anything else as it is."""
    return [f"{v:.6g}" if isinstance(v, float) else v for v in values]


class TeleopLog:
    """One CSV file: record(arms) appends the current state of each ArmTeleop."""

    def __init__(self, directory=LOG_DIR):
        directory.mkdir(exist_ok=True)
        self.path = directory / f"{FILE_PREFIX}{datetime.now():%Y%m%d_%H%M%S}.csv"
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)
        self._start = time.monotonic()
        self.rows = 0

    def record(self, arms):
        t = time.monotonic() - self._start
        for arm in arms:
            self._writer.writerow(formatted(self.row(t, arm)))
            self.rows += 1

    @staticmethod
    def row(t, arm):
        """The values of COLUMNS for one arm at time t."""
        return ([t, arm.omni.name, arm.psm.name, int(arm.calibrated), int(arm.clutched), arm.pos_scale]
                + pose_values(arm.console_pose) + [int(arm.jaw_closed)]
                + pose_values(arm.T_cmd) + [arm.jaw_cmd]
                + pose_values(arm.measured_tip_in_base()) + [arm.psm.get_jaw_angle()])

    def close(self):
        self._file.close()
        print(f"Recorded {self.rows} rows to {self.path}", flush=True)
