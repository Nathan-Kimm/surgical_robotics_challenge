#!/usr/bin/env python3
"""
omni_control.py

Teleoperate the AMBF PSMs with two 3D Systems Touch (Geomagic Touch /
Phantom Omni) haptic styluses: the left one drives psm1, the right one psm2.

    Touch x2 --libHD (omni_test.OmniDevice)--> this script --AMBF Python client--> simulator

Position is DELTA and clutched: every control cycle the stylus displacement
since the previous cycle, scaled by POS_SCALE, is added to the commanded
tool-tip position. Holding the clutch button freezes the tool, position and
orientation, so you can bring the stylus back to the middle of its small
workspace; the tool's reach is unlimited.

Orientation is ABSOLUTE with a fixed alignment, so there is no homing offset
to register: with the stylus level, pointing into the screen, buttons up
(STYLUS_NEUTRAL, the OpenHaptics identity pose) the tool has its home
orientation, and any stylus rotation away from that, as the user sees it, is
applied to the tool as the camera sees it:
    R_tool_cam = R_stylus * STYLUS_NEUTRAL^-1 * R_tool_home_cam.
This needs no axis pairing because a Touch's frame and the AMBF camera frame
are the same frame for a user facing the screen (OMNI_TO_CAMERA is for a
Touch standing at an angle to it). While the clutch is held the orientation
stays frozen; on release it follows the stylus again, so release it with the
stylus about where it was, or the tool snaps to the new orientation.

Home (double-click a stylus's clutch button, press Enter for both arms, or
the Home button of omni_control_gui.py) puts the tool back at its home
position only; the orientation always comes from the stylus.

The jaw is open while the jaw button is up and closed while it is held.

Frames
    omni   : OpenHaptics device space, +X right, +Y up, +Z toward the user,
             origin in the middle of the workspace, millimetres.
    camera : AMBF CameraFrame, +X right, +Y up, +Z toward the viewer.
    stylus : OpenHaptics end-effector frame; -Z runs along the stylus to its
             tip, so the identity orientation points into the screen.
    tool   : PSM tip frame; at home +Z along the shaft into the scene.

Run (ROS 2 and the AMBF workspace sourced, or use ../console/ros2_python.sh):
    ./run_env_SIMPLE_LND_420006.sh              # terminal 1, repo root
    ../console/ros2_python.sh omni_control.py   # terminal 2, this folder
Pairing the two devices under their names: see omni_test.py.

Settings are the variables below; there are no command-line arguments.
"""

import math
import sys
import threading
import time
from pathlib import Path

from PyKDL import Frame, Rotation, Vector

# Use the surgical_robotics_challenge package of this checkout, even if another
# clone is pip-installed.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils import coordinate_frames
from omni_test import open_devices, close_devices, BUTTON_1, BUTTON_2

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
LEFT, RIGHT = "Left Device", "Right Device"             # the names Touch_HeadlessSetup paired the devices under
HAND_TO_PSM = {LEFT: "psm1", RIGHT: "psm2"}             # psm1 is on the left of the camera view
HOME_TIP_IN_CAMERA = {"psm1": coordinate_frames.PSM1.T_tip_cam,   # home poses, as in the
                      "psm2": coordinate_frames.PSM2.T_tip_cam}   # other teleop scripts

CONTROL_RATE = 100                  # Hz
REPORT_PERIOD = 2.0                 # s, between status lines
POS_SCALE = 0.001                   # simulation units per stylus millimetre; raise to amplify motion
JAW_OPEN = 0.5                      # rad, jaw angle with the jaw button up

CLUTCH_BUTTON = BUTTON_1            # lower (dark) stylus button: hold to move the stylus with the tool frozen
JAW_BUTTON = BUTTON_2               # upper (light) stylus button: hold to close the jaw
HOME_CLICKS = 2                     # clutch-button clicks that send the tool to its home position ...
HOME_CLICK_WINDOW = 0.6             # s, ... when made within this time

# Stylus orientation that gives the tool its home orientation: level, pointing
# into the screen, buttons up (the OpenHaptics identity). Rotation.RotX(-angle)
# for a neutral grip that points down by angle.
STYLUS_NEUTRAL = Rotation()

# Omni axes -> camera axes, per device. Identity: the Touch faces the user, who
# faces the screen. Rotation.RotY(angle) for a Touch turned about the vertical.
OMNI_TO_CAMERA = {LEFT: Rotation(), RIGHT: Rotation()}


# ----------------------------------------------------------------------------
# Conversions
# ----------------------------------------------------------------------------
def in_camera_axes(pose, omni_to_camera):
    """Stylus pose in omni axes -> the same pose in camera axes."""
    return Frame(omni_to_camera * pose.M * omni_to_camera.Inverse(), omni_to_camera * pose.p)


def jaw_angle(closed):
    return 0.0 if closed else JAW_OPEN


# ----------------------------------------------------------------------------
# Teleoperation
# ----------------------------------------------------------------------------
def load_psm(simulation_manager, camera, name):
    """PSM handle with its home pose placed in front of the camera."""
    psm = PSM(simulation_manager, name, add_joint_errors=False)
    psm.set_home_pose(psm.get_T_w_b() * camera.get_T_c_w() * HOME_TIP_IN_CAMERA[name])
    return psm


class ArmTeleop:
    """One stylus driving one PSM: servo_cp and jaw commands from the stylus motion."""

    def __init__(self, omni, psm, camera, omni_to_camera):
        self.omni = omni
        self.psm = psm
        self.camera = camera
        self.omni_to_camera = omni_to_camera
        self.home_tip_in_camera = HOME_TIP_IN_CAMERA[psm.name]
        self.T_cmd = psm.T_t_b_home          # commanded tip pose in the PSM base frame
        self.pos_scale = POS_SCALE
        self.stylus = None                   # latest stylus pose in camera axes [mm], None until calibrated
        self.calibrated = False
        self.clutched = False
        self.jaw_closed = False
        self.home_requested = False
        self.click_times = []                # recent clutch-button presses, for the double click

    def camera_in_base(self):
        return self.psm.get_T_w_b() * self.camera.get_T_c_w()

    def tip_in_camera(self):
        """Commanded tool-tip pose in the camera frame."""
        return self.camera_in_base().Inverse() * self.T_cmd

    def stylus_turn(self):
        """Rotation of the stylus away from STYLUS_NEUTRAL, camera axes; identity until calibrated."""
        if self.stylus is None:
            return Rotation()
        return self.stylus.M * STYLUS_NEUTRAL.Inverse()

    def request_home(self):
        """Home at the next control cycle (safe to call from another thread)."""
        self.home_requested = True

    def home(self):
        """Tool back to its home position; the orientation keeps coming from the stylus."""
        self.home_requested = False
        self.T_cmd = Frame(self.T_cmd.M, self.psm.T_t_b_home.p)

    def count_click(self):
        """Home when the clutch button is clicked HOME_CLICKS times within HOME_CLICK_WINDOW seconds."""
        now = time.monotonic()
        self.click_times = [t for t in self.click_times if now - t < HOME_CLICK_WINDOW] + [now]
        if len(self.click_times) >= HOME_CLICKS:
            self.click_times = []
            self.request_home()

    def update(self):
        sample = self.omni.read()
        self.calibrated = sample.calibrated
        clutched = bool(sample.buttons & CLUTCH_BUTTON)
        if clutched and not self.clutched:
            self.count_click()
        self.clutched = clutched
        self.jaw_closed = bool(sample.buttons & JAW_BUTTON)
        previous, self.stylus = self.stylus, (in_camera_axes(sample.pose, self.omni_to_camera)
                                              if sample.calibrated else None)
        if self.home_requested:
            self.home()
        if self.stylus is not None and not self.clutched:   # the clutch freezes the tool, position and orientation
            T_c_b = self.camera_in_base()
            step = Vector() if previous is None else T_c_b.M * ((self.stylus.p - previous.p) * self.pos_scale)
            self.T_cmd = Frame(T_c_b.M * self.stylus_turn() * self.home_tip_in_camera.M, self.T_cmd.p + step)
        self.psm.servo_cp(self.T_cmd)
        self.psm.set_jaw_angle(jaw_angle(self.jaw_closed))

    def report(self):
        p = self.tip_in_camera().p
        stylus = "-" if self.stylus is None else " ".join(f"{math.degrees(a):.0f}" for a in self.stylus.M.GetRPY())
        state = "ok" if self.calibrated else "NOT CALIBRATED (rest the stylus in the inkwell)"
        return (f"{self.omni.name} {state}, clutch {'HELD' if self.clutched else 'free'},"
                f" jaw {'closed' if self.jaw_closed else 'open'} -> {self.psm.name}"
                f" tip(cam) [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] stylus rpy(deg) [{stylus}]")


class OmniTeleop:
    """Both styluses driving both PSMs."""

    def __init__(self, simulation_manager, omnis):
        self.camera = ECM(simulation_manager, "CameraFrame")
        self.arms = {omni.name: ArmTeleop(omni, load_psm(simulation_manager, self.camera, HAND_TO_PSM[omni.name]),
                                          self.camera, OMNI_TO_CAMERA[omni.name])
                     for omni in omnis}

    def update(self):
        for arm in self.arms.values():
            arm.update()

    def request_home(self):
        """Home both arms at the next control cycle (safe to call from another thread)."""
        for arm in self.arms.values():
            arm.request_home()

    def set_scale(self, pos_scale):
        for arm in self.arms.values():
            arm.pos_scale = pos_scale

    def report(self):
        print("   |   ".join(arm.report() for arm in self.arms.values()), flush=True)


def watch_enter(callback):
    """Call callback() each time Enter is pressed in the terminal (daemon thread; idle without a terminal)."""
    def loop():
        for _ in sys.stdin:
            callback()
    threading.Thread(target=loop, daemon=True).start()


def connect(client_name):
    """Both Touch devices and the teleop attached to the running simulator; exits with a hint if anything is missing."""
    try:
        omnis = open_devices(tuple(HAND_TO_PSM))
    except (OSError, RuntimeError) as error:
        sys.exit(f"{error}\nIs each Touch plugged in, switched on, and paired under its name (see omni_test.py)?")
    print("Both Touch devices found; connecting to the simulator", flush=True)
    simulation_manager = SimulationManager(client_name)
    try:
        teleop = OmniTeleop(simulation_manager, omnis)
    except (RuntimeError, AttributeError) as error:
        close_devices(omnis)
        sys.exit(f"PSMs or camera not found in the simulation ({error}). "
                 "Is the simulator running? Start it with ./run_env_SIMPLE_LND_420006.sh")
    print("PSMs moving to the home pose; teleoperation running (Ctrl+C to stop).\n"
          "Rest each stylus in its inkwell once to calibrate, then hold them level, pointing at the screen.\n"
          "The tools follow the styluses; hold the lower button to move a stylus with its tool frozen,\n"
          "double-click it (or press Enter) to put the tool back at its home position,\n"
          "hold the upper button to close the jaw.", flush=True)
    return simulation_manager, omnis, teleop


def main():
    simulation_manager, omnis, teleop = connect("omni_control")
    watch_enter(teleop.request_home)
    rate = simulation_manager.create_rate(CONTROL_RATE)
    last_report = time.monotonic()
    try:
        while not simulation_manager.is_shutdown():
            teleop.update()
            if time.monotonic() - last_report >= REPORT_PERIOD:
                teleop.report()
                last_report = time.monotonic()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    except Exception:
        if not simulation_manager.is_shutdown():   # a real error; after Ctrl+C ROS is already down
            raise
    finally:
        close_devices(omnis)


if __name__ == "__main__":
    main()
