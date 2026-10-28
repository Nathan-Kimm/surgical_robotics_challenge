#!/usr/bin/env python3
"""
udp_CRTK_console.py

Teleoperate the AMBF PSMs with the dVTrainer (Mimic Mantis Duo) surgeon console.

    console --UDP (ITP packets)--> this script --AMBF Python client--> simulator

Each console packet carries, per hand, a position increment [micrometres] and
the handle's absolute roll, pitch, yaw [rad], which count whole turns instead
of wrapping (packet layout in listen_upd_console.py; the revised
Mantis_Client marks this format with Qw = 2.0). Position increments are
summed between control cycles and added to a commanded tool-tip pose in the
PSM base frame; the console only sends them while the pedal is down, so the
pedal is the position clutch.

Orientation is absolute, as in the MTM and Geomagic teleoperation scripts:
nothing clutches it, so no offset between hand and tool can build up. The
handle orientation is composed from the console's yaw, pitch, roll the way
Mantis_Client composes them, except that the roll (the grip's free spin,
whose zero on the console is arbitrary) is counted from the homing pose. At
homing (hold the grips the way they should correspond to the tools' home
pose, finger loops one above the other, and pinch both grips fully closed
twice within a second, or press Enter) the tools go to their home pose, the
handle orientation read at that moment is registered to it and the roll
read then becomes the roll zero. Afterwards, every cycle, the rotation the
handle has made since homing is applied to the tool's home orientation,
expressed in tool axes through the pairing HANDLE_TO_TOOL:
    R_tool = R_home * C^-1 * R_handle_home^-1 * R_handle * C.
Commands go through
the CRTK-style method API of psm_arm.PSM (servo_cp, set_jaw_angle). Only the
simulator has to run.

Frames
    camera : AMBF CameraFrame, +X right, +Y up, +Z toward the viewer.
    console: ITP frame of the position increments, +X away from the surgeon,
             +Y right, +Z down (CONSOLE_TO_CAMERA).
    handle : the grip's gimbal axes: roll about +x (along the grip, toward the
             surgeon), pitch about +y (right), yaw about +z (up), composed
             R_handle = Rz(yaw) * Ry(pitch - PITCH_ZERO) * Rx(roll - roll at
             homing) (ANGLE_ORDER, the Mantis_Client order; its constant
             base-axis map cancels in the rotation since homing).
    tool   : PSM tip frame; at home +Z along the shaft into the scene, +X down,
             +Y left. HANDLE_TO_TOOL pairs each handle axis with a tool axis
             and gives the sense of each angle; udp_CRTK_console_gui.py
             changes it live and prints the lines to paste here.

Run (ROS 2 and the AMBF workspace sourced, or use ./ros2_python.sh):
    ./run_env_SIMPLE_LND_420006.sh              # terminal 1, repo root
    ./ros2_python.sh udp_CRTK_console.py        # terminal 2, this folder
then hold both grips naturally, pointing at the screen, and pinch both
closed twice.

Settings are the variables below; there are no command-line arguments.
"""

import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PyKDL import Frame, Rotation, Vector

# Use the surgical_robotics_challenge package of this checkout, even if another
# clone is pip-installed.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils import coordinate_frames
from listen_upd_console import unpack_packet, checksum_ok, absolute_format, position_delta, handle_rpy

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
UDP_IP = "0.0.0.0"
UDP_PORT = 36000                    # the console sends to this port

LEFT, RIGHT = 0, 1                  # console hand index in the packet
HAND_TO_PSM = {LEFT: "psm1", RIGHT: "psm2"}   # psm1 is on the left of the camera view
HOME_TIP_IN_CAMERA = {"psm1": coordinate_frames.PSM1.T_tip_cam,   # home poses, as in the
                      "psm2": coordinate_frames.PSM2.T_tip_cam}   # other teleop scripts

CONTROL_RATE = 100                  # Hz
REPORT_PERIOD = 2.0                 # s, between status lines
POS_SCALE = 0.00001                 # metres per console unit (1 um); raise to amplify motion
JAW_OPEN = 0.5                      # rad, jaw angle with the grip fully open
GRASP_CLOSED = 1229                 # console grasp value with the grip fully closed

# Homing gesture: pinch both grips fully closed HOME_PINCHES times within
# HOME_PINCH_WINDOW seconds (a single long pinch, as when both tools hold
# something, does nothing). A grip counts as pinched below PINCH_CLOSED
# opening and as released again above PINCH_OPEN.
HOME_PINCHES = 2
HOME_PINCH_WINDOW = 1.0             # s
PINCH_CLOSED = 0.15                 # grip opening fraction: 0 fully closed, 1 fully open
PINCH_OPEN = 0.35

# Console translation axes -> camera axes (rows: camera x, y, z).
CONSOLE_TO_CAMERA = np.array([[0, 1, 0],
                              [0, 0, -1],
                              [-1, 0, 0]])

# Handle angle -> the tool body axis its gimbal axis corresponds to, and the
# sense of the angle (+1: the console counts it the right-hand way about that
# axis, -1: the mirrored way), per hand. The three axes must be a permutation
# of x, y, z. Default from the geometry at home: the grip's roll axis is the
# tool shaft (z), pitch tips the grip up and down like the tool's y, yaw swings
# it left and right like the tool's x.
HANDLE_TO_TOOL = {
    LEFT: {"roll": ("z", +1), "pitch": ("y", +1), "yaw": ("x", +1)},
    RIGHT: {"roll": ("z", +1), "pitch": ("y", +1), "yaw": ("x", +1)},
}

# Gimbal axis of each handle angle (handle frame: +x along the grip toward the
# surgeon, +y right, +z up) and the order the gimbal composes them, outermost first.
HANDLE_AXIS = {"roll": "x", "pitch": "y", "yaw": "z"}
ANGLE_ORDER = ("yaw", "pitch", "roll")

# Console pitch reading [rad] with the grip level (perpendicular to the yaw
# axis). It sets where the yaw axis lies relative to the grip: if a pure yaw
# swing of the hand also spins the tool about its shaft, set this to the
# pitch the console reports while you hold the grip level.
PITCH_ZERO = 0.0


# ----------------------------------------------------------------------------
# Console input (UDP, background thread)
# ----------------------------------------------------------------------------
class ConsoleReceiver:
    """Receives console packets; sums each hand's position increments, keeps its latest orientation."""

    def __init__(self, ip, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.pos_sum = {LEFT: np.zeros(3), RIGHT: np.zeros(3)}   # micrometres, while the pedal is down
        self.rpy = {LEFT: None, RIGHT: None}                      # handle roll, pitch, yaw [rad]
        self.grasp = {LEFT: 0, RIGHT: 0}
        self.pedal = False
        self.old_format = False          # True while the console sends encoder increments (Qw = 1.0)
        self.packets = 0
        self.running = True
        threading.Thread(target=self._receive_loop, daemon=True).start()

    def _receive_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            packet = unpack_packet(data)
            if packet is None or not checksum_ok(packet):
                continue
            with self.lock:
                self.packets += 1
                self.pedal = bool(packet.surgeon_mode)
                self.old_format = not absolute_format(packet)
                for hand in (LEFT, RIGHT):
                    self.grasp[hand] = getattr(packet, f"grasp{hand}")
                    if not self.old_format:
                        self.rpy[hand] = np.array(handle_rpy(packet, hand))
                    if self.pedal:
                        self.pos_sum[hand] += position_delta(packet, hand)

    def take(self, hand):
        """Return (position increment [um] since the last call, handle rpy [rad] or None, grasp, pedal down)."""
        with self.lock:
            dpos, self.pos_sum[hand] = self.pos_sum[hand], np.zeros(3)
            return dpos, self.rpy[hand], self.grasp[hand], self.pedal

    def status(self):
        """Return (packets since the last call, pedal pressed, old packet format)."""
        with self.lock:
            packets, self.packets = self.packets, 0
            return packets, self.pedal, self.old_format

    def close(self):
        self.running = False
        self.sock.close()


# ----------------------------------------------------------------------------
# Conversions
# ----------------------------------------------------------------------------
ROTATION_ABOUT = {"x": Rotation.RotX, "y": Rotation.RotY, "z": Rotation.RotZ}
ANGLE_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def to_vector(xyz):
    return Vector(float(xyz[0]), float(xyz[1]), float(xyz[2]))


def to_rotation(matrix):
    """3x3 array (rows) -> PyKDL Rotation."""
    return Rotation(*(float(v) for v in np.asarray(matrix).flatten()))


def wrap_angle(angle):
    """Angle(s) [rad] -> the same angle(s) in [-pi, pi), for display."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def handle_rotation(mapping, rpy, roll_zero=0.0):
    """Console roll, pitch, yaw [rad] -> handle orientation in the console base frame,
    Rz(yaw) * Ry(pitch - PITCH_ZERO) * Rx(roll - roll_zero), each angle with its sense from `mapping`."""
    zero = {"roll": roll_zero, "pitch": PITCH_ZERO, "yaw": 0.0}
    R = Rotation()
    for name in ANGLE_ORDER:
        sense = mapping[name][1]
        R = R * ROTATION_ABOUT[HANDLE_AXIS[name]](sense * (float(rpy[ANGLE_INDEX[name]]) - zero[name]))
    return R


def tool_in_handle(mapping):
    """HANDLE_TO_TOOL[hand] -> rotation whose columns are the tool body axes in handle body coordinates."""
    C = np.zeros((3, 3))
    for name, (axis, _) in mapping.items():
        C[AXIS_INDEX[HANDLE_AXIS[name]], AXIS_INDEX[axis]] = -1.0   # at home each tool axis points against its handle axis
    if abs(np.linalg.det(C)) < 0.5:
        raise ValueError(f"HANDLE_TO_TOOL must use each tool axis once, got {mapping}")
    if np.linalg.det(C) < 0:
        C[:, AXIS_INDEX[mapping["yaw"][0]]] *= -1          # keep it a rotation, not a reflection
    return to_rotation(C)


def grip_opening(grasp):
    """Console grasp value -> grip opening fraction: 1 fully open, 0 fully closed."""
    return min(max(1.0 - grasp / GRASP_CLOSED, 0.0), 1.0)


def jaw_angle(grasp):
    """Console grasp value -> jaw angle [rad]: JAW_OPEN when open, 0 when fully closed."""
    return JAW_OPEN * grip_opening(grasp)


# ----------------------------------------------------------------------------
# Teleoperation
# ----------------------------------------------------------------------------
def load_psm(simulation_manager, camera, name):
    """PSM handle with its home pose placed in front of the camera."""
    psm = PSM(simulation_manager, name, add_joint_errors=False)
    psm.set_home_pose(psm.get_T_w_b() * camera.get_T_c_w() * HOME_TIP_IN_CAMERA[name])
    return psm


class ConsoleTeleop:
    """Turns console motion into servo_cp and jaw commands for the PSMs."""

    def __init__(self, simulation_manager, console):
        self.console = console
        self.camera = ECM(simulation_manager, "CameraFrame")
        self.psms = {hand: load_psm(simulation_manager, self.camera, name)
                     for hand, name in HAND_TO_PSM.items()}
        self.T_cmd = {hand: psm.T_t_b_home for hand, psm in self.psms.items()}  # tip in base frame
        self.handle_to_tool = {hand: dict(mapping) for hand, mapping in HANDLE_TO_TOOL.items()}
        self.tool_in_handle = {hand: tool_in_handle(mapping) for hand, mapping in HANDLE_TO_TOOL.items()}
        self.reference = {hand: None for hand in self.psms}    # (handle rpy, tool rotation in camera axes) at homing
        self.home_requested = False
        self.handle_rpy = {hand: None for hand in self.psms}  # last angles seen
        self.grip = {hand: 1.0 for hand in self.psms}         # grip opening fraction, for the pinch gesture
        self.pinched = False                                   # both grips closed (with hysteresis)
        self.pinch_times = []                                  # when both grips last closed
        self.pedal = False

    def camera_in_base(self, psm):
        return psm.get_T_w_b() * self.camera.get_T_c_w()

    def tip_in_camera(self, hand):
        """Commanded tool-tip pose of one hand's PSM, in the camera frame."""
        return self.camera_in_base(self.psms[hand]).Inverse() * self.T_cmd[hand]

    def handle_turn(self, hand):
        """Rotation the handle has made since homing, in its homing frame; identity before homing."""
        rpy = self.handle_rpy[hand]
        if rpy is None or self.reference[hand] is None:
            return Rotation()
        mapping, rpy_ref = self.handle_to_tool[hand], self.reference[hand][0]
        roll_zero = float(rpy_ref[ANGLE_INDEX["roll"]])   # the grip's spin at homing is the gimbal's neutral
        return handle_rotation(mapping, rpy_ref, roll_zero).Inverse() * handle_rotation(mapping, rpy, roll_zero)

    def homed(self):
        return all(reference is not None for reference in self.reference.values())

    def request_home(self):
        """Home at the next control cycle (safe to call from another thread)."""
        self.home_requested = True

    def home(self):
        """Tools to their home pose; the handles' current orientations now mean 'home'."""
        self.home_requested = False
        for hand, psm in self.psms.items():
            self.T_cmd[hand] = psm.T_t_b_home
            if self.handle_rpy[hand] is None:
                self.home_requested = True                     # no angles yet: try again next cycle
            else:
                self.reference[hand] = (self.handle_rpy[hand], HOME_TIP_IN_CAMERA[psm.name].M)

    def set_handle_mapping(self, hand, name, axis, sign):
        """Change one handle angle's tool axis or sense. The tool keeps its orientation, so home again afterwards."""
        mapping = dict(self.handle_to_tool[hand], **{name: (axis, sign)})
        self.tool_in_handle[hand] = tool_in_handle(mapping)       # raises ValueError if a tool axis repeats
        self.handle_to_tool[hand] = mapping
        if self.handle_rpy[hand] is not None:
            self.reference[hand] = (self.handle_rpy[hand], self.tip_in_camera(hand).M)

    def update(self):
        if self.home_requested:
            self.home()
        for hand, psm in self.psms.items():
            dpos, rpy, grasp, pedal = self.console.take(hand)
            self.handle_rpy[hand] = rpy
            self.grip[hand] = grip_opening(grasp)
            self.pedal = pedal
            T = self.T_cmd[hand]
            R_cb = self.camera_in_base(psm).M
            step = R_cb * to_vector(CONSOLE_TO_CAMERA @ dpos * POS_SCALE)
            self.T_cmd[hand] = Frame(self.commanded_rotation(hand, R_cb, T.M), T.p + step)

            psm.servo_cp(self.T_cmd[hand])
            psm.set_jaw_angle(jaw_angle(grasp))
        self.detect_pinch()

    def detect_pinch(self):
        """Home when both grips are pinched closed HOME_PINCHES times within HOME_PINCH_WINDOW seconds."""
        openings = self.grip.values()
        if self.pinched:
            self.pinched = not any(opening > PINCH_OPEN for opening in openings)
        elif all(opening < PINCH_CLOSED for opening in openings):
            self.pinched = True
            now = time.monotonic()
            self.pinch_times = [t for t in self.pinch_times if now - t < HOME_PINCH_WINDOW] + [now]
            if len(self.pinch_times) >= HOME_PINCHES:
                self.pinch_times = []
                self.request_home()

    def commanded_rotation(self, hand, R_cb, R_now):
        """Tool rotation in the base frame: the handle's turn since homing applied to the tool; unchanged before homing."""
        if self.handle_rpy[hand] is None or self.reference[hand] is None:
            return R_now
        R_tool_ref = self.reference[hand][1]
        C = self.tool_in_handle[hand]
        return R_cb * R_tool_ref * C.Inverse() * self.handle_turn(hand) * C   # handle turn, in tool axes

    def report(self, seconds):
        packets, pedal, old_format = self.console.status()
        if old_format:
            print("WARNING: console sends Qw = 1.0 (encoder increments), orientation is ignored;"
                  " rebuild Mantis_Client with the absolute-angle change", flush=True)
        parts = []
        for hand, psm in self.psms.items():
            p = self.tip_in_camera(hand).p
            rpy = self.handle_rpy[hand]
            angles = " ".join(f"{np.degrees(wrap_angle(a)):.0f}" for a in rpy) if rpy is not None else "-"
            parts.append(f"{psm.name} tip(cam) [{p[0]:.3f} {p[1]:.3f} {p[2]:.3f}] handle rpy(deg) [{angles}]"
                         f" grip {self.grip[hand]:.0%}")
        homed = "homed" if self.homed() else "NOT HOMED (hold the grips naturally, pinch both closed twice)"
        print(f"console {packets / seconds:.0f} pkt/s  pedal {'DOWN' if pedal else 'UP'}  {homed}  "
              + "  ".join(parts), flush=True)


def watch_enter(callback):
    """Call callback() each time Enter is pressed in the terminal (daemon thread; idle without a terminal)."""
    def loop():
        for _ in sys.stdin:
            callback()
    threading.Thread(target=loop, daemon=True).start()


def connect(client_name):
    """Console receiver and teleop attached to the running simulator; exits with a hint if there is none."""
    console = ConsoleReceiver(UDP_IP, UDP_PORT)
    print(f"Console on {UDP_IP}:{UDP_PORT}; connecting to the simulator", flush=True)
    simulation_manager = SimulationManager(client_name)
    try:
        teleop = ConsoleTeleop(simulation_manager, console)
    except (RuntimeError, AttributeError) as error:
        console.close()
        sys.exit(f"PSMs or camera not found in the simulation ({error}). "
                 "Is the simulator running? Start it with ./run_env_SIMPLE_LND_420006.sh")
    print("PSMs moving to the home pose; teleoperation running (Ctrl+C to stop).\n"
          "Hold both grips naturally, pointing at the screen, and pinch both closed twice within a second\n"
          "(or press Enter) to home.", flush=True)
    return simulation_manager, console, teleop


def main():
    simulation_manager, console, teleop = connect("udp_crtk_console")
    watch_enter(teleop.request_home)
    rate = simulation_manager.create_rate(CONTROL_RATE)
    last_report = time.monotonic()
    try:
        while not simulation_manager.is_shutdown():
            teleop.update()
            if time.monotonic() - last_report >= REPORT_PERIOD:
                teleop.report(time.monotonic() - last_report)
                last_report = time.monotonic()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    except Exception:
        if not simulation_manager.is_shutdown():   # a real error; after Ctrl+C ROS is already down
            raise
    finally:
        console.close()


if __name__ == "__main__":
    main()
