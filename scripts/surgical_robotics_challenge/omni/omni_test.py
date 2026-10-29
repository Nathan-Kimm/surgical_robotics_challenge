#!/usr/bin/env python3
"""
omni_test.py

Print the stylus position and absolute orientation of the 3D Systems Touch
(formerly Geomagic Touch / Sensable Phantom Omni) haptic devices, one line
per device.

The devices are read straight through the OpenHaptics HD API (libHD, loaded
with ctypes): no ROS driver and no simulator are involved. Each sample is
taken inside libHD's 1 kHz servo loop with hdScheduleSynchronous, the way
the SDK's QueryDevice and CoulombForceDual examples do it, and holds

    position     stylus tip in the device base frame [mm]
    orientation  stylus rotation in the base frame, taken from
                 HD_CURRENT_TRANSFORM, so absolute: nothing is integrated
                 and nothing drifts or wraps
    buttons      the two stylus buttons, BUTTON_1 lower (dark) and BUTTON_2
                 upper (light); read for omni_control.py, not printed here

Frames
    base   : the device's own frame, +X right, +Y up, +Z toward the user,
             origin in the middle of the workspace.
    stylus : the pose printed; -Z runs along the stylus to its tip, so the
             identity orientation points into the screen. Roll, pitch, yaw
             follow PyKDL's GetRPY, R = Rz(yaw) * Ry(pitch) * Rx(roll), and
             are printed in degrees.

Setup, once: install the driver (setup_touch_driver.sh) and list each
device's serial number in DEVICE_SERIALS (Touch_HeadlessSetup LIST prints
the serials of the connected devices):
    sudo ./setup_touch_driver.sh
    /opt/TouchDriver/bin/Touch_HeadlessSetup LIST
Nothing has to be sourced: libHD is in /usr/lib and the driver keeps its
device configs in ~/.3dsystems/config. open_devices() pairs the devices at
every start, because the driver opens the /dev/ttyACM<Channel> port written
in <name>.config and only checks the serial, and the ports are numbered in
the order the devices were plugged in. Keep one config per serial there:
the driver uses the first config whose serial matches, so a stale
"Default Device.config" (Touch_HeadlessSetup run without arguments) hijacks
the device it names.
Run (no ROS needed; the VS Code Run button works too):
    python3 omni_test.py                         # Ctrl+C to stop
A Touch calibrates the first time its stylus rests in the inkwell; until
then its line says so and the numbers are not meaningful.

Settings are the variables below; there are no command-line arguments.
"""

import ctypes
import math
import subprocess
import sys
import time
from collections import namedtuple

from PyKDL import Frame, Rotation, Vector

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
HD_LIBRARY = "libHD.so.3.4"                      # installed to /usr/lib by setup_touch_driver.sh
PAIRING_TOOL = "/opt/TouchDriver/bin/Touch_HeadlessSetup"   # installed by setup_touch_driver.sh
DEVICE_SERIALS = {"Left Device": "18019000318",  # device name -> its serial number (Touch_HeadlessSetup LIST)
                  "Right Device": "18019000315"}
DEVICE_NAMES = tuple(DEVICE_SERIALS)
PRINT_RATE = 10.0                                # Hz

# ----------------------------------------------------------------------------
# OpenHaptics HD API, from /usr/include/HD/hdDefines.h
# ----------------------------------------------------------------------------
HD_SUCCESS = 0x0000
HD_CURRENT_BUTTONS = 0x2000              # bit mask of the stylus buttons
HD_CURRENT_TRANSFORM = 0x2052            # 16 doubles, column-major, translation in mm
HD_VERSION = 0x2500
HD_DEVICE_MODEL_TYPE = 0x2501
HD_DEVICE_DRIVER_VERSION = 0x2502
HD_DEVICE_SERIAL_NUMBER = 0x2504
HD_CALLBACK_DONE = 0
HD_DEFAULT_SCHEDULER_PRIORITY = 0xFFFF // 2
HD_CALIBRATION_OK = 0x5000
HD_CALIBRATION_NEEDS_UPDATE = 0x5001
HD_CALIBRATION_INKWELL = 1 << 2          # the Touch and the Omni calibrate from the inkwell
BUTTON_1 = 1 << 0                        # HD_DEVICE_BUTTON_1, the lower (dark) stylus button
BUTTON_2 = 1 << 1                        # HD_DEVICE_BUTTON_2, the upper (light) stylus button

HDSchedulerCallback = ctypes.CFUNCTYPE(ctypes.c_uint, ctypes.c_void_p)


class HDErrorInfo(ctypes.Structure):
    _fields_ = [("errorCode", ctypes.c_uint),
                ("internalErrorCode", ctypes.c_int),
                ("hHD", ctypes.c_uint)]


# function name: (argument types, return type) of the libHD functions used here
HD_SIGNATURES = {
    "hdInitDevice": ([ctypes.c_char_p], ctypes.c_uint),
    "hdDisableDevice": ([ctypes.c_uint], None),
    "hdMakeCurrentDevice": ([ctypes.c_uint], None),
    "hdStartScheduler": ([], None),
    "hdStopScheduler": ([], None),
    "hdScheduleSynchronous": ([HDSchedulerCallback, ctypes.c_void_p, ctypes.c_ushort], None),
    "hdBeginFrame": ([ctypes.c_uint], None),
    "hdEndFrame": ([ctypes.c_uint], None),
    "hdGetDoublev": ([ctypes.c_uint, ctypes.POINTER(ctypes.c_double)], None),
    "hdGetIntegerv": ([ctypes.c_uint, ctypes.POINTER(ctypes.c_int)], None),
    "hdGetString": ([ctypes.c_uint], ctypes.c_char_p),
    "hdGetError": ([], HDErrorInfo),
    "hdGetErrorString": ([ctypes.c_uint], ctypes.c_char_p),
    "hdCheckCalibration": ([], ctypes.c_uint),
    "hdUpdateCalibration": ([ctypes.c_uint], None),
}


def load_hd_library():
    """libHD with the signatures above declared."""
    hd = ctypes.CDLL(HD_LIBRARY)
    for name, (argtypes, restype) in HD_SIGNATURES.items():
        function = getattr(hd, name)
        function.argtypes, function.restype = argtypes, restype
    return hd


def check_error(hd, message):
    """Raise RuntimeError if libHD has recorded an error."""
    error = hd.hdGetError()
    if error.errorCode != HD_SUCCESS:
        text = (hd.hdGetErrorString(error.errorCode) or b"unknown error").decode()
        raise RuntimeError(f"{message}: {text} (error 0x{error.errorCode:04X}, internal {error.internalErrorCode})")


# ----------------------------------------------------------------------------
# Devices
# ----------------------------------------------------------------------------
Sample = namedtuple("Sample", "pose buttons calibrated")   # stylus Frame in the base frame [mm], button bits, bool


class OmniDevice:
    """One Touch through libHD: read() returns its current stylus pose and buttons.

    Create devices with open_devices(): libHD runs one servo loop for all of them.
    """

    def __init__(self, hd, name):
        self.hd = hd
        self.name = name
        self.handle = hd.hdInitDevice(name.encode())
        check_error(hd, f"Failed to initialize '{name}'")
        self._transform = (ctypes.c_double * 16)()
        self._buttons = ctypes.c_int()
        self._calibration = HD_CALIBRATION_NEEDS_UPDATE
        self._callback = HDSchedulerCallback(self._sample)   # keep a reference: libHD holds a bare pointer

    def info(self, pname):
        """A device string such as its model or serial number."""
        self.hd.hdMakeCurrentDevice(self.handle)
        return (self.hd.hdGetString(pname) or b"?").decode()

    def _sample(self, _user_data):
        """Servo-thread callback: copies this device's state out; calibrates when the device asks for it."""
        hd = self.hd
        hd.hdBeginFrame(self.handle)                 # also makes this device the current one
        hd.hdGetDoublev(HD_CURRENT_TRANSFORM, self._transform)
        hd.hdGetIntegerv(HD_CURRENT_BUTTONS, self._buttons)
        hd.hdEndFrame(self.handle)
        self._calibration = hd.hdCheckCalibration()
        if self._calibration == HD_CALIBRATION_NEEDS_UPDATE:
            hd.hdUpdateCalibration(HD_CALIBRATION_INKWELL)
        return HD_CALLBACK_DONE

    def read(self):
        """Runs _sample in the servo thread and returns the Sample."""
        self.hd.hdScheduleSynchronous(self._callback, None, HD_DEFAULT_SCHEDULER_PRIORITY)
        check_error(self.hd, f"'{self.name}' error")
        t = self._transform   # column-major: the columns are the stylus axes, t[12:15] the tip
        pose = Frame(Rotation(t[0], t[4], t[8],
                              t[1], t[5], t[9],
                              t[2], t[6], t[10]),
                     Vector(t[12], t[13], t[14]))
        return Sample(pose, self._buttons.value, self._calibration == HD_CALIBRATION_OK)


def pair_device(name):
    """Rewrites the driver config of the named device so that it points at the port its Touch is on now."""
    result = subprocess.run([PAIRING_TOOL, f"name={name}", f"sn={DEVICE_SERIALS[name]}"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        lines = result.stdout.strip().splitlines() or ["no output"]
        detail = next((line for line in lines if line.startswith("Error")), lines[-1])
        raise RuntimeError(f"Could not pair '{name}' (serial {DEVICE_SERIALS[name]}): {detail}")


def open_devices(names=DEVICE_NAMES):
    """The named devices, paired, initialized, then libHD's servo loop started: once, after all of them, as the SDK requires."""
    hd = load_hd_library()
    for name in names:
        pair_device(name)
    devices = [OmniDevice(hd, name) for name in names]
    hd.hdStartScheduler()
    check_error(hd, "Failed to start the servo loop")
    return devices


def close_devices(devices):
    """Stops the servo loop, then releases the devices."""
    if not devices:
        return
    hd = devices[0].hd
    hd.hdStopScheduler()
    for device in devices:
        hd.hdDisableDevice(device.handle)


# ----------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------
HEADER = (f"{'device':<13}{'x(mm)':>9} {'y(mm)':>9} {'z(mm)':>9}"
          f" {'roll(deg)':>10} {'pitch(deg)':>11} {'yaw(deg)':>9}")


def format_pose(name, pose, calibrated):
    p = pose.p
    roll, pitch, yaw = (math.degrees(a) for a in pose.M.GetRPY())
    line = (f"{name:<13}{p.x():9.2f} {p.y():9.2f} {p.z():9.2f}"
            f" {roll:10.1f} {pitch:11.1f} {yaw:9.1f}")
    if not calibrated:
        line += "   not calibrated: rest the stylus in the inkwell"
    return line


def main():
    try:
        devices = open_devices()
    except (OSError, RuntimeError) as e:
        sys.exit(f"{e}\nIs each Touch plugged in and switched on, its serial number in DEVICE_SERIALS,"
                 " and the driver set up (setup_touch_driver.sh)?")
    for device in devices:
        print(f"{device.name}: {device.info(HD_DEVICE_MODEL_TYPE)}, serial {device.info(HD_DEVICE_SERIAL_NUMBER)},"
              f" driver {device.info(HD_DEVICE_DRIVER_VERSION)}, OpenHaptics {device.info(HD_VERSION)}")
    print("(Ctrl+C to stop)\n")
    print(HEADER)
    try:
        while True:
            for device in devices:
                sample = device.read()
                print(format_pose(device.name, sample.pose, sample.calibrated))
            time.sleep(1.0 / PRINT_RATE)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(e)
    finally:
        close_devices(devices)


if __name__ == "__main__":
    main()
