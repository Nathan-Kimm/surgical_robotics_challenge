#!/usr/bin/env python3
"""
omni_control_gui.py

omni_control.py plus a window that follows both styluses and both simulated
arms, to check the mapping by eye and tune the position scale while
teleoperating.

For each arm the window shows, live:
    stylus position   stylus tip in camera axes [mm]
    stylus angles     roll, pitch, yaw of the stylus [deg]
    stylus turn       the stylus's rotation away from its neutral pose
                      (level, pointing into the screen, buttons up) about the
                      camera X, Y, Z axes [deg]
    tool turn         rotation of the commanded tool away from its home
                      orientation, about the camera X, Y, Z axes [deg]; it
                      equals the stylus turn whenever the clutch is free
    tool position     commanded tool tip in the camera frame [m]
Those sliders only display; the styluses drive the arms, as in
omni_control.py. The scale slider is live: drag it to change how far the
tools move per stylus millimetre (POS_SCALE in omni_control.py; the value
is printed at exit).

How to use: rest each stylus in its inkwell once, then hold them level,
pointing at the screen. The tools follow; the lower button freezes a tool
while held, a double-click (or Home, or Enter in the terminal) puts it back
at its home position, the upper button closes the jaw. Turn a stylus about
one axis at a time: the tool should turn the same way on screen.

Run (simulator up):
    ../console/ros2_python.sh omni_control_gui.py
"""

import math
import time
import tkinter as tk

from omni_control import CONTROL_RATE, LEFT, POS_SCALE, REPORT_PERIOD, RIGHT, connect, watch_enter

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
AXES = ("x", "y", "z")
ANGLES = ("roll", "pitch", "yaw")
ANGLE_RANGE = 180.0             # deg, angle sliders run from -ANGLE_RANGE to +ANGLE_RANGE
STYLUS_RANGE = 150.0            # mm, stylus position sliders run this far either side of zero
POSITION_RANGE = 0.15           # m, tool position sliders run this far either side of the home position
SCALE_RANGE = (0.0002, 0.005)   # simulation units per stylus mm, ends of the scale slider
SLIDER_LENGTH = 240             # px


# ----------------------------------------------------------------------------
# Window
# ----------------------------------------------------------------------------
def add_header(parent, text):
    tk.Label(parent, text=text, anchor="w", font=("TkDefaultFont", 10, "bold")).pack(fill="x", pady=(10, 0))


def add_slider(parent, label, centre, half_range, resolution):
    """Display-only slider; returns the variable that moves it."""
    var = tk.DoubleVar(value=centre)
    tk.Scale(parent, label=label, from_=centre - half_range, to=centre + half_range, resolution=resolution,
             orient=tk.HORIZONTAL, length=SLIDER_LENGTH, variable=var, state=tk.DISABLED).pack()
    return var


class ArmPanel:
    """One arm: its status line, stylus sliders on the left and tool sliders on the right."""

    def __init__(self, parent, arm):
        self.arm = arm
        self.home = arm.home_tip_in_camera
        box = tk.LabelFrame(parent, text=f"{arm.omni.name} -> {arm.psm.name}", padx=8, pady=4)
        box.pack(side=tk.LEFT, anchor="n", padx=8, pady=8)
        self.status = tk.Label(box, anchor="w")
        self.status.pack(fill="x")
        columns = tk.Frame(box)
        columns.pack()

        stylus = tk.Frame(columns)
        stylus.pack(side=tk.LEFT, anchor="n", padx=(0, 12))
        add_header(stylus, "Stylus position, camera axes [mm]")
        self.stylus_position = {axis: add_slider(stylus, axis, 0.0, STYLUS_RANGE, 0.1) for axis in AXES}
        add_header(stylus, "Stylus angles [deg]")
        self.stylus_angle = {name: add_slider(stylus, name, 0.0, ANGLE_RANGE, 0.1) for name in ANGLES}
        add_header(stylus, "Stylus turn from neutral, camera axes [deg]")
        self.stylus_turn = {axis: add_slider(stylus, axis, 0.0, ANGLE_RANGE, 0.1) for axis in AXES}

        tool = tk.Frame(columns)
        tool.pack(side=tk.LEFT, anchor="n")
        add_header(tool, "Tool turn from home, camera axes [deg]")
        self.tool_turn = {axis: add_slider(tool, axis, 0.0, ANGLE_RANGE, 0.1) for axis in AXES}
        add_header(tool, "Tool tip position, camera frame [m]")
        self.tool_position = {axis: add_slider(tool, axis, self.home.p[i], POSITION_RANGE, 0.001)
                              for i, axis in enumerate(AXES)}

    def refresh(self):
        """Move the sliders to the latest stylus pose and tool command."""
        stylus = self.arm.stylus
        if stylus is not None:
            for i, axis in enumerate(AXES):
                self.stylus_position[axis].set(round(stylus.p[i], 1))
            for name, angle in zip(ANGLES, stylus.M.GetRPY()):
                self.stylus_angle[name].set(round(math.degrees(angle), 1))
        stylus_turn = self.arm.stylus_turn().GetRot()
        T_tip = self.arm.tip_in_camera()
        tool_turn = (T_tip.M * self.home.M.Inverse()).GetRot()
        for i, axis in enumerate(AXES):
            self.stylus_turn[axis].set(round(math.degrees(stylus_turn[i]), 1))
            self.tool_turn[axis].set(round(math.degrees(tool_turn[i]), 1))
            self.tool_position[axis].set(round(T_tip.p[i], 3))
        calibrated = "calibrated" if self.arm.calibrated else "NOT CALIBRATED: rest the stylus in the inkwell"
        clutch = "HELD, tool frozen" if self.arm.clutched else "free, tool follows"
        jaw = "closed" if self.arm.jaw_closed else "open"
        self.status.config(text=f"Touch {calibrated}   |   clutch {clutch}   |   jaw {jaw}")


class OmniWindow:
    """The Home button, the scale slider and one panel per arm."""

    def __init__(self, teleop):
        self.teleop = teleop
        self.app = tk.Tk()
        self.app.title("Omni teleop: styluses -> PSMs")
        self.app.protocol("WM_DELETE_WINDOW", self.close)
        self.open = True
        top = tk.Frame(self.app)
        top.pack(fill="x", padx=16, pady=(8, 0))
        tk.Button(top, text="Home", command=teleop.request_home).pack(side=tk.LEFT)
        tk.Label(top, anchor="w", text="Home: tools back to their home position (or double-click a stylus's lower"
                 " button, or Enter). Orientation always follows the stylus; the lower button freezes the tool"
                 " while held, the upper one closes the jaw.").pack(side=tk.LEFT, padx=12)
        scale = tk.Scale(self.app, label="Position scale [simulation units per stylus mm]", from_=SCALE_RANGE[0],
                         to=SCALE_RANGE[1], resolution=0.0001, orient=tk.HORIZONTAL, length=2 * SLIDER_LENGTH,
                         command=lambda value: teleop.set_scale(float(value)))
        scale.set(POS_SCALE)
        scale.pack(padx=16, pady=4)
        panels = tk.Frame(self.app)
        panels.pack()
        self.panels = [ArmPanel(panels, teleop.arms[hand]) for hand in (LEFT, RIGHT) if hand in teleop.arms]

    def refresh(self):
        for panel in self.panels:
            panel.refresh()
        self.app.update()

    def close(self):
        self.open = False
        self.app.destroy()


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def main():
    simulation_manager, omnis, teleop = connect("omni_control_gui")
    watch_enter(teleop.request_home)
    window = OmniWindow(teleop)

    rate = simulation_manager.create_rate(CONTROL_RATE)
    last_report = time.monotonic()
    try:
        while window.open and not simulation_manager.is_shutdown():
            teleop.update()
            window.refresh()
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
        from omni_test import close_devices
        close_devices(omnis)
        print(f"POS_SCALE = {next(iter(teleop.arms.values())).pos_scale:g}   # for omni_control.py")


if __name__ == "__main__":
    main()
