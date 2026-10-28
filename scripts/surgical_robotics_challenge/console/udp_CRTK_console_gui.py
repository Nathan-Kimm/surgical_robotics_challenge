#!/usr/bin/env python3
"""
udp_CRTK_console_gui.py

udp_CRTK_console.py plus a window of sliders that follow the console and the
simulated arms, to check and tune the handle-to-tool pairing (HANDLE_TO_TOOL)
by eye while teleoperating.

For each hand the window shows, live:
    handle angles   roll, pitch, yaw as the console sends them, wrapped to
                    one turn [deg] (the console counts whole turns)
    grip            grip opening, 0 pinched closed .. 1 open
    handle turn     the handle's rotation since homing about its own x (roll),
                    y (pitch), z (yaw) axes [deg]
    mapping         which tool axis each handle angle turns about and in which
                    sense; change it from the menus
    tool turn       rotation of the commanded tool away from its home
                    orientation, about the camera X, Y, Z axes [deg]
    tool position   commanded tool tip in the camera frame [m]
The sliders only display; the console drives the arms, as in udp_CRTK_console.py.
Handle sliders are on the left of each hand's panel, tool sliders on the right;
the menus in between say which tool axis each handle axis drives.

How to use: hold both grips naturally, pointing at the screen with the finger
loops one above the other (the roll read then becomes the roll zero), and pinch
both fully closed twice within a second (or press Home, or Enter in the terminal).
The tools go to their home pose and from then on
follow the grips' orientation. Turn one grip about one axis at a time: roll
should spin the tool about its shaft, pitch tip it up and down, yaw swing it
left and right, each the same way as your hand. If one is wrong, pick another
entry in that angle's menu (letter = tool axis, sign = sense). A change keeps
the tool where it is, so home again afterwards. Every change prints the
HANDLE_TO_TOOL line to paste into udp_CRTK_console.py, and both lines are
printed again at exit.

Run (simulator up, console streaming):
    ./ros2_python.sh udp_CRTK_console_gui.py
"""

import time
import tkinter as tk

import numpy as np

from udp_CRTK_console import (ANGLE_INDEX, CONTROL_RATE, HAND_TO_PSM, HOME_TIP_IN_CAMERA, LEFT, RIGHT,
                              REPORT_PERIOD, connect, watch_enter, wrap_angle)

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
HAND_NAMES = {LEFT: "LEFT", RIGHT: "RIGHT"}
ANGLES = ("roll", "pitch", "yaw")
AXES = ("x", "y", "z")
AXIS_CHOICES = ("+x", "-x", "+y", "-y", "+z", "-z")   # menu entries: sense and tool axis
ANGLE_RANGE = 180.0      # deg, angle sliders run from -ANGLE_RANGE to +ANGLE_RANGE
POSITION_RANGE = 0.15    # m, position sliders run this far either side of the home position
SLIDER_LENGTH = 280      # px


# ----------------------------------------------------------------------------
# Mapping <-> text
# ----------------------------------------------------------------------------
def choice(axis, sign):
    """('z', -1) -> '-z', the menu entry for one angle's mapping."""
    return ("+" if sign > 0 else "-") + axis


def mapping_from_choice(text):
    """'-z' -> ('z', -1)."""
    return text[1], (1 if text[0] == "+" else -1)


def paste_line(hand, mapping):
    """One hand's HANDLE_TO_TOOL entry, formatted like the dict in udp_CRTK_console.py."""
    entries = ", ".join(f'"{name}": ("{axis}", {sign:+d})' for name, (axis, sign) in mapping.items())
    return f"    {HAND_NAMES[hand]}: {{{entries}}},"


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


class HandPanel:
    """Mapping menus of one hand, its handle sliders on the left and its tool sliders on the right."""

    def __init__(self, parent, hand, teleop):
        self.hand = hand
        self.teleop = teleop
        self.home = HOME_TIP_IN_CAMERA[HAND_TO_PSM[hand]]
        box = tk.LabelFrame(parent, text=f"{HAND_NAMES[hand]} hand -> {HAND_TO_PSM[hand]}", padx=8, pady=4)
        box.pack(side=tk.LEFT, anchor="n", padx=8, pady=8)
        add_header(box, "Mapping: handle angle -> tool axis, sense")
        self.menus = self.add_menus(box)
        columns = tk.Frame(box)
        columns.pack()

        handle = tk.Frame(columns)
        handle.pack(side=tk.LEFT, anchor="n", padx=(0, 12))
        add_header(handle, "Handle angles from the console [deg]")
        self.handle = {name: add_slider(handle, name, 0.0, ANGLE_RANGE, 0.1) for name in ANGLES}
        add_header(handle, "Grip opening [0 pinched closed .. 1 open]")
        self.grip = add_slider(handle, "grip", 0.5, 0.5, 0.01)
        add_header(handle, "Handle turn since homing, handle axes [deg]")
        self.handle_turn = {axis: add_slider(handle, f"{axis} ({name} axis)", 0.0, ANGLE_RANGE, 0.1)
                            for axis, name in zip(AXES, ANGLES)}

        tool = tk.Frame(columns)
        tool.pack(side=tk.LEFT, anchor="n")
        add_header(tool, "Tool turn from home, camera axes [deg]")
        self.turn = {axis: add_slider(tool, axis, 0.0, ANGLE_RANGE, 0.1) for axis in AXES}
        add_header(tool, "Tool tip position, camera frame [m]")
        self.position = {axis: add_slider(tool, axis, self.home.p[i], POSITION_RANGE, 0.001)
                         for i, axis in enumerate(AXES)}

    def add_menus(self, box):
        """One option menu per handle angle; returns their variables."""
        row = tk.Frame(box)
        row.pack(pady=4)
        menus = {}
        for column, name in enumerate(ANGLES):
            var = tk.StringVar(value=choice(*self.teleop.handle_to_tool[self.hand][name]))
            var.trace_add("write", lambda *_, name=name, var=var: self.change_mapping(name, var.get()))
            tk.Label(row, text=f"{name} ->").grid(row=0, column=2 * column, padx=(8, 2))
            tk.OptionMenu(row, var, *AXIS_CHOICES).grid(row=0, column=2 * column + 1)
            menus[name] = var
        return menus

    def change_mapping(self, name, text):
        axis, sign = mapping_from_choice(text)
        try:
            self.teleop.set_handle_mapping(self.hand, name, axis, sign)
        except ValueError as error:
            print(f"not applied: {error}", flush=True)
            return
        print(paste_line(self.hand, self.teleop.handle_to_tool[self.hand]), flush=True)

    def refresh(self):
        """Move the sliders to the latest console angles and tool command."""
        rpy = self.teleop.handle_rpy[self.hand]
        if rpy is not None:   # the console's angles count whole turns: show them wrapped to +-180
            for name in ANGLES:
                self.handle[name].set(round(float(np.degrees(wrap_angle(rpy[ANGLE_INDEX[name]]))), 1))
        self.grip.set(round(self.teleop.grip[self.hand], 2))
        handle_turn = self.teleop.handle_turn(self.hand).GetRot()
        for i, axis in enumerate(AXES):
            self.handle_turn[axis].set(round(float(np.degrees(handle_turn[i])), 1))
        T_tip = self.teleop.tip_in_camera(self.hand)
        turn = (T_tip.M * self.home.M.Inverse()).GetRot()
        for i, axis in enumerate(AXES):
            self.turn[axis].set(round(float(np.degrees(turn[i])), 1))
            self.position[axis].set(round(T_tip.p[i], 3))


class ConsoleWindow:
    """A status line, the Home button and one panel per hand."""

    def __init__(self, teleop):
        self.teleop = teleop
        self.app = tk.Tk()
        self.app.title("Console teleop: handle angles -> tool orientation")
        self.app.protocol("WM_DELETE_WINDOW", self.close)
        self.open = True
        top = tk.Frame(self.app)
        top.pack(fill="x", padx=16, pady=(8, 0))
        tk.Button(top, text="Home", command=teleop.request_home).pack(side=tk.LEFT)
        self.status = tk.Label(top, anchor="w")
        self.status.pack(side=tk.LEFT, padx=12)
        panels = tk.Frame(self.app)
        panels.pack()
        self.panels = [HandPanel(panels, hand, teleop) for hand in (LEFT, RIGHT)]

    def refresh(self):
        for panel in self.panels:
            panel.refresh()
        pedal = "DOWN (position follows)" if self.teleop.pedal else "UP (position held)"
        homed = "homed, orientation follows the grips (double pinch or Home to home again)" if self.teleop.homed() \
            else "NOT HOMED: hold the grips naturally, pointing at the screen, pinch both closed twice (or Home)"
        self.status.config(text=f"Pedal {pedal}   |   {homed}")
        self.app.update()

    def close(self):
        self.open = False
        self.app.destroy()


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def print_mappings(teleop):
    print("HANDLE_TO_TOOL for udp_CRTK_console.py:")
    for hand in (LEFT, RIGHT):
        print(paste_line(hand, teleop.handle_to_tool[hand]))


def main():
    simulation_manager, console, teleop = connect("udp_crtk_console_gui")
    watch_enter(teleop.request_home)
    window = ConsoleWindow(teleop)
    print_mappings(teleop)

    rate = simulation_manager.create_rate(CONTROL_RATE)
    last_report = time.monotonic()
    try:
        while window.open and not simulation_manager.is_shutdown():
            teleop.update()
            window.refresh()
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
        print()
        print_mappings(teleop)


if __name__ == "__main__":
    main()
