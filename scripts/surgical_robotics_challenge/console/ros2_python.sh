#!/bin/bash
# python3 inside the ROS 2 + AMBF environment of this machine.
#
#   ./ros2_python.sh udp_CRTK_console.py        # run a script directly, from any shell
#   ./ros2_python.sh listen_upd_console.py
#
# VS Code: .vscode/settings.json and .vscode/launch.json point at this file as
# the Python interpreter, so the Run and Debug buttons work on any script here.
# Paths follow scripts/README.md (AMBF workspace + ~/venv_src); edit if yours differ.
source ~/ros_ambf_ws/install/setup.bash     # AMBF 3.0 workspace, also sources /opt/ros/jazzy
exec ~/venv_src/bin/python3 "$@"            # Python 3.12 venv with system site packages
