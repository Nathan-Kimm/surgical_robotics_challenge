#!/bin/bash
# setup_touch_driver.sh
#
# Install the driver of the 3D Systems Touch (Geomagic Touch / Phantom Omni)
# haptic device and the OpenHaptics 3.4 SDK that omni_control.py reads it
# through, on Ubuntu 24.04. The sources are the two folders from 3D Systems
# (support.3dsystems.com, Haptics, Linux downloads):
#
#   TouchDriver_2024_09_19                      libPhantomIOLib42, udev rule, setup tools
#   openhaptics_3.4-0-developer-edition-amd64   libHD/libHL, headers, examples, docs
#
# It is the vendor's install_haptic_driver and install scripts without the
# prompts and the reboot, plus what Ubuntu 24.04 needs on top:
#   - libHD links libncurses.so.5 and libtinfo.so.5, which 24.04 no longer
#     ships; it only uses stdscr and wgetch from them, so links to the .so.6 do.
#   - TouchCheckup wants libglut.so.3; 24.04 ships libglut.so.3.12.
#   - the vendor's udev rule only covers product 0304; this Touch reports
#     2988:0302, so the rule here covers every 3D Systems device.
#   - ModemManager is told not to probe the Touch's serial port as a modem.
#   - the Google Drive download dropped the executable bits of the tools.
#
# Run once:
#     sudo ./setup_touch_driver.sh
# Then, as your normal user (no sudo: the device config goes to ~/.3dsystems):
#     plug the Touch in and switch it on;  lsusb -d 2988:  must list it
#     /opt/TouchDriver/bin/Touch_HeadlessSetup      # pairs it as "Default Device"
#     python3 omni_control.py
# Optional checks: /opt/TouchDriver/ListCOMPortHapticDevices prints its serial
# port, and (cd /opt/TouchDriver/bin && ./TouchCheckup) opens the diagnostics
# GUI. Building the SDK's C examples also needs build-essential and freeglut3-dev.

set -euo pipefail

# Sources (the folders as downloaded) and destinations.
TOUCH_DRIVER_DIR=/home/nathan/Desktop/TouchDriver_2024_09_19-20261026T153618Z-1-001/TouchDriver_2024_09_19
OPENHAPTICS_DIR=/home/nathan/Desktop/openhaptics_3.4-0-developer-edition-amd64-20261026T153615Z-1-001/openhaptics_3.4-0-developer-edition-amd64
TOOLS_DIR=/opt/TouchDriver                  # Touch_HeadlessSetup, TouchCheckup, Touch_AdvancedConfig
SDK_DIR=/opt/OpenHaptics/Developer/3.4-0    # examples, docs, libsrc (the vendor's location)
LIB_DIR=/usr/lib                            # the vendor's location; TouchCheckup looks there by absolute path
INCLUDE_DIR=/usr/include
SYSTEM_LIB_DIR=/usr/lib/x86_64-linux-gnu    # Ubuntu's own libraries
UDEV_RULE=/etc/udev/rules.d/99-3dsystems.rules

if [ "$EUID" -ne 0 ]; then
    echo "Run as root:  sudo $0"
    exit 1
fi
for dir in "$TOUCH_DRIVER_DIR" "$OPENHAPTICS_DIR"; do
    if [ ! -d "$dir" ]; then
        echo "Not found: $dir  (edit the path at the top of this script)"
        exit 1
    fi
done

echo "Touch driver -> $LIB_DIR, $TOOLS_DIR, $UDEV_RULE"
install -m 644 "$TOUCH_DRIVER_DIR"/usr/lib/*.so "$LIB_DIR"/
mkdir -p "$TOOLS_DIR"
cp -r "$TOUCH_DRIVER_DIR/bin" "$TOOLS_DIR"/
cp "$TOUCH_DRIVER_DIR/ListCOMPortHapticDevices" "$TOOLS_DIR"/
chmod 755 "$TOOLS_DIR"/bin/Touch_* "$TOOLS_DIR"/bin/TouchCheckup "$TOOLS_DIR"/ListCOMPortHapticDevices
# Device node readable and writable by everyone: the vendor's rule, widened
# from product 0304 to every 3D Systems device (this Touch reports 2988:0302),
# plus a line that keeps ModemManager from probing it as a modem when plugged in.
{
    echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="2988", GROUP="users", MODE="0666"'
    echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="2988", ENV{ID_MM_DEVICE_IGNORE}="1"'
} > "$UDEV_RULE"
udevadm control --reload
udevadm trigger --subsystem-match=usb --subsystem-match=tty

echo "OpenHaptics SDK -> $SDK_DIR, $LIB_DIR, $INCLUDE_DIR"
mkdir -p "$(dirname "$SDK_DIR")"
cp -r "$OPENHAPTICS_DIR/opt/OpenHaptics/Developer/3.4-0" "$(dirname "$SDK_DIR")"/
chmod -R a+rX "$SDK_DIR"
cp -r "$OPENHAPTICS_DIR"/usr/include/* "$INCLUDE_DIR"/
install -m 644 "$OPENHAPTICS_DIR"/usr/lib/* "$LIB_DIR"/
for lib in HD HL QH QHGLUTWrapper; do
    ln -sfn "lib$lib.so.3.4.0" "$LIB_DIR/lib$lib.so.3.4"
    ln -sfn "lib$lib.so.3.4.0" "$LIB_DIR/lib$lib.so"
done
echo "export OH_SDK_BASE=$SDK_DIR" > /etc/profile.d/openhaptics.sh

echo "Ubuntu 24.04 compatibility links -> $LIB_DIR"
ln -sfn "$SYSTEM_LIB_DIR/libncurses.so.6" "$LIB_DIR/libncurses.so.5"
ln -sfn "$SYSTEM_LIB_DIR/libtinfo.so.6" "$LIB_DIR/libtinfo.so.5"
ln -sfn "$SYSTEM_LIB_DIR/libglut.so.3.12" "$LIB_DIR/libglut.so.3"
ldconfig

cat <<EOF

Done. Next, as your normal user (not with sudo):
  1. Plug the Touch in and switch it on;  lsusb -d 2988:  must list it.
  2. $TOOLS_DIR/bin/Touch_HeadlessSetup      pairs it as "Default Device" (config in ~/.3dsystems)
  3. python3 omni_control.py                  prints position and orientation
EOF
