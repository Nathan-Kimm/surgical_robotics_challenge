#!/usr/bin/env python3
"""
listen_upd_console.py

Listen for the UDP command packets sent by the dVTrainer (Mimic Mantis Duo)
surgeon console and print the position and orientation they carry.

The console runs Mantis_Client, which sends one ITP ``u_struct`` packet per
haptic-loop tick (a few hundred Hz) to <this PC>:36000. The packet is defined
in Mantis_Client/include/ITPteleoperation.h (``#pragma pack(1)``, little
endian, 124 bytes):

    unsigned int sequence              packet counter
    unsigned int pactyp, version       unused (0)
    int    delx[2], dely[2], delz[2]   position increment per arm [micrometers]
    double Qx[2], Qy[2], Qz[2]         handle roll, pitch, yaw per arm [rad], absolute,
                                       not wrapped: whole turns add up
    double Qw[2]                       2.0 = this format; 1.0 = old encoder-increment format
    int    buttonstate[2]              same value as grasp
    int    grasp[2]                    grip opening, int(opening * -5000)
    int    surgeon_mode                foot pedal: 1 = pressed, 0 = released
    int    checksum                    see checksum_ok()

Index 0 is the left master, index 1 the right master. Positions are
INCREMENTS since the previous packet (zero while the pedal is released), so
this script integrates them into a running total. Orientations are ABSOLUTE
angles read from the Mantis SDK, sent in every packet whether or not the
pedal is pressed. They are not wrapped: a handle turned around several times
reads several times 2 pi (left roll -1457 deg was seen on 2026-10-25). They
come from the revised Mantis_Client (October 2026),
which marks the format with Qw = 2.0; the original client sent gimbal
encoder increments with Qw = 1.0.

References (telesurgery-qos-analysis repo):
    Mantis_Client/src/comm.cpp                      sender
    SurRoL_dVTrainer/tests/dVTrainer/Console.py     receiver (old format)

Usage:
    python3 listen_upd_console.py

The bind address, port and print rate are plain variables in main(). Use
port 5001 behind the NetFI network emulator, or print rate 0 to print every
packet.
"""

import math
import socket
import struct
import time
from collections import namedtuple

# ----------------------------------------------------------------------------
# Packet definition
# ----------------------------------------------------------------------------
PACKET_FORMAT = "<IIIiiiiiiddddddddiiiiii"
PACKET_FIELDS = (
    "sequence pactyp version "
    "delx0 delx1 dely0 dely1 delz0 delz1 "
    "Qx0 Qx1 Qy0 Qy1 Qz0 Qz1 Qw0 Qw1 "
    "buttonstate0 buttonstate1 grasp0 grasp1 surgeon_mode checksum"
).split()
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)  # 124 bytes
ABSOLUTE_FORMAT = 2.0                         # Qw written by the revised Mantis_Client

ConsolePacket = namedtuple("ConsolePacket", PACKET_FIELDS)

ARM_NAMES = ("L", "R")  # index 0 = left master, 1 = right master


def unpack_packet(data):
    """Bytes -> ConsolePacket, or None if the datagram has the wrong size."""
    if len(data) != PACKET_SIZE:
        return None
    return ConsolePacket(*struct.unpack(PACKET_FORMAT, data))


def checksum_ok(p):
    """Recompute the sender's checksum.

    Mantis_Client/src/comm.cpp sums the fields below; the variant in
    ITPteleoperation.h also adds grasp[], so both are accepted.
    """
    base = (p.surgeon_mode
            + p.delx0 + p.dely0 + p.delz0
            + p.delx1 + p.dely1 + p.delz1
            + p.buttonstate0 + p.buttonstate1
            + p.sequence)
    with_grasp = base + p.grasp0 + p.grasp1
    return p.checksum in (to_int32(base), to_int32(with_grasp))


def to_int32(value):
    """Wrap a Python int the way C 32-bit ``int`` arithmetic does."""
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value >= 0x80000000 else value


def absolute_format(p):
    """True if the packet carries absolute handle angles (Qw = 2.0)."""
    return p.Qw0 == ABSOLUTE_FORMAT


def position_delta(p, arm):
    """Position increment of one arm (x, y, z) in micrometers."""
    return (getattr(p, f"delx{arm}"), getattr(p, f"dely{arm}"), getattr(p, f"delz{arm}"))


def handle_rpy(p, arm):
    """Absolute handle orientation of one arm (roll, pitch, yaw) in radians."""
    return (getattr(p, f"Qx{arm}"), getattr(p, f"Qy{arm}"), getattr(p, f"Qz{arm}"))


# ----------------------------------------------------------------------------
# Running state
# ----------------------------------------------------------------------------
class ConsoleState:
    """Counts packets, integrates the position increments and keeps the latest orientation."""

    def __init__(self):
        self.packets = 0
        self.bad_checksums = 0
        self.bad_sizes = 0
        self.last = None
        self.position = [[0, 0, 0], [0, 0, 0]]              # micrometers, per arm
        self.rotation = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]  # roll, pitch, yaw [rad], per arm

    def update(self, packet):
        self.packets += 1
        self.last = packet
        for arm in range(2):
            for axis, delta in enumerate(position_delta(packet, arm)):
                self.position[arm][axis] += delta
            self.rotation[arm] = list(handle_rpy(packet, arm))


# ----------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------
def print_state(state, sender, rate_hz):
    p = state.last
    pedal = "DOWN" if p.surgeon_mode else "UP"
    print(f"seq {p.sequence}  {rate_hz:.0f} Hz  pedal {pedal}  from {sender[0]}"
          f"  bad checksum {state.bad_checksums}  bad size {state.bad_sizes}")
    if not absolute_format(p):
        print(f"  WARNING: Qw = {p.Qw0}, the console still sends encoder increments;"
              " the angles below are not valid")
    print(f"  {'':3} {'X(mm)':>9} {'Y(mm)':>9} {'Z(mm)':>9}"
          f" {'roll(deg)':>10} {'pitch(deg)':>10} {'yaw(deg)':>10} {'grasp':>7}")
    for arm, name in enumerate(ARM_NAMES):
        x, y, z = (um / 1000.0 for um in state.position[arm])
        roll, pitch, yaw = (math.degrees(a) for a in state.rotation[arm])
        grasp = getattr(p, f"grasp{arm}")
        print(f"  {name:3} {x:9.3f} {y:9.3f} {z:9.3f}"
              f" {roll:10.1f} {pitch:10.1f} {yaw:10.1f} {grasp:7d}")
    print()


# ----------------------------------------------------------------------------
# Main loop
# ----------------------------------------------------------------------------
def listen(ip, port, print_rate):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    sock.settimeout(1.0)
    print(f"Listening for console packets on {ip}:{port} (Ctrl+C to stop)\n")

    state = ConsoleState()
    print_period = 1.0 / print_rate if print_rate > 0 else 0.0
    next_print = time.monotonic() + print_period
    window_start, window_packets = time.monotonic(), 0

    try:
        while True:
            try:
                data, sender = sock.recvfrom(2048)
            except socket.timeout:
                print(f"waiting for packets on {ip}:{port} ...")
                continue

            packet = unpack_packet(data)
            if packet is None:
                state.bad_sizes += 1
                continue
            if not checksum_ok(packet):
                state.bad_checksums += 1
                continue
            state.update(packet)
            window_packets += 1

            now = time.monotonic()
            if now >= next_print:
                rate_hz = window_packets / max(now - window_start, 1e-9)
                print_state(state, sender, rate_hz)
                next_print = now + print_period
                window_start, window_packets = now, 0
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f"\nReceived {state.packets} valid packets "
              f"({state.bad_checksums} bad checksum, {state.bad_sizes} bad size)")


def main():
    # Edit these to match your setup; no command-line arguments are needed.
    ip = "0.0.0.0"      # interface to bind, 0.0.0.0 = all interfaces
    port = 36000        # UDP port the console sends to (Omni_Comm::port_Robot in comm.cpp)
    print_rate = 10.0   # Hz, 0 = print every packet

    listen(ip, port, print_rate)


if __name__ == "__main__":
    main()
