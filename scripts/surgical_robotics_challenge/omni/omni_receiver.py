# Receives the Touch/Omni state sent by omni_udp_sender.py. Run this script
# to print the input using only the Python standard library, or run
# omni_control_gui.py --udp to use the input in AMBF.
import math
import select
import socket
import struct
import time

LISTEN_IP = "0.0.0.0"  # 0.0.0.0 = accept on any of this PC's IPs
PORT = 5005            # must match DEST in omni_udp_sender.py
PRINT_RATE = 10        # Hz
TIMEOUT = 0.5          # seconds without a packet before the sender counts as gone

# Must match PACKET in omni_udp_sender.py exactly.
PACKET = struct.Struct("<Id" + "6dB?" * 2)  # 112 bytes, little-endian, no padding

DEVICE_NAMES = ("Left Device", "Right Device")
BUTTON_1 = 1 << 0
BUTTON_2 = 1 << 1


def unpack(data):
    """Packet bytes -> (seq, send time, [left state, right state])."""
    seq, sent_at, *values = PACKET.unpack(data)

    states = []
    for i in range(len(DEVICE_NAMES)):
        v = values[i * 8:(i + 1) * 8]
        states.append({
            "position": tuple(v[0:3]),     # x, y, z in mm
            "orientation": tuple(v[3:6]),  # roll, pitch, yaw in degrees
            "buttons": v[6],
            "calibrated": v[7],
        })

    return seq, sent_at, states


def print_state(name, state):
    x, y, z = state["position"]
    roll, pitch, yaw = state["orientation"]
    b1 = bool(state["buttons"] & BUTTON_1)
    b2 = bool(state["buttons"] & BUTTON_2)

    print(
        f"{name:<13} "
        f"XYZ: ({x:7.2f}, {y:7.2f}, {z:7.2f}) mm  "
        f"RPY: ({roll:6.1f}, {pitch:6.1f}, {yaw:6.1f}) deg  "
        f"Buttons: {int(b1)}, {int(b2)}"
    )

    if not state["calibrated"]:
        print("  Not calibrated: place stylus in inkwell")


class OmniReceiver:
    """Cache the newest pair of stylus states for a nonblocking control loop."""

    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((LISTEN_IP, PORT))
            self.socket.setblocking(False)
        except OSError:
            self.socket.close()
            raise
        self.states = None
        self.sender = None
        self.last_seq = None
        self.last_received = 0.0

    def poll(self):
        if self.states is not None and time.monotonic() - self.last_received >= TIMEOUT:
            self.states = None
            self.last_seq = None
            return  # One empty control cycle resets the stylus position baseline.

        # Drain queued packets without letting a busy socket stall the GUI.
        for _ in range(256):
            try:
                data, addr = self.socket.recvfrom(PACKET.size + 1)
            except BlockingIOError:
                break

            if len(data) != PACKET.size:
                continue
            seq, _, states = unpack(data)
            if self.last_seq is not None and seq <= self.last_seq:
                continue  # arrived late or twice: a newer state was already used
            if not all(math.isfinite(value) for state in states
                       for value in state["position"] + state["orientation"]):
                continue
            self.states = states
            self.sender = addr[0]
            self.last_seq = seq
            self.last_received = time.monotonic()

    def close(self):
        self.socket.close()


def main():
    receiver = OmniReceiver()
    print(f"Listening for omni data on UDP port {PORT}...")
    print("Press Ctrl+C to stop\n")

    last_print = 0.0
    try:
        while True:
            select.select([receiver.socket], [], [], 1.0 / PRINT_RATE)
            previous_seq = receiver.last_seq
            receiver.poll()
            if receiver.states is None:
                if previous_seq is not None:
                    print(f"No data for {TIMEOUT} s: sender stopped or network down.\n")
                continue
            if receiver.last_seq == previous_seq:
                continue
            if previous_seq is None:
                print(f"Receiving from {receiver.sender}\n")
            now = time.monotonic()
            if now - last_print >= 1.0 / PRINT_RATE:
                last_print = now
                for name, state in zip(DEVICE_NAMES, receiver.states):
                    print_state(name, state)
                print()

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        receiver.close()


if __name__ == "__main__":
    main()
