# -*- coding: utf-8 -*-
"""
Two-way exchange of coordinates (x, y, yaw) between two Raspberry Pis over Bluetooth RFCOMM.

Each side SIMULTANEOUSLY:
  - sends its own coordinates (get_my_coords())
  - receives the other robot's coordinates (see on_peer_coords())

There is a single connection (one RFCOMM stream), but it's full-duplex:
receiving and sending run in parallel threads over the same socket.

Requires prior setup (see BT_QUICK.txt):
  - compat mode for bluetoothd
  - sdptool add --channel=1 SP on the device that listens (--listen)
  - pairing the devices (bluetoothctl pair/trust, one side as initiator)

Usage:
  # On robotA - listens for an incoming connection ("listen" role on first run):
  sudo python3 bt_coords.py --listen --peer robotB

  # On robotB - connects to robotA ("connect" role):
  sudo python3 bt_coords.py --connect robotA

Once the connection is established, BOTH sides are equal peers - each one
sends and receives coordinates at the same time.
"""

import json
import socket
import struct
import subprocess
import sys
import threading
import time
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"

SOL_BLUETOOTH = 274
BT_SECURITY = 4
BT_SECURITY_SDP = 0

PACKET_FORMAT = "<Ifff"  # seq (uint32), x, y, yaw (float32 each), little-endian
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

_running = True


def load_device_config():
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: {DEVICE_ID_PATH} not found", file=sys.stderr)
        sys.exit(1)


def set_no_security(sock):
    """Disable the encryption/authentication requirement on the RFCOMM socket"""
    try:
        sock.setsockopt(SOL_BLUETOOTH, BT_SECURITY, struct.pack("BB", BT_SECURITY_SDP, 0))
    except OSError as e:
        print(f"  (failed to lower security level: {e})", file=sys.stderr)


def ensure_sdp_service(channel):
    """
    Registers a Serial Port (SPP) service on the given RFCOMM channel via sdptool.

    The SDP record lives in bluetoothd's runtime memory and is lost on every
    restart/reboot - so we register it on every startup of the listening
    side instead of relying on a one-time manual registration.
    """
    try:
        subprocess.run(
            ["sdptool", "add", f"--channel={channel}", "SP"],
            capture_output=True, text=True, timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"  (failed to register SDP service: {e})", file=sys.stderr)


def get_my_coords():
    """
    Returns the CURRENT coordinates of this robot: (x, y, yaw).

    STUB: replace with a real source (odometry, lidar, SLAM, etc.).
    For example, read from a shared variable/queue that your
    positioning module updates.
    """
    t = time.time()
    return (1.0 * (t % 10), 2.0, (t * 0.5) % 6.28)


def on_peer_coords(peer_id, seq, x, y, yaw):
    """
    Called when coordinates are received from the other robot.

    STUB: replace the print() with real handling
    (world model update, navigation, etc.).
    """
    print(f"  ← [{peer_id}] #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")


def pack_coords(seq, x, y, yaw):
    return struct.pack(PACKET_FORMAT, seq, x, y, yaw)


def unpack_coords(data):
    return struct.unpack(PACKET_FORMAT, data)


def sender_loop(sock, my_id, rate_hz):
    """Thread: periodically sends its own coordinates"""
    global _running
    period = 1.0 / rate_hz
    seq = 0

    while _running:
        x, y, yaw = get_my_coords()
        try:
            sock.send(pack_coords(seq, x, y, yaw))
            print(f"  → [{my_id}] #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        except OSError as e:
            print(f"  ✗ Send error: {e}", file=sys.stderr)
            _running = False
            break

        seq += 1
        time.sleep(period)


def receiver_loop(sock, peer_id):
    """Thread: continuously receives the peer's coordinates"""
    global _running
    buf = b""

    while _running:
        try:
            data = sock.recv(1024)
        except OSError as e:
            print(f"  ✗ Receive error: {e}", file=sys.stderr)
            _running = False
            break

        if not data:
            print(f"\n[{peer_id}] Connection closed by peer")
            _running = False
            break

        buf += data
        while len(buf) >= PACKET_SIZE:
            packet, buf = buf[:PACKET_SIZE], buf[PACKET_SIZE:]
            seq, x, y, yaw = unpack_coords(packet)
            on_peer_coords(peer_id, seq, x, y, yaw)


def run_duplex(sock, my_id, peer_id, rate_hz):
    """Runs receiving and sending in parallel over the established connection"""
    global _running
    _running = True

    t_send = threading.Thread(target=sender_loop, args=(sock, my_id, rate_hz), daemon=True)
    t_recv = threading.Thread(target=receiver_loop, args=(sock, peer_id), daemon=True)

    t_send.start()
    t_recv.start()

    try:
        while _running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Stopped by user")
        _running = False

    t_send.join(timeout=1)
    t_recv.join(timeout=1)


def run_listen(my_id, peer_id, channel=1, rate_hz=10, retry_delay=2.0):
    """
    Listens for incoming connections in an infinite loop: after a
    disconnect (peer reboot/shutdown, link loss) it waits for a new
    connection again instead of exiting. Stops only on Ctrl+C.
    """
    print(f"[{my_id}] Waiting for connection from {peer_id} on channel {channel}...")
    ensure_sdp_service(channel)

    while True:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        set_no_security(sock)

        try:
            sock.bind((socket.BDADDR_ANY, channel))
            sock.listen(1)

            conn, addr = sock.accept()
            print(f"  ✓ Connected: {addr}\n")
            print(f"  === Two-way coordinate exchange (Ctrl+C to stop) ===")

            run_duplex(conn, my_id, peer_id, rate_hz)
            conn.close()
            print(f"\n[{my_id}] Connection lost, waiting for reconnection...")
        except KeyboardInterrupt:
            print(f"\n[{my_id}] Stopped by user")
            sock.close()
            break
        except OSError as e:
            print(f"  ✗ Error: {e}, retrying in {retry_delay:.0f}s...", file=sys.stderr)
            time.sleep(retry_delay)
        finally:
            sock.close()


def run_connect(my_id, peer_id, bd_addr, channel=1, rate_hz=10, retry_delay=2.0):
    """
    Connects to the listening side in an infinite loop: keeps retrying
    until the listening side comes up, and reconnects again after a
    link drop. Stops only on Ctrl+C.
    """
    print(f"[{my_id}] Connecting to {peer_id} ({bd_addr}) on channel {channel}...")

    while True:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        set_no_security(sock)

        try:
            sock.bind((socket.BDADDR_ANY, 0))
            sock.connect((bd_addr, channel))
            print(f"  ✓ Connected\n")
            print(f"  === Two-way coordinate exchange (Ctrl+C to stop) ===")

            run_duplex(sock, my_id, peer_id, rate_hz)
            print(f"\n[{my_id}] Connection lost, trying to reconnect...")
        except KeyboardInterrupt:
            print(f"\n[{my_id}] Stopped by user")
            sock.close()
            break
        except OSError as e:
            print(f"  ✗ Failed to connect: {e}, retrying in {retry_delay:.0f}s...", file=sys.stderr)
        finally:
            sock.close()

        time.sleep(retry_delay)


def autodetect_role(my_id, config):
    """
    Determines the role (listen/connect) and the peer without flags, based on the config.

    Takes the single peer from `peers` in device_id.local.json. The role is
    decided deterministically by comparing IDs (the alphabetically smaller
    one listens), so both sides end up with opposite but consistent results
    without having to manually specify which one is the server and which is
    the client.
    """
    peers = config.get("peers", {})
    if not peers:
        print("Error: no peers in device_id.local.json", file=sys.stderr)
        print("  Add one via: python3 bt_setup.py --save-peer <id> <bd_addr>", file=sys.stderr)
        sys.exit(1)

    if len(peers) > 1:
        print(f"Warning: config has multiple peers ({list(peers)}), "
              f"using the first one", file=sys.stderr)

    peer_id = next(iter(peers))
    bd_addr = peers[peer_id].get("bd_addr")
    if not bd_addr:
        print(f"Error: peer '{peer_id}' has no bd_addr set", file=sys.stderr)
        sys.exit(1)

    is_listener = my_id < peer_id
    return is_listener, peer_id, bd_addr


def main():
    parser = ArgumentParser(description="Two-way exchange of coordinates (x, y, yaw) over Bluetooth")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--listen", action="store_true", help="Explicit: wait for an incoming connection")
    group.add_argument("--connect", type=str, metavar="TARGET_ID", help="Explicit: connect to TARGET_ID")

    parser.add_argument("--peer", type=str, metavar="TARGET_ID", help="Peer ID (for --listen, used in logs)")
    parser.add_argument("--force-addr", type=str, help="Explicit Bluetooth address (for --connect)")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM channel (default 1)")
    parser.add_argument("--rate", type=float, default=10.0, help="Rate for sending own coordinates, Hz")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")
    print(f"ID={my_id}\n")

    if args.listen or args.connect:
        # Explicit mode (flags given) - the old behavior
        if args.listen:
            peer_id = args.peer or "peer"
            run_listen(my_id, peer_id, args.channel, args.rate)
        else:
            peer_id = args.connect
            bd_addr = config.get("peers", {}).get(peer_id, {}).get("bd_addr") or args.force_addr
            if not bd_addr:
                print(f"Error: address for {peer_id} not found in config", file=sys.stderr)
                print(f"  Use --force-addr or bt_setup.py --save-peer", file=sys.stderr)
                sys.exit(1)
            run_connect(my_id, peer_id, bd_addr, args.channel, args.rate)
    else:
        # No flags - role is determined automatically from the config
        is_listener, peer_id, bd_addr = autodetect_role(my_id, config)
        print(f"Role auto-detected: "
              f"{'listening (listener)' if is_listener else 'connecting (connector)'}, "
              f"peer: {peer_id}\n")

        if is_listener:
            run_listen(my_id, peer_id, args.channel, args.rate)
        else:
            run_connect(my_id, peer_id, bd_addr, args.channel, args.rate)


if __name__ == "__main__":
    main()
