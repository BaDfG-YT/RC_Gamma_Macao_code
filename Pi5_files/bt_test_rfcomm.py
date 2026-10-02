# -*- coding: utf-8 -*-
"""
Bluetooth test via /dev/rfcomm devices (more reliable on RPi).

Requires: sudo rfcomm listen hci0 1 running on the server

Usage:
  # On the server (robotA/bropi) - in a separate terminal:
  $ sudo rfcomm listen hci0 1

  # On the server - in another terminal:
  $ python3 bt_test_rfcomm.py --server robotB

  # On the client (robotB/bropi2):
  $ python3 bt_test_rfcomm.py --client robotA
"""

import json
import os
import select
import struct
import sys
import time
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"


def load_device_config():
    """Loads the current device's config"""
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: {DEVICE_ID_PATH} not found", file=sys.stderr)
        sys.exit(1)


def get_peer_addr(config, peer_id, force_addr=None):
    """Gets the peer's address from the config"""
    if force_addr:
        return force_addr

    peers = config.get("peers", {})
    if peer_id in peers:
        return peers[peer_id].get("bd_addr")

    return None


def wait_for_client(rfcomm_device="/dev/rfcomm0"):
    """Waits for a client connection (requires rfcomm listen on the server)"""
    print(f"  Waiting for connection on {rfcomm_device}...")

    # Check that the device exists
    if not os.path.exists(rfcomm_device):
        print(f"  ✗ Error: {rfcomm_device} not found", file=sys.stderr)
        print(f"    Run on the server: sudo rfcomm listen hci0 1", file=sys.stderr)
        return None

    # Open the device
    try:
        f = os.open(rfcomm_device, os.O_RDWR)
        print(f"  ✓ Connected to {rfcomm_device}")
        return f
    except OSError as e:
        print(f"  ✗ Error opening {rfcomm_device}: {e}", file=sys.stderr)
        return None


def connect_to_server(bd_addr, rfcomm_device="/dev/rfcomm0", timeout=10):
    """Connects to the server via rfcomm"""
    print(f"  Connecting to {bd_addr} on {rfcomm_device}...")

    # Use rfcomm connect
    import subprocess

    try:
        result = subprocess.run(
            ["sudo", "rfcomm", "connect", rfcomm_device, bd_addr, "1"],
            timeout=timeout,
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            time.sleep(0.5)  # Allow time for the connection to be established

            # Open the device
            f = os.open(rfcomm_device, os.O_RDWR)
            print(f"  ✓ Connected to {bd_addr}")
            return f
        else:
            print(f"  ✗ Connection error: {result.stderr}", file=sys.stderr)
            return None
    except subprocess.TimeoutExpired:
        print(f"  ✗ Connection timeout", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  ✗ Error: {e}", file=sys.stderr)
        return None


def send_data(fd, data):
    """Sends data"""
    try:
        os.write(fd, data)
        return True
    except Exception as e:
        print(f"  ✗ Send error: {e}", file=sys.stderr)
        return False


def recv_data(fd, size, timeout=2):
    """Receives data with a timeout"""
    try:
        ready = select.select([fd], [], [], timeout)
        if ready[0]:
            return os.read(fd, size)
        else:
            return None
    except Exception as e:
        print(f"  ✗ Receive error: {e}", file=sys.stderr)
        return None


def run_server(my_id, peer_id):
    """Server: receives and responds to tests"""
    print(f"[{my_id}] Starting Bluetooth server")
    print(f"  Make sure this is running: sudo rfcomm listen hci0 1")

    fd = wait_for_client()
    if fd is None:
        return

    try:
        print(f"\n[{my_id}] SERVER — starting test...\n")
        packets_received = 0

        while True:
            data = recv_data(fd, 1024)
            if not data:
                print(f"[{my_id}] Connection closed")
                break

            packets_received += 1
            msg_type = data[0]

            if msg_type == 1:  # PING
                if len(data) >= 5:
                    seq = struct.unpack('<I', data[1:5])[0]
                    print(f"  ← PING #{seq}")

                    response = struct.pack('<BI', 2, seq)
                    send_data(fd, response)
                    print(f"  → PONG #{seq}")

            elif msg_type == 3:  # FAST_DATA
                if len(data) >= 13:
                    seq, x, y, yaw = struct.unpack('<Ifff', data[1:13])
                    print(f"  ← DATA #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

                    ack = struct.pack('<BI', 4, seq)
                    send_data(fd, ack)
                    print(f"  → ACK #{seq}")

            elif msg_type == 99:  # END
                print(f"  ← End signal")
                break

        print(f"\n[{my_id}] Test complete ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Interrupted")
    finally:
        os.close(fd)


def run_client(my_id, peer_id, bd_addr):
    """Client: sends tests"""
    print(f"[{my_id}] Starting Bluetooth client")

    fd = connect_to_server(bd_addr)
    if fd is None:
        return

    try:
        print(f"\n[{my_id}] CLIENT — starting test...\n")

        # PING/PONG
        print("  === PING/PONG ===")
        for i in range(3):
            ping = struct.pack('<BI', 1, i)
            send_data(fd, ping)
            print(f"  → PING #{i}")

            response = recv_data(fd, 1024)
            if response and response[0] == 2:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← PONG #{seq} ✓")
            else:
                print(f"  (no response)")
            time.sleep(0.1)

        # Fast exchange
        print("\n  === Fast exchange (x, y, yaw) ===")
        for i in range(5):
            data_packet = struct.pack('<Ifff', i, 1.5 + i*0.1, 2.0 - i*0.2, 0.5*i)
            msg = struct.pack('<B', 3) + data_packet
            send_data(fd, msg)
            x, y, yaw = struct.unpack('<fff', data_packet)
            print(f"  → DATA #{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

            response = recv_data(fd, 1024)
            if response and response[0] == 4:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← ACK #{seq} ✓")
            else:
                print(f"  (no response)")
            time.sleep(0.05)

        # Speed test
        print("\n  === Speed test (100 packets) ===")
        start_time = time.time()
        packets_sent = 0

        for i in range(100):
            data_packet = struct.pack('<Ifff', i, 1.0, 2.0, 3.0)
            msg = struct.pack('<B', 3) + data_packet
            send_data(fd, msg)
            packets_sent += 1

            response = recv_data(fd, 1024, timeout=0.5)
            if not response:
                break

        elapsed = time.time() - start_time
        rate = packets_sent / elapsed if elapsed > 0 else 0
        print(f"  Sent: {packets_sent} packets in {elapsed:.3f}s ({rate:.1f} pkt/sec)")

        # Finishing
        print("\n  === Finishing ===")
        end_msg = struct.pack('<B', 99)
        send_data(fd, end_msg)
        print(f"  → End signal")

        print(f"\n[{my_id}] Test complete ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Interrupted")
    finally:
        os.close(fd)


def main():
    parser = ArgumentParser(description="Bluetooth test via /dev/rfcomm")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--server", type=str, metavar="TARGET_ID",
                      help="Run the server")
    group.add_argument("--client", type=str, metavar="TARGET_ID",
                      help="Run the client")

    parser.add_argument("--force-addr", type=str,
                      help="Explicit Bluetooth address")
    parser.add_argument("--device", type=str, default="/dev/rfcomm0",
                      help="RFCOMM device (default /dev/rfcomm0)")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")

    print(f"Config: ID={my_id}\n")

    if args.server:
        run_server(my_id, args.server)
    elif args.client:
        bd_addr = get_peer_addr(config, args.client, args.force_addr)
        if not bd_addr:
            print(f"Error: address for {args.client} not found", file=sys.stderr)
            sys.exit(1)
        run_client(my_id, args.client, bd_addr)


if __name__ == "__main__":
    main()
