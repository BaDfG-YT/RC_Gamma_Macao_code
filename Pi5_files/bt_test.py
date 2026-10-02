# -*- coding: utf-8 -*-
"""
Test of a Bluetooth connection and fast data exchange between two Raspberry Pis.

Uses RFCOMM to establish a stable connection.
Requires the devices to be paired beforehand via bluetoothctl.

Usage:
  # On the listener device (robotA):
  python3 bt_test.py --server --target-device robotB --bd-addr AA:BB:CC:DD:EE:FF

  # On the initiator device (robotB):
  python3 bt_test.py --client --target-device robotA --bd-addr 11:22:33:44:55:66
"""

import json
import os
import socket
import struct
import sys
import time
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"


def load_device_id():
    """Loads the current device's ID from device_id.local.json"""
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            data = json.load(f)
            return data.get("id", "unknown")
    except FileNotFoundError:
        return "unknown"


def run_server(target_device, channel=1):
    """Starts a Bluetooth server (listener) on the RFCOMM channel"""
    my_id = load_device_id()
    print(f"[{my_id}] Starting Bluetooth server, channel {channel}")
    print(f"  Waiting for connection from {target_device}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.bind((socket.BDADDR_ANY, channel))
    sock.listen(1)
    sock.settimeout(None)

    try:
        client_sock, client_addr = sock.accept()
        print(f"  ✓ Connected: {client_addr}")

        test_connection(client_sock, my_id, target_device, is_server=True)
    except KeyboardInterrupt:
        print("\n  Cancelling...")
    except Exception as e:
        print(f"  ✗ Server error: {e}", file=sys.stderr)
    finally:
        sock.close()


def run_client(target_device, bd_addr, channel=1):
    """Starts a Bluetooth client (initiator) connecting to the server"""
    my_id = load_device_id()
    print(f"[{my_id}] Starting Bluetooth client")
    print(f"  Connecting to {target_device} ({bd_addr}) on channel {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(5)

    try:
        sock.connect((bd_addr, channel))
        print(f"  ✓ Connected to {target_device}")

        test_connection(sock, my_id, target_device, is_server=False)
    except socket.timeout:
        print(f"  ✗ Connection timeout to {bd_addr}", file=sys.stderr)
    except ConnectionRefusedError:
        print(f"  ✗ Connection refused. Make sure the server is running.", file=sys.stderr)
    except Exception as e:
        print(f"  ✗ Client error: {e}", file=sys.stderr)
    finally:
        sock.close()


def test_connection(sock, my_id, peer_id, is_server):
    """Tests the connection and data exchange"""
    sock.settimeout(2)

    try:
        role = "SERVER" if is_server else "CLIENT"
        print(f"\n[{my_id}] {role} — starting test...")

        if is_server:
            test_server(sock, my_id, peer_id)
        else:
            test_client(sock, my_id, peer_id)

        print(f"[{my_id}] Test complete ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Interrupted by user")
    except socket.timeout:
        print(f"[{my_id}] Connection timeout", file=sys.stderr)
    except Exception as e:
        print(f"[{my_id}] Error in test: {e}", file=sys.stderr)


def test_server(sock, my_id, peer_id):
    """Server: receives tests from the client and responds"""
    packets_received = 0
    bytes_received = 0

    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print(f"[{my_id}] Connection closed by peer")
                break

            packets_received += 1
            bytes_received += len(data)

            msg_type = data[0]

            if msg_type == 1:  # PING
                if len(data) >= 5:
                    seq = struct.unpack('<I', data[1:5])[0]
                    print(f"  ← PING #{seq} ({len(data)} bytes)")

                    response = struct.pack('<BI', 2, seq)  # PONG
                    sock.send(response)
                    print(f"  → PONG #{seq}")

            elif msg_type == 3:  # FAST_DATA
                if len(data) >= 13:
                    seq, x, y, yaw = struct.unpack('<Ifff', data[1:13])
                    print(f"  ← DATA #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

                    ack = struct.pack('<BI', 4, seq)  # ACK
                    sock.send(ack)
                    print(f"  → ACK #{seq}")

            elif msg_type == 99:  # END
                print(f"  ← End-of-test signal")
                break

        except socket.timeout:
            if packets_received > 0:
                print(f"  Timeout. Received {packets_received} packets, {bytes_received} bytes")
            break


def test_client(sock, my_id, peer_id):
    """Client: sends different data types and waits for responses"""

    print("\n  === PING/PONG test ===")
    for i in range(3):
        ping = struct.pack('<BI', 1, i)  # msg_type=1 (PING), seq=i
        sock.send(ping)
        print(f"  → PING #{i}")

        response = sock.recv(1024)
        if response and response[0] == 2:  # PONG
            seq = struct.unpack('<I', response[1:5])[0]
            print(f"  ← PONG #{seq} ✓")

        time.sleep(0.1)

    print("\n  === Fast data exchange test (x, y, yaw) ===")
    for i in range(5):
        data_packet = struct.pack('<Ifff', i, 1.5 + i*0.1, 2.0 - i*0.2, 0.5*i)
        msg = struct.pack('<B', 3) + data_packet  # msg_type=3 (FAST_DATA)
        sock.send(msg)
        x, y, yaw = struct.unpack('<fff', data_packet)
        print(f"  → DATA #{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        response = sock.recv(1024)
        if response and response[0] == 4:  # ACK
            seq = struct.unpack('<I', response[1:5])[0]
            print(f"  ← ACK #{seq} ✓")

        time.sleep(0.05)

    print("\n  === Speed test (100 packets in a row) ===")
    start_time = time.time()
    packets_sent = 0

    for i in range(100):
        data_packet = struct.pack('<Ifff', i, 1.0, 2.0, 3.0)
        msg = struct.pack('<B', 3) + data_packet
        sock.send(msg)
        packets_sent += 1

        try:
            response = sock.recv(1024)
            if not response:
                break
        except socket.timeout:
            pass

    elapsed = time.time() - start_time
    rate = packets_sent / elapsed if elapsed > 0 else 0
    print(f"  Sent: {packets_sent} packets in {elapsed:.3f}s ({rate:.1f} pkt/sec)")

    print("\n  === Finishing ===")
    end_msg = struct.pack('<B', 99)  # END
    sock.send(end_msg)
    print(f"  → End signal")


def main():
    parser = ArgumentParser(description="Test of a Bluetooth connection between two Raspberry Pis")
    parser.add_argument(
        "--server",
        action="store_true",
        help="Run in server (listener) mode"
    )
    parser.add_argument(
        "--client",
        action="store_true",
        help="Run in client (initiator) mode"
    )
    parser.add_argument(
        "--target-device",
        type=str,
        required=True,
        help="Target device ID (e.g., robotA)"
    )
    parser.add_argument(
        "--bd-addr",
        type=str,
        help="Target device's Bluetooth address (e.g., AA:BB:CC:DD:EE:FF). Required for --client"
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=1,
        help="RFCOMM channel (default 1)"
    )

    args = parser.parse_args()

    if not args.server and not args.client:
        parser.print_help()
        sys.exit(1)

    if args.server:
        run_server(args.target_device, args.channel)
    elif args.client:
        if not args.bd_addr:
            print("Error: --bd-addr is required for --client mode", file=sys.stderr)
            sys.exit(1)
        run_client(args.target_device, args.bd_addr, args.channel)


if __name__ == "__main__":
    main()
