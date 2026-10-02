# -*- coding: utf-8 -*-
"""
Extended version of bt_test.py with automatic address lookup from device_id.local.json.

Usage:
  # Run the server (automatic port selection)
  python3 bt_test_auto.py --server robotB

  # Run the client with automatic address lookup from the config
  python3 bt_test_auto.py --client robotA

  # Run with an explicit address (if not saved in the config)
  python3 bt_test_auto.py --client robotA --force-addr AA:BB:CC:DD:EE:FF
"""

import json
import socket
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
    """Gets the peer's address from the config, or uses the explicit address"""
    if force_addr:
        return force_addr

    peers = config.get("peers", {})
    if peer_id in peers:
        return peers[peer_id].get("bd_addr")

    return None


def run_server(my_config, target_device, channel=1):
    """Starts the Bluetooth server"""
    my_id = my_config.get("id", "unknown")
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


def run_client(my_config, target_device, bd_addr, channel=1):
    """Starts the Bluetooth client"""
    my_id = my_config.get("id", "unknown")
    print(f"[{my_id}] Starting Bluetooth client")
    print(f"  Connecting to {target_device} ({bd_addr}) on channel {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(5)

    try:
        sock.connect((bd_addr, channel))
        print(f"  ✓ Connected to {target_device}")

        test_connection(sock, my_id, target_device, is_server=False)
    except socket.timeout:
        print(f"  ✗ Connection timeout", file=sys.stderr)
    except ConnectionRefusedError:
        print(f"  ✗ Connection refused", file=sys.stderr)
    except Exception as e:
        print(f"  ✗ Client error: {e}", file=sys.stderr)
    finally:
        sock.close()


def test_connection(sock, my_id, peer_id, is_server):
    """Tests the connection and data exchange"""
    sock.settimeout(2)

    try:
        role = "SERVER" if is_server else "CLIENT"
        print(f"\n[{my_id}] {role} — starting test...\n")

        if is_server:
            test_server(sock, my_id, peer_id)
        else:
            test_client(sock, my_id, peer_id)

        print(f"\n[{my_id}] Test complete ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Interrupted by user")
    except socket.timeout:
        print(f"[{my_id}] Connection timeout", file=sys.stderr)
    except Exception as e:
        print(f"[{my_id}] Error: {e}", file=sys.stderr)


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
                    response = struct.pack('<BI', 2, seq)
                    sock.send(response)
                    print(f"  → PONG #{seq}")

            elif msg_type == 3:  # FAST_DATA
                if len(data) >= 13:
                    seq, x, y, yaw = struct.unpack('<Ifff', data[1:13])
                    print(f"  ← DATA #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
                    ack = struct.pack('<BI', 4, seq)
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
    """Client: sends data and waits for responses"""

    print("  === PING/PONG ===")
    for i in range(3):
        ping = struct.pack('<BI', 1, i)
        sock.send(ping)
        print(f"  → PING #{i}")
        try:
            response = sock.recv(1024)
            if response and response[0] == 2:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← PONG #{seq} ✓")
        except socket.timeout:
            print(f"  (no response)")
        time.sleep(0.1)

    print("\n  === Fast exchange (x, y, yaw) ===")
    for i in range(5):
        data_packet = struct.pack('<Ifff', i, 1.5 + i*0.1, 2.0 - i*0.2, 0.5*i)
        msg = struct.pack('<B', 3) + data_packet
        sock.send(msg)
        x, y, yaw = struct.unpack('<fff', data_packet)
        print(f"  → DATA #{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        try:
            response = sock.recv(1024)
            if response and response[0] == 4:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← ACK #{seq} ✓")
        except socket.timeout:
            print(f"  (no response)")
        time.sleep(0.05)

    print("\n  === Speed test (100 packets) ===")
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
    end_msg = struct.pack('<B', 99)
    sock.send(end_msg)
    print(f"  → End signal")


def main():
    parser = ArgumentParser(description="Bluetooth test with automatic address loading")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--server", type=str, metavar="TARGET_ID", help="Run the server (waits for connection from TARGET_ID)")
    group.add_argument("--client", type=str, metavar="TARGET_ID", help="Run the client (connect to TARGET_ID)")

    parser.add_argument("--force-addr", type=str, help="Explicit Bluetooth address (if not saved in the config)")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM channel (default 1)")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")

    if args.server:
        print(f"Config: ID={my_id}")
        run_server(config, args.server, args.channel)

    elif args.client:
        print(f"Config: ID={my_id}")
        bd_addr = get_peer_addr(config, args.client, args.force_addr)

        if not bd_addr:
            print(f"Error: address for {args.client} not found in config", file=sys.stderr)
            print(f"  Use --force-addr or save the address via bt_setup.py", file=sys.stderr)
            sys.exit(1)

        run_client(config, args.client, bd_addr, args.channel)


if __name__ == "__main__":
    main()
