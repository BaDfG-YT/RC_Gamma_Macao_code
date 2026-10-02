# -*- coding: utf-8 -*-
"""
Utility for setting up a Bluetooth connection between Raspberry Pis.

Usage:
  # Scan for available Bluetooth devices
  python3 bt_setup.py --scan

  # Pair a device
  python3 bt_setup.py --pair AA:BB:CC:DD:EE:FF

  # Get the list of paired devices
  python3 bt_setup.py --list

  # Save an address to device_id.local.json
  python3 bt_setup.py --save-peer robotA AA:BB:CC:DD:EE:FF
"""

import json
import subprocess
import sys
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"


def load_device_id():
    """Loads the device config"""
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"id": "unknown", "type": "robot"}


def save_device_id(data):
    """Saves the device config"""
    DEVICE_DIR.mkdir(parents=True, exist_ok=True)
    with open(DEVICE_ID_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def run_command(cmd):
    """Runs a shell command and returns its output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.stdout.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", -1
    except Exception as e:
        print(f"Command execution error: {e}", file=sys.stderr)
        return "", -1


def scan_devices():
    """Scans for available Bluetooth devices"""
    print("Scanning for Bluetooth devices (15 seconds)...\n")
    output, _ = run_command("bluetoothctl --timeout 15 scan on 2>/dev/null | grep -E '\\[NEW\\]|Device'")

    devices = {}
    for line in output.split('\n'):
        if 'Device' in line:
            parts = line.split()
            if len(parts) >= 2:
                addr = parts[1]
                name = ' '.join(parts[2:]) if len(parts) > 2 else "Unknown"
                devices[addr] = name

    if devices:
        print("Devices found:")
        for addr, name in devices.items():
            print(f"  {addr}  {name}")
    else:
        print("No devices found. Make sure Bluetooth is enabled.")

    return devices


def list_paired():
    """Shows the list of paired devices"""
    output, _ = run_command("bluetoothctl paired-devices")

    devices = {}
    for line in output.split('\n'):
        if line.strip():
            parts = line.split()
            if len(parts) >= 2:
                addr = parts[1]
                name = ' '.join(parts[2:]) if len(parts) > 2 else "Unknown"
                devices[addr] = name

    if devices:
        print("Paired devices:")
        for addr, name in devices.items():
            print(f"  {addr}  {name}")
    else:
        print("No paired devices found.")

    return devices


def pair_device(bd_addr):
    """Pairs a device"""
    print(f"Pairing with {bd_addr}...")

    run_command(f"bluetoothctl trust {bd_addr}")
    output, code = run_command(f"bluetoothctl pair {bd_addr}")

    if code == 0 and "successful" in output.lower():
        print(f"✓ Successfully paired")
        return True
    else:
        print(f"✗ Pairing error: {output}")
        return False


def save_peer(peer_id, bd_addr):
    """Saves the peer device's address to the config"""
    device_id = load_device_id()

    if "peers" not in device_id:
        device_id["peers"] = {}

    device_id["peers"][peer_id] = {
        "bd_addr": bd_addr,
        "added_at": __import__('datetime').datetime.now().isoformat()
    }

    save_device_id(device_id)
    print(f"✓ Address for {peer_id} saved: {bd_addr}")
    print(f"  File: {DEVICE_ID_PATH}")


def show_config():
    """Shows the current device config"""
    data = load_device_id()
    print(f"Device config ({DEVICE_ID_PATH}):")
    print(json.dumps(data, indent=2, ensure_ascii=False))


def main():
    parser = ArgumentParser(description="Utility for setting up a Bluetooth connection")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--scan", action="store_true", help="Scan for Bluetooth devices")
    group.add_argument("--list", action="store_true", help="List paired devices")
    group.add_argument("--pair", type=str, metavar="BD_ADDR", help="Pair a device (e.g., AA:BB:CC:DD:EE:FF)")
    group.add_argument("--save-peer", nargs=2, metavar=("PEER_ID", "BD_ADDR"), help="Save a peer's address to the config")
    group.add_argument("--config", action="store_true", help="Show the device config")

    args = parser.parse_args()

    if args.scan:
        scan_devices()
    elif args.list:
        list_paired()
    elif args.pair:
        pair_device(args.pair)
    elif args.save_peer:
        save_peer(args.save_peer[0], args.save_peer[1])
    elif args.config:
        show_config()


if __name__ == "__main__":
    main()
