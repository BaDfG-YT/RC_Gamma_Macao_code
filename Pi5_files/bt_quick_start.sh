#!/bin/bash
# Quick start for testing a Bluetooth connection between two Raspberry Pis

set -e

echo "=== Bluetooth test — quick start ==="
echo

DEVICE_ID_FILE="../device/device_id.local.json"

# Check that device_id exists
if [ ! -f "$DEVICE_ID_FILE" ]; then
    echo "❌ Error: $DEVICE_ID_FILE not found"
    echo "Make sure the device_id.local.json file exists."
    exit 1
fi

# Get the current device's ID
MY_ID=$(python3 -c "import json; print(json.load(open('$DEVICE_ID_FILE'))['id'])")
echo "📱 Current device: $MY_ID"
echo

# Menu
echo "What do you want to do?"
echo "1) Scan for Bluetooth devices"
echo "2) Pair a device"
echo "3) Show paired devices"
echo "4) Save the peer device's address"
echo "5) Start the Bluetooth server"
echo "6) Start the Bluetooth client"
echo "0) Exit"
echo

read -p "Choose an option [0-6]: " choice

case $choice in
    1)
        echo "Scanning (this will take 15 seconds)..."
        python3 bt_setup.py --scan
        ;;
    2)
        read -p "Enter the Bluetooth address (AA:BB:CC:DD:EE:FF): " bd_addr
        python3 bt_setup.py --pair "$bd_addr"
        ;;
    3)
        python3 bt_setup.py --list
        ;;
    4)
        read -p "Enter the peer device's ID (robotA/robotB): " peer_id
        read -p "Enter its Bluetooth address (AA:BB:CC:DD:EE:FF): " bd_addr
        python3 bt_setup.py --save-peer "$peer_id" "$bd_addr"
        ;;
    5)
        read -p "Enter the target device's ID: " target_id
        echo "Starting the server (waiting for connection from $target_id)..."
        echo "Press Ctrl+C to cancel"
        python3 bt_test_auto.py --server "$target_id"
        ;;
    6)
        read -p "Enter the target device's ID: " target_id
        read -p "Enter the address (press Enter to load from config): " bd_addr_input

        if [ -z "$bd_addr_input" ]; then
            echo "Starting the client (connecting to $target_id)..."
            python3 bt_test_auto.py --client "$target_id"
        else
            echo "Starting the client with an explicit address..."
            python3 bt_test_auto.py --client "$target_id" --force-addr "$bd_addr_input"
        fi
        ;;
    0)
        echo "Goodbye!"
        ;;
    *)
        echo "❌ Unknown option"
        exit 1
        ;;
esac

echo
echo "✓ Done"
