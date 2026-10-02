# Bluetooth connection test between Raspberry Pis

Two scripts for testing the Bluetooth (BT) connection and quick data exchange between two Raspberry Pi boards.

## Files

- **bt_test.py** — main connection and data-exchange test
- **bt_setup.py** — utility for configuring and pairing devices

## Prerequisites

### 1. Make sure each Pi has a device_id

Each device needs a `device/device_id.local.json`:

```json
{
  "id": "robotA",
  "type": "robot",
  "target_host": null
}
```

The ID can be any unique string (robotA, robotB, bot1, bot2, etc.).

### 2. Find the devices' Bluetooth addresses

On one of the Pis, run:

```bash
python3 bt_setup.py --scan
```

This scans for available Bluetooth devices and shows their addresses (format: AA:BB:CC:DD:EE:FF).

### 3. Pair the devices

On each Pi:

```bash
# Pair with the other device
python3 bt_setup.py --pair AA:BB:CC:DD:EE:FF

# Check the list of paired devices
python3 bt_setup.py --list
```

### 4. Save the addresses to the config

On each Pi, save the target device's address:

```bash
# On robotA: save robotB's address
python3 bt_setup.py --save-peer robotB AA:BB:CC:DD:EE:FF

# On robotB: save robotA's address
python3 bt_setup.py --save-peer robotA 11:22:33:44:55:66
```

This updates `device_id.local.json`:

```json
{
  "id": "robotB",
  "type": "robot",
  "target_host": null,
  "peers": {
    "robotA": {
      "bd_addr": "11:22:33:44:55:66",
      "added_at": "2026-10-02T..."
    }
  }
}
```

## Running the test

### Option 1: Simple test (if addresses are known)

**On the first Pi (server):**
```bash
python3 bt_test.py --server --target-device robotB --bd-addr AA:BB:CC:DD:EE:FF
```

**On the second Pi (client):**
```bash
python3 bt_test.py --client --target-device robotA --bd-addr 11:22:33:44:55:66
```

The server will wait for a connection from the client.

### Option 2: Automatically read the address from the config

bt_test.py can be extended to automatically read the address from `device_id.local.json`.

## What is tested

1. **Connection (RFCOMM)** — basic connection establishment
2. **PING/PONG** — round-trip check (3 iterations)
3. **Fast data exchange** — sending position (x, y, yaw) with acknowledgment (5 iterations)
4. **Throughput** — 100 packets in a row with timing

## Message types

| Type | Code | Description | Format |
|-----|-----|---------|--------|
| PING | 1 | Connectivity check | `[1][seq:4]` |
| PONG | 2 | Reply to PING | `[2][seq:4]` |
| FAST_DATA | 3 | Position data | `[3][seq:4][x:4][y:4][yaw:4]` |
| ACK | 4 | Acknowledgment | `[4][seq:4]` |
| END | 99 | End of test | `[99]` |

All numeric values are little-endian (same as in uart_sharing.py).

## Troubleshooting

### Error: "No such file or directory" when running on Linux

Some systems need extra dependencies:

```bash
sudo apt-get install python3-bluez bluez-tools
```

### Error: "Connection refused"

- Make sure the server is started first
- Check that the Bluetooth address is correct
- Make sure the devices are paired

### Timeout when connecting

- Make sure both devices are powered on and Bluetooth is active
- Use `bluetoothctl` to check status:
  ```bash
  bluetoothctl show
  bluetoothctl paired-devices
  ```

### Low throughput

This can be normal for RFCOMM — its throughput is limited. If you need higher speed, consider:
- BLE (Bluetooth Low Energy) with a custom GATT service
- SPP (Serial Port Profile) with a dedicated channel
- A wired connection (USB, UART, CAN)

## Example output

### Server

```
[robotA] Starting Bluetooth server, channel 1
  Waiting for connection from robotB...
  ✓ Connected: ('AA:BB:CC:DD:EE:FF', 0)

[robotA] SERVER — starting test...
  ← PING #0 (5 bytes)
  → PONG #0
  ← PING #1 (5 bytes)
  → PONG #1
  ← DATA #0: x=1.500, y=2.000, yaw=0.000
  → ACK #0
  ...
  ← End-of-test signal
[robotA] Test complete ✓
```

### Client

```
[robotB] Starting Bluetooth client
  Connecting to robotA (11:22:33:44:55:66) on channel 1...
  ✓ Connected to robotA

[robotB] CLIENT — starting test...
  === PING/PONG test ===
  → PING #0
  ← PONG #0 ✓
  ...
  === Fast data exchange test (x, y, yaw) ===
  → DATA #0: x=1.500, y=2.000, yaw=0.000
  ← ACK #0 ✓
  ...
  === Speed test (100 packets in a row) ===
  Sent: 100 packets in 0.523s (191.2 pkt/s)
  ...
  [robotB] Test complete ✓
```
