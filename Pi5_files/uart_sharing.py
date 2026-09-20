import serial
import struct

# Было '/dev/ttyACM0' (USB) -> теперь аппаратный UART Pi.
# /dev/serial0 - стандартный симлинк Raspberry Pi OS на включённый основной UART,
# работает независимо от того, как он называется внутри (ttyAMA0 / ttyS0).
ser = serial.Serial('/dev/serial0', 115200, timeout=1)

packet_size = 12  # 3 float * 4 байта

while True:
    data = ser.read(packet_size)
    if len(data) != packet_size:
        continue

    x, y, yaw = struct.unpack('<fff', data)   # little-endian, 3 float
    print(f"x={x:.4f}, y={y:.4f}, yaw={yaw:.4f}")