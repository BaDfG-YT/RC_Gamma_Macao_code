import serial
import struct

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

packet_size = 12  # 3 float * 4 байта

while True:
    data = ser.read(packet_size)
    if len(data) != packet_size:
        continue

    x, y, yaw = struct.unpack('<fff', data)   # little-endian, 3 float
    print(f"x={x:.4f}, y={y:.4f}, yaw={yaw:.4f}")