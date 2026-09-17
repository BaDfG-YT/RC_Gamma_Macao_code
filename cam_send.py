from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import math

# =========================
# Настройки камеры
# =========================
FRAME_W = 640
FRAME_H = 480

# Центр зеркала на изображении
CENTER_X = FRAME_W / 2
CENTER_Y = FRAME_H / 2

# =========================
# HSV диапазоны
# =========================

# Мяч (оранжевый) — оставить как есть
BALL_LOWER = np.array([4, 110, 150])
BALL_UPPER = np.array([23, 210, 255])
BALL_MIN_AREA = 50

# Жёлтые ворота
YELLOW_LOWER = np.array([25, 163, 147])
YELLOW_UPPER = np.array([40, 255, 255])
YELLOW_MIN_AREA = 200

# Синие ворота
BLUE_LOWER = np.array([90, 73, 80])
BLUE_UPPER = np.array([134, 255, 255])
BLUE_MIN_AREA = 200
# BLUE_LOWER = np.array([95, 239, 73])
# BLUE_UPPER = np.array([116, 255, 255])
# BLUE_MIN_AREA = 200

# =========================
# Настройки USB -> Pico
# =========================
PICO_PORT = "/dev/ttyACM0"
PICO_BAUD = 115200
SEND_PERIOD = 0.05

# =========================
# Инициализация
# =========================
picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H)}
    )
)
picam2.start()
time.sleep(2.0)

ser = serial.Serial(PICO_PORT, PICO_BAUD, timeout=0.1)
time.sleep(2.0)


def compute_angle_360(cx, cy, center_x, center_y):
    dx = cx - center_x
    dy = cy - center_y

    angle = math.degrees(math.atan2(dx, -dy))

    if angle > 180:
        angle -= 360
    if angle <= -180:
        angle += 360

    return -angle


def preprocess_mask(hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return mask


# Для мяча — как было: по крупнейшему контуру
def find_object_angle_area(hsv, lower, upper, min_area):
    mask = preprocess_mask(hsv, lower, upper)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    angle = 0.0
    area_to_send = 0

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > min_area:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w / 2.0
            cy = y + h / 2.0

            angle = compute_angle_360(cx, cy, CENTER_X, CENTER_Y)
            area_to_send = int(area)

    return angle, area_to_send


# Для ворот — центр и площадь всех точек цвета
def find_color_cloud_angle_area(hsv, lower, upper, min_area):
    mask = preprocess_mask(hsv, lower, upper)

    area_to_send = int(cv2.countNonZero(mask))
    if area_to_send < min_area:
        return 0.0, 0

    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] == 0:
        return 0.0, 0

    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]

    angle = compute_angle_360(cx, cy, CENTER_X, CENTER_Y)
    return angle, area_to_send


def send_line(name, angle, area):
    msg = f"{name},{angle:.2f},{area}\n"
    ser.write(msg.encode("utf-8"))
    return msg.strip()


def find_and_send_all():
    frame = picam2.capture_array()
    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Мяч — по крупнейшему контуру
    ball_angle, ball_area = find_object_angle_area(
        hsv, BALL_LOWER, BALL_UPPER, BALL_MIN_AREA
    )

    # Ворота — по всем точкам цвета
    yellow_angle, yellow_area = find_color_cloud_angle_area(
        hsv, YELLOW_LOWER, YELLOW_UPPER, YELLOW_MIN_AREA
    )

    blue_angle, blue_area = find_color_cloud_angle_area(
        hsv, BLUE_LOWER, BLUE_UPPER, BLUE_MIN_AREA
    )

    out1 = send_line("BALL", ball_angle, ball_area)
    out2 = send_line("YELLOW", yellow_angle, yellow_area)
    out3 = send_line("BLUE", blue_angle, blue_area)

    ser.flush()

    print(out1)
    print(out2)
    print(out3)
    print("-----")


try:
    while True:
        t0 = time.time()
        find_and_send_all()

        dt = time.time() - t0
        sleep_time = SEND_PERIOD - dt
        if sleep_time > 0:
            time.sleep(sleep_time)

except KeyboardInterrupt:
    pass

finally:
    try:
        ser.close()
    except Exception:
        pass
    try:
        picam2.stop()
    except Exception:
        pass

# git test