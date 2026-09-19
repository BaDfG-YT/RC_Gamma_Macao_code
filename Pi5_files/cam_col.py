from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import cv2
import numpy as np
import signal

LOWER = np.array([5, 120, 80])
UPPER = np.array([25, 255, 255])
MIN_AREA = 500

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (640, 480)}
)
picam2.configure(config)

def process_frame(request):
    frame = picam2.capture_array()

    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, LOWER, UPPER)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > MIN_AREA:
            (x, y), radius = cv2.minEnclosingCircle(c)
            cx, cy = int(x), int(y)
            radius = int(radius)

            print(f"Center: ({cx}, {cy})  Radius: {radius}  Area: {int(area)}")

picam2.post_callback = process_frame

picam2.start_preview(Preview.QTGL)  # � ���� ��� ��������
picam2.start()

print("Running... Ctrl+C to stop")
signal.pause()