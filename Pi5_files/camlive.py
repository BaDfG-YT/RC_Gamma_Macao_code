from flask import Flask, Response
import cv2
import numpy as np
from picamera2 import Picamera2

app = Flask(__name__)

# Camera setup
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Orange in HSV (tune for your lighting)
LOWER = np.array([0, 103, 209])
UPPER = np.array([13, 255, 255])
MIN_AREA = 100

def gen():
    while True:
        frame = picam2.capture_array()  # comes out as RGB
        bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER, UPPER)

        mask = cv2.medianBlur(mask, 5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > MIN_AREA:
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(bgr, f"orange area={int(area)}", (x, max(20, y-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            continue

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")

@app.route("/")
def index():
    return '<h3>Camera stream</h3><img src="/video">'

@app.route("/video")
def video():
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    # 0.0.0.0 so it's reachable from the network
    app.run(host="0.0.0.0", port=5000, threaded=True)
