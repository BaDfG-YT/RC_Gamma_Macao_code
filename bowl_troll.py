#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# teleop.py - ручное управление роботом с клавиатуры через браузер + живая камера.
#
# Открой http://<ip_пи>:5000, кликни по странице (чтобы ловились клавиши):
#   W - вперёд  move(50,50)      S - назад   move(-50,-50)
#   D - направо move(50,-50)     A - налево  move(-50,50)
#   E - дуга    move(50,0)       Q - дуга    move(0,50)
#   K - удар (kick)
# Отпустил клавишу - робот останавливается.
#
# Протокол на пику:
#   "MOV,<a>,<b>\n" - скорости моторов (шлётся непрерывно, 20 Гц, как keepalive)
#   "KICK\n"        - одиночный удар
# Пика должна останавливаться сама, если MOV не приходит > 400 мс (deadman).

import threading
import time

import serial
import cv2
from picamera2 import Picamera2
from flask import Flask, Response, request, jsonify

# =====================================================================
PICO_PORT = "/dev/ttyACM0"
PICO_BAUD = 115200
SEND_PERIOD = 0.05  # 20 Гц keepalive

FRAME_W = 640
FRAME_H = 480
ROTATE_180 = True   # камера смонтирована вверх ногами

SPEED_FWD = 50
SPEED_TRN = 30

WEB_PORT = 5000
# =====================================================================

FRAME_LOCK = threading.Lock()
FRAME_JPEG = None

CMD_LOCK = threading.Lock()
CMD = {"a": 0, "b": 0}      # текущая команда движения (keepalive)
KICK_QUEUE = []             # одиночные команды

# какие (a, b) соответствуют клавишам
KEYMAP = {
    "w": (SPEED_FWD, SPEED_FWD),
    "s": (-SPEED_FWD, -SPEED_FWD),
    "d": (SPEED_TRN, -SPEED_TRN),
    "a": (-SPEED_TRN, SPEED_TRN),
    "e": (SPEED_TRN, 0),
    "q": (0, SPEED_TRN),
}


def camera_worker(stop_evt):
    global FRAME_JPEG
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
    picam2.start()
    time.sleep(1.5)
    try:
        while not stop_evt.is_set():
            frame = picam2.capture_array()
            if ROTATE_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # показать текущую команду поверх картинки
            with CMD_LOCK:
                a, b = CMD["a"], CMD["b"]
            cv2.putText(bgr, f"MOV {a} {b}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if (a or b) else (180, 180, 180), 2)

            ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                with FRAME_LOCK:
                    FRAME_JPEG = jpg.tobytes()
            time.sleep(0.03)   # ~30 fps
    finally:
        try:
            picam2.stop()
        except Exception:
            pass


def pico_sender_worker(stop_evt):
    ser = None
    last_log = 0.0
    while not stop_evt.is_set():
        try:
            if ser is None or not ser.is_open:
                print(f"[pico] opening {PICO_PORT}...")
                ser = serial.Serial(PICO_PORT, PICO_BAUD, timeout=0.1)
                time.sleep(2.0)
                print(f"[pico] connected")

            # одиночные команды (kick)
            cmds = []
            with CMD_LOCK:
                if KICK_QUEUE:
                    cmds = KICK_QUEUE[:]
                    KICK_QUEUE.clear()
                a, b = CMD["a"], CMD["b"]

            for c in cmds:
                ser.write((c + "\n").encode())

            # keepalive движения - шлём всегда, даже 0,0
            ser.write(f"MOV,{a},{b}\n".encode())

            now = time.time()
            if now - last_log >= 1.0:
                print(f"[pico] MOV,{a},{b}")
                last_log = now

            time.sleep(SEND_PERIOD)
        except Exception as e:
            print(f"[pico] ERROR: {e}")
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(0.5)


# ============================ Web ============================

app = Flask(__name__)

PAGE = """
<!doctype html><html><head><meta charset="utf-8"><title>Teleop</title>
<style>
body{background:#111;color:#eee;font-family:sans-serif;text-align:center}
img{margin-top:10px;border:1px solid #444;max-width:95vw}
#st{font-size:20px;margin:8px;color:#6f6}
kbd{background:#333;padding:2px 8px;border-radius:4px;margin:0 2px}
</style></head>
<body>
<h2>Robot teleop</h2>
<div>
<kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> движение,
<kbd>Q</kbd><kbd>E</kbd> дуги, <kbd>K</kbd> удар. Кликни по странице!
</div>
<div id="st">stop</div>
<img src="/stream">
<script>
const moveKeys = new Set(["w","a","s","d","q","e"]);
let pressed = new Set();
let lastSent = "";

function send(){
  // приоритет: последняя нажатая из активных
  let key = "";
  for (const k of pressed) key = k;   // Set хранит порядок добавления
  if (key !== lastSent) {
    lastSent = key;
    document.getElementById("st").textContent = key ? key.toUpperCase() : "stop";
    fetch("/key", {method:"POST", headers:{"Content-Type":"application/json"},
                   body: JSON.stringify({key: key})});
  }
}

document.addEventListener("keydown", (ev) => {
  const k = ev.key.toLowerCase();
  if (ev.repeat) return;
  if (k === "k") {
    fetch("/kick", {method:"POST"});
    const st = document.getElementById("st");
    st.textContent = "KICK!";
    setTimeout(send, 300);
    return;
  }
  if (moveKeys.has(k)) { pressed.add(k); send(); }
});

document.addEventListener("keyup", (ev) => {
  const k = ev.key.toLowerCase();
  if (moveKeys.has(k)) { pressed.delete(k); send(); }
});

// страховка: ушёл фокус со страницы - стоп
window.addEventListener("blur", () => { pressed.clear(); send(); });
</script>
</body></html>
"""


@app.route("/")
def index():
    return PAGE


@app.route("/key", methods=["POST"])
def key():
    k = (request.get_json(force=True) or {}).get("key", "")
    a, b = KEYMAP.get(k, (0, 0))
    with CMD_LOCK:
        CMD["a"] = a
        CMD["b"] = b
    return jsonify(ok=True, a=a, b=b)


@app.route("/kick", methods=["POST"])
def kick():
    with CMD_LOCK:
        KICK_QUEUE.append("KICK")
    return jsonify(ok=True)


@app.route("/stream")
def stream():
    def gen():
        while True:
            with FRAME_LOCK:
                frame = FRAME_JPEG
            if frame is not None:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
    stop_evt = threading.Event()
    threading.Thread(target=camera_worker, args=(stop_evt,), daemon=True).start()
    threading.Thread(target=pico_sender_worker, args=(stop_evt,), daemon=True).start()
    try:
        app.run(host="0.0.0.0", port=WEB_PORT, threaded=True)
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()