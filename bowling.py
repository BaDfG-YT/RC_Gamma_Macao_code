#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# cam_red.py - детект красного (кегли) внутри ROI по центру кадра,
# отправка центра масс на пику.
# Протокол: "RED,<dx>,<area>\n"
#   dx   - отклонение центра масс от центра ROI по ширине, px
#          (минус = левее, плюс = правее); 9999 = цели нет
#   area - количество красных пикселей внутри ROI
#
# Запуск:  python3 cam_red.py
# Стрим для настройки: http://<ip_пи>:5000  (ENABLE_WEB = True)
#   /       - подробная разметка (пиксели, контуры, центр масс, ROI, статистика)
#   /mask   - чистая бинарная маска (только внутри ROI) на весь экран

import math
import threading
import time

import numpy as np
import serial
import cv2
from picamera2 import Picamera2
from flask import Flask, Response

# =====================================================================
# Конфиг
# =====================================================================

PICO_PORT = "/dev/ttyACM0"
PICO_BAUD = 115200
SEND_PERIOD = 0.01

FRAME_W = 640
FRAME_H = 480

# --- ROI (зона поиска) по центру кадра ---
# Задаётся шириной/высотой в пикселях; центр ROI = центр кадра.
# Можно сдвинуть центр ROI отдельно (например, если камера смотрит не совсем прямо).
ROI_W = 300
ROI_H = 60
ROI_CENTER_X = FRAME_W // 2 - 20
ROI_CENTER_Y = FRAME_H // 2 - 0

ROI_X0 = max(0, ROI_CENTER_X - ROI_W // 2)
ROI_Y0 = max(0, ROI_CENTER_Y - ROI_H // 2)
ROI_X1 = min(FRAME_W, ROI_X0 + ROI_W)
ROI_Y1 = min(FRAME_H, ROI_Y0 + ROI_H)

CENTER_X = (ROI_X0 + ROI_X1) / 2.0  # dx считаем от центра ROI, не всего кадра

x_ofst = -5
manual_ofst_x = -5

# Красный в HSV живёт на "стыке" круга (H около 0),
# поэтому нужны ДВА диапазона: около 0 и около 179.
RED1_LOWER = np.array([155, 110, 64])
RED1_UPPER = np.array([179, 255, 255])
RED2_LOWER = np.array([0, 70, 0])
RED2_UPPER = np.array([5, 255, 255])

RED_MIN_AREA = 45  # минимум красных пикселей, меньше - считаем что цели нет

# Контуры меньше этой площади подписываются серым (шум)
CONTOUR_NOISE_AREA = 20

NO_TARGET_DX = 9999.0

ENABLE_WEB = True  # веб-стрим с разметкой для настройки диапазонов
WEB_PORT = 5000

# =====================================================================

FRAME_LOCK = threading.Lock()
FRAME_JPEG = None
MASK_JPEG = None

RED_STATE_LOCK = threading.Lock()
RED_STATE = {"dx": NO_TARGET_DX, "area": 0}


def red_mask(hsv_roi):
    """Маска красного, ТОЛЬКО для уже вырезанного ROI-фрагмента."""
    m1 = cv2.inRange(hsv_roi, RED1_LOWER, RED1_UPPER)
    m2 = cv2.inRange(hsv_roi, RED2_LOWER, RED2_UPPER)
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.medianBlur(mask, 3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return mask


def find_red_dx_area(hsv_full):
    """
    Ищет красное ТОЛЬКО внутри ROI (вырезаем фрагмент до поиска -
    дешевле по CPU, чем маскировать весь кадр, и гарантированно
    не даёт ничему снаружи повлиять на результат).
    Возвращает dx относительно центра ROI и координаты центра масс
    В КООРДИНАТАХ ПОЛНОГО КАДРА (для отрисовки).
    """
    hsv_roi = hsv_full[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1]
    mask_roi = red_mask(hsv_roi)

    area = int(cv2.countNonZero(mask_roi))
    if area < RED_MIN_AREA:
        return NO_TARGET_DX, 0, mask_roi, None

    m = cv2.moments(mask_roi, binaryImage=True)
    if m["m00"] == 0:
        return NO_TARGET_DX, 0, mask_roi, None

    cx_roi = m["m10"] / m["m00"]
    cy_roi = m["m01"] / m["m00"]

    # в координаты полного кадра - для отрисовки на overlay
    cx_full = ROI_X0 + cx_roi
    cy_full = ROI_Y0 + cy_roi

    dx = cx_full - CENTER_X + x_ofst + manual_ofst_x
    return dx, area, mask_roi, (int(cx_full), int(cy_full))


# =====================================================================
# Подробная разметка
# =====================================================================

def draw_roi_box(vis):
    cv2.rectangle(vis, (ROI_X0, ROI_Y0), (ROI_X1, ROI_Y1), (255, 255, 0), 1)
    cv2.putText(vis, "ROI", (ROI_X0 + 4, ROI_Y0 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)


def draw_detailed_vis(bgr, mask_roi, dx, area, center, fps):
    vis = bgr.copy()

    # --- 1. Подсветка красных пикселей ВНУТРИ ROI (полупрозрачная заливка) ---
    roi_view = vis[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1]
    overlay = roi_view.copy()
    overlay[mask_roi > 0] = (0, 0, 255)
    cv2.addWeighted(overlay, 0.25, roi_view, 0.75, 0, roi_view)
    vis[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1] = roi_view

    # затемнить область ВНЕ ROI, чтобы визуально подчеркнуть зону поиска
    dim = vis.copy()
    dim[:] = (dim * 0.55).astype(np.uint8)
    dim[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1] = vis[ROI_Y0:ROI_Y1, ROI_X0:ROI_X1]
    vis = dim

    # --- 2. Контуры внутри ROI: каждый со своей площадью ---
    contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    n_valid = 0
    for c in contours:
        c_area = int(cv2.contourArea(c))
        x, y, w, h = cv2.boundingRect(c)
        # смещаем координаты контура в систему полного кадра
        x += ROI_X0
        y += ROI_Y0
        if c_area < CONTOUR_NOISE_AREA:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (120, 120, 120), 1)
            continue
        n_valid += 1
        c_shifted = c + np.array([ROI_X0, ROI_Y0])
        cv2.drawContours(vis, [c_shifted], -1, (0, 255, 255), 1)
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 170, 255), 1)
        cv2.putText(vis, f"{c_area}", (x, max(12, y - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    # --- 3. Рамка ROI (рисуем поверх затемнения - всегда видна) ---
    draw_roi_box(vis)

    # --- 4. Центральная вертикаль ROI и центр масс ---
    cv2.line(vis, (int(CENTER_X), ROI_Y0), (int(CENTER_X), ROI_Y1), (255, 255, 255), 1)

    if center is not None:
        cx, cy = center
        cv2.circle(vis, (cx + x_ofst, cy), 4, (255, 0, 0), -1)
        cv2.circle(vis, (cx + x_ofst, cy), 12, (255, 255, 255), 1)
        cv2.line(vis, (int(CENTER_X), cy), (cx + x_ofst, cy), (0, 255, 255), 1)
        cv2.line(vis, (cx + x_ofst + manual_ofst_x, ROI_Y0), (cx + x_ofst + manual_ofst_x, ROI_Y1), (255, 0, 255), 1)
        arrow_y = ROI_Y1 + 20
        cv2.arrowedLine(vis, (int(CENTER_X) , arrow_y), (cx + x_ofst, arrow_y),
                        (0, 255, 0), 2, tipLength=0.2)

    # --- 5. Панель статистики ---
    panel_h = 92
    panel = vis[0:panel_h, 0:250].copy()
    cv2.rectangle(vis, (0, 0), (250, panel_h), (0, 0, 0), -1)
    cv2.addWeighted(panel, 0.35, vis[0:panel_h, 0:250], 0.65, 0,
                    vis[0:panel_h, 0:250])

    if center is not None:
        side = "LEFT" if dx < 0 else ("RIGHT" if dx > 0 else "CENTER")
        col = (0, 255, 0)
        lines = [
            f"dx = {dx:+.1f} px  ({side})",
            f"area = {area} px",
            f"contours = {n_valid}",
            f"fps = {fps:.1f}",
        ]
    else:
        col = (0, 0, 255)
        lines = [
            "NO RED TARGET",
            f"area = {area} px (min {RED_MIN_AREA})",
            f"ROI {ROI_W}x{ROI_H}",
            f"fps = {fps:.1f}",
        ]

    y = 20
    for ln in lines:
        if ln:
            cv2.putText(vis, ln, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        y += 22

    # --- 6. Мини-маска ROI (правый верхний угол) ---
    mini_w, mini_h = ROI_W, ROI_H
    mini = cv2.resize(mask_roi, (mini_w, mini_h), interpolation=cv2.INTER_NEAREST)
    mini_bgr = cv2.cvtColor(mini, cv2.COLOR_GRAY2BGR)
    x0 = FRAME_W - mini_w - 8
    y0 = 8
    vis[y0:y0 + mini_h, x0:x0 + mini_w] = mini_bgr
    cv2.rectangle(vis, (x0 - 1, y0 - 1), (x0 + mini_w, y0 + mini_h),
                  (200, 200, 200), 1)
    cv2.putText(vis, "mask (ROI)", (x0 + 4, y0 + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return vis


def camera_worker(stop_evt):
    global FRAME_JPEG, MASK_JPEG

    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
    picam2.start()
    time.sleep(2.0)

    fps = 0.0
    fps_t0 = time.time()
    fps_frames = 0

    try:
        while not stop_evt.is_set():
            t0 = time.time()

            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            dx, area_sent, mask_roi, center = find_red_dx_area(hsv)
            area_raw = int(cv2.countNonZero(mask_roi))

            with RED_STATE_LOCK:
                RED_STATE["dx"] = dx
                RED_STATE["area"] = area_sent

            fps_frames += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                fps = fps_frames / (now - fps_t0)
                fps_frames = 0
                fps_t0 = now

            if ENABLE_WEB:
                vis = draw_detailed_vis(bgr, mask_roi, dx, area_raw, center, fps)

                ok, jpg = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                ok2, jpg2 = cv2.imencode(".jpg", mask_roi, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                with FRAME_LOCK:
                    if ok:
                        FRAME_JPEG = jpg.tobytes()
                    if ok2:
                        MASK_JPEG = jpg2.tobytes()

            dt = time.time() - t0
            if SEND_PERIOD - dt > 0:
                time.sleep(SEND_PERIOD - dt)
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
                print(f"[pico] connected {PICO_PORT}")

            with RED_STATE_LOCK:
                dx = RED_STATE["dx"]
                area = RED_STATE["area"]

            ser.write(f"RED,{dx:.1f},{area}\n".encode("utf-8"))

            now = time.time()
            if now - last_log >= 1.0:
                print(f"[pico] sent RED,{dx:.1f},{area}")
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


# ============ Веб-стрим для настройки ============

app = Flask(__name__)

PAGE_TPL = """
<!doctype html><html><head><meta charset="utf-8"><title>{title}</title>
<style>body{{background:#111;color:#eee;font-family:sans-serif;text-align:center}}
img{{margin-top:12px;border:1px solid #444;max-width:95vw}}
a{{color:#6cf;margin:0 8px}}</style></head>
<body><h2>{title}</h2>
<p><a href="/">overlay</a>|<a href="/mask">mask</a></p>
<img src="{stream}"></body></html>
"""


@app.route("/")
def index():
    return PAGE_TPL.format(title="Red tracker (kegli, ROI)", stream="/stream")


@app.route("/mask")
def mask_page():
    return PAGE_TPL.format(title="Red mask (ROI)", stream="/stream_mask")


def mjpeg_gen(get_frame):
    while True:
        frame = get_frame()
        if frame is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)


@app.route("/stream")
def stream():
    def get():
        with FRAME_LOCK:
            return FRAME_JPEG
    return Response(mjpeg_gen(get), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stream_mask")
def stream_mask():
    def get():
        with FRAME_LOCK:
            return MASK_JPEG
    return Response(mjpeg_gen(get), mimetype="multipart/x-mixed-replace; boundary=frame")


def main():
    stop_evt = threading.Event()

    t_cam = threading.Thread(target=camera_worker, args=(stop_evt,), daemon=True)
    t_cam.start()

    t_pico = threading.Thread(target=pico_sender_worker, args=(stop_evt,), daemon=True)
    t_pico.start()

    try:
        if ENABLE_WEB:
            app.run(host="0.0.0.0", port=WEB_PORT, threaded=True)
        else:
            while True:
                time.sleep(1.0)
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()