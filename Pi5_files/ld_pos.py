#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import io
import math
import threading
import time
from collections import deque

import numpy as np
from lidar_cfg import CRC_TABLE, CRC_INIT
import serial
from serial.tools import list_ports
from flask import Flask, Response
from PIL import Image, ImageDraw

# ---------------- LD19 protocol ----------------

PKT = 47
HDR = b"\x54\x2C"
NPTS = 12

FIELD_W = 1180.0  # mm
FIELD_H = 1000.0  # mm

# If the lidar isn't mounted at the robot's center, an offset can be added here later
LIDAR_OFFSET_X = 0.0  # mm
LIDAR_OFFSET_Y = 0.0  # mm




def crc_ok(mv):
    crc = CRC_INIT
    for b in mv[2:46]:
        crc = int(CRC_TABLE[(crc ^ b) & 0xFF])
    return (crc & 0xFF) == mv[46]


def parse_packet_polar(mv):
    s = (int(mv[4]) | (int(mv[5]) << 8)) / 100.0
    e = (int(mv[42]) | (int(mv[43]) << 8)) / 100.0
    d = (360.0 - s + e) if (s > 270.0 and e < 90.0) else (e - s)

    t = np.linspace(0.0, 1.0, NPTS, dtype=np.float32)
    ang = ((s + d * t) % 360.0).astype(np.float32)

    dist = np.empty(NPTS, np.int32)
    off = 6
    for i in range(NPTS):
        lo = int(mv[off])
        hi = int(mv[off + 1])
        dist[i] = lo | (hi << 8)
        off += 3

    mask = dist > 0
    if not np.any(mask):
        return None, None
    return ang[mask], dist[mask].astype(np.float32)


def autodetect(vid=None, pid=None):
    for p in list_ports.comports():
        if not p.device:
            continue
        if vid and pid:
            if p.vid is None or p.pid is None:
                continue
            if f"{p.vid:04x}".lower() != vid.lower() or f"{p.pid:04x}".lower() != pid.lower():
                continue
        return p.device
    return None


# ---------------- shared state ----------------

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.points_xy = np.empty((0, 2), dtype=np.float32)
        self.pose = None
        self.port = "N/A"
        self.bytes_per_sec = 0
        self.hdr_per_sec = 0

    def update_points(self, pts):
        with self.lock:
            self.points_xy = pts

    def update_pose(self, pose):
        with self.lock:
            self.pose = pose

    def update_stats(self, port, bytes_sec, hdr_sec):
        with self.lock:
            self.port = port
            self.bytes_per_sec = bytes_sec
            self.hdr_per_sec = hdr_sec

    def snapshot(self):
        with self.lock:
            pts = self.points_xy.copy()
            pose = None if self.pose is None else dict(self.pose)
            return pts, pose, self.port, self.bytes_per_sec, self.hdr_per_sec


class PoseFilter:
    def __init__(self, hist_len=5, alpha_xy=0.25, alpha_yaw=0.2,
                 max_step_xy=8.0, max_step_yaw=8.0, max_score=25.0):
        self.hist_len = hist_len
        self.alpha_xy = alpha_xy
        self.alpha_yaw = alpha_yaw
        self.max_step_xy = max_step_xy
        self.max_step_yaw = max_step_yaw
        self.max_score = max_score

        self.hist_x = deque(maxlen=hist_len)
        self.hist_y = deque(maxlen=hist_len)
        self.hist_yaw = deque(maxlen=hist_len)

        self.filtered = None

    def update(self, pose):
        if pose is None:
            return self.filtered

        # discard solutions that are too bad
        if pose["score"] > self.max_score:
            return self.filtered

        self.hist_x.append(pose["x"])
        self.hist_y.append(pose["y"])
        self.hist_yaw.append(pose["yaw"])

        # while there's little history — just accept the value
        med_x = float(np.median(np.array(self.hist_x, dtype=np.float32)))
        med_y = float(np.median(np.array(self.hist_y, dtype=np.float32)))
        med_yaw = float(np.median(np.array(self.hist_yaw, dtype=np.float32))) % 180.0

        measured = {
            "x": med_x,
            "y": med_y,
            "yaw": med_yaw,
            "score": pose["score"],
        }

        if self.filtered is None:
            self.filtered = measured
            return self.filtered

        # clamp the step in XY
        dx = measured["x"] - self.filtered["x"]
        dy = measured["y"] - self.filtered["y"]

        dx = clamp(dx, -self.max_step_xy, self.max_step_xy)
        dy = clamp(dy, -self.max_step_xy, self.max_step_xy)

        limited_x = self.filtered["x"] + dx
        limited_y = self.filtered["y"] + dy

        # clamp the step in angle
        dyaw = angle_diff_deg(measured["yaw"], self.filtered["yaw"])
        dyaw = clamp(dyaw, -self.max_step_yaw, self.max_step_yaw)
        limited_yaw = self.filtered["yaw"] + dyaw
        while limited_yaw < 0.0:
            limited_yaw += 180.0
        while limited_yaw >= 180.0:
            limited_yaw -= 180.0

        # EMA smoothing
        fx = self.alpha_xy * limited_x + (1.0 - self.alpha_xy) * self.filtered["x"]
        fy = self.alpha_xy * limited_y + (1.0 - self.alpha_xy) * self.filtered["y"]
        fyaw = angle_blend_deg(self.filtered["yaw"], limited_yaw, self.alpha_yaw)

        self.filtered = {
            "x": fx,
            "y": fy,
            "yaw": fyaw,
            "score": pose["score"],
        }
        return self.filtered


STATE = SharedState()

POSE_FILTER = PoseFilter(
    hist_len=5,
    alpha_xy=0.25,
    alpha_yaw=0.20,
    max_step_xy=8.0,
    max_step_yaw=8.0,
    max_score=25.0
)

# ---------------- pose estimation ----------------

def rot2d(points, deg):
    a = np.deg2rad(deg)
    c = np.cos(a)
    s = np.sin(a)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    return points @ R.T


def robust_minmax(v, q_low=2.0, q_high=98.0):
    return np.percentile(v, q_low), np.percentile(v, q_high)


def estimate_pose(points_xy, field_w=FIELD_W, field_h=FIELD_H, prev_pose=None):
    if points_xy.shape[0] < 30:
        return None

    best = None

    # if a pose already exists — search near it
    if prev_pose is not None:
        yaw0 = prev_pose["yaw"] % 180.0
        coarse_angles = np.arange(yaw0 - 8.0, yaw0 + 8.1, 1.0, dtype=np.float32)
    else:
        coarse_angles = np.arange(0.0, 180.0, 1.0, dtype=np.float32)

    for theta in coarse_angles:
        t = theta % 180.0
        rp = rot2d(points_xy, -t)
        u = rp[:, 0]
        v = rp[:, 1]

        umin, umax = robust_minmax(u)
        vmin, vmax = robust_minmax(v)

        span_u = umax - umin
        span_v = vmax - vmin

        span_err = abs(span_u - field_w) + abs(span_v - field_h)

        du = np.minimum(np.abs(u - umin), np.abs(u - umax))
        dv = np.minimum(np.abs(v - vmin), np.abs(v - vmax))
        wall_dist = np.minimum(du, dv)

        fit_err = np.percentile(wall_dist, 70)

        score = span_err * 3.0 + fit_err

        if best is None or score < best["score"]:
            best = {
                "theta": float(t),
                "umin": float(umin),
                "vmin": float(vmin),
                "score": float(score),
            }

    theta0 = best["theta"]
    fine_angles = np.arange(theta0 - 1.0, theta0 + 1.01, 0.1, dtype=np.float32)

    for theta in fine_angles:
        t = theta % 180.0
        rp = rot2d(points_xy, -t)
        u = rp[:, 0]
        v = rp[:, 1]

        umin, umax = robust_minmax(u)
        vmin, vmax = robust_minmax(v)

        span_u = umax - umin
        span_v = vmax - vmin

        span_err = abs(span_u - field_w) + abs(span_v - field_h)

        du = np.minimum(np.abs(u - umin), np.abs(u - umax))
        dv = np.minimum(np.abs(v - vmin), np.abs(v - vmax))
        wall_dist = np.minimum(du, dv)

        fit_err = np.percentile(wall_dist, 70)
        score = span_err * 3.0 + fit_err

        if score < best["score"]:
            best = {
                "theta": float(t),
                "umin": float(umin),
                "vmin": float(vmin),
                "score": float(score),
            }

    x_robot = -best["umin"]
    y_robot = -best["vmin"]
    yaw = best["theta"] % 180.0

    return {
        "x": float(x_robot),
        "y": float(y_robot),
        "yaw": float(yaw),
        "score": float(best["score"]),
    }

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def angle_diff_deg(a, b):
    """
    Minimal angle difference for the [0, 180) range.
    """
    d = a - b
    while d >= 90.0:
        d -= 180.0
    while d < -90.0:
        d += 180.0
    return d

def angle_blend_deg(prev, new, alpha):
    d = angle_diff_deg(new, prev)
    out = prev + alpha * d
    while out < 0.0:
        out += 180.0
    while out >= 180.0:
        out -= 180.0
    return out

# ---------------- serial worker ----------------

def serial_worker(get_port, baud, stop_evt, timeout=0.02):
    prev = None
    last_new_ts = 0.0

    while not stop_evt.is_set():
        port = get_port()
        if not port:
            time.sleep(0.2)
            continue

        ser = None
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            ser.dtr = False
            ser.rts = False
            time.sleep(0.05)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.10)
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            buf = bytearray()
            t0 = time.time()
            bytes_sec = 0
            hdr_sec = 0

            st = 0
            pkt = bytearray(PKT)
            idx = 0

            # current full revolution
            all_pts = []

            while not stop_evt.is_set():
                chunk = ser.read(4096)
                if chunk:
                    buf += chunk
                    bytes_sec += len(chunk)

                i = 0
                while i < len(buf):
                    b = buf[i]
                    i += 1

                    if st == 0:
                        if b == HDR[0]:
                            pkt[0] = b
                            idx = 1
                            st = 1
                    elif st == 1:
                        if b == HDR[1]:
                            pkt[1] = b
                            idx = 2
                            st = 2
                            hdr_sec += 1
                        else:
                            st = 0
                    else:
                        pkt[idx] = b
                        idx += 1
                        if idx == PKT:
                            mv = memoryview(pkt)
                            if crc_ok(mv):
                                ang, dist = parse_packet_polar(mv)
                                if ang is not None:
                                    a = np.deg2rad(ang)
                                    # direction is already corrected here
                                    xs = dist * np.cos(a)
                                    ys = dist * np.sin(a)
                                    pts = np.column_stack((xs, ys)).astype(np.float32)
                                    all_pts.append(pts)

                                s = (int(mv[4]) | (int(mv[5]) << 8)) / 100.0
                                now = time.time()

                                # new revolution
                                if prev is not None and prev > 340.0 and s < 20.0 and (now - last_new_ts) > 0.08:
                                    if all_pts:
                                        full_pts = np.vstack(all_pts)

                                        # moderate range-based outlier rejection
                                        rr = np.linalg.norm(full_pts, axis=1)
                                        m = (rr > 5.0) & (rr < 2500.0)
                                        full_pts = full_pts[m]

                                        STATE.update_points(full_pts)

                                        pose_raw = estimate_pose(full_pts)
                                        STATE.update_pose(pose_raw)

                                    all_pts = []
                                    last_new_ts = now

                                prev = s

                            st = 0

                if i > 0:
                    del buf[:i]

                now = time.time()
                if now - t0 >= 1.0:
                    STATE.update_stats(port, bytes_sec, hdr_sec)
                    bytes_sec = 0
                    hdr_sec = 0
                    t0 = now

                if not chunk:
                    time.sleep(0.001)

        except Exception as e:
            STATE.update_stats(f"ERR: {e}", 0, 0)
            time.sleep(0.5)
        finally:
            try:
                if ser:
                    ser.close()
            except Exception:
                pass


# ---------------- rendering ----------------

def mm_to_px(x_mm, y_mm, img_w, img_h, field_w, field_h, margin=40):
    scale = min((img_w - 2 * margin) / field_w, (img_h - 2 * margin) / field_h)
    px = margin + x_mm * scale
    py = img_h - margin - y_mm * scale
    return px, py, scale


def render_frame():
    img_w, img_h = 900, 700
    img = Image.new("RGB", (img_w, img_h), (15, 15, 15))
    draw = ImageDraw.Draw(img)

    pts, pose, port, bytes_sec, hdr_sec = STATE.snapshot()

    # field
    x0, y0, scale = mm_to_px(0, 0, img_w, img_h, FIELD_W, FIELD_H)
    x1, y1, _ = mm_to_px(FIELD_W, FIELD_H, img_w, img_h, FIELD_W, FIELD_H)

    left = min(x0, x1)
    right = max(x0, x1)
    top = min(y0, y1)
    bottom = max(y0, y1)

    draw.rectangle([left, top, right, bottom], outline=(220, 220, 220), width=3)

    # 10 mm grid
    for x in np.arange(10, FIELD_W, 10):
        px, _, _ = mm_to_px(x, 0, img_w, img_h, FIELD_W, FIELD_H)
        draw.line([(px, top), (px, bottom)], fill=(45, 45, 45), width=1)

    for y in np.arange(10, FIELD_H, 10):
        _, py, _ = mm_to_px(0, y, img_w, img_h, FIELD_W, FIELD_H)
        draw.line([(left, py), (right, py)], fill=(45, 45, 45), width=1)

    # corner labels
    draw.text((left + 4, bottom + 4), "(0,0)", fill=(180, 180, 180))
    draw.text((right - 80, bottom + 4), f"({FIELD_W:.0f},0)", fill=(180, 180, 180))
    draw.text((left + 4, top - 18), f"(0,{FIELD_H:.0f})", fill=(180, 180, 180))

    if pose is not None:
        x = pose["x"]
        y = pose["y"]
        yaw = pose["yaw"]

        # clamp inside the field for drawing
        x_clamped = min(max(x, 0.0), FIELD_W)
        y_clamped = min(max(y, 0.0), FIELD_H)

        px, py, _ = mm_to_px(x_clamped, y_clamped, img_w, img_h, FIELD_W, FIELD_H)

        # robot
        robot_r_mm = 6.0
        robot_r_px = robot_r_mm * scale

        draw.ellipse(
            [px - robot_r_px, py - robot_r_px, px + robot_r_px, py + robot_r_px],
            outline=(255, 80, 80),
            fill=(120, 20, 20),
            width=3
        )

        # heading
        a = math.radians(yaw)
        hx = px + math.cos(a) * 14 * scale
        hy = py - math.sin(a) * 14 * scale
        draw.line([(px, py), (hx, hy)], fill=(255, 255, 0), width=3)

        # label
        txt1 = f"x={x:.1f} mm   y={y:.1f} mm   yaw={yaw:.1f} deg"
        txt2 = f"score={pose['score']:.2f}"
        draw.text((20, 20), txt1, fill=(255, 255, 255))
        draw.text((20, 42), txt2, fill=(180, 220, 180))
    else:
        draw.text((20, 20), "pose: no estimate", fill=(255, 120, 120))

    draw.text((20, img_h - 26), f"port={port}   bytes/s={bytes_sec}   hdr/s={hdr_sec}", fill=(180, 180, 180))

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    return buf.getvalue()


# ---------------- web ----------------

app = Flask(__name__)


@app.route("/")
def index():
    return """
    <!doctype html>
    <html>
      <head>
        <meta charset="utf-8">
        <title>Robot pose in rectangle</title>
        <style>
          body { background:#111; color:#eee; font-family:sans-serif; text-align:center; }
          img { margin-top:20px; border:1px solid #444; max-width:95vw; height:auto; }
        </style>
      </head>
      <body>
        <h2>Robot pose in 118 x 100 mm field</h2>
        <img src="/stream" alt="pose stream">
      </body>
    </html>
    """


@app.route("/stream")
def stream():
    def gen():
        while True:
            frame = render_frame()
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)

    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ---------------- main ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--vid", type=str)
    ap.add_argument("--pid", type=str)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=8000)
    args = ap.parse_args()

    get_port = (lambda: autodetect(args.vid, args.pid)) if args.auto else (lambda: args.port)

    stop_evt = threading.Event()
    t = threading.Thread(target=serial_worker, args=(get_port, args.baud, stop_evt), daemon=True)
    t.start()

    try:
        app.run(host=args.host, port=args.web_port, threaded=True)
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()