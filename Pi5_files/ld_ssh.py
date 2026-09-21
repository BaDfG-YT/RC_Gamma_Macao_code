#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LD19 web viewer over SSH

LD19 — 47-byte packets, header 0x54 0x2C
"""

import argparse
import io
import math
import threading
import time

import numpy as np
import lidar_cfg
import serial
from serial.tools import list_ports
from flask import Flask, Response
from PIL import Image, ImageDraw

# python ld_ssh.py --port /dev/ttyUSB0
# С автодетектом CP2102:
# python ld_ssh.py --auto


# =====================================================================
# Lidar protocols
# =====================================================================

class LD19Protocol:
    """LDRobot LD19 / STL-19P / D500 — 47-byte packets, header 0x54 0x2C."""
    name = "ld19"
    PKT = 47
    HDR = b"\x54\x2C"
    NPTS = 12

    # CRC-8 table и init — из config/config.defaults.json (lidar_cfg.py)
    CRC_TABLE = lidar_cfg.CRC_TABLE

    DEFAULT_BAUD = 230400

    @classmethod
    def verify(cls, mv, mode):
        if mode == "none":
            return True
        crc = lidar_cfg.CRC_INIT
        for b in mv[2:46]:
            crc = int(cls.CRC_TABLE[(crc ^ b) & 0xFF])
        crc_ok = (crc & 0xFF) == mv[46]
        if mode == "crc":
            return crc_ok
        add_ok = (sum(mv[:46]) & 0xFF) == mv[46]
        if mode == "add":
            return add_ok
        return crc_ok or add_ok

    @classmethod
    def parse(cls, mv):
        s = (int(mv[4]) | (int(mv[5]) << 8)) / 100.0
        e = (int(mv[42]) | (int(mv[43]) << 8)) / 100.0
        d = (360.0 - s + e) if (s > 270.0 and e < 90.0) else (e - s)

        t = np.linspace(0.0, 1.0, cls.NPTS, dtype=np.float32)
        ang = ((s + d * t) % 360.0).astype(np.float32)

        dist = np.empty(cls.NPTS, np.int32)
        off = 6
        for i in range(cls.NPTS):
            dist[i] = int(mv[off]) | (int(mv[off + 1]) << 8)
            off += 3

        m = dist > 0
        if not np.any(m):
            return None, None
        return ang[m], dist[m].astype(np.float32)

    @classmethod
    def packet_start_angle(cls, mv):
        return (int(mv[4]) | (int(mv[5]) << 8)) / 100.0


# =====================================================================
# Serial autodetect
# =====================================================================

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


# =====================================================================
# Shared state
# =====================================================================

class SharedState:
    def __init__(self, w=800, h=600, scale=0.05, ang_res=0.1):
        self.w = w
        self.h = h
        self.scale = scale
        self.ang_res = ang_res
        self.bins = int(round(360.0 / ang_res))

        self.dist_cur = np.full(self.bins, -1.0, dtype=np.float32)
        self.dist_last = np.full(self.bins, -1.0, dtype=np.float32)
        self.ang_bins_deg = (np.arange(self.bins, dtype=np.float32) + 0.5) * ang_res
        self.ang_bins_rad = np.deg2rad(self.ang_bins_deg)

        self.bytes_per_sec = 0
        self.hdr_per_sec = 0
        self.port = "N/A"
        self.proto_name = "?"
        self.last_swap_ts = 0.0
        self.lock = threading.Lock()

    def update_polar(self, ang_deg, dist_mm):
        idx = np.floor(ang_deg / self.ang_res + 0.5).astype(np.int32) % self.bins
        with self.lock:
            self.dist_cur[idx] = dist_mm

    def swap_scan(self):
        with self.lock:
            self.dist_last[:] = self.dist_cur
            self.dist_cur.fill(-1.0)
            self.last_swap_ts = time.time()

    def set_stats(self, port, bytes_sec, hdr_sec, proto_name):
        with self.lock:
            self.port = port
            self.bytes_per_sec = bytes_sec
            self.hdr_per_sec = hdr_sec
            self.proto_name = proto_name

    def snapshot(self):
        with self.lock:
            return (
                self.dist_last.copy(),
                self.ang_bins_rad.copy(),
                self.port,
                self.bytes_per_sec,
                self.hdr_per_sec,
                self.proto_name,
            )


# =====================================================================
# Serial worker — общий для любого протокола
# =====================================================================

def serial_worker(get_port, baud, stop_evt, check_mode, hyst, state, protocol,
                  block_size=4096, timeout=0.02):
    prev = None
    last_new_ts = 0.0

    HDR = protocol.HDR
    PKT = protocol.PKT
    proto_name = protocol.name

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

            while not stop_evt.is_set():
                chunk = ser.read(block_size)
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
                            # header[0] совпал, но header[1] нет —
                            # вернуться в st=0; если этот же байт — header[0],
                            # перепроверить
                            if b == HDR[0]:
                                pkt[0] = b
                                idx = 1
                                st = 1
                            else:
                                st = 0
                    else:
                        pkt[idx] = b
                        idx += 1
                        if idx == PKT:
                            mv = memoryview(pkt)
                            if protocol.verify(mv, check_mode):
                                ang, dist = protocol.parse(mv)
                                if ang is not None:
                                    state.update_polar(ang, dist)

                                start_a = protocol.packet_start_angle(mv)
                                now = time.time()
                                if (prev is not None
                                        and (prev - start_a) > 180.0
                                        and (now - last_new_ts) > 0.08):
                                    state.swap_scan()
                                    last_new_ts = now
                                # запасной wrap-детектор (для LD19 со скачком 360→0 в окне hyst)
                                elif (prev is not None
                                        and prev > (360.0 - hyst)
                                        and start_a < hyst
                                        and (now - last_new_ts) > 0.10):
                                    state.swap_scan()
                                    last_new_ts = now
                                prev = start_a
                            st = 0

                if i > 0:
                    del buf[:i]

                now = time.time()
                if now - t0 >= 1.0:
                    state.set_stats(port, bytes_sec, hdr_sec, proto_name)
                    bytes_sec = 0
                    hdr_sec = 0
                    t0 = now

                if not chunk:
                    time.sleep(0.001)

        except Exception as e:
            state.set_stats(f"ERR: {e}", 0, 0, proto_name)
            time.sleep(0.5)
        finally:
            try:
                if ser:
                    ser.close()
            except Exception:
                pass


# =====================================================================
# Rendering
# =====================================================================

def render_frame(state: SharedState, baud: int) -> bytes:
    dist_last, ang_bins_rad, port, bytes_sec, hdr_sec, proto_name = state.snapshot()
    img = Image.new("RGB", (state.w, state.h), (0, 0, 0))
    draw = ImageDraw.Draw(img)

    cx = state.w // 2
    cy = state.h // 2

    draw.line((0, cy, state.w, cy), fill=(60, 60, 60))
    draw.line((cx, 0, cx, state.h), fill=(60, 60, 60))

    m = dist_last >= 0.0
    if np.any(m):
        d = dist_last[m]
        a = ang_bins_rad[m]
        xs = d * np.cos(a)
        ys = d * np.sin(a)

        sx = (cx + xs * state.scale).astype(np.int32)
        sy = (cy + ys * state.scale).astype(np.int32)

        valid = (sx >= 0) & (sx < state.w) & (sy >= 0) & (sy < state.h)
        for x, y in zip(sx[valid], sy[valid]):
            draw.point((int(x), int(y)), fill=(230, 230, 230))

    txt = (f"proto:{proto_name} port:{port} baud:{baud} "
           f"bytes/s:{bytes_sec} hdr/s:{hdr_sec} bins:{state.bins}")
    draw.text((10, 10), txt, fill=(255, 255, 255))

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


# =====================================================================
# Web app
# =====================================================================

app = Flask(__name__)
STATE = None
BAUD = 0


@app.route("/")
def index():
    return """
    <!doctype html>
    <html>
      <head>
        <meta charset="utf-8">
        <title>Lidar over SSH</title>
        <style>
          body { background:#111; color:#eee; font-family: sans-serif; text-align:center; }
          img { border:1px solid #444; margin-top:20px; max-width:95vw; height:auto; }
        </style>
      </head>
      <body>
        <h2>Lidar visualization</h2>
        <img src="/stream" alt="lidar stream">
      </body>
    </html>
    """


@app.route("/stream")
def stream():
    def gen():
        while True:
            frame = render_frame(STATE, BAUD)
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.05)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


# =====================================================================
# Main
# =====================================================================

def main():
    global STATE, BAUD

    ap = argparse.ArgumentParser("Lidar web viewer over SSH")
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--vid", type=str)
    ap.add_argument("--pid", type=str)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=None,
                    help="по умолчанию: 230400")
    ap.add_argument("--w", type=int, default=800)
    ap.add_argument("--h", type=int, default=600)
    ap.add_argument("--scale", type=float, default=0.05)
    ap.add_argument("--check", choices=["auto", "add", "crc", "none"], default="none")
    ap.add_argument("--ang-res", type=float, default=0.1)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=8000)
    args = ap.parse_args()

    protocol = LD19Protocol
    BAUD = args.baud if args.baud is not None else protocol.DEFAULT_BAUD

    print(f"Lidar protocol: {protocol.name}  "
          f"(packet={protocol.PKT}b, header={protocol.HDR.hex()}, baud={BAUD})")

    get_port = (lambda: autodetect(args.vid, args.pid)) if args.auto else (lambda: args.port)

    STATE = SharedState(w=args.w, h=args.h, scale=args.scale, ang_res=args.ang_res)

    stop_evt = threading.Event()
    t = threading.Thread(
        target=serial_worker,
        args=(get_port, BAUD, stop_evt, args.check, 20.0, STATE, protocol),
        daemon=True
    )
    t.start()

    try:
        app.run(host=args.host, port=args.web_port, threaded=True)
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()