#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import io
import math
import threading
import time
from collections import deque

import numpy as np
import serial
from serial.tools import list_ports
from flask import Flask, Response
from PIL import Image, ImageDraw

from picamera2 import Picamera2
import cv2

# LD19 (как раньше — по умолчанию):
# python3 ld_cam.py --port /dev/ttyUSB0
# COIN-D4A:
# python3 ld_cam.py --lidar d4a --port /dev/ttyUSB0

# =====================================================================
# Lidar protocols
# =====================================================================

class LD19Protocol:
    """LDRobot LD19 / STL-19P / D500 — 47-byte packets, header 0x54 0x2C."""
    name = "ld19"
    PKT = 47
    HDR = b"\x54\x2C"
    NPTS = 12
    DEFAULT_BAUD = 230400

    CRC_TABLE = np.array([
        0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
        0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
        0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
        0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
        0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
        0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
        0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
        0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
        0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
        0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
        0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
        0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
        0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
        0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
        0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
        0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
    ], dtype=np.uint8)

    @classmethod
    def verify(cls, mv):
        crc = 0xD8
        for b in mv[2:46]:
            crc = int(cls.CRC_TABLE[(crc ^ b) & 0xFF])
        return (crc & 0xFF) == mv[46]

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


class CoinD4AProtocol:
    """Guoke / FDRobot COIN-D4A — 85-byte packets, header 0xAA 0x55."""
    name = "d4a"
    LSN = 25
    PKT = 10 + LSN * 3
    HDR = b"\xAA\x55"
    NPTS = LSN
    DEFAULT_BAUD = 230400

    @classmethod
    def verify(cls, mv):
        return mv[0] == 0xAA and mv[1] == 0x55

    @classmethod
    def parse(cls, mv):
        lsn = mv[3]
        if lsn == 0:
            return None, None

        fsa = mv[4] | (mv[5] << 8)
        lsa = mv[6] | (mv[7] << 8)

        fsa_deg = (fsa >> 1) / 64.0
        lsa_deg = (lsa >> 1) / 64.0

        diff = lsa_deg - fsa_deg
        if diff < 0:
            diff += 360.0

        ang = np.empty(lsn, dtype=np.float32)
        dist = np.empty(lsn, dtype=np.float32)

        for k in range(lsn):
            base = 10 + k * 3
            b1 = mv[base + 1]
            b2 = mv[base + 2]
            dist_mm = ((b2 << 8) | b1) / 4.0

            a = (fsa_deg + diff * k / (lsn - 1)) % 360.0 if lsn > 1 else fsa_deg
            ang[k] = a
            dist[k] = dist_mm

        m = (dist > 50.0) & (dist < 12000.0)
        if not np.any(m):
            return None, None
        return ang[m], dist[m]

    @classmethod
    def packet_start_angle(cls, mv):
        fsa = mv[4] | (mv[5] << 8)
        return (fsa >> 1) / 64.0


PROTOCOLS = {
    "ld19": LD19Protocol,
    "d4a":  CoinD4AProtocol,
}

# =====================================================================
# Конфигурация поля и т.д. — без изменений
# =====================================================================

FIELD_W = 1822.0
FIELD_H = 2432.0

# FIELD_W = 1060.0
# FIELD_H = 1330.0

LIDAR_OFFSET_X = 0.0
LIDAR_OFFSET_Y = 0.0

TARGET_X = FIELD_W / 2
TARGET_Y = FIELD_H / 2

PICO_PORT = "/dev/ttyACM0"
PICO_BAUD = 115200
ENABLE_LIDAR_VISUALIZATION = True

ENABLE_CAMERA_SENDING = True
ENABLE_CAMERA_VISUALIZATION = False
CAMERA_WEB_PORT = 5000

CAM_FRAME_LOCK = threading.Lock()
CAM_FRAME_JPEG = None

FRAME_W = 640
FRAME_H = 480

CENTER_X = FRAME_W / 2 - 18
CENTER_Y = FRAME_H / 2 + 30

BALL_LOWER = np.array([2, 85, 90])
BALL_UPPER = np.array([22, 180, 255])
BALL_MIN_AREA = 50

YELLOW_LOWER = np.array([20, 120, 195])
YELLOW_UPPER = np.array([32, 255, 255])

YELLOW_MIN_AREA = 200

BLUE_LOWER = np.array([90, 73, 80])
BLUE_UPPER = np.array([134, 255, 255])
BLUE_MIN_AREA = 200

NO_TARGET_ANGLE = 400.0

CAM_SEND_PERIOD = 0.05

UPDATE_PERIOD = 0.05
CLEAR_ON_WRAP = False
MAX_POINTS_FOR_POSE = 2000
MAX_POINTS_TO_DRAW = 500
# === Pose sanity check ===
OWN_GOAL_COLOR = "blue"   # будет переписан из args в main()
POSE_SANITY_PERIOD_S = 1.0
POSE_SANITY_FLIP_THRESHOLD_DEG = 90.0
POSE_SANITY_MIN_DIST_MM = 300.0   # ближе этого к воротам не проверяем
POSE_SANITY_COOLDOWN_S = 2.0      # после флипа подождать прежде чем снова проверять

CAM_STATE_LOCK = threading.Lock()
CAM_STATE = {
    "yellow_angle": NO_TARGET_ANGLE,
    "yellow_visible": False,
    "blue_angle":   NO_TARGET_ANGLE,
    "blue_visible": False,
    "ts": 0.0,
}

WALL_BIN_MM = 80.0
WALL_BAND_MM = 120.0
MIN_WALL_PEAK = 15
SWAP_PENALTY_GAIN = 30

KEEP_LAST_SCANS = 1

# =====================================================================
# Вспомогательные функции — без изменений
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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap180(a):
    while a < 0.0:
        a += 180.0
    while a >= 180.0:
        a -= 180.0
    return a


def wrap360(a):
    while a < 0.0:
        a += 360.0
    while a >= 360.0:
        a -= 360.0
    return a


ROBOT_YAW_OFFSET_DEG = -90.0
ROBOT_YAW_SIGN = 1.0

def field_yaw_to_robot_yaw(field_yaw_deg):
    return wrap360(ROBOT_YAW_OFFSET_DEG + ROBOT_YAW_SIGN * field_yaw_deg)

def closest_unwrapped_angle(angle_deg, prev_unwrapped):
    candidates = []
    for k in range(-4, 5):
        candidates.append(float(angle_deg) + 360.0 * k)
    return min(candidates, key=lambda a: abs(a - prev_unwrapped))

def angle_diff_360(a, b):
    d = a - b
    while d >= 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d

def rot2d(points, deg):
    a = np.deg2rad(deg)
    c = np.cos(a)
    s = np.sin(a)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    return points @ R.T

def find_wall_positions(arr, bin_size=20.0, min_sep=200.0):
    lo = float(np.min(arr))
    hi = float(np.max(arr))
    if hi - lo < bin_size * 4:
        return None

    bins = np.arange(lo, hi + bin_size, bin_size)
    if bins.size < 3:
        return None

    hist, edges = np.histogram(arr, bins=bins)
    if hist.size < 2:
        return None

    centers = 0.5 * (edges[:-1] + edges[1:])

    i1 = int(np.argmax(hist))
    c1 = centers[i1]
    h1 = int(hist[i1])

    mask = np.abs(centers - c1) >= min_sep
    if not np.any(mask):
        return None

    hist2 = hist.copy()
    hist2[~mask] = -1
    i2 = int(np.argmax(hist2))
    if hist2[i2] < 0:
        return None

    c2 = centers[i2]
    h2 = int(hist[i2])

    if c1 < c2:
        return float(c1), float(c2), h1, h2
    return float(c2), float(c1), h2, h1


def wall_fit_score(u, v, umin, umax, vmin, vmax, field_w, field_h):
    span_u = umax - umin
    span_v = vmax - vmin

    span_err = abs(span_u - field_w) + abs(span_v - field_h)

    du = np.minimum(np.abs(u - umin), np.abs(u - umax))
    dv = np.minimum(np.abs(v - vmin), np.abs(v - vmax))
    wall_dist = np.minimum(du, dv)

    fit_err = float(np.median(np.clip(wall_dist, 0.0, 80.0)))

    return span_err, fit_err


# =====================================================================
# Shared state, navigation, estimate_pose — без изменений
# =====================================================================

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.points_xy = np.empty((0, 2), dtype=np.float32)
        self.pose = None
        self.port = "N/A"
        self.proto_name = "?"
        self.bytes_per_sec = 0
        self.hdr_per_sec = 0

    def update_points(self, pts):
        with self.lock:
            self.points_xy = pts

    def update_pose(self, pose):
        with self.lock:
            self.pose = pose

    def update_stats(self, port, bytes_sec, hdr_sec, proto_name):
        with self.lock:
            self.port = port
            self.bytes_per_sec = bytes_sec
            self.hdr_per_sec = hdr_sec
            self.proto_name = proto_name

    def snapshot(self):
        with self.lock:
            pts = self.points_xy.copy()
            pose = None if self.pose is None else dict(self.pose)
            return pts, pose, self.port, self.bytes_per_sec, self.hdr_per_sec, self.proto_name


STATE = SharedState()
LAST_POSE = None

NAV_STATE = {
    "target_x": TARGET_X,
    "target_y": TARGET_Y,
    "cmd_angle": 0.0,
    "arrived": 1,
    "distance": None,
    "dx": None,
    "dy": None,
}

PICO_TX_LOCK = threading.Lock()
PICO_TX_LINES = []

def pico_queue_line(line):
    global PICO_TX_LINES
    if not line.endswith("\n"):
        line += "\n"
    with PICO_TX_LOCK:
        PICO_TX_LINES.append(line)
        if len(PICO_TX_LINES) > 20:
            PICO_TX_LINES = PICO_TX_LINES[-20:]


def estimate_main_angles(points_xy):
    pts = points_xy - np.mean(points_xy, axis=0)
    cov = np.cov(pts.T)
    vals, vecs = np.linalg.eig(cov)

    main_vec = vecs[:, np.argmax(vals)]
    ang = math.degrees(math.atan2(main_vec[1], main_vec[0]))
    ang = wrap180(ang)

    a1 = ang
    a2 = wrap180(ang + 90.0)

    angles = []
    for base in [a1, a2]:
        for d in np.arange(-15.0, 15.1, 1.0):
            angles.append(wrap180(base + d))

    return np.array(sorted(set(round(a, 2) for a in angles)), dtype=np.float32)


def estimate_pose(points_xy, field_w=FIELD_W, field_h=FIELD_H, prev_pose=None):
    if points_xy.shape[0] < 30:
        return None

    best = None

    if prev_pose is not None:
        yaw0 = prev_pose["yaw"] % 180.0
        angles_prev = [wrap180(yaw0 + d) for d in np.arange(-12.0, 12.1, 1.0)]
        angles_pca = estimate_main_angles(points_xy)

        coarse_angles = np.array(
            sorted(set([round(a, 2) for a in angles_prev] + [round(float(a), 2) for a in angles_pca])),
            dtype=np.float32
        )
    else:
        coarse_angles = estimate_main_angles(points_xy)

    for theta in coarse_angles:
        t = wrap180(float(theta))
        rp = rot2d(points_xy, -t)
        u = rp[:, 0]
        v = rp[:, 1]

        uwalls = find_wall_positions(u, bin_size=WALL_BIN_MM, min_sep=min(field_w, field_h) * 0.4)
        vwalls = find_wall_positions(v, bin_size=WALL_BIN_MM, min_sep=min(field_w, field_h) * 0.4)
        if uwalls is None or vwalls is None:
            continue

        umin, umax, hu1, hu2 = uwalls
        vmin, vmax, hv1, hv2 = vwalls

        span_u = umax - umin
        span_v = vmax - vmin

        normal_err = abs(span_u - field_w) + abs(span_v - field_h)
        swapped_err = abs(span_u - field_h) + abs(span_v - field_w)

        _, fit_err = wall_fit_score(u, v, umin, umax, vmin, vmax, field_w, field_h)

        strength_penalty = 0.0
        if min(hu1, hu2) < MIN_WALL_PEAK:
            strength_penalty += 300.0
        if min(hv1, hv2) < MIN_WALL_PEAK:
            strength_penalty += 300.0

        du = np.minimum(np.abs(u - umin), np.abs(u - umax))
        dv = np.minimum(np.abs(v - vmin), np.abs(v - vmax))
        wall_dist = np.minimum(du, dv)
        support = float(np.mean(wall_dist <= WALL_BAND_MM))

        swap_penalty = 0.0
        if swapped_err < normal_err:
            swap_penalty += (normal_err - swapped_err + 1.0) * SWAP_PENALTY_GAIN

        score = normal_err * 5.0 + fit_err + strength_penalty + swap_penalty - support * 120.0

        if best is None or score < best["score"]:
            best = {
                "theta": t, "umin": float(umin), "umax": float(umax),
                "vmin": float(vmin), "vmax": float(vmax),
                "span_u": float(span_u), "span_v": float(span_v),
                "normal_err": float(normal_err), "swapped_err": float(swapped_err),
                "fit_err": float(fit_err), "support": float(support),
                "score": float(score),
            }

    if best is None:
        return None

    theta0 = best["theta"]
    fine_angles = np.arange(theta0 - 1.0, theta0 + 1.01, 0.1, dtype=np.float32)

    for theta in fine_angles:
        t = wrap180(float(theta))
        rp = rot2d(points_xy, -t)
        u = rp[:, 0]
        v = rp[:, 1]

        uwalls = find_wall_positions(u, bin_size=WALL_BIN_MM, min_sep=min(field_w, field_h) * 0.4)
        vwalls = find_wall_positions(v, bin_size=WALL_BIN_MM, min_sep=min(field_w, field_h) * 0.4)
        if uwalls is None or vwalls is None:
            continue

        umin, umax, hu1, hu2 = uwalls
        vmin, vmax, hv1, hv2 = vwalls

        span_u = umax - umin
        span_v = vmax - vmin

        normal_err = abs(span_u - field_w) + abs(span_v - field_h)
        swapped_err = abs(span_u - field_h) + abs(span_v - field_w)

        _, fit_err = wall_fit_score(u, v, umin, umax, vmin, vmax, field_w, field_h)

        strength_penalty = 0.0
        if min(hu1, hu2) < MIN_WALL_PEAK:
            strength_penalty += 300.0
        if min(hv1, hv2) < MIN_WALL_PEAK:
            strength_penalty += 300.0

        du = np.minimum(np.abs(u - umin), np.abs(u - umax))
        dv = np.minimum(np.abs(v - vmin), np.abs(v - vmax))
        wall_dist = np.minimum(du, dv)
        support = float(np.mean(wall_dist <= WALL_BAND_MM))

        swap_penalty = 0.0
        if swapped_err < normal_err:
            swap_penalty += (normal_err - swapped_err + 1.0) * SWAP_PENALTY_GAIN

        score = normal_err * 5.0 + fit_err + strength_penalty + swap_penalty - support * 120.0

        if score < best["score"]:
            best = {
                "theta": t, "umin": float(umin), "umax": float(umax),
                "vmin": float(vmin), "vmax": float(vmax),
                "span_u": float(span_u), "span_v": float(span_v),
                "normal_err": float(normal_err), "swapped_err": float(swapped_err),
                "fit_err": float(fit_err), "support": float(support),
                "score": float(score),
            }

    base_x = -best["umin"]
    base_y = -best["vmin"]

    fit_yaw = best["theta"]

    variants = [
        {"x": base_x,             "y": base_y,             "yaw_base": fit_yaw},
        {"x": FIELD_W - base_x,   "y": FIELD_H - base_y,   "yaw_base": fit_yaw + 180.0},
    ]

    if prev_pose is not None:
        prev_x = prev_pose["x"]
        prev_y = prev_pose["y"]
        prev_yaw_unwrapped = prev_pose.get("yaw_unwrapped", prev_pose["yaw"])

        best_variant = None
        best_variant_score = None

        for vv in variants:
            yu = closest_unwrapped_angle(vv["yaw_base"], prev_yaw_unwrapped)
            pos_err = math.hypot(vv["x"] - prev_x, vv["y"] - prev_y)
            yaw_err = abs(yu - prev_yaw_unwrapped)
            variant_score = pos_err + yaw_err * 2.0

            if best_variant_score is None or variant_score < best_variant_score:
                best_variant_score = variant_score
                best_variant = vv
                best_yaw_unwrapped = yu

        x_robot = best_variant["x"]
        y_robot = best_variant["y"]
        yaw_unwrapped = best_yaw_unwrapped
    else:
        x_robot = base_x
        y_robot = base_y
        yaw_unwrapped = fit_yaw

    yaw = wrap360(yaw_unwrapped)
    robot_yaw = field_yaw_to_robot_yaw(yaw_unwrapped)

    if LIDAR_OFFSET_X != 0.0 or LIDAR_OFFSET_Y != 0.0:
        a = math.radians(yaw)
        dx = LIDAR_OFFSET_X * math.cos(a) - LIDAR_OFFSET_Y * math.sin(a)
        dy = LIDAR_OFFSET_X * math.sin(a) + LIDAR_OFFSET_Y * math.cos(a)
        x_robot -= dx
        y_robot -= dy

    return {
        "x": float(x_robot), "y": float(y_robot),
        "yaw": float(yaw), "yaw_unwrapped": float(yaw_unwrapped),
        "robot_yaw": float(robot_yaw), "fit_yaw": float(fit_yaw),
        "score": float(best["score"]),
        "umin": best["umin"], "umax": best["umax"],
        "vmin": best["vmin"], "vmax": best["vmax"],
        "span_u": best["span_u"], "span_v": best["span_v"],
        "normal_err": best["normal_err"], "swapped_err": best["swapped_err"],
        "fit_err": best["fit_err"], "support": best["support"],
    }


def smooth_pose(prev_pose, new_pose, alpha_xy=0.4, alpha_yaw=0.2,
                max_step_xy=60.0, max_step_yaw=180.0):
    if new_pose is None:
        return prev_pose
    if prev_pose is None:
        return dict(new_pose)

    dx = new_pose["x"] - prev_pose["x"]
    dy = new_pose["y"] - prev_pose["y"]
    dx = clamp(dx, -max_step_xy, max_step_xy)
    dy = clamp(dy, -max_step_xy, max_step_xy)

    x = prev_pose["x"] + alpha_xy * dx
    y = prev_pose["y"] + alpha_xy * dy

    prev_yaw_unwrapped = prev_pose.get("yaw_unwrapped", prev_pose["yaw"])
    new_yaw_unwrapped = new_pose.get("yaw_unwrapped", new_pose["yaw"])

    dyaw = new_yaw_unwrapped - prev_yaw_unwrapped
    dyaw = clamp(dyaw, -max_step_yaw, max_step_yaw)

    yaw_unwrapped = prev_yaw_unwrapped + alpha_yaw * dyaw
    yaw = wrap360(yaw_unwrapped)

    out = dict(new_pose)
    out["x"] = x
    out["y"] = y
    out["yaw"] = yaw
    out["yaw_unwrapped"] = yaw_unwrapped
    out["robot_yaw"] = field_yaw_to_robot_yaw(yaw_unwrapped)
    return out


def compute_drive_command(pose, tx, ty, stop_dx=30.0, stop_dy=30.0):
    if pose is None:
        return 0.0, 1, None, None, None

    dx = tx - pose["x"]
    dy = ty - pose["y"]
    dist = math.hypot(dx, dy)

    arrived = (abs(dx) <= stop_dx) and (abs(dy) <= stop_dy)
    if arrived:
        return 0.0, 1, dist, dx, dy

    world_angle_math = math.degrees(math.atan2(dy, dx))
    world_angle_robot = wrap360(90.0 - world_angle_math)

    robot_yaw = pose.get("robot_yaw", field_yaw_to_robot_yaw(pose["yaw"]))
    cmd_angle = angle_diff_360(world_angle_robot, robot_yaw)

    return cmd_angle, 0, dist, dx, dy


# =====================================================================
# Serial worker — теперь параметризован протоколом
# =====================================================================

def serial_worker(get_port, baud, stop_evt, protocol, timeout=0.02):
    global LAST_POSE

    HDR = protocol.HDR
    PKT = protocol.PKT
    proto_name = protocol.name

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

            scan_buffer = []
            current_scan_pts = []

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
                            if protocol.verify(mv):
                                ang, dist = protocol.parse(mv)
                                if ang is not None:
                                    a = np.deg2rad(ang)
                                    xs = -dist * np.cos(a)
                                    ys = dist * np.sin(a)
                                    pts = np.column_stack((xs, ys)).astype(np.float32)
                                    current_scan_pts.append(pts)

                                s = protocol.packet_start_angle(mv)
                                now = time.time()

                                if (scan_buffer or current_scan_pts) and (now - last_new_ts) > UPDATE_PERIOD:
                                    parts = []
                                    if scan_buffer:
                                        parts.extend(scan_buffer)
                                    if current_scan_pts:
                                        parts.append(np.vstack(current_scan_pts))

                                    full_pts = np.vstack(parts)

                                    rr = np.linalg.norm(full_pts, axis=1)
                                    m = (rr > 90.0) & (rr < 2800.0)
                                    full_pts = full_pts[m]

                                    if full_pts.shape[0] > MAX_POINTS_FOR_POSE:
                                        idx_keep = np.linspace(
                                            0, full_pts.shape[0] - 1, MAX_POINTS_FOR_POSE
                                        ).astype(np.int32)
                                        full_pts = full_pts[idx_keep]

                                    STATE.update_points(full_pts)

                                    pose_raw = estimate_pose(full_pts, prev_pose=LAST_POSE)
                                    pose_smooth = smooth_pose(LAST_POSE, pose_raw)
                                    LAST_POSE = pose_smooth
                                    STATE.update_pose(pose_smooth)

                                    if pose_smooth is not None:
                                        cmd_angle, arrived, dist_to_target, dx, dy = compute_drive_command(
                                            pose_smooth,
                                            NAV_STATE["target_x"],
                                            NAV_STATE["target_y"],
                                            stop_dx=100.0,
                                            stop_dy=100.0
                                        )
                                        NAV_STATE["cmd_angle"] = cmd_angle
                                        NAV_STATE["arrived"] = arrived
                                        NAV_STATE["distance"] = dist_to_target
                                        NAV_STATE["dx"] = dx
                                        NAV_STATE["dy"] = dy

                                    last_new_ts = now

                                # детектор начала оборота — универсальный
                                if (prev is not None
                                        and (prev - s) > 180.0):
                                    if current_scan_pts:
                                        scan_buffer.append(np.vstack(current_scan_pts))
                                        current_scan_pts = []
                                        if len(scan_buffer) > KEEP_LAST_SCANS:
                                            scan_buffer.pop(0)

                                prev = s
                            st = 0

                if i > 0:
                    del buf[:i]

                now = time.time()
                if now - t0 >= 1.0:
                    STATE.update_stats(port, bytes_sec, hdr_sec, proto_name)
                    bytes_sec = 0
                    hdr_sec = 0
                    t0 = now

                if not chunk:
                    time.sleep(0.001)

        except Exception as e:
            STATE.update_stats(f"ERR: {e}", 0, 0, proto_name)
            time.sleep(0.5)
        finally:
            try:
                if ser:
                    ser.close()
            except Exception:
                pass


# =====================================================================
# Дальше — pico_sender_worker, render_frame, web app, camera_worker —
# идентичны исходному файлу, без изменений.
# Они вставлены ниже целиком.
# =====================================================================

def pico_sender_worker(stop_evt):
    ser = None
    last_cmd_send = 0.0

    while not stop_evt.is_set():
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(PICO_PORT, PICO_BAUD, timeout=0.1)
                time.sleep(2.0)

            now = time.time()

            if now - last_cmd_send >= 0.05:
                angle = NAV_STATE["cmd_angle"]
                arrived = NAV_STATE["arrived"]

                _, pose, _, _, _, _ = STATE.snapshot()

                if pose is not None:
                    robot_yaw = pose.get("robot_yaw", 0.0)
                    x = pose["x"]
                    y = pose["y"]
                    pose_valid = 1
                else:
                    robot_yaw = 0.0
                    x = 0.0
                    y = 0.0
                    pose_valid = 0

                dist = NAV_STATE["distance"] if NAV_STATE["distance"] is not None else 0.0
                tx = NAV_STATE["target_x"]
                ty = NAV_STATE["target_y"]

                cmd_line = f"CMD,{angle:.2f},{arrived},{robot_yaw:.2f}\n"
                pos_line = f"POS,{pose_valid},{x:.1f},{y:.1f},{dist:.1f},{tx:.1f},{ty:.1f}\n"

                ser.write(cmd_line.encode("utf-8"))
                ser.write(pos_line.encode("utf-8"))

                last_cmd_send = now

            lines = []
            with PICO_TX_LOCK:
                if PICO_TX_LINES:
                    lines = PICO_TX_LINES[:]
                    PICO_TX_LINES.clear()

            for line in lines:
                ser.write(line.encode("utf-8"))

            time.sleep(0.005)

        except Exception:
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(0.5)


VIEW_EXTRA_MM = 750.0

def mm_to_px(x_mm, y_mm, img_w, img_h, field_w, field_h, margin=40):
    view_w = field_w + 2 * VIEW_EXTRA_MM
    view_h = field_h + 2 * VIEW_EXTRA_MM

    scale = min((img_w - 2 * margin) / view_w, (img_h - 2 * margin) / view_h)

    used_w = view_w * scale
    used_h = view_h * scale

    offset_x = (img_w - used_w) / 2
    offset_y = (img_h - used_h) / 2

    px = offset_x + (x_mm + VIEW_EXTRA_MM) * scale
    py = offset_y + used_h - (y_mm + VIEW_EXTRA_MM) * scale

    return px, py, scale


def debug_uv_to_field(u, v, pose):
    if pose is None:
        return None
    x_field = u - pose["umin"]
    y_field = v - pose["vmin"]
    return x_field, y_field


def transform_local_points_to_field(pts_local, pose):
    if pts_local.shape[0] == 0 or pose is None:
        return np.empty((0, 2), dtype=np.float32)

    a = math.radians(pose["yaw"])
    c = math.cos(a)
    s = math.sin(a)

    x = pose["x"] + pts_local[:, 0] * c + pts_local[:, 1] * s
    y = pose["y"] - pts_local[:, 0] * s + pts_local[:, 1] * c

    return np.column_stack((x, y)).astype(np.float32)


def render_frame():
    img_w, img_h = 1200, 800
    img = Image.new("RGB", (img_w, img_h), (15, 15, 15))
    draw = ImageDraw.Draw(img)

    pts_local, pose, port, bytes_sec, hdr_sec, proto_name = STATE.snapshot()

    x0, y0, scale = mm_to_px(0, 0, img_w, img_h, FIELD_W, FIELD_H)
    x1, y1, _ = mm_to_px(FIELD_W, FIELD_H, img_w, img_h, FIELD_W, FIELD_H)

    left = min(x0, x1)
    right = max(x0, x1)
    top = min(y0, y1)
    bottom = max(y0, y1)

    draw.rectangle([left, top, right, bottom], outline=(220, 220, 220), width=3)

    if pose is not None and all(k in pose for k in ["umin", "umax", "vmin", "vmax"]):
        debug_corners_uv = [
            (pose["umin"], pose["vmin"]),
            (pose["umax"], pose["vmin"]),
            (pose["umax"], pose["vmax"]),
            (pose["umin"], pose["vmax"]),
        ]
        debug_corners_px = []
        for u, v in debug_corners_uv:
            x_f, y_f = debug_uv_to_field(u, v, pose)
            px, py, _ = mm_to_px(x_f, y_f, img_w, img_h, FIELD_W, FIELD_H)
            debug_corners_px.append((px, py))
        draw.line(debug_corners_px + [debug_corners_px[0]], fill=(255, 0, 0), width=2)

    for x in np.arange(100, FIELD_W, 100):
        px, _, _ = mm_to_px(x, 0, img_w, img_h, FIELD_W, FIELD_H)
        draw.line([(px, top), (px, bottom)], fill=(45, 45, 45), width=1)

    for y in np.arange(100, FIELD_H, 100):
        _, py, _ = mm_to_px(0, y, img_w, img_h, FIELD_W, FIELD_H)
        draw.line([(left, py), (right, py)], fill=(45, 45, 45), width=1)

    draw.text((left + 4, bottom + 4), "(0,0)", fill=(180, 180, 180))
    draw.text((right - 110, bottom + 4), f"({FIELD_W:.0f},0)", fill=(180, 180, 180))
    draw.text((left + 4, top - 18), f"(0,{FIELD_H:.0f})", fill=(180, 180, 180))

    tx = NAV_STATE["target_x"]
    ty = NAV_STATE["target_y"]
    tpx, tpy, _ = mm_to_px(tx, ty, img_w, img_h, FIELD_W, FIELD_H)

    draw.line([(tpx - 8, tpy), (tpx + 8, tpy)], fill=(0, 255, 255), width=2)
    draw.line([(tpx, tpy - 8), (tpx, tpy + 8)], fill=(0, 255, 255), width=2)
    draw.text((tpx + 8, tpy - 18), "TARGET", fill=(0, 255, 255))

    if pts_local.shape[0] > 0 and pose is not None and all(k in pose for k in ["umin", "umax", "vmin", "vmax"]):
        pts_draw = pts_local
        if pts_draw.shape[0] > MAX_POINTS_TO_DRAW:
            idx_keep = np.linspace(0, pts_draw.shape[0] - 1, MAX_POINTS_TO_DRAW).astype(np.int32)
            pts_draw = pts_draw[idx_keep]

        pts_field = transform_local_points_to_field(pts_draw, pose)

        rp_dbg = rot2d(pts_draw, -pose.get("fit_yaw", pose["yaw"]))
        u_dbg = rp_dbg[:, 0]
        v_dbg = rp_dbg[:, 1]

        du_dbg = np.minimum(np.abs(u_dbg - pose["umin"]), np.abs(u_dbg - pose["umax"]))
        dv_dbg = np.minimum(np.abs(v_dbg - pose["vmin"]), np.abs(v_dbg - pose["vmax"]))
        wall_dist_dbg = np.minimum(du_dbg, dv_dbg)

        is_wall_dbg = wall_dist_dbg <= WALL_BAND_MM

        for i, (x_mm, y_mm) in enumerate(pts_field):
            qx, qy, _ = mm_to_px(x_mm, y_mm, img_w, img_h, FIELD_W, FIELD_H)

            if 0 <= qx < img_w and 0 <= qy < img_h:
                if is_wall_dbg[i]:
                    draw.circle((int(qx), int(qy)), fill=(0, 255, 0), radius=2)
                else:
                    draw.circle((int(qx), int(qy)), fill=(255, 100, 0), radius=1.5)

    if pose is not None:
        x = pose["x"]
        y = pose["y"]
        yaw = pose["yaw"]

        px, py, _ = mm_to_px(x, y, img_w, img_h, FIELD_W, FIELD_H)

        robot_r_mm = 40.0
        robot_r_px = robot_r_mm * scale

        draw.ellipse(
            [px - robot_r_px, py - robot_r_px, px + robot_r_px, py + robot_r_px],
            outline=(255, 80, 80), fill=(120, 20, 20), width=3
        )

        robot_yaw = pose.get("robot_yaw", field_yaw_to_robot_yaw(yaw))
        a = math.radians(robot_yaw)

        hx = px + math.sin(a) * 120 * scale
        hy = py - math.cos(a) * 120 * scale
        draw.line([(px, py), (hx, hy)], fill=(255, 255, 0), width=3)

        draw.line([(px, py), (tpx, tpy)], fill=(0, 180, 255), width=2)

        dx_txt = "N/A" if NAV_STATE["dx"] is None else f"{NAV_STATE['dx']:.1f}"
        dy_txt = "N/A" if NAV_STATE["dy"] is None else f"{NAV_STATE['dy']:.1f}"
        dist_txt = "N/A" if NAV_STATE["distance"] is None else f"{NAV_STATE['distance']:.1f}"

        robot_yaw = pose.get("robot_yaw", field_yaw_to_robot_yaw(yaw))
        txt1 = f"x={x:.1f} mm   y={y:.1f} mm   yaw={robot_yaw:.1f} deg   field_yaw={yaw:.1f}"
        txt2 = f"score={pose['score']:.2f}"
        txt3 = f"cmd_angle={NAV_STATE['cmd_angle']:.1f} deg   arrived={NAV_STATE['arrived']}   dist={dist_txt} mm"
        txt4 = f"dx={dx_txt} mm   dy={dy_txt} mm   points={pts_local.shape[0]}"
        txt5 = f"span_u={pose.get('span_u', 0):.1f} span_v={pose.get('span_v', 0):.1f}"
        txt6 = f"normal_err={pose.get('normal_err', 0):.1f} swapped_err={pose.get('swapped_err', 0):.1f}"
        txt7 = f"fit_err={pose.get('fit_err', 0):.1f} support={pose.get('support', 0):.2f}"

        draw.text((20, 20), txt1, fill=(255, 255, 255))
        draw.text((20, 42), txt2, fill=(180, 220, 180))
        draw.text((20, 64), txt3, fill=(120, 220, 255))
        draw.text((20, 86), txt4, fill=(120, 220, 255))
        draw.text((20, 108), txt5, fill=(255, 180, 120))
        draw.text((20, 130), txt6, fill=(255, 180, 120))
        draw.text((20, 152), txt7, fill=(255, 180, 120))
    else:
        draw.text((20, 20), "pose: no estimate", fill=(255, 120, 120))
        draw.text((20, 42), f"points={pts_local.shape[0]}", fill=(180, 180, 180))

    draw.text(
        (20, img_h - 26),
        f"proto={proto_name}   port={port}   bytes/s={bytes_sec}   hdr/s={hdr_sec}   pico={PICO_PORT}",
        fill=(180, 180, 180)
    )

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    return buf.getvalue()


app = Flask(__name__)


@app.route("/")
def index():
    return f"""
    <!doctype html>
    <html>
      <head>
        <meta charset="utf-8">
        <title>Static field + robot + cloud</title>
        <style>
          body {{ background:#111; color:#eee; font-family:sans-serif; text-align:center; }}
          img {{ margin-top:20px; border:1px solid #444; max-width:95vw; height:auto; }}
        </style>
      </head>
      <body>
        <h2>Static field + dynamic robot + noisy cloud</h2>
        <p>Field: {FIELD_W:.0f} x {FIELD_H:.0f} mm</p>
        <p>Target: ({TARGET_X:.0f}, {TARGET_Y:.0f}) mm</p>
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
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


# === Камера — без изменений ===

def compute_angle_360(cx, cy, center_x, center_y):
    dx = cx - center_x
    dy = cy - center_y
    angle = -(math.degrees(math.atan2(dx, -dy)) - 90)
    if angle > 180:
        angle -= 360
    if angle <= -180:
        angle += 360
    return angle


def preprocess_mask(hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    return mask


def find_object_angle_area(hsv, lower, upper, min_area):
    mask = preprocess_mask(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area > min_area:
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w / 2.0
            cy = y + h / 2.0
            angle = compute_angle_360(cx, cy, CENTER_X, CENTER_Y)
            return angle, int(area)

    return NO_TARGET_ANGLE, 0


def find_color_cloud_angle_area(hsv, lower, upper, min_area):
    mask = preprocess_mask(hsv, lower, upper)

    area_to_send = int(cv2.countNonZero(mask))
    if area_to_send < min_area:
        return NO_TARGET_ANGLE, 0

    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] == 0:
        return NO_TARGET_ANGLE, 0

    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    angle = compute_angle_360(cx, cy, CENTER_X, CENTER_Y)
    return angle, area_to_send


def camera_worker(stop_evt):
    if not ENABLE_CAMERA_SENDING:
        return

    picam2 = None
    try:
        picam2 = Picamera2()
        picam2.configure(picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)}))
        picam2.start()
        time.sleep(2.0)

        while not stop_evt.is_set():
            t0 = time.time()

            frame = picam2.capture_array()
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            if ENABLE_CAMERA_VISUALIZATION:
                vis = bgr.copy()

                ball_mask = preprocess_mask(hsv, BALL_LOWER, BALL_UPPER)
                yellow_mask = preprocess_mask(hsv, YELLOW_LOWER, YELLOW_UPPER)
                blue_mask = preprocess_mask(hsv, BLUE_LOWER, BLUE_UPPER)

                def draw_all_contours(mask, min_area, dim_color, bright_color, label, mode):
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    valid = []
                    for c in contours:
                        area = cv2.contourArea(c)
                        if area >= min_area:
                            valid.append((c, area))

                    for c, area in valid:
                        x, y, w, h = cv2.boundingRect(c)
                        cv2.rectangle(vis, (x, y), (x + w, y + h), dim_color, 1)

                    if not valid:
                        return

                    if mode == "largest_contour":
                        best_c, best_area = max(valid, key=lambda item: item[1])
                        x, y, w, h = cv2.boundingRect(best_c)
                        cx = int(x + w / 2)
                        cy = int(y + h / 2)
                        ang = compute_angle_360(cx, cy, CENTER_X, CENTER_Y)
                        cv2.rectangle(vis, (x, y), (x + w, y + h), bright_color, 3)
                        cv2.circle(vis, (cx, cy), 6, bright_color, -1)
                        cv2.putText(vis, f"{label} USED ang={ang:.1f} area={int(best_area)}",
                                    (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bright_color, 2)

                    elif mode == "color_cloud":
                        area_to_send = int(cv2.countNonZero(mask))
                        if area_to_send < min_area:
                            return
                        m = cv2.moments(mask, binaryImage=True)
                        if m["m00"] == 0:
                            return
                        cx = int(m["m10"] / m["m00"])
                        cy = int(m["m01"] / m["m00"])
                        ang = compute_angle_360(cx, cy, CENTER_X, CENTER_Y)
                        x, y, w, h = cv2.boundingRect(mask)
                        cv2.rectangle(vis, (x, y), (x + w, y + h), bright_color, 3)
                        cv2.circle(vis, (cx, cy), 7, bright_color, -1)
                        cv2.putText(vis, f"{label} USED ang={ang:.1f} area={area_to_send}",
                                    (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bright_color, 2)

                draw_all_contours(ball_mask, BALL_MIN_AREA, (0, 70, 120), (0, 140, 255), "BALL", "largest_contour")
                draw_all_contours(yellow_mask, YELLOW_MIN_AREA, (0, 120, 120), (0, 255, 255), "YELLOW", "color_cloud")
                draw_all_contours(blue_mask, BLUE_MIN_AREA, (120, 40, 40), (255, 0, 0), "BLUE", "color_cloud")

                cv2.circle(vis, (int(CENTER_X), int(CENTER_Y)), 6, (255, 255, 255), -1)
                cv2.line(vis, (int(CENTER_X) - 12, int(CENTER_Y)), (int(CENTER_X) + 12, int(CENTER_Y)), (255, 255, 255), 1)
                cv2.line(vis, (int(CENTER_X), int(CENTER_Y) - 12), (int(CENTER_X), int(CENTER_Y) + 12), (255, 255, 255), 1)

                ok, jpg = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    global CAM_FRAME_JPEG
                    with CAM_FRAME_LOCK:
                        CAM_FRAME_JPEG = jpg.tobytes()

            ball_angle, ball_area = find_object_angle_area(hsv, BALL_LOWER, BALL_UPPER, BALL_MIN_AREA)
            yellow_angle, yellow_area = find_color_cloud_angle_area(hsv, YELLOW_LOWER, YELLOW_UPPER, YELLOW_MIN_AREA)
            blue_angle, blue_area = find_color_cloud_angle_area(hsv, BLUE_LOWER, BLUE_UPPER, BLUE_MIN_AREA)

            with CAM_STATE_LOCK:
                CAM_STATE["yellow_angle"]   = yellow_angle
                CAM_STATE["yellow_visible"] = (abs(yellow_angle) <= 200.0)
                CAM_STATE["blue_angle"]     = blue_angle
                CAM_STATE["blue_visible"]   = (abs(blue_angle) <= 200.0)
                CAM_STATE["ts"]             = time.time()

            pico_queue_line(f"BALL,{ball_angle:.2f},{ball_area}")
            pico_queue_line(f"YELLOW,{yellow_angle:.2f},{yellow_area}")
            pico_queue_line(f"BLUE,{blue_angle:.2f},{blue_area}")

            dt = time.time() - t0
            sleep_time = CAM_SEND_PERIOD - dt
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"Camera worker error: {e}")
    finally:
        try:
            if picam2:
                picam2.stop()
        except Exception:
            pass


camera_app = Flask("camera_app")


@camera_app.route("/")
def camera_index():
    return """
    <!doctype html>
    <html>
      <head>
        <meta charset="utf-8">
        <title>Camera stream</title>
        <style>
          body { background:#111; color:#eee; font-family:sans-serif; text-align:center; }
          img { margin-top:20px; border:1px solid #444; max-width:95vw; height:auto; }
        </style>
      </head>
      <body>
        <h2>Camera stream</h2>
        <img src="/stream" alt="camera stream">
      </body>
    </html>
    """


@camera_app.route("/stream")
def camera_stream():
    def gen():
        while True:
            with CAM_FRAME_LOCK:
                frame = CAM_FRAME_JPEG
            if frame is not None:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


def camera_web_worker(stop_evt):
    if not ENABLE_CAMERA_VISUALIZATION:
        return
    camera_app.run(host="0.0.0.0", port=CAMERA_WEB_PORT, threaded=True)


# =====================================================================
# Main
# =====================================================================

def pose_sanity_worker(stop_evt):
    """
    Периодически проверяет, согласована ли лидарная поза с тем, где
    робот видит свои ворота (они всегда на Y=0). При расхождении > 90°
    флипает LAST_POSE — это лечит "переворот" поля после столкновения.
    """
    global LAST_POSE

    last_check = 0.0

    while not stop_evt.is_set():
        time.sleep(0.1)
        now = time.time()
        if now - last_check < POSE_SANITY_PERIOD_S:
            continue
        last_check = now

        # 1. snapshot позы
        _, pose, _, _, _, _ = STATE.snapshot()
        if pose is None:
            continue

        # 2. snapshot камеры
        with CAM_STATE_LOCK:
            cam = dict(CAM_STATE)
        if now - cam["ts"] > 2.0:
            continue   # камера давно не отдавала кадров

        # 3. выбрать угол своих ворот
        if OWN_GOAL_COLOR == "blue":
            own_angle   = cam["blue_angle"]
            own_visible = cam["blue_visible"]
        else:
            own_angle   = cam["yellow_angle"]
            own_visible = cam["yellow_visible"]

        if not own_visible:
            continue

        # 4. ожидаемый мировой угол на свои ворота (field_w/2, 0)
        dx = FIELD_W / 2.0 - pose["x"]
        dy = 0.0           - pose["y"]
        if math.hypot(dx, dy) < POSE_SANITY_MIN_DIST_MM:
            continue   # слишком близко, угол шумный

        # та же формула что в compute_drive_command: 0=+Y, 90=+X
        world_angle_math = math.degrees(math.atan2(dy, dx))
        expected_world_angle = wrap360(90.0 - world_angle_math)

        # 5. фактический мировой угол от позы + камеры
        robot_yaw = pose.get("robot_yaw", field_yaw_to_robot_yaw(pose["yaw"]))
        actual_world_angle = wrap360(robot_yaw + own_angle)

        diff = abs(angle_diff_360(actual_world_angle, expected_world_angle))

        # 6. если расходятся > 90° — лидар перевёрнут, флипаем
        if diff > POSE_SANITY_FLIP_THRESHOLD_DEG:
            new_pose = dict(pose)
            new_pose["x"] = FIELD_W - pose["x"]
            new_pose["y"] = FIELD_H - pose["y"]
            yu = pose.get("yaw_unwrapped", pose["yaw"]) + 180.0
            new_pose["yaw_unwrapped"] = yu
            new_pose["yaw"]           = wrap360(yu)
            new_pose["robot_yaw"]     = field_yaw_to_robot_yaw(yu)

            LAST_POSE = new_pose
            STATE.update_pose(new_pose)

            print(f"[POSE SANITY] FLIPPED  diff={diff:.1f}°  "
                  f"expected={expected_world_angle:.1f}°  "
                  f"actual={actual_world_angle:.1f}°  "
                  f"x={new_pose['x']:.0f} y={new_pose['y']:.0f}")

            # дать позе осесть прежде чем снова проверять
            last_check = now + POSE_SANITY_COOLDOWN_S

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar", choices=list(PROTOCOLS.keys()), default="ld19",
                    help="Тип лидара: ld19 или d4a")
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--vid", type=str)
    ap.add_argument("--pid", type=str)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=None,
                    help="по умолчанию: значение DEFAULT_BAUD выбранного протокола")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--web-port", type=int, default=8000)
    ap.add_argument("--own-color", choices=["blue", "yellow"], default="blue",
                help="Цвет защищаемых ворот (должен совпадать с gk_own_goal_color на Pico)")
    args = ap.parse_args()
    
    global OWN_GOAL_COLOR
    OWN_GOAL_COLOR = args.own_color
    print(f"Own goal color: {OWN_GOAL_COLOR}")

    protocol = PROTOCOLS[args.lidar]
    baud = args.baud if args.baud is not None else protocol.DEFAULT_BAUD

    print(f"Lidar protocol: {protocol.name} (packet={protocol.PKT}b, "
          f"header={protocol.HDR.hex()}, baud={baud})")

    get_port = (lambda: autodetect(args.vid, args.pid)) if args.auto else (lambda: args.port)

    stop_evt = threading.Event()

    t_lidar = threading.Thread(
        target=serial_worker,
        args=(get_port, baud, stop_evt, protocol),
        daemon=True
    )
    t_lidar.start()

    t_pico = threading.Thread(target=pico_sender_worker, args=(stop_evt,), daemon=True)
    t_pico.start()

    t_cam = threading.Thread(target=camera_worker, args=(stop_evt,), daemon=True)
    t_cam.start()

    t_cam_web = threading.Thread(target=camera_web_worker, args=(stop_evt,), daemon=True)
    t_cam_web.start()
    
    t_sanity = threading.Thread(target=pose_sanity_worker, args=(stop_evt,), daemon=True)
    t_sanity.start()

    try:
        if ENABLE_LIDAR_VISUALIZATION:
            app.run(host=args.host, port=args.web_port, threaded=True)
        else:
            print("Lidar visualization/web stream disabled.")
            print("Lidar positioning and Pico sending are still running.")
            while True:
                time.sleep(1.0)
    finally:
        stop_evt.set()


if __name__ == "__main__":
    main()