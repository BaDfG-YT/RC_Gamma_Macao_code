#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import io
import math
import threading
import time

import numpy as np
from lidar_cfg import CRC_TABLE, CRC_INIT
import serial
from serial.tools import list_ports
from flask import Flask, Response
from PIL import Image, ImageDraw

PKT = 47
HDR = b"\x54\x2C"
NPTS = 12

FIELD_W = 1822.0
FIELD_H = 2432.0

# FIELD_W = 1600.0
# FIELD_H = 1800.0

LIDAR_OFFSET_X = 0.0
LIDAR_OFFSET_Y = 0.0

TARGET_X = FIELD_W / 2
TARGET_Y = FIELD_H / 2

PICO_PORT = "/dev/ttyACM0"
PICO_BAUD = 115200
ENABLE_LIDAR_VISUALIZATION = True

UPDATE_PERIOD = 0.05
CLEAR_ON_WRAP = False
MAX_POINTS_FOR_POSE = 2000
MAX_POINTS_TO_DRAW = 500

# Parameters for robust wall detection
WALL_BIN_MM = 35.0
WALL_BAND_MM = 45.0
MIN_WALL_PEAK = 6

# New parameter: penalty for swapped long/short sides
SWAP_PENALTY_GAIN = 30

KEEP_LAST_SCANS = 1




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

ROBOT_YAW_OFFSET_DEG = 180.0
ROBOT_YAW_SIGN = 1.0

def field_yaw_to_robot_yaw(field_yaw_deg):
    return wrap360(ROBOT_YAW_OFFSET_DEG + ROBOT_YAW_SIGN * field_yaw_deg)

def choose_continuous_yaw(fit_yaw, prev_pose):
    if prev_pose is None:
        return float(fit_yaw)

    prev_unwrapped = prev_pose.get("yaw_unwrapped", prev_pose.get("yaw", 0.0))

    candidates = []
    for k in range(-4, 5):
        candidates.append(float(fit_yaw) + 180.0 * k)

    return min(candidates, key=lambda a: abs(a - prev_unwrapped))

def closest_unwrapped_angle(angle_deg, prev_unwrapped):
    candidates = []
    for k in range(-4, 5):
        candidates.append(float(angle_deg) + 360.0 * k)

    return min(candidates, key=lambda a: abs(a - prev_unwrapped))

def angle_diff_deg(a, b):
    d = a - b
    while d >= 90.0:
        d -= 180.0
    while d < -90.0:
        d += 180.0
    return d

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


def robust_minmax(v, q_low=2.0, q_high=98.0):
    return np.percentile(v, q_low), np.percentile(v, q_high)


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

def estimate_main_angles(points_xy):
    pts = points_xy - np.mean(points_xy, axis=0)

    cov = np.cov(pts.T)
    vals, vecs = np.linalg.eig(cov)

    main_vec = vecs[:, np.argmax(vals)]
    ang = math.degrees(math.atan2(main_vec[1], main_vec[0]))
    ang = wrap180(ang)

    # check both the main direction and the perpendicular one
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

        # IMPORTANT: separation is taken from the shorter side, so peaks can be searched for consistently
        uwalls = find_wall_positions(u, bin_size=WALL_BIN_MM, min_sep=min(field_w, field_h) * 0.4)
        vwalls = find_wall_positions(v, bin_size=WALL_BIN_MM, min_sep=min(field_w, field_h) * 0.4)
        if uwalls is None or vwalls is None:
            continue

        umin, umax, hu1, hu2 = uwalls
        vmin, vmax, hv1, hv2 = vwalls

        span_u = umax - umin
        span_v = vmax - vmin

        # Normal side correspondence
        normal_err = abs(span_u - field_w) + abs(span_v - field_h)

        # Swapped side correspondence (90° rotation)
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
                "theta": t,
                "umin": float(umin),
                "umax": float(umax),
                "vmin": float(vmin),
                "vmax": float(vmax),
                "span_u": float(span_u),
                "span_v": float(span_v),
                "normal_err": float(normal_err),
                "swapped_err": float(swapped_err),
                "fit_err": float(fit_err),
                "support": float(support),
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
                "theta": t,
                "umin": float(umin),
                "umax": float(umax),
                "vmin": float(vmin),
                "vmax": float(vmax),
                "span_u": float(span_u),
                "span_v": float(span_v),
                "normal_err": float(normal_err),
                "swapped_err": float(swapped_err),
                "fit_err": float(fit_err),
                "support": float(support),
                "score": float(score),
            }

    base_x = -best["umin"]
    base_y = -best["vmin"]

    fit_yaw = best["theta"]

    # Two physically possible poses for the same rectangle:
    # 1) normal
    # 2) rotated 180°, with mirrored coordinates
    variants = [
        {
            "x": base_x,
            "y": base_y,
            "yaw_base": fit_yaw,
        },
        {
            "x": FIELD_W - base_x,
            "y": FIELD_H - base_y,
            "yaw_base": fit_yaw + 180.0,
        },
    ]

    if prev_pose is not None:
        prev_x = prev_pose["x"]
        prev_y = prev_pose["y"]
        prev_yaw_unwrapped = prev_pose.get("yaw_unwrapped", prev_pose["yaw"])

        best_variant = None
        best_variant_score = None

        for v in variants:
            yu = closest_unwrapped_angle(v["yaw_base"], prev_yaw_unwrapped)

            pos_err = math.hypot(v["x"] - prev_x, v["y"] - prev_y)
            yaw_err = abs(yu - prev_yaw_unwrapped)

            # position matters more than angle, to avoid a mirror flip
            variant_score = pos_err + yaw_err * 2.0

            if best_variant_score is None or variant_score < best_variant_score:
                best_variant_score = variant_score
                best_variant = v
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
        "x": float(x_robot),
        "y": float(y_robot),
        "yaw": float(yaw),
        "yaw_unwrapped": float(yaw_unwrapped),
        "robot_yaw": float(robot_yaw),
        "fit_yaw": float(fit_yaw),
        "score": float(best["score"]),

        # debug
        "umin": best["umin"],
        "umax": best["umax"],
        "vmin": best["vmin"],
        "vmax": best["vmax"],
        "span_u": best["span_u"],
        "span_v": best["span_v"],
        "normal_err": best["normal_err"],
        "swapped_err": best["swapped_err"],
        "fit_err": best["fit_err"],
        "support": best["support"],
    }


def smooth_pose(prev_pose, new_pose, alpha_xy=0.4, alpha_yaw=0.3,
                max_step_xy=60.0, max_step_yaw=200.0):
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

    # IMPORTANT: keep all debug fields from new_pose
    out = dict(new_pose)

    # but replace the smoothed coordinates and angle
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

    # math angle: 0° right, + counterclockwise
    world_angle_math = math.degrees(math.atan2(dy, dx))

    # angle in the robot/field system: 0° up/forward, + clockwise
    world_angle_robot = wrap360(90.0 - world_angle_math)

    robot_yaw = pose.get("robot_yaw", field_yaw_to_robot_yaw(pose["yaw"]))

    # command relative to the robot's front
    cmd_angle = angle_diff_360(world_angle_robot, robot_yaw)

    return cmd_angle, 0, dist, dx, dy


def serial_worker(get_port, baud, stop_evt, timeout=0.02):
    global LAST_POSE

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

            all_pts = []
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
                                    xs = -dist * np.cos(a)
                                    ys = dist * np.sin(a)
                                    pts = np.column_stack((xs, ys)).astype(np.float32)
                                    current_scan_pts.append(pts)

                                s = (int(mv[4]) | (int(mv[5]) << 8)) / 100.0
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

                                if prev is not None and prev > 340.0 and s < 20.0:
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


def pico_sender_worker(stop_evt):
    ser = None

    while not stop_evt.is_set():
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(PICO_PORT, PICO_BAUD, timeout=0.1)
                time.sleep(2.0)

            angle = NAV_STATE["cmd_angle"]
            arrived = NAV_STATE["arrived"]

            _, pose, _, _, _ = STATE.snapshot()

            if pose is not None:
                robot_yaw = angle_diff_360(pose.get("robot_yaw", 0.0), 0.0)
            else:
                robot_yaw = 0.0

            line = f"CMD,{angle:.2f},{arrived},{robot_yaw:.2f}\n"
            ser.write(line.encode("utf-8"))

            time.sleep(0.05)

        except Exception:
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(0.5)


VIEW_EXTRA_MM = 750.0  # margin around the field in mm

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
    """
    Converts a point from the found wall UV-system back into field coordinates.
    In the UV-system:
      umin -> x=0
      vmin -> y=0
    """
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

    # consistent with estimate_pose(), which uses rot2d(points, -yaw)
    x = pose["x"] + pts_local[:, 0] * c + pts_local[:, 1] * s
    y = pose["y"] - pts_local[:, 0] * s + pts_local[:, 1] * c

    return np.column_stack((x, y)).astype(np.float32)


def render_frame():
    img_w, img_h = 1200, 800
    img = Image.new("RGB", (img_w, img_h), (15, 15, 15))
    draw = ImageDraw.Draw(img)

    pts_local, pose, port, bytes_sec, hdr_sec = STATE.snapshot()

    x0, y0, scale = mm_to_px(0, 0, img_w, img_h, FIELD_W, FIELD_H)
    x1, y1, _ = mm_to_px(FIELD_W, FIELD_H, img_w, img_h, FIELD_W, FIELD_H)

    left = min(x0, x1)
    right = max(x0, x1)
    top = min(y0, y1)
    bottom = max(y0, y1)

    draw.rectangle([left, top, right, bottom], outline=(220, 220, 220), width=3)

        # DEBUG: the rectangle the algorithm actually found from the point cloud
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

        # found rectangle — red
        draw.line(
            debug_corners_px + [debug_corners_px[0]],
            fill=(255, 0, 0),
            width=2
        )

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

                # DEBUG: compute UV coordinates of points relative to the found yaw
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
            outline=(255, 80, 80),
            fill=(120, 20, 20),
            width=3
        )

        robot_yaw = pose.get("robot_yaw", field_yaw_to_robot_yaw(yaw))

        a = math.radians(robot_yaw)

        # 0° = up/front of the robot
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
        f"port={port}   bytes/s={bytes_sec}   hdr/s={hdr_sec}   pico={PICO_PORT}",
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

    t_lidar = threading.Thread(
        target=serial_worker,
        args=(get_port, args.baud, stop_evt),
        daemon=True
    )
    t_lidar.start()

    t_pico = threading.Thread(
        target=pico_sender_worker,
        args=(stop_evt,),
        daemon=True
    )
    t_pico.start()

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