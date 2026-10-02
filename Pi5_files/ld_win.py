#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import time
import threading
from collections import deque

import numpy as np
from lidar_cfg import CRC_TABLE, CRC_INIT
import pygame
import serial
from serial.tools import list_ports

PKT = 47
HDR = b"\x54\x2C"
NPTS = 12

# --- checksum options (disable with --check none) ---


def add_ok(mv): return (sum(mv[:46]) & 0xFF) == mv[46]





def crc_ok(mv):
    crc = CRC_INIT
    for b in mv[2:46]:
        crc = int(CRC_TABLE[(crc ^ b) & 0xFF])
    return (crc & 0xFF) == mv[46]


def verify(mv, mode):
    if mode == "add":
        return add_ok(mv)
    if mode == "crc":
        return crc_ok(mv)
    if mode == "none":
        return True
    return add_ok(mv) or crc_ok(mv)

# --- packet parsing into polar coordinates (angle in degrees, distance in mm) ---


def parse_packet_polar(mv):
    s = (int(mv[4]) | (int(mv[5]) << 8))/100.0
    e = (int(mv[42]) | (int(mv[43]) << 8))/100.0
    d = (360.0-s+e) if (s > 270.0 and e < 90.0) else (e-s)
    t = np.linspace(0.0, 1.0, NPTS, dtype=np.float32)
    ang = ((s+d*t) % 360.0).astype(np.float32)
    dist = np.empty(NPTS, np.int32)
    off = 6
    for i in range(NPTS):
        lo = int(mv[off])
        hi = int(mv[off+1])
        dist[i] = lo | (hi << 8)
        off += 3
    m = dist > 0
    if not np.any(m):
        return None, None
    return ang[m], dist[m].astype(np.float32)

# --- autodetect COM ---


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

# --- serial worker: parsing + state-machine, outputs ('NEW',) and ('POLAR', ang, dist) ---


def serial_worker(get_port, baud, out_q, stop_evt, check_mode, hyst, stats,
                  block_size=4096, timeout=0.02):
    prev = None
    last_new_ts = 0.0
    while not stop_evt.is_set():
        port = get_port()
        if not port:
            time.sleep(0.1)
            continue
        ser = None
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            # toggle DTR/RTS lines
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
            state = 0
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
                    if state == 0:
                        if b == HDR[0]:
                            pkt[0] = b
                            idx = 1
                            state = 1
                    elif state == 1:
                        if b == HDR[1]:
                            pkt[1] = b
                            idx = 2
                            state = 2
                            hdr_sec += 1
                        else:
                            state = 0
                    else:
                        pkt[idx] = b
                        idx += 1
                        if idx == PKT:
                            mv = memoryview(pkt)
                            if verify(mv, check_mode):
                                # parse/dispatch
                                ang, dist = parse_packet_polar(mv)
                                if ang is not None:
                                    out_q.append(("POLAR", ang, dist))
                                # start of a new sweep - debounced with a 100 ms minimum
                                s = (int(mv[4]) | (int(mv[5]) << 8))/100.0
                                now = time.time()
                                if prev is not None and prev > (360.0-hyst) and s < hyst and (now-last_new_ts) > 0.10:
                                    out_q.append(("NEW",))
                                    last_new_ts = now
                                prev = s
                            state = 0
                if i > 0:
                    del buf[:i]
                now = time.time()
                if now-t0 >= 1.0:
                    stats["bytes"] = bytes_sec
                    stats["hdr"] = hdr_sec
                    bytes_sec = 0
                    hdr_sec = 0
                    t0 = now
                if not chunk:
                    time.sleep(0.001)
        except Exception:
            time.sleep(0.2)
        finally:
            try:
                if ser:
                    ser.close()
            except Exception:
                pass

# --- visualization: stable sweep (double-buffer per angle) ---


def main():
    ap = argparse.ArgumentParser("LD19 stable sweep (double-buffer per angle)")
    ap.add_argument("--auto", action="store_true")
    ap.add_argument("--vid", type=str)
    ap.add_argument("--pid", type=str)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--w", type=int, default=800)
    ap.add_argument("--h", type=int, default=600)
    ap.add_argument("--scale", type=float, default=0.05)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--flush-ms", type=int, default=30)
    ap.add_argument(
        "--check", choices=["auto", "add", "crc", "none"], default="none")
    ap.add_argument("--ang-res", type=float, default=0.1,
                    help="bin step in degrees, e.g. 0.1 => 3600 bins")
    args = ap.parse_args()

    get_port = (lambda: autodetect(args.vid, args.pid)
                ) if args.auto else (lambda: args.port)

    q = deque()
    stop = threading.Event()
    stats = {"bytes": 0, "hdr": 0}
    threading.Thread(target=serial_worker,
                     args=(get_port, args.baud, q, stop,
                           args.check, 20.0, stats),
                     daemon=True).start()

    pygame.init()
    screen = pygame.display.set_mode((args.w, args.h))
    pygame.display.set_caption(
        "LD19 stable sweep - drag: pan, wheel: zoom, R reset, C clear")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas,DejaVu Sans Mono,Monospace", 16)

    surface = pygame.Surface((args.w, args.h))
    frame = np.zeros((args.w, args.h, 3), np.uint8)
    scale = args.scale
    offx = offy = 0.0
    last_flush = time.time()

    # fixed-angle bins
    res = args.ang_res
    bins = int(round(360.0/res))
    dist_cur = np.full(bins, -1.0, dtype=np.float32)   # current sweep
    # last completed sweep (always shows the previous full revolution)
    dist_last = np.full(bins, -1.0, dtype=np.float32)
    ang_bins_deg = (np.arange(bins, dtype=np.float32)+0.5)*res
    ang_bins_rad = np.deg2rad(ang_bins_deg)

    def to_screen_xy(xs, ys):
        sx = (args.w*0.5+offx+xs*scale).astype(np.int32, copy=False)
        sy = (args.h*0.5+offy+ys*scale).astype(np.int32, copy=False)
        m = (sx >= 0) & (sx < args.w) & (sy >= 0) & (sy < args.h)
        return sx[m], sy[m]

    last_swap_ts = 0.0  # debounce for the swap event
    running = True
    panning = False
    last = (0, 0)
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                running = False
            elif e.type == pygame.MOUSEBUTTONDOWN:
                if e.button == 1:
                    panning = True
                    last = pygame.mouse.get_pos()
                elif e.button == 4:
                    mx, my = pygame.mouse.get_pos()
                    wx = (mx-(args.w*0.5+offx))/scale
                    wy = (my-(args.h*0.5+offy))/scale
                    scale *= 1.15
                    offx += mx-((args.w*0.5+offx)+wx*scale)
                    offy += my-((args.h*0.5+offy)+wy*scale)
                elif e.button == 5:
                    mx, my = pygame.mouse.get_pos()
                    wx = (mx-(args.w*0.5+offx))/scale
                    wy = (my-(args.h*0.5+offy))/scale
                    scale /= 1.15
                    offx += mx-((args.w*0.5+offx)+wx*scale)
                    offy += my-((args.h*0.5+offy)+wy*scale)
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                panning = False
            elif e.type == pygame.MOUSEMOTION and panning:
                mx, my = pygame.mouse.get_pos()
                offx += mx-last[0]
                offy += my-last[1]
                last = (mx, my)
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_r:
                    scale = args.scale
                    offx = offy = 0.0
                elif e.key == pygame.K_c:
                    dist_cur.fill(-1.0)
                    dist_last.fill(-1.0)
                    frame[:] = 0

        # queue
        need_swap = False
        while q:
            it = q.popleft()
            if it[0] == "NEW":
                # debounce NEW: no more often than once per 80 ms
                if time.time() - last_swap_ts > 0.08:
                    need_swap = True
            elif it[0] == "POLAR":
                _, ang_deg, dist_mm = it
                idx = np.floor(ang_deg / res + 0.5).astype(np.int32) % bins
                dist_cur[idx] = dist_mm  # update the bin by index

        # if a full sweep came in — swap the buffers and clear the current one
        if need_swap:
            dist_last[:] = dist_cur
            dist_cur.fill(-1.0)
            last_swap_ts = time.time()

        # rendering by timer: always draws dist_last
        if (time.time()-last_flush)*1000.0 >= args.flush_ms:
            frame[:] = 0
            m = dist_last >= 0.0
            if np.any(m):
                d = dist_last[m]
                a = ang_bins_rad[m]
                xs = d*np.cos(a)
                ys = -d*np.sin(a)
                sx, sy = to_screen_xy(
                    xs.astype(np.float32), ys.astype(np.float32))
                frame[sx, sy] = (230, 230, 230)

            # axes
            frame[:, int(args.h*0.5+offy), :] = (60, 60, 60)
            frame[int(args.w*0.5+offx), :, :] = (60, 60, 60)

            px = pygame.surfarray.pixels3d(surface)
            px[:] = frame
            del px
            screen.blit(surface, (0, 0))
            status = f"port:{get_port() or 'N/A'} baud:{args.baud} bytes/s:{stats['bytes']} hdr/s:{stats['hdr']} bins:{bins} res:{res}° fps:{int(clock.get_fps())}"
            screen.blit(font.render(status, True, (255, 255, 255)), (10, 10))
            pygame.display.flip()
            last_flush = time.time()

        clock.tick(args.fps)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()
