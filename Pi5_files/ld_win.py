#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import time
import threading
from collections import deque

import numpy as np
import pygame
import serial
from serial.tools import list_ports

PKT = 47
HDR = b"\x54\x2C"
NPTS = 12

# --- checksum options (����� --check none) ---


def add_ok(mv): return (sum(mv[:46]) & 0xFF) == mv[46]


CRC_TABLE = np.array([
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
], dtype=np.uint8)


def crc_ok(mv):
    crc = 0xD8
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

# --- ������ ������ � �������� ����� (���� � ��������, ��������� ��) ---


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

# --- serial worker: ����� + state-machine, ��� ('NEW',) � ('POLAR', ang, dist) ---


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
            # ������� ������� DTR/RTS
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
                                # ����/���������
                                ang, dist = parse_packet_polar(mv)
                                if ang is not None:
                                    out_q.append(("POLAR", ang, dist))
                                # ������ ������ ������� � ��������� ������������ 100 ��
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

# --- ������������: ���������� ����� (������� ����� �� �����) ---


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
                    help="��� �� ���� (�), ����. 0.1 => 3600 �����")
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
        "LD19 stable sweep � ������: ���, ���: ���, R reset, C clear")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Consolas,DejaVu Sans Mono,Monospace", 16)

    surface = pygame.Surface((args.w, args.h))
    frame = np.zeros((args.w, args.h, 3), np.uint8)
    scale = args.scale
    offx = offy = 0.0
    last_flush = time.time()

    # ������� �������� �����
    res = args.ang_res
    bins = int(round(360.0/res))
    dist_cur = np.full(bins, -1.0, dtype=np.float32)   # ������� ������
    # ����������� ������ (������ ������ ���)
    dist_last = np.full(bins, -1.0, dtype=np.float32)
    ang_bins_deg = (np.arange(bins, dtype=np.float32)+0.5)*res
    ang_bins_rad = np.deg2rad(ang_bins_deg)

    def to_screen_xy(xs, ys):
        sx = (args.w*0.5+offx+xs*scale).astype(np.int32, copy=False)
        sy = (args.h*0.5+offy+ys*scale).astype(np.int32, copy=False)
        m = (sx >= 0) & (sx < args.w) & (sy >= 0) & (sy < args.h)
        return sx[m], sy[m]

    last_swap_ts = 0.0  # ������ �� ������ swap
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

        # ����
        need_swap = False
        while q:
            it = q.popleft()
            if it[0] == "NEW":
                # �������� NEW: �� ���� ��� � 80 ��
                if time.time() - last_swap_ts > 0.08:
                    need_swap = True
            elif it[0] == "POLAR":
                _, ang_deg, dist_mm = it
                idx = np.floor(ang_deg / res + 0.5).astype(np.int32) % bins
                dist_cur[idx] = dist_mm  # ��������� ����� �� ���

        # ���� �������� ����� ���� � swap ������� � ������������
        if need_swap:
            dist_last[:] = dist_cur
            dist_cur.fill(-1.0)
            last_swap_ts = time.time()

        # ����������� �� �������: ������ ������ dist_last
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

            # �����
            frame[:, int(args.h*0.5+offy), :] = (60, 60, 60)
            frame[int(args.w*0.5+offx), :, :] = (60, 60, 60)

            px = pygame.surfarray.pixels3d(surface)
            px[:] = frame
            del px
            screen.blit(surface, (0, 0))
            status = f"port:{get_port() or 'N/A'} baud:{args.baud} bytes/s:{stats['bytes']} hdr/s:{stats['hdr']} bins:{bins} res:{res}� fps:{int(clock.get_fps())}"
            screen.blit(font.render(status, True, (255, 255, 255)), (10, 10))
            pygame.display.flip()
            last_flush = time.time()

        clock.tick(args.fps)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()
