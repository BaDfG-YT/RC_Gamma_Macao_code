# -*- coding: utf-8 -*-
"""
Передача координат (x, y, yaw) между двумя распаями по Bluetooth RFCOMM.

Основано на рабочей конфигурации bt_simple_test.py.
Требует предварительной настройки (см. BT_QUICK.txt):
  - compat-режим bluetoothd
  - sdptool add --channel=1 SP на СЕРВЕРЕ
  - сопряжение устройств (bluetoothctl pair/trust, с ОДНОЙ стороны)

Использование:
  # На устройстве-источнике координат (например, лидар/камера) - сервер:
  sudo python3 bt_coords.py --server robotB

  # На устройстве-получателе координат - клиент:
  sudo python3 bt_coords.py --client robotA
"""

import json
import socket
import struct
import sys
import time
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"

SOL_BLUETOOTH = 274
BT_SECURITY = 4
BT_SECURITY_SDP = 0

PACKET_FORMAT = "<Ifff"  # seq (uint32), x, y, yaw (float32 each), little-endian
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


def load_device_config():
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Ошибка: {DEVICE_ID_PATH} не найден", file=sys.stderr)
        sys.exit(1)


def set_no_security(sock):
    """Отключить требование шифрования/аутентификации на RFCOMM сокете"""
    try:
        sock.setsockopt(SOL_BLUETOOTH, BT_SECURITY, struct.pack("BB", BT_SECURITY_SDP, 0))
    except OSError as e:
        print(f"  (не удалось снизить security level: {e})", file=sys.stderr)


def pack_coords(seq, x, y, yaw):
    return struct.pack(PACKET_FORMAT, seq, x, y, yaw)


def unpack_coords(data):
    seq, x, y, yaw = struct.unpack(PACKET_FORMAT, data)
    return seq, x, y, yaw


def run_server(my_id, peer_id, channel=1):
    """
    Сервер (получатель координат): принимает пакеты x,y,yaw и печатает их.
    Замените print() на вашу логику обработки координат.
    """
    print(f"[{my_id}] Запуск BT сервера координат на канале {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    set_no_security(sock)

    try:
        sock.bind((socket.BDADDR_ANY, channel))
        sock.listen(1)
        print(f"  Слушаю на канале {channel}...")

        conn, addr = sock.accept()
        print(f"  ✓ Подключено: {addr}\n")
        print("  === Приём координат (Ctrl+C для остановки) ===")

        buf = b""
        count = 0
        t_start = time.time()

        while True:
            data = conn.recv(1024)
            if not data:
                print(f"\n[{my_id}] Соединение закрыто пиром")
                break

            buf += data
            while len(buf) >= PACKET_SIZE:
                packet, buf = buf[:PACKET_SIZE], buf[PACKET_SIZE:]
                seq, x, y, yaw = unpack_coords(packet)
                count += 1
                print(f"  ← #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        elapsed = time.time() - t_start
        rate = count / elapsed if elapsed > 0 else 0
        print(f"\n  Получено пакетов: {count} за {elapsed:.1f}с ({rate:.1f} пак/сек)")

        conn.close()
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Остановлено пользователем")
    except Exception as e:
        print(f"  ✗ Ошибка: {e}", file=sys.stderr)
    finally:
        sock.close()


def run_client(my_id, peer_id, bd_addr, channel=1, rate_hz=10, source=None):
    """
    Клиент (источник координат): отправляет x,y,yaw с заданной частотой.
    По умолчанию отправляет тестовые синтетические координаты.
    Для реальных данных подключите source() - функцию без аргументов,
    возвращающую (x, y, yaw).
    """
    print(f"[{my_id}] Подключение к {bd_addr} на канале {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    set_no_security(sock)

    try:
        sock.bind((socket.BDADDR_ANY, 0))
        sock.connect((bd_addr, channel))
        print(f"  ✓ Подключено\n")
        print(f"  === Отправка координат с частотой {rate_hz} Гц (Ctrl+C для остановки) ===")

        period = 1.0 / rate_hz
        seq = 0
        t_start = time.time()

        while True:
            if source is not None:
                x, y, yaw = source()
            else:
                # Тестовые данные - синтетическое движение по кругу
                t = time.time() - t_start
                x = 1.0 * (t % 10)
                y = 2.0
                yaw = (t * 0.5) % 6.28

            packet = pack_coords(seq, x, y, yaw)
            sock.send(packet)
            print(f"  → #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

            seq += 1
            time.sleep(period)

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        rate = seq / elapsed if elapsed > 0 else 0
        print(f"\n[{my_id}] Остановлено. Отправлено {seq} пакетов за {elapsed:.1f}с ({rate:.1f} пак/сек)")
    except Exception as e:
        print(f"  ✗ Ошибка: {e}", file=sys.stderr)
    finally:
        sock.close()


def main():
    parser = ArgumentParser(description="Передача координат (x, y, yaw) по Bluetooth")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--server", type=str, metavar="TARGET_ID", help="Принимать координаты (сервер)")
    group.add_argument("--client", type=str, metavar="TARGET_ID", help="Отправлять координаты (клиент)")

    parser.add_argument("--force-addr", type=str, help="Явный Bluetooth адрес")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM канал (по умолчанию 1)")
    parser.add_argument("--rate", type=float, default=10.0, help="Частота отправки, Гц (по умолчанию 10)")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")
    print(f"ID={my_id}\n")

    if args.server:
        run_server(my_id, args.server, args.channel)
    else:
        bd_addr = config.get("peers", {}).get(args.client, {}).get("bd_addr") or args.force_addr
        if not bd_addr:
            print(f"Ошибка: адрес для {args.client} не найден в конфиге", file=sys.stderr)
            print(f"  Используйте --force-addr или bt_setup.py --save-peer", file=sys.stderr)
            sys.exit(1)
        run_client(my_id, args.client, bd_addr, args.channel, args.rate)


if __name__ == "__main__":
    main()
