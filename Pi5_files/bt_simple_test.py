# -*- coding: utf-8 -*-
"""
Простой Bluetooth тест без rfcomm - прямые сокеты.
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


def load_device_config():
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Ошибка: {DEVICE_ID_PATH} не найден", file=sys.stderr)
        sys.exit(1)


SOL_BLUETOOTH = 274
BT_SECURITY = 4
BT_SECURITY_SDP = 0


def set_no_security(sock):
    """Отключить требование шифрования/аутентификации на RFCOMM сокете"""
    try:
        # struct bt_security { uint8_t level; uint8_t key_size; }
        sock.setsockopt(SOL_BLUETOOTH, BT_SECURITY, struct.pack("BB", BT_SECURITY_SDP, 0))
    except OSError as e:
        print(f"  (не удалось снизить security level: {e})", file=sys.stderr)


def run_server(my_id, peer_id, channel=1):
    """Сервер - слушает входящие соединения"""
    print(f"[{my_id}] Запуск BT сервера на канале {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    set_no_security(sock)

    try:
        sock.bind((socket.BDADDR_ANY, channel))
        sock.listen(1)
        print(f"  Слушаю на канале {channel}...")

        conn, addr = sock.accept()
        print(f"  ✓ Подключено: {addr}\n")

        # Тест
        print("  === Тест ===")
        for i in range(5):
            data = conn.recv(1024)
            if data:
                print(f"  ← Получено: {len(data)} байт")
                conn.send(b"OK")
                print(f"  → Отправлено: OK")
            time.sleep(0.1)

        conn.close()
        print("\n  ✓ Тест завершён\n")
    except Exception as e:
        print(f"  ✗ Ошибка: {e}", file=sys.stderr)
    finally:
        sock.close()


def run_client(my_id, peer_id, bd_addr, channel=1):
    """Клиент - подключается к серверу"""
    print(f"[{my_id}] Подключение к {bd_addr} на канале {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    set_no_security(sock)

    try:
        sock.bind((socket.BDADDR_ANY, 0))
        sock.connect((bd_addr, channel))
        print(f"  ✓ Подключено\n")

        # Тест
        print("  === Тест ===")
        for i in range(5):
            data = f"Test {i}".encode()
            sock.send(data)
            print(f"  → Отправлено: {data.decode()}")

            resp = sock.recv(1024)
            if resp:
                print(f"  ← Получено: {resp.decode()}")

            time.sleep(0.2)

        print("\n  ✓ Тест завершён\n")
    except Exception as e:
        print(f"  ✗ Ошибка: {e}", file=sys.stderr)
    finally:
        sock.close()


def main():
    parser = ArgumentParser(description="Простой BT тест")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--server", type=str, metavar="TARGET")
    group.add_argument("--client", type=str, metavar="TARGET")

    parser.add_argument("--force-addr", type=str)
    parser.add_argument("--channel", type=int, default=1)

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")
    print(f"ID={my_id}\n")

    if args.server:
        run_server(my_id, args.server, args.channel)
    else:
        bd_addr = config.get("peers", {}).get(args.client, {}).get("bd_addr") or args.force_addr
        if not bd_addr:
            print(f"Ошибка: адрес для {args.client} не найден", file=sys.stderr)
            sys.exit(1)
        run_client(my_id, args.client, bd_addr, args.channel)


if __name__ == "__main__":
    main()
