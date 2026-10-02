# -*- coding: utf-8 -*-
"""
Расширенная версия bt_test.py с автоматическим получением адресов из device_id.local.json.

Использование:
  # Запустить сервер (автоматический выбор порта)
  python3 bt_test_auto.py --server robotB

  # Запустить клиент с автоматическим получением адреса из конфига
  python3 bt_test_auto.py --client robotA

  # Запустить с явным адресом (если не сохранён в конфиге)
  python3 bt_test_auto.py --client robotA --force-addr AA:BB:CC:DD:EE:FF
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
    """Загружает конфиг текущего устройства"""
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Ошибка: {DEVICE_ID_PATH} не найден", file=sys.stderr)
        sys.exit(1)


def get_peer_addr(config, peer_id, force_addr=None):
    """Получает адрес пира из конфига или использует явный адрес"""
    if force_addr:
        return force_addr

    peers = config.get("peers", {})
    if peer_id in peers:
        return peers[peer_id].get("bd_addr")

    return None


def run_server(my_config, target_device, channel=1):
    """Запускает Bluetooth сервер"""
    my_id = my_config.get("id", "unknown")
    print(f"[{my_id}] Запуск Bluetooth сервера, канал {channel}")
    print(f"  Ожидание подключения от {target_device}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.bind((socket.BDADDR_ANY, channel))
    sock.listen(1)
    sock.settimeout(None)

    try:
        client_sock, client_addr = sock.accept()
        print(f"  ✓ Подключено: {client_addr}")

        test_connection(client_sock, my_id, target_device, is_server=True)
    except KeyboardInterrupt:
        print("\n  Отмена...")
    except Exception as e:
        print(f"  ✗ Ошибка сервера: {e}", file=sys.stderr)
    finally:
        sock.close()


def run_client(my_config, target_device, bd_addr, channel=1):
    """Запускает Bluetooth клиент"""
    my_id = my_config.get("id", "unknown")
    print(f"[{my_id}] Запуск Bluetooth клиента")
    print(f"  Подключение к {target_device} ({bd_addr}) на канале {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(5)

    try:
        sock.connect((bd_addr, channel))
        print(f"  ✓ Подключено к {target_device}")

        test_connection(sock, my_id, target_device, is_server=False)
    except socket.timeout:
        print(f"  ✗ Таймаут подключения", file=sys.stderr)
    except ConnectionRefusedError:
        print(f"  ✗ Соединение отклонено", file=sys.stderr)
    except Exception as e:
        print(f"  ✗ Ошибка клиента: {e}", file=sys.stderr)
    finally:
        sock.close()


def test_connection(sock, my_id, peer_id, is_server):
    """Тестирует соединение и обмен данными"""
    sock.settimeout(2)

    try:
        role = "СЕРВЕР" if is_server else "КЛИЕНТ"
        print(f"\n[{my_id}] {role} — начало теста...\n")

        if is_server:
            test_server(sock, my_id, peer_id)
        else:
            test_client(sock, my_id, peer_id)

        print(f"\n[{my_id}] Тест завершён ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Прерывание пользователем")
    except socket.timeout:
        print(f"[{my_id}] Таймаут соединения", file=sys.stderr)
    except Exception as e:
        print(f"[{my_id}] Ошибка: {e}", file=sys.stderr)


def test_server(sock, my_id, peer_id):
    """Сервер: принимает тесты от клиента и отвечает"""
    packets_received = 0
    bytes_received = 0

    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print(f"[{my_id}] Соединение закрыто пиром")
                break

            packets_received += 1
            bytes_received += len(data)
            msg_type = data[0]

            if msg_type == 1:  # PING
                if len(data) >= 5:
                    seq = struct.unpack('<I', data[1:5])[0]
                    print(f"  ← PING #{seq} ({len(data)} байт)")
                    response = struct.pack('<BI', 2, seq)
                    sock.send(response)
                    print(f"  → PONG #{seq}")

            elif msg_type == 3:  # FAST_DATA
                if len(data) >= 13:
                    seq, x, y, yaw = struct.unpack('<Ifff', data[1:13])
                    print(f"  ← DATA #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
                    ack = struct.pack('<BI', 4, seq)
                    sock.send(ack)
                    print(f"  → ACK #{seq}")

            elif msg_type == 99:  # END
                print(f"  ← Сигнал конца теста")
                break

        except socket.timeout:
            if packets_received > 0:
                print(f"  Таймаут. Получено {packets_received} пакетов, {bytes_received} байт")
            break


def test_client(sock, my_id, peer_id):
    """Клиент: отправляет данные и ждёт ответов"""

    print("  === PING/PONG ===")
    for i in range(3):
        ping = struct.pack('<BI', 1, i)
        sock.send(ping)
        print(f"  → PING #{i}")
        try:
            response = sock.recv(1024)
            if response and response[0] == 2:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← PONG #{seq} ✓")
        except socket.timeout:
            print(f"  (нет ответа)")
        time.sleep(0.1)

    print("\n  === Быстрый обмен (x, y, yaw) ===")
    for i in range(5):
        data_packet = struct.pack('<Ifff', i, 1.5 + i*0.1, 2.0 - i*0.2, 0.5*i)
        msg = struct.pack('<B', 3) + data_packet
        sock.send(msg)
        x, y, yaw = struct.unpack('<fff', data_packet)
        print(f"  → DATA #{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        try:
            response = sock.recv(1024)
            if response and response[0] == 4:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← ACK #{seq} ✓")
        except socket.timeout:
            print(f"  (нет ответа)")
        time.sleep(0.05)

    print("\n  === Тест скорости (100 пакетов) ===")
    start_time = time.time()
    packets_sent = 0

    for i in range(100):
        data_packet = struct.pack('<Ifff', i, 1.0, 2.0, 3.0)
        msg = struct.pack('<B', 3) + data_packet
        sock.send(msg)
        packets_sent += 1

        try:
            response = sock.recv(1024)
            if not response:
                break
        except socket.timeout:
            pass

    elapsed = time.time() - start_time
    rate = packets_sent / elapsed if elapsed > 0 else 0
    print(f"  Отправлено: {packets_sent} пакетов за {elapsed:.3f}с ({rate:.1f} пак/сек)")

    print("\n  === Завершение ===")
    end_msg = struct.pack('<B', 99)
    sock.send(end_msg)
    print(f"  → Сигнал конца")


def main():
    parser = ArgumentParser(description="Bluetooth тест с автоматической загрузкой адресов")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--server", type=str, metavar="TARGET_ID", help="Запустить сервер (ожидает подключение от TARGET_ID)")
    group.add_argument("--client", type=str, metavar="TARGET_ID", help="Запустить клиент (подключиться к TARGET_ID)")

    parser.add_argument("--force-addr", type=str, help="Явный Bluetooth адрес (если не сохранён в конфиге)")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM канал (по умолчанию 1)")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")

    if args.server:
        print(f"Конфиг: ID={my_id}")
        run_server(config, args.server, args.channel)

    elif args.client:
        print(f"Конфиг: ID={my_id}")
        bd_addr = get_peer_addr(config, args.client, args.force_addr)

        if not bd_addr:
            print(f"Ошибка: адрес для {args.client} не найден в конфиге", file=sys.stderr)
            print(f"  Используйте --force-addr или сохраните адрес через bt_setup.py", file=sys.stderr)
            sys.exit(1)

        run_client(config, args.client, bd_addr, args.channel)


if __name__ == "__main__":
    main()
