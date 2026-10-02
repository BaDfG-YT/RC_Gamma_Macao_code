# -*- coding: utf-8 -*-
"""
Bluetooth тест через /dev/rfcomm устройства (более надёжно на RPi).

Требует: sudo rfcomm listen hci0 1 запущенным на сервере

Использование:
  # На сервере (robotA/bropi) - в отдельном терминале:
  $ sudo rfcomm listen hci0 1

  # На сервере - в другом терминале:
  $ python3 bt_test_rfcomm.py --server robotB

  # На клиенте (robotB/bropi2):
  $ python3 bt_test_rfcomm.py --client robotA
"""

import json
import os
import select
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
    """Получает адрес пира из конфига"""
    if force_addr:
        return force_addr

    peers = config.get("peers", {})
    if peer_id in peers:
        return peers[peer_id].get("bd_addr")

    return None


def wait_for_client(rfcomm_device="/dev/rfcomm0"):
    """Ждёт подключения клиента (требует rfcomm listen на сервере)"""
    print(f"  Ожидание подключения на {rfcomm_device}...")

    # Проверить что устройство существует
    if not os.path.exists(rfcomm_device):
        print(f"  ✗ Ошибка: {rfcomm_device} не найден", file=sys.stderr)
        print(f"    Запустите на сервере: sudo rfcomm listen hci0 1", file=sys.stderr)
        return None

    # Открыть устройство
    try:
        f = os.open(rfcomm_device, os.O_RDWR)
        print(f"  ✓ Подключено к {rfcomm_device}")
        return f
    except OSError as e:
        print(f"  ✗ Ошибка открытия {rfcomm_device}: {e}", file=sys.stderr)
        return None


def connect_to_server(bd_addr, rfcomm_device="/dev/rfcomm0", timeout=10):
    """Подключиться к серверу через rfcomm"""
    print(f"  Подключение к {bd_addr} на {rfcomm_device}...")

    # Использовать rfcomm connect
    import subprocess

    try:
        result = subprocess.run(
            ["sudo", "rfcomm", "connect", rfcomm_device, bd_addr, "1"],
            timeout=timeout,
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            time.sleep(0.5)  # Дать время на установку соединения

            # Открыть устройство
            f = os.open(rfcomm_device, os.O_RDWR)
            print(f"  ✓ Подключено к {bd_addr}")
            return f
        else:
            print(f"  ✗ Ошибка подключения: {result.stderr}", file=sys.stderr)
            return None
    except subprocess.TimeoutExpired:
        print(f"  ✗ Таймаут подключения", file=sys.stderr)
        return None
    except Exception as e:
        print(f"  ✗ Ошибка: {e}", file=sys.stderr)
        return None


def send_data(fd, data):
    """Отправить данные"""
    try:
        os.write(fd, data)
        return True
    except Exception as e:
        print(f"  ✗ Ошибка отправки: {e}", file=sys.stderr)
        return False


def recv_data(fd, size, timeout=2):
    """Получить данные с таймаутом"""
    try:
        ready = select.select([fd], [], [], timeout)
        if ready[0]:
            return os.read(fd, size)
        else:
            return None
    except Exception as e:
        print(f"  ✗ Ошибка приёма: {e}", file=sys.stderr)
        return None


def run_server(my_id, peer_id):
    """Сервер: принимает и отвечает на тесты"""
    print(f"[{my_id}] Запуск Bluetooth сервера")
    print(f"  Убедитесь что запущено: sudo rfcomm listen hci0 1")

    fd = wait_for_client()
    if fd is None:
        return

    try:
        print(f"\n[{my_id}] СЕРВЕР — начало теста...\n")
        packets_received = 0

        while True:
            data = recv_data(fd, 1024)
            if not data:
                print(f"[{my_id}] Соединение закрыто")
                break

            packets_received += 1
            msg_type = data[0]

            if msg_type == 1:  # PING
                if len(data) >= 5:
                    seq = struct.unpack('<I', data[1:5])[0]
                    print(f"  ← PING #{seq}")

                    response = struct.pack('<BI', 2, seq)
                    send_data(fd, response)
                    print(f"  → PONG #{seq}")

            elif msg_type == 3:  # FAST_DATA
                if len(data) >= 13:
                    seq, x, y, yaw = struct.unpack('<Ifff', data[1:13])
                    print(f"  ← DATA #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

                    ack = struct.pack('<BI', 4, seq)
                    send_data(fd, ack)
                    print(f"  → ACK #{seq}")

            elif msg_type == 99:  # END
                print(f"  ← Сигнал конца")
                break

        print(f"\n[{my_id}] Тест завершён ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Прерывание")
    finally:
        os.close(fd)


def run_client(my_id, peer_id, bd_addr):
    """Клиент: отправляет тесты"""
    print(f"[{my_id}] Запуск Bluetooth клиента")

    fd = connect_to_server(bd_addr)
    if fd is None:
        return

    try:
        print(f"\n[{my_id}] КЛИЕНТ — начало теста...\n")

        # PING/PONG
        print("  === PING/PONG ===")
        for i in range(3):
            ping = struct.pack('<BI', 1, i)
            send_data(fd, ping)
            print(f"  → PING #{i}")

            response = recv_data(fd, 1024)
            if response and response[0] == 2:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← PONG #{seq} ✓")
            else:
                print(f"  (нет ответа)")
            time.sleep(0.1)

        # Быстрый обмен
        print("\n  === Быстрый обмен (x, y, yaw) ===")
        for i in range(5):
            data_packet = struct.pack('<Ifff', i, 1.5 + i*0.1, 2.0 - i*0.2, 0.5*i)
            msg = struct.pack('<B', 3) + data_packet
            send_data(fd, msg)
            x, y, yaw = struct.unpack('<fff', data_packet)
            print(f"  → DATA #{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

            response = recv_data(fd, 1024)
            if response and response[0] == 4:
                seq = struct.unpack('<I', response[1:5])[0]
                print(f"  ← ACK #{seq} ✓")
            else:
                print(f"  (нет ответа)")
            time.sleep(0.05)

        # Тест скорости
        print("\n  === Тест скорости (100 пакетов) ===")
        start_time = time.time()
        packets_sent = 0

        for i in range(100):
            data_packet = struct.pack('<Ifff', i, 1.0, 2.0, 3.0)
            msg = struct.pack('<B', 3) + data_packet
            send_data(fd, msg)
            packets_sent += 1

            response = recv_data(fd, 1024, timeout=0.5)
            if not response:
                break

        elapsed = time.time() - start_time
        rate = packets_sent / elapsed if elapsed > 0 else 0
        print(f"  Отправлено: {packets_sent} пакетов за {elapsed:.3f}с ({rate:.1f} пак/сек)")

        # Завершение
        print("\n  === Завершение ===")
        end_msg = struct.pack('<B', 99)
        send_data(fd, end_msg)
        print(f"  → Сигнал конца")

        print(f"\n[{my_id}] Тест завершён ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Прерывание")
    finally:
        os.close(fd)


def main():
    parser = ArgumentParser(description="Bluetooth тест через /dev/rfcomm")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--server", type=str, metavar="TARGET_ID",
                      help="Запустить сервер")
    group.add_argument("--client", type=str, metavar="TARGET_ID",
                      help="Запустить клиент")

    parser.add_argument("--force-addr", type=str,
                      help="Явный Bluetooth адрес")
    parser.add_argument("--device", type=str, default="/dev/rfcomm0",
                      help="RFCOMM устройство (по умолчанию /dev/rfcomm0)")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")

    print(f"Конфиг: ID={my_id}\n")

    if args.server:
        run_server(my_id, args.server)
    elif args.client:
        bd_addr = get_peer_addr(config, args.client, args.force_addr)
        if not bd_addr:
            print(f"Ошибка: адрес для {args.client} не найден", file=sys.stderr)
            sys.exit(1)
        run_client(my_id, args.client, bd_addr)


if __name__ == "__main__":
    main()
