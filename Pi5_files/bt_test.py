# -*- coding: utf-8 -*-
"""
Тест Bluetooth соединения и обмена быстрыми данными между двумя распай.

Использует RFCOMM для установки стабильного соединения.
Требует предварительного сопряжения устройств через bluetoothctl.

Использование:
  # На устройстве-слушателе (robotA):
  python3 bt_test.py --server --target-device robotB --bd-addr AA:BB:CC:DD:EE:FF

  # На устройстве-инициаторе (robotB):
  python3 bt_test.py --client --target-device robotA --bd-addr 11:22:33:44:55:66
"""

import json
import os
import socket
import struct
import sys
import time
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"


def load_device_id():
    """Загружает ID текущего устройства из device_id.local.json"""
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            data = json.load(f)
            return data.get("id", "unknown")
    except FileNotFoundError:
        return "unknown"


def run_server(target_device, channel=1):
    """Запускает Bluetooth сервер (слушатель) на RFCOMM канале"""
    my_id = load_device_id()
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


def run_client(target_device, bd_addr, channel=1):
    """Запускает Bluetooth клиент (инициатор) с подключением к серверу"""
    my_id = load_device_id()
    print(f"[{my_id}] Запуск Bluetooth клиента")
    print(f"  Подключение к {target_device} ({bd_addr}) на канале {channel}...")

    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(5)

    try:
        sock.connect((bd_addr, channel))
        print(f"  ✓ Подключено к {target_device}")

        test_connection(sock, my_id, target_device, is_server=False)
    except socket.timeout:
        print(f"  ✗ Таймаут подключения к {bd_addr}", file=sys.stderr)
    except ConnectionRefusedError:
        print(f"  ✗ Соединение отклонено. Убедитесь, что сервер запущен.", file=sys.stderr)
    except Exception as e:
        print(f"  ✗ Ошибка клиента: {e}", file=sys.stderr)
    finally:
        sock.close()


def test_connection(sock, my_id, peer_id, is_server):
    """Тестирует соединение и обмен данными"""
    sock.settimeout(2)

    try:
        role = "СЕРВЕР" if is_server else "КЛИЕНТ"
        print(f"\n[{my_id}] {role} — начало теста...")

        if is_server:
            test_server(sock, my_id, peer_id)
        else:
            test_client(sock, my_id, peer_id)

        print(f"[{my_id}] Тест завершён ✓\n")
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Прерывание пользователем")
    except socket.timeout:
        print(f"[{my_id}] Таймаут соединения", file=sys.stderr)
    except Exception as e:
        print(f"[{my_id}] Ошибка в тесте: {e}", file=sys.stderr)


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

                    response = struct.pack('<BI', 2, seq)  # PONG
                    sock.send(response)
                    print(f"  → PONG #{seq}")

            elif msg_type == 3:  # FAST_DATA
                if len(data) >= 13:
                    seq, x, y, yaw = struct.unpack('<Ifff', data[1:13])
                    print(f"  ← DATA #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

                    ack = struct.pack('<BI', 4, seq)  # ACK
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
    """Клиент: отправляет разные типы данных и ждёт ответов"""

    print("\n  === Тест PING/PONG ===")
    for i in range(3):
        ping = struct.pack('<BI', 1, i)  # msg_type=1 (PING), seq=i
        sock.send(ping)
        print(f"  → PING #{i}")

        response = sock.recv(1024)
        if response and response[0] == 2:  # PONG
            seq = struct.unpack('<I', response[1:5])[0]
            print(f"  ← PONG #{seq} ✓")

        time.sleep(0.1)

    print("\n  === Тест быстрого обмена данными (x, y, yaw) ===")
    for i in range(5):
        data_packet = struct.pack('<Ifff', i, 1.5 + i*0.1, 2.0 - i*0.2, 0.5*i)
        msg = struct.pack('<B', 3) + data_packet  # msg_type=3 (FAST_DATA)
        sock.send(msg)
        x, y, yaw = struct.unpack('<fff', data_packet)
        print(f"  → DATA #{i}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        response = sock.recv(1024)
        if response and response[0] == 4:  # ACK
            seq = struct.unpack('<I', response[1:5])[0]
            print(f"  ← ACK #{seq} ✓")

        time.sleep(0.05)

    print("\n  === Тест скорости (100 пакетов подряд) ===")
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
    end_msg = struct.pack('<B', 99)  # END
    sock.send(end_msg)
    print(f"  → Сигнал конца")


def main():
    parser = ArgumentParser(description="Тест Bluetooth соединения между двумя распай")
    parser.add_argument(
        "--server",
        action="store_true",
        help="Запустить в режиме сервера (слушателя)"
    )
    parser.add_argument(
        "--client",
        action="store_true",
        help="Запустить в режиме клиента (инициатора)"
    )
    parser.add_argument(
        "--target-device",
        type=str,
        required=True,
        help="ID целевого устройства (например, robotA)"
    )
    parser.add_argument(
        "--bd-addr",
        type=str,
        help="Bluetooth адрес целевого устройства (например, AA:BB:CC:DD:EE:FF). Требуется для --client"
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=1,
        help="RFCOMM канал (по умолчанию 1)"
    )

    args = parser.parse_args()

    if not args.server and not args.client:
        parser.print_help()
        sys.exit(1)

    if args.server:
        run_server(args.target_device, args.channel)
    elif args.client:
        if not args.bd_addr:
            print("Ошибка: --bd-addr требуется для режима --client", file=sys.stderr)
            sys.exit(1)
        run_client(args.target_device, args.bd_addr, args.channel)


if __name__ == "__main__":
    main()
