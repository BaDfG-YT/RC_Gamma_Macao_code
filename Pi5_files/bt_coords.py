# -*- coding: utf-8 -*-
"""
Двусторонний обмен координатами (x, y, yaw) между двумя распаями по Bluetooth RFCOMM.

Каждая сторона ОДНОВРЕМЕННО:
  - отправляет свои координаты (get_my_coords())
  - принимает координаты другого робота (см. on_peer_coords())

Соединение одно (RFCOMM стрим), но полнодуплексное: приём и отправка
работают в параллельных потоках поверх одного сокета.

Требует предварительной настройки (см. BT_QUICK.txt):
  - compat-режим bluetoothd
  - sdptool add --channel=1 SP на устройстве, которое слушает (--listen)
  - сопряжение устройств (bluetoothctl pair/trust, инициатор - одна сторона)

Использование:
  # На robotA - слушает входящее соединение (роль "listen" при первом запуске):
  sudo python3 bt_coords.py --listen --peer robotB

  # На robotB - подключается к robotA (роль "connect"):
  sudo python3 bt_coords.py --connect robotA

После установления соединения ОБЕ стороны равноправны - каждая и шлёт,
и принимает координаты одновременно.
"""

import json
import socket
import struct
import subprocess
import sys
import threading
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

_running = True


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


def ensure_sdp_service(channel):
    """
    Регистрирует Serial Port (SPP) сервис на заданном RFCOMM-канале через sdptool.

    SDP-запись живёт в runtime-памяти bluetoothd и пропадает при каждом
    перезапуске/перезагрузке - поэтому регистрируем её при каждом старте
    слушающей стороны, а не только один раз вручную.
    """
    try:
        subprocess.run(
            ["sdptool", "add", f"--channel={channel}", "SP"],
            capture_output=True, text=True, timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired) as e:
        print(f"  (не удалось зарегистрировать SDP-сервис: {e})", file=sys.stderr)


def get_my_coords():
    """
    Возвращает ТЕКУЩИЕ координаты этого робота: (x, y, yaw).

    ЗАГЛУШКА: замените на реальный источник (одометрия, лидар, SLAM и т.п.).
    Например, читать из общей переменной/очереди, которую обновляет
    ваш модуль позиционирования.
    """
    t = time.time()
    return (1.0 * (t % 10), 2.0, (t * 0.5) % 6.28)


def on_peer_coords(peer_id, seq, x, y, yaw):
    """
    Вызывается при получении координат от другого робота.

    ЗАГЛУШКА: замените print() на реальную обработку
    (обновление модели мира, навигация и т.п.).
    """
    print(f"  ← [{peer_id}] #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")


def pack_coords(seq, x, y, yaw):
    return struct.pack(PACKET_FORMAT, seq, x, y, yaw)


def unpack_coords(data):
    return struct.unpack(PACKET_FORMAT, data)


def sender_loop(sock, my_id, rate_hz):
    """Поток: периодически отправляет свои координаты"""
    global _running
    period = 1.0 / rate_hz
    seq = 0

    while _running:
        x, y, yaw = get_my_coords()
        try:
            sock.send(pack_coords(seq, x, y, yaw))
            print(f"  → [{my_id}] #{seq}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        except OSError as e:
            print(f"  ✗ Ошибка отправки: {e}", file=sys.stderr)
            _running = False
            break

        seq += 1
        time.sleep(period)


def receiver_loop(sock, peer_id):
    """Поток: непрерывно принимает координаты пира"""
    global _running
    buf = b""

    while _running:
        try:
            data = sock.recv(1024)
        except OSError as e:
            print(f"  ✗ Ошибка приёма: {e}", file=sys.stderr)
            _running = False
            break

        if not data:
            print(f"\n[{peer_id}] Соединение закрыто пиром")
            _running = False
            break

        buf += data
        while len(buf) >= PACKET_SIZE:
            packet, buf = buf[:PACKET_SIZE], buf[PACKET_SIZE:]
            seq, x, y, yaw = unpack_coords(packet)
            on_peer_coords(peer_id, seq, x, y, yaw)


def run_duplex(sock, my_id, peer_id, rate_hz):
    """Запускает приём и отправку параллельно поверх установленного соединения"""
    global _running
    _running = True

    t_send = threading.Thread(target=sender_loop, args=(sock, my_id, rate_hz), daemon=True)
    t_recv = threading.Thread(target=receiver_loop, args=(sock, peer_id), daemon=True)

    t_send.start()
    t_recv.start()

    try:
        while _running:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print(f"\n[{my_id}] Остановлено пользователем")
        _running = False

    t_send.join(timeout=1)
    t_recv.join(timeout=1)


def run_listen(my_id, peer_id, channel=1, rate_hz=10, retry_delay=2.0):
    """
    Слушает входящие соединения в бесконечном цикле: после разрыва
    (перезагрузка/выключение пира, потеря связи) снова ждёт нового
    подключения, не завершая процесс. Останавливается только по Ctrl+C.
    """
    print(f"[{my_id}] Жду подключения от {peer_id} на канале {channel}...")
    ensure_sdp_service(channel)

    while True:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        set_no_security(sock)

        try:
            sock.bind((socket.BDADDR_ANY, channel))
            sock.listen(1)

            conn, addr = sock.accept()
            print(f"  ✓ Подключено: {addr}\n")
            print(f"  === Двусторонний обмен координатами (Ctrl+C для остановки) ===")

            run_duplex(conn, my_id, peer_id, rate_hz)
            conn.close()
            print(f"\n[{my_id}] Соединение потеряно, жду повторного подключения...")
        except KeyboardInterrupt:
            print(f"\n[{my_id}] Остановлено пользователем")
            sock.close()
            break
        except OSError as e:
            print(f"  ✗ Ошибка: {e}, повтор через {retry_delay:.0f}с...", file=sys.stderr)
            time.sleep(retry_delay)
        finally:
            sock.close()


def run_connect(my_id, peer_id, bd_addr, channel=1, rate_hz=10, retry_delay=2.0):
    """
    Подключается к слушающей стороне в бесконечном цикле: повторяет
    попытки, пока слушающая сторона не поднимется, и переподключается
    заново после разрыва связи. Останавливается только по Ctrl+C.
    """
    print(f"[{my_id}] Подключение к {peer_id} ({bd_addr}) на канале {channel}...")

    while True:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        set_no_security(sock)

        try:
            sock.bind((socket.BDADDR_ANY, 0))
            sock.connect((bd_addr, channel))
            print(f"  ✓ Подключено\n")
            print(f"  === Двусторонний обмен координатами (Ctrl+C для остановки) ===")

            run_duplex(sock, my_id, peer_id, rate_hz)
            print(f"\n[{my_id}] Соединение потеряно, пробую переподключиться...")
        except KeyboardInterrupt:
            print(f"\n[{my_id}] Остановлено пользователем")
            sock.close()
            break
        except OSError as e:
            print(f"  ✗ Не удалось подключиться: {e}, повтор через {retry_delay:.0f}с...", file=sys.stderr)
        finally:
            sock.close()

        time.sleep(retry_delay)


def autodetect_role(my_id, config):
    """
    Определяет роль (listen/connect) и пира без флагов, на основе конфига.

    Берёт единственного пира из peers в device_id.local.json. Роль решается
    детерминированно сравнением ID (меньший по алфавиту - слушает), поэтому
    на обеих сторонах получается противоположный, но согласованный результат
    без необходимости вручную задавать кто сервер, а кто клиент.
    """
    peers = config.get("peers", {})
    if not peers:
        print("Ошибка: в device_id.local.json нет ни одного peer'а", file=sys.stderr)
        print("  Добавьте через: python3 bt_setup.py --save-peer <id> <bd_addr>", file=sys.stderr)
        sys.exit(1)

    if len(peers) > 1:
        print(f"Предупреждение: в конфиге несколько peers ({list(peers)}), "
              f"беру первого", file=sys.stderr)

    peer_id = next(iter(peers))
    bd_addr = peers[peer_id].get("bd_addr")
    if not bd_addr:
        print(f"Ошибка: у peer '{peer_id}' не задан bd_addr", file=sys.stderr)
        sys.exit(1)

    is_listener = my_id < peer_id
    return is_listener, peer_id, bd_addr


def main():
    parser = ArgumentParser(description="Двусторонний обмен координатами (x, y, yaw) по Bluetooth")

    group = parser.add_mutually_exclusive_group()
    group.add_argument("--listen", action="store_true", help="Явно: ждать входящее соединение")
    group.add_argument("--connect", type=str, metavar="TARGET_ID", help="Явно: подключиться к TARGET_ID")

    parser.add_argument("--peer", type=str, metavar="TARGET_ID", help="ID пира (для --listen, для логов)")
    parser.add_argument("--force-addr", type=str, help="Явный Bluetooth адрес (для --connect)")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM канал (по умолчанию 1)")
    parser.add_argument("--rate", type=float, default=10.0, help="Частота отправки своих координат, Гц")

    args = parser.parse_args()

    config = load_device_config()
    my_id = config.get("id", "unknown")
    print(f"ID={my_id}\n")

    if args.listen or args.connect:
        # Явный режим (флаги переданы) - старое поведение
        if args.listen:
            peer_id = args.peer or "peer"
            run_listen(my_id, peer_id, args.channel, args.rate)
        else:
            peer_id = args.connect
            bd_addr = config.get("peers", {}).get(peer_id, {}).get("bd_addr") or args.force_addr
            if not bd_addr:
                print(f"Ошибка: адрес для {peer_id} не найден в конфиге", file=sys.stderr)
                print(f"  Используйте --force-addr или bt_setup.py --save-peer", file=sys.stderr)
                sys.exit(1)
            run_connect(my_id, peer_id, bd_addr, args.channel, args.rate)
    else:
        # Без флагов - роль определяется автоматически по конфигу
        is_listener, peer_id, bd_addr = autodetect_role(my_id, config)
        print(f"Роль определена автоматически: "
              f"{'слушаю (listener)' if is_listener else 'подключаюсь (connector)'}, "
              f"пир: {peer_id}\n")

        if is_listener:
            run_listen(my_id, peer_id, args.channel, args.rate)
        else:
            run_connect(my_id, peer_id, bd_addr, args.channel, args.rate)


if __name__ == "__main__":
    main()
