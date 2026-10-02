# -*- coding: utf-8 -*-
"""
Утилита для настройки Bluetooth соединения между распаями.

Использование:
  # Сканировать доступные Bluetooth устройства
  python3 bt_setup.py --scan

  # Сопрячь устройство
  python3 bt_setup.py --pair AA:BB:CC:DD:EE:FF

  # Получить список сопряжённых устройств
  python3 bt_setup.py --list

  # Сохранить адрес в device_id.local.json
  python3 bt_setup.py --save-peer robotA AA:BB:CC:DD:EE:FF
"""

import json
import subprocess
import sys
from argparse import ArgumentParser
from pathlib import Path

DEVICE_DIR = Path(__file__).parent.parent / "device"
DEVICE_ID_PATH = DEVICE_DIR / "device_id.local.json"


def load_device_id():
    """Загружает конфиг устройства"""
    try:
        with open(DEVICE_ID_PATH, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        return {"id": "unknown", "type": "robot"}


def save_device_id(data):
    """Сохраняет конфиг устройства"""
    DEVICE_DIR.mkdir(parents=True, exist_ok=True)
    with open(DEVICE_ID_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def run_command(cmd):
    """Запускает shell команду и возвращает вывод"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.stdout.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", -1
    except Exception as e:
        print(f"Ошибка выполнения команды: {e}", file=sys.stderr)
        return "", -1


def scan_devices():
    """Сканирует доступные Bluetooth устройства"""
    print("Сканирование Bluetooth устройств (15 секунд)...\n")
    output, _ = run_command("bluetoothctl --timeout 15 scan on 2>/dev/null | grep -E '\\[NEW\\]|Device'")

    devices = {}
    for line in output.split('\n'):
        if 'Device' in line:
            parts = line.split()
            if len(parts) >= 2:
                addr = parts[1]
                name = ' '.join(parts[2:]) if len(parts) > 2 else "Unknown"
                devices[addr] = name

    if devices:
        print("Найденные устройства:")
        for addr, name in devices.items():
            print(f"  {addr}  {name}")
    else:
        print("Устройства не найдены. Убедитесь, что Bluetooth включён.")

    return devices


def list_paired():
    """Показывает список сопряжённых устройств"""
    output, _ = run_command("bluetoothctl paired-devices")

    devices = {}
    for line in output.split('\n'):
        if line.strip():
            parts = line.split()
            if len(parts) >= 2:
                addr = parts[1]
                name = ' '.join(parts[2:]) if len(parts) > 2 else "Unknown"
                devices[addr] = name

    if devices:
        print("Сопряжённые устройства:")
        for addr, name in devices.items():
            print(f"  {addr}  {name}")
    else:
        print("Сопряжённых устройств не найдено.")

    return devices


def pair_device(bd_addr):
    """Сопрягает устройство"""
    print(f"Сопряжение с {bd_addr}...")

    run_command(f"bluetoothctl trust {bd_addr}")
    output, code = run_command(f"bluetoothctl pair {bd_addr}")

    if code == 0 and "successful" in output.lower():
        print(f"✓ Успешно сопряжено")
        return True
    else:
        print(f"✗ Ошибка сопряжения: {output}")
        return False


def save_peer(peer_id, bd_addr):
    """Сохраняет адрес пирующего устройства в конфиг"""
    device_id = load_device_id()

    if "peers" not in device_id:
        device_id["peers"] = {}

    device_id["peers"][peer_id] = {
        "bd_addr": bd_addr,
        "added_at": __import__('datetime').datetime.now().isoformat()
    }

    save_device_id(device_id)
    print(f"✓ Адрес {peer_id} сохранён: {bd_addr}")
    print(f"  Файл: {DEVICE_ID_PATH}")


def show_config():
    """Показывает текущий конфиг устройства"""
    data = load_device_id()
    print(f"Конфиг устройства ({DEVICE_ID_PATH}):")
    print(json.dumps(data, indent=2, ensure_ascii=False))


def main():
    parser = ArgumentParser(description="Утилита для настройки Bluetooth соединения")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--scan", action="store_true", help="Сканировать Bluetooth устройства")
    group.add_argument("--list", action="store_true", help="Список сопряжённых устройств")
    group.add_argument("--pair", type=str, metavar="BD_ADDR", help="Сопрячь устройство (например, AA:BB:CC:DD:EE:FF)")
    group.add_argument("--save-peer", nargs=2, metavar=("PEER_ID", "BD_ADDR"), help="Сохранить адрес пира в конфиг")
    group.add_argument("--config", action="store_true", help="Показать конфиг устройства")

    args = parser.parse_args()

    if args.scan:
        scan_devices()
    elif args.list:
        list_paired()
    elif args.pair:
        pair_device(args.pair)
    elif args.save_peer:
        save_peer(args.save_peer[0], args.save_peer[1])
    elif args.config:
        show_config()


if __name__ == "__main__":
    main()
