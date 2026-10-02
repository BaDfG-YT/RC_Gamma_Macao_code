#!/bin/bash
# Быстрый старт для теста Bluetooth соединения между двумя распаями

set -e

echo "=== Bluetooth тест — быстрый старт ==="
echo

DEVICE_ID_FILE="../device/device_id.local.json"

# Проверяем наличие device_id
if [ ! -f "$DEVICE_ID_FILE" ]; then
    echo "❌ Ошибка: $DEVICE_ID_FILE не найден"
    echo "Убедитесь, что файл device_id.local.json существует."
    exit 1
fi

# Получаем ID текущего устройства
MY_ID=$(python3 -c "import json; print(json.load(open('$DEVICE_ID_FILE'))['id'])")
echo "📱 Текущее устройство: $MY_ID"
echo

# Меню
echo "Что вы хотите сделать?"
echo "1) Сканировать Bluetooth устройства"
echo "2) Сопрячь устройство"
echo "3) Показать сопряжённые устройства"
echo "4) Сохранить адрес пирующего устройства"
echo "5) Запустить Bluetooth сервер"
echo "6) Запустить Bluetooth клиент"
echo "0) Выход"
echo

read -p "Выберите опцию [0-6]: " choice

case $choice in
    1)
        echo "Сканирование (это займёт 15 секунд)..."
        python3 bt_setup.py --scan
        ;;
    2)
        read -p "Введите Bluetooth адрес (AA:BB:CC:DD:EE:FF): " bd_addr
        python3 bt_setup.py --pair "$bd_addr"
        ;;
    3)
        python3 bt_setup.py --list
        ;;
    4)
        read -p "Введите ID пирующего устройства (robotA/robotB): " peer_id
        read -p "Введите его Bluetooth адрес (AA:BB:CC:DD:EE:FF): " bd_addr
        python3 bt_setup.py --save-peer "$peer_id" "$bd_addr"
        ;;
    5)
        read -p "Введите ID целевого устройства: " target_id
        echo "Запуск сервера (ожидание подключения от $target_id)..."
        echo "Нажмите Ctrl+C для отмены"
        python3 bt_test_auto.py --server "$target_id"
        ;;
    6)
        read -p "Введите ID целевого устройства: " target_id
        read -p "Введите адрес (Enter для загрузки из конфига): " bd_addr_input

        if [ -z "$bd_addr_input" ]; then
            echo "Запуск клиента (подключение к $target_id)..."
            python3 bt_test_auto.py --client "$target_id"
        else
            echo "Запуск клиента с явным адресом..."
            python3 bt_test_auto.py --client "$target_id" --force-addr "$bd_addr_input"
        fi
        ;;
    0)
        echo "До свидания!"
        ;;
    *)
        echo "❌ Неизвестная опция"
        exit 1
        ;;
esac

echo
echo "✓ Готово"
