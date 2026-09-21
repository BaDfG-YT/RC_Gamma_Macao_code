import json
import subprocess
import sys
import pathlib
import platform


def deep_merge(base: dict, overrides: dict) -> None:
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value


def load_config(name: str, folder: pathlib.Path) -> dict:
    with open(folder / f"{name}.defaults.json") as f:
        config = json.load(f)

    local_path = folder / f"{name}.local.json"
    if local_path.exists():
        with open(local_path) as f:
            deep_merge(config, json.load(f))

    return config


env_name = sys.argv[1]  # "main" или "troll"
root = pathlib.Path(__file__).parent.parent      # RC
pico_root = root / "RC_pico"                     # RC_pico — platformio.ini, robot_config
device_root = root / "device"                    # RC/device — device_id.defaults/.local.json

device = load_config("device_id", device_root)

print(f"[{device['id']}] Прошивка окружения '{env_name}'...")

home = pathlib.Path.home()
if platform.system() == "Windows":
    pio = home / ".platformio" / "penv" / "Scripts" / "pio.exe"
else:
    pio = home / ".platformio" / "penv" / "bin" / "pio"

subprocess.run([str(pio), "run", "-e", env_name], cwd=pico_root, check=True)

fw_local = pico_root / ".pio" / "build" / env_name / "firmware.elf"

if device["type"] == "robot":
    print(f"[{device['id']}] Локальная заливка через flash_pico.sh")
    subprocess.run(["bash", "-c", f'~/flash_pico.sh "{fw_local}"'], check=True)
else:
    host = device["target_host"]
    print(f"[{device['id']}] Отправка на {host} по SSH")
    subprocess.run(["scp", str(fw_local), f"bropi@{host}:/tmp/firmware.elf"], check=True)
    subprocess.run(["ssh", f"bropi@{host}", "~/flash_pico.sh /tmp/firmware.elf"], check=True)