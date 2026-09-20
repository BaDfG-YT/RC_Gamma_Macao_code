import json
import subprocess
import sys
import pathlib
import platform

env_name = sys.argv[1]  # "main" или "troll"
root = pathlib.Path(__file__).parent.parent

with open(root / "device_id.json") as f:
    device = json.load(f)

print(f"[{device['id']}] Прошивка окружения '{env_name}'...")

home = pathlib.Path.home()
if platform.system() == "Windows":
    pio = home / ".platformio" / "penv" / "Scripts" / "pio.exe"
else:
    pio = home / ".platformio" / "penv" / "bin" / "pio"

subprocess.run([str(pio), "run", "-e", env_name], cwd=root, check=True)

if device["type"] == "robot":
    print(f"[{device['id']}] Локальная заливка через flash_pico.sh")
    subprocess.run(["bash", "-c", "~/flash_pico.sh"], check=True)
else:
    host = device["target_host"]
    print(f"[{device['id']}] Отправка на {host} по SSH")
    fw = root / ".pio" / "build" / env_name / "firmware.uf2"
    subprocess.run(["scp", str(fw), f"bropi@{host}:/tmp/"], check=True)
    subprocess.run(["ssh", f"bropi@{host}", "~/flash_pico.sh"], check=True)