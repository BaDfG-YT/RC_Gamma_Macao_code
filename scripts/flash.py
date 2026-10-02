import subprocess
import sys
import pathlib
import platform

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from config_utils import load_config

env_name = sys.argv[1]  # "main" or "troll"
root = pathlib.Path(__file__).parent.parent      # RC
pico_root = root / "RC_pico"                     # RC_pico — platformio.ini, config
device_root = root / "device"                    # RC/device — device_id.defaults/.local.json

device = load_config("device_id", device_root)

print(f"[{device['id']}] Flashing environment '{env_name}'...")

home = pathlib.Path.home()
if platform.system() == "Windows":
    pio = home / ".platformio" / "penv" / "Scripts" / "pio.exe"
else:
    pio = home / ".platformio" / "penv" / "bin" / "pio"

subprocess.run([str(pio), "run", "-e", env_name], cwd=pico_root, check=True)

fw_local = pico_root / ".pio" / "build" / env_name / "firmware.elf"

if device["type"] == "robot":
    print(f"[{device['id']}] Local flashing via flash_pico.sh")
    subprocess.run(["bash", "-c", f'~/flash_pico.sh "{fw_local}"'], check=True)
else:
    host = device["target_host"]
    print(f"[{device['id']}] Sending to {host} over SSH")
    subprocess.run(["scp", str(fw_local), f"bropi@{host}:/tmp/firmware.elf"], check=True)
    subprocess.run(["ssh", f"bropi@{host}", "~/flash_pico.sh /tmp/firmware.elf"], check=True)