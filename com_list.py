from serial.tools import list_ports

for p in list_ports.comports():
    vid = f"{p.vid:04X}" if p.vid is not None else "----"
    pid = f"{p.pid:04X}" if p.pid is not None else "----"
    print(f"{p.device:8}  VID:PID={vid}:{pid}  {p.description}")
