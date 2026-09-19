from picamera2 import Picamera2, Preview
from time import sleep

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())

# �����: preview ��������� �� picam2.start()
picam2.start_preview(Preview.QTGL)   # ���� ���� ������� ���� (X/Wayland)
picam2.start()

sleep(30)  # ���������� 30 ���

picam2.stop()
picam2.stop_preview()