from picamera2 import Picamera2, Preview
from time import sleep

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration())

# Note: preview must be started before picam2.start()
picam2.start_preview(Preview.QTGL)   # requires a display server (X/Wayland)
picam2.start()

sleep(30)  # run for 30 seconds

picam2.stop()
picam2.stop_preview()
