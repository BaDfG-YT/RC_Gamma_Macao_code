#include <Arduino.h>

// Test firmware: sends x, y, yaw over hardware UART to the Pi.
// Packet format: 3 floats (little-endian), 12 bytes total —
// matches the py script on the Pi (struct.unpack('<fff', data)).
//
// Wiring (check against your own pinout):
//   Pico GP0 (TX) -> Pi RX (pin 10, GPIO15)
//   Pico GP1 (RX) -> Pi TX (pin 8,  GPIO14)
//   Pico GND      -> Pi GND
// TX of one device always goes to RX of the other (crossed).

void setup() {
  Serial1.setTX(0);   // GP0 -> TX
  Serial1.setRX(1);   // GP1 -> RX
  Serial1.begin(115200);
}

void loop() {
  static float t = 0.0f;

  // Test values — replace with real x/y/yaw from your positioning logic
  float x   = sinf(t) * 100.0f;
  float y   = cosf(t) * 100.0f;
  float yaw = fmodf(t * 10.0f, 360.0f);

  uint8_t packet[12];
  memcpy(packet + 0, &x,   sizeof(float));
  memcpy(packet + 4, &y,   sizeof(float));
  memcpy(packet + 8, &yaw, sizeof(float));

  Serial1.write(packet, sizeof(packet));

  t += 0.05f;
  delay(50);
}