#include <Arduino.h>

// Тестовая прошивка: отправка x, y, yaw по аппаратному UART на Pi.
// Формат пакета: 3 float (little-endian), итого 12 байт —
// совпадает с py-скриптом на Pi (struct.unpack('<fff', data)).
//
// Подключение (проверьте под свою распиновку):
//   Pico GP0 (TX) -> Pi RX (пин 10, GPIO15)
//   Pico GP1 (RX) -> Pi TX (пин 8,  GPIO14)
//   Pico GND      -> Pi GND
// TX одного устройства всегда идёт на RX другого (крест-накрест).

void setup() {
  Serial1.setTX(0);   // GP0 -> TX
  Serial1.setRX(1);   // GP1 -> RX
  Serial1.begin(115200);
}

void loop() {
  static float t = 0.0f;

  // Тестовые значения — заменить на реальные x/y/yaw из вашей логики позиционирования
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