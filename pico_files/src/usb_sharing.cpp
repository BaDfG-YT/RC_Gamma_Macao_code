#include <Arduino.h>

struct Packet {
  float x;
  float y;
  float yaw;
};

void setup() {
  Serial.begin(115200);
  delay(2000);
}

void loop() {
  Packet p;
  p.x = 1.23f;
  p.y = 4.56f;
  p.yaw = 78.9f;

  Serial.write(reinterpret_cast<const uint8_t*>(&p), sizeof(p));
  delay(100);
}