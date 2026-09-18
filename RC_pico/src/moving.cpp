#include <Arduino.h>
#include <MotorDriver.h>

// Простой тест запуска мотора MF4015 v2 (Waveshare RP2350-CAN)
// Крутит мотор A: 3 c вперёд -> 1 c стоп -> 3 c назад -> 1 c стоп

int speed = 30; // старт на малой скорости, максимум 100

void setup()
{
  Serial.begin(115200);
  delay(2000); // время открыть Serial Monitor

  Serial.println("Motor test: init CAN...");
  motor_init(); // если XL2515 не ответит - зависнет внутри, см. комментарии в драйвере
  Serial.println("CAN OK, motors enabled");

  for (int i = 0; i < 5; i++)
  {
    stop();
    delay(2);
  }
}

void loop()
{
  long t = millis() / 1000 % 8;

  if (t < 3)
    move(speed, speed, speed, speed);
  else if (t < 4)
    stop();
  else if (t < 7)
    move(-speed, -speed, -speed, -speed);
  else
    stop();

  delay(2);
}