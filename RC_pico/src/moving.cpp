#include <Arduino.h>
#include <MotorDriver.h>

// Simple start test for the MF4015 v2 motor (Waveshare RP2350-CAN)
// Spins motor A: 3 s forward -> 1 s stop -> 3 s reverse -> 1 s stop

int speed = 30; // start at low speed, max is 100

void setup()
{
  Serial.begin(115200);
  delay(2000); // time to open the Serial Monitor

  Serial.println("Motor test: init CAN...");
  motor_init(); // if XL2515 doesn't respond, it will hang inside - see comments in the driver
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