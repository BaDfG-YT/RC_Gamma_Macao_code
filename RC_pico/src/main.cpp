#include <Arduino.h>

void setup()
{
    pinMode(LED_BUILTIN, 1);
};

void loop()
{
    digitalWrite(LED_BUILTIN, (millis() / 2000) % 2);
};