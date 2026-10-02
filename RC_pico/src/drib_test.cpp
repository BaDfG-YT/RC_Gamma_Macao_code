#include <Arduino.h>
#include <Dribbler.h>

void setup()
{
    pinMode(LED_BUILTIN, 1);
};

void loop()
{
    digitalWrite(LED_BUILTIN, (millis() / 2000) % 2);
};

// not finished