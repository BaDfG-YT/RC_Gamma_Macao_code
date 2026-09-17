#include <Arduino.h>

int kick_pin = 16;
int btn_pin = 12;

bool flg = 0;

void setup()
{
    pinMode(kick_pin, 1);
    pinMode(btn_pin, INPUT_PULLUP);
    digitalWrite(kick_pin, 0);
}

void kick()
{
    digitalWrite(kick_pin, 1);
    delay(20);
    digitalWrite(kick_pin, 0);
}

bool btnf()
{
    return !digitalRead(btn_pin);
}

void loop()
{
    if (btnf() && !flg)
    {
        kick();
        flg = 1;
    }
    if (!btnf())
        flg = 0;
}