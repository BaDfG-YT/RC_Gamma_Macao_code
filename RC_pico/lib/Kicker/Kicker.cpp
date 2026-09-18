#include <Arduino.h>

const int kick_pin = 17;

void kick_init()
{
    pinMode(kick_pin, 1);
    digitalWrite(kick_pin, 0);
}

long kick_t = 0;
bool kick_f = 0;
void kick()
{
    if ((millis() - kick_t > 1000) && !kick_f)
    {
        digitalWrite(kick_pin, 1);
        delay(15);
        digitalWrite(kick_pin, 0);
        kick_f = 1;
    }
    if (kick_f)
    {
        kick_t = millis();
        kick_f = 0;
    }
}

void discharge()
{
    for (int i = 0; i < 5; i++)
    {
        kick();
        delay(1500 - i * 300);
    }
}