#include <Arduino.h>
#include <Encoder.h>

void setup()
{
    encoder_init();
    Serial.begin(9600);
}

int frq = 200;
bool buz_on = 0, btn_flg = 0;

void loop()
{
    EncEvent ev = encoder_read();

    if (buz_on)
    {
        if (ev == ENC_LEFT)
            frq -= 10;
        else if (ev == ENC_RIGHT)
            frq += 10;
    }

    if (ev == ENC_BUTTON_PRESSED)
    {
        buz_on = !buz_on;
        Serial.println("BUTTON");
    }

    Serial.println(frq);
    delay(5);
}