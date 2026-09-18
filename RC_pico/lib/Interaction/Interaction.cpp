#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Interaction.h"

const int OLED_SDA = 20; // GP20 = I2C0 SDA
const int OLED_SCL = 21; // GP21 = I2C0 SCL

const int OLED_W = 128;
const int OLED_H = 64;
const int OLED_ADDR = 0x3C;

const int led_pin = 18;

int led_br = 0;

// Ядро earlephilhower: MbedI2C нет, вместо него готовые объекты Wire (I2C0)
// и Wire1 (I2C1). GP20/GP21 - это линии аппаратного I2C0, поэтому Wire.
// Пины назначаются через setSDA/setSCL строго ДО Wire.begin().

void led_init()
{
    pinMode(led_pin, 1);
    led_br = 0;
}

void led_show()
{
    analogWrite(led_pin, led_br);
}

void led_set(int br){
    led_br = br;
}

void led_clear(){
    led_set(0);
}

void led_blink(float fq, int br)
{
    led_set(255 * (int(millis() / (1000 / fq / 2)) % 2));
}

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

void oled_init()
{
    Wire.setSDA(OLED_SDA);
    Wire.setSCL(OLED_SCL);
    Wire.begin();

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        while (true)
        {
            delay(100);
        }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.display();
}

void oled_clear()
{
    display.clearDisplay();
}

void oled_show()
{
    display.display();
}

void oled_text(int x, int y, const char *text, uint8_t size)
{
    display.setTextSize(size);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, y);
    display.print(text);
}

void oled_text(int x, int y, String text, uint8_t size)
{
    oled_text(x, y, text.c_str(), size);
}

void interaction_init()
{
    led_init();
    oled_init();
    oled_clear();

    oled_text(0, 0, "Hello", 1);
    oled_text(0, 16, "RP2350 OLED", 1);
    oled_text(0, 32, "X: 123", 2);

    oled_show();
}