#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Interaction.h"

const int OLED_SDA = 20;
const int OLED_SCL = 21;

const int OLED_W = 128;
const int OLED_H = 64;
const int OLED_ADDR = 0x3C;

const int LED_PIN = 6;
const int NUM_LEDS = 60;

// cannot be named i2c0 / i2c1 — these are macros from pico-sdk
arduino::MbedI2C oledI2C(OLED_SDA, OLED_SCL);

Adafruit_SSD1306 display(OLED_W, OLED_H, &oledI2C, -1);

#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

int led_old = 0;
void led_set(int index, uint8_t r, uint8_t g, uint8_t b, bool del_old)
{
    index = (index + 19) % NUM_LEDS;
    if (index < 0)
        index = NUM_LEDS - abs(index);

    leds[index] = CRGB(r, g, b);

    if (del_old)
    {
        if (led_old != index)
            leds[led_old] = CRGB(0, 0, 0);
        led_old = index;
    }
}

void strip_init()
{
    // FastLED for WS2812B
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);

    // Brightness (0..255). For testing don't set 255, to avoid overloading the power supply.
    FastLED.setBrightness(50);

    // Clear
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    led_set(0, 255, 0, 0, 0);
    FastLED.show();
}

void strip_show() { FastLED.show(); }
void strip_clear(bool shw)
{
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    if (shw)
    {
        strip_show();
    }
}

void strip_fill(uint8_t r, uint8_t g, uint8_t b)
{
    fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
}

// int t_blink;
// void blink(uint8_t r, uint8_t g, uint8_t b)
// {
//     bool st = 0;
//     if (!st)
//     {
//         t_blink = millis();
//         st = 1;
//     }
//     int dt = millis() - t_blink;
//     if (dt / 1000 % 2)
//         strip_fill(r, g, b);
//     else
//         strip_clear();
// }

void strip_range(int led_min, int led_max, uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = led_min; i <= led_max; i++)
    {
        led_set(i, r, g, b, 0);
    }
}

int val_to_ledind(float val, int val_max)
{
    int ind = lround(val * NUM_LEDS / val_max);
    return ind;
}

void strip_loading()
{
    led_set(millis() / 50 % NUM_LEDS, 255, 0, 0, 0);
}

void smile(bool anim)
{
    strip_clear(1);
    strip_range(5, 6, 200, 255, 0);
    strip_range(54, 55, 200, 255, 0);
    strip_show();

    if (anim)
    {
        for (int i = 1; i < 11; i++)
        {
            strip_range(29 - i, 28 + i, 200, 255, 0);
            strip_show();
            delay(100);
        }
    }

    strip_range(20, 39, 200, 255, 0);
    strip_show();
}

void oled_init()
{
    oledI2C.begin();

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
    strip_init();
    oled_init();
    oled_clear();

    oled_text(0, 0, "Hello", 1);
    oled_text(0, 16, "Pico OLED", 1);
    oled_text(0, 32, "X: 123", 2);

    oled_show();
}