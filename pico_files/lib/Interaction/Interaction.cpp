#include <Arduino.h>
#include <FastLED.h>

const int LED_PIN = 20;
const int NUM_LEDS = 60;
const int led_ofst = 19;

#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

int led_old = 0;
void led_set(int index, uint8_t r, uint8_t g, uint8_t b, bool del_old)
{
    index = (index + 19) % NUM_LEDS;
    if (index < 0)
        index = 60 - abs(index);

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
    // FastLED для WS2812B
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);

    // Яркость (0..255). Для теста не ставь 255, чтобы не упереться в питание.
    FastLED.setBrightness(50);

    // Очистить
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    led_set(0, 255, 0, 0, 0);
    FastLED.show();
}

void strip_show() { FastLED.show(); }
void strip_clear(bool shw = 0)
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

void smile(bool anim = 1)
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