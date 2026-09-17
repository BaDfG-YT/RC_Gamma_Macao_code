#pragma once
#include <Arduino.h>
#include <FastLED.h>

extern const int LED_PIN = 20;
extern const int NUM_LEDS = 60;
extern CRGB leds[NUM_LEDS];

void strip_init();

void led_set(int index, uint8_t r, uint8_t g, uint8_t b, bool del_old);
void strip_fill(uint8_t r, uint8_t g, uint8_t b);

void strip_clear(bool shw = 0);
void strip_show();

int val_to_ledind(float val, int val_max);

void strip_loading();
void smile(bool anim = 1);