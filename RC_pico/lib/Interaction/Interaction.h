#pragma once

#include <Arduino.h>

extern const int LED_PIN;
extern const int NUM_LEDS;
extern const int OLED_W;
extern const int OLED_H;
extern const int OLED_ADDR;

void interaction_init();

void strip_init();
void strip_show();
void strip_clear(bool shw = 0);
void strip_fill(uint8_t r, uint8_t g, uint8_t b);
void strip_range(int led_min, int led_max, uint8_t r, uint8_t g, uint8_t b);
void strip_loading();
void smile(bool anim = 1);

void led_set(int index, uint8_t r, uint8_t g, uint8_t b, bool del_old);
int  val_to_ledind(float val, int val_max);

void oled_init();
void oled_clear();
void oled_show();
void oled_text(int x, int y, const char *text, uint8_t size);
void oled_text(int x, int y, String text, uint8_t size);