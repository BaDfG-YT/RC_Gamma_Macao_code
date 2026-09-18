#pragma once

#include <Arduino.h>

extern const int LED_PIN;
extern const int NUM_LEDS;
extern const int OLED_W;
extern const int OLED_H;
extern const int OLED_ADDR;

void interaction_init();
void oled_init();


void oled_init();
void oled_clear();
void oled_show();
void oled_text(int x, int y, const char *text, uint8_t size);
void oled_text(int x, int y, String text, uint8_t size);

void led_set(int br);
void led_clear();
void led_blink(float fq, int br);
void led_show();