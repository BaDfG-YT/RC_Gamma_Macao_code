#pragma once
#include <Arduino.h>

extern const int kick_pin;

void kick_init();
void kick();
void discharge();