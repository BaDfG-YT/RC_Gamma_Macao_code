#pragma once

#include <Arduino.h>

void breaker_init();
bool ball_hole();
int  breaker_raw();       // raw value, for debugging
int  breaker_filtered();  // filtered value, for debugging