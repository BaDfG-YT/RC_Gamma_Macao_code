#pragma once

#include <Arduino.h>

void breaker_init();
bool ball_hole();
int  breaker_raw();       // сырое значение, для отладки
int  breaker_filtered();  // отфильтрованное значение, для отладки