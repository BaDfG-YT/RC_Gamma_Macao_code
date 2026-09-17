#pragma once
#include <Arduino.h>

void motor();
void motor_init();
void move(int32_t spA, int32_t spB, int32_t spC, int32_t spD);
void move_angle(float ang, int speed, int u);
void stop();