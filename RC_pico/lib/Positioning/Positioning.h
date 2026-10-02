#pragma once
#include <Arduino.h>

// === Red target (pins) from the camera ===
// dx - deviation of the red centroid from the frame center along the width, px
//      (negative = left, positive = right)
extern float g_red_dx;
extern int g_red_area;
extern unsigned long g_last_red_ms;
extern bool g_red_visible;

void pos_init();
void pos_read_cam();
void procces_cam(char *line);