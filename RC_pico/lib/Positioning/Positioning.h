#pragma once
#include <Arduino.h>

// === Красная цель (кегли) с камеры ===
// dx - отклонение центра масс красного от центра кадра по ширине, px
//      (минус = левее, плюс = правее)
extern float g_red_dx;
extern int g_red_area;
extern unsigned long g_last_red_ms;
extern bool g_red_visible;

void pos_init();
void pos_read_cam();
void procces_cam(char *line);