#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#include <Sharing.h>
#include <Positioning.h>

// =========================
// Camera: red target (кегли)
// Протокол с Pi 5: "RED,<dx>,<area>\n"
//   dx   - отклонение центра масс красного от центра кадра по ширине, px
//          (минус = цель левее центра, плюс = правее)
//   area - количество красных пикселей после фильтрации
// =========================

float g_red_dx = 0.0f;              // отклонение по ширине, px
int g_red_area = 0;                 // площадь красного
unsigned long g_last_red_ms = 0;    // millis() последнего пакета
bool g_red_visible = false;

// Pi шлёт dx=9999, если красного нет в кадре
static const float NO_TARGET_DX = 9999.0f;
static const float DX_VALID_ABS = 1000.0f;

// Сколько мс без пакетов считаем камеру потерянной
static const unsigned long CAM_TIMEOUT_MS = 500;

void procces_cam(char *line)
{
    char *cmd = strtok(line, ",");
    char *dxStr = strtok(nullptr, ",");
    char *areaStr = strtok(nullptr, ",");

    if (cmd == nullptr || dxStr == nullptr || areaStr == nullptr)
        return;

    if (strcmp(cmd, "RED") != 0)
        return;

    float dx = atof(dxStr);
    int area = atoi(areaStr);

    g_red_dx = dx;
    g_red_area = area;
    g_last_red_ms = millis();
    g_red_visible = (fabsf(dx) <= DX_VALID_ABS) && (area > 0);
}

void pos_read_cam()
{
    char line[64];

    while (share_read_line(line, sizeof(line)))
    {
        procces_cam(line);
    }

    // если пакеты перестали приходить - цель невалидна
    if (millis() - g_last_red_ms > CAM_TIMEOUT_MS)
        g_red_visible = false;
}

void pos_init()
{
    share_init();
}