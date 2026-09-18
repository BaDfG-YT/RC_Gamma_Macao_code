#pragma once
#include <Arduino.h>

void motor();
void motor_init();
void move(int32_t spA, int32_t spB);
void stop(bool hold = 1);
void obsession(int32_t sp);

struct EncoderState
{
    bool valid;
    uint16_t pos;     // позиция с учётом offset, 0..16383
    uint16_t raw;     // сырая позиция
    uint16_t offset;  // нулевое смещение
};

bool readEncoders(EncoderState &encA, EncoderState &encB);
bool readAngles(float &angA_deg, float &angB_deg);
bool readTurns(float &turnsA, float &turnsB);

// Колбэк прогресса поездки: пройдено A, пройдено B, цель (градусы)
typedef void (*MoveProgressCb)(float pa, float pb, float target);

bool move_deg(float deg, int speed, unsigned long timeout_ms = 25000,
              MoveProgressCb progress = nullptr);

bool move_mm(float mm, int speed, unsigned long timeout_ms = 25000,
              MoveProgressCb progress = nullptr);

// Источник ошибки для подруливания: возвращает текущую ошибку курса
// (>0 - робот должен довернуть в одну сторону, <0 - в другую)
typedef float (*ErrorCb)();

bool move_mm_pid(float mm, ErrorCb error_src, float kp, float ki, float kd, int speed,
                unsigned long timeout_ms = 20000, MoveProgressCb progress = nullptr);

bool turn_deg_tank(float angle, int speed, unsigned long timeout_ms = 25000,
              MoveProgressCb progress = nullptr);

bool turn_deg_pivot(float angle, int speed, bool dir = true,
                    unsigned long timeout_ms = 8000, MoveProgressCb progress = nullptr);