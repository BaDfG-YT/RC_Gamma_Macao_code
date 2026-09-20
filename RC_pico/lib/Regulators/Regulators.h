#pragma once

#include <Arduino.h>

// =============================================================
// PD-регулятор
// =============================================================
//
// Использование:
//   PD pd_ball;
//   ...
//   float u = pd_reg(pd_ball, error, kp, kd);
//
// pd_reset(pd_ball);  // при потере цели, чтобы D-составляющая не дёрнула

struct PD
{
    float e_old;
    unsigned long t;
};

float pd_reg(PD &s, float e, float kp, float kd);
void  pd_reset(PD &s);

// =============================================================
// Скользящее окно (медиана / среднее)
// =============================================================
//
// Полезно для прерывателя (breaker), сигналов с шумом.
// MedianFilter — устойчив к одиночным выбросам, рекомендуется для bool-сенсоров.
// MeanFilter   — сглаживает плавно, рекомендуется для аналоговых.

static const int FILTER_MAX_WIN = 16;

struct MedianFilter
{
    int   buf[FILTER_MAX_WIN];
    int   win;     // размер окна (≤ FILTER_MAX_WIN)
    int   idx;     // куда писать следующее значение
    int   filled;  // сколько значений уже накопили (≤ win)
};

void  median_init(MedianFilter &f, int window);
int   median_push(MedianFilter &f, int value);

struct MeanFilter
{
    float buf[FILTER_MAX_WIN];
    int   win;
    int   idx;
    int   filled;
    float sum;
};

void  mean_init(MeanFilter &f, int window);
float mean_push(MeanFilter &f, float value);

// =============================================================
// Экспоненциальный фильтр (low-pass, по сути EMA)
// =============================================================
//
// alpha ∈ (0, 1]. Чем ближе к 1 — тем быстрее реакция, меньше сглаживания.
// Типичные значения: 0.1 — сильное сглаживание, 0.5 — лёгкое.

struct LPF
{
    float y;
    float alpha;
    bool  initialized;
};

void  lpf_init(LPF &f, float alpha);
float lpf_push(LPF &f, float x);

// =============================================================
// Дебаунсер для bool-сигналов (брейкер, кнопки)
// =============================================================
//
// Срабатывает только если сигнал держится в новом состоянии заданное время.
// Помогает против дребезга и одиночных шумовых импульсов.

struct Debouncer
{
    bool          state;        // текущее устойчивое состояние
    bool          last_raw;     // последнее сырое значение
    unsigned long t_change;     // когда сырое значение поменялось
    unsigned long stable_ms;    // сколько мс должно быть стабильно
};

void debounce_init(Debouncer &d, bool initial_state, unsigned long stable_ms);
bool debounce_push(Debouncer &d, bool raw);