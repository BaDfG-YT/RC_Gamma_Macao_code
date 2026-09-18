#include "Regulators.h"

// =============================================================
// PID
// =============================================================

float pid_reg(PID &s, float e, float kp, float ki, float kd, float i_limit)
{
    unsigned long dt = millis() - s.t;

    // интегрируем и обновляем D только на "тиках" (не чаще раза в 3 мс) -
    // та же логика квантования, что была в старом pd_reg
    if (dt >= 3)
    {
        s.integral += e * dt;   // накопление в единицах "ошибка * мс"
        s.integral = constrain(s.integral, -i_limit, i_limit);
    }

    float u = e * kp + s.integral * ki + (e - s.e_old) * kd;

    if (dt >= 3)
        s.e_old = e;
    s.t = millis();

    return u;
}

void pid_reset(PID &s)
{
    s.e_old = 0.0f;
    s.integral = 0.0f;
    s.t = millis();
}

// =============================================================
// Медианный фильтр
// =============================================================

void median_init(MedianFilter &f, int window)
{
    if (window < 1) window = 1;
    if (window > FILTER_MAX_WIN) window = FILTER_MAX_WIN;
    f.win = window;
    f.idx = 0;
    f.filled = 0;
}

int median_push(MedianFilter &f, int value)
{
    f.buf[f.idx] = value;
    f.idx = (f.idx + 1) % f.win;
    if (f.filled < f.win) f.filled++;

    int tmp[FILTER_MAX_WIN];
    for (int i = 0; i < f.filled; i++)
        tmp[i] = f.buf[i];

    for (int i = 1; i < f.filled; i++)
    {
        int key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key)
        {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    return tmp[f.filled / 2];
}

// =============================================================
// Среднее по окну
// =============================================================

void mean_init(MeanFilter &f, int window)
{
    if (window < 1) window = 1;
    if (window > FILTER_MAX_WIN) window = FILTER_MAX_WIN;
    f.win = window;
    f.idx = 0;
    f.filled = 0;
    f.sum = 0.0f;
    for (int i = 0; i < FILTER_MAX_WIN; i++) f.buf[i] = 0.0f;
}

float mean_push(MeanFilter &f, float value)
{
    if (f.filled < f.win)
    {
        f.buf[f.idx] = value;
        f.sum += value;
        f.idx = (f.idx + 1) % f.win;
        f.filled++;
    }
    else
    {
        f.sum -= f.buf[f.idx];
        f.buf[f.idx] = value;
        f.sum += value;
        f.idx = (f.idx + 1) % f.win;
    }
    return f.sum / f.filled;
}

// =============================================================
// LPF (EMA)
// =============================================================

void lpf_init(LPF &f, float alpha)
{
    if (alpha <= 0.0f) alpha = 0.01f;
    if (alpha > 1.0f)  alpha = 1.0f;
    f.alpha = alpha;
    f.y = 0.0f;
    f.initialized = false;
}

float lpf_push(LPF &f, float x)
{
    if (!f.initialized)
    {
        f.y = x;
        f.initialized = true;
    }
    else
    {
        f.y = f.alpha * x + (1.0f - f.alpha) * f.y;
    }
    return f.y;
}

// =============================================================
// Debouncer
// =============================================================

void debounce_init(Debouncer &d, bool initial_state, unsigned long stable_ms)
{
    d.state = initial_state;
    d.last_raw = initial_state;
    d.t_change = millis();
    d.stable_ms = stable_ms;
}

bool debounce_push(Debouncer &d, bool raw)
{
    if (raw != d.last_raw)
    {
        d.last_raw = raw;
        d.t_change = millis();
    }
    else
    {
        if (raw != d.state && (millis() - d.t_change) >= d.stable_ms)
            d.state = raw;
    }
    return d.state;
}