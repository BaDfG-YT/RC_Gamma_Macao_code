#pragma once

#include <Arduino.h>

// =============================================================
// PD regulator
// =============================================================
//
// Usage:
//   PD pd_ball;
//   ...
//   float u = pd_reg(pd_ball, error, kp, kd);
//
// pd_reset(pd_ball);  // on target loss, so the D term doesn't jerk

struct PD
{
    float e_old;
    unsigned long t;
};

float pd_reg(PD &s, float e, float kp, float kd);
void  pd_reset(PD &s);

// =============================================================
// Sliding window (median / mean)
// =============================================================
//
// Useful for the breaker and other noisy signals.
// MedianFilter — robust to single outliers, recommended for bool sensors.
// MeanFilter   — smooths gradually, recommended for analog signals.

static const int FILTER_MAX_WIN = 16;

struct MedianFilter
{
    int   buf[FILTER_MAX_WIN];
    int   win;     // window size (≤ FILTER_MAX_WIN)
    int   idx;     // where to write the next value
    int   filled;  // how many values have been accumulated so far (≤ win)
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
// Exponential filter (low-pass, effectively an EMA)
// =============================================================
//
// alpha ∈ (0, 1]. The closer to 1 — the faster the response, less smoothing.
// Typical values: 0.1 — strong smoothing, 0.5 — light.

struct LPF
{
    float y;
    float alpha;
    bool  initialized;
};

void  lpf_init(LPF &f, float alpha);
float lpf_push(LPF &f, float x);

// =============================================================
// Debouncer for bool signals (breaker, buttons)
// =============================================================
//
// Only triggers if the signal holds the new state for the given time.
// Helps against bounce and single noise spikes.

struct Debouncer
{
    bool          state;        // current stable state
    bool          last_raw;     // last raw value
    unsigned long t_change;     // when the raw value last changed
    unsigned long stable_ms;    // how many ms it must stay stable
};

void debounce_init(Debouncer &d, bool initial_state, unsigned long stable_ms);
bool debounce_push(Debouncer &d, bool raw);