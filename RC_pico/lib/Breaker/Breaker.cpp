#include "Breaker.h"
#include <Regulators.h>

static const int   PHOTO_TRANS_PIN  = 29;
static const int   BREAKER_THRESHOLD = 600;
static const float BREAKER_LPF_ALPHA = 0.2f;   // 0..1: lower = stronger smoothing

static LPF break_filter;
static bool breaker_initialized = false;

static void ensure_init()
{
    if (!breaker_initialized)
    {
        lpf_init(break_filter, BREAKER_LPF_ALPHA);
        breaker_initialized = true;
    }
}

void breaker_init()
{
    ensure_init();
}

int breaker_raw()
{
    return analogRead(PHOTO_TRANS_PIN);
}

int breaker_filtered()
{
    ensure_init();
    return (int)lround(lpf_push(break_filter, analogRead(PHOTO_TRANS_PIN)));
}

bool ball_hole()
{
    ensure_init();
    static bool state = false;
    float filt = lpf_push(break_filter, analogRead(PHOTO_TRANS_PIN));

    const float TH_ON  = 700.0f;   // above — ball present
    const float TH_OFF = 400.0f;   // below — no ball

    if (!state && filt >= TH_ON)
        state = true;
    else if (state && filt <= TH_OFF)
        state = false;

    return state;
}