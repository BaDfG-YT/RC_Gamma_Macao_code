#include <Arduino.h>

// -------------------- EC11 on module (blue board, KY-040 style) --------------------
// Differences of the module from a "bare" EC11:
//  - the board has 10k pull-ups to VCC on CLK/DT (sometimes also an RC filter),
//    edges are slowed down - the old "check the pin after 300us" method missed steps;
//  - the module has one full quadrature cycle (4 transitions) per click,
//    while the bare EC11 from the old circuit produced 2 transitions per click -
//    hence "two clicks = one event".
// Solution: a full table-based quadrature decoder (x4) + a steps-per-click divider.

const int ENC_A = 4;
const int ENC_B = 2;
const int ENC_SW = 3;

// How many quadrature transitions correspond to one click (detent).
// KY-040/EC11 module: 4. If events start firing every other click - set to 2.
const int STEPS_PER_DETENT = 4;

enum EncEvent
{
    ENC_NONE = 0,

    ENC_RIGHT = 1,
    ENC_LEFT = 2,

    ENC_RIGHT_HOLD = 3,
    ENC_LEFT_HOLD = 4,

    ENC_BUTTON_PRESSED = 5,
    ENC_BUTTON_RELEASED = 6
};

// -------------------- state --------------------
static uint8_t prevState = 0;   // previous state (A<<1)|B
static int8_t stepAcc = 0;      // quadrature step accumulator

static bool btnStable = HIGH;   // confirmed button state
static bool btnLastRaw = HIGH;  // last raw reading
static unsigned long btnLastChange = 0;
const unsigned long BTN_DEBOUNCE_MS = 20;

// Quadrature transition table: index = (prev<<2)|curr,
// value = +1 (clockwise), -1 (counterclockwise), 0 (no movement/bounce).
static const int8_t QUAD_TABLE[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

static inline uint8_t readAB()
{
    return (uint8_t)((digitalRead(ENC_A) << 1) | digitalRead(ENC_B));
}

void encoder_init()
{
    // Keep pull-ups enabled: on CLK/DT they duplicate the on-board ones
    // (harmless), while on SW many modules have no pull-up of their own - there it's required.
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(ENC_SW, INPUT_PULLUP);

    prevState = readAB();
    stepAcc = 0;

    btnStable = digitalRead(ENC_SW);
    btnLastRaw = btnStable;
    btnLastChange = millis();
}

EncEvent encoder_read()
{
    // ---------- button: non-blocking debounce ----------
    bool raw = digitalRead(ENC_SW); // LOW = pressed

    if (raw != btnLastRaw)
    {
        btnLastRaw = raw;
        btnLastChange = millis();
    }
    else if (raw != btnStable && (millis() - btnLastChange) >= BTN_DEBOUNCE_MS)
    {
        btnStable = raw;
        return (btnStable == LOW) ? ENC_BUTTON_PRESSED : ENC_BUTTON_RELEASED;
    }

    // ---------- rotation: table-based quadrature decoder ----------
    uint8_t curr = readAB();

    if (curr != prevState)
    {
        int8_t dir = QUAD_TABLE[(prevState << 2) | curr];
        prevState = curr;

        if (dir != 0)
        {
            stepAcc += dir;

            // We only emit an event once a full click has accumulated
            // and the encoder is sitting in a detent (both channels HIGH for EC11 at rest) -
            // this filters out half-steps during slow rotation.
            if (stepAcc >= STEPS_PER_DETENT)
            {
                stepAcc = 0;
                bool hold = (btnStable == LOW);
                return hold ? ENC_RIGHT_HOLD : ENC_RIGHT;
            }
            if (stepAcc <= -STEPS_PER_DETENT)
            {
                stepAcc = 0;
                bool hold = (btnStable == LOW);
                return hold ? ENC_LEFT_HOLD : ENC_LEFT;
            }
        }
        else
        {
            // Invalid transition (skip/bounce) - reset the accumulator
            // so we don't count a phantom step.
            stepAcc = 0;
        }
    }

    return ENC_NONE;
}