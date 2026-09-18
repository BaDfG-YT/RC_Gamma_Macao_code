#pragma once
#include <Arduino.h>

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
void encoder_init();
EncEvent encoder_read();