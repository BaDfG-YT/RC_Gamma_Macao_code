#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>

Servo dribbler;

void drib_set(int sp)
{
    sp = constrain(sp, 0, 100);
    dribbler.write(constrain(map(sp, 0, 100, 1100, 1500), 1100 , 1500));
}

void drib_stop()
{
    drib_set(0);
}

void drib_init()
{   
    dribbler.attach(18);
    // drib_stop();
    delay(1000);
    dribbler.write(1000);
}