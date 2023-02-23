#include "PeterStepper.h"
#include <Arduino.h>

PeterStepper::PeterStepper(short _stepPin, short _dirPin, bool _reverse)
{
    dirPin = _dirPin;
    stepPin = _stepPin;
    pinMode(_dirPin, OUTPUT);
    pinMode(_stepPin, OUTPUT);

    reverse = _reverse;

    if (_reverse) 
    {
        CW = 1;
        CCW = 0;
    } 
    else 
    {
        CW = 0;
        CCW = 1;
    }

    direction = CW;
    minPulseWidth = 25;

    currentPosition = 0;

    total_steps = 0;
    acc_steps = 0;
    acc_time = 0;
    vel_steps = 0;
    vel_time = 0;
    dec_steps = 0;
    dec_time = 0;

    steps_moved = 0;

    pt = 0;

    current_v = 0.0;
    speed = 200;
    maxSpeed = 1000;
    
    current_acc = 0;
    current_dec = 0;
    acceleration = 200;
    maxAcceleration = 5000;
    
    useJerk = true;
    jerk = 100;
}

void PeterStepper::setPins(short _stepPin, short _dirPin)
{
    stepPin = _stepPin;
    dirPin = _dirPin;
}

void PeterStepper::setReverse(bool _reverse)
{
    reverse = _reverse;
}

void PeterStepper::step()
{   
    digitalWrite(dirPin, direction);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(minPulseWidth);
    digitalWrite(stepPin, HIGH);

    if (CCW == 1)   direction ? currentPosition -= 1 : currentPosition += 1;
    else            direction ? currentPosition += 1 : currentPosition -= 1;
}

void PeterStepper::setDirection(bool dir)
{
    direction = dir;
    digitalWrite(dirPin, direction);
    delayMicroseconds(5); // Minimum 5us between new dir value and pulse according to DM542T datasheet.
}

void PeterStepper::moveTo(short pos, float total_time, float t_acc, float t_dec) 
{   

    if (currentPosition == pos) {
        return;
    }

    short steps_to_move = pos - currentPosition;

    steps_to_move > 0 ? setDirection(CW) : setDirection(CCW);
    steps_to_move = abs(steps_to_move);
    total_steps = steps_to_move;

    steps_moved = 0;

    // Turns percentages of time into seconds
    t_acc *= total_time;
    t_dec *= total_time;

    // Get acceleration
    current_acc = findAcceleration(steps_to_move, total_time, t_acc, t_dec);
    current_dec = (-current_acc*t_acc)/(t_dec);
    // Set steps to take
    acc_steps = 0.5*current_acc*t_acc*t_acc;
    acc_time = t_acc;

    dec_steps = 0.5*current_acc*t_acc*t_dec;
    dec_time = t_dec;

    vel_steps = steps_to_move - acc_steps - dec_steps;
    vel_time = total_time - t_acc - t_dec;

}

void PeterStepper::move(short rel_pos, float total_time, float t_acc_in, float t_acc_out) {
    moveTo(currentPosition+rel_pos, total_time, t_acc_in, t_acc_out);
}

bool PeterStepper::shouldStep(short steps) 
{   
    return abs(steps) > abs(steps_moved) && abs(steps_moved) < total_steps;
}

bool PeterStepper::run(unsigned long elapsed) 
{
    int s = 0;
    unsigned long acc_time_us = acc_time*1e6;
    unsigned long vel_time_us = vel_time*1e6;
    unsigned long dec_time_us = dec_time*1e6;
    
    // Acceleration phase
    if (elapsed <= acc_time_us) {

        float t = elapsed*1.0e-6;
        s = 0.5*current_acc*t*t;
    }
    
    // Constant velocity phase
    else if (elapsed - acc_time_us < vel_time_us) {

        float t = (elapsed-acc_time_us)*1e-6;
        float v = current_acc*acc_time;
        s = acc_steps + v*t;

    }
    
    // Deceleration phase
    else
    
    {

        float t = (elapsed-acc_time_us-vel_time_us)*1e-6;
        float v = current_acc*acc_time;

        // Rounding to include the last step
        s = acc_steps + vel_steps + 0.5*current_dec*t*t + v*t;
    }

    if (shouldStep(s)) {
        step();
        steps_moved += 1;
    }

    if (steps_moved == total_steps) {
        return true;
    }

    return false;
}

float PeterStepper::findAcceleration(short steps, float t, float t_acc, float t_dec) 
{    
    // Formula derived from s = 0.5at^2 + v_0*t for each of the three phases.
    return (2*steps) / (t_acc*(2*t - t_acc - t_dec));
}

void PeterStepper::reset() 
{
    current_v = 0;
    current_acc = 0;
    current_dec = 0;
    acc_steps = 0;
    dec_steps = 0;
}