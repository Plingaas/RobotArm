#include "Stepper.hpp"

Stepper::Stepper(short _stepPin, short _dirPin, short _limitSwitchPin, MultiPlexer& mp, bool _reverse, float _vmax, float _amax)
{
    dirPin = _dirPin;
    stepPin = _stepPin;
    limitSwitchPin = _limitSwitchPin;
    pinMode(_dirPin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    pinMode(_limitSwitchPin, INPUT_PULLUP);

    encoder.setMP(mp);
    encoder.setMPChannel(mp.getActiveChannelCount());

    reverse = _reverse;
    

    CW = !_reverse;
    CCW = _reverse;

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
    maxSpeed = _vmax;
    
    j = 2000;
    a = 0;
    v = 0;
    _s = 0;

    current_acc = 0;
    current_dec = 0;
    acceleration = 200;
    maxAcceleration = _amax;

    pid.params().set_params(0.001, 0, 1.0);
    vpid.params().set_params(0.001, 0.0, 0.0);
    apid.params().set_params(0.001, 0.0, 0.0);
}

void Stepper::setPins(short _stepPin, short _dirPin)
{
    stepPin = _stepPin;
    dirPin = _dirPin;
}

void Stepper::setReverse(bool _reverse)
{
    reverse = _reverse;
}

void Stepper::step()
{   

    if (readLimitSwitch() && direction == CCW) {
        //return;
    }

    digitalWrite(dirPin, direction);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(minPulseWidth);
    digitalWrite(stepPin, HIGH);

    steps_moved += 1;
    if (CCW == 1)   direction ? currentPosition -= 1 : currentPosition += 1;
    else            direction ? currentPosition += 1 : currentPosition -= 1;
}

void Stepper::setDirection(bool dir)
{
    direction = dir;
    digitalWrite(dirPin, direction);
    delayMicroseconds(5); // Minimum 5us between new dir value and pulse according to DM542T datasheet.
}

void Stepper::setTarget(float _steps) {
    target = (int)_steps;
}

int Stepper::angleToSteps(float angle) {
    return round(angle * steps_per_deg);
}

void Stepper::moveToJerk(short pos, float vmax, float amax, float total_time) {

    target = pos;
    short dist = pos - currentPosition;
    maxSpeed = vmax;
    maxAcceleration = amax;

    dist > 0 ? setDirection(CW) : setDirection(CCW);
    dist = abs(dist);
    total_steps = dist;
    steps_moved = 0;

    float t_vmax = sqrt(2*vmax / j);
    float t_amax = amax/j;
    
    float t1 = t_vmax < t_amax ? t_vmax : t_amax;
    float t2 = t_vmax < t_amax ? 0 : (vmax/amax) - t1;

    float s0_3 = 0.5*j*t1*t1*t1 + 0.5*j*t1*t1*t2 + 0.5*amax*(t1*t1 + t2*t2) + amax*t1*t2;
    
    float s_coast = dist - (2*s0_3);
    float t3 = s_coast/vmax;
    
    times = {t1, t1+t2, t1+t2+t1, t1+t2+t1+t3, t1+t2+t1+t3+t1, t1+t2+t1+t3+t1+t2, t1+t2+t1+t3+t1+t2+t1};
}

bool Stepper::run3(float dt) {
    
    t_total += dt;

    if (t_total < times[0]) {
        a += j * dt;
    }

    if (t_total >= times[0] && t_total < times[1]) {
        a = maxAcceleration;
    }

    if (t_total >= times[1] && t_total < times[2]) {
        a -= j * dt;
    }

    if (t_total >= times[2] && t_total < times[3]) {
        v = maxSpeed;
    }

    if (t_total >= times[3] && t_total < times[4]) {
        a -= j * dt;
    }

    if (t_total >= times[4] && t_total < times[5]) {
        a = -maxAcceleration;
    }

    if (t_total >= times[5] && t_total < times[6]) {
        a += j * dt;
    }

    v += a * dt;
    _s += abs(v * dt);

    if (shouldStep( (int) _s)) {
        step();
    }

    if (steps_moved == total_steps) {
        _s = 0;
        v = 0;
        a = 0;
        return true;
    }

    return false;
}

bool Stepper::run2(unsigned long elapsed) {
    float s = 0.0f;
    
    float j_vel = 0.5f*j*pow(times[0], 2);
    float a_vel = maxAcceleration*(times[1]-times[0]);
    float j1_steps = (1.0f/6.0f) * j * pow(times[0],3);
    float a_steps = 0.5f*maxAcceleration*pow(times[1]-times[0], 2) + j_vel*(times[1]-times[0]);
    float j2_steps = (1.0f/6.0f)*(-j)*pow(times[0],3) + 0.5*maxAcceleration*pow(times[0], 2) + (j_vel+a_vel)*times[0];
    float coast_steps = maxSpeed * (times[3]-times[2]);
    float j3_steps = (1.0f/6.0f)*(-j)*pow(times[0],3) + maxSpeed*times[0];
    float d_steps = 0.5f*(-maxAcceleration)*pow(times[1]-times[0], 2) + (maxSpeed-j_vel)*(times[1]-times[0]);
    float j4_steps = (1.0f/6.0f)*(j)*pow(times[0],3) + 0.5f*(-maxAcceleration)*pow(times[0], 2) + (j_vel)*times[0];

    if (elapsed < times[0]*1e6) {
        float t = elapsed * 1e-6;
        s = (1.0f/6.0f) * j * pow(t,3);
    } else {
        s += j1_steps;
    }

    if (elapsed >= times[0]*1e6 && elapsed < times[1]*1e6) {
        float t = (elapsed - times[0]*1e6) * 1e-6;
        s += 0.5*maxAcceleration*pow(t, 2) + j_vel*t;
    } else if (elapsed >= times[1]*1e6) {
        s += a_steps;
    }

    if (elapsed >= times[1]*1e6 && elapsed < times[2]*1e6) {
        float t = (elapsed - times[1]*1e6) * 1e-6;
        s += (1.0/6.0)*(-j)*pow(t,3) + 0.5*maxAcceleration*pow(t, 2) + (j_vel+a_vel)*t;
        
    } else if (elapsed >= times[2]*1e6)  {
        s += j2_steps;
    }

    if (elapsed >= times[2]*1e6 && elapsed < times[3]*1e6) {
        float t = (elapsed - times[2]*1e6) * 1e-6;
        s += maxSpeed * t;
    } else if (elapsed >= times[3]*1e6)  {
        s += coast_steps;
    }

    if (elapsed >= times[3]*1e6 && elapsed < times[4]*1e6) {
        float t = (elapsed - times[3]*1e6) * 1e-6;
        s += (1.0/6.0)*(-j)*pow(t,3) + maxSpeed*t;
    } else if (elapsed >= times[4]*1e6)  {
        s += j3_steps;
    }

    if (elapsed >= times[4]*1e6 && elapsed < times[5]*1e6) {
        float t = (elapsed - times[4]*1e6) * 1e-6;
        s += 0.5*(-maxAcceleration)*pow(t, 2) + (maxSpeed-j_vel)*t;
    } else if (elapsed >= times[5]*1e6)  {
        s += d_steps;
    }

    if (elapsed >= times[5]*1e6 && elapsed < times[6]*1e6) {
        float t = (elapsed - times[5]*1e6) * 1e-6;
        s += (1.0/6.0)*j*pow(t,3) + 0.5*(-maxAcceleration)*pow(t, 2) + j_vel*t;
    } else if (elapsed >= times[6]*1e6)  {
        s = total_steps;
    }
    

    if (shouldStep( (int) s)) {
        step();
    }

    if (steps_moved == total_steps) {
        return true;
    }

    return false;
}

void Stepper::moveToAngle(float angle, float total_time, float t_acc, float t_dec) {
    moveTo(angleToSteps(angle), total_time, t_acc, t_dec);
}

void Stepper::moveTo(short pos, float total_time, float t_acc, float t_dec) 
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

void Stepper::move(short rel_pos, float total_time, float t_acc_in, float t_acc_out) {
    moveTo(currentPosition+rel_pos, total_time, t_acc_in, t_acc_out);
}

bool Stepper::shouldStep(short steps) 
{   
    return (abs(steps) > abs(steps_moved) && abs(steps_moved) < total_steps);
}

bool Stepper::run(unsigned long elapsed) 
{
    int s = 0;
    unsigned long acc_time_us = acc_time*1e6;
    unsigned long vel_time_us = vel_time*1e6;
    //unsigned long dec_time_us = dec_time*1e6;
    
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
    }

    if (steps_moved == total_steps) {
        return true;
    }

    return false;
}

float Stepper::findAcceleration(short steps, float t, float t_acc, float t_dec) {    
    // Formula derived from s = 0.5at^2 + v_0*t for each of the three phases.
    return (2*steps) / (t_acc*(2*t - t_acc - t_dec));
}

void Stepper::reset() {
    current_v = 0;
    current_acc = 0;
    current_dec = 0;
    acc_steps = 0;
    dec_steps = 0;
}

void Stepper::update(float dt) {

    _s += speed * dt;
    float s_reg = pid.regulate(target, currentPosition, dt);
    float v_target = constrain(s_reg/dt, -maxSpeed, maxSpeed);
    acceleration = constrain((v_target - speed)/dt, -maxAcceleration, maxAcceleration);

    speed += acceleration * dt;
    speed = constrain(speed, -maxSpeed, maxSpeed);

    Serial.println(speed);

    if (direction == CW && speed < 0) {
        setDirection(CCW);
    }

    if (direction == CCW && speed > 0) {
        setDirection(CW);
    }

    if ((int) _s > currentPosition || (int) _s < currentPosition) {
        step();
    }
}

bool Stepper::readLimitSwitch() {
    return digitalRead(limitSwitchPin);
}