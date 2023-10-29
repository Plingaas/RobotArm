#ifndef STEPPER_HPP
#define STEPPER_HPP
#include <Arduino.h>
#include "IK.hpp"
#include "PID.hpp"
#include "MultiPlexer.hpp"
#include "Encoder.hpp"

#include <vector>

class Stepper {
    
    public:
        Stepper(short _stepPin, short _dirPin, short _limitSwitchPin, MultiPlexer& _mp, bool _reverse, float _vmax, float _amax);
        
        short dirPin;
        short stepPin;
        short limitSwitchPin;
        bool reverse;

        bool CW;
        bool CCW;
        bool direction;
        int minPulseWidth;

        int currentPosition;

        short total_steps;
        short acc_steps;
        float acc_time;
        short vel_steps;
        float vel_time;
        short dec_steps;
        float dec_time;

        unsigned long pt;

        short steps_moved;

        float current_v;
        float speed;
        float maxSpeed;
        
        float current_acc;
        float current_dec;
        float acceleration;
        float maxAcceleration;
        
        float _s;
        float v;
        float a;
        float j;
        float t_total;
        
        float lower_limit;
        float upper_limit;
        float steps_per_deg;

        unsigned long lastStepTime;

        
        int target;
        int current;
        std::vector<float> times;
        void update(float dt);
        void setTarget(float _steps);

        void setPins(short stepPin, short dirPin);
        void setReverse(bool reverse);
        void setDirection(bool dir);
        
        bool shouldStep(short steps);
        void step();

        bool run(unsigned long elapsed);
        bool run2(unsigned long elapsed);
        bool run3(float dt);

        bool runPID(float dt);

        void move(short rel_pos, float total_time, float t_acc_in, float t_acc_out);
        void moveTo(short abs_pos, float total_time, float t_acc_in, float t_acc_out);
        void moveToJerk(short pos, float maxVel, float maxAcc, float total_time);
        void moveToAngle(float angle, float total_time, float t_acc, float t_dec);
        int angleToSteps(float angle);

        float findAcceleration(short steps, float t, float t_in, float t_out);
        void reset();

        bool readLimitSwitch();

        Encoder encoder;
        PID pid;
        PID vpid;
        PID apid;
        PID jpid;
    
};

#endif