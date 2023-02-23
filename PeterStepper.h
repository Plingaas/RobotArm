#ifndef PeterStepper_h
#define PeterStepper_h
#include "InverseKinematics.h"
#include <Arduino.h>

class PeterStepper{
    
    public:
        PeterStepper(short _stepPin, short _dirPin, bool _reverse);
        short dirPin;
        short stepPin;
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
        int speed;
        int maxSpeed;
        
        float current_acc;
        float current_dec;
        int acceleration;
        int maxAcceleration;
        
        bool useJerk;
        int jerk;

        void setPins(short stepPin, short dirPin);
        void setReverse(bool reverse);
        void step();
        bool shouldStep(short steps);
        void setDirection(bool dir);
        bool run(unsigned long elapsed);
        void moveTo(short abs_pos, float total_time, float t_acc_in, float t_acc_out);
        void move(short rel_pos, float total_time, float t_acc_in, float t_acc_out);
        float findAcceleration(short steps, float t, float t_in, float t_out);
        void reset();
    private:
        
};

#endif