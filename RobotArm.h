#ifndef RobotArm_h
#define RobotArm_h
#include <Arduino.h>
#include "PeterStepper.h"
#include "InverseKinematics.h"

struct Position 
{
    float x = 0;
    float y = 0;
    float z = 0;
    float rx = 0;
    float ry = 0;
    float rz = 0;
};

class RobotArm {
public:
    RobotArm();

    PeterStepper actuators[6] = 
    {
        PeterStepper(4, 5, false),
        PeterStepper(6, 7, true),
        PeterStepper(8, 9, true),
        PeterStepper(6, 7, false),
        PeterStepper(8, 9, false),
        PeterStepper(10, 11, false)
    };

    float acc_time_percentage;
    float dec_time_percentage;
    unsigned short delay_after_move;
    float speed;
    float acceleration;

    void setStepperPins(short index, short stepPin, short dirPin);
    void setStepperReverse(short index, bool _reverse);
    Angles solveIK(float x, float y, float z);
    
    void goToWithSpeed(float x, float y, float z, float rx, float ry, float rz);
    void goToWithTime(float x, float y, float z, float move_time);
    void goToHome();
    void goToRest();
    void run();
  
};

#endif