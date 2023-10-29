#ifndef ROBOTARMCONTROLLER_HPP
#define ROBOTARMCONTROLLER_HPP
#include <Arduino.h>
#include "Stepper.hpp"
#include "BSpline.hpp"
#include "Multiplexer.hpp"
#include "Encoder.hpp"
#include "PWMServo.h"

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
    MultiPlexer MP;
    Stepper actuators[6] = 
    {
        
        Stepper(0, 1, 24, MP, false, 500, 200),
        Stepper(3, 4, 25, MP, false, 500, 200),
        Stepper(6, 7, 26, MP, false, 500, 200),
        Stepper(10, 11, 27, MP, false, 500, 200),
        Stepper(2, 12, 28, MP, false, 500, 200),
        Stepper(8, 9, 29, MP, false, 500, 200)
    };

    int arduinoPin;
    
    float acc_time_percentage;
    float dec_time_percentage;
    unsigned short delay_after_move;
    float speed;
    float acceleration;
    bool useJerk;

    Vector3 target;
    
    void update(float dt);
    void setTarget(Vector3 _target);
    void goToWithSpeed(float x, float y, float z, float rx, float ry, float rz);
    void goToWithTime(float x, float y, float z, float move_time, const Matrix3& orientation = LOOKDOWN);
    void goToWithTime2(float x, float y, float z, float move_time, const Matrix3& orientation = LOOKDOWN);
    void goToHome(float t = 10);
    void goToRest(float t = 10);
    void goToAngles(float (&angles)[6]);
    void run();
    void run2();
    
    void calibrate();
    void writePosition();
    void splineMove(BSpline spline);

    void grip();
    void release();

};

#endif