#ifndef InverseKinematics_h
#define InverseKinematics_h
#include "math.h"
// LINK LENGTHS
const float a1 = 169.77; 
const float a2 = 64.2;
const float a3 = 305;
const float a4 = 222.63;
const float a5 = 36.25;

// LINK MAX ROTATION LIMITS
const short J1_LOWER_LIMIT = -170;
const short J1_UPPER_LIMIT = 170;

const short J2_LOWER_LIMIT = 0;
const short J2_UPPER_LIMIT = 132;

const short J3_LOWER_LIMIT = -140;
const short J3_UPPER_LIMIT = 1;

const short J4_LOWER_LIMIT = -180;
const short J4_UPPER_LIMIT = 180;

const short J5_LOWER_LIMIT = -135;
const short J5_UPPER_LIMIT = 135;

const short J6_LOWER_LIMIT = -180;
const short J6_UPPER_LIMIT = 180;

// CONSTANTS
const float TO_DEGREES = 180/M_PI;
const float J1_STEPS_PER_DEG = 88.88888889;
const float J2_STEPS_PER_DEG = 55.55555567;
const float J3_STEPS_PER_DEG = 55.55555567;
const float J4_STEPS_PER_DEG = 0;
const float J5_STEPS_PER_DEG = 0;
const float J6_STEPS_PER_DEG = 0;

struct Angles
{
    float theta1;
    float theta2;
    float theta3;
    bool legal;
};

Angles IK(float x, float y, float z);
bool withinWorkarea(Angles);

#endif