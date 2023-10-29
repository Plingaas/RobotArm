#ifndef IK2_HPP
#define IK2_HPP

#include <math.h>
#include "math.hpp"

// LINK LENGTHS
const float a1 = 169.77f;
const float a2 = 64.2f;
const float a3 = 305.0f;
const float a4 = 222.63f;
const float a5 = 36.25f;

const float a6 = 64.0f; // End effector length
const float a7 = 0.0f; // End effector pinch offset;

// LINK MAX ROTATION LIMITS
const float J1_LOWER_LIMIT = -2.96705972839f; // -170 deg
const float J1_UPPER_LIMIT = 2.96705972839f; // 170 deg

const float J2_LOWER_LIMIT = -0.87266462599f; // 50 deg
const float J2_UPPER_LIMIT = 2.30383461263f; // 132 deg

const float J3_LOWER_LIMIT = -2.44346095279f; // -140 deg
const float J3_UPPER_LIMIT = 0.01745329251f; // 1 deg

const float J4_LOWER_LIMIT = -PI; // -180 deg
const float J4_UPPER_LIMIT = PI; // 180 deg

const float J5_LOWER_LIMIT = -2.35619449019f; // -135 deg
const float J5_UPPER_LIMIT = 2.35619449019f; // 135 deg

const float J6_LOWER_LIMIT = -PI; // -180 deg
const float J6_UPPER_LIMIT = PI; // 180 deg

// CONSTANTS
const float TO_DEGREES = 180.0f / PI;
const float TO_RADIANS = PI / 180.0f;

// Steps per revolution
const int J1_SPR = 400; 
const int J2_SPR = 400; 
const int J3_SPR = 400; 
const int J4_SPR = 400; 
const int J5_SPR = 800; 
const int J6_SPR = 400; 

// Gear ratio + pulley
const float J1_GR = 20 * (60 / 15);
const float J2_GR = 50;
const float J3_GR = 50;
const float J4_GR = 14 * (28 / 10); // gearbox 14, input pulley 10 teeth, output pulley 28 teeth.
const float J5_GR = 9.525; // 1 rev of motor = 8mm move = (8/5.08) teeth move. Output pulley is 15 teeth thus 15/(8/5.08) = 9.525 is gear ratio.
const float J6_GR = 20 / 1;

// Stepper motor steps per degree rotation.
const float J1_STEPS_PER_DEG = 88.88888889;
const float J2_STEPS_PER_DEG = 55.55555567;
const float J3_STEPS_PER_DEG = 55.55555567;
const float J4_STEPS_PER_DEG = 43.55555555;
const float J5_STEPS_PER_DEG = 21.16666667; // Unsure? 1/4 steps -> 800 steps per rev. 1 rev = 8mm extension. Pulley diameter is 28mm, thus circumference is 87.96mm. 800 steps = 8/87.96 revolutions -> 8796 steps/rev -> 24.33 steps/deg.
const float J6_STEPS_PER_DEG = 21.11111111;

// PROJECTION MATRIXES AT REST POSITION
static const Matrix3 P0_1
        {
                1, 0, 0,
                0, 0, -1,
                0, 1, 0
        };

// Projection of joint 2 on joint 1
static const Matrix3 P1_2
        {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
        };

// Projection of joint 3 on joint 2
static const Matrix3 P2_3
        {
                0, 0, -1,
                1, 0, 0,
                0, -1, 0
        };

// Projection of joint 4 on joint 3
static const Matrix3 P3_4
        {
                1, 0, 0,
                0, 0, -1,
                0, 1, 0
        };

// Projection of joint 5 on joint 4
static const Matrix3 P4_5
        {
                1, 0, 0,
                0, 0, 1,
                0, -1, 0
        };

// Projection of joint 6 on joint 5
static const Matrix3 P5_6
        {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
        };

// Projection of joint 6 on joint 0
static const Matrix3 P6_0
        {
                0, 0, -1,
                0, 1, 0,
                1, 0, 0
        };

// Projection of joint 0 on joint 6
static const Matrix3 P6_0_Inverse
        {
                0, 0, 1,
                0, 1, 0,
                -1, 0, 0
        };

static const Matrix3 LOOKFORWARD
        {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
        };

static const Matrix3 LOOKBACKWARDS
        {
                -1, 0, 0,
                0, 1, 0,
                0, 0, -1
        };

static const Matrix3 LOOKDOWN
        {
                0, 0, 1,
                0, 1, 0,
                -1, 0, 0
        };

static const Matrix3 LOOKLEFT
        {
                0, -1, 0,
                1, 0, 0,
                0, 0, 1
        };

static const Matrix3 LOOKRIGHT
        {
                0, 1, 0,
                -1, 0, 0,
                0, 0, 1
        };
static const Matrix3 LOOKUP
        {
                0, 0, -1,
                0, 1, 0,
                1, 0, 0    
        };

/**
 * @brief Represents the angles for the robot's joints.
 *
 * This struct holds all the joint angles, and allows for
 * multiplication using operator overload.
 */
struct Angles {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;

    inline void operator*=(float value) {
        theta1 *= value;
        theta2 *= value;
        theta3 *= value;
        theta4 *= value;
        theta5 *= value;
        theta6 *= value;
    };

    inline void operator*(float value) {
        return *this *= value;
    }
};

/**
 * @brief Calculates the inverse kinematics.
 *
 * Calculates the IK from base frame 0 to end effector
 * frame 6.
 *
 * @param x Target x position.
 * @param y Target y position.
 * @param z Target z position.
 * @param R_target Target orientation.
 * @return Returns the angles for all the joints.
 */
Angles IK(float x, float y, float z, const Matrix3 &R_target = P6_0);

/**
 * @brief Calculates the inverse kinematics.
 *
 * Calculates the IK from base frame 0 to end effector
 * frame 6.
 *
 * @param position Target position
 * @param R_target Target orientation.
 * @return Returns the angles for all the joints.
 */
Angles IK(const Vector3 &position, const Matrix3 &R_target = P6_0);

/**
 * @brief Calculates the inverse kinematics for positioning
 *
 * Calculates the IK from base frame 0 to frame 3. This
 * takes care of the positoning of the end effector.
 *
 * @param position The target end effector position.
 * @param R_target The target end effector orientation
 * @return Returns the angles for all the joints.
 */
Angles IK0_3(const Vector3 &position, const Matrix3 &R_target = P6_0);

/**
 * @brief Calculates the inverse kinematics for orientation
 *
 * Calculates the IK from base frame 3 to frame 6. This
 * takes care of the orientation of the end effector.
 *
 * @param angles The angles of joint 1, 2 and 3.
 * @param R_target The target end effector orientation
 * @return Returns the angles for all the joints.
 */
Angles IK3_6(const Vector3 &angles, const Matrix3 &R_target = P6_0);

/**
 * @brief Checks if the angles are within the robots workspace.
 *
 * @param Angles The joint angles.
 * @return Returns true if within workspace, false if not.
 */
bool withinWorkArea(Angles);

#endif //IK_HPP
