#include "BSpline.hpp"


void BSpline::update() {
    t += increment;

    if (t > tmax) {
        t = 0.0f,
        finished = true;
    }
}

void BSpline::reset() {
    t = 0.0f;
    finished = false;
}

Vector3 BSpline::getSplinePoint() {
    // Influential points
    int p0, p1, p2, p3;

    // Gets the indices given the lerp value t
    p1 = (int)t + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;
    
    float _t = t - (int)t;
    float tt = _t * _t;
    float ttt = tt * _t;

    // Influential field values
    float q1 = -ttt + 2.0f*tt - _t;
    float q2 = 3.0f*ttt - 5.0f*tt + 2.0f;
    float q3 = -3.0f*ttt + 4.0f*tt + _t;
    float q4 = ttt - tt;

    // Sum the influence
    float tx = 0.5f * (viapoints[p0].x * q1 + viapoints[p1].x * q2 + viapoints[p2].x * q3 + viapoints[p3].x * q4);
    float ty = 0.5f * (viapoints[p0].y * q1 + viapoints[p1].y * q2 + viapoints[p2].y * q3 + viapoints[p3].y * q4);
    float tz = 0.5f * (viapoints[p0].z * q1 + viapoints[p1].z * q2 + viapoints[p2].z * q3 + viapoints[p3].z * q4);

    return Vector3{tx, ty, tz};
};

void BSpline::generatePath() {

    path = {};
    calculateTmax();
    float _t = 0;

    while (_t < tmax) {

        
    }
}