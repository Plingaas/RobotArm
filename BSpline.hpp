#ifndef BSPLINE_HPP
#define BSPLINE_HPP

#include "math.hpp"
#include <vector>


class BSpline {
    
public:
    std::vector<Vector3> viapoints = {};
    std::vector<Vector3> path = {};

    float increment = 0.0001f;
    float t = 0.0f;
    float tmax = 0;
    bool finished = false;

    BSpline() {};
    BSpline(std::vector<Vector3> _viapoints) : viapoints(_viapoints) {calculateTmax();};

    float getTmax() {return tmax;}
    void calculateTmax() {tmax = viapoints.size() - 2;}
    void setIncrement(float _increment) {increment = _increment;}
    
    void update();
    void reset();

    Vector3 getSplinePoint();
    void addPoint(Vector3 _viapoint) {viapoints.emplace_back(_viapoint);};
    void addPoints(std::vector<Vector3> _viapoints) {viapoints.insert(viapoints.end(), _viapoints.begin(), _viapoints.end());};
    Vector3 getPoint(int index) {return viapoints[index];};
    void generatePath();
};

#endif