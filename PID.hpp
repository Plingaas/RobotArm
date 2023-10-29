#ifndef PID_HPP
#define PID_HPP

#include "math.hpp"

struct PIDParameters {
    float kp;
    float ti;
    float td;

    void set_params(float _kp, float _ti, float _td) {
        kp = _kp;
        ti = _ti;
        td = _td;
    };
};

class PID {

public:

    PID() : PID(0.01f, 0.01f, 0.01f) {}

    PID(float kp, float ti, float td) : params_{kp, ti, td} {}

    float regulate(float setPoint, float measuredValue, float dt) {
        
        if (dt == 0) return 0;

        float currentError = (setPoint - measuredValue);

        integral_ += (currentError * dt);
        float diff = ((currentError - prevError_));
        
        // Windup guard for integral
        if (windupGuard_) {
            integral_ = std::max(-windupGuard_, std::min(integral_, windupGuard_));
        }

        // Save current error as previous error for next iteration
        prevError_ = currentError;

        // Scale
        float P = (currentError * params_.kp);
        float I = (integral_ * params_.ti);
        float D = (diff * params_.td);

        //float sum = P + I + D - error because D = infinity?
        float sum = P + I + D;
    
        return sum;
    }

    void setWindupGuard(const float &windupGuard) {
        windupGuard_ = windupGuard;
    }

    [[nodiscard]] float error() const {
        return prevError_;
    }

    [[nodiscard]] PIDParameters &params() {
        return params_;
    }

    [[nodiscard]] const PIDParameters &params() const {
        return params_;
    }

private:

    float integral_ = 0;
    float prevError_ = 0;
    PIDParameters params_;
    float windupGuard_ = 0.1;
};

#endif //ROBOTCONTROLLER_PID_HPP
