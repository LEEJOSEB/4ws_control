#pragma once
#include "basic_types.hpp"
#include "math_utils.hpp"
#include <cmath>

using namespace math_utils; 

class PIDController
{
public:
    PIDController(double min_output, double max_output)
        : min_output_(min_output), max_output_(max_output),
          integral_(0), prev_error_(0) {} 

    void set_PID_gain(double kp, double kd)
    {
        // i gain은 편의상 0.0으로 고정.
        kp_ = kp;
        ki_ = 0.0; // ki;
        kd_ = kd;
    }

    double compute(double target_value, double measured_value, float alpha, double dt = 0.05)
    {
        double error = target_value - measured_value;
        integral_ += error * dt;

        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        derivative = lowPassFilter(derivative, pre_derivative_, alpha);
        pre_derivative_ = derivative;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        output = clip(output, min_output_, max_output_);

        return output;
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double min_output_;
    double max_output_;
    double integral_;
    double prev_error_;

    double pre_derivative_ = 0.0;
};
