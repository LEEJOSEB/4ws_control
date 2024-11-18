#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cmath>

struct Point {
    double x;
    double y;
};

struct PointFR {
    double F;
    double R;
    double FL;
    double FR;
    double RL;
    double RR;
};

struct ControlInput {
    double vx;
    double vy;
    double yr;
};

struct GasAndBrake {
    double gas;
    double brake;
};

// 두 점을 연결하는 벡터의 외적 계산
double cross_product(Point p1, Point p2) {
    return p1.x * p2.y - p1.y * p2.x;
}

double determine_side(Point reference, Point target, Point origin) {
    double cross = 0.0;
    Point refVector = {reference.x - origin.x, reference.y - origin.y};
    Point targetVector = {target.x - origin.x, target.y - origin.y};

    return cross = cross_product(refVector, targetVector);
}

double satuation_abs(double val, double s) {
    if (val >= 0 and val < s){
        val = s;
    }
    else if (val < 0 and val > -s) {
        val = -s;
    }

    return val;
}

template <typename T>
T clip(const T &value, const T &min_val, const T &max_val)
{
    return std::max(min_val, std::min(value, max_val));
}

double low_pass_filter(double val, double pre_val, float alpha)
{
    return val * alpha + pre_val * (1-alpha);
}

double nomalize_angle(double rad_angle)
{
    double rad_ang = rad_angle;
    while (rad_ang > M_PI)
        rad_ang -= 2 * M_PI;
    while (rad_ang < -M_PI)
        rad_ang += 2 * M_PI;
    return rad_ang;
}

double sign_determinater(double df, double dr){
    double diff_delta = df - dr;
    double sign = 0.0;

    if (diff_delta >= 0){
        sign = 1.0;
    }
    else{
        sign = -1.0;
    }

    return sign;
}


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

        derivative = low_pass_filter(derivative, pre_derivative_, alpha);
        pre_derivative_ = derivative;

        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        if (output < min_output_)
        {
            output = min_output_;
        } else if (output > max_output_){
            output = max_output_;
        }

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

#endif  // PID_CONTROLLER_HPP
