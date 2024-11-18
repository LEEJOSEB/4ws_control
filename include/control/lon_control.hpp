#ifndef LON_CONTROLLER_HPP
#define LON_CONTROLLER_HPP

#include <iostream>

#include "control/PIDController.hpp"


class LonController
{
private:
    shared_ptr<PIDController> gas_pid;

    float current_speed_;
    float target_speed_;

    float gas_scale_;  

    float pre_calc_speed = 0.0;


public:
    LonController()
        : gas_scale_(10.0){
            gas_pid = std::make_shared<PIDController>(0.0, 1.0);
        }

    void set_lon_data(float current_speed)
    {
        this->current_speed_ = current_speed;
    }

    void set_lon_target_speed(float target_speed)
    {
        this->target_speed_ = target_speed;
    }

    double get_lon_target_speed()
    {
        return this->target_speed_;
    }


    void set_lon_PD_gain(double ga_kp, double ga_kd)
    {
        // i gain은 편의상 0.0으로 고정.
        gas_pid->set_PID_gain(ga_kp, ga_kd);
    }

    GasAndBrake calc_gas_n_brake()
    {

        float error = target_speed_ - current_speed_;
        GasAndBrake return_val;
        double gas_val = gas_pid->compute(target_speed_, current_speed_, 0.1);


        return_val.gas = target_speed_ + gas_val * gas_scale_;



        return_val.gas = clip(return_val.gas, -10.0, 10.0);
        

        this->pre_calc_speed = return_val.gas;

        return return_val;
    }

    float get_target_speed()
    {
        return this->target_speed_;
    }
};

#endif // LON_CONTROLLER_HPP
