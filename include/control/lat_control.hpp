#ifndef LAT_CONTROLLERS_HPP
#define LAT_CONTROLLERS_HPP

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

#include "control/PIDController.hpp"
#include "control/callback_data_manage.hpp"

class Stanley
{
private:
    double kp_;             // Proportional gain
    double ki_;             // Integral gain
    double kd_;             // Derivative gain
    double target_heading_; // Target heading
    double prevCrossTrackError;
    double integral;
    double anti_windup_max_;
    double Lf;
    double Lr;
    double L;
    double width;
    double crossTrackError_;
    float speed_;
    float ego_heading_;
    double curvature_;
    double heading_gain_;
    double heading_term;
        
public:
    Stanley()
        : prevCrossTrackError(0.0), integral(0.0)
    {
        //요기!!
        Lf = 0.494/2;
        Lr = 0.494/2;
        L = 0.494;
        width = 0.364;
        
        kp_ = 0.1;
        ki_ = 0.0;
        // kd_ = 1.5;
        target_heading_ = 0.0;
        ego_heading_ = 0.0;

        crossTrackError_ = 0.0;
        speed_ = 0.0;
        


        anti_windup_max_ = 0.0;
        heading_gain_ = 1.0;

        heading_term = 0.0;
    }

    void set_stanly_gain(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void set_heading_gain(double h_gain)
    {
        this->heading_gain_ = h_gain;
    }

    void set_anti_windup_max(double anti_windup_max)
    {
        this->anti_windup_max_ = anti_windup_max;
    }

    void set_stanly_data(float speed, double target_heading, float ego_heading,
                         double crossTrackError, double curvature = 0.0)
    {
        this->speed_ = clip(speed, 0.9F, 10.0F);

        this->target_heading_ = nomalize_angle(target_heading);
        this->ego_heading_ = nomalize_angle(ego_heading);
        this->crossTrackError_ = crossTrackError;
        this->curvature_ = curvature; // 튜닝에 사용하는 param 일단은 0.0
    }



    PointFR calc_stanley_steer(double dt = 1)
    {
        // Calculate the heading error


        double tmp_headin_error = nomalize_angle(target_heading_ - ego_heading_);

        // double tmp_headin_error = (target_heading_ - ego_heading_);
        double headingError = this->heading_gain_ * (tmp_headin_error);

        // Normalize the heading error to the range [-pi, pi]
        // headingError = nomalize_angle(headingError);
        heading_term = headingError;
        // Calculate the proportional term
        double proportionalTerm = kp_ * crossTrackError_;

        // Calculate the cross track error derivative
        double crossTrackErrorDerivative = (crossTrackError_ - prevCrossTrackError) / dt;

        // Calculate the derivative term
        double derivativeTerm = kd_ * crossTrackErrorDerivative;

        // Update the previous cross track error for the next iteration
        prevCrossTrackError = crossTrackError_;

        // Calculate the integral term
        integral += crossTrackError_ * dt;
        integral = clip(integral, -anti_windup_max_, anti_windup_max_);


        double integralTerm = ki_ * integral;

        // Calculate the steering angle using Stanley Method
        double PID_steer = proportionalTerm + derivativeTerm + integralTerm;

        double headingErrorTermF = headingError * Lf/(Lf+Lr); 
        double headingErrorTermR = -headingError * Lr/(Lf+Lr); 


        double steeringAngleF = headingErrorTermF + atan2(PID_steer, (0.1 + this->speed_)); 
        double steeringAngleR = headingErrorTermR + atan2(PID_steer, (0.1 + this->speed_)); 
        PointFR steeringAngle;

        steeringAngle.F = nomalize_angle(steeringAngleF);
        steeringAngle.R = nomalize_angle(steeringAngleR);

        return steeringAngle;
    }

    double get_heading_term()
    {
        return heading_term;
    }

    double get_stanley_P_gain()
    {
        return kp_;
    }

    double get_stanley_integral_val()
    {
        return this->integral;
    }

    float get_speed_in_stanley()
    {
        return this->speed_;
    }

    double set_stanley_integral_val(double inte_init)
    {
        this->integral = inte_init;
    }
};

class FeedForward
{
private:
    double Lf;
    double Lr;
    double curvature_;
public:
    FeedForward()
    {
        Lf = 0.494/2;
        Lr = 0.494/2;
    }

    void set_curvature(double curvature) 
    {
        this->curvature_ = curvature;
    }
    
    double calc_FF_SteerF()
    {
        double FF_SteerF = atan2(curvature_ * Lf, 1.0)*0.5;

        return FF_SteerF;
    }

    double calc_FF_SteerR()
    {
        double FF_SteerR = atan2(curvature_ * Lr, 1.0)*0.5;

        return FF_SteerR;
    }
};




class KimmController
{
private:
    double kp_;             // Proportional gain
    double ego_heading_;

    double target_heading_; // Target heading
    double crossTrackError_;
    double crossTrackError_lf_;

    double heading_gain_;
    double heading_term;
    double target_speed;

    double heading_err_range;
    
    double Lf;
    double Lr;
    double L;
    double width;
    double cx_;
    double cy_;

    double tmp_x_error;
    double tmp_y_error;
    double tmp_heading_error;

    double kp_dc_vx;
    double kp_dc_vy;
    double kp_dc_yr;

    double ki_dc_vx;
    double ki_dc_vy;
    double ki_dc_yr;

    double kd_dc_vx;
    double kd_dc_vy;
    double kd_dc_yr;

    double vx_anti_windup_max_;
    double vy_anti_windup_max_;
    double yr_anti_windup_max_;

    double integral_x;
    double integral_y;
    double integral_yr;

    double pre_x;
    double pre_y;
    double pre_yr;


    double error_reset_threshold;

    double target_cx_;
    double target_cy_;
    double target_hd_;

    double target_vx_dk;
    double target_vy_dk;
    double target_yr_dk;

public:
    KimmController()
    {      
        kp_ = 1.0;

        target_heading_ = 0.0;
        ego_heading_ = 0.0;

        crossTrackError_ = 0.0;        
        crossTrackError_lf_ =0.0;

        heading_gain_ = 1.0;
        target_speed = 0.0;
        heading_err_range = M_PI/2;
        // 요기!!
        Lf = 0.494/2.0;
        Lr = 0.494/2.0;
        L = 0.494;
        width = 0.364;

        cx_ = 0.0;
        cy_ = 0.0;

        kp_dc_vx = 1.0;
        kp_dc_vy = 1.0;
        kp_dc_yr = 1.0;

        ki_dc_vx = 0.1;
        ki_dc_vy = 0.1;
        ki_dc_yr = 0.1;

        kd_dc_vx = 0.0;
        kd_dc_vy = 0.0;
        kd_dc_yr = 0.0;

        vx_anti_windup_max_ = 10;
        vy_anti_windup_max_ = 10;
        yr_anti_windup_max_ = 10;

        integral_x = 0.0;
        integral_y = 0.0;
        integral_yr = 0.0;
        pre_x = 0.0;
        pre_y = 0.0;
        pre_yr = 0.0;
        error_reset_threshold = 0.01;

        tmp_x_error = 0.0;
        tmp_y_error = 0.0;
        tmp_heading_error = 0.0;

        target_cx_ = 0.0;
        target_cy_ = 0.0;
        target_hd_ = 0.0;

        target_vx_dk = 0.0;
        target_vy_dk = 0.0;
        target_yr_dk = 0.0;
    }

    void set_kimm_gain(double kp)
    {
        kp_ = kp;
    }

    void set_kimm_docking_gain(double kp_x,double kp_y,double kp_yr,double ki_x,double ki_y,double ki_yr, double kd_x,double kd_y,double kd_yr)
    {
        kp_dc_vx = kp_x;
        kp_dc_vy = kp_y;
        kp_dc_yr = kp_yr;

        ki_dc_vx = ki_x;
        ki_dc_vy = ki_y;
        ki_dc_yr = ki_yr;

        kd_dc_vx = kd_x;
        kd_dc_vy = kd_y;
        kd_dc_yr = kd_yr;
    }

    void set_docking_anti_windup_max(double vx_anti_wu, double vy_anti_wu, double yr_anti_wu)
    {
        this->vx_anti_windup_max_ = vx_anti_wu;
        this->vy_anti_windup_max_ = vy_anti_wu;
        this->yr_anti_windup_max_ = yr_anti_wu;
    }

    void set_kimm_heading_gain(double h_gain)
    {
        this->heading_gain_ = h_gain;
    }

    void set_kimm_target_speed(double target_sp)
    {
        this->target_speed = target_sp;
    }

    void set_kimm_hd_rangge_div(double hd_err_rangge_div)
    {
        this->heading_err_range = M_PI / hd_err_rangge_div;
    }


    void set_kimm_control_data(double target_heading, float ego_heading, double crossTrackError, double crossTrackError_lf)
    {

        this->target_heading_ = nomalize_angle(target_heading);
        this->ego_heading_ = nomalize_angle(ego_heading);
        this->crossTrackError_ = crossTrackError;
        this->crossTrackError_lf_ = crossTrackError_lf;
    }

    void set_odom(double cx, double cy)
    {
        this->cx_ = cx;
        this->cy_ = cy;
    }

    void set_goal_point(double target_cx, double target_cy, double target_hd)
    {
        this->target_cx_ = target_cx;
        this->target_cy_ = target_cy;
        this->target_hd_ = target_hd;
    }

    double get_x_error()
    {
        return this->tmp_x_error; 
    }
    
    double get_y_error()
    {
        return this->tmp_y_error;
    }

    double get_yr_error()
    {
        return this->tmp_heading_error;
    }

    double PI_control(double error, double &integral_term, double kp, double ki, double anti_wu_max) 
    {
        // if (fabs(error) < this->error_reset_threshold) 
        // {
        //     integral_term = 0.0;  // Reset integral term
        // } 
        // else 
        // {
        //     integral_term += error;  // Accumulate error
        // }

        integral_term += error; // Update integral term
        integral_term = clip(integral_term, -anti_wu_max, anti_wu_max);
        return kp * error + ki * integral_term; // PI control output
    }

    double PID_control(double error, double &integral_term, double &previous_error, double kp, double ki, double kd, double anti_wu_max) 
    {
        // Proportional term
        double proportional_term = kp * error;

        // Integral term
        integral_term += error; // Update integral term
        integral_term = clip(integral_term, -anti_wu_max, anti_wu_max);

        // Derivative term
        double derivative_term = kd * (error - previous_error); // Calculate rate of change of error

        // Update previous error
        previous_error = error;

        // PID output
        return proportional_term + ki * integral_term + derivative_term;
    }

    ControlInput calc_control_input()
    {
        // Calculate the heading error
        double yr_const = this->target_speed / sqrt(pow(this->Lf,2) + pow(this->width/2.0 ,2));

        double tmp_headin_error = nomalize_angle(target_heading_ - ego_heading_);
        double lat_error_term = kp_ / this->Lf * crossTrackError_lf_;
        // double tmp_headin_error = (target_heading_ - ego_heading_);
        double target_yr = this->heading_gain_ * (tmp_headin_error) + lat_error_term;
        target_yr = clip(target_yr, -yr_const, yr_const);
        
        double target_speed_constraints = - this->target_speed * abs(tmp_headin_error) / this->heading_err_range + this->target_speed;
        
        target_speed_constraints = clip(target_speed_constraints, 0.0, this->target_speed);

        // Calculate the proportional term
        double target_vy = kp_ * crossTrackError_;

        target_vy = clip(target_vy, -target_speed_constraints, target_speed_constraints);

        double target_vx = sqrt(pow(target_speed_constraints, 2) - pow(target_vy, 2));
        
        



        ControlInput CI;
        CI.vx = target_vx;
        CI.vy = target_vy;
        CI.yr = target_yr;
        

        return CI;
    }

    ControlInput calc_docking_control_input(bool sub_flag)
    {
        
        this->tmp_x_error = (this->target_cx_ - this->cx_)*cos(ego_heading_) + (this->target_cy_ - this->cy_)*sin(ego_heading_);
        this->tmp_y_error = -(this->target_cx_ - this->cx_)*sin(ego_heading_) + (this->target_cy_ - this->cy_)*cos(ego_heading_);
        this->tmp_heading_error = nomalize_angle(this->target_hd_ - ego_heading_);

        // double target_vx_dk = kp_dc_vx *tmp_x_error;
        // double target_vy_dk = -kp_dc_vy *tmp_y_error;
        // double target_yr_dk = -kp_dc_yr *tmp_headin_error;

        
        if (sub_flag)
        {
           
            this->target_vx_dk = PID_control(this->tmp_x_error, this->integral_x,this->pre_x, kp_dc_vx, ki_dc_vx, kd_dc_vx, vx_anti_windup_max_);
            this->target_vy_dk = PID_control(this->tmp_y_error, this->integral_y,this->pre_y, kp_dc_vy, ki_dc_vy, kd_dc_vy, vy_anti_windup_max_);
            this->target_yr_dk = PID_control(this->tmp_heading_error, this->integral_yr,this->pre_yr, kp_dc_yr, ki_dc_yr, kd_dc_yr ,yr_anti_windup_max_);

        }
        


        

        // Calculate the heading error
        double yr_const = this->target_speed / sqrt(pow(this->Lf,2) + pow(this->width/2.0 ,2));

        

        this->target_yr_dk = clip(this->target_yr_dk, -yr_const, yr_const);
        this->target_vx_dk = clip(this->target_vx_dk, -this->target_speed, this->target_speed);
        this->target_vy_dk = clip(this->target_vy_dk, -this->target_speed, this->target_speed);

 

        ControlInput CI_ck;
        CI_ck.vx = this->target_vx_dk;
        CI_ck.vy = this->target_vy_dk;
        CI_ck.yr = this->target_yr_dk;
        

        return CI_ck;
    }



};



class CombinedSteer : public FeedForward, public Stanley, public KimmController
{
private:
    CallbackClass *cb_data_;

    float FF_weight_, stanly_weight_;
    double deltaMax;

    double pre_com_steerF;
    double pre_com_steerR;
    float com_steer_alpha;
    double width;
    double L;
    double FF_steerF;
    double FF_steerR;
    PointFR stanley_steer;
    

public:
    CombinedSteer(CallbackClass *cb_data)
        : FeedForward(), Stanley(),KimmController(), FF_weight_(1),
          stanly_weight_(1), deltaMax(3.141592/2.0 * 89.999 / 90.0) // 89.9sda
    {

        // 요기!!
        this->cb_data_ = cb_data;
        L = 0.494;
        width = 0.364;
        pre_com_steerF = 0;
        pre_com_steerR = 0;
        com_steer_alpha = 0.9;

    }


    ControlInput get_calc_control_input()
    {
        ControlInput tmp_control_input;
        tmp_control_input = calc_control_input();
    
        return tmp_control_input;

    }

    ControlInput get_calc_docking_control_input()
    {
        ControlInput tmp_dk_control_input;
        tmp_dk_control_input = calc_docking_control_input(cb_data_->get_odom_sub_flag2());
        if (cb_data_->get_odom_sub_flag2())
        {
            cb_data_->set_down_odom_sub_flag2();
        }
        
        
        return tmp_dk_control_input;

    }



    PointFR calc_combined_steer()
    {
        double FeedForwardSteeringF = calc_FF_SteerF();
        double FeedForwardSteeringR = calc_FF_SteerR();

        FF_steerF = clip(FeedForwardSteeringF, -deltaMax, deltaMax);
        FF_steerR = clip(FeedForwardSteeringR, -deltaMax, deltaMax);

        if (cb_data_->get_odom_sub_flag())
        {
            // stanley
            this->stanley_steer = calc_stanley_steer();
            this->stanley_steer.F = clip(this->stanley_steer.F, -deltaMax, deltaMax);
            this->stanley_steer.R = clip(this->stanley_steer.R, -deltaMax, deltaMax);
            cb_data_->set_down_odom_sub_flag();
        }

        // Combine the steering angles
        PointFR combinedSteering;
        combinedSteering.F = FF_weight_ * FF_steerF + stanly_weight_ * stanley_steer.F;
        combinedSteering.R = FF_weight_ * FF_steerR + stanly_weight_ * stanley_steer.R;

        double combined_steerF = clip(combinedSteering.F, -deltaMax, deltaMax);
        double combined_steerR = clip(combinedSteering.R, -deltaMax, deltaMax);

        double com_lpf_steerF = low_pass_filter(combined_steerF, pre_com_steerF, com_steer_alpha);
        double com_lpf_steerR = low_pass_filter(combined_steerR, pre_com_steerR, com_steer_alpha);

        pre_com_steerF = com_lpf_steerF;
        pre_com_steerR = com_lpf_steerR;

        PointFR com_lpf_steer;
        com_lpf_steer.F = clip(com_lpf_steerF, -deltaMax, deltaMax);
        com_lpf_steer.R = clip(com_lpf_steerR, -deltaMax, deltaMax);
    

        com_lpf_steer.FL = atan2(tan(com_lpf_steerF) , (1 - (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        com_lpf_steer.FR = atan2(tan(com_lpf_steerF) , (1 + (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        com_lpf_steer.RL = atan2(tan(com_lpf_steerR) , (1 - (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        com_lpf_steer.RR = atan2(tan(com_lpf_steerR) , (1 + (width/(2*L)) * (tan(com_lpf_steerF) - tan(com_lpf_steerR)))); //4ws
        
        
        com_lpf_steer.FL = nomalize_angle(com_lpf_steer.FL);
        com_lpf_steer.FR = nomalize_angle(com_lpf_steer.FR);
        com_lpf_steer.RL = nomalize_angle(com_lpf_steer.RL);
        com_lpf_steer.RR = nomalize_angle(com_lpf_steer.RR);

        com_lpf_steer.FL = clip(com_lpf_steer.FL, -deltaMax, deltaMax);
        com_lpf_steer.FR = clip(com_lpf_steer.FR, -deltaMax, deltaMax);
        com_lpf_steer.RL = clip(com_lpf_steer.RL, -deltaMax, deltaMax);
        com_lpf_steer.RR = clip(com_lpf_steer.RR, -deltaMax, deltaMax);

        return com_lpf_steer;
    }


    double get_FF_steerF()
    {
        return FF_steerF;
    }

    double get_FF_steerR()
    {
        return FF_steerR;
    }

    double get_stanley_steerF()
    {
        return stanley_steer.F;
    }    

    double get_stanley_steerR()
    {
        return stanley_steer.R;
    }    


};

#endif // LAT_CONTROLLERS_HPP
