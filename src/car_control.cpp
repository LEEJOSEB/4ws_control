#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>

#include "control/callback_data_manage.hpp"
#include "control/PIDController.hpp"
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"
#include "control/kalman_filter.hpp"

// #include "ranger_msgs/msg/actuator_state_array.hpp"
// #include "ranger_msgs/msg/actuator_state.hpp"
// #include "ranger_msgs/msg/motor_state.hpp"

using namespace std;

bool IS_PRINT = true;
bool c_mode = false;
int docking_mode = 0;
int control_mode_pre  = 0;
enum State {FORWARD, REVERSE};
State state = FORWARD;
double target_speed_x_prev = 0.0, target_speed_y_prev = 0.0, target_yr_prev = 0.0;
ControlInput kimm_ci_dk;

class CarControl : public rclcpp::Node
{
private:
    std::shared_ptr<KalmanFilter2D> km_filter_ptr;
    KalmanFilter2D *km_filter;
    
    std::shared_ptr<CallbackClass> callback_data_ptr;
    CallbackClass *cb_data;

    std::shared_ptr<CombinedSteer> lat_control_ptr;
    CombinedSteer *lat_control;

    std::shared_ptr<LonController> lon_control_ptr;
    std::shared_ptr<LonController> lon_control_ptr_fl;
    std::shared_ptr<LonController> lon_control_ptr_fr;
    std::shared_ptr<LonController> lon_control_ptr_rl;
    std::shared_ptr<LonController> lon_control_ptr_rr;

    LonController *lon_control;
    LonController *lon_control_fl;
    LonController *lon_control_fr;
    LonController *lon_control_rl;
    LonController *lon_control_rr;
    

    std::shared_ptr<ControlGainTuning> param_manage_ptr;  
    ControlGainTuning *param_manage;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr lo_odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lo_curr_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lo_imu__sub;
    
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pl_local_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pl_cont_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pl_pyaw_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pl_control_flag_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pl_pyaws_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr issac_state_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_mode_sub;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_point_sub;


    // rclcpp::Subscription<ranger_msgs::msg::ActuatorStateArray>::SharedPtr ranger_data_sub;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ranger_data_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tmp_data_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr issac_cmd_pub;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_execution_time_ = now();

public:
    CarControl()
        : Node("car_control")
        // <ROS 노드 선언>---------------------------------------------------------
    // ---------------------------------------------------------</ROS 노드 선언>
    {        

        // <클레스들 인스턴스화>------------------------------------------------------
        
        km_filter_ptr = std::make_shared<KalmanFilter2D>();
        km_filter = km_filter_ptr.get();
        
        callback_data_ptr = std::make_shared<CallbackClass>(km_filter);
        cb_data = callback_data_ptr.get();

        lat_control_ptr = std::make_shared<CombinedSteer>(cb_data);
        lat_control = lat_control_ptr.get();
    
        lon_control_ptr = std::make_shared<LonController>();
        lon_control_ptr_fl = std::make_shared<LonController>();
        lon_control_ptr_fr = std::make_shared<LonController>();
        lon_control_ptr_rl = std::make_shared<LonController>();
        lon_control_ptr_rr = std::make_shared<LonController>();

        lon_control = lon_control_ptr.get();
        lon_control_fl = lon_control_ptr_fl.get();
        lon_control_fr = lon_control_ptr_fr.get();
        lon_control_rl = lon_control_ptr_rl.get();
        lon_control_rr = lon_control_ptr_rr.get();

        param_manage_ptr = std::make_shared<ControlGainTuning>(cb_data, lat_control, lon_control, km_filter);
        param_manage = param_manage_ptr.get();

        // ------------------------------------------------------</클레스들 인스턴스화>
        // <SUBSCRIBER> -----------------------------------------------------------
        
        // Local
        lo_odom_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Local/utm", 1, std::bind(&CallbackClass::lo_odom_cb, cb_data, std::placeholders::_1));
        lo_curr_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", 1, std::bind(&CallbackClass::lo_yaw_cb, cb_data, std::placeholders::_1));
        // lo_imu__sub = this->create_subscription<sensor_msgs::msg::Imu>("/Local/imu_hpc/out", 1, std::bind(&CallbackClass::lo_imu_cb, cb_data, std::placeholders::_1));
        
        // Planning
        pl_local_sub = this->create_subscription<nav_msgs::msg::Path>("/Planning/local_path", 1, std::bind(&CallbackClass::pl_local_path_cb, cb_data, std::placeholders::_1));
        pl_control_flag_sub = this->create_subscription<std_msgs::msg::Bool>("/Planning/Control_SW", 1, std::bind(&CallbackClass::pl_control_sw_cb, cb_data, std::placeholders::_1));
        
        issac_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/isaac_joint_states", 1, std::bind(&CallbackClass::issac_state_cb, cb_data, std::placeholders::_1));
        control_mode_sub = this->create_subscription<std_msgs::msg::Int32>("/Control/mod", 1, std::bind(&CallbackClass::control_mode_cb, cb_data, std::placeholders::_1));
        goal_point_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Planning/goal_point", 1, std::bind(&CallbackClass::goal_point_cb, cb_data, std::placeholders::_1));
        
        
      
        
        
        //ranger data
        // ranger_data_sub = this->create_subscription<ranger_msgs::msg::ActuatorStateArray>("/ranger_states", 1, std::bind(&CallbackClass::ranger_data_cb, cb_data, std::placeholders::_1));
        // cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/ranger/cmd_vel", 1, std::bind(&CallbackClass::ranger_vel_cb, cb_data, std::placeholders::_1));
        // -----------------------------------------------------------</SUBSCRIBER>
        // <PUBLISHER> ------------------------------------------------------------

        ranger_data_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Control/ranger_data", 1);
        tmp_data_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Control/tmp_plot_val", 1);
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/ranger_mini/cmd_vel", 1);
        issac_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_commands", 1);
        // ------------------------------------------------------------</PUBLISHER>
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&CarControl::timer_callback, this));
        
        
    }

    void timer_callback()
    {

        auto start_time = std::chrono::steady_clock::now();
        rclcpp::Time current_time = now();
        
        auto time_diff = current_time - last_execution_time_;
        last_execution_time_ = now();
        double time_diff_ms = time_diff.seconds() * 1000.0;
        last_execution_time_ = current_time;

        // 요기!!!!
        double wheel_radius = 0.105;
        double L = 0.494 *2;
        double Lf = L / 2.0;
        double width = 0.364;
        double deltaMax = 3.141592/2.0 ;

        int control_method = 0;
        int test = 1;

        float curr_speed = cb_data->get_speed();
        float wheel_speed_FL = cb_data->get_wheel_speed_FL(); // m/s 
        float wheel_speed_FR = cb_data->get_wheel_speed_FR(); // m/s 
        float wheel_speed_RL = cb_data->get_wheel_speed_RL(); // m/s 
        float wheel_speed_RR = cb_data->get_wheel_speed_RR(); // m/s 

        float wheel_steer_FL = cb_data->get_wheel_steer_FL();
        float wheel_steer_FR = cb_data->get_wheel_steer_FR();
        float wheel_steer_RL = cb_data->get_wheel_steer_RL();
        float wheel_steer_RR = cb_data->get_wheel_steer_RR();

        int control_mode = cb_data->get_control_mode();
        
        
        float yaw = cb_data->get_yaw();
        Point odom = cb_data->get_odom();
        double lat_error = cb_data->calc_n_get_lat_error();
        double yaw_rate = cb_data->get_yawrate();

        Point odom_lf;
        odom_lf.x = Lf;
        odom_lf.y = 0.0;
        double lat_error_lf = cb_data->calc_n_get_lat_error(odom_lf);
        double path_curature = cb_data->calc_path_curvature_center(1.0);
        double pd_path_yaw = cb_data->get_pd_path_yaw();
        vector<Point> rel_local_path = cb_data->get_relative_path();

        // lat_error가 404면 뭔가 잘못됨.
        if (lat_error == 404 || pd_path_yaw == 404)
        {
            cout << "404!!!" << endl;
            // return;
        }

        

        

        // <계산> -----------------------------------------------------------
        param_manage->set_normal_param();

        // Control Switch
        // int cs = cb_data->get_control_switch();
        bool control_sw = cb_data->get_control_sw();

        GasAndBrake acc_val_FL;
        GasAndBrake acc_val_FR;
        GasAndBrake acc_val_RL;
        GasAndBrake acc_val_RR;

        PointFR steerAngle;

        double target_speed = lon_control->get_target_speed() / 3.6;
        

        double target_speed_FR;
        double target_speed_FL;
        double target_speed_RR;
        double target_speed_RL;


        lat_control->set_stanly_data(curr_speed, pd_path_yaw, yaw, lat_error);
        lat_control->set_curvature(path_curature);

        lat_control->set_kimm_target_speed(target_speed);
        lat_control->set_kimm_control_data(pd_path_yaw, yaw, lat_error, lat_error_lf);
        lat_control->set_odom(odom.x, odom.y);

        


        ControlInput kimm_ci;
        kimm_ci = lat_control->get_calc_control_input();
        steerAngle = lat_control->calc_combined_steer();

        if (control_mode == 2){
            if (control_mode_pre ==  0){
                docking_mode = 1;
            }
            else if (std::fabs(nomalize_angle(lat_control->get_yr_error())) < 0.02)
            {
                docking_mode = 2;
            }
            
            
        }

        else if (control_mode == 0){
            docking_mode = 0;
        }


        // Point odom = 0.0;
        control_mode_pre = control_mode;
        double sat_min_value = 0.01;
        
        PointFR steerAngle_dk;
        double target_speed_dk_FL;
        double target_speed_dk_FR;
        double target_speed_dk_RL;
        double target_speed_dk_RR;

                        

        if (control_mode == 1 || control_mode == 2)
        {
            lat_control->set_goal_point(cb_data->get_target_point_x(),cb_data->get_target_point_y(),cb_data->get_target_point_yaw());
            kimm_ci_dk = lat_control->get_calc_docking_control_input();

            double vx_safe = kimm_ci_dk.vx;
            if (vx_safe > -sat_min_value && vx_safe < sat_min_value) {
                vx_safe = (vx_safe >= 0.0) ? sat_min_value : -sat_min_value;  // Set to 0.01 if positive, -0.01 if negative
            }
            double beta_dk = atan(kimm_ci_dk.vy / vx_safe);

            double Rc_dem = clip(pow(kimm_ci_dk.yr, 2), sat_min_value, std::numeric_limits<double>::infinity());
            
            // double Rc_dk = sqrt(pow(kimm_ci_dk.vx, 2) + pow(kimm_ci_dk.vy, 2)) * kimm_ci_dk.yr / Rc_dem;
            double Rc_dk = (kimm_ci_dk.yr * kimm_ci_dk.vy - kimm_ci_dk.yr * kimm_ci_dk.vx)/ Rc_dem;

            
            double R_dk = Rc_dk * cos(beta_dk);

            

            
            double epsilon = 0.1;

            if (state == FORWARD && kimm_ci_dk.vx < -epsilon) {
                state = REVERSE;
            }
            else if ( (state == REVERSE && kimm_ci_dk.vx > epsilon) ){
                state = FORWARD;
            }

            double vx_sign = 1.0;

            if (state == FORWARD){
                vx_sign = 1.0;
            }
            else{
                vx_sign = -1.0;
            }
            steerAngle_dk.F = atan((kimm_ci_dk.yr * L / 2 + kimm_ci_dk.vy) / std::fabs(kimm_ci_dk.vx)*vx_sign);
            steerAngle_dk.R = atan((-kimm_ci_dk.yr * L / 2 + kimm_ci_dk.vy) / std::fabs(kimm_ci_dk.vx)*vx_sign);
        
            // steerAngle_dk.F = atan((kimm_ci_dk.yr * L / 2 + kimm_ci_dk.vy) / (state == REVERSE) ? -std::fabs(kimm_ci_dk.vx) : std::fabs(kimm_ci_dk.vx));
            // steerAngle_dk.R = atan((-kimm_ci_dk.yr * L / 2 + kimm_ci_dk.vy) / (state == REVERSE) ? -std::fabs(kimm_ci_dk.vx) : std::fabs(kimm_ci_dk.vx));
            // steerAngle_dk.F = atan((Rc_dk * sin(beta_dk) + L/2) / R_dk);
            // steerAngle_dk.R = atan((Rc_dk * sin(beta_dk) - L/2) / R_dk);

            steerAngle_dk.FL = atan2(tan(steerAngle_dk.F) , (1 - (width/(2*L)) * (tan(steerAngle_dk.F) - tan(steerAngle_dk.R)))); //4ws
            steerAngle_dk.FR = atan2(tan(steerAngle_dk.F) , (1 + (width/(2*L)) * (tan(steerAngle_dk.F) - tan(steerAngle_dk.R)))); //4ws
            steerAngle_dk.RL = atan2(tan(steerAngle_dk.R) , (1 - (width/(2*L)) * (tan(steerAngle_dk.F) - tan(steerAngle_dk.R)))); //4ws
            steerAngle_dk.RR = atan2(tan(steerAngle_dk.R) , (1 + (width/(2*L)) * (tan(steerAngle_dk.F) - tan(steerAngle_dk.R)))); //4ws
            
            
            steerAngle_dk.FL = nomalize_angle(steerAngle_dk.FL);
            steerAngle_dk.FR = nomalize_angle(steerAngle_dk.FR);
            steerAngle_dk.RL = nomalize_angle(steerAngle_dk.RL);
            steerAngle_dk.RR = nomalize_angle(steerAngle_dk.RR);

            steerAngle_dk.FL = clip(steerAngle_dk.FL, -deltaMax, deltaMax);
            steerAngle_dk.FR = clip(steerAngle_dk.FR, -deltaMax, deltaMax);
            steerAngle_dk.RL = clip(steerAngle_dk.RL, -deltaMax, deltaMax);
            steerAngle_dk.RR = clip(steerAngle_dk.RR, -deltaMax, deltaMax);
            

            target_speed_dk_FL = kimm_ci_dk.vx * cos(steerAngle_dk.FL) - kimm_ci_dk.yr * cos(steerAngle_dk.FL) * width / 2 \
                            + kimm_ci_dk.yr * sin(steerAngle_dk.FL) * L / 2 + kimm_ci_dk.vy * sin(steerAngle_dk.FL);

            target_speed_dk_FR = kimm_ci_dk.vx * cos(steerAngle_dk.FR) + kimm_ci_dk.yr * cos(steerAngle_dk.FR) * width / 2 \
                            + kimm_ci_dk.yr * sin(steerAngle_dk.FR) * L / 2 + kimm_ci_dk.vy * sin(steerAngle_dk.FR);

            target_speed_dk_RL = kimm_ci_dk.vx * cos(steerAngle_dk.RL) - kimm_ci_dk.yr * cos(steerAngle_dk.RL) * width / 2 \
                            - kimm_ci_dk.yr * sin(steerAngle_dk.RL) * L / 2 + kimm_ci_dk.vy * sin(steerAngle_dk.RL);

            target_speed_dk_RR = kimm_ci_dk.vx * cos(steerAngle_dk.RR) + kimm_ci_dk.yr * cos(steerAngle_dk.RR) * width / 2 \
                            - kimm_ci_dk.yr * sin(steerAngle_dk.RR) * L / 2 + kimm_ci_dk.vy * sin(steerAngle_dk.RR);


            kimm_ci_dk.vx = std::fabs(kimm_ci_dk.vx)*vx_sign;

            if (std::fabs(kimm_ci_dk.vx) < 0.05)
            {
                kimm_ci_dk.vx = 0.0;   
            }
        }
        

        
        
        

        

        


        //
        double target_yr = 0.0;
        double target_speed_x = 0.0;
        double target_speed_y = 0.0;

        double R = L * (cos(steerAngle.F) * cos(steerAngle.R)) / satuation_abs((sin(steerAngle.F) * cos(steerAngle.R) - sin(steerAngle.R) * cos(steerAngle.F)), 0.0000001);
        double Rf = R / cos(steerAngle.F);
        double Rr = R / cos(steerAngle.R);

        double target_beta = atan( ((R * sin(steerAngle.F) - Lf * cos(steerAngle.F)) / cos(steerAngle.F)) * (1 / R));
        double Rc = R / cos(target_beta);

        if (abs(Rf) > abs(Rr))
        {
            target_yr = target_speed / Rf;
        }
        else
        {
            target_yr = target_speed / Rr;
        }

        
        target_speed_x = target_yr * Rc * cos(target_beta);
        target_speed_y = target_yr * Rc * sin(target_beta);



        target_speed_FL = target_speed_x * cos(steerAngle.FL) - target_yr * cos(steerAngle.FL) * width / 2 \
                        + target_yr * sin(steerAngle.FL) * L / 2 + target_speed_y * sin(steerAngle.FL);

        target_speed_FR = target_speed_x * cos(steerAngle.FR) + target_yr * cos(steerAngle.FR) * width / 2 \
                        + target_yr * sin(steerAngle.FR) * L / 2 + target_speed_y * sin(steerAngle.FR);

        target_speed_RL = target_speed_x * cos(steerAngle.RL) - target_yr * cos(steerAngle.RL) * width / 2 \
                        - target_yr * sin(steerAngle.RL) * L / 2 + target_speed_y * sin(steerAngle.RL);

        target_speed_RR = target_speed_x * cos(steerAngle.RR) + target_yr * cos(steerAngle.RR) * width / 2 \
                        + target_yr * sin(steerAngle.RR) * L / 2 + target_speed_y * sin(steerAngle.RR);




        lon_control_fl->set_lon_target_speed(target_speed_dk_FL);
        lon_control_fl->set_lon_data(wheel_speed_FL);
        acc_val_FL = lon_control_fl->calc_gas_n_brake();

        lon_control_fr->set_lon_target_speed(target_speed_dk_FR);                                              
        lon_control_fr->set_lon_data(wheel_speed_FR);
        acc_val_FR = lon_control_fr->calc_gas_n_brake();

        lon_control_rl->set_lon_target_speed(target_speed_dk_RL);
        lon_control_rl->set_lon_data(wheel_speed_RL);
        acc_val_RL = lon_control_rl->calc_gas_n_brake();

        lon_control_rr->set_lon_target_speed(target_speed_dk_RR);
        lon_control_rr->set_lon_data(wheel_speed_RR);
        acc_val_RR = lon_control_rr->calc_gas_n_brake();

        double c_mode_condition = 0.6;
        double c_mode_gain = 2.0;
        if (control_mode == 1 || control_mode == 2)
        {
            c_mode_gain = 0.5;
        }
        else{
            docking_mode = 0;
        }
        // RCLCPP_INFO(this->get_logger(), "Docking_mode: %d, Control_mode: %d", docking_mode, control_mode);
        if (test)
        {
            if (control_mode == 1 || control_mode == 2)
            {

                target_speed_x = kimm_ci_dk.vx;
                target_speed_y = kimm_ci_dk.vy;
                target_yr = kimm_ci_dk.yr;

                if (docking_mode == 1){
                    target_speed_x = 0;
                    target_speed_y = 0;
                }
                else
                {
                    target_yr = 0;
                }
                
                // cout << "Docking Mode!" << endl;
            } 
            else{
                target_speed_x = kimm_ci.vx;
                target_speed_y = 0;
                target_yr = kimm_ci.yr;
            }
        }

        

        // if(c_mode)
        // {
        //     if (abs(target_speed_y) < abs(target_yr*(L/2)) * c_mode_gain * c_mode_condition)
        //     {
        //         c_mode = false;
        //     }
        // }
        // else{
        //     if (abs(target_speed_y)* c_mode_condition > abs(target_yr*(L/2)) * c_mode_gain)
        //     {
        //         c_mode = true;
        //     }
        // }


        // if ((abs(target_speed_y) < 0.4) && (abs(target_yr*(L/2))  * c_mode_gain< 0.6)){
        //     c_mode = false;
        // }

        // if (c_mode){
        //     target_yr = 0;
        // }

        // else{
            
        //     target_speed_y = 0; 

        // }

        double alpha = 0.006;
        target_speed_x = applyLPF(target_speed_x, target_speed_x_prev, alpha);
        target_speed_y = applyLPF(target_speed_y, target_speed_y_prev, alpha);
        target_yr = applyLPF(target_yr, target_yr_prev, alpha);

        target_speed_x_prev = target_speed_x; 
        target_speed_y_prev = target_speed_y;
        target_yr_prev = target_yr;


    

        switch (control_sw)
        {

        case true: // 정지 모드

            acc_val_FL.gas = 0.0;
            acc_val_FR.gas = 0.0;
            acc_val_RL.gas = 0.0;
            acc_val_RR.gas = 0.0;

            target_yr = 0.0;
            target_speed_x = 0.0;
            target_speed_y = 0.0;

            cout << "Stop Mode!!" << endl;

            break;

        case false: // normal 모드
            break;
        }

        switch (control_mode)
        {

        case 3: // 정지 모드

            acc_val_FL.gas = 0.0;
            acc_val_FR.gas = 0.0;
            acc_val_RL.gas = 0.0;
            acc_val_RR.gas = 0.0;

            target_yr = 0.0;
            target_speed_x = 0.0;
            target_speed_y = 0.0;

            cout << "Stop Mode!!" << endl;

            break;

        case false: // normal 모드
            break;
        }

 
        // ---------------------------------------------------------- </계산>
        // <ros topic pub> -------------------------------------------------

        // 제어 값 pub

        auto car_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();

        car_data_msg->data.push_back(acc_val_FL.gas / wheel_radius);
        car_data_msg->data.push_back(acc_val_FR.gas / wheel_radius);
        car_data_msg->data.push_back(acc_val_RL.gas / wheel_radius);
        car_data_msg->data.push_back(acc_val_RR.gas / wheel_radius);

        car_data_msg->data.push_back(steerAngle.FL);      
        car_data_msg->data.push_back(steerAngle.FR);   
        car_data_msg->data.push_back(steerAngle.RL);    
        car_data_msg->data.push_back(steerAngle.RR);   




        ranger_data_pub->publish(*car_data_msg);  

        // 제어 방식 2

        

        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();


        if (control_method == 0)
        {
            twist_msg->linear.x = target_speed_x;
            twist_msg->linear.y = target_speed_y;
            twist_msg->angular.z = target_yr;
        }
        else
        {
            twist_msg->linear.x = kimm_ci.vx;
            twist_msg->linear.y = kimm_ci.vy;
            twist_msg->angular.z = kimm_ci.yr;
        }
        


        cmd_vel_pub->publish(*twist_msg);  


        

        auto issac_cmd_msgs =  std::make_shared<sensor_msgs::msg::JointState>();

        double wheel_diff_FL = std::fabs(wheel_steer_FL - steerAngle_dk.FL);
        double wheel_diff_FR = std::fabs(wheel_steer_FR - steerAngle_dk.FR);
        double wheel_diff_RL = std::fabs(wheel_steer_RL - steerAngle_dk.RL);
        double wheel_diff_RR = std::fabs(wheel_steer_RR - steerAngle_dk.RR);
 
        std::vector<double> steering_position_ = {
            steerAngle_dk.FL,   // Front Left
            steerAngle_dk.FR,   // Front Right
            steerAngle_dk.RL,   // Rear Left
            steerAngle_dk.RR    // Rear Right
        };

        // Set target speeds for each wheel
        std::vector<double> wheel_velocity_ = {
            acc_val_FL.gas, // Front Left
            acc_val_FR.gas, // Front Right
            acc_val_RL.gas, // Rear Left
            acc_val_RR.gas  // Rear Right
        };

        // std::vector<double> wheel_velocity_ = {
        //     target_speed_dk_FL, // Front Left
        //     target_speed_dk_FR, // Front Right
        //     target_speed_dk_RL, // Rear Left
        //     target_speed_dk_RR  // Rear Right
        // };


        issac_cmd_msgs->name = {
            "fl_steering_wheel_joint", 
            "fr_steering_wheel_joint", 
            "rl_steering_wheel_joint", 
            "rr_steering_wheel_joint",
            "fl_wheel_joint", 
            "fr_wheel_joint", 
            "rl_wheel_joint", 
            "rr_wheel_joint"
        };

        issac_cmd_msgs->position = steering_position_;
        issac_cmd_msgs->position.resize(8, NAN); 
        issac_cmd_msgs->velocity.resize(8, NAN);  // 4개의 바퀴와 4개의 조향 관절 (총 8개)
        std::copy(wheel_velocity_.begin(), wheel_velocity_.end(), issac_cmd_msgs->velocity.begin() + 4);

        issac_cmd_msgs->velocity[5] = -issac_cmd_msgs->velocity[5];
        issac_cmd_msgs->velocity[7] = -issac_cmd_msgs->velocity[7];
     
        if (std::max({wheel_diff_FL,wheel_diff_FR,wheel_diff_RL,wheel_diff_RR}) > 0.1745)
        {
            issac_cmd_msgs->velocity[4] = 0.0;
            issac_cmd_msgs->velocity[5] = 0.0;
            issac_cmd_msgs->velocity[6] = 0.0;
            issac_cmd_msgs->velocity[7] = 0.0;

        }

        if (sqrt(pow(lat_control->get_x_error(),2)+pow(lat_control->get_y_error(),2)) < 0.05 && std::fabs(nomalize_angle(lat_control->get_yr_error())) < 0.05)
        {
            issac_cmd_msgs->velocity[4] = 0.0;
            issac_cmd_msgs->velocity[5] = 0.0;
            issac_cmd_msgs->velocity[6] = 0.0;
            issac_cmd_msgs->velocity[7] = 0.0;

            issac_cmd_msgs->position[0] = 0.0;
            issac_cmd_msgs->position[1] = 0.0;
            issac_cmd_msgs->position[2] = 0.0;
            issac_cmd_msgs->position[3] = 0.0;
        }

        issac_cmd_pub->publish(*issac_cmd_msgs); // ys!!

        // 디버그용 토픽
        auto tmp_plot_val_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
 
        tmp_plot_val_msg->data.push_back(docking_mode);                         //1
        tmp_plot_val_msg->data.push_back(control_mode);                           //2
        tmp_plot_val_msg->data.push_back(lat_control->get_x_error());  
        tmp_plot_val_msg->data.push_back(lat_control->get_y_error());                        //4
        tmp_plot_val_msg->data.push_back(nomalize_angle(lat_control->get_yr_error()));     //5
        tmp_plot_val_msg->data.push_back(kimm_ci_dk.vx);                     //6
        tmp_plot_val_msg->data.push_back(kimm_ci_dk.vy);                     //7
        tmp_plot_val_msg->data.push_back(kimm_ci_dk.yr);                     //8
        tmp_plot_val_msg->data.push_back(steerAngle_dk.F);                     //9 
        tmp_plot_val_msg->data.push_back(steerAngle_dk.R);  
        tmp_plot_val_msg->data.push_back(steerAngle_dk.FL);                     //9 
        tmp_plot_val_msg->data.push_back(steerAngle_dk.RL);  



        auto end_time = std::chrono::steady_clock::now();
        auto duration = end_time - start_time;
        double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count()* 1000.0;



        tmp_plot_val_msg->data.push_back(seconds); 
        tmp_plot_val_msg->data.push_back(time_diff_ms); 

        for (size_t i = 0; i < rel_local_path.size(); i += 5)
        {
            Point tmp_p = rel_local_path[i];

            tmp_plot_val_msg->data.push_back(tmp_p.x);  
            tmp_plot_val_msg->data.push_back(tmp_p.y);      
            
        }

        
        

        

        tmp_data_pub->publish(*tmp_plot_val_msg);
        

        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

