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
#include "control/lat_control.hpp"
#include "control/lon_control.hpp"
#include "control/mission_param.hpp"
#include "control/kalman_filter.hpp"
#include "control/control_types.hpp"
#include "control/motion_calculator.hpp"
#include "control/math_utils.hpp"

// #include "ranger_msgs/msg/actuator_state_array.hpp"
// #include "ranger_msgs/msg/actuator_state.hpp"
// #include "ranger_msgs/msg/motor_state.hpp"

using namespace std;
using namespace math_utils;

class CarControl : public rclcpp::Node
{
private:
    std::shared_ptr<KalmanFilter2D> km_filter_ptr;
    KalmanFilter2D* km_filter;
    
    std::shared_ptr<CallbackClass> callback_data_ptr;
    CallbackClass* cb_data;

    std::shared_ptr<CombinedSteer> lat_control_ptr;
    CombinedSteer* lat_control;

    std::shared_ptr<LonController> lon_control_ptr;
    std::shared_ptr<LonController> lon_control_ptr_fl;
    std::shared_ptr<LonController> lon_control_ptr_fr;
    std::shared_ptr<LonController> lon_control_ptr_rl;
    std::shared_ptr<LonController> lon_control_ptr_rr;

    LonController* lon_control;
    LonController* lon_control_fl;
    LonController* lon_control_fr;
    LonController* lon_control_rl;
    LonController* lon_control_rr;
    

    std::shared_ptr<ControlGainTuning> param_manage_ptr;
    ControlGainTuning* param_manage;

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
    rclcpp::Time last_execution_time_;
    double time_diff_ms_;
    bool IS_PRINT = true;
    bool c_mode = false;
    int docking_mode = 0;
    ControlMode control_mode_pre = ControlMode::NORMAL;  
    DrivingState driving_state = DrivingState::FORWARD;
    double target_speed_x_prev = 0.0, target_speed_y_prev = 0.0, target_yr_prev = 0.0;
    ControlInput kimm_ci_dk;

    double wheel_radius;
    double L;
    double Lf;
    double width;
    double deltaMax;

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
        lo_odom_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/odom_gt", 1, std::bind(&CallbackClass::lo_odom_cb, cb_data, std::placeholders::_1));
        lo_curr_sub = this->create_subscription<std_msgs::msg::Float64>("/heading", 1, std::bind(&CallbackClass::lo_yaw_cb, cb_data, std::placeholders::_1));
        // lo_imu__sub = this->create_subscription<sensor_msgs::msg::Imu>("/Local/imu_hpc/out", 1, std::bind(&CallbackClass::lo_imu_cb, cb_data, std::placeholders::_1));
        
        // Planning
        pl_local_sub = this->create_subscription<nav_msgs::msg::Path>("/path_drone", 1, std::bind(&CallbackClass::pl_local_path_cb, cb_data, std::placeholders::_1));
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

private:
    void timer_callback() {
        // auto start_time = std::chrono::steady_clock::now();
        // rclcpp::Time current_time = this->now();
        
        // // 1. 시간 관련 업데이트
        // updateTimeDiff(start_time);  // ROS Time 사용

        // 2. 파라미터 업데이트
        updateParameters();

        // 3. 차량 상태 업데이트
        VehicleState current_state = updateVehicleState();

        // 4. 제어 모드에 따른 제어 입력 계산
        ControlInput control_input_kimm_sim, control_input_real_vehicle, control_input_scout;
        PointFR steerAngle_kimm_sim, steerAngle_real_vehicle, steerAngle_input_scout;
        MotionCalculator::WheelSpeeds wheel_speeds_kimm_sim, wheel_speeds_real_vehicle, wheel_speeds_scout;

        ControlMode current_mode = static_cast<ControlMode>(cb_data->get_control_mode());
        updateDockingMode(current_mode);

        switch(current_mode) {
            case ControlMode::NORMAL:
                calculateNormalControl(current_state, control_input_kimm_sim, control_input_real_vehicle, control_input_scout, 
                    steerAngle_kimm_sim, steerAngle_real_vehicle, steerAngle_input_scout, wheel_speeds_kimm_sim, wheel_speeds_real_vehicle, wheel_speeds_scout);
                break;
            case ControlMode::DOCKING:
                calculateDockingControl(current_state, control_input_kimm_sim, control_input_real_vehicle,
                    steerAngle_kimm_sim, steerAngle_real_vehicle, current_mode, wheel_speeds_kimm_sim, wheel_speeds_real_vehicle, wheel_speeds_scout);
                break;
            case ControlMode::ARRIVAL:
                calculateDockingControl(current_state, control_input_kimm_sim, control_input_real_vehicle,
                    steerAngle_kimm_sim, steerAngle_real_vehicle, current_mode, wheel_speeds_kimm_sim, wheel_speeds_real_vehicle, wheel_speeds_scout);
                break;
            case ControlMode::STOP:
                // 모든 제어 입력을 0으로 설정
                control_input_kimm_sim = {0.0, 0.0, 0.0};
                control_input_real_vehicle = {0.0, 0.0, 0.0};
                control_input_scout = {0.0, 0.0, 0.0};
                
                // 모든 조향각과 휠 속도를 0으로 설정
                steerAngle_kimm_sim = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                steerAngle_real_vehicle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                steerAngle_input_scout = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                wheel_speeds_kimm_sim = {0.0, 0.0, 0.0, 0.0};
                wheel_speeds_real_vehicle = {0.0, 0.0, 0.0, 0.0};
                wheel_speeds_scout = {0.0, 0.0, 0.0, 0.0};
                break;
            default:
                break;
        }

        // 5. 제어 명령 발행
        publishControlCommands(control_input_real_vehicle, steerAngle_kimm_sim, wheel_speeds_kimm_sim);

        // 6. 디버그 정보 발행
        // rclcpp::Time end_time = this->now();
        // rclcpp::Duration execution_time = end_time - current_time;
        publishDebugInfo(steerAngle_real_vehicle, control_input_kimm_sim);


        return;
    }

    

    // void updateTimeDiff(const rclcpp::Time& current_time) {
    //     if (last_execution_time_.nanoseconds() != 0) {  // 첫 실행이 아닌 경우에만
    //         auto time_diff = current_time - last_execution_time_;
    //         time_diff_ms_ = time_diff.seconds() * 1000.0;
    //     }
    //     last_execution_time_ = current_time;
    // }
    
    void updateParameters() {
        param_manage->read_json();
        param_manage->set_normal_param();
        wheel_radius = param_manage->getWheelRadius();
        L = param_manage->getWheelbase();
        Lf = param_manage->getFrontLength();
        width = param_manage->getTrackWidth();
        deltaMax = param_manage->getMaxSteerAngle();
    }
    
    VehicleState updateVehicleState() {
        VehicleState state;
        
        // 기본 상태
        state.speed = lon_control->get_target_speed() / 3.6;
        state.yaw = cb_data->get_yaw();
        state.yaw_rate = cb_data->get_yawrate();
        state.position = cb_data->get_odom();
        
        // 전방 위치
        state.position_lf = {param_manage->getFrontLength(), 0.0};
        
        // 경로 관련 상태
        state.path_states.lateral_error = cb_data->calc_n_get_lat_error();
        state.path_states.lateral_error_lf = cb_data->calc_n_get_lat_error(state.position_lf);
        state.path_states.curvature = cb_data->calc_path_curvature_center(1.0);
        state.path_states.path_yaw = cb_data->get_pd_path_yaw();
        return state;
    }
    void calculateNormalControl(
        const VehicleState& state,
        ControlInput& control_input_kimm_sim,
        ControlInput& control_input_real_vehicle,
        ControlInput& control_input_scout,
        PointFR& steerAngle_kimm_sim,
        PointFR& steerAngle_real_vehicle,
        PointFR& steerAngle_scout,
        MotionCalculator::WheelSpeeds& wheel_speeds_kimm_sim,
        MotionCalculator::WheelSpeeds& wheel_speeds_real_vehicle,
        MotionCalculator::WheelSpeeds& wheel_speeds_scout
    ) {
        // 1. 기본 제어 입력 계산
        double target_speed = lon_control->get_target_speed() / 3.6;
        
        // 2. Lateral Control 업데이트
        lat_control->set_stanly_data(state.speed, 
                                    state.path_states.path_yaw, 
                                    state.yaw, 
                                    state.path_states.lateral_error);
        lat_control->set_curvature(state.path_states.curvature);
        lat_control->set_kimm_control_data(state.speed, 
                                         state.path_states.path_yaw, 
                                         state.yaw, 
                                         state.path_states.lateral_error, 
                                         state.path_states.lateral_error_lf);
        lat_control->set_odom(state.position.x, state.position.y);

        // 3. KIMM Simulation 제어 입력 계산
        control_input_real_vehicle = lat_control->calc_control_input_real_car();
        control_input_kimm_sim = lat_control->calc_control_input_sim();
        control_input_scout = control_input_real_vehicle;
  
        steerAngle_kimm_sim = MotionCalculator::calculate4WSAngles(
            control_input_kimm_sim,
            param_manage->getTrackWidth(),
            param_manage->getWheelbase(),
            param_manage->getMaxSteerAngle()
        );
        wheel_speeds_kimm_sim = MotionCalculator::calculateWheelSpeeds(
            control_input_kimm_sim,
            steerAngle_kimm_sim,
            param_manage->getTrackWidth(),
            param_manage->getWheelbase()
        );
    }

    void updateDockingMode(ControlMode current_mode) {
        // NORMAL -> DOCKING/ARRIVAL 전환 시에만 docking_mode를 1로 설정
        if (current_mode == ControlMode::DOCKING &&  control_mode_pre == ControlMode::NORMAL) {
            docking_mode = 1;
        }
        // NORMAL 모드일 때는 docking_mode를 0으로 설정
        else if (current_mode == ControlMode::NORMAL) {
            docking_mode = 0;
        }
        // docking_mode 상태 전이 로직
        else if (current_mode == ControlMode::DOCKING) {
            if (docking_mode == 1 && 
                std::fabs(normalizeAngle(lat_control->get_yr_error())) < param_manage->getYawThreshold()) {
                docking_mode = 2;
            }
            else if (docking_mode == 2 && 
                        sqrt(pow(lat_control->get_x_error(),2) + pow(lat_control->get_y_error(),2)) < 0.05) {
                docking_mode = 1;
            }
        }
        if (current_mode == ControlMode::ARRIVAL && control_mode_pre == ControlMode::NORMAL) {
            docking_mode = 1;
        }
        // NORMAL 모드일 때는 docking_mode를 0으로 설정
        else if (current_mode == ControlMode::NORMAL) {
            docking_mode = 0;
        }
        // docking_mode 상태 전이 로직
        else if (current_mode == ControlMode::ARRIVAL) {
            if (docking_mode == 1 && 
                std::fabs(normalizeAngle(lat_control->get_yr_error())) < param_manage->getYawThreshold()) {
                docking_mode = 2;
            }
            else if (docking_mode == 2 && 
                        sqrt(pow(lat_control->get_x_error(),2) + pow(lat_control->get_y_error(),2)) < 0.05) {
                docking_mode = 1;
            }
        }

        control_mode_pre = current_mode;
    }
    void calculateDockingControl(
            const VehicleState& state,
            ControlInput& control_input_kimm_sim,
            ControlInput& control_input_real_vehicle,
            PointFR& steerAngle_kimm_sim,
            PointFR& steerAngle_real_vehicle,
            ControlMode& current_mode,
            MotionCalculator::WheelSpeeds& wheel_speeds_kimm_sim,
            MotionCalculator::WheelSpeeds& wheel_speeds_real_vehicle,
            MotionCalculator::WheelSpeeds& wheel_speeds_scout
            ) {
        // 1. 도킹 목표점 설정
        lat_control->set_goal_point(
            cb_data->get_target_point_x(),
            cb_data->get_target_point_y(),
            cb_data->get_target_point_yaw()
        );
        lat_control->set_odom(state.position.x, state.position.y);

        // 2. 도킹 제어 입력 계산
        control_input_kimm_sim = lat_control->get_calc_docking_control_input();
        control_input_real_vehicle = lat_control->get_calc_docking_control_input();

        if (current_mode == ControlMode::ARRIVAL)
        {
            control_input_real_vehicle.yr = lat_control->calc_static_yaw_rate();
        }   
        // 3. 주행 방향 결정
        const double epsilon = 0.1;
        if (driving_state == DrivingState::FORWARD && control_input_real_vehicle.vx < -epsilon) {
            driving_state = DrivingState::REVERSE;
        }
        else if (driving_state == DrivingState::REVERSE && control_input_real_vehicle.vx > epsilon) {
            driving_state = DrivingState::FORWARD;
        }

        // 4. 주행 방향에 따른 속도 부호 조정
        double vx_sign = (driving_state == DrivingState::FORWARD) ? 1.0 : -1.0;
        control_input_real_vehicle.vx = std::fabs(control_input_real_vehicle.vx) * vx_sign;

        // 5. 최소 속도 처리
        if (std::fabs(control_input_real_vehicle.vx) < 0.05) {
            control_input_real_vehicle.vx = 0.0;
        }


        // 9. 도킹 모드에 따른 제어 입력 조정
        if (docking_mode == 1) {
            control_input_real_vehicle.vx = 0;
            control_input_real_vehicle.vy = 0;
        }
        else if (docking_mode == 2) {
            control_input_real_vehicle.yr = 0;
        }

        steerAngle_kimm_sim = MotionCalculator::calculate4WSAngles(
            control_input_kimm_sim,
            param_manage->getTrackWidth(),
            param_manage->getWheelbase(),
            param_manage->getMaxSteerAngle()
        );
        wheel_speeds_kimm_sim = MotionCalculator::calculateWheelSpeeds(
            control_input_kimm_sim,
            steerAngle_kimm_sim,
            param_manage->getTrackWidth(),
            param_manage->getWheelbase()
        );

    }

    void publishControlCommands(const ControlInput& control_input, const PointFR& steerAngle,
        const MotionCalculator::WheelSpeeds& wheel_speeds_kimm_sim) {
        
        // 1. 차량 데이터 발행
        publishCarData(steerAngle, wheel_speeds_kimm_sim);
        
        // 2. Twist 메시지 발행
        publishTwistCommand(control_input);
        
        // 3. Isaac 명령 발행
        publishIsaacCommands(steerAngle, wheel_speeds_kimm_sim);
    }

    void publishDebugInfo(const PointFR& steerAngle, ControlInput control_input_kimm_sim) {
        auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        msg->data.clear();

        msg->data = {
            lon_control->get_target_speed() / 3.6, //1
            cb_data->get_pd_path_yaw(), //2
            cb_data->calc_n_get_lat_error(), //3
            lat_control->get_x_error(), //4
            lat_control->get_y_error(), //5
            normalizeAngle(lat_control->get_yr_error()), //6
            control_input_kimm_sim.vx, //7
            control_input_kimm_sim.vy, //8
            control_input_kimm_sim.yr, //9
            steerAngle.F, //10
            steerAngle.R, //11
            steerAngle.FL, //12
            steerAngle.RL, //13
            cb_data->get_target_point_x(), //14
            cb_data->get_target_point_y(), //15
            cb_data->get_target_point_yaw() //16
        };

        tmp_data_pub->publish(*msg);
    }
    

    void publishCarData(const PointFR& steerAngle, const MotionCalculator::WheelSpeeds& wheel_speeds) {
        auto car_data_msg = std::make_shared<std_msgs::msg::Float32MultiArray>();

        // 휠 속도 데이터 추가
        car_data_msg->data.push_back(wheel_speeds.fl / wheel_radius);
        car_data_msg->data.push_back(wheel_speeds.fr / wheel_radius);
        car_data_msg->data.push_back(wheel_speeds.rl / wheel_radius);
        car_data_msg->data.push_back(wheel_speeds.rr / wheel_radius);

        // 조향각 데이터 추가
        car_data_msg->data.push_back(steerAngle.FL);      
        car_data_msg->data.push_back(steerAngle.FR);   
        car_data_msg->data.push_back(steerAngle.RL);    
        car_data_msg->data.push_back(steerAngle.RR);   

        ranger_data_pub->publish(*car_data_msg);
    }

    void publishTwistCommand(const ControlInput& control_input) {
        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        
        twist_msg->linear.x = control_input.vx;
        twist_msg->linear.y = control_input.vy;
        twist_msg->angular.z = control_input.yr;
        
        cmd_vel_pub->publish(*twist_msg);
    }

    void publishIsaacCommands(const PointFR& steerAngle, const MotionCalculator::WheelSpeeds& wheel_speeds) {
        auto issac_cmd_msgs = std::make_shared<sensor_msgs::msg::JointState>();

        // 현재 조향각과의 차이 계산
        double wheel_diff_FL = std::fabs(cb_data->get_wheel_steer_FL() - steerAngle.FL);
        double wheel_diff_FR = std::fabs(cb_data->get_wheel_steer_FR() - steerAngle.FR);
        double wheel_diff_RL = std::fabs(cb_data->get_wheel_steer_RL() - steerAngle.RL);
        double wheel_diff_RR = std::fabs(cb_data->get_wheel_steer_RR() - steerAngle.RR);

        // 조향 위치 설정
        std::vector<double> steering_position = {
            steerAngle.FL,   // Front Left
            steerAngle.FR,   // Front Right
            steerAngle.RL,   // Rear Left
            steerAngle.RR    // Rear Right
        };

        // 휠 속도 설정
        std::vector<double> wheel_velocity = {
            wheel_speeds.fl,  // Front Left
            wheel_speeds.fr,  // Front Right
            wheel_speeds.rl,  // Rear Left
            wheel_speeds.rr   // Rear Right
        };

        // Joint 이름 설정
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

        // 위치와 속도 데이터 설정
        issac_cmd_msgs->position = steering_position;
        issac_cmd_msgs->position.resize(8, NAN);
        issac_cmd_msgs->velocity.resize(8, NAN);
        std::copy(wheel_velocity.begin(), wheel_velocity.end(), issac_cmd_msgs->velocity.begin() + 4);

        // 오른쪽 휠 방향 반전
        // issac_cmd_msgs->velocity[5] = -issac_cmd_msgs->velocity[5];
        // issac_cmd_msgs->velocity[7] = -issac_cmd_msgs->velocity[7];

        // 조향각 차이가 큰 경우 휠 속도 0으로 설정
        if (std::max({wheel_diff_FL, wheel_diff_FR, wheel_diff_RL, wheel_diff_RR}) > 0.0872665) {
            std::fill(issac_cmd_msgs->velocity.begin() + 4, issac_cmd_msgs->velocity.end(), 0.0);
        }

        issac_cmd_pub->publish(*issac_cmd_msgs);
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


