#pragma once
#include "control/callback_data_manage.hpp"

enum class ControlMode {
    NORMAL = 0,
    DOCKING = 1,
    ARRIVAL = 2,
    STOP = 3
};

enum class DrivingState {
    FORWARD,
    REVERSE
};

struct WheelStates {
    struct {
        double speed;    // m/s
        double steer;    // rad
    } FL, FR, RL, RR;
};

struct PathStates {
    double lateral_error;
    double lateral_error_lf;
    double curvature;
    double path_yaw;
};

struct VehicleState {
    double speed;        // vehicle speed
    double yaw;         // current yaw
    double yaw_rate;    // current yaw rate
    Point position;     // current position
    Point position_lf;  // front position
    WheelStates wheel_states;
    PathStates path_states;
    ControlMode control_mode;
    DrivingState driving_state;
};

// 기본 제어 명령 구조체
struct BaseControlCommands {
    double target_speed_x;
    double target_speed_y; 
    double target_yaw_rate;
    PointFR steering_angles;
    WheelSpeeds wheel_speeds;
};

// 일반 주행용 제어 명령
struct NormalControlCommands : public BaseControlCommands {
    double path_tracking_error;
    double heading_error;
};

// 도킹용 제어 명령
struct DockingControlCommands : public BaseControlCommands {
    double position_error_x;
    double position_error_y;
    double heading_error;
    DrivingState driving_direction;
    int docking_phase;  // 1: 정렬, 2: 접근
}; 