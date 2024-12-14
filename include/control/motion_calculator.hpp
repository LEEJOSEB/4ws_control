#pragma once
#include "basic_types.hpp"
#include "control/math_utils.hpp"

class MotionCalculator {
public:
    struct WheelSpeeds {
        double fl, fr, rl, rr;
    };

    struct WheelVelocity {
        double vx;
        double vy;
    };

    static double optimizeSteeringAngle(double angle) {
        // Normalize angle to [-2π, 2π] range first
        while (angle > 2 * M_PI) angle -= 2 * M_PI;
        while (angle < -2 * M_PI) angle += 2 * M_PI;
        
        // If angle > π/2, subtract π
        if (angle > M_PI/2) {
            angle -= M_PI;
        }
        // If angle < -π/2, add π
        else if (angle < -M_PI/2) {
            angle += M_PI;
        }
        
        return angle;
    }
    // 개별 휠의 속도 계산
    static double calculateWheelSpeed(
        const ControlInput& input,
        double steer_angle,
        bool is_front,
        bool is_left,
        double track_width,   // 추가된 파라미터
        double wheelbase)     // 추가된 파라미터
    {
        double width_term = (is_left ? -1.0 : 1.0) * input.yr * cos(steer_angle) * track_width / 2;
        double length_term = (is_front ? 1.0 : -1.0) * input.yr * sin(steer_angle) * wheelbase / 2;
        
        return input.vx * cos(steer_angle) + width_term + length_term + input.vy * sin(steer_angle);
    }

    // 모든 휠의 속도를 한번에 계산
    static WheelSpeeds calculateWheelSpeeds(
        const ControlInput& input,
        const PointFR& steer_angles,
        double track_width,   // 추가된 파라미터
        double wheelbase)     // 추가된 파라미터
    {
        WheelSpeeds speeds;
        
        speeds.fl = calculateWheelSpeed(input, steer_angles.FL, true, true, track_width, wheelbase);   // Front Left
        speeds.fr = calculateWheelSpeed(input, steer_angles.FR, true, false, track_width, wheelbase);  // Front Right
        speeds.rl = calculateWheelSpeed(input, steer_angles.RL, false, true, track_width, wheelbase);  // Rear Left
        speeds.rr = calculateWheelSpeed(input, steer_angles.RR, false, false, track_width, wheelbase); // Rear Right
        
        return speeds;
    }

    // 각 휠에서의 속도 벡터 계산
    static WheelVelocity calculateWheelVelocityVector(
        const ControlInput& input,
        bool is_front,
        bool is_left,
        double track_width,   // 추가된 파라미터
        double wheelbase)     // 추가된 파라미터
    {
        WheelVelocity vel;
        
        // 횡방향 거리에 의한 속도 성분
        double width_term = (is_left ? -1.0 : 1.0) * input.yr * track_width / 2;
        
        // 종방향 거리에 의한 속도 성분
        double length_term = (is_front ? 1.0 : -1.0) * input.yr * wheelbase / 2;
        
        // 각 휠에서의 속도 계산
        vel.vx = input.vx + width_term;
        vel.vy = input.vy + length_term;
        
        return vel;
    }

    // 4륜 조향각 계산
    static PointFR calculate4WSAngles(
        const ControlInput& input,
        double track_width,   // 추가된 파라미터
        double wheelbase,     // 추가된 파라미터
        double max_steer_angle, // 추가된 파라미터
        bool normalize_and_clip = true)
    {
        PointFR angles;
        
        // 각 휠의 속도 벡터 계산
        WheelVelocity vel_fl = calculateWheelVelocityVector(input, true, true, track_width, wheelbase);   // Front Left
        WheelVelocity vel_fr = calculateWheelVelocityVector(input, true, false, track_width, wheelbase);  // Front Right
        WheelVelocity vel_rl = calculateWheelVelocityVector(input, false, true, track_width, wheelbase);  // Rear Left
        WheelVelocity vel_rr = calculateWheelVelocityVector(input, false, false, track_width, wheelbase); // Rear Right

        // 각 휠의 조향각 계산
        angles.FL = atan2(vel_fl.vy, vel_fl.vx);
        angles.FR = atan2(vel_fr.vy, vel_fr.vx);
        angles.RL = atan2(vel_rl.vy, vel_rl.vx);
        angles.RR = atan2(vel_rr.vy, vel_rr.vx);

        if (normalize_and_clip) {
            // 각도 정규화
            angles.FL = math_utils::normalizeAngle(angles.FL);
            angles.FR = math_utils::normalizeAngle(angles.FR);
            angles.RL = math_utils::normalizeAngle(angles.RL);
            angles.RR = math_utils::normalizeAngle(angles.RR);

            //각도 제한
            angles.FL = optimizeSteeringAngle(angles.FL);
            angles.FR = optimizeSteeringAngle(angles.FR);
            angles.RL = optimizeSteeringAngle(angles.RL);
            angles.RR = optimizeSteeringAngle(angles.RR);
        }

        // 전/후륜 대표 조향각 계산
        angles.F = (angles.FL + angles.FR) / 2.0;
        angles.R = (angles.RL + angles.RR) / 2.0;

        return angles;
    }
}; 