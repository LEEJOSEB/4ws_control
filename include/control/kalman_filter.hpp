#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <chrono>

#include "control/PIDController.hpp"
#include "control/callback_data_manage.hpp"

using namespace Eigen;
using namespace std;

class KalmanFilter2D 
{
private:
    double dt;
    std::chrono::time_point<std::chrono::steady_clock> now_time;
    std::chrono::time_point<std::chrono::steady_clock> old_time;
    typedef std::chrono::steady_clock Clock;
    Matrix4d A;
    Matrix<double, 4, 4> H;
    Matrix4d Q;
    Matrix<double, 4, 4> R;
    Vector4d x;
    Matrix4d P;
    Vector4d z;
    Vector2d zp;
    Vector4d xp;
    Matrix4d Pp;
    Matrix<double, 2, 4> Hp;
    Matrix<double, 2, 2> Rp;
    Matrix<double, 4, 4> K;
    Matrix<double, 4, 2> Kp;
    double px_;
    double py_;
public:
    // 생성자
    KalmanFilter2D(){
        // 초기화
        
        dt = 0.01;
        old_time = Clock::now();
        now_time = Clock::now();
        A << 1, dt, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, dt,
             0, 0, 0, 1;

        H << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        Hp << 0, 1, 0, 0,
             0, 0, 0, 1;


        Q << 2, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 2, 0,
             0, 0, 0, 1;

        R << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

        Rp << 1, 0,
                0, 1;

        x << 0, 0, 0, 0;

        P << 10, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 10, 0,
             0, 0, 0, 1;

        
    
    }
    void set_x_space(double px, double py){
        x << px, 0, py, 0;
    }
    // 예측 및 업데이트 함수
    void predictUpdate1(double vx, double vy) {
        // 예측 단계
        now_time = Clock::now();

        std::chrono::duration<double> elapsed_seconds = now_time - old_time;  
        double elapsed_seconds_double = elapsed_seconds.count();
        old_time = now_time;
        dt = elapsed_seconds_double;

        A << 1, dt, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, dt,
             0, 0, 0, 1;

        zp << vx, vy;
        

        xp = A * x;
        Pp = A * P * A.transpose() + Q;

        Hp << 0, 1, 0, 0,
                  0, 0, 0, 1;

        // Rp << 1, 0,
        //         0, 1;

        Kp = Pp * Hp.transpose() * (Hp * Pp * Hp.transpose() + Rp).inverse();
        x = xp + Kp * (zp - Hp * xp);
        P = Pp - Kp* Hp * Pp;

        // cout << "Kp * (zp - Hp * xp) : " << Kp * (zp - Hp * xp) << endl;
        

        
    }

    void predictUpdate2(double px, double py, double vx, double vy) {
        // 예측 단계
        
       
        now_time = Clock::now();

        std::chrono::duration<double> elapsed_seconds = now_time - old_time;  
        double elapsed_seconds_double = elapsed_seconds.count();
        old_time = now_time;
        dt = elapsed_seconds_double;

        A << 1, dt, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, dt,
             0, 0, 0, 1;
             
        z << px, vx, py, vy;

        xp = A * x;
        Pp = A * P * A.transpose() + Q;

        H << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

        K = Pp * H.transpose() * (H * Pp * H.transpose() + R).inverse();
        x = xp + K * (z - H * xp);
        P = Pp - K * H * Pp;
        

    }

    void set_Q_value(double q1, double q2)
    {
        Q << q1, 0, 0, 0,
             0, q2, 0, 0,
             0, 0, q1, 0,
             0, 0, 0, q2;
    }

    void set_P_value(double p1, double p2)
    {
        P << p1, 0, 0, 0,
             0, p2, 0, 0,
             0, 0, p1, 0,
             0, 0, 0, p2;
    }

    void set_R_value(double r1, double r2, double r3)
    {
        R << r1, 0, 0, 0,
             0, r2, 0, 0,
             0, 0, r1, 0,
             0, 0, 0, r2;

        Rp << r3, 0,
                0, r3;
    }

    void set_time(std::chrono::seconds timeInSeconds)
    {
        
    }
    // 상태 반환 함수
    Vector4d getState() const {
        return x;
    }


    double get_kf_vx()
    {
        return x(1);
    }

    double get_kf_vy()
    {
        return x(3);
    }

    double get_dt()
    {
        return dt;
    }
  
};


#endif // KALMAN_FILTER_HPP
