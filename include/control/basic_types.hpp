#pragma once

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

struct WheelSpeeds {
    double fl, fr, rl, rr;
};

struct GasAndBrake {
    double gas;
    double brake;
};

struct ControlInput {
    double vx;
    double vy;
    double yr;
}; 