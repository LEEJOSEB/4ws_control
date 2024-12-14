#pragma once
#include <cmath>
#include "basic_types.hpp"

namespace math_utils {
    // 상수 정의
    constexpr double EPSILON = 1e-9;  // 허용 오차
    
    // 기본 수학 함수들
    inline bool isZero(double value) {
        return std::abs(value) < EPSILON;
    }

    inline double applyLPF(double input, double prev_output, double alpha) {
        if (isZero(input)) {
            return 0.0;
        }
        return alpha * input + (1.0 - alpha) * prev_output;
    }

    // 벡터 연산 관련 함수들
    inline double crossProduct(const Point& p1, const Point& p2) {
        return p1.x * p2.y - p1.y * p2.x;
    }

    inline double determineSide(const Point& reference, const Point& target, const Point& origin) {
        Point refVector = {reference.x - origin.x, reference.y - origin.y};
        Point targetVector = {target.x - origin.x, target.y - origin.y};
        return crossProduct(refVector, targetVector);
    }

    // 값 제한/변환 함수들
    inline double saturateAbs(double val, double s) {
        if (val >= 0 && val < s) {
            return s;
        }
        else if (val < 0 && val > -s) {
            return -s;
        }
        return val;
    }

    template <typename T>
    inline T clip(const T& value, const T& min_val, const T& max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    inline double lowPassFilter(double val, double pre_val, float alpha) {
        return val * alpha + pre_val * (1-alpha);
    }

    inline double normalizeAngle(double rad_angle) {
        double rad_ang = rad_angle;
        while (rad_ang > M_PI)
            rad_ang -= 2 * M_PI;
        while (rad_ang < -M_PI)
            rad_ang += 2 * M_PI;
        return rad_ang;
    }

    inline double signDeterminer(double df, double dr) {
        return (df - dr >= 0) ? 1.0 : -1.0;
    }
}  // namespace math_utils 