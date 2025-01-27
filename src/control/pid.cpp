#include "pid.h"
#include <algorithm>
#include <cmath>

double PID::update(double measurement, double desired) {
    double error = desired - measurement;
    double p = params_.kP * error;
    integralError += error * params_.dt;
    double i = params_.kI * integralError;
    double d = params_.kD * (error - prevError_)/ params_.dt;
    prevError_ = error;

    double u = p + i + d;
    if (params_.enable_ramp_rate_limit) {
        double delta_u = u - setU_;
        double max_delta_u = params_.ramp_rate * params_.dt;
        delta_u = std::clamp(delta_u, -max_delta_u, max_delta_u);
        u = setU_ + delta_u;
        setU_ = u;
    }

    return std::clamp(u, params_.kUMin, params_.kUMax);
}

PIDParams PID::defaultParams() {
  PIDParams p{};
  p.kD = 0.0;
  p.kD = 0.0;
  p.dt = 0.0;
  p.kUMax = 0.0;
  p.kUMin = 0.0;
  p.enable_ramp_rate_limit = false;
  p.ramp_rate = 0.0;
  return p;
}