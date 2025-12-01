#include "hokuyo_go/modular_scanner/parametric_scanner.h"

namespace hokuyo_go
{

ParametricScanner::ParametricScanner(double v_target, double delta_1, double delta_2)
    : t_param_(0.0),
      v_target_(v_target),
      delta_1_(delta_1),
      delta_2_(delta_2),
      phi_offset_(0.0),
      theta_offset_(0.0),
      sqrt2_over_1000_(sqrt(2.0) / 1000.0)
{
    last_update_time_ = std::chrono::high_resolution_clock::now();
}

void ParametricScanner::setOffsets(double phi_offset, double theta_offset)
{
    phi_offset_ = phi_offset;
    theta_offset_ = theta_offset;
}

void ParametricScanner::getOffsets(double &phi_offset, double &theta_offset) const
{
    phi_offset = phi_offset_;
    theta_offset = theta_offset_;
}

void ParametricScanner::setPatternLimits(double delta_1, double delta_2)
{
    delta_1_ = delta_1;
    delta_2_ = delta_2;
}

void ParametricScanner::getPatternLimits(double &delta_1, double &delta_2) const
{
    delta_1 = delta_1_;
    delta_2 = delta_2_;
}

double ParametricScanner::f1(double t) const
{
    return phi_offset_ + delta_1_ * sin(t);
}

double ParametricScanner::f2(double t) const
{
    double freq = 3.0 + sqrt2_over_1000_;
    return theta_offset_ + (delta_2_ * 2) * (cos(freq * t) + 1.0) / 2.0 - delta_2_;
}

double ParametricScanner::df1_dt(double t, double dt) const
{
    return (f1(t + dt) - f1(t)) / dt;
}

double ParametricScanner::df2_dt(double t, double dt) const
{
    return (f2(t + dt) - f2(t)) / dt;
}

double ParametricScanner::calculateSpeed(double t, double dt) const
{
    double df1 = df1_dt(t, dt);
    double df2 = df2_dt(t, dt);
    return sqrt(df1 * df1 + df2 * df2);
}

void ParametricScanner::updateParameterTime()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt_elapsed = current_time - last_update_time_;
    double delta_t_actual = dt_elapsed.count();

    last_update_time_ = current_time;

    if (delta_t_actual < 0.001)
    {
        return;
    }

    double dt_small = 0.001;
    double current_speed = calculateSpeed(t_param_, dt_small);

    if (current_speed > 0.0)
    {
        double delta_s = v_target_ * delta_t_actual;
        double delta_t_param = delta_s / current_speed;
        t_param_ += delta_t_param;
    }
    else
    {
        t_param_ += delta_t_actual;
    }
}

void ParametricScanner::getTargetAngles(double &pan_target, double &tilt_target)
{
    pan_target = f1(t_param_);
    tilt_target = f2(t_param_);
}

void ParametricScanner::reset()
{
    // Start at t value where tilt is near zero (neutral position)
    // f2(t) = 0 when cos(freq * t) = 0, i.e., freq * t = PI/2
    // freq = 3.0 + sqrt2_over_1000_ ≈ 3.0
    // t = PI / (2 * freq) ≈ PI / 6 ≈ 0.524
    double freq = 3.0 + sqrt2_over_1000_;
    t_param_ = M_PI / (2.0 * freq);
    last_update_time_ = std::chrono::high_resolution_clock::now();
}

} // namespace hokuyo_go
