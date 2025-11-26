#ifndef HOKUYO_GO_PARAMETRIC_SCANNER_H
#define HOKUYO_GO_PARAMETRIC_SCANNER_H

#include <cmath>
#include <chrono>

namespace hokuyo_go
{

class ParametricScanner
{
public:
    ParametricScanner(double v_target, double delta_1, double delta_2);

    // Set offsets for scanning pattern center
    void setOffsets(double phi_offset, double theta_offset);
    
    // Get current offsets
    void getOffsets(double &phi_offset, double &theta_offset) const;

    // Update parameter time for constant velocity scanning
    void updateParameterTime();

    // Get target angles using parametric scanning
    void getTargetAngles(double &pan_target, double &tilt_target);

    // Reset parameter time
    void reset();

    // Get current parameter time
    double getParameterTime() const { return t_param_; }

    // Setters for pattern limits
    void setPatternLimits(double delta_1, double delta_2);
    void getPatternLimits(double &delta_1, double &delta_2) const;

private:
    double t_param_;                // Parameter time for scanning equations
    double v_target_;               // Target velocity in rad/s
    double delta_1_;                // Pan limit
    double delta_2_;                // Tilt limit
    double phi_offset_;             // Phase offset for scanning pattern
    double theta_offset_;           // Additional tilt offset
    double sqrt2_over_1000_;        // Irrational frequency component
    std::chrono::high_resolution_clock::time_point last_update_time_;

    // Parametric functions
    double f1(double t) const;
    double f2(double t) const;

    // Derivative functions for velocity calculation
    double df1_dt(double t, double dt) const;
    double df2_dt(double t, double dt) const;

    // Calculate instantaneous speed for constant velocity implementation
    double calculateSpeed(double t, double dt) const;
};

} // namespace hokuyo_go

#endif // HOKUYO_GO_PARAMETRIC_SCANNER_H
