// =========================================================================
//  ptu_calib_cpp  –  solve_node.cpp
//
//  Two-stage kinematic calibration solver.
//
//  STAGE 1  –  Tilt + laser calibration  (uses tilt-sweep data, pan ≈ 0)
//  ─────────────────────────────────────────────────────────────────────
//  Parameters: tilt_params[7]  = {dt_tilt(3), dr_tilt(3), enc_tilt(1)}
//              laser_params[6] = {dt_laser(3), dr_laser(3)}
//              plane[4]        = {nx, ny, nz, d}
//
//  Cost:  Varying tilt with pan locked near zero sweeps the horizontal
//         laser strip vertically on the wall.  The strips at different
//         heights define a plane, strongly constraining tilt & laser.
//         Pan corrections are zero (identity) – pan is nearly locked.
//
//  STAGE 2  –  Pan calibration  (uses pan-sweep data with tilt spread)
//  ─────────────────────────────────────────────────────────────────────
//  Parameters: pan_params[7] = {dt_pan(3), dr_pan(3), enc_pan(1)}
//              plane[4]      = {nx, ny, nz, d}
//
//  The tilt + laser corrections from Stage 1 are FROZEN.
//  Pan data is acquired with a spread of tilt angles (not tilt=0)
//  so the projected strips define a plane, not a degenerate line.
//
//  Both stages use Ceres auto-diff with Huber loss for robustness.
// =========================================================================

#include <ros/ros.h>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "ptu_calib_cpp/kinematic_model.h"
#include "ptu_calib_cpp/scan_capture.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <dirent.h>

namespace ptu_calib {

// =====================================================================
//  Subsample inlier points to keep the problem size manageable
// =====================================================================
static std::vector<std::vector<Eigen::Vector2d>>
subsample(const CaptureSet& cs, int max_per_capture = 30)
{
    std::vector<std::vector<Eigen::Vector2d>> out;
    for (auto& cap : cs.captures) {
        std::vector<Eigen::Vector2d> sub;
        int N = static_cast<int>(cap.inlier_pts.size());
        if (N <= max_per_capture) {
            sub = cap.inlier_pts;
        } else {
            int step = std::max(1, N / max_per_capture);
            for (int i = 0; i < N && static_cast<int>(sub.size()) < max_per_capture; i += step)
                sub.push_back(cap.inlier_pts[i]);
        }
        out.push_back(std::move(sub));
    }
    return out;
}

// =====================================================================
//  Initial plane estimate from uncalibrated FK
// =====================================================================
static void estimateInitialPlane(
    const CaptureSet& cs,
    const std::vector<std::vector<Eigen::Vector2d>>& sub,
    double n_out[3], double& d_out)
{
    std::vector<Eigen::Vector3d> all;
    for (size_t k = 0; k < cs.captures.size(); ++k) {
        Eigen::Matrix4d T = forward_kinematics_nominal(
            cs.captures[k].pan_rad, cs.captures[k].tilt_rad);
        for (auto& p2 : sub[k]) {
            Eigen::Vector4d ph(p2.x(), p2.y(), 0.0, 1.0);
            Eigen::Vector3d pb = (T * ph).head<3>();
            all.push_back(pb);
        }
    }
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (auto& p : all) centroid += p;
    centroid /= all.size();

    Eigen::MatrixXd M(all.size(), 3);
    for (size_t i = 0; i < all.size(); ++i)
        M.row(i) = (all[i] - centroid).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV);
    Eigen::Vector3d n = svd.matrixV().col(2);
    double d = n.dot(centroid);
    if (d < 0) { n = -n; d = -d; }

    n_out[0] = n.x(); n_out[1] = n.y(); n_out[2] = n.z();
    d_out = d;

    // Report uncalibrated RMSE
    double sse = 0;
    for (auto& p : all) { double e = n.dot(p) - d; sse += e*e; }
    double rmse_mm = std::sqrt(sse / all.size()) * 1000.0;
    ROS_INFO("  Initial plane: n=[%.4f, %.4f, %.4f]  d=%.4f m  uncal RMSE=%.2f mm",
             n.x(), n.y(), n.z(), d, rmse_mm);
}

// =====================================================================
//  STAGE 1  –  Tilt + laser cost functor  (pan corrections = identity)
// =====================================================================
struct TiltLaserPlaneCost {
    double theta_pan, theta_tilt;
    double px, py;  // 2D point in laser frame
    double d_norm;  // plane distance for range-normalisation (m)

    TiltLaserPlaneCost(double tp, double tt, double x, double y, double dn)
        : theta_pan(tp), theta_tilt(tt), px(x), py(y), d_norm(dn) {}

    template<typename T>
    bool operator()(const T* tilt_params,   // [7]
                    const T* laser_params,  // [6]
                    const T* plane,         // [4]: nx,ny,nz,d
                    T* residual) const
    {
        // Pan params = zero (identity corrections) since pan ≈ 0 in tilt data
        T pan_params[7] = {T(0),T(0),T(0),T(0),T(0),T(0),T(0)};

        auto Tfull = forward_kinematics_t<T>(
            T(theta_pan), T(theta_tilt),
            pan_params, tilt_params, laser_params);

        // Project 2D point
        Eigen::Matrix<T,4,1> ph;
        ph << T(px), T(py), T(0), T(1);
        Eigen::Matrix<T,4,1> pb = Tfull * ph;

        // Plane: n·p - d = 0
        T nx = plane[0], ny = plane[1], nz = plane[2], d = plane[3];
        T n_norm = ceres::sqrt(nx*nx + ny*ny + nz*nz + T(1e-12));
        T dist = (nx*pb(0) + ny*pb(1) + nz*pb(2) - d) / n_norm;

        // Range-normalised residual: milliradians (≈ angular error × 1000)
        // Makes the cost function invariant to wall distance.
        residual[0] = dist / T(d_norm) * T(1000.0);
        return true;
    }
};

// Regularization on any parameter block
struct RegCost {
    int idx;
    double weight;
    RegCost(int i, double w) : idx(i), weight(w) {}

    template<typename T>
    bool operator()(const T* params, T* residual) const {
        residual[0] = T(weight) * params[idx];
        return true;
    }
};

// =====================================================================
//  STAGE 2  –  Pan cost functor (tilt + laser corrections frozen)
// =====================================================================
struct PanPlaneCost {
    double theta_pan, theta_tilt;
    double px, py;
    double frozen_tilt[7];   // from Stage 1
    double frozen_laser[6];  // from Stage 1
    double d_norm;           // plane distance for range-normalisation (m)

    PanPlaneCost(double tp, double tt, double x, double y,
                 const double* tilt_result, const double* laser_result, double dn)
        : theta_pan(tp), theta_tilt(tt), px(x), py(y), d_norm(dn)
    {
        for (int i = 0; i < 7; ++i) frozen_tilt[i]  = tilt_result[i];
        for (int i = 0; i < 6; ++i) frozen_laser[i]  = laser_result[i];
    }

    template<typename T>
    bool operator()(const T* pan_params,   // [7]
                    const T* plane,        // [4]
                    T* residual) const
    {
        // Tilt & laser params are frozen constants from Stage 1
        T tilt_p[7];
        for (int i = 0; i < 7; ++i) tilt_p[i] = T(frozen_tilt[i]);
        T laser_p[6];
        for (int i = 0; i < 6; ++i) laser_p[i] = T(frozen_laser[i]);

        auto Tfull = forward_kinematics_t<T>(
            T(theta_pan), T(theta_tilt),
            pan_params, tilt_p, laser_p);

        Eigen::Matrix<T,4,1> ph;
        ph << T(px), T(py), T(0), T(1);
        Eigen::Matrix<T,4,1> pb = Tfull * ph;

        T nx = plane[0], ny = plane[1], nz = plane[2], d = plane[3];
        T n_norm = ceres::sqrt(nx*nx + ny*ny + nz*nz + T(1e-12));
        T dist = (nx*pb(0) + ny*pb(1) + nz*pb(2) - d) / n_norm;

        // Range-normalised residual: milliradians
        residual[0] = dist / T(d_norm) * T(1000.0);
        return true;
    }
};

// =====================================================================
//  solve_tilt_laser  –  Stage 1  (pan ≈ 0, so pan corrections = identity)
// =====================================================================
bool solve_tilt_laser(const CaptureSet& tilt_data,
                      double tilt_params_out[7],
                      double laser_params_out[6],
                      double reg_weight,
                      double enc_bound_rad,
                      double dt_bound,
                      double dr_bound)
{
    auto sub = subsample(tilt_data, 30);
    size_t n_caps = tilt_data.captures.size();

    ROS_INFO("======================================================");
    ROS_INFO("  STAGE 1: TILT + LASER CALIBRATION  (%zu captures)", n_caps);
    ROS_INFO("======================================================");

    if (n_caps < 3) {
        ROS_ERROR("Need >= 3 tilt captures");
        return false;
    }

    int total_pts = 0;
    for (auto& s : sub) total_pts += s.size();
    ROS_INFO("  Total subsampled points: %d", total_pts);

    // FK corrections accumulate across outer iterations
    double tilt_params[7] = {};
    double laser_params[6] = {};
    double plane[4];

    // Helper: re-estimate plane via SVD using current FK corrections
    auto refit_plane = [&]() {
        TransformCorrection tc; tc.from_array(tilt_params);
        TransformCorrection lc; lc.from_array(laser_params);
        std::vector<Eigen::Vector3d> all;
        for (size_t k = 0; k < n_caps; ++k) {
            Eigen::Matrix4d T = forward_kinematics(
                tilt_data.captures[k].pan_rad, tilt_data.captures[k].tilt_rad,
                TransformCorrection(), 0.0,
                tc, tilt_params[6], lc);
            for (auto& p2 : sub[k]) {
                Eigen::Vector4d ph(p2.x(), p2.y(), 0.0, 1.0);
                all.push_back((T * ph).head<3>());
            }
        }
        Eigen::Vector3d cen = Eigen::Vector3d::Zero();
        for (auto& p : all) cen += p;
        cen /= all.size();
        Eigen::MatrixXd M(all.size(), 3);
        for (size_t i = 0; i < all.size(); ++i)
            M.row(i) = (all[i] - cen).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV);
        Eigen::Vector3d n = svd.matrixV().col(2);
        double d = n.dot(cen);
        if (d < 0) { n = -n; d = -d; }
        plane[0] = n.x(); plane[1] = n.y(); plane[2] = n.z(); plane[3] = d;
        // RMSE
        double sse = 0;
        for (auto& p : all) { double e = n.dot(p) - d; sse += e*e; }
        return std::sqrt(sse / all.size()) * 1000.0;
    };

    // Initial plane fit (at zero corrections = nominal FK)
    double rmse0 = refit_plane();
    ROS_INFO("  Initial plane: n=[%.4f,%.4f,%.4f] d=%.4f  RMSE=%.2f mm",
             plane[0], plane[1], plane[2], plane[3], rmse0);

    // Range-normalisation: dividing residuals by d makes them angular
    // (milliradians), so the cost is invariant to wall distance.
    double d_norm = std::max(std::abs(plane[3]), 0.1);
    ROS_INFO("  Range normalisation distance: %.3f m (residuals in mrad)", d_norm);

    // -----------------------------------------------------------------
    // Regularization weights – scaled to match mrad-scale data residuals.
    //
    //   With range-normalised residuals in milliradians, the data cost
    //   is much smaller than with absolute mm.  Regularization must be
    //   weak enough to let the solver move.
    //
    //   trans_reg * 0.001m  should give a residual ~ 0.1 mrad
    //     ⇒ trans_reg ≈ 100
    //   rot_reg * 0.017rad should give a residual ~ 0.1 mrad
    //     ⇒ rot_reg  ≈ 6
    //
    //   With bounds in place, regularization is mainly to break
    //   degeneracies, not to prevent over-fitting.
    // -----------------------------------------------------------------
    double trans_reg = reg_weight * 10000.0;   // 100 at reg_weight=0.01
    double rot_reg   = reg_weight *   600.0;   //   6
    double enc_reg   = reg_weight *   600.0;   //   6

    // ── Alternating optimization: freeze plane → solve FK → refit plane ──
    //
    // LASER MOUNT is FROZEN throughout Stage 1.
    // The laser mount is a rigid mechanical assembly whose dimensions
    // are well-known from CAD.  Optimizing it simultaneously with the
    // tilt joint creates a severe degeneracy (13 params from 1 wall).
    // Freezing the laser eliminates this degeneracy and forces the
    // solver to attribute all error to the tilt joint origin.
    // -----------------------------------------------------------------
    ROS_INFO("  Laser mount params FROZEN at zero (not optimised in Stage 1)");
    const int outer_iters = 10;
    bool solve_ok = false;

    for (int outer = 0; outer < outer_iters; ++outer) {

        // Build a fresh Ceres problem each outer iteration
        ceres::Problem problem;

        // Cauchy loss on data residuals — 1.0 mrad threshold is ~1mm at 1m.
        // Beyond this, the cost grows sub-quadratically (robust to outliers).
        ceres::LossFunction* data_loss = new ceres::CauchyLoss(1.0);

        for (size_t k = 0; k < n_caps; ++k) {
            for (auto& pt : sub[k]) {
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<TiltLaserPlaneCost, 1, 7, 6, 4>(
                        new TiltLaserPlaneCost(
                            tilt_data.captures[k].pan_rad,
                            tilt_data.captures[k].tilt_rad,
                            pt.x(), pt.y(), d_norm)),
                    data_loss,
                    tilt_params, laser_params, plane);
            }
        }

        // Regularization toward zero  (pulls params toward zero = nominal)
        for (int i = 0; i < 3; ++i)
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegCost, 1, 7>(
                    new RegCost(i, trans_reg)), nullptr, tilt_params);
        for (int i = 3; i < 6; ++i)
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegCost, 1, 7>(
                    new RegCost(i, rot_reg)), nullptr, tilt_params);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RegCost, 1, 7>(
                new RegCost(6, enc_reg)), nullptr, tilt_params);

        // Bounds – tilt FK corrections
        for (int i = 0; i < 3; ++i) {
            problem.SetParameterLowerBound(tilt_params, i, -dt_bound);
            problem.SetParameterUpperBound(tilt_params, i,  dt_bound);
        }
        for (int i = 3; i < 6; ++i) {
            problem.SetParameterLowerBound(tilt_params, i, -dr_bound);
            problem.SetParameterUpperBound(tilt_params, i,  dr_bound);
        }
        problem.SetParameterLowerBound(tilt_params, 6, -enc_bound_rad);
        problem.SetParameterUpperBound(tilt_params, 6,  enc_bound_rad);

        // FREEZE the laser mount (not optimised in Stage 1)
        problem.SetParameterBlockConstant(laser_params);

        // FREEZE the plane – only tilt FK corrections are optimized
        problem.SetParameterBlockConstant(plane);

        // Solve
        ceres::Solver::Options opts;
        opts.linear_solver_type = ceres::DENSE_QR;
        opts.max_num_iterations = 500;
        opts.function_tolerance  = 1e-12;
        opts.parameter_tolerance = 1e-12;
        opts.gradient_tolerance  = 1e-12;
        opts.minimizer_progress_to_stdout = (outer == 0);  // verbose first iter only

        ceres::Solver::Summary summary;
        ceres::Solve(opts, &problem, &summary);
        solve_ok = summary.IsSolutionUsable();

        // Re-estimate plane from corrected FK
        double rmse = refit_plane();
        ROS_INFO("  Outer iter %d/%d: %d successful iters, cost %.4f→%.4f, RMSE=%.3f mm  (plane n=[%.4f,%.4f,%.4f] d=%.4f)",
                 outer+1, outer_iters, summary.num_successful_steps,
                 summary.initial_cost, summary.final_cost,
                 rmse, plane[0], plane[1], plane[2], plane[3]);

        if (!solve_ok) break;
    }

    for (int i = 0; i < 7; ++i) tilt_params_out[i] = tilt_params[i];
    for (int i = 0; i < 6; ++i) laser_params_out[i] = laser_params[i];

    // Report
    ROS_INFO("--- Tilt calibration results ---");
    ROS_INFO("  dt_tilt:   [%+.4f, %+.4f, %+.4f] mm",
             tilt_params[0]*1000, tilt_params[1]*1000, tilt_params[2]*1000);
    ROS_INFO("  dr_tilt:   [%+.4f, %+.4f, %+.4f] deg",
             tilt_params[3]*180/M_PI, tilt_params[4]*180/M_PI, tilt_params[5]*180/M_PI);
    ROS_INFO("  enc_tilt:  %+.4f deg", tilt_params[6]*180/M_PI);
    ROS_INFO("--- Laser mount results ---");
    ROS_INFO("  dt_laser:  [%+.4f, %+.4f, %+.4f] mm",
             laser_params[0]*1000, laser_params[1]*1000, laser_params[2]*1000);
    ROS_INFO("  dr_laser:  [%+.4f, %+.4f, %+.4f] deg",
             laser_params[3]*180/M_PI, laser_params[4]*180/M_PI, laser_params[5]*180/M_PI);
    ROS_INFO("  plane:     n=[%.4f, %.4f, %.4f]  d=%.4f m",
             plane[0], plane[1], plane[2], plane[3]);

    // Final RMSE (from last refit_plane)
    TransformCorrection tc; tc.from_array(tilt_params);
    TransformCorrection lc; lc.from_array(laser_params);
    double sse = 0; int cnt = 0;
    Eigen::Vector3d nv(plane[0], plane[1], plane[2]); nv.normalize();
    double d_n = plane[3] / Eigen::Vector3d(plane[0],plane[1],plane[2]).norm();
    for (size_t k = 0; k < n_caps; ++k) {
        Eigen::Matrix4d T = forward_kinematics(
            tilt_data.captures[k].pan_rad, tilt_data.captures[k].tilt_rad,
            TransformCorrection(), 0.0, tc, tilt_params[6], lc);
        for (auto& p2 : sub[k]) {
            Eigen::Vector4d ph(p2.x(), p2.y(), 0, 1);
            Eigen::Vector3d pb = (T * ph).head<3>();
            double e = nv.dot(pb) - d_n;
            sse += e*e; ++cnt;
        }
    }
    ROS_INFO("  Tilt+Laser calibrated plane RMSE: %.3f mm (%d pts)",
             std::sqrt(sse / cnt) * 1000.0, cnt);

    return solve_ok;
}

// =====================================================================
//  solve_pan  –  Stage 2  (tilt + laser corrections frozen from Stage 1)
// =====================================================================
bool solve_pan(const CaptureSet& pan_data,
               const double frozen_tilt[7],
               const double frozen_laser[6],
               double pan_params_out[7],
               double reg_weight,
               double enc_bound_rad,
               double dt_bound,
               double dr_bound)
{
    auto sub = subsample(pan_data, 30);
    size_t n_caps = pan_data.captures.size();

    ROS_INFO("======================================================");
    ROS_INFO("  STAGE 2: PAN CALIBRATION  (%zu captures)", n_caps);
    ROS_INFO("======================================================");

    if (n_caps < 3) {
        ROS_ERROR("Need >= 3 pan captures");
        return false;
    }

    int total_pts = 0;
    for (auto& s : sub) total_pts += s.size();
    ROS_INFO("  Total subsampled points: %d", total_pts);
    ROS_INFO("  Frozen tilt params: dt=[%.4f,%.4f,%.4f]mm  dr=[%.4f,%.4f,%.4f]deg  enc=%.4fdeg",
             frozen_tilt[0]*1000, frozen_tilt[1]*1000, frozen_tilt[2]*1000,
             frozen_tilt[3]*180/M_PI, frozen_tilt[4]*180/M_PI, frozen_tilt[5]*180/M_PI,
             frozen_tilt[6]*180/M_PI);
    ROS_INFO("  Frozen laser params: dt=[%.4f,%.4f,%.4f]mm  dr=[%.4f,%.4f,%.4f]deg",
             frozen_laser[0]*1000, frozen_laser[1]*1000, frozen_laser[2]*1000,
             frozen_laser[3]*180/M_PI, frozen_laser[4]*180/M_PI, frozen_laser[5]*180/M_PI);

    // Frozen tilt+laser corrections for FK
    TransformCorrection tc; tc.from_array(frozen_tilt);
    double enc_tilt = frozen_tilt[6];
    TransformCorrection lc; lc.from_array(frozen_laser);

    double pan_params[7] = {};
    double plane[4];

    // Helper: re-estimate plane via SVD using current pan FK corrections
    auto refit_plane = [&]() {
        TransformCorrection pc; pc.from_array(pan_params);
        std::vector<Eigen::Vector3d> all;
        for (size_t k = 0; k < n_caps; ++k) {
            Eigen::Matrix4d T = forward_kinematics(
                pan_data.captures[k].pan_rad, pan_data.captures[k].tilt_rad,
                pc, pan_params[6], tc, enc_tilt, lc);
            for (auto& p2 : sub[k]) {
                Eigen::Vector4d ph(p2.x(), p2.y(), 0.0, 1.0);
                all.push_back((T * ph).head<3>());
            }
        }
        Eigen::Vector3d cen = Eigen::Vector3d::Zero();
        for (auto& p : all) cen += p;
        cen /= all.size();
        Eigen::MatrixXd M(all.size(), 3);
        for (size_t i = 0; i < all.size(); ++i)
            M.row(i) = (all[i] - cen).transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV);
        Eigen::Vector3d n = svd.matrixV().col(2);
        double d = n.dot(cen);
        if (d < 0) { n = -n; d = -d; }
        plane[0] = n.x(); plane[1] = n.y(); plane[2] = n.z(); plane[3] = d;
        double sse = 0;
        for (auto& p : all) { double e = n.dot(p) - d; sse += e*e; }
        return std::sqrt(sse / all.size()) * 1000.0;
    };

    double rmse0 = refit_plane();
    ROS_INFO("  Initial plane: n=[%.4f,%.4f,%.4f] d=%.4f  RMSE=%.2fmm",
             plane[0], plane[1], plane[2], plane[3], rmse0);

    // Range-normalisation distance (same logic as Stage 1)
    double d_norm = std::max(std::abs(plane[3]), 0.1);
    ROS_INFO("  Range normalisation distance: %.3f m (residuals in mrad)", d_norm);

    // Regularization weights – scaled to match mrad-scale data residuals
    // (same scale as Stage 1)
    double trans_reg = reg_weight * 10000.0;   // 100 at reg_weight=0.01
    double rot_reg   = reg_weight *   600.0;   //   6
    double enc_reg   = reg_weight *   600.0;   //   6

    // ── Alternating optimization: freeze plane → solve FK → refit plane ──
    const int outer_iters = 10;
    bool solve_ok = false;

    for (int outer = 0; outer < outer_iters; ++outer) {

        ceres::Problem problem;

        // Cauchy loss on data residuals (same as Stage 1)
        ceres::LossFunction* data_loss = new ceres::CauchyLoss(1.0);

        for (size_t k = 0; k < n_caps; ++k) {
            for (auto& pt : sub[k]) {
                problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<PanPlaneCost, 1, 7, 4>(
                        new PanPlaneCost(pan_data.captures[k].pan_rad,
                                        pan_data.captures[k].tilt_rad,
                                        pt.x(), pt.y(),
                                        frozen_tilt, frozen_laser, d_norm)),
                    data_loss,
                    pan_params, plane);
            }
        }

        for (int i = 0; i < 3; ++i)
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegCost, 1, 7>(
                    new RegCost(i, trans_reg)), nullptr, pan_params);
        for (int i = 3; i < 6; ++i)
            problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<RegCost, 1, 7>(
                    new RegCost(i, rot_reg)), nullptr, pan_params);
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<RegCost, 1, 7>(
                new RegCost(6, enc_reg)), nullptr, pan_params);

        // Bounds
        for (int i = 0; i < 3; ++i) {
            problem.SetParameterLowerBound(pan_params, i, -dt_bound);
            problem.SetParameterUpperBound(pan_params, i,  dt_bound);
        }
        for (int i = 3; i < 6; ++i) {
            problem.SetParameterLowerBound(pan_params, i, -dr_bound);
            problem.SetParameterUpperBound(pan_params, i,  dr_bound);
        }
        problem.SetParameterLowerBound(pan_params, 6, -enc_bound_rad);
        problem.SetParameterUpperBound(pan_params, 6,  enc_bound_rad);

        // FREEZE the plane
        problem.SetParameterBlockConstant(plane);

        ceres::Solver::Options opts;
        opts.linear_solver_type = ceres::DENSE_QR;
        opts.max_num_iterations = 500;
        opts.function_tolerance  = 1e-12;
        opts.parameter_tolerance = 1e-12;
        opts.gradient_tolerance  = 1e-12;
        opts.minimizer_progress_to_stdout = (outer == 0);

        ceres::Solver::Summary summary;
        ceres::Solve(opts, &problem, &summary);
        solve_ok = summary.IsSolutionUsable();

        double rmse = refit_plane();
        ROS_INFO("  Outer iter %d/%d: %d successful iters, cost %.4f→%.4f, RMSE=%.3f mm",
                 outer+1, outer_iters, summary.num_successful_steps,
                 summary.initial_cost, summary.final_cost, rmse);

        if (!solve_ok) break;
    }

    for (int i = 0; i < 7; ++i) pan_params_out[i] = pan_params[i];

    ROS_INFO("--- Pan calibration results ---");
    ROS_INFO("  dt_pan:  [%+.4f, %+.4f, %+.4f] mm",
             pan_params[0]*1000, pan_params[1]*1000, pan_params[2]*1000);
    ROS_INFO("  dr_pan:  [%+.4f, %+.4f, %+.4f] deg",
             pan_params[3]*180/M_PI, pan_params[4]*180/M_PI, pan_params[5]*180/M_PI);
    ROS_INFO("  enc_pan: %+.4f deg", pan_params[6]*180/M_PI);
    ROS_INFO("  plane:   n=[%.4f, %.4f, %.4f]  d=%.4f m",
             plane[0], plane[1], plane[2], plane[3]);

    // Final RMSE
    TransformCorrection pc; pc.from_array(pan_params);
    double sse = 0; int cnt = 0;
    Eigen::Vector3d nv(plane[0], plane[1], plane[2]); nv.normalize();
    double d_n = plane[3] / Eigen::Vector3d(plane[0],plane[1],plane[2]).norm();
    for (size_t k = 0; k < n_caps; ++k) {
        Eigen::Matrix4d T = forward_kinematics(
            pan_data.captures[k].pan_rad, pan_data.captures[k].tilt_rad,
            pc, pan_params[6], tc, enc_tilt, lc);
        for (auto& p2 : sub[k]) {
            Eigen::Vector4d ph(p2.x(), p2.y(), 0, 1);
            Eigen::Vector3d pb = (T * ph).head<3>();
            double e = nv.dot(pb) - d_n;
            sse += e*e; ++cnt;
        }
    }
    ROS_INFO("  Pan-calibrated plane RMSE: %.3f mm (%d pts)",
             std::sqrt(sse / cnt) * 1000.0, cnt);

    return solve_ok;
}

// =====================================================================
//  Full two-stage solver  (TILT+LASER first, then PAN)
// =====================================================================
bool solve_two_stage(const CaptureSet& pan_data,
                     const CaptureSet& tilt_data,
                     CalibResult& result,
                     double reg_weight = 0.01,
                     double enc_bound_deg = 8.0,
                     double dt_bound_m = 0.002,
                     double dr_bound_rad = 0.035)
{
    double enc_bound_rad = enc_bound_deg * M_PI / 180.0;

    // Stage 1: Tilt + Laser  (pan corrections = identity)
    double tilt_params[7] = {};
    double laser_params[6] = {};
    if (!solve_tilt_laser(tilt_data, tilt_params, laser_params,
                          reg_weight, enc_bound_rad, dt_bound_m, dr_bound_rad)) {
        ROS_ERROR("Stage 1 (tilt+laser) failed!");
        return false;
    }

    // Stage 2: Pan  (tilt+laser frozen from Stage 1)
    double pan_params[7] = {};
    if (!solve_pan(pan_data, tilt_params, laser_params, pan_params,
                   reg_weight, enc_bound_rad, dt_bound_m, dr_bound_rad)) {
        ROS_ERROR("Stage 2 (pan) failed!");
        return false;
    }

    // Pack result
    result.pan_corr.from_array(pan_params);
    result.pan_encoder_zero = pan_params[6];
    result.tilt_corr.from_array(tilt_params);
    result.tilt_encoder_zero = tilt_params[6];
    result.laser_corr.from_array(laser_params);

    return true;
}

// =====================================================================
//  Validation: compute plane RMSE on a dataset using the full calib
// =====================================================================
double validate(const CaptureSet& data, const CalibResult& cal,
                const std::string& label = "validation")
{
    auto sub = subsample(data, 50);

    // Project all points
    std::vector<Eigen::Vector3d> all_pts;
    for (size_t k = 0; k < data.captures.size(); ++k) {
        Eigen::Matrix4d T = forward_kinematics(
            data.captures[k].pan_rad, data.captures[k].tilt_rad, cal);
        for (auto& p2 : sub[k]) {
            Eigen::Vector4d ph(p2.x(), p2.y(), 0, 1);
            all_pts.push_back((T * ph).head<3>());
        }
    }

    if (all_pts.size() < 4) {
        ROS_WARN("Not enough points for validation");
        return 1e9;
    }

    // Fit plane
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (auto& p : all_pts) centroid += p;
    centroid /= all_pts.size();

    Eigen::MatrixXd M(all_pts.size(), 3);
    for (size_t i = 0; i < all_pts.size(); ++i)
        M.row(i) = (all_pts[i] - centroid).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinV);
    Eigen::Vector3d n = svd.matrixV().col(2);
    double d = n.dot(centroid);

    double sse = 0;
    for (auto& p : all_pts) { double e = n.dot(p)-d; sse += e*e; }
    double rmse_mm = std::sqrt(sse / all_pts.size()) * 1000.0;

    // Uncalibrated comparison
    CalibResult zero;
    std::vector<Eigen::Vector3d> uncal_pts;
    for (size_t k = 0; k < data.captures.size(); ++k) {
        Eigen::Matrix4d T = forward_kinematics(
            data.captures[k].pan_rad, data.captures[k].tilt_rad, zero);
        for (auto& p2 : sub[k]) {
            Eigen::Vector4d ph(p2.x(), p2.y(), 0, 1);
            uncal_pts.push_back((T * ph).head<3>());
        }
    }
    Eigen::Vector3d uc_cen = Eigen::Vector3d::Zero();
    for (auto& p : uncal_pts) uc_cen += p;
    uc_cen /= uncal_pts.size();
    Eigen::MatrixXd Mu(uncal_pts.size(), 3);
    for (size_t i = 0; i < uncal_pts.size(); ++i)
        Mu.row(i) = (uncal_pts[i] - uc_cen).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svdu(Mu, Eigen::ComputeThinV);
    Eigen::Vector3d nu = svdu.matrixV().col(2);
    double du = nu.dot(uc_cen);
    double sse_u = 0;
    for (auto& p : uncal_pts) { double e = nu.dot(p)-du; sse_u += e*e; }
    double rmse_uncal = std::sqrt(sse_u / uncal_pts.size()) * 1000.0;

    ROS_INFO("======================================================");
    ROS_INFO("  VALIDATION (%s): %zu captures, %zu points",
             label.c_str(), data.captures.size(), all_pts.size());
    ROS_INFO("  Uncalibrated plane RMSE: %.3f mm", rmse_uncal);
    ROS_INFO("  Calibrated   plane RMSE: %.3f mm", rmse_mm);
    ROS_INFO("  Improvement: %.1f%%",
             (1.0 - rmse_mm / std::max(rmse_uncal, 1e-12)) * 100.0);
    ROS_INFO("======================================================");

    return rmse_mm;
}

}  // namespace ptu_calib

// =====================================================================
//  main – standalone solver (reads CSV files)
// =====================================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "solve_node");
    ros::NodeHandle nh, pnh("~");

    std::string pan_csv, tilt_csv;
    pnh.param<std::string>("pan_data",  pan_csv,  "");
    pnh.param<std::string>("tilt_data", tilt_csv, "");

    double reg_weight, enc_bound_deg, dt_bound_mm, dr_bound_deg;
    pnh.param("reg_weight",    reg_weight,    0.01);
    pnh.param("enc_bound_deg", enc_bound_deg, 8.0);
    pnh.param("dt_bound_mm",   dt_bound_mm,   2.0);
    pnh.param("dr_bound_deg",  dr_bound_deg,  2.0);

    std::string output_dir;
    pnh.param<std::string>("output_dir", output_dir, ".");

    // --- auto-detect latest CSVs when paths not provided ---
    auto find_latest_csv = [](const std::string& dir, const std::string& prefix) -> std::string {
        std::vector<std::string> matches;
        DIR* dp = opendir(dir.c_str());
        if (!dp) return "";
        while (struct dirent* entry = readdir(dp)) {
            std::string name(entry->d_name);
            if (name.size() > prefix.size() &&
                name.substr(0, prefix.size()) == prefix &&
                name.size() > 4 && name.substr(name.size()-4) == ".csv")
            {
                matches.push_back(name);
            }
        }
        closedir(dp);
        if (matches.empty()) return "";
        std::sort(matches.begin(), matches.end());
        return dir + "/" + matches.back();   // latest by timestamp in filename
    };

    if (pan_csv.empty()) {
        pan_csv = find_latest_csv(output_dir, "pan_data_");
        if (pan_csv.empty()) {
            ROS_ERROR("No pan_data_*.csv found in %s – provide ~pan_data", output_dir.c_str());
            return 1;
        }
        ROS_INFO("Auto-selected pan  CSV: %s", pan_csv.c_str());
    }
    if (tilt_csv.empty()) {
        tilt_csv = find_latest_csv(output_dir, "tilt_data_");
        if (tilt_csv.empty()) {
            ROS_ERROR("No tilt_data_*.csv found in %s – provide ~tilt_data", output_dir.c_str());
            return 1;
        }
        ROS_INFO("Auto-selected tilt CSV: %s", tilt_csv.c_str());
    }

    ROS_INFO("Loading pan  data: %s", pan_csv.c_str());
    ROS_INFO("Loading tilt data: %s", tilt_csv.c_str());

    ptu_calib::CaptureSet pan_data, tilt_data;
    try {
        pan_data  = ptu_calib::load_captures_csv(pan_csv);
        tilt_data = ptu_calib::load_captures_csv(tilt_csv);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to load data: %s", e.what());
        return 1;
    }

    ROS_INFO("Pan  captures: %zu", pan_data.captures.size());
    ROS_INFO("Tilt captures: %zu", tilt_data.captures.size());

    ptu_calib::CalibResult result;
    double dt_bound_m   = dt_bound_mm / 1000.0;
    double dr_bound_rad = dr_bound_deg * M_PI / 180.0;

    if (!ptu_calib::solve_two_stage(pan_data, tilt_data, result,
                                     reg_weight, enc_bound_deg,
                                     dt_bound_m, dr_bound_rad))
    {
        ROS_ERROR("Calibration failed!");
        return 1;
    }

    // Validate on both datasets
    ptu_calib::validate(pan_data,  result, "pan_data");
    ptu_calib::validate(tilt_data, result, "tilt_data");

    // Print combined results
    ROS_INFO("======================================================");
    ROS_INFO("  COMBINED CALIBRATION RESULT");
    ROS_INFO("======================================================");
    ROS_INFO("  Pan  encoder zero: %+.4f deg", result.pan_encoder_zero  * 180/M_PI);
    ROS_INFO("  Tilt encoder zero: %+.4f deg", result.tilt_encoder_zero * 180/M_PI);

    // Generate URDF
    std::string urdf = ptu_calib::generate_calibrated_urdf(result);
    ROS_INFO("\n%s", urdf.c_str());

    // Save URDF
    std::string urdf_path = output_dir + "/calibrated_pantilt.urdf";
    {
        std::ofstream f(urdf_path);
        if (f.is_open()) {
            f << urdf;
            ROS_INFO("Saved calibrated URDF → %s", urdf_path.c_str());
        } else {
            ROS_ERROR("Could not write %s", urdf_path.c_str());
        }
    }

    return 0;
}
