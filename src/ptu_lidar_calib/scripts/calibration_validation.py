#!/usr/bin/env python3
"""
Calibration Validation Sequence

Re-acquires scans at a fresh set of poses (different from calibration) and
evaluates the quality of the calibrated kinematic chain by measuring how
well all scans, when projected to the base frame, lie on a single plane.

For a perfect calibration, all projected points lie exactly on one plane.

Reports:
  - Per-capture point-to-plane RMSE (mm)
  - Aggregate statistics (mean, max, std)
  - Comparison: calibrated vs uncalibrated
  - PASS/FAIL verdict against user-defined threshold

Usage:
  1. Run calibration_acquisition.py + calibration_solver.py first
  2. Keep the device in front of the same wall (do NOT move it)
  3. rosrun ptu_lidar_calib calibration_validation.py \\
       --data <validation_data.npz> \\
       --delta <delta_values from solver output, or a .npy file>

  Or use the launch file which runs acquisition → solver → validation.
"""

import numpy as np
import math
import argparse
import sys
import os
import time

# Import the solver module (same package)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from calibration_solver import (
    forward_kinematics, load_captures, subsample_points
)

# Try to import ROS (validation can also be run offline)
try:
    import rospy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False


def _transform_points_to_base(captures, delta, max_pts=50):
    """
    Transform inlier points from each capture to the base frame using FK.
    Returns list of Nx3 arrays (one per capture).
    """
    all_base_pts = []
    for cap in captures:
        T_k = forward_kinematics(cap['pan_rad'], cap['tilt_rad'], delta)
        pts_2d = cap['inlier_points']
        # Subsample if too many
        if len(pts_2d) > max_pts:
            step = max(1, len(pts_2d) // max_pts)
            pts_2d = pts_2d[::step][:max_pts]
        ones = np.ones((len(pts_2d), 1))
        zeros = np.zeros((len(pts_2d), 1))
        pts_h = np.hstack([pts_2d, zeros, ones])  # Nx4  (x,y,0,1) in laser
        pts_base = (T_k @ pts_h.T).T[:, :3]  # Nx3
        all_base_pts.append(pts_base)
    return all_base_pts


def _fit_plane(point_arrays):
    """Fit a plane (n, d) to a list of Nx3 point arrays using SVD."""
    all_pts = np.vstack(point_arrays)  # Mx3
    centroid = all_pts.mean(axis=0)
    centered = all_pts - centroid
    _, S, Vt = np.linalg.svd(centered, full_matrices=False)
    normal = Vt[2]  # smallest singular value direction
    d = normal @ centroid
    return normal, d


def _compute_plane_errors(point_arrays, normal, d):
    """
    Compute per-capture RMSE and per-point errors for plane (n, d).
    Returns:
        per_capture_rmse_mm: list of floats
        all_errors_mm: flat array of all signed distances (mm)
    """
    per_capture_rmse = []
    all_errors = []
    for pts in point_arrays:
        dists = pts @ normal - d  # signed distances
        rmse = np.sqrt(np.mean(dists ** 2))
        per_capture_rmse.append(rmse * 1000.0)  # convert to mm
        all_errors.extend((dists * 1000.0).tolist())
    return per_capture_rmse, np.array(all_errors)


class CalibrationValidator:
    """
    Validation pipeline:
      1. Transform all scan points to base frame using FK(δ)
      2. Fit a global plane to all base-frame points
      3. Compute point-to-plane distances
      4. Report statistics and PASS/FAIL
    """

    def __init__(self, delta, plane_rmse_threshold_mm=1.0):
        """
        Args:
            delta: 18-element correction vector from the solver
            plane_rmse_threshold_mm: max acceptable global RMSE (mm)
        """
        self.delta = np.array(delta, dtype=float)
        self.plane_rmse_threshold_mm = plane_rmse_threshold_mm

    def validate_from_file(self, data_file, **kwargs):
        """
        Run validation on a pre-recorded dataset.
        This can be either the calibration data itself (self-validation)
        or a separately acquired validation dataset.
        """
        captures, fov = load_captures(data_file)
        return self._run_validation(captures, fov)

    def _run_validation(self, captures, fov):
        """Core validation logic."""
        print("\n" + "=" * 74)
        print("  CALIBRATION VALIDATION")
        print("=" * 74)
        print(f"  Captures:             {len(captures)}")
        print(f"  FOV half:             {fov:.1f}°")
        print(f"  Plane RMSE threshold: {self.plane_rmse_threshold_mm:.2f} mm")
        print(f"  δ norm:               {np.linalg.norm(self.delta):.6f}")

        if len(captures) < 2:
            print("ERROR: Need at least 2 captures for validation!")
            return None

        # ── Calibrated ──
        print("\n─── Calibrated (δ applied) ───")
        pts_cal = _transform_points_to_base(captures, self.delta)
        n_cal, d_cal = _fit_plane(pts_cal)
        rmse_cal, errs_cal = _compute_plane_errors(pts_cal, n_cal, d_cal)

        print(f"  Fitted plane: n=[{n_cal[0]:.4f}, {n_cal[1]:.4f}, {n_cal[2]:.4f}], d={d_cal:.4f} m")

        # ── Uncalibrated (δ=0) ──
        print("\n─── Uncalibrated (δ=0) ───")
        delta_zero = np.zeros(18)
        pts_uncal = _transform_points_to_base(captures, delta_zero)
        n_uncal, d_uncal = _fit_plane(pts_uncal)
        rmse_uncal, errs_uncal = _compute_plane_errors(pts_uncal, n_uncal, d_uncal)

        print(f"  Fitted plane: n=[{n_uncal[0]:.4f}, {n_uncal[1]:.4f}, {n_uncal[2]:.4f}], d={d_uncal:.4f} m")

        # ── Print comparison ──
        self._print_results(rmse_cal, errs_cal, rmse_uncal, errs_uncal,
                           len(captures))

        # ── PASS / FAIL ──
        global_rmse = np.sqrt(np.mean(errs_cal ** 2))
        passed = global_rmse <= self.plane_rmse_threshold_mm

        return {
            'passed': passed,
            'global_rmse_mm': global_rmse,
            'per_capture_rmse_mm': rmse_cal,
            'per_capture_rmse_uncal_mm': rmse_uncal,
            'plane_normal': n_cal,
            'plane_d': d_cal,
        }

    def _print_results(self, rmse_cal, errs_cal, rmse_uncal, errs_uncal, n_captures):
        """Print validation statistics."""

        # Per-capture details
        print("\n─── Per-Capture Plane RMSE (mm) ───")
        print(f"  {'Cap':>4s}  {'Uncalibrated':>14s}  {'Calibrated':>14s}  {'Improvement':>12s}")
        print(f"  {'─'*48}")
        for i in range(n_captures):
            imp = (1.0 - rmse_cal[i] / max(rmse_uncal[i], 1e-12)) * 100.0
            print(f"  {i:4d}  {rmse_uncal[i]:14.4f}  {rmse_cal[i]:14.4f}  {imp:+11.1f}%")

        # Aggregate
        rmse_cal_arr = np.array(rmse_cal)
        rmse_uncal_arr = np.array(rmse_uncal)

        global_rmse_cal = np.sqrt(np.mean(errs_cal ** 2))
        global_rmse_uncal = np.sqrt(np.mean(errs_uncal ** 2))

        print(f"\n─── Aggregate Statistics ───")
        print(f"  {'Metric':<30s} {'Uncalibrated':>14s} {'Calibrated':>14s} {'Improvement':>14s}")
        print(f"  {'─'*72}")

        items = [
            ("Global RMSE (mm)",       global_rmse_uncal,        global_rmse_cal),
            ("Per-capture RMSE mean",  np.mean(rmse_uncal_arr),  np.mean(rmse_cal_arr)),
            ("Per-capture RMSE std",   np.std(rmse_uncal_arr),   np.std(rmse_cal_arr)),
            ("Per-capture RMSE max",   np.max(rmse_uncal_arr),   np.max(rmse_cal_arr)),
            ("Max point error (mm)",   np.max(np.abs(errs_uncal)), np.max(np.abs(errs_cal))),
        ]
        for label, uncal, cal in items:
            imp = (1.0 - cal / max(uncal, 1e-12)) * 100.0
            print(f"  {label:<30s} {uncal:>14.4f} {cal:>14.4f} {imp:>+13.1f}%")

        # Verdict
        print(f"\n─── Verdict ───")
        print(f"  Global RMSE: {global_rmse_cal:.4f} mm "
              f"(threshold: {self.plane_rmse_threshold_mm:.2f} mm)  "
              f"{'✓ PASS' if global_rmse_cal <= self.plane_rmse_threshold_mm else '✗ FAIL'}")
        passed = global_rmse_cal <= self.plane_rmse_threshold_mm
        print(f"\n  Overall: {'✓ PASS' if passed else '✗ FAIL'}")
        print("=" * 74)


# ===================================================================== #
#  CLI Entry Point                                                        #
# ===================================================================== #

def main():
    parser = argparse.ArgumentParser(
        description='Calibration validation (Section 4.9)')
    parser.add_argument('--data', required=True,
                        help='Path to validation .npz file (from calibration_acquisition.py)')
    parser.add_argument('--delta', required=True,
                        help='Comma-separated 18 delta values, OR path to a .npy file')
    parser.add_argument('--plane-threshold', type=float, default=10.0,
                        help='Max acceptable global plane RMSE in mm (default: 10.0)')

    args = parser.parse_args()

    # Load delta
    if os.path.exists(args.delta):
        delta = np.load(args.delta)
        print(f"Loaded δ from {args.delta}")
    else:
        try:
            delta = np.array([float(x) for x in args.delta.split(',')])
        except ValueError:
            print(f"ERROR: Could not parse --delta. Provide 18 comma-separated floats or a .npy file.")
            sys.exit(1)

    if len(delta) != 18:
        print(f"ERROR: δ must have 18 elements, got {len(delta)}")
        sys.exit(1)

    if not os.path.exists(args.data):
        print(f"ERROR: File not found: {args.data}")
        sys.exit(1)

    validator = CalibrationValidator(delta,
                                     plane_rmse_threshold_mm=args.plane_threshold)
    result = validator.validate_from_file(args.data)

    if result is not None:
        sys.exit(0 if result['passed'] else 1)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()
