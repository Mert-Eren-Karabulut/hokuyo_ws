#!/usr/bin/env python3
"""
Full Calibration Pipeline Orchestrator

Runs the complete calibration workflow:
  Phase 1: Acquire calibration scans (grid-scan)
  Phase 2: Solve for 18-parameter kinematic correction δ*
  Phase 3: Acquire validation scans (different grid)
  Phase 4: Validate calibration quality

Usage:
  rosrun ptu_lidar_calib calibration_pipeline.py

  Or step by step:
    rosrun ptu_lidar_calib calibration_acquisition.py   (Phase 1)
    rosrun ptu_lidar_calib calibration_solver.py --data <file>  (Phase 2, offline)
    rosrun ptu_lidar_calib calibration_acquisition.py   (Phase 3, different grid)
    rosrun ptu_lidar_calib calibration_validation.py --data <file> --delta <values>  (Phase 4, offline)
"""

import numpy as np
import rospy
import sys
import os
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from calibration_acquisition import CalibrationAcquisition
from calibration_solver import solve as run_solver
from calibration_validation import CalibrationValidator


def main():
    rospy.init_node('calibration_pipeline', anonymous=False)

    # ── Parameters ─────────────────────────────────────────────
    # Calibration grid
    cal_pan_limit = rospy.get_param('~cal_pan_limit_deg', 25.0)
    cal_tilt_limit = rospy.get_param('~cal_tilt_limit_deg', 15.0)
    cal_pan_steps = rospy.get_param('~cal_pan_steps', 5)
    cal_tilt_steps = rospy.get_param('~cal_tilt_steps', 5)

    # Validation grid (different from calibration to test generalization)
    val_pan_limit = rospy.get_param('~val_pan_limit_deg', 20.0)
    val_tilt_limit = rospy.get_param('~val_tilt_limit_deg', 12.0)
    val_pan_steps = rospy.get_param('~val_pan_steps', 4)
    val_tilt_steps = rospy.get_param('~val_tilt_steps', 4)

    # Shared parameters
    fov_half_deg = rospy.get_param('~fov_half_deg', 15.0)
    settle_time = rospy.get_param('~settle_time', 1.5)
    lam = rospy.get_param('~regularization_lambda', 0.01)
    max_pairs = rospy.get_param('~max_pairs', 200)
    plane_weight = rospy.get_param('~plane_weight', 5.0)

    # Thresholds
    plane_rmse_threshold = rospy.get_param('~plane_rmse_threshold_mm', 10.0)

    # Output
    output_dir = rospy.get_param('~output_dir',
                                  os.path.join(SCRIPT_DIR, '..', 'results'))
    os.makedirs(output_dir, exist_ok=True)

    ts = time.strftime('%Y%m%d_%H%M%S')

    print("\n" + "█" * 74)
    print("█  FULL CALIBRATION PIPELINE")
    print("█" * 74)
    print(f"█  Calibration grid: {cal_pan_steps}×{cal_tilt_steps} "
          f"(pan ±{cal_pan_limit}°, tilt ±{cal_tilt_limit}°)")
    print(f"█  Validation grid:  {val_pan_steps}×{val_tilt_steps} "
          f"(pan ±{val_pan_limit}°, tilt ±{val_tilt_limit}°)")
    print(f"█  FOV half:         {fov_half_deg}°")
    print(f"█  Output dir:       {output_dir}")
    print("█" * 74 + "\n")

    # ══════════════════════════════════════════════════════════════
    #  PHASE 1: Calibration Data Acquisition
    # ══════════════════════════════════════════════════════════════
    print("\n" + "▓" * 74)
    print("▓  PHASE 1: CALIBRATION DATA ACQUISITION")
    print("▓" * 74 + "\n")

    # Set params for acquisition node
    rospy.set_param('~pan_limit_deg', cal_pan_limit)
    rospy.set_param('~tilt_limit_deg', cal_tilt_limit)
    rospy.set_param('~pan_steps', cal_pan_steps)
    rospy.set_param('~tilt_steps', cal_tilt_steps)
    rospy.set_param('~fov_half_deg', fov_half_deg)
    rospy.set_param('~settle_time', settle_time)
    rospy.set_param('~output_dir', output_dir)

    acq_cal = CalibrationAcquisition()
    success = acq_cal.run()
    if not success:
        rospy.logerr("PHASE 1 FAILED: Not enough calibration captures!")
        return

    cal_file = os.path.join(output_dir, f'calib_data_{ts}.npz')
    acq_cal.save(cal_file)

    # ══════════════════════════════════════════════════════════════
    #  PHASE 2: Solve Calibration
    # ══════════════════════════════════════════════════════════════
    print("\n" + "▓" * 74)
    print("▓  PHASE 2: SOLVING CALIBRATION")
    print("▓" * 74 + "\n")

    delta_opt, urdf_corrections, encoder_offsets, corrected_urdf = \
        run_solver(cal_file, lam=lam, max_pairs=max_pairs, alpha_plane=plane_weight)

    if delta_opt is None:
        rospy.logerr("PHASE 2 FAILED: Solver did not converge!")
        return

    # Save delta for later use
    delta_file = os.path.join(output_dir, f'delta_{ts}.npy')
    np.save(delta_file, delta_opt)
    rospy.loginfo(f"Saved δ to {delta_file}")

    # ══════════════════════════════════════════════════════════════
    #  PHASE 3: Validation Data Acquisition
    # ══════════════════════════════════════════════════════════════
    print("\n" + "▓" * 74)
    print("▓  PHASE 3: VALIDATION DATA ACQUISITION")
    print("▓  (using different grid to test generalization)")
    print("▓" * 74 + "\n")

    # Set validation grid params, then construct a fresh acquisition object.
    # (The old __new__ approach silently broke because subscriber callbacks
    #  updated the original object's attributes, not the copy's.)
    rospy.set_param('~pan_limit_deg', val_pan_limit)
    rospy.set_param('~tilt_limit_deg', val_tilt_limit)
    rospy.set_param('~pan_steps', val_pan_steps)
    rospy.set_param('~tilt_steps', val_tilt_steps)
    rospy.set_param('~fov_half_deg', fov_half_deg)
    rospy.set_param('~settle_time', settle_time)
    rospy.set_param('~output_dir', output_dir)
    acq_val = CalibrationAcquisition()  # creates own subscribers, reads updated params

    success = acq_val.run()
    if not success:
        rospy.logerr("PHASE 3 FAILED: Not enough validation captures!")
        rospy.logwarn("Running self-validation on calibration data instead...")
        val_file = cal_file
    else:
        val_file = os.path.join(output_dir, f'valid_data_{ts}.npz')
        acq_val.save(val_file)

    # ══════════════════════════════════════════════════════════════
    #  PHASE 4: Validate
    # ══════════════════════════════════════════════════════════════
    print("\n" + "▓" * 74)
    print("▓  PHASE 4: VALIDATION")
    print("▓" * 74 + "\n")

    validator = CalibrationValidator(delta_opt,
                                     plane_rmse_threshold_mm=plane_rmse_threshold)
    result = validator.validate_from_file(val_file)

    if result is not None:
        if result['passed']:
            rospy.loginfo("═══ CALIBRATION VALIDATED SUCCESSFULLY ═══")
        else:
            rospy.logwarn("═══ CALIBRATION VALIDATION FAILED — consider re-running with more captures ═══")

    # ── Summary ────────────────────────────────────────────────
    print("\n" + "█" * 74)
    print("█  PIPELINE COMPLETE")
    print("█" * 74)
    print(f"█  Calibration data: {cal_file}")
    print(f"█  Validation data:  {val_file}")
    print(f"█  Delta file:       {delta_file}")
    print("█" * 74 + "\n")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
