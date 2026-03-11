#!/usr/bin/env python3
"""
Kinematic Chain Calibration Solver

Estimates 14 correction parameters for the pan-tilt + LiDAR kinematic chain.

Parameter vector  delta in R^14:
    delta = [ dt_pan(3), dr_pan(3),       # corrections to joint2 (pan) origin
              enc_pan_zero(1),             # pan encoder zero offset
              enc_tilt_zero(1),            # tilt encoder zero offset
              dt_laser(3), dr_laser(3) ]   # corrections to composite laser transform

The tilt joint origin (joint1) is NOT calibrated separately because its
geometric corrections are degenerate with laser mount corrections when
using a single flat wall.  The tilt encoder zero IS calibrated.

Cost function:
  (A) Global plane consistency: all projected points must lie on one plane
  (B) Tikhonov regularization: sqrt(lam) * delta

Dependencies:  numpy, scipy
"""

import numpy as np
import argparse
import sys
import os
import time
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


# ===================================================================== #
#  URDF Nominal Transforms  (from real_pantilt.urdf)                     #
# ===================================================================== #

def _T(xyz, rpy):
    """Build a 4x4 homogeneous transform from xyz + rpy.

    URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    This is extrinsic XYZ (fixed-axis) in scipy.
    """
    T = np.eye(4)
    T[:3, :3] = R.from_euler('XYZ', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


# Nominal URDF transforms (the hand-measured values we want to correct)
# joint2: base_link -> pan_link
T_PAN_NOMINAL = _T([0.0, -0.0285, 0.092], [0.0, 0.0, 0.0])

# joint1: pan_link -> tilt_link  (before tilt rotation)
T_TILT_NOMINAL = _T([0.0, -0.009, 0.022], [0.0, 1.57, 0.0])

# hokuyo_base_joint + hokuyo_joint composed: tilt_link -> laser
# hokuyo_base_joint:  tilt_link -> hokuyo_base  xyz="-0.045 0 0"  rpy="0 0 0"
# hokuyo_joint:       hokuyo_base -> laser      xyz="-0.061 0.014 0.040"  rpy="1.57 3.14 -1.57"
T_HBASE = _T([-0.045, 0.0, 0.0], [0.0, 0.0, 0.0])
T_HJOINT = _T([-0.061, 0.014, 0.040], [1.57, 3.14, -1.57])
T_LASER_NOMINAL = T_HBASE @ T_HJOINT


# ===================================================================== #
#  SE(3) helpers                                                          #
# ===================================================================== #

def _Rz(angle):
    """Rotation matrix about Z."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _apply_correction(T_nominal, delta_t, delta_r):
    """
    Apply a small correction (dt, dr) to a nominal transform.
    T_corrected = T_nominal * dT
    where dT has translation dt and rotation from Euler dr = (droll, dpitch, dyaw)
    using the URDF extrinsic X-Y-Z convention.
    """
    dT = np.eye(4)
    dT[:3, :3] = R.from_euler('XYZ', delta_r).as_matrix()
    dT[:3, 3] = delta_t
    return T_nominal @ dT


# ===================================================================== #
#  Forward Kinematics with Corrections                                    #
# ===================================================================== #

def forward_kinematics(theta_pan, theta_tilt, delta):
    """
    Compute  T_laser^base(theta_p, theta_t; delta)  with correction parameters.

    delta in R^14:
        [0:3]   dt_pan         - translation correction to pan origin
        [3:6]   dr_pan         - rotation correction to pan origin (roll, pitch, yaw)
        [6]     enc_pan_zero   - pan encoder zero offset
        [7]     enc_tilt_zero  - tilt encoder zero offset
        [8:11]  dt_laser       - translation correction to laser composite transform
        [11:14] dr_laser       - rotation correction to laser composite transform

    Kinematic chain:
        T = T_pan_corr * Rz(-(theta_p + enc_pan)) * T_tilt_nom * Rz(theta_t + enc_tilt) * T_laser_corr

    The tilt joint origin is NOT corrected (kept at nominal) because with a
    single flat wall, tilt-origin and laser-mount corrections are degenerate.
    """
    dt_pan = delta[0:3]
    dr_pan = delta[3:6]
    enc_pan = delta[6]
    enc_tilt = delta[7]
    dt_laser = delta[8:11]
    dr_laser = delta[11:14]

    # Corrected static transforms
    T_pan = _apply_correction(T_PAN_NOMINAL, dt_pan, dr_pan)
    T_laser = _apply_correction(T_LASER_NOMINAL, dt_laser, dr_laser)

    # Joint rotations (including encoder zero offsets)
    # Pan joint: axis (0,0,-1) -> rotation about Z by -(theta_pan + encoder_offset)
    R_pan = np.eye(4)
    R_pan[:3, :3] = _Rz(-(theta_pan + enc_pan))

    # Tilt joint: axis (0,0,1) -> rotation about Z by +(theta_tilt + encoder_offset)
    R_tilt = np.eye(4)
    R_tilt[:3, :3] = _Rz(theta_tilt + enc_tilt)

    # Full chain: base -> pan_link -> tilt_link -> laser
    T_total = T_pan @ R_pan @ T_TILT_NOMINAL @ R_tilt @ T_laser
    return T_total


# ===================================================================== #
#  Load Captured Data                                                     #
# ===================================================================== #

def load_captures(data_file):
    """Load data from calibration_acquisition.py output."""
    data = np.load(data_file, allow_pickle=True)
    n = int(data['n_captures'])
    captures = []
    for i in range(n):
        cap = {
            'pan_rad': float(data[f'cap_{i}_pan_rad']),
            'tilt_rad': float(data[f'cap_{i}_tilt_rad']),
            'line_dir': data[f'cap_{i}_line_dir'],
            'line_mean': data[f'cap_{i}_line_mean'],
            'inlier_points': data[f'cap_{i}_inlier_points'],
            'all_points': data[f'cap_{i}_all_points'],
            'timestamp': float(data[f'cap_{i}_timestamp']),
        }
        captures.append(cap)
    fov = float(data.get('fov_half_deg', 15.0))
    return captures, fov


# ===================================================================== #
#  Per-Capture Line Quality Filter                                        #
# ===================================================================== #

def filter_captures(captures, line_rmse_thresh_mm=3.0, point_err_thresh_mm=None):
    """
    Filter captures based on per-scan line quality.

    For each capture, the inlier points should lie on a straight line
    (since the lidar is planar and sees a flat wall).  Captures whose
    line RMSE exceeds the threshold are rejected entirely.

    Optionally, within surviving captures, individual points whose
    perpendicular distance to the fitted line exceeds point_err_thresh_mm
    are removed.

    Args:
        captures: list of capture dicts with 'inlier_points', 'line_dir', 'line_mean'
        line_rmse_thresh_mm: reject captures with line RMSE above this (mm)
        point_err_thresh_mm: clip individual points beyond this (mm), or None to skip

    Returns:
        filtered: list of captures that passed the quality check
        stats: dict with filtering statistics
    """
    if point_err_thresh_mm is None:
        point_err_thresh_mm = line_rmse_thresh_mm * 2.0

    accepted = []
    rejected_indices = []
    total_pts_before = 0
    total_pts_after = 0

    print(f"\n--- Per-Capture Line Quality Filter ---")
    print(f"  Line RMSE threshold:  {line_rmse_thresh_mm:.1f} mm")
    print(f"  Point error threshold: {point_err_thresh_mm:.1f} mm")
    print(f"  {'Cap':>4s}  {'Pan':>7s}  {'Tilt':>7s}  {'Pts':>4s}  "
          f"{'Line RMSE':>10s}  {'Max Err':>8s}  {'Status':>8s}  {'Pts After':>10s}")

    for i, cap in enumerate(captures):
        pts = cap['inlier_points']
        line_dir = cap['line_dir']
        line_mean = cap['line_mean']
        total_pts_before += len(pts)

        # Perpendicular distances to the fitted line
        n_perp = np.array([-line_dir[1], line_dir[0]])
        dists = (pts - line_mean) @ n_perp
        rmse_mm = np.sqrt(np.mean(dists ** 2)) * 1000.0
        max_err_mm = np.max(np.abs(dists)) * 1000.0

        pan_deg = np.degrees(cap['pan_rad'])
        tilt_deg = np.degrees(cap['tilt_rad'])

        if rmse_mm > line_rmse_thresh_mm:
            rejected_indices.append(i)
            print(f"  {i:4d}  {pan_deg:+6.1f}  {tilt_deg:+6.1f}  {len(pts):4d}  "
                  f"{rmse_mm:9.2f}mm  {max_err_mm:7.2f}mm  REJECT")
            continue

        # Clip individual outlier points
        point_mask = np.abs(dists) * 1000.0 <= point_err_thresh_mm
        pts_clean = pts[point_mask]
        n_clipped = len(pts) - len(pts_clean)
        total_pts_after += len(pts_clean)

        status = "OK" if n_clipped == 0 else f"-{n_clipped}pts"
        print(f"  {i:4d}  {pan_deg:+6.1f}  {tilt_deg:+6.1f}  {len(pts):4d}  "
              f"{rmse_mm:9.2f}mm  {max_err_mm:7.2f}mm  {status:>8s}  {len(pts_clean):>10d}")

        # Update capture with cleaned points
        cap_clean = dict(cap)
        cap_clean['inlier_points'] = pts_clean
        accepted.append(cap_clean)

    stats = {
        'n_input': len(captures),
        'n_accepted': len(accepted),
        'n_rejected': len(rejected_indices),
        'rejected_indices': rejected_indices,
        'pts_before': total_pts_before,
        'pts_after': total_pts_after,
    }

    print(f"\n  Result: {stats['n_accepted']}/{stats['n_input']} captures accepted, "
          f"{stats['pts_after']}/{stats['pts_before']} points retained")

    return accepted, stats


# ===================================================================== #
#  Subsample inlier points for plane cost                                 #
# ===================================================================== #

def subsample_points(captures, max_pts_per_capture=30):
    """Deterministically subsample inlier points for the plane cost term."""
    subsampled = []
    for cap in captures:
        pts = cap['inlier_points']
        if len(pts) > max_pts_per_capture:
            step = max(1, len(pts) // max_pts_per_capture)
            pts = pts[::step][:max_pts_per_capture]
        subsampled.append(pts)
    return subsampled


# ===================================================================== #
#  Cost Function -- Global Plane Consistency + Regularization              #
# ===================================================================== #

N_KINEMATIC = 14

PARAM_LABELS = [
    'dt_pan_x', 'dt_pan_y', 'dt_pan_z',
    'dr_pan_roll', 'dr_pan_pitch', 'dr_pan_yaw',
    'enc_pan_zero', 'enc_tilt_zero',
    'dt_laser_x', 'dt_laser_y', 'dt_laser_z',
    'dr_laser_roll', 'dr_laser_pitch', 'dr_laser_yaw',
]


def compute_residuals(params, captures, subsampled_pts,
                      lam=0.01, alpha_plane=5.0):
    """
    Residual vector for least_squares.

    params in R^18 = [delta(14), n(3), d(1)]

    (A) Global plane consistency:
        For each capture k, each subsampled inlier point p_L in R^2:
            p_base = FK(theta_k; delta) * [p_L; 0; 1]
            residual = alpha_plane * (n_hat . p_base[:3] - d)

    (B) Normal unit-length soft constraint: 10 * (||n|| - 1)

    (C) Tikhonov regularization: sqrt(lam) * delta
    """
    delta = params[:N_KINEMATIC]
    n_raw = params[N_KINEMATIC:N_KINEMATIC+3]
    d_plane = params[N_KINEMATIC+3]

    n_norm = np.linalg.norm(n_raw)
    if n_norm < 1e-10:
        n_hat = np.array([0.0, 1.0, 0.0])
    else:
        n_hat = n_raw / n_norm

    residuals = []

    # -- (A) Global plane consistency --
    for k, cap in enumerate(captures):
        T_k = forward_kinematics(cap['pan_rad'], cap['tilt_rad'], delta)

        pts_2d = subsampled_pts[k]
        ones = np.ones((len(pts_2d), 1))
        zeros = np.zeros((len(pts_2d), 1))
        pts_h = np.hstack([pts_2d, zeros, ones])  # Nx4

        pts_base = (T_k @ pts_h.T).T[:, :3]  # Nx3

        plane_dists = pts_base @ n_hat - d_plane
        residuals.extend((alpha_plane * plane_dists).tolist())

    # -- (B) Normal unit-length soft constraint --
    residuals.append(10.0 * (n_norm - 1.0))

    # -- (C) Tikhonov regularization on delta --
    reg = np.sqrt(lam) * delta
    residuals.extend(reg.tolist())

    return np.array(residuals)


# ===================================================================== #
#  Compute Corrected URDF Values                                          #
# ===================================================================== #

def compute_corrected_urdf(delta):
    """
    From 14-param delta, compute corrected URDF joint origins.
    The tilt joint (joint1) is unchanged.
    """
    dt_pan = delta[0:3]
    dr_pan = delta[3:6]
    dt_laser = delta[8:11]
    dr_laser = delta[11:14]

    # Pan joint (joint2): base_link -> pan_link
    T_pan_corrected = _apply_correction(T_PAN_NOMINAL, dt_pan, dr_pan)
    pan_xyz = T_pan_corrected[:3, 3]
    pan_rpy = R.from_matrix(T_pan_corrected[:3, :3]).as_euler('XYZ')

    # Tilt joint (joint1): unchanged
    tilt_xyz = T_TILT_NOMINAL[:3, 3]
    tilt_rpy = R.from_matrix(T_TILT_NOMINAL[:3, :3]).as_euler('XYZ')

    # Laser: tilt_link -> laser (composite of hokuyo_base_joint + hokuyo_joint)
    T_laser_corrected = _apply_correction(T_LASER_NOMINAL, dt_laser, dr_laser)
    laser_xyz = T_laser_corrected[:3, 3]
    laser_rpy = R.from_matrix(T_laser_corrected[:3, :3]).as_euler('XYZ')

    return {
        'joint2': {'xyz': pan_xyz, 'rpy': pan_rpy},
        'joint1': {'xyz': tilt_xyz, 'rpy': tilt_rpy},
        'laser':  {'xyz': laser_xyz, 'rpy': laser_rpy},
    }


# ===================================================================== #
#  Print Results                                                          #
# ===================================================================== #

def print_results(delta, encoder_offsets, corrected_urdf,
                  residual_info, plane_normal=None, plane_d=None):
    """Print all calibration results to terminal."""
    print("\n" + "=" * 74)
    print("  CALIBRATION RESULTS")
    print("=" * 74)

    # Raw correction vector with observability scores
    param_obs = residual_info.get('param_observability', {})
    obs_rank = residual_info.get('observability_rank', '?')
    print(f"\n--- Correction Vector delta ({N_KINEMATIC} params) ---")
    if param_obs:
        print(f"  Observable directions: {obs_rank}/{N_KINEMATIC}")
        print(f"  (V = well-observed, X = poorly observed / in null space)")
    for lbl, val in zip(PARAM_LABELS, delta):
        if lbl.startswith('dt_'):
            unit = 'm'
            extra = f" ({val*1000:+.3f} mm)"
        elif lbl.startswith('dr_') or lbl.startswith('enc_'):
            unit = 'rad'
            extra = f" ({np.degrees(val):+.4f} deg)"
        else:
            unit = ''
            extra = ''
        obs_score = param_obs.get(lbl, 1.0)
        obs_tag = 'V' if obs_score > 0.05 else 'X'
        print(f"  {obs_tag} {lbl:20s} = {val:+.6f} {unit}{extra}")

    # Recovered plane
    if plane_normal is not None:
        print(f"\n--- Recovered Wall Plane (in base frame) ---")
        print(f"  Normal: [{plane_normal[0]:.4f}, {plane_normal[1]:.4f}, {plane_normal[2]:.4f}]")
        print(f"  Offset: {plane_d:.4f} m")

    # Encoder zero offsets
    print("\n--- Encoder Zero Offsets ---")
    print(f"  Pan  encoder zero:  {encoder_offsets['pan_zero_rad']:+.6f} rad "
          f"({np.degrees(encoder_offsets['pan_zero_rad']):+.4f} deg)")
    print(f"  Tilt encoder zero:  {encoder_offsets['tilt_zero_rad']:+.6f} rad "
          f"({np.degrees(encoder_offsets['tilt_zero_rad']):+.4f} deg)")

    # Corrected URDF values
    print("\n--- Corrected URDF Joint Origins ---")
    print("\n  joint2 (pan) -- base_link -> pan_link:")
    print(f"    ORIGINAL:  xyz=\"0.0 -0.0285 0.092\"  rpy=\"0.0 0.0 0.0\"")
    j2 = corrected_urdf['joint2']
    print(f"    CORRECTED: xyz=\"{j2['xyz'][0]:.6f} {j2['xyz'][1]:.6f} {j2['xyz'][2]:.6f}\"  "
          f"rpy=\"{j2['rpy'][0]:.6f} {j2['rpy'][1]:.6f} {j2['rpy'][2]:.6f}\"")

    print("\n  joint1 (tilt) -- pan_link -> tilt_link:  [UNCHANGED]")
    j1 = corrected_urdf['joint1']
    print(f"    xyz=\"{j1['xyz'][0]:.6f} {j1['xyz'][1]:.6f} {j1['xyz'][2]:.6f}\"  "
          f"rpy=\"{j1['rpy'][0]:.6f} {j1['rpy'][1]:.6f} {j1['rpy'][2]:.6f}\"")

    print("\n  hokuyo (laser) -- tilt_link -> laser  [replaces hokuyo_base + hokuyo_joint]:")
    print(f"    ORIGINAL:  (composite) xyz and rpy from two joints")
    jl = corrected_urdf['laser']
    print(f"    CORRECTED: xyz=\"{jl['xyz'][0]:.6f} {jl['xyz'][1]:.6f} {jl['xyz'][2]:.6f}\"  "
          f"rpy=\"{jl['rpy'][0]:.6f} {jl['rpy'][1]:.6f} {jl['rpy'][2]:.6f}\"")

    # Optimization info
    print("\n--- Optimization Summary ---")
    print(f"  Cost (initial): {residual_info['cost_initial']:.6f}")
    print(f"  Cost (final):   {residual_info['cost_final']:.6f}")
    print(f"  Reduction:      {residual_info['reduction']:.2f}%")
    print(f"  Func evals:     {residual_info['n_fev']}")
    print(f"  N captures:     {residual_info['n_captures']}")
    print(f"  Termination:    {residual_info['message']}")

    # Print copy-paste ready URDF blocks
    print("\n--- Copy-Paste URDF Blocks ---")
    print(f"""
    <!-- CALIBRATED joint2 (pan): base_link -> pan_link -->
    <joint name="joint2" type="revolute">
        <parent link="base_link" />
        <child link="pan_link" />
        <origin xyz="{j2['xyz'][0]:.6f} {j2['xyz'][1]:.6f} {j2['xyz'][2]:.6f}"
                rpy="{j2['rpy'][0]:.6f} {j2['rpy'][1]:.6f} {j2['rpy'][2]:.6f}" />
        <axis xyz="0 0 -1" />
        <limit effort="10" lower="-3.1416" upper="3.1416" velocity="3" />
        <dynamics damping="1.0" />
    </joint>

    <!-- joint1 (tilt): pan_link -> tilt_link  [UNCHANGED] -->

    <!-- CALIBRATED laser: tilt_link -> laser (replaces hokuyo_base + hokuyo_joint) -->
    <joint name="hokuyo_joint_calibrated" type="fixed">
        <parent link="tilt_link" />
        <child link="laser" />
        <origin xyz="{jl['xyz'][0]:.6f} {jl['xyz'][1]:.6f} {jl['xyz'][2]:.6f}"
                rpy="{jl['rpy'][0]:.6f} {jl['rpy'][1]:.6f} {jl['rpy'][2]:.6f}" />
        <dynamics damping="1.0" />
    </joint>
    (Note: remove hokuyo_base link, hokuyo_base_joint, and old hokuyo_joint)
""")
    print("=" * 74)


# ===================================================================== #
#  Observability Analysis                                                 #
# ===================================================================== #

def _observability_analysis(params, captures, subsampled_pts, lam, alpha_plane):
    """
    Compute numerical Jacobian and SVD to identify which of the 14 kinematic
    parameters are observable given the calibration data.
    """
    eps = 1e-7
    r0 = compute_residuals(params, captures, subsampled_pts,
                           lam=lam, alpha_plane=alpha_plane)
    n_residuals = len(r0)

    J = np.zeros((n_residuals, N_KINEMATIC))
    for i in range(N_KINEMATIC):
        p_plus = params.copy()
        p_plus[i] += eps
        J[:, i] = (compute_residuals(p_plus, captures, subsampled_pts,
                                      lam=lam, alpha_plane=alpha_plane) - r0) / eps

    U, S, Vt = np.linalg.svd(J, full_matrices=False)

    reg_floor = np.sqrt(lam) * 1.5
    rank = int(np.sum(S > reg_floor))

    param_obs = np.zeros(N_KINEMATIC)
    for k in range(len(S)):
        if S[k] > reg_floor:
            param_obs += (S[k] ** 2) * (Vt[k, :] ** 2)
    if param_obs.max() > 0:
        param_obs = param_obs / param_obs.max()

    param_observability = {}
    for i, lbl in enumerate(PARAM_LABELS):
        param_observability[lbl] = float(param_obs[i])

    return S, Vt, rank, param_observability


# ===================================================================== #
#  Main Solver                                                            #
# ===================================================================== #

def solve(data_file, lam=0.01, alpha_plane=5.0, dt_bound=0.010, dr_bound=0.05,
          enc_bound_deg=3.0, line_rmse_thresh_mm=3.0, **kwargs):
    """
    Full calibration solver.

    Optimizes 14 kinematic + 4 plane = 18 total parameters.
    Returns (delta, encoder_offsets, corrected_urdf) or (None, None, None).
    """
    print("=" * 74)
    print("  KINEMATIC CHAIN CALIBRATION SOLVER")
    print("=" * 74)

    # 1. Load
    print(f"\nLoading data from: {data_file}")
    captures, fov = load_captures(data_file)
    print(f"  Captures: {len(captures)}")
    print(f"  FOV half: {fov:.1f} deg")

    pan_range = [np.degrees(c['pan_rad']) for c in captures]
    tilt_range = [np.degrees(c['tilt_rad']) for c in captures]
    print(f"  Pan  range: [{min(pan_range):+.1f} deg, {max(pan_range):+.1f} deg]")
    print(f"  Tilt range: [{min(tilt_range):+.1f} deg, {max(tilt_range):+.1f} deg]")

    total_inliers = sum(len(c['inlier_points']) for c in captures)
    print(f"  Total inlier points: {total_inliers}")

    # 2. Filter captures by per-scan line quality
    if line_rmse_thresh_mm > 0:
        captures, filter_stats = filter_captures(
            captures, line_rmse_thresh_mm=line_rmse_thresh_mm)
    else:
        print("\n  Line quality filter: DISABLED")

    if len(captures) < 4:
        print("ERROR: Need at least 4 captures for calibration!")
        return None, None, None

    # 2. Subsample points for plane cost
    subsampled_pts = subsample_points(captures, max_pts_per_capture=30)
    total_sub = sum(len(p) for p in subsampled_pts)
    print(f"\n  Subsampled {total_sub} points for plane consistency cost")

    # 3. Initial plane estimate from uncalibrated (delta=0) projected points
    print("\n  Estimating initial wall plane from uncalibrated FK ...")
    all_base_pts = []
    delta_zero = np.zeros(N_KINEMATIC)
    for k, cap in enumerate(captures):
        T_k = forward_kinematics(cap['pan_rad'], cap['tilt_rad'], delta_zero)
        pts_2d = subsampled_pts[k]
        ones = np.ones((len(pts_2d), 1))
        zeros = np.zeros((len(pts_2d), 1))
        pts_h = np.hstack([pts_2d, zeros, ones])
        pts_base = (T_k @ pts_h.T).T[:, :3]
        all_base_pts.append(pts_base)
    all_base_pts = np.vstack(all_base_pts)
    centroid = all_base_pts.mean(axis=0)
    _, S_plane, Vt_plane = np.linalg.svd(all_base_pts - centroid, full_matrices=False)
    n_init = Vt_plane[2]
    d_init = n_init @ centroid
    if d_init < 0:
        n_init = -n_init
        d_init = -d_init

    print(f"  Initial plane normal: [{n_init[0]:.4f}, {n_init[1]:.4f}, {n_init[2]:.4f}]")
    print(f"  Initial plane offset: {d_init:.4f} m")
    uncal_dists = all_base_pts @ n_init - d_init
    print(f"  Uncalibrated plane RMSE: {np.sqrt(np.mean(uncal_dists**2))*1000:.2f} mm")

    # 4. Optimize  params = [delta(14), n(3), d(1)] = 18 total
    n_total_params = N_KINEMATIC + 4
    params_init = np.zeros(n_total_params)
    params_init[N_KINEMATIC:N_KINEMATIC+3] = n_init
    params_init[N_KINEMATIC+3] = d_init

    # Encoder offset bounds
    enc_bound = np.radians(enc_bound_deg)

    lb = np.concatenate([
        # dt_pan(3), dr_pan(3)
        [-dt_bound]*3 + [-dr_bound]*3,
        # enc_pan, enc_tilt
        [-enc_bound, -enc_bound],
        # dt_laser(3), dr_laser(3)
        [-dt_bound]*3 + [-dr_bound]*3,
        # plane normal, plane offset
        [-1.0, -1.0, -1.0, -20.0],
    ])
    ub = np.concatenate([
        [dt_bound]*3 + [dr_bound]*3,
        [enc_bound, enc_bound],
        [dt_bound]*3 + [dr_bound]*3,
        [1.0, 1.0, 1.0, 20.0],
    ])

    print(f"\nOptimizing {n_total_params} parameters "
          f"({N_KINEMATIC} kinematic + 3 plane normal + 1 plane offset)")
    print(f"  lam (regularization) = {lam}")
    print(f"  alpha (plane weight) = {alpha_plane}")
    print(f"  Bounds: translations +/-{dt_bound*1000:.0f} mm, "
          f"rotations +/-{np.degrees(dr_bound):.1f} deg, "
          f"encoder +/-{np.degrees(enc_bound):.1f} deg")

    r0 = compute_residuals(params_init, captures, subsampled_pts,
                           lam=lam, alpha_plane=alpha_plane)
    cost0 = 0.5 * np.sum(r0 ** 2)
    print(f"  Initial cost: {cost0:.6f}  ({len(r0)} residuals)")

    t0 = time.time()
    result = least_squares(
        compute_residuals,
        params_init,
        args=(captures, subsampled_pts, lam, alpha_plane),
        method='trf',
        bounds=(lb, ub),
        ftol=1e-12,
        xtol=1e-12,
        gtol=1e-12,
        max_nfev=5000,
        verbose=1
    )
    solve_time = time.time() - t0

    params_opt = result.x
    delta_opt = params_opt[:N_KINEMATIC]
    n_opt = params_opt[N_KINEMATIC:N_KINEMATIC+3]
    n_opt_norm = np.linalg.norm(n_opt)
    n_opt_hat = n_opt / n_opt_norm if n_opt_norm > 1e-10 else np.array([0., 1., 0.])
    d_opt = params_opt[N_KINEMATIC+3]

    cost_final = 0.5 * np.sum(result.fun ** 2)
    reduction = (1.0 - cost_final / max(cost0, 1e-15)) * 100.0

    print(f"\n  Solved in {solve_time:.1f}s")
    print(f"  Final cost: {cost_final:.6f}")
    print(f"  Reduction:  {reduction:.2f}%")

    # 5. Observability analysis
    print("\nRunning observability analysis ...")
    sing_vals, Vt, rank, param_obs = _observability_analysis(
        params_opt, captures, subsampled_pts, lam, alpha_plane)

    print(f"\n  Observable subspace rank: {rank}/{N_KINEMATIC}")
    print(f"  delta norm: {np.linalg.norm(delta_opt):.6f}")

    # 6. Check for parameters hitting bounds
    bound_tol = 1e-4
    at_bound = []
    for i in range(N_KINEMATIC):
        if abs(delta_opt[i] - lb[i]) < bound_tol or abs(delta_opt[i] - ub[i]) < bound_tol:
            at_bound.append(PARAM_LABELS[i])
    if at_bound:
        print(f"\n  WARNING: {len(at_bound)} parameter(s) at bounds: {', '.join(at_bound)}")
        print(f"    Consider wider bounds or a richer calibration environment.")

    # Extract outputs
    encoder_offsets = {
        'pan_zero_rad': float(delta_opt[6]),
        'tilt_zero_rad': float(delta_opt[7]),
    }

    corrected_urdf = compute_corrected_urdf(delta_opt)

    residual_info = {
        'cost_initial': cost0,
        'cost_final': cost_final,
        'reduction': reduction,
        'n_fev': result.nfev,
        'n_captures': len(captures),
        'message': result.message,
        'observability_rank': rank,
        'param_observability': param_obs,
    }

    print_results(delta_opt, encoder_offsets, corrected_urdf,
                  residual_info, n_opt_hat, d_opt)

    return delta_opt, encoder_offsets, corrected_urdf


# ===================================================================== #
#  CLI Entry Point                                                        #
# ===================================================================== #

def main():
    parser = argparse.ArgumentParser(
        description='Kinematic chain calibration solver')
    parser.add_argument('--data', required=True,
                        help='Path to .npz file from calibration_acquisition.py')
    parser.add_argument('--lambda', dest='lam', type=float, default=0.01,
                        help='Tikhonov regularization weight (default: 0.01)')
    parser.add_argument('--plane-weight', type=float, default=5.0,
                        help='Weight for global plane consistency term (default: 5.0)')
    parser.add_argument('--dt-bound', type=float, default=0.010,
                        help='Max translation correction in meters (default: 0.010 = 10mm)')
    parser.add_argument('--dr-bound', type=float, default=0.05,
                        help='Max rotation correction in radians (default: 0.05 = 2.87 deg)')
    parser.add_argument('--enc-bound', type=float, default=3.0,
                        help='Max encoder zero offset in degrees (default: 3.0)')
    parser.add_argument('--line-rmse-threshold', type=float, default=3.0,
                        help='Max acceptable per-scan line RMSE in mm (default: 3.0). '
                             'Captures with worse line fit are rejected. Set to 0 to disable.')

    args = parser.parse_args()

    if not os.path.exists(args.data):
        print(f"ERROR: File not found: {args.data}")
        sys.exit(1)

    solve(args.data, lam=args.lam, alpha_plane=args.plane_weight,
          dt_bound=args.dt_bound, dr_bound=args.dr_bound,
          enc_bound_deg=args.enc_bound,
          line_rmse_thresh_mm=args.line_rmse_threshold)


if __name__ == '__main__':
    main()
