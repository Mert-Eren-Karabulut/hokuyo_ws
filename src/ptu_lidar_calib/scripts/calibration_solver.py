#!/usr/bin/env python3
"""
Kinematic Chain Calibration Solver  —  Section 4.9 of the report.

Estimates 18 correction parameters (6 per link × 3 links) for the
pan-tilt + LiDAR kinematic chain.

Parameter vector  δ ∈ R^18:
    δ = [ δt_pan(3), δr_pan(3),          # corrections to joint2 origin
          δt_tilt(3), δr_tilt(3),         # corrections to joint1 origin
          δt_laser(3), δr_laser(3) ]      # corrections to hokuyo_joint + hokuyo_base_joint

The method uses two complementary cost terms:

  (A) Pairwise SE(3) error (Eq. 28-30):
      For pairs (i,j), compute E_ij = T_ij^meas · (T_ij^pred)^{-1}
      Minimize Σ w_ij ‖log(E_ij(δ))‖²

  (B) Global plane consistency:
      Transform all inlier points to the base frame using FK(θ_p, θ_t; δ).
      All points should lie on a single plane.
      Minimize Σ_k Σ_{p ∈ P_k} (n · p_base - d)²

Both terms together make all 18 parameters well-observable.

Workflow (Section 4.9.10):
    1. Load captured scans  D = { (P_k, θ_p^(k), θ_t^(k)) }
    2. Compute pairwise ICP alignments  T_ij^meas  and predicted  T_ij^pred
    3. Solve hybrid cost (SE(3) + plane consistency) + λ‖δ‖²
    4. Separate encoder-zero offsets from geometric URDF corrections (Eq. 32)
    5. Print results

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

    URDF rpy convention: R = Rz(yaw) · Ry(pitch) · Rx(roll)
    This equals scipy intrinsic 'xyz' with args (roll, pitch, yaw).
    """
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


# Nominal URDF transforms (the hand-measured values we want to correct)
# joint2: base_link → pan_link
T_PAN_NOMINAL = _T([0.0, -0.0285, 0.092], [0.0, 0.0, 0.0])

# joint1: pan_link → tilt_link  (before tilt rotation)
T_TILT_NOMINAL = _T([0.0, -0.009, 0.022], [0.0, 1.57, 0.0])

# hokuyo_base_joint + hokuyo_joint composed: tilt_link → laser
# hokuyo_base_joint:  tilt_link → hokuyo_base  xyz="-0.045 0 0"  rpy="0 0 0"
# hokuyo_joint:       hokuyo_base → laser      xyz="-0.061 0.014 0.040"  rpy="1.57 3.14 -1.57"
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


def _skew(v):
    """Skew-symmetric matrix from 3-vector."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def _se3_log(T):
    """
    Matrix logarithm  SE(3) → se(3).
    Returns a 6-vector [ω(3), v(3)] using the Rodrigues log map.
    """
    R_mat = T[:3, :3]
    t = T[:3, 3]

    # Rotation log
    cos_angle = np.clip((np.trace(R_mat) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_angle)

    if abs(angle) < 1e-10:
        # Near identity
        omega = np.zeros(3)
        v = t
    elif abs(angle - np.pi) < 1e-6:
        # Near 180°  — use special extraction
        # Find the column of R+I with largest norm
        RpI = R_mat + np.eye(3)
        col = np.argmax(np.linalg.norm(RpI, axis=0))
        omega_dir = RpI[:, col] / np.linalg.norm(RpI[:, col])
        omega = omega_dir * angle
        # V^-1 for angle ≈ π
        V_inv = np.eye(3) - 0.5 * _skew(omega) + \
                (1.0 / (angle * angle)) * (1.0 - (angle * np.sin(angle)) / (2.0 * (1.0 - cos_angle))) * (_skew(omega) @ _skew(omega))
        v = V_inv @ t
    else:
        omega_hat = (R_mat - R_mat.T) / (2.0 * np.sin(angle))
        omega_vec = np.array([omega_hat[2, 1], omega_hat[0, 2], omega_hat[1, 0]])
        omega = omega_vec * angle

        # Inverse of V matrix  (left Jacobian inverse)
        K = _skew(omega_vec)
        half = angle / 2.0
        V_inv = np.eye(3) - 0.5 * _skew(omega) + \
                (1.0 - half / np.tan(half)) / (angle * angle) * (_skew(omega) @ _skew(omega))
        v = V_inv @ t

    return np.concatenate([omega, v])


def _apply_correction(T_nominal, delta_t, delta_r):
    """
    Apply a small correction (δt, δr) to a nominal transform.
    T_corrected = T_nominal · ΔT
    where ΔT has translation δt and rotation from Euler δr = (δroll, δpitch, δyaw)
    using the URDF extrinsic X-Y-Z convention.
    """
    dT = np.eye(4)
    dT[:3, :3] = R.from_euler('xyz', delta_r).as_matrix()
    dT[:3, 3] = delta_t
    return T_nominal @ dT


# ===================================================================== #
#  Forward Kinematics with Corrections                                    #
# ===================================================================== #

def forward_kinematics(theta_pan, theta_tilt, delta):
    """
    Compute  T_laser^base(θ_p, θ_t; δ)  with correction parameter vector δ.

    δ ∈ R^18 = [δt_pan(3), δr_pan(3), δt_tilt(3), δr_tilt(3), δt_laser(3), δr_laser(3)]

    Kinematic chain (Eq. 26):
        T = T_pan^base · R_z(θ_p) · T_tilt^pan · R_z(θ_t) · T_laser^tilt

    Note: joint2 axis is (0,0,-1) so pan rotation = R_z(-θ_p) about standard Z,
    but since the URDF axis definition handles the sign, we use the axis directly:
        R_pan = exp(PAN_AXIS * θ_p) = R_z(-θ_p)
    """
    dt_pan = delta[0:3]
    dr_pan = delta[3:6]
    dt_tilt = delta[6:9]
    dr_tilt = delta[9:12]
    dt_laser = delta[12:15]
    dr_laser = delta[15:18]

    # Corrected static transforms
    T_pan = _apply_correction(T_PAN_NOMINAL, dt_pan, dr_pan)
    T_tilt = _apply_correction(T_TILT_NOMINAL, dt_tilt, dr_tilt)
    T_laser = _apply_correction(T_LASER_NOMINAL, dt_laser, dr_laser)

    # Joint rotations
    # Pan joint: axis (0,0,-1) → rotation about Z by -θ_pan
    R_pan = np.eye(4)
    R_pan[:3, :3] = _Rz(-theta_pan)

    # Tilt joint: axis (0,0,1) → rotation about Z by +θ_tilt
    R_tilt = np.eye(4)
    R_tilt[:3, :3] = _Rz(theta_tilt)

    # Full chain: base → pan_link → tilt_link → laser
    T_total = T_pan @ R_pan @ T_tilt @ R_tilt @ T_laser
    return T_total


# ===================================================================== #
#  Pairwise Alignment via Point-to-Plane ICP  (Eq. 25)                   #
# ===================================================================== #

def point_to_plane_icp(source_pts, target_pts, max_iterations=30,
                       distance_threshold=0.05, tolerance=1e-7):
    """
    Simple point-to-plane ICP between two 2D point sets (represented as Nx2).
    Both are in the laser frame (z=0 plane), so we work in 2D.

    We fit a line (plane in 2D) to target, then align source to it.

    Returns T_ij ∈ SE(3)  that transforms source (j) into target (i) frame.
    """
    # Since these are 2D laser scans, the "plane" is a line.
    # Fit line to target via SVD
    target_mean = target_pts.mean(axis=0)
    target_centered = target_pts - target_mean
    _, _, Vt = np.linalg.svd(target_centered, full_matrices=False)
    line_dir = Vt[0]
    line_normal = np.array([-line_dir[1], line_dir[0]])  # 2D normal

    # Iterative alignment
    src = source_pts.copy()
    T_accum = np.eye(3)  # 2D homogeneous (3x3)

    for iteration in range(max_iterations):
        # Point-to-line distances for correspondence
        src_centered = src - target_mean
        signed_dists = src_centered @ line_normal

        # Build least-squares for 2D rigid transform that minimizes point-to-line error
        # For point-to-plane in 2D: minimize Σ (n · (R·p_i + t - q_mean))²
        # Linearize around current estimate: rotation angle α, translation (tx, ty)
        # Residual_i = n · ((-sin α · px + cos α · py + ty) - qy_mean, ...) ≈
        #            = n_x·(- α·py + tx) + n_y·(α·px + ty) + n·(p - q_mean)
        # Simplify to: α·(n_x·(-py) + n_y·px) + n_x·tx + n_y·ty + signed_dist_i = 0

        nx, ny = line_normal
        A = np.zeros((len(src), 3))
        b = -signed_dists

        for i_pt in range(len(src)):
            px, py = src[i_pt]
            A[i_pt, 0] = nx * (-(py - target_mean[1])) + ny * (px - target_mean[0])  # d/d(alpha)
            A[i_pt, 1] = nx  # d/d(tx)
            A[i_pt, 2] = ny  # d/d(ty)

        # Solve Ax = b
        try:
            x, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            break

        alpha, tx, ty = x

        # Build incremental 2D transform
        ca, sa = np.cos(alpha), np.sin(alpha)
        dT = np.array([[ca, -sa, tx],
                        [sa,  ca, ty],
                        [0,    0,  1]])

        # Apply
        src_h = np.hstack([src, np.ones((len(src), 1))])
        src_h = (dT @ src_h.T).T
        src = src_h[:, :2]
        T_accum = dT @ T_accum

        if abs(alpha) < tolerance and abs(tx) < tolerance and abs(ty) < tolerance:
            break

    # Convert 2D result to SE(3) (rotation about Z + translation in XY)
    T_3d = np.eye(4)
    total_angle = np.arctan2(T_accum[1, 0], T_accum[0, 0])
    T_3d[:3, :3] = _Rz(total_angle)
    T_3d[0, 3] = T_accum[0, 2]
    T_3d[1, 3] = T_accum[1, 2]
    # z translation stays 0 (2D scans)

    return T_3d


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
#  Pair Selection & Weighting  (Eq. 31)                                   #
# ===================================================================== #

def select_pairs(captures, max_pairs=None):
    """
    Generate all unique pairs (i, j) with i < j.
    Weight by angular baseline (Eq. 31):
        w_ij = sqrt( (θ_p^i - θ_p^j)² + (θ_t^i - θ_t^j)² )
    """
    N = len(captures)
    pairs = []
    for i in range(N):
        for j in range(i + 1, N):
            dp = captures[i]['pan_rad'] - captures[j]['pan_rad']
            dt = captures[i]['tilt_rad'] - captures[j]['tilt_rad']
            w = np.sqrt(dp * dp + dt * dt)
            if w < 1e-6:
                continue  # skip near-identical configurations
            pairs.append((i, j, w))

    # Sort by weight descending (larger baselines first)
    pairs.sort(key=lambda x: -x[2])

    if max_pairs is not None and len(pairs) > max_pairs:
        pairs = pairs[:max_pairs]

    return pairs


# ===================================================================== #
#  Compute Measured Pairwise Transforms via ICP  (Eq. 25)                 #
# ===================================================================== #

def compute_measured_transforms(captures, pairs):
    """
    For each pair (i, j), compute T_ij^meas via point-to-plane ICP.
    T_ij^meas aligns scan j into scan i's laser frame.
    """
    T_meas = {}
    for i, j, w in pairs:
        pts_i = captures[i]['inlier_points']
        pts_j = captures[j]['inlier_points']

        T_ij = point_to_plane_icp(pts_j, pts_i)
        T_meas[(i, j)] = T_ij

    return T_meas


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
#  Cost Function — Global Plane Consistency + Regularization              #
# ===================================================================== #

PARAM_LABELS = [
    'dt_pan_x', 'dt_pan_y', 'dt_pan_z',
    'dr_pan_roll', 'dr_pan_pitch', 'dr_pan_yaw',
    'dt_tilt_x', 'dt_tilt_y', 'dt_tilt_z',
    'dr_tilt_roll', 'dr_tilt_pitch', 'dr_tilt_yaw',
    'dt_laser_x', 'dt_laser_y', 'dt_laser_z',
    'dr_laser_roll', 'dr_laser_pitch', 'dr_laser_yaw',
]


def compute_residuals(params, captures, subsampled_pts,
                      lam=0.01, alpha_plane=5.0):
    """
    Residual vector for least_squares:

    params ∈ R^22 = [δ(18), n(3), d(1)]

    (A) Global plane consistency:
        For each capture k, each subsampled inlier point p_L ∈ R^2:
            p_base = FK(θ_k; δ) · [p_L; 0; 1]
            residual = alpha_plane · (n̂ · p_base[:3] - d)

    (B) Normal unit-length soft constraint: 10 · (‖n‖ - 1)

    (C) Tikhonov regularization: sqrt(λ) · δ   (favours minimum-norm δ)

    The SE(3) pairwise ICP term is intentionally omitted: 2D line-to-line
    ICP is degenerate (translation along the line is unobservable), which
    corrupts the measurement and harms the solver. The plane-consistency
    term alone constrains all parameters that the environment geometry
    makes observable.
    """
    delta = params[:18]
    n_raw = params[18:21]
    d_plane = params[21]

    # Normalize plane normal for residual computation
    n_norm = np.linalg.norm(n_raw)
    if n_norm < 1e-10:
        n_hat = np.array([0.0, 1.0, 0.0])
    else:
        n_hat = n_raw / n_norm

    residuals = []

    # ── (A) Global plane consistency ──
    for k, cap in enumerate(captures):
        T_k = forward_kinematics(cap['pan_rad'], cap['tilt_rad'], delta)

        pts_2d = subsampled_pts[k]
        # Laser frame points: (x, y, 0) in laser frame — 2D scan in z=0 plane
        ones = np.ones((len(pts_2d), 1))
        zeros = np.zeros((len(pts_2d), 1))
        pts_h = np.hstack([pts_2d, zeros, ones])  # Nx4

        pts_base = (T_k @ pts_h.T).T[:, :3]  # Nx3

        # Point-to-plane distance residuals
        plane_dists = pts_base @ n_hat - d_plane
        residuals.extend((alpha_plane * plane_dists).tolist())

    # ── (B) Normal unit-length soft constraint ──
    residuals.append(10.0 * (n_norm - 1.0))

    # ── (C) Tikhonov regularization on δ ──
    reg = np.sqrt(lam) * delta
    residuals.extend(reg.tolist())

    return np.array(residuals)


# ===================================================================== #
#  Correction Separation  (Section 4.9.7)                                #
# ===================================================================== #

def separate_corrections(delta):
    """
    Separate the 18 optimized parameters into:
      - URDF geometric corrections (applied to the URDF file)
      - Encoder zero offsets (applied to the controller)

    Pan link (joint axis = Z, axis (0,0,-1)):
      δγ_pan (yaw, rotation about Z) → encoder zero offset
      Everything else → URDF

    Tilt link (joint axis = Z in the rotated frame, originally Y):
      The tilt joint origin has rpy="0 1.57 0", so the joint rotates about
      the local Z axis. After the 90° pitch, the local Z maps to global Y.
      δγ_tilt (yaw = rotation about local Z) → encoder zero offset
      Everything else → URDF

    Laser link: all corrections → URDF (no encoder)
    """
    dt_pan = delta[0:3]
    dr_pan = delta[3:6]   # (roll, pitch, yaw) = (φ, ψ, γ)
    dt_tilt = delta[6:9]
    dr_tilt = delta[9:12]
    dt_laser = delta[12:15]
    dr_laser = delta[15:18]

    # Encoder zero offsets (Eq. 32)
    # Pan: yaw (index 2) is about the joint axis Z
    encoder_pan_zero = dr_pan[2]
    # Tilt: yaw (index 2) is about the local Z axis (which is the tilt rotation axis)
    encoder_tilt_zero = dr_tilt[2]

    # URDF corrections: remove the encoder-axis component
    urdf_dr_pan = dr_pan.copy()
    urdf_dr_pan[2] = 0.0   # yaw removed → goes to encoder

    urdf_dr_tilt = dr_tilt.copy()
    urdf_dr_tilt[2] = 0.0  # yaw removed → goes to encoder

    urdf_corrections = {
        'pan': {'dt': dt_pan, 'dr': urdf_dr_pan},
        'tilt': {'dt': dt_tilt, 'dr': urdf_dr_tilt},
        'laser': {'dt': dt_laser, 'dr': dr_laser},  # all goes to URDF
    }
    encoder_offsets = {
        'pan_zero_rad': float(encoder_pan_zero),
        'tilt_zero_rad': float(encoder_tilt_zero),
    }

    return urdf_corrections, encoder_offsets


# ===================================================================== #
#  Compute Corrected URDF Values                                          #
# ===================================================================== #

def compute_corrected_urdf(urdf_corrections):
    """Apply corrections to nominal values and return new URDF parameters."""
    # Pan joint (joint2): base_link → pan_link
    T_pan_corrected = _apply_correction(T_PAN_NOMINAL,
                                         urdf_corrections['pan']['dt'],
                                         urdf_corrections['pan']['dr'])
    pan_xyz = T_pan_corrected[:3, 3]
    pan_rpy = R.from_matrix(T_pan_corrected[:3, :3]).as_euler('xyz')

    # Tilt joint (joint1): pan_link → tilt_link
    T_tilt_corrected = _apply_correction(T_TILT_NOMINAL,
                                          urdf_corrections['tilt']['dt'],
                                          urdf_corrections['tilt']['dr'])
    tilt_xyz = T_tilt_corrected[:3, 3]
    tilt_rpy = R.from_matrix(T_tilt_corrected[:3, :3]).as_euler('xyz')

    # Laser: tilt_link → laser (composite of hokuyo_base_joint + hokuyo_joint)
    T_laser_corrected = _apply_correction(T_LASER_NOMINAL,
                                           urdf_corrections['laser']['dt'],
                                           urdf_corrections['laser']['dr'])
    laser_xyz = T_laser_corrected[:3, 3]
    laser_rpy = R.from_matrix(T_laser_corrected[:3, :3]).as_euler('xyz')

    return {
        'joint2': {'xyz': pan_xyz, 'rpy': pan_rpy},
        'joint1': {'xyz': tilt_xyz, 'rpy': tilt_rpy},
        'laser':  {'xyz': laser_xyz, 'rpy': laser_rpy},
    }


# ===================================================================== #
#  Print Results                                                          #
# ===================================================================== #

def print_results(delta, urdf_corrections, encoder_offsets, corrected_urdf,
                  residual_info, plane_normal=None, plane_d=None):
    """Print all calibration results to terminal."""
    print("\n" + "=" * 74)
    print("  CALIBRATION RESULTS")
    print("=" * 74)

    # Raw correction vector
    print("\n─── Raw Correction Vector δ (18 params) ───")
    # Raw correction vector with observability scores
    print("\n─── Raw Correction Vector δ (18 params) ───")
    param_obs = residual_info.get('param_observability', {})
    obs_rank = residual_info.get('observability_rank', '?')
    if param_obs:
        print(f"  Observable directions: {obs_rank}/18")
        print(f"  (✓ = well-observed, ✗ = poorly observed / in null space)")
    labels = PARAM_LABELS
    for lbl, val in zip(labels, delta):
        unit = 'rad' if 'dr_' in lbl else 'm'
        extra = f" ({np.degrees(val):+.4f}°)" if unit == 'rad' else \
                f" ({val*1000:+.3f} mm)"
        obs_score = param_obs.get(lbl, 1.0)
        obs_tag = '✓' if obs_score > 0.05 else '✗'
        print(f"  {obs_tag} {lbl:20s} = {val:+.6f} {unit}{extra}")

    # Recovered plane
    if plane_normal is not None:
        print(f"\n─── Recovered Wall Plane (in base frame) ───")
        print(f"  Normal: [{plane_normal[0]:.4f}, {plane_normal[1]:.4f}, {plane_normal[2]:.4f}]")
        print(f"  Offset: {plane_d:.4f} m")

    # Encoder zero offsets
    print("\n─── Encoder Zero Offsets (Section 4.9.7, Eq. 32) ───")
    print(f"  Pan  encoder zero:  {encoder_offsets['pan_zero_rad']:+.6f} rad "
          f"({np.degrees(encoder_offsets['pan_zero_rad']):+.4f}°)")
    print(f"  Tilt encoder zero:  {encoder_offsets['tilt_zero_rad']:+.6f} rad "
          f"({np.degrees(encoder_offsets['tilt_zero_rad']):+.4f}°)")
    print(f"  → When commanding θ_cmd, the controller should move to θ_cmd − θ_zero")

    # URDF corrections
    print("\n─── URDF Geometric Corrections ───")
    for link_name, corr in urdf_corrections.items():
        dt = corr['dt']
        dr = corr['dr']
        print(f"\n  [{link_name}]")
        print(f"    Translation δt: [{dt[0]:+.6f}, {dt[1]:+.6f}, {dt[2]:+.6f}] m")
        print(f"    Rotation    δr: [{dr[0]:+.6f}, {dr[1]:+.6f}, {dr[2]:+.6f}] rad")
        print(f"                  = [{np.degrees(dr[0]):+.4f}, {np.degrees(dr[1]):+.4f}, "
              f"{np.degrees(dr[2]):+.4f}]°")

    # Corrected URDF values
    print("\n─── Corrected URDF Joint Origins ───")
    print("\n  joint2 (pan) — base_link → pan_link:")
    print(f"    ORIGINAL:  xyz=\"0.0 -0.0285 0.092\"  rpy=\"0.0 0.0 0.0\"")
    j2 = corrected_urdf['joint2']
    print(f"    CORRECTED: xyz=\"{j2['xyz'][0]:.6f} {j2['xyz'][1]:.6f} {j2['xyz'][2]:.6f}\"  "
          f"rpy=\"{j2['rpy'][0]:.6f} {j2['rpy'][1]:.6f} {j2['rpy'][2]:.6f}\"")

    print("\n  joint1 (tilt) — pan_link → tilt_link:")
    print(f"    ORIGINAL:  xyz=\"0.0 -0.009 0.022\"  rpy=\"0.0 1.57 0.0\"")
    j1 = corrected_urdf['joint1']
    print(f"    CORRECTED: xyz=\"{j1['xyz'][0]:.6f} {j1['xyz'][1]:.6f} {j1['xyz'][2]:.6f}\"  "
          f"rpy=\"{j1['rpy'][0]:.6f} {j1['rpy'][1]:.6f} {j1['rpy'][2]:.6f}\"")

    print("\n  hokuyo (laser) — tilt_link → laser  [replaces hokuyo_base + hokuyo_joint]:")
    print(f"    ORIGINAL:  (composite) xyz and rpy from two joints")
    jl = corrected_urdf['laser']
    print(f"    CORRECTED: xyz=\"{jl['xyz'][0]:.6f} {jl['xyz'][1]:.6f} {jl['xyz'][2]:.6f}\"  "
          f"rpy=\"{jl['rpy'][0]:.6f} {jl['rpy'][1]:.6f} {jl['rpy'][2]:.6f}\"")

    # Optimization info
    print("\n─── Optimization Summary ───")
    print(f"  Cost (initial): {residual_info['cost_initial']:.6f}")
    print(f"  Cost (final):   {residual_info['cost_final']:.6f}")
    print(f"  Reduction:      {residual_info['reduction']:.2f}%")
    print(f"  Func evals:     {residual_info['n_fev']}")
    print(f"  N pairs:        {residual_info['n_pairs']}")
    print(f"  N captures:     {residual_info['n_captures']}")
    print(f"  Termination:    {residual_info['message']}")

    # Print copy-paste ready URDF blocks
    print("\n─── Copy-Paste URDF Blocks ───")
    print(f"""
    <!-- CALIBRATED joint2 (pan): base_link → pan_link -->
    <joint name="joint2" type="revolute">
        <parent link="base_link" />
        <child link="pan_link" />
        <origin xyz="{j2['xyz'][0]:.6f} {j2['xyz'][1]:.6f} {j2['xyz'][2]:.6f}"
                rpy="{j2['rpy'][0]:.6f} {j2['rpy'][1]:.6f} {j2['rpy'][2]:.6f}" />
        <axis xyz="0 0 -1" />
        <limit effort="10" lower="-3.1416" upper="3.1416" velocity="3" />
        <dynamics damping="1.0" />
    </joint>

    <!-- CALIBRATED joint1 (tilt): pan_link → tilt_link -->
    <joint name="joint1" type="revolute">
        <parent link="pan_link" />
        <child link="tilt_link" />
        <origin xyz="{j1['xyz'][0]:.6f} {j1['xyz'][1]:.6f} {j1['xyz'][2]:.6f}"
                rpy="{j1['rpy'][0]:.6f} {j1['rpy'][1]:.6f} {j1['rpy'][2]:.6f}" />
        <axis xyz="0 0 1" />
        <limit effort="10" lower="-0.8727" upper="0.8727" velocity="3" />
        <dynamics damping="1.0" />
    </joint>

    <!-- CALIBRATED laser: tilt_link → laser (replaces hokuyo_base + hokuyo_joint) -->
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
#  Main Solver                                                            #
# ===================================================================== #

def _observability_analysis(params, captures, subsampled_pts, lam, alpha_plane):
    """
    Compute numerical Jacobian and SVD to identify which of the 18 kinematic
    parameters are observable given the calibration environment geometry.

    Returns:
        singular_values: array of 18 singular values
        V: 18x18 right singular vectors (rows of V^T)
        rank: effective rank (number of observable directions)
        param_observability: dict mapping param label → observability score [0,1]
    """
    eps = 1e-7
    r0 = compute_residuals(params, captures, subsampled_pts,
                           lam=lam, alpha_plane=alpha_plane)
    n_residuals = len(r0)

    # Jacobian w.r.t. first 18 params (δ only)
    J = np.zeros((n_residuals, 18))
    for i in range(18):
        p_plus = params.copy()
        p_plus[i] += eps
        J[:, i] = (compute_residuals(p_plus, captures, subsampled_pts,
                                      lam=lam, alpha_plane=alpha_plane) - r0) / eps

    U, S, Vt = np.linalg.svd(J, full_matrices=False)

    # Effective rank: singular values above threshold
    # Regularization contributes sqrt(lam) to each parameter's SV,
    # so the threshold should be above that
    reg_floor = np.sqrt(lam) * 1.5
    rank = int(np.sum(S > reg_floor))

    # Per-parameter observability score:
    # For each parameter j, sum the squared components of the right singular
    # vectors weighted by the corresponding singular values (above threshold)
    param_obs = np.zeros(18)
    for k in range(len(S)):
        if S[k] > reg_floor:
            param_obs += (S[k] ** 2) * (Vt[k, :] ** 2)
    # Normalize to [0, 1]
    if param_obs.max() > 0:
        param_obs = param_obs / param_obs.max()

    param_observability = {}
    for i, lbl in enumerate(PARAM_LABELS):
        param_observability[lbl] = float(param_obs[i])

    return S, Vt, rank, param_observability


def solve(data_file, lam=0.01, alpha_plane=5.0, dt_bound=0.010, dr_bound=0.05, **kwargs):
    """
    Full calibration pipeline:
      1. Load data
      2. Subsample points for plane-consistency cost
      3. Optimize δ (18 kinematic + 4 plane = 22 params)
      4. Observability analysis
      5. Separate corrections
      6. Print results
    Returns (delta, urdf_corrections, encoder_offsets, corrected_urdf).

    Note: max_pairs and alpha_plane are accepted via kwargs for backward compat.
    """
    # Accept legacy keyword args
    max_pairs = kwargs.get('max_pairs', 200)

    print("=" * 74)
    print("  KINEMATIC CHAIN CALIBRATION SOLVER  (Section 4.9)")
    print("=" * 74)

    # 1. Load
    print(f"\nLoading data from: {data_file}")
    captures, fov = load_captures(data_file)
    print(f"  Captures: {len(captures)}")
    print(f"  FOV half: {fov:.1f}°")

    pan_range = [np.degrees(c['pan_rad']) for c in captures]
    tilt_range = [np.degrees(c['tilt_rad']) for c in captures]
    print(f"  Pan  range: [{min(pan_range):+.1f}°, {max(pan_range):+.1f}°]")
    print(f"  Tilt range: [{min(tilt_range):+.1f}°, {max(tilt_range):+.1f}°]")

    total_inliers = sum(len(c['inlier_points']) for c in captures)
    print(f"  Total inlier points: {total_inliers}")

    if len(captures) < 4:
        print("ERROR: Need at least 4 captures for calibration!")
        return None, None, None, None

    # 2. Subsample points for plane cost
    subsampled_pts = subsample_points(captures, max_pts_per_capture=30)
    total_sub = sum(len(p) for p in subsampled_pts)
    print(f"\n  Subsampled {total_sub} points for plane consistency cost")

    # 3. Initial plane estimate from uncalibrated (δ=0) projected points
    #    Much better than using one pose's laser direction
    print("\n  Estimating initial wall plane from uncalibrated FK ...")
    all_base_pts = []
    delta_zero = np.zeros(18)
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
    n_init = Vt_plane[2]  # normal = smallest singular value direction
    d_init = n_init @ centroid
    # Ensure normal points away from sensor (positive d if wall is in front)
    if d_init < 0:
        n_init = -n_init
        d_init = -d_init

    print(f"  Initial plane normal: [{n_init[0]:.4f}, {n_init[1]:.4f}, {n_init[2]:.4f}]")
    print(f"  Initial plane offset: {d_init:.4f} m")
    uncal_dists = all_base_pts @ n_init - d_init
    print(f"  Uncalibrated plane RMSE: {np.sqrt(np.mean(uncal_dists**2))*1000:.2f} mm")

    # 4. Optimize  params = [δ(18), n(3), d(1)] = 22 total
    params_init = np.zeros(22)
    params_init[18:21] = n_init
    params_init[21] = d_init

    # Parameter bounds: the URDF is approximately correct, so δ must be small.
    # This prevents the solver from finding degenerate equivalent configurations
    # (e.g. swapping 90° between tilt pitch and laser roll).
    # Plane normal components: [-1, 1], Plane offset: [-20, 20] m
    lb = np.concatenate([
        np.tile([-dt_bound, -dt_bound, -dt_bound, -dr_bound, -dr_bound, -dr_bound], 3),  # 18 δ
        [-1.0, -1.0, -1.0],   # plane normal
        [-20.0],               # plane offset
    ])
    ub = np.concatenate([
        np.tile([dt_bound, dt_bound, dt_bound, dr_bound, dr_bound, dr_bound], 3),  # 18 δ
        [1.0, 1.0, 1.0],      # plane normal
        [20.0],                # plane offset
    ])

    print(f"\nOptimizing 22 parameters (18 kinematic + 3 plane normal + 1 plane offset)")
    print(f"  λ (regularization) = {lam}")
    print(f"  α (plane weight)   = {alpha_plane}")
    print(f"  δ bounds: translations ±{dt_bound*1000:.0f} mm, rotations ±{np.degrees(dr_bound):.1f}°")

    # Evaluate initial cost
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
    delta_opt = params_opt[:18]
    n_opt = params_opt[18:21]
    n_opt_norm = np.linalg.norm(n_opt)
    if n_opt_norm > 1e-10:
        n_opt_hat = n_opt / n_opt_norm
    else:
        n_opt_hat = np.array([0.0, 1.0, 0.0])
    d_opt = params_opt[21]

    cost_final = 0.5 * np.sum(result.fun ** 2)
    reduction = (1.0 - cost_final / max(cost0, 1e-15)) * 100.0

    print(f"\n  Solved in {solve_time:.1f}s")
    print(f"  Final cost: {cost_final:.6f}")
    print(f"  Reduction:  {reduction:.2f}%")

    # 5. Observability analysis
    print("\nRunning observability analysis ...")
    sing_vals, Vt, rank, param_obs = _observability_analysis(
        params_opt, captures, subsampled_pts, lam, alpha_plane)

    print(f"\n  Observable subspace rank: {rank}/18")
    print(f"  δ norm: {np.linalg.norm(delta_opt):.6f}")

    # 6. Check for parameters hitting bounds
    bound_tol = 1e-4  # consider "at bound" if within this margin
    at_bound = []
    for i in range(18):
        if abs(delta_opt[i] - lb[i]) < bound_tol or abs(delta_opt[i] - ub[i]) < bound_tol:
            at_bound.append(PARAM_LABELS[i])
    if at_bound:
        print(f"\n  ⚠ {len(at_bound)} parameter(s) at bounds: {', '.join(at_bound)}")
        print(f"    These may need wider bounds (--dt-bound / --dr-bound) or")
        print(f"    a richer calibration environment (e.g. corner of two walls).")

    residual_info = {
        'cost_initial': cost0,
        'cost_final': cost_final,
        'reduction': reduction,
        'n_fev': result.nfev,
        'n_pairs': 0,
        'n_captures': len(captures),
        'message': result.message,
        'observability_rank': rank,
        'param_observability': param_obs,
    }

    # 6. Separate corrections
    urdf_corrections, encoder_offsets = separate_corrections(delta_opt)

    # 7. Compute corrected URDF values
    corrected_urdf = compute_corrected_urdf(urdf_corrections)

    # 8. Print
    print_results(delta_opt, urdf_corrections, encoder_offsets,
                  corrected_urdf, residual_info, n_opt_hat, d_opt)

    return delta_opt, urdf_corrections, encoder_offsets, corrected_urdf


# ===================================================================== #
#  CLI Entry Point                                                        #
# ===================================================================== #

def main():
    parser = argparse.ArgumentParser(
        description='Kinematic chain calibration solver (Section 4.9)')
    parser.add_argument('--data', required=True,
                        help='Path to .npz file from calibration_acquisition.py')
    parser.add_argument('--lambda', dest='lam', type=float, default=0.01,
                        help='Tikhonov regularization weight (default: 0.01)')
    parser.add_argument('--plane-weight', type=float, default=5.0,
                        help='Weight for global plane consistency term (default: 5.0)')
    parser.add_argument('--dt-bound', type=float, default=0.010,
                        help='Max translation correction in meters (default: 0.010 = 10mm)')
    parser.add_argument('--dr-bound', type=float, default=0.05,
                        help='Max rotation correction in radians (default: 0.05 = 2.87°)')

    args = parser.parse_args()

    if not os.path.exists(args.data):
        print(f"ERROR: File not found: {args.data}")
        sys.exit(1)

    solve(args.data, lam=args.lam, alpha_plane=args.plane_weight,
          dt_bound=args.dt_bound, dr_bound=args.dr_bound)


if __name__ == '__main__':
    main()
