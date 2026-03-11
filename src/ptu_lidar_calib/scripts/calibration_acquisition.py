#!/usr/bin/env python3
"""
Automated Grid-Scan Calibration Data Acquisition

Moves the PTU through a grid of (pan, tilt) configurations facing a flat wall,
captures a laser scan + joint state at each pose, extracts the forward-facing
ROI (±fov_half_deg), fits a line via RANSAC, and stores the result.

Outputs a .npz file ready for calibration_solver.py

Section 4.9 of the report specifies N=20-50 configurations covering the joint
space uniformly, with ±15° FOV extraction from each scan.
"""

import numpy as np
import rospy
import math
import time
import os
import sys
from sensor_msgs.msg import LaserScan, JointState
from threading import Lock, Event
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import tf2_ros
import colorsys

# Import FK for RViz viz fallback (works without robot_state_publisher)
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    if SCRIPT_DIR not in sys.path:
        sys.path.insert(0, SCRIPT_DIR)
    from calibration_solver import forward_kinematics
    _HAS_FK = True
except ImportError:
    _HAS_FK = False


class ScanCapture:
    """Holds one captured scan configuration."""
    __slots__ = ['pan_rad', 'tilt_rad', 'points_laser', 'line_dir', 'line_mean',
                 'inlier_points', 'timestamp']

    def __init__(self):
        self.pan_rad = 0.0
        self.tilt_rad = 0.0
        self.points_laser = None      # Nx2 array of all valid (x,y) in laser frame
        self.line_dir = None           # unit direction of dominant line (2,)
        self.line_mean = None          # mean point on the line (2,)
        self.inlier_points = None      # Mx2 inlier points on the line
        self.timestamp = 0.0


class CalibrationAcquisition:
    """
    Automated grid-scan data acquisition for pan-tilt LiDAR calibration.

    1. Generates a grid of (pan, tilt) angles within user-specified limits
    2. Commands the PTU to each pose via /joint_command
    3. Waits for convergence, then captures /scan
    4. Extracts forward-facing FOV, runs RANSAC line fit
    5. Stores captures and saves to disk
    """

    def __init__(self):
        # Only init node if not already initialized (e.g. when called from pipeline)
        try:
            rospy.init_node('calibration_acquisition', anonymous=False)
        except rospy.exceptions.ROSException:
            pass  # already initialized by pipeline

        # ---------- Parameters ----------
        # Grid limits (degrees) - these define the pan/tilt range to sweep
        self.pan_limit_deg = rospy.get_param('~pan_limit_deg', 25.0)
        self.tilt_limit_deg = rospy.get_param('~tilt_limit_deg', 15.0)

        # Grid resolution
        self.pan_steps = rospy.get_param('~pan_steps', 5)
        self.tilt_steps = rospy.get_param('~tilt_steps', 5)

        # Forward-facing FOV half-angle for ROI extraction (degrees)
        # Section 4.9: alpha_max = 15° (30° total)
        self.fov_half_deg = rospy.get_param('~fov_half_deg', 15.0)

        # RANSAC parameters
        self.ransac_iterations = rospy.get_param('~ransac_iterations', 600)
        self.ransac_inlier_thresh_m = rospy.get_param('~ransac_inlier_thresh', 0.015)
        self.min_inliers = rospy.get_param('~min_inliers', 30)

        # Scan range filter
        self.range_min = rospy.get_param('~range_min', 0.10)
        self.range_max = rospy.get_param('~range_max', 4.0)

        # Motion parameters
        self.settle_time = rospy.get_param('~settle_time', 1.5)  # seconds to wait after reaching pose
        self.position_tolerance_deg = rospy.get_param('~position_tolerance_deg', 1.0)
        self.motion_timeout = rospy.get_param('~motion_timeout', 10.0)

        # Output
        self.output_dir = rospy.get_param('~output_dir',
                                          os.path.join(os.path.dirname(__file__), '..', 'results'))

        # ---------- State ----------
        self.scan_lock = Lock()
        self.js_lock = Lock()
        self.last_scan = None
        self.last_js = None
        self.have_scan = Event()
        self.captures = []

        # ---------- ROS I/O ----------
        self.cmd_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_cb, queue_size=5)
        self.js_sub = rospy.Subscriber('/joint_states', JointState, self._js_cb, queue_size=50)

        # ---------- RViz Visualization ----------
        self.viz_pub = rospy.Publisher('/calibration/points', MarkerArray,
                                       queue_size=5, latch=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.accumulated_base_pts = []  # [(Nx3 array, capture_idx), ...]

        rospy.loginfo("=" * 70)
        rospy.loginfo("  CALIBRATION DATA ACQUISITION")
        rospy.loginfo("=" * 70)
        rospy.loginfo(f"  Pan range:  ±{self.pan_limit_deg:.1f}°  ({self.pan_steps} steps)")
        rospy.loginfo(f"  Tilt range: ±{self.tilt_limit_deg:.1f}° ({self.tilt_steps} steps)")
        rospy.loginfo(f"  Total grid: {self.pan_steps * self.tilt_steps} poses")
        rospy.loginfo(f"  FOV half:   {self.fov_half_deg:.1f}° (ROI extraction)")
        rospy.loginfo(f"  RANSAC:     {self.ransac_iterations} iters, "
                      f"{self.ransac_inlier_thresh_m*1000:.1f} mm thresh")
        rospy.loginfo("=" * 70)

    # ------------------------------------------------------------------ #
    #  ROS Callbacks                                                       #
    # ------------------------------------------------------------------ #

    def _scan_cb(self, msg):
        with self.scan_lock:
            self.last_scan = msg
            self.have_scan.set()

    def _js_cb(self, msg):
        with self.js_lock:
            self.last_js = msg

    # ------------------------------------------------------------------ #
    #  PTU Commanding                                                      #
    # ------------------------------------------------------------------ #

    def _get_current_joints(self):
        """Return (tilt_rad, pan_rad) or (None, None)."""
        with self.js_lock:
            js = self.last_js
        if js is None:
            return None, None
        tilt, pan = None, None
        for i, name in enumerate(js.name):
            if name == 'joint1':
                tilt = js.position[i]
            elif name == 'joint2':
                pan = js.position[i]
        return tilt, pan

    def _command_pose(self, tilt_rad, pan_rad):
        """Publish a joint command."""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['joint1', 'joint2']
        msg.position = [tilt_rad, pan_rad]
        self.cmd_pub.publish(msg)

    def _go_to_pose(self, pan_deg, tilt_deg):
        """
        Command the PTU to a target pose and wait until it converges.
        Returns True on success, False on timeout.
        """
        pan_rad = math.radians(pan_deg)
        tilt_rad = math.radians(tilt_deg)
        tol_rad = math.radians(self.position_tolerance_deg)

        t0 = time.time()
        rate = rospy.Rate(50)  # command at 50 Hz like the existing system

        while not rospy.is_shutdown():
            self._command_pose(tilt_rad, pan_rad)

            cur_tilt, cur_pan = self._get_current_joints()
            if cur_tilt is not None and cur_pan is not None:
                if abs(cur_tilt - tilt_rad) <= tol_rad and abs(cur_pan - pan_rad) <= tol_rad:
                    # Reached target — keep commanding for settle_time
                    settle_end = time.time() + self.settle_time
                    while time.time() < settle_end and not rospy.is_shutdown():
                        self._command_pose(tilt_rad, pan_rad)
                        rate.sleep()
                    return True

            if (time.time() - t0) > self.motion_timeout:
                rospy.logwarn(f"Timeout reaching pan={pan_deg:.1f}° tilt={tilt_deg:.1f}°")
                return False

            rate.sleep()
        return False

    # ------------------------------------------------------------------ #
    #  Scan Processing                                                     #
    # ------------------------------------------------------------------ #

    def _extract_fov_points(self, scan):
        """
        Extract 2D laser points within ±fov_half_deg of the forward direction.
        Returns Nx2 numpy array in the laser frame.

        Section 4.9, Eq. 24:
          P_tilde_k = { p in P_k : |alpha(p)| < alpha_max }
        """
        fov_half_rad = math.radians(self.fov_half_deg)
        pts = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < self.range_min or r > self.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > fov_half_rad:
                continue
            pts.append([r * math.cos(angle), r * math.sin(angle)])

        if len(pts) == 0:
            return np.empty((0, 2))
        return np.array(pts)

    def _ransac_line_fit(self, pts):
        """
        RANSAC 2D line fit on Nx2 points.
        Returns (direction, mean, inlier_pts) or (None, None, None) on failure.
        """
        N = len(pts)
        if N < 2:
            return None, None, None

        th = max(1e-6, self.ransac_inlier_thresh_m)
        rng = np.random.default_rng()

        best_count = 0
        best_p1 = best_p2 = None

        for _ in range(self.ransac_iterations):
            idx = rng.choice(N, 2, replace=False)
            p1, p2 = pts[idx[0]], pts[idx[1]]
            v = p2 - p1
            nv = np.linalg.norm(v)
            if nv < 1e-6:
                continue
            v /= nv
            n = np.array([-v[1], v[0]])  # perpendicular

            dists = np.abs((pts - p1) @ n)
            count = np.sum(dists <= th)

            if count > best_count:
                best_count = count
                best_p1, best_p2 = p1, p2
                if best_count > 0.7 * N:
                    break

        if best_count < 2 or best_p1 is None:
            return None, None, None

        # Collect inliers for best model
        v = best_p2 - best_p1
        v /= np.linalg.norm(v)
        n = np.array([-v[1], v[0]])
        dists = np.abs((pts - best_p1) @ n)
        inlier_mask = dists <= th
        inliers = pts[inlier_mask]

        if len(inliers) < 2:
            return None, None, None

        # Refine direction via SVD
        mean = inliers.mean(axis=0)
        centered = inliers - mean
        _, _, Vt = np.linalg.svd(centered, full_matrices=False)
        direction = Vt[0]
        if direction[1] < 0:
            direction = -direction

        return direction, mean, inliers

    # ------------------------------------------------------------------ #
    #  Capture One Pose                                                    #
    # ------------------------------------------------------------------ #

    def _capture_at_current_pose(self, pan_deg, tilt_deg):
        """
        Grab the latest scan, extract FOV, fit line.
        Returns a ScanCapture or None on failure.
        """
        # Wait for a fresh scan
        self.have_scan.clear()
        if not self.have_scan.wait(timeout=2.0):
            rospy.logwarn("No /scan received within 2s")
            return None

        with self.scan_lock:
            scan = self.last_scan

        if scan is None:
            return None

        # Read actual joint positions at capture time
        cur_tilt, cur_pan = self._get_current_joints()
        if cur_tilt is None or cur_pan is None:
            rospy.logwarn("No /joint_states available")
            return None

        # Extract forward-facing FOV
        pts = self._extract_fov_points(scan)
        if len(pts) < self.min_inliers:
            rospy.logwarn(f"Only {len(pts)} points in FOV (need {self.min_inliers})")
            return None

        # RANSAC line fit
        direction, mean, inliers = self._ransac_line_fit(pts)
        if direction is None or len(inliers) < self.min_inliers:
            n_inliers = 0 if inliers is None else len(inliers)
            rospy.logwarn(f"RANSAC failed: {n_inliers} inliers (need {self.min_inliers})")
            return None

        # Build capture
        cap = ScanCapture()
        cap.pan_rad = cur_pan
        cap.tilt_rad = cur_tilt
        cap.points_laser = pts
        cap.line_dir = direction
        cap.line_mean = mean
        cap.inlier_points = inliers
        cap.timestamp = scan.header.stamp.to_sec()

        return cap

    # ------------------------------------------------------------------ #
    #  RViz Visualization                                                  #
    # ------------------------------------------------------------------ #

    def _publish_rviz_markers(self, cap, capture_idx):
        """Publish capture points to /calibration/points for RViz verification.

        Three marker layers:
          - fov_points     (gray, laser frame)  : all FOV points — context
          - inlier_points  (green, laser frame)  : RANSAC inliers — the wall
          - wall_accumulated (colored, base_link) : all captures on the wall
        """
        stamp = rospy.Time.now()
        markers = MarkerArray()

        # 1. All FOV points in laser frame (gray)
        mk_fov = Marker()
        mk_fov.header.frame_id = 'laser'
        mk_fov.header.stamp = stamp
        mk_fov.ns = 'fov_points'
        mk_fov.id = 0
        mk_fov.type = Marker.POINTS
        mk_fov.action = Marker.ADD
        mk_fov.pose.orientation.w = 1.0
        mk_fov.scale.x = 0.006
        mk_fov.scale.y = 0.006
        mk_fov.color = ColorRGBA(0.5, 0.5, 0.5, 0.4)
        mk_fov.lifetime = rospy.Duration(0)
        for pt in cap.points_laser:
            mk_fov.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=0.0))
        markers.markers.append(mk_fov)

        # 2. RANSAC inliers in laser frame (bright green)
        mk_in = Marker()
        mk_in.header.frame_id = 'laser'
        mk_in.header.stamp = stamp
        mk_in.ns = 'inlier_points'
        mk_in.id = 0
        mk_in.type = Marker.POINTS
        mk_in.action = Marker.ADD
        mk_in.pose.orientation.w = 1.0
        mk_in.scale.x = 0.010
        mk_in.scale.y = 0.010
        mk_in.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        mk_in.lifetime = rospy.Duration(0)
        for pt in cap.inlier_points:
            mk_in.points.append(Point(x=float(pt[0]), y=float(pt[1]), z=0.0))
        markers.markers.append(mk_in)

        # 3. Accumulated inliers in base_link (colored per capture)
        #    All captures should form a flat sheet on the wall.
        #    Use our own FK computation (independent of robot_state_publisher).
        T = None
        if _HAS_FK:
            try:
                T = forward_kinematics(cap.pan_rad, cap.tilt_rad, np.zeros(18))
            except Exception as e:
                rospy.logwarn(f"  FK computation failed: {e}")
        if T is None:
            # Fallback to TF tree (requires robot_state_publisher)
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    'base_link', 'laser', rospy.Time(0), rospy.Duration(1.0))
                tr = tf_msg.transform.translation
                q  = tf_msg.transform.rotation
                T  = self._quat_to_matrix(q.x, q.y, q.z, q.w)
                T[0, 3], T[1, 3], T[2, 3] = tr.x, tr.y, tr.z
            except Exception as e:
                rospy.logwarn(f"  TF lookup failed: {e}")
        if T is not None:
            pts_2d = cap.inlier_points
            N = len(pts_2d)
            pts_h = np.hstack([pts_2d, np.zeros((N, 1)), np.ones((N, 1))])
            pts_base = (T @ pts_h.T).T[:, :3]
            self.accumulated_base_pts.append((pts_base, capture_idx))

        mk_wall = Marker()
        mk_wall.header.frame_id = 'base_link'
        mk_wall.header.stamp = stamp
        mk_wall.ns = 'wall_accumulated'
        mk_wall.id = 0
        mk_wall.type = Marker.POINTS
        mk_wall.action = Marker.ADD
        mk_wall.pose.orientation.w = 1.0
        mk_wall.scale.x = 0.004
        mk_wall.scale.y = 0.004
        mk_wall.lifetime = rospy.Duration(0)
        for base_pts, idx in self.accumulated_base_pts:
            hue = (idx * 0.618033988749895) % 1.0  # golden ratio for color spread
            r, g, b = colorsys.hsv_to_rgb(hue, 0.9, 1.0)
            color = ColorRGBA(float(r), float(g), float(b), 1.0)
            for pt in base_pts:
                mk_wall.points.append(
                    Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2])))
                mk_wall.colors.append(color)
        markers.markers.append(mk_wall)

        self.viz_pub.publish(markers)

    @staticmethod
    def _quat_to_matrix(x, y, z, w):
        """Quaternion (x,y,z,w) to 4x4 homogeneous matrix."""
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y),   0],
            [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x),   0],
            [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y), 0],
            [0,             0,             0,             1]])

    # ------------------------------------------------------------------ #
    #  Grid Generation                                                     #
    # ------------------------------------------------------------------ #

    def _generate_grid(self):
        """
        Generate a list of (pan_deg, tilt_deg) covering the joint space uniformly.
        The grid spans ±pan_limit_deg and ±tilt_limit_deg.
        """
        if self.pan_steps == 1:
            pan_angles = [0.0]
        else:
            pan_angles = np.linspace(-self.pan_limit_deg, self.pan_limit_deg,
                                     self.pan_steps).tolist()

        if self.tilt_steps == 1:
            tilt_angles = [0.0]
        else:
            tilt_angles = np.linspace(-self.tilt_limit_deg, self.tilt_limit_deg,
                                      self.tilt_steps).tolist()

        grid = []
        for p in pan_angles:
            for t in tilt_angles:
                grid.append((p, t))

        return grid

    # ------------------------------------------------------------------ #
    #  Main Acquisition Loop                                               #
    # ------------------------------------------------------------------ #

    def run(self):
        """Execute the full grid-scan acquisition."""
        # Wait for initial data
        rospy.loginfo("Waiting for /scan and /joint_states ...")
        timeout_t = time.time() + 10.0
        while not rospy.is_shutdown():
            tilt, pan = self._get_current_joints()
            if tilt is not None and self.last_scan is not None:
                break
            if time.time() > timeout_t:
                rospy.logerr("Timed out waiting for /scan and /joint_states.")
                rospy.logerr("Make sure urg_node and the Arduino are running.")
                return False
            rospy.sleep(0.2)

        grid = self._generate_grid()
        n_total = len(grid)
        rospy.loginfo(f"\nStarting grid acquisition: {n_total} poses\n")

        # First go home
        rospy.loginfo("Homing to (0, 0) ...")
        self._go_to_pose(0.0, 0.0)

        for idx, (pan_deg, tilt_deg) in enumerate(grid):
            if rospy.is_shutdown():
                break

            rospy.loginfo("-" * 50)
            rospy.loginfo(f"Pose {idx+1}/{n_total}: pan={pan_deg:+.1f}° tilt={tilt_deg:+.1f}°")

            # Move to pose
            if not self._go_to_pose(pan_deg, tilt_deg):
                rospy.logwarn(f"  Skipping pose (motion timeout)")
                continue

            # Capture
            cap = self._capture_at_current_pose(pan_deg, tilt_deg)
            if cap is None:
                rospy.logwarn(f"  Capture FAILED — skipping")
                continue

            self.captures.append(cap)
            self._publish_rviz_markers(cap, len(self.captures) - 1)
            rospy.loginfo(f"  ✓ Captured | inliers={len(cap.inlier_points)} | "
                          f"actual pan={math.degrees(cap.pan_rad):+.2f}° "
                          f"tilt={math.degrees(cap.tilt_rad):+.2f}°")

        # Return home
        rospy.loginfo("\nReturning home ...")
        self._go_to_pose(0.0, 0.0)

        rospy.loginfo("=" * 70)
        rospy.loginfo(f"  ACQUISITION COMPLETE: {len(self.captures)}/{n_total} captures")
        rospy.loginfo("=" * 70)

        return len(self.captures) >= 4  # minimum viable

    def save(self, filename=None):
        """Save captures to a .npz file for the solver."""
        if not self.captures:
            rospy.logerr("No captures to save!")
            return None

        if filename is None:
            os.makedirs(self.output_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'calib_data_{ts}.npz')

        data = {
            'n_captures': len(self.captures),
            'fov_half_deg': self.fov_half_deg,
        }
        for i, cap in enumerate(self.captures):
            data[f'cap_{i}_pan_rad'] = cap.pan_rad
            data[f'cap_{i}_tilt_rad'] = cap.tilt_rad
            data[f'cap_{i}_line_dir'] = cap.line_dir
            data[f'cap_{i}_line_mean'] = cap.line_mean
            data[f'cap_{i}_inlier_points'] = cap.inlier_points
            data[f'cap_{i}_all_points'] = cap.points_laser
            data[f'cap_{i}_timestamp'] = cap.timestamp

        np.savez_compressed(filename, **data)
        sz_mb = os.path.getsize(filename) / (1024 * 1024)
        rospy.loginfo(f"Saved {len(self.captures)} captures → {filename} ({sz_mb:.2f} MB)")
        return filename


def main():
    try:
        acq = CalibrationAcquisition()
        success = acq.run()
        if success:
            path = acq.save()
            if path:
                rospy.loginfo(f"\n>>> Data file: {path}")
                rospy.loginfo(">>> Next: rosrun ptu_lidar_calib calibration_solver.py --data <file>")
        else:
            rospy.logerr("Acquisition failed (not enough captures).")
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
