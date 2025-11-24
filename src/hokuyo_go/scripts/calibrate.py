#!/usr/bin/env python3
"""
Two-Phase Pan-Tilt LiDAR Calibration System

PHASE 1: Data Collection (run WITH your laser_real_position node)
  - Passively records joint states and point clouds during normal scanning
  - Saves data to disk

PHASE 2: Offline Optimization (run AFTER data collection, no motors)
  - Loads saved data
  - Optimizes calibration parameters
  - Generates URDF corrections

Usage:
  Phase 1: rosrun your_package calibrate.py --collect --duration 120
  Phase 2: rosrun your_package calibrate.py --optimize --data calibration_data.npz
"""

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs.point_cloud2 as pc2
from scipy.optimize import differential_evolution
import argparse
import pickle
import yaml
import time
import os
from collections import deque

class CalibrationDataCollector:
    """Phase 1: Collect data during normal scanning operation"""
    
    def __init__(self, duration=120):
        rospy.init_node('calibration_collector', anonymous=False)
        
        self.duration = duration
        self.scan_data = []
        self.current_joints = None
        
        # Buffer for synchronization
        self.joint_buffer = deque(maxlen=100)  # Keep last 100 joint states
        
        # Subscribers (passive observation only)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback, queue_size=100)
        self.cloud_sub = rospy.Subscriber('output', PointCloud2, self.cloud_callback, queue_size=10)
        
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("="*70)
        rospy.loginfo("CALIBRATION DATA COLLECTOR - PHASE 1")
        rospy.loginfo("="*70)
        rospy.loginfo("This node will passively collect data while your scanning node runs.")
        rospy.loginfo(f"Collection duration: {duration} seconds")
        rospy.loginfo("Make sure your laser_real_position node is running!")
        rospy.loginfo("")
        rospy.loginfo("Start your scan now with:")
        rospy.loginfo("  roslaunch your_package scan.launch")
        rospy.loginfo("")
    
    def joint_callback(self, msg):
        """Store joint states with timestamps"""
        if len(msg.position) >= 2:
            joint_data = {
                'time': msg.header.stamp.to_sec(),
                'tilt': msg.position[0],
                'pan': msg.position[1]
            }
            self.joint_buffer.append(joint_data)
    
    def find_closest_joint_state(self, cloud_time):
        """Find joint state closest in time to point cloud"""
        if not self.joint_buffer:
            return None
        
        # Find closest timestamp
        min_dt = float('inf')
        closest = None
        
        for joint_data in self.joint_buffer:
            dt = abs(joint_data['time'] - cloud_time)
            if dt < min_dt:
                min_dt = dt
                closest = joint_data
        
        # Only accept if within 100ms
        if min_dt < 0.1:
            return closest
        return None
    
    def cloud_callback(self, msg):
        """Store point clouds with synchronized joint states"""
        # Check if collection time has elapsed
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed > self.duration:
            return
        
        # Find matching joint state
        cloud_time = msg.header.stamp.to_sec()
        joint_state = self.find_closest_joint_state(cloud_time)
        
        if joint_state is None:
            rospy.logwarn_throttle(5, "No synchronized joint state found for point cloud")
            return
        
        # Extract points
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        
        if len(points) < 500:  # Minimum quality threshold
            return
        
        # Downsample to reduce memory (keep every Nth point)
        points_array = np.array(points)
        if len(points_array) > 5000:
            indices = np.random.choice(len(points_array), 5000, replace=False)
            points_array = points_array[indices]
        
        # Store scan
        scan_data = {
            'pan': joint_state['pan'],
            'tilt': joint_state['tilt'],
            'points': points_array,
            'timestamp': cloud_time
        }
        self.scan_data.append(scan_data)
        
        # Progress reporting
        if len(self.scan_data) % 10 == 0:
            rospy.loginfo(f"Collected {len(self.scan_data)} scans | "
                         f"Elapsed: {elapsed:.1f}/{self.duration}s | "
                         f"Pan: {np.degrees(joint_state['pan']):.1f}° | "
                         f"Tilt: {np.degrees(joint_state['tilt']):.1f}°")
    
    def run(self):
        """Run collection until duration expires"""
        rospy.loginfo("Waiting for data...")
        
        rate = rospy.Rate(1)  # 1Hz status updates
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            
            if elapsed > self.duration:
                rospy.loginfo(f"\nCollection complete: {len(self.scan_data)} scans collected")
                break
            
            rate.sleep()
        
        return self.scan_data
    
    def save_data(self, filename='calibration_data.npz'):
        """Save collected data to file"""
        if not self.scan_data:
            rospy.logerr("No data to save!")
            return False
        
        # Prepare data for saving
        data_dict = {
            'num_scans': len(self.scan_data),
            'collection_time': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        
        # Save each scan
        for i, scan in enumerate(self.scan_data):
            data_dict[f'scan_{i}_pan'] = scan['pan']
            data_dict[f'scan_{i}_tilt'] = scan['tilt']
            data_dict[f'scan_{i}_points'] = scan['points']
            data_dict[f'scan_{i}_time'] = scan['timestamp']
        
        np.savez_compressed(filename, **data_dict)
        rospy.loginfo(f"Data saved to {filename}")
        rospy.loginfo(f"File size: {os.path.getsize(filename) / 1024 / 1024:.2f} MB")
        
        return True


class CalibrationOptimizer:
    """Phase 2: Offline optimization of calibration parameters"""
    
    def __init__(self, data_file):
        self.scan_data = []
        self.load_data(data_file)
        
        # Initial URDF parameters
        self.initial_params = np.array([
            0.0, -0.009, 0.022, 0.0, 1.57, 0.0,  # joint1 (tilt) origin
            0.0, -0.0285, 0.092, 0.0, 0.0, -1.0, # joint2 (pan) origin + axis
            -0.061, 0.014, 0.040, 1.57, 3.14, -1.57  # laser offset
        ])
        
        print("\n" + "="*70)
        print("CALIBRATION OPTIMIZER - PHASE 2")
        print("="*70)
        print(f"Loaded {len(self.scan_data)} scans")
        print(f"Joint range: Pan [{self.get_joint_range('pan')}°], "
              f"Tilt [{self.get_joint_range('tilt')}°]")
        print("="*70 + "\n")
    
    def load_data(self, filename):
        """Load collected data from file"""
        print(f"Loading data from {filename}...")
        data = np.load(filename, allow_pickle=True)
        
        num_scans = int(data['num_scans'])
        
        for i in range(num_scans):
            scan = {
                'pan': float(data[f'scan_{i}_pan']),
                'tilt': float(data[f'scan_{i}_tilt']),
                'points': data[f'scan_{i}_points'],
                'timestamp': float(data[f'scan_{i}_time'])
            }
            self.scan_data.append(scan)
        
        print(f"Loaded {len(self.scan_data)} scans")
    
    def get_joint_range(self, joint):
        """Get min/max range of joint values"""
        if joint == 'pan':
            values = [s['pan'] for s in self.scan_data]
        else:
            values = [s['tilt'] for s in self.scan_data]
        return f"{np.degrees(np.min(values)):.1f} to {np.degrees(np.max(values)):.1f}"
    
    def extract_planes_ransac(self, points, num_planes=2, min_points=200):
        """Extract planar surfaces using RANSAC"""
        planes = []
        remaining = points.copy()
        
        for _ in range(num_planes):
            if len(remaining) < min_points:
                break
            
            best_inliers = []
            best_model = None
            
            # RANSAC iterations
            for _ in range(150):
                if len(remaining) < 3:
                    break
                
                # Random sample
                idx = np.random.choice(len(remaining), 3, replace=False)
                sample = remaining[idx]
                
                # Fit plane
                v1 = sample[1] - sample[0]
                v2 = sample[2] - sample[0]
                normal = np.cross(v1, v2)
                
                if np.linalg.norm(normal) < 1e-6:
                    continue
                
                normal = normal / np.linalg.norm(normal)
                d = -np.dot(normal, sample[0])
                
                # Find inliers (2cm threshold)
                distances = np.abs(np.dot(remaining, normal) + d)
                inliers_mask = distances < 0.02
                inliers = remaining[inliers_mask]
                
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_model = (normal, d)
            
            if best_model and len(best_inliers) >= min_points:
                planes.append({
                    'normal': best_model[0],
                    'd': best_model[1],
                    'points': best_inliers,
                    'centroid': np.mean(best_inliers, axis=0)
                })
                
                # Remove inliers
                mask = np.ones(len(remaining), dtype=bool)
                for i, pt in enumerate(remaining):
                    if any(np.allclose(pt, inlier, atol=1e-6) for inlier in best_inliers[:100]):
                        mask[i] = False
                remaining = remaining[mask]
        
        return planes
    
    def axis_angle_to_matrix(self, axis, angle):
        """Convert axis-angle to rotation matrix using Rodrigues formula"""
        axis = axis / (np.linalg.norm(axis) + 1e-10)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        return R
    
    def compute_transform_chain(self, pan, tilt, params):
        """Compute forward kinematics with calibration parameters"""
        # Extract parameters
        j1_trans = params[0:3]
        j1_rpy = params[3:6]
        j2_trans = params[6:9]
        j2_axis = params[9:12]
        j2_axis = j2_axis / (np.linalg.norm(j2_axis) + 1e-10)
        laser_trans = params[12:15]
        laser_rpy = params[15:18]
        
        # Build transformation matrices
        # 1. Base to pan link (joint2 - pan motion)
        T_base_pan = np.eye(4)
        pan_rot = self.axis_angle_to_matrix(j2_axis, pan)
        T_base_pan[0:3, 0:3] = pan_rot
        T_base_pan[0:3, 3] = j2_trans
        
        # 2. Pan link to tilt link (joint1 - tilt motion)
        T_pan_tilt = np.eye(4)
        # Base rotation from RPY
        cr, cp, cy = np.cos(j1_rpy)
        sr, sp, sy = np.sin(j1_rpy)
        base_rot = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        # Tilt rotation around z-axis (after 90deg base rotation)
        tilt_axis = np.array([0, 0, 1])
        tilt_rot = self.axis_angle_to_matrix(tilt_axis, tilt)
        T_pan_tilt[0:3, 0:3] = tilt_rot @ base_rot
        T_pan_tilt[0:3, 3] = j1_trans
        
        # 3. Tilt link to laser
        T_tilt_laser = np.eye(4)
        cr, cp, cy = np.cos(laser_rpy)
        sr, sp, sy = np.sin(laser_rpy)
        laser_rot = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        T_tilt_laser[0:3, 0:3] = laser_rot
        T_tilt_laser[0:3, 3] = laser_trans
        
        # Chain: base -> pan -> tilt -> laser -> map
        T_total = T_base_pan @ T_pan_tilt @ T_tilt_laser
        
        return T_total
    
    def calibration_cost(self, params):
        """Cost function: plane consistency across views"""
        all_planes = []
        
        # Subsample scans for speed (use every 3rd scan)
        scan_subset = self.scan_data[::3]
        
        # Transform each scan and extract planes
        for scan in scan_subset:
            T = self.compute_transform_chain(scan['pan'], scan['tilt'], params)
            
            # Transform points to map frame
            points_hom = np.hstack([scan['points'], np.ones((len(scan['points']), 1))])
            transformed = (T @ points_hom.T).T[:, 0:3]
            
            # Extract planes
            planes = self.extract_planes_ransac(transformed, num_planes=2, min_points=150)
            all_planes.extend(planes)
        
        if len(all_planes) < 3:
            return 1e8
        
        # Cost: plane consistency
        plane_cost = 0.0
        comparisons = 0
        
        for i in range(len(all_planes)):
            for j in range(i+1, min(i+20, len(all_planes))):  # Limit comparisons
                normal_similarity = abs(np.dot(all_planes[i]['normal'], 
                                              all_planes[j]['normal']))
                
                if normal_similarity > 0.95:  # Same physical plane
                    # Distance between parallel planes
                    d_diff = abs(all_planes[i]['d'] - all_planes[j]['d'])
                    plane_cost += d_diff ** 2
                    
                    # Point-to-plane distances (subsample)
                    sample_pts = all_planes[j]['points'][::20]
                    for pt in sample_pts:
                        dist = abs(np.dot(pt, all_planes[i]['normal']) + all_planes[i]['d'])
                        plane_cost += dist ** 2 * 0.05
                    
                    comparisons += 1
        
        if comparisons == 0:
            return 1e8
        
        # Regularization
        param_diff = params - self.initial_params
        regularization = 0.05 * np.sum(param_diff ** 2)
        
        return plane_cost / comparisons + regularization
    
    def optimize(self):
        """Run optimization"""
        print("\nStarting optimization...")
        print("This will take 10-30 minutes depending on data size.\n")
        
        # Parameter bounds
        bounds = [
            # joint1 translation (±5cm from initial)
            (-0.05, 0.05), (-0.06, 0.04), (-0.03, 0.07),
            # joint1 rotation (±15 deg around initial)
            (-0.25, 0.25), (1.3, 1.85), (-0.25, 0.25),
            # joint2 translation (±5cm)
            (-0.05, 0.05), (-0.08, 0.02), (0.04, 0.14),
            # joint2 axis (allow variation)
            (-1, 1), (-1, 1), (-1, 1),
            # laser translation (±5cm)
            (-0.11, -0.01), (-0.04, 0.06), (-0.01, 0.09),
            # laser rotation (±20 deg)
            (1.2, 1.95), (2.8, 3.5), (-1.95, -1.2)
        ]
        
        iteration = [0]
        
        def callback(xk, convergence):
            iteration[0] += 1
            cost = self.calibration_cost(xk)
            print(f"Iteration {iteration[0]:3d} | Cost: {cost:12.6f} | Convergence: {convergence:.6f}")
            return False
        
        result = differential_evolution(
            self.calibration_cost,
            bounds,
            maxiter=40,
            popsize=12,
            mutation=(0.5, 1.5),
            recombination=0.7,
            disp=False,
            workers=1,
            updating='deferred',
            callback=callback,
            seed=42
        )
        
        self.optimized_params = result.x
        
        print(f"\n{'='*70}")
        print("OPTIMIZATION COMPLETE")
        print(f"{'='*70}")
        print(f"Final cost: {result.fun:.6f}")
        print(f"Success: {result.success}")
        print(f"Message: {result.message}")
        
        return result
    
    def print_results(self):
        """Print calibration results"""
        params = self.optimized_params
        axis_norm = params[9:12] / np.linalg.norm(params[9:12])
        
        print("\n" + "="*70)
        print("CALIBRATION CORRECTIONS")
        print("="*70)
        
        print("\n1. JOINT1 (Tilt) - Origin Correction:")
        print(f"   Original: xyz=\"0.000 -0.009 0.022\" rpy=\"0.000 1.570 0.000\"")
        print(f"   New:      xyz=\"{params[0]:.6f} {params[1]:.6f} {params[2]:.6f}\" "
              f"rpy=\"{params[3]:.6f} {params[4]:.6f} {params[5]:.6f}\"")
        print(f"   Δ:        xyz=\"{params[0]:.4f} {params[1]+0.009:.4f} {params[2]-0.022:.4f}\" "
              f"rpy=\"{params[3]:.4f} {params[4]-1.57:.4f} {params[5]:.4f}\"")
        
        print("\n2. JOINT2 (Pan) - Origin Correction:")
        print(f"   Original: xyz=\"0.000 -0.0285 0.092\"")
        print(f"   New:      xyz=\"{params[6]:.6f} {params[7]:.6f} {params[8]:.6f}\"")
        print(f"   Δ:        xyz=\"{params[6]:.4f} {params[7]+0.0285:.4f} {params[8]-0.092:.4f}\"")
        
        print("\n3. JOINT2 (Pan) - Axis Correction:")
        print(f"   Original: axis=\"0.000 0.000 -1.000\"")
        print(f"   New:      axis=\"{axis_norm[0]:.6f} {axis_norm[1]:.6f} {axis_norm[2]:.6f}\"")
        
        print("\n4. LASER (Hokuyo) - Mounting Correction:")
        print(f"   Original: xyz=\"-0.061 0.014 0.040\" rpy=\"1.570 3.140 -1.570\"")
        print(f"   New:      xyz=\"{params[12]:.6f} {params[13]:.6f} {params[14]:.6f}\" "
              f"rpy=\"{params[15]:.6f} {params[16]:.6f} {params[17]:.6f}\"")
        print(f"   Δ:        xyz=\"{params[12]+0.061:.4f} {params[13]-0.014:.4f} {params[14]-0.040:.4f}\" "
              f"rpy=\"{params[15]-1.57:.4f} {params[16]-3.14:.4f} {params[17]+1.57:.4f}\"")
        
        print("\n" + "="*70)
        print("UPDATED URDF")
        print("="*70)
        print(f"""
<joint name="joint1" type="revolute">
    <parent link="pan_link" />
    <child link="tilt_link" />
    <origin xyz="{params[0]:.6f} {params[1]:.6f} {params[2]:.6f}" 
            rpy="{params[3]:.6f} {params[4]:.6f} {params[5]:.6f}" />
    <axis xyz="0 0 1" />
    <limit effort="10" lower="-0.8727" upper="0.8727" velocity="3" />
    <dynamics damping="1.0" />
</joint>

<joint name="joint2" type="revolute">
    <parent link="base_link" />
    <child link="pan_link" />
    <origin xyz="{params[6]:.6f} {params[7]:.6f} {params[8]:.6f}" rpy="0 0 0" />
    <axis xyz="{axis_norm[0]:.6f} {axis_norm[1]:.6f} {axis_norm[2]:.6f}" />
    <limit effort="10" lower="-3.1416" upper="3.1416" velocity="3" />
    <dynamics damping="1.0" />
</joint>

<joint name="hokuyo_joint" type="fixed">
    <origin xyz="{params[12]:.6f} {params[13]:.6f} {params[14]:.6f}" 
            rpy="{params[15]:.6f} {params[16]:.6f} {params[17]:.6f}" />
    <parent link="hokuyo_base" />
    <child link="laser" />
    <dynamics damping="1.0" />
</joint>
""")
        print("="*70)
    
    def save_results(self, filename='calibration_results.yaml'):
        """Save results to YAML"""
        params = self.optimized_params
        axis_norm = params[9:12] / np.linalg.norm(params[9:12])
        
        results = {
            'calibration_date': time.strftime('%Y-%m-%d %H:%M:%S'),
            'num_scans_used': len(self.scan_data),
            'joint1': {
                'xyz': [float(params[0]), float(params[1]), float(params[2])],
                'rpy': [float(params[3]), float(params[4]), float(params[5])]
            },
            'joint2': {
                'xyz': [float(params[6]), float(params[7]), float(params[8])],
                'axis': [float(axis_norm[0]), float(axis_norm[1]), float(axis_norm[2])]
            },
            'laser': {
                'xyz': [float(params[12]), float(params[13]), float(params[14])],
                'rpy': [float(params[15]), float(params[16]), float(params[17])]
            }
        }
        
        with open(filename, 'w') as f:
            yaml.dump(results, f, default_flow_style=False)
        
        print(f"\nResults saved to {filename}")


def main():
    parser = argparse.ArgumentParser(description='Pan-Tilt LiDAR Calibration')
    parser.add_argument('--collect', action='store_true', help='Phase 1: Collect calibration data')
    parser.add_argument('--optimize', action='store_true', help='Phase 2: Optimize calibration')
    parser.add_argument('--duration', type=int, default=120, help='Collection duration in seconds')
    parser.add_argument('--data', type=str, default='calibration_data.npz', help='Data file path')
    
    args = parser.parse_args()
    
    if args.collect:
        # Phase 1: Data collection
        collector = CalibrationDataCollector(duration=args.duration)
        collector.run()
        collector.save_data(args.data)
        
        print("\n" + "="*70)
        print("PHASE 1 COMPLETE")
        print("="*70)
        print(f"\nNext steps:")
        print(f"1. Stop your scanning node (Ctrl+C)")
        print(f"2. Run optimization:")
        print(f"   python3 calibrate.py --optimize --data {args.data}")
        print("="*70)
        
    elif args.optimize:
        # Phase 2: Optimization
        if not os.path.exists(args.data):
            print(f"Error: Data file '{args.data}' not found!")
            print("Run with --collect first to gather data.")
            return
        
        optimizer = CalibrationOptimizer(args.data)
        optimizer.optimize()
        optimizer.print_results()
        optimizer.save_results()
        
    else:
        parser.print_help()


if __name__ == '__main__':
    main()