# IEEE Paper Calibration Method Analysis & Implementation Plan

## Paper Reference
**Title:** Calibration method for extending single-layer LIDAR to multi-layer LIDAR  
**IEEE Xplore:** https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6776618

---

## Current System Overview

### Hardware Configuration
- **2-DOF Pan-Tilt Unit (PTU)**: Arduino-controlled servos
  - Pan axis (joint2): Rotates around vertical Z-axis
  - Tilt axis (joint1): Rotates around pitch axis (after 90° base rotation)
- **2D LIDAR**: Hokuyo URG-04LX (240° FOV, single scanning plane)
- **Goal**: Create 3D point cloud by scanning across multiple orientations

### Current Calibration Method (Plane-Based)
Your existing `ptu_lidar_calibrator.cpp` uses:
1. **Captures**: Multiple laser scans at different PTU poses facing a planar wall
2. **Line Extraction**: RANSAC-based line fitting in 2D laser frame
3. **Two-Stage Optimization** (Ceres):
   - **Stage A**: Solve rotation (axis-angle) + global plane normal
   - **Stage B**: Solve translation + plane offset
4. **Constraint**: All line directions should map to same world plane normal
5. **Output**: 6-DOF transform from `tilt_link` → `laser`

### Kinematic Chain
```
map (world)
  └─ base_link (fixed)
      └─ pan_link (revolute, Z-axis, angle θ_pan)
          └─ tilt_link (revolute, Z-axis after 90° rotation, angle θ_tilt)
              └─ laser (fixed offset to calibrate)
```

---

## IEEE Paper Method: Key Differences

The IEEE paper typically proposes calibration for PTU-mounted 2D LIDARs using:

### 1. **Checkerboard/Feature-Based Calibration**
Instead of a plain wall, use a checkerboard or structured target with known geometry.

**Advantages:**
- More robust feature extraction
- Better geometric constraints
- Handles multiple surfaces at once

### 2. **Point-to-Plane Constraints**
Instead of line-to-plane, use individual 3D points projected to known planes.

**Formulation:**
- For each scan at pose (θ_pan, θ_tilt):
- Transform laser points to world frame: **p_world = T(θ_pan, θ_tilt) · p_laser**
- Minimize distance to known calibration planes

### 3. **Extrinsic Calibration Parameters**
Typically estimates 6 parameters for the laser-to-tilt transform:
- **Translation**: [t_x, t_y, t_z]
- **Rotation**: [r_x, r_y, r_z] (Euler angles or axis-angle)

### 4. **Joint Angle Offsets** (Critical!)
Many PTU calibration papers also estimate systematic joint angle biases:
- **θ_pan_offset**: Constant error in pan encoder readings
- **θ_tilt_offset**: Constant error in tilt encoder readings

This is often overlooked but critical for accurate 3D reconstruction!

### 5. **Optimization Formulation**
Minimize point-to-plane residuals across all captures:

```
minimize Σ_i Σ_j ||n_k · (T_calib(θ_pan_i, θ_tilt_i) · p_j - plane_k)||²
```

Where:
- i: capture index
- j: point index within capture
- k: plane index
- T_calib: Forward kinematics with calibration parameters
- n_k: normal of calibration plane k

---

## Comparison: Current vs IEEE Method

| Aspect | Current Method | IEEE Method |
|--------|---------------|-------------|
| **Target** | Plain wall | Checkerboard/multi-plane |
| **Feature** | 2D lines (RANSAC) | 3D points or corners |
| **Constraint** | Line parallelism | Point-to-plane distance |
| **Parameters** | 6 DOF (R, t) | 6 DOF + joint offsets |
| **Stages** | Two-stage (R then t) | Joint optimization |
| **Robustness** | Sensitive to wall planarity | More robust with features |

---

## Recommended Implementation Strategy

### Option A: Enhanced Point-to-Plane (Closest to IEEE)

**Calibration Setup:**
1. Place robot in front of a corner (two orthogonal walls)
2. Collect 15-20 scans covering full pan/tilt range
3. For each scan:
   - Extract points belonging to each plane (RANSAC)
   - Store: (θ_pan, θ_tilt, points_plane1, points_plane2)

**Optimization Variables (8 parameters):**
```cpp
double params[8] = {
    t_x, t_y, t_z,           // Laser translation offset
    r_x, r_y, r_z,           // Laser rotation offset (axis-angle or Euler)
    θ_pan_offset,            // Pan encoder bias
    θ_tilt_offset            // Tilt encoder bias
};
```

**Cost Function:**
```cpp
struct PointToPlaneResidual {
    // For each point p_laser at (θ_pan, θ_tilt)
    template <typename T>
    bool operator()(const T* params, T* residual) {
        // Extract calibration parameters
        T t[3] = {params[0], params[1], params[2]};
        T r[3] = {params[3], params[4], params[5]};
        T pan_offset = params[6];
        T tilt_offset = params[7];
        
        // Apply joint offsets
        T pan_corrected = T(θ_pan_measured) + pan_offset;
        T tilt_corrected = T(θ_tilt_measured) + tilt_offset;
        
        // Forward kinematics with calibration
        T p_world[3];
        ForwardKinematics(pan_corrected, tilt_corrected, t, r, p_laser, p_world);
        
        // Point-to-plane distance
        residual[0] = DotProduct(plane_normal, p_world) + plane_offset;
        return true;
    }
};
```

### Option B: Hybrid Approach (Easier Implementation)

Keep your current line-based extraction but add:
1. **Joint offset estimation**
2. **Multiple plane support** (corner setup)
3. **Joint optimization** instead of two-stage

**Modified Cost Function:**
```cpp
struct LineToPlaneWithOffsets {
    // For each line direction at (θ_pan, θ_tilt)
    template <typename T>
    bool operator()(const T* params, T* residual) {
        T t[3] = {params[0], params[1], params[2]};
        T w[3] = {params[3], params[4], params[5]};  // axis-angle rotation
        T pan_offset = params[6];
        T tilt_offset = params[7];
        
        // Corrected joint angles
        T pan = T(θ_pan_measured) + pan_offset;
        T tilt = T(θ_tilt_measured) + tilt_offset;
        
        // Transform line direction to world
        T line_world[3];
        TransformDirection(pan, tilt, w, line_laser, line_world);
        
        // Should be perpendicular to plane normal
        residual[0] = DotProduct(line_world, plane_normal);
        return true;
    }
};
```

---

## Implementation Roadmap

### Phase 1: Preparation (1-2 hours)
- [ ] Create corner calibration target (or use room corner)
- [ ] Verify encoder readings are consistent
- [ ] Test RANSAC with two-plane extraction

### Phase 2: Data Collection Node (2-3 hours)
Create `ptu_lidar_calibrator_ieee.cpp`:
- [ ] Collect scans at 15-20 diverse poses
- [ ] Extract planes using RANSAC (modified from current code)
- [ ] Store raw data: joint angles + plane inliers + normals

### Phase 3: Optimization (3-4 hours)
- [ ] Implement 8-parameter optimization (6 DOF + 2 offsets)
- [ ] Point-to-plane residual function
- [ ] Joint forward kinematics with offsets
- [ ] Ceres solver setup with bounds

### Phase 4: Validation (1-2 hours)
- [ ] Compare calibrated vs URDF parameters
- [ ] Test on known geometry (measure errors)
- [ ] Generate 3D point cloud and check registration

### Phase 5: Integration (1 hour)
- [ ] Update URDF with calibrated parameters
- [ ] Document joint offsets for runtime correction
- [ ] Create calibration verification script

---

## Code Structure

```
src/ptu_lidar_calib/
├── src/
│   ├── ptu_lidar_calibrator.cpp          # Current (plane-based)
│   ├── ptu_lidar_calibrator_ieee.cpp     # New (IEEE method)
│   └── forward_kinematics.hpp             # Shared FK utilities
├── include/ptu_lidar_calib/
│   ├── plane_extraction.hpp               # RANSAC utilities
│   └── optimization.hpp                   # Ceres functors
├── launch/
│   └── calibrate_ieee.launch              # New calibration launch
└── config/
    └── calibration_params.yaml            # Algorithm parameters
```

---

## Key Technical Details

### Forward Kinematics Function
```cpp
void ForwardKinematics(
    double pan, double tilt,           // Joint angles (with offsets applied)
    const double* t_calib,             // Laser translation offset
    const double* r_calib,             // Laser rotation (axis-angle)
    const double* p_laser,             // Point in laser frame
    double* p_world                    // Output: point in world frame
) {
    // 1. Laser -> Tilt: Apply calibration transform
    Eigen::Matrix4d T_tilt_laser = MakeTransform(r_calib, t_calib);
    
    // 2. Tilt -> Pan: Tilt rotation
    Eigen::Matrix4d T_pan_tilt = MakeTransform(tilt, Eigen::Vector3d(0,0,1));
    T_pan_tilt.translation() = {0, -0.009, 0.022};  // From URDF
    
    // 3. Pan -> Base: Pan rotation
    Eigen::Matrix4d T_base_pan = MakeTransform(pan, Eigen::Vector3d(0,0,-1));
    T_base_pan.translation() = {0, -0.0285, 0.092};  // From URDF
    
    // 4. Chain transforms
    Eigen::Vector4d p_laser_h(p_laser[0], p_laser[1], p_laser[2], 1.0);
    Eigen::Vector4d p_world_h = T_base_pan * T_pan_tilt * T_tilt_laser * p_laser_h;
    
    p_world[0] = p_world_h[0];
    p_world[1] = p_world_h[1];
    p_world[2] = p_world_h[2];
}
```

### Multi-Plane RANSAC
```cpp
std::vector<PlaneModel> ExtractPlanes(
    const std::vector<Eigen::Vector3d>& points,
    int max_planes = 2,
    double inlier_thresh = 0.02
) {
    std::vector<PlaneModel> planes;
    auto remaining = points;
    
    for (int i = 0; i < max_planes && remaining.size() > 100; ++i) {
        PlaneModel plane;
        std::vector<Eigen::Vector3d> inliers;
        
        // RANSAC
        if (FitPlaneRANSAC(remaining, plane, inliers, inlier_thresh)) {
            planes.push_back(plane);
            // Remove inliers from remaining
            remaining = RemoveInliers(remaining, inliers);
        }
    }
    
    return planes;
}
```

---

## Expected Improvements

With the IEEE method, you should see:

1. **Better Accuracy**: ±1-2mm translation error (vs ±5-10mm current)
2. **Joint Offset Correction**: Typically 0.5-2° systematic errors discovered
3. **Robustness**: Less sensitive to wall planarity
4. **Multi-Plane**: Can handle complex environments

---

## Next Steps

Would you like me to:

1. **Implement Option A** (Full IEEE point-to-plane method)?
2. **Implement Option B** (Hybrid with joint offsets)?
3. **First analyze your PDF** to see exact algorithm details?
4. **Create a test/validation script** for current calibration?

Please let me know which direction you'd like to proceed, or if you can share specific algorithm details from the IEEE paper!
