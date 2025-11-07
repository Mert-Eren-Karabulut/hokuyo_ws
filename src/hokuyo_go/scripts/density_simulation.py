import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import time

# ========================================
# Scanner Parameters (from C++ code)
# ========================================

V_TARGET = 1.0                          # Target velocity (rad/s)
PAN_MAX= 150.0 * np.pi / 180.0        # Max Pan angle (radians) 
TILT_MAX = 45.0 * np.pi / 180.0       # Max Tilt angle (radians)
PAN_MIN= -150.0 * np.pi / 180.0       # Min Pan angle (radians)
TILT_MIN = -60.0 * np.pi / 180.0      # Min Tilt angle (radians)
DELTA_1 = 60.0 * np.pi / 180.0         # Pattern Pan limit (radians)
DELTA_2 = 45.0 * np.pi / 180.0         # Pattern Tilt limit (radians)
OMEGA = 3.0 + np.sqrt(2) / 100.0       # Irrational frequency
SCAN_RATE = 10.0                        # Hz
LIDAR_FOV = 240.0 * np.pi / 180.0      # 240 degrees
LIDAR_POINTS = 683
ANGULAR_SPACING = 0.36 * np.pi / 180.0  # 0.36 degrees
SPHERE_RADIUS = 1.0                     # meters

# Focal point offsets (center of scanning pattern)
PHI_OFFSET = 45.0 * np.pi / 180.0       # Pan offset (radians) - default 0° (x+)
THETA_OFFSET = 0.0 * np.pi / 180.0     # Tilt offset (radians) - default 0°

# ========================================
# Parametric Scanning Functions
# ========================================

def f1(t):
    """Pan function with offset"""
    return PHI_OFFSET + DELTA_1 * np.sin(t)

def f2(t):
    """Tilt function with offset"""
    return THETA_OFFSET + (DELTA_2*2) * (np.cos(OMEGA * t) + 1.0) / 2.0 - DELTA_2

def calculate_speed(t, dt=0.001):
    """Calculate instantaneous speed along parametric curve"""
    df1 = (f1(t + dt) - f1(t)) / dt
    df2 = (f2(t + dt) - f2(t)) / dt
    return np.sqrt(df1**2 + df2**2)

def generate_trajectory(scan_duration):
    """
    Generate scanner trajectory over scan_duration seconds.
    Returns arrays of pan and tilt angles at 10Hz intervals.
    """
    n_samples = int(scan_duration * SCAN_RATE)
    pan_angles = np.zeros(n_samples)
    tilt_angles = np.zeros(n_samples)
    
    t_param = 0.0
    dt_sample = 1.0 / SCAN_RATE  # 0.1 seconds between samples
    
    for i in range(n_samples):
        # Record current position
        pan_angles[i] = f1(t_param)
        tilt_angles[i] = f2(t_param)
        
        # Advance parameter time for constant velocity
        current_speed = calculate_speed(t_param)
        if current_speed > 0.0:
            delta_s = V_TARGET * dt_sample
            delta_t_param = delta_s / current_speed
            t_param += delta_t_param
        else:
            t_param += dt_sample
    
    return pan_angles, tilt_angles

# ========================================
# Coordinate Transformations
# ========================================

def spherical_to_cartesian(phi, theta):
    """
    Convert spherical coordinates to Cartesian on unit sphere.
    phi: pan angle (rotation around Z)
    theta: tilt angle (elevation from XY plane)
    """
    x = np.cos(theta) * np.cos(phi)
    y = np.cos(theta) * np.sin(phi)
    z = np.sin(theta)
    return np.array([x, y, z])

def rotation_z(angle):
    """Rotation matrix around Z axis"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def rotation_y(angle):
    """Rotation matrix around Y axis"""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])

def is_point_in_fov(point_query, pan_scanner, tilt_scanner):
    """
    Check if a point on the sphere is visible by the lidar at given scanner pose.
    
    point_query: [x, y, z] in world frame (on unit sphere)
    pan_scanner: current pan angle of scanner
    tilt_scanner: current tilt angle of scanner
    
    Returns: True if point is in lidar's FOV
    """
    # Transform point to lidar frame (inverse of scanner rotation)
    R_z_inv = rotation_z(-pan_scanner)
    R_y_inv = rotation_y(-tilt_scanner)
    point_lidar = R_y_inv @ R_z_inv @ point_query
    
    x_lidar = point_lidar[0]
    y_lidar = point_lidar[1]
    z_lidar = point_lidar[2]
    
    # Calculate angle in the XY scanning plane
    alpha = np.arctan2(y_lidar, x_lidar)
    
    # Check if within ±120° FOV
    fov_limit = LIDAR_FOV / 2.0  # ±120°
    in_fov = np.abs(alpha) <= fov_limit
    
    # Check if point is in front (optional, but reasonable)
    in_front = x_lidar > -0.5  # Allow some wraparound
    
    return in_fov and in_front

# ========================================
# Point Density Calculation
# ========================================

def calculate_point_density(phi_query, theta_query, scan_duration):
    """
    Calculate point density at a specific location on the sphere.
    
    phi_query: pan angle of query point (radians)
    theta_query: tilt angle of query point (radians)
    scan_duration: duration of scan (seconds)
    
    Returns: point density (points per square meter)
    """
    # Generate scanner trajectory
    pan_trajectory, tilt_trajectory = generate_trajectory(scan_duration)
    
    # Convert query point to Cartesian
    point_query = spherical_to_cartesian(phi_query, theta_query)
    
    # Count how many times this point is visible
    n_visible = 0
    for pan_scanner, tilt_scanner in zip(pan_trajectory, tilt_trajectory):
        if is_point_in_fov(point_query, pan_scanner, tilt_scanner):
            n_visible += 1
    
    # Calculate density
    # Area covered by one lidar ray on unit sphere
    delta_area = ANGULAR_SPACING  # For small angles on unit sphere
    density = n_visible / delta_area
    
    return density, n_visible

def calculate_density_grid(scan_duration, resolution=30):
    """
    Calculate point density over entire sphere surface.
    
    scan_duration: duration of scan (seconds)
    resolution: number of samples in each dimension
    
    Returns: phi_grid, theta_grid, density_grid
    """
    # Create grid of pan and tilt angles
    phi_range = np.linspace(-np.pi, np.pi, resolution)
    theta_range = np.linspace(-np.pi/2, np.pi/2, resolution)
    
    phi_grid, theta_grid = np.meshgrid(phi_range, theta_range)
    density_grid = np.zeros_like(phi_grid)
    
    # Generate trajectory once
    pan_trajectory, tilt_trajectory = generate_trajectory(scan_duration)
    
    print(f"Calculating density grid ({resolution}x{resolution} points)...")
    start_time = time.time()
    
    # Calculate density for each grid point
    total_points = resolution * resolution
    for i in range(resolution):
        for j in range(resolution):
            phi_q = phi_grid[i, j]
            theta_q = theta_grid[i, j]
            
            # Convert to Cartesian
            point_query = spherical_to_cartesian(phi_q, theta_q)
            
            # Count visible instances
            n_visible = 0
            for pan_scanner, tilt_scanner in zip(pan_trajectory, tilt_trajectory):
                if is_point_in_fov(point_query, pan_scanner, tilt_scanner):
                    n_visible += 1
            
            # Calculate density
            delta_area = ANGULAR_SPACING
            density_grid[i, j] = n_visible / delta_area
        
        # Progress indicator
        if (i + 1) % 5 == 0:
            elapsed = time.time() - start_time
            progress = ((i + 1) * resolution) / total_points * 100
            print(f"  Progress: {progress:.1f}% ({elapsed:.1f}s elapsed)")
    
    elapsed = time.time() - start_time
    print(f"Calculation complete in {elapsed:.1f}s")
    
    return phi_grid, theta_grid, density_grid

# ========================================
# Visualization Functions
# ========================================

def visualize_density_heatmap(phi_grid, theta_grid, density_grid, scan_duration):
    """2D heatmap visualization of point density"""
    fig, ax = plt.subplots(figsize=(12, 8))
    
    im = ax.contourf(phi_grid * 180/np.pi, theta_grid * 180/np.pi, 
                     density_grid, levels=50, cmap='hot')
    
    ax.set_xlabel('Pan Angle (degrees)', fontsize=12)
    ax.set_ylabel('Tilt Angle (degrees)', fontsize=12)
    ax.set_title(f'Point Density Heatmap\nScan Duration: {scan_duration}s', fontsize=14)
    ax.grid(True, alpha=0.3)
    
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Points per m²', fontsize=12)
    
    plt.tight_layout()
    return fig

def visualize_density_3d_sphere(phi_grid, theta_grid, density_grid, scan_duration):
    """3D sphere visualization with density as color"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert to Cartesian coordinates
    x = SPHERE_RADIUS * np.cos(theta_grid) * np.cos(phi_grid)
    y = SPHERE_RADIUS * np.cos(theta_grid) * np.sin(phi_grid)
    z = SPHERE_RADIUS * np.sin(theta_grid)
    
    # Normalize density for color mapping
    norm_density = density_grid / np.max(density_grid) if np.max(density_grid) > 0 else density_grid
    
    # Plot surface
    surf = ax.plot_surface(x, y, z, facecolors=cm.hot(norm_density),
                           linewidth=0, antialiased=True, alpha=0.9)
    
    # x+ indicator arrow
    ax.quiver(0, 0, 0, SPHERE_RADIUS + 0.1, 0, 0, color='blue', arrow_length_ratio=0.1, linewidth=2)
    
    ax.set_xlabel('X (m)', fontsize=10)
    ax.set_ylabel('Y (m)', fontsize=10)
    ax.set_zlabel('Z (m)', fontsize=10)
    ax.set_title(f'3D Point Density on Sphere\nScan Duration: {scan_duration}s', fontsize=14)
    
    # Add colorbar
    m = cm.ScalarMappable(cmap=cm.hot)
    m.set_array(density_grid)
    cbar = plt.colorbar(m, ax=ax, shrink=0.5, aspect=5)
    cbar.set_label('Points per m²', fontsize=10)
    
    # Set equal aspect ratio
    max_range = SPHERE_RADIUS
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    
    plt.tight_layout()
    return fig

def visualize_trajectory(scan_duration):
    """Visualize the scanner trajectory in pan-tilt space"""
    pan_trajectory, tilt_trajectory = generate_trajectory(scan_duration)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    time_points = np.arange(len(pan_trajectory)) / SCAN_RATE
    
    # Pan angle over time
    ax1.plot(time_points, pan_trajectory * 180/np.pi, 'b-', linewidth=1.5)
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Pan Angle (degrees)', fontsize=11)
    ax1.set_title('Scanner Pan Motion', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=(DELTA_1+PHI_OFFSET)*180/np.pi, color='r', linestyle='--', alpha=0.5, label='Pattern Pan limit')
    ax1.axhline(y=(-DELTA_1+PHI_OFFSET)*180/np.pi, color='r', linestyle='--', alpha=0.5)
    # mechanism limits
    ax1.axhline(y=PAN_MAX*180/np.pi, color='g', linestyle='--', alpha=0.5, label='Mechanism Pan limit')
    ax1.axhline(y=PAN_MIN*180/np.pi, color='g', linestyle='--', alpha=0.5)
    ax1.legend()
    
    # Tilt angle over time
    ax2.plot(time_points, tilt_trajectory * 180/np.pi, 'g-', linewidth=1.5)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Tilt Angle (degrees)', fontsize=11)
    ax2.set_title('Scanner Tilt Motion', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=(DELTA_2+THETA_OFFSET)*180/np.pi, color='r', linestyle='--', alpha=0.5, label='Pattern Tilt limit')
    ax2.axhline(y=(-DELTA_2+THETA_OFFSET)*180/np.pi, color='r', linestyle='--', alpha=0.5)
    # mechanism limits
    ax2.axhline(y=TILT_MAX*180/np.pi, color='g', linestyle='--', alpha=0.5, label='Mechanism Tilt limit')
    ax2.axhline(y=TILT_MIN*180/np.pi, color='g', linestyle='--', alpha=0.5)
    ax2.legend()
    
    plt.tight_layout()
    return fig

def plot_trajectory_2d(scan_duration):
    """Plot 2D trajectory in pan-tilt space"""
    pan_trajectory, tilt_trajectory = generate_trajectory(scan_duration)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot trajectory with color gradient for time
    points = ax.scatter(pan_trajectory * 180/np.pi, tilt_trajectory * 180/np.pi,
                       c=np.arange(len(pan_trajectory)), cmap='viridis',
                       s=2, alpha=0.6)
    
    ax.set_xlabel('Pan Angle (degrees)', fontsize=12)
    ax.set_ylabel('Tilt Angle (degrees)', fontsize=12)
    ax.set_title(f'Scanner Trajectory in Joint Space\nDuration: {scan_duration}s', fontsize=14)
    ax.grid(True, alpha=0.3)
    
    # Add limits
    ax.axvline(x=(DELTA_1+PHI_OFFSET) *180/np.pi, color='r', linestyle='--', alpha=0.5, label='Pattern Pan limit')
    ax.axvline(x=(-DELTA_1+PHI_OFFSET) *180/np.pi, color='r', linestyle='--', alpha=0.5)
    ax.axhline(y=(DELTA_2+THETA_OFFSET) *180/np.pi, color='r', linestyle='--', alpha=0.5, label='Pattern Tilt limit')
    ax.axhline(y=(-DELTA_2+THETA_OFFSET) *180/np.pi, color='r', linestyle='--', alpha=0.5)
    # mechanism limits
    ax.axvline(x=PAN_MAX*180/np.pi, color='g', linestyle='--', alpha=0.5, label='Mechanism Pan limit')
    ax.axvline(x=PAN_MIN*180/np.pi, color='g', linestyle='--', alpha=0.5)
    ax.axhline(y=TILT_MAX*180/np.pi, color='g', linestyle='--', alpha=0.5, label='Mechanism Tilt limit')
    ax.axhline(y=TILT_MIN*180/np.pi, color='g', linestyle='--', alpha=0.5)
    ax.legend()

    cbar = plt.colorbar(points, ax=ax)
    cbar.set_label('Time Step', fontsize=11)
    
    plt.tight_layout()
    return fig

# ========================================
# Main Execution
# ========================================

if __name__ == "__main__":

    #test if our offsets are within mechanism limits when combined with pattern limits
    assert (PHI_OFFSET + DELTA_1) <= PAN_MAX, "Pan offset + pattern limit exceeds mechanism max by " + str((PHI_OFFSET + DELTA_1 - PAN_MAX)*180/np.pi) + "°"
    assert (PHI_OFFSET - DELTA_1) >= PAN_MIN, "Pan offset - pattern limit exceeds mechanism min by " + str((PHI_OFFSET - DELTA_1 - PAN_MIN)*180/np.pi) + "°"
    assert (THETA_OFFSET + DELTA_2) <= TILT_MAX, "Tilt offset + pattern limit exceeds mechanism max by " + str((THETA_OFFSET + DELTA_2 - TILT_MAX)*180/np.pi) + "°"
    assert (THETA_OFFSET - DELTA_2) >= TILT_MIN, "Tilt offset - pattern limit exceeds mechanism min by " + str((THETA_OFFSET - DELTA_2 - TILT_MIN)*180/np.pi) + "°"
    print("=" * 60)
    print("3D Lidar Scanner Point Density Simulator")
    print("=" * 60)
    print(f"Scanner Parameters:")
    print(f"  Target velocity: {V_TARGET} rad/s")
    print(f"  Pattern Pan Max limit: {(DELTA_1+PHI_OFFSET)*180/np.pi:.1f}°")
    print(f"  Pattern Pan Min limit: {(-DELTA_1+PHI_OFFSET)*180/np.pi:.1f}°")
    print(f"  Pattern Tilt Max limit: {(DELTA_2+THETA_OFFSET)*180/np.pi:.1f}°")
    print(f"  Pattern Tilt Min limit: {(-DELTA_2+THETA_OFFSET)*180/np.pi:.1f}°")
    print(f"  Lidar FOV: {LIDAR_FOV*180/np.pi:.0f}°")
    print(f"  Lidar points: {LIDAR_POINTS}")
    print(f"  Scan rate: {SCAN_RATE} Hz")
    print(f"  Focal point: Pan={PHI_OFFSET*180/np.pi:.1f}°, Tilt={THETA_OFFSET*180/np.pi:.1f}°")
    print("=" * 60)
    
    # Test parameters
    SCAN_DURATION = 10.0  # seconds
    GRID_RESOLUTION = 80  # Higher = more detailed but slower
    
    # 1. Visualize trajectory
    print("\n1. Generating trajectory visualization...")
    fig_traj = visualize_trajectory(SCAN_DURATION)
    fig_traj2d = plot_trajectory_2d(SCAN_DURATION)
    
    # 2. Calculate density at a specific point (example)
    print("\n2. Calculating density at specific point (0°, 0°)...")
    phi_test = 0.0  # degrees
    theta_test = 0.0  # degrees
    density, n_visible = calculate_point_density(
        phi_test * np.pi/180, 
        theta_test * np.pi/180, 
        SCAN_DURATION
    )
    print(f"   Point (pan={phi_test}°, tilt={theta_test}°):")
    print(f"   Visible {n_visible} times")
    print(f"   Density: {density:.2f} points/m²")
    
    # 3. Calculate full density grid
    print(f"\n3. Calculating full density grid...")
    phi_grid, theta_grid, density_grid = calculate_density_grid(
        SCAN_DURATION, 
        resolution=GRID_RESOLUTION
    )
    
    print(f"\n   Statistics:")
    print(f"   Max density: {np.max(density_grid):.2f} points/m²")
    print(f"   Min density: {np.min(density_grid):.2f} points/m²")
    print(f"   Mean density: {np.mean(density_grid):.2f} points/m²")
    print(f"   Std density: {np.std(density_grid):.2f} points/m²")
    
    # 4. Create visualizations
    print("\n4. Creating visualizations...")
    fig_heatmap = visualize_density_heatmap(phi_grid, theta_grid, density_grid, SCAN_DURATION)
    fig_3d = visualize_density_3d_sphere(phi_grid, theta_grid, density_grid, SCAN_DURATION)
    
    print("\n" + "=" * 60)
    print("Visualization complete! Close figures to exit.")
    print("=" * 60)
    
    plt.show()