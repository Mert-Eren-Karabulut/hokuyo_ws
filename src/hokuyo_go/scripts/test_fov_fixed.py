import numpy as np
import matplotlib.pyplot as plt

# Test the FIXED FOV logic
def rotation_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def rotation_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def spherical_to_cartesian(phi, theta):
    x = np.cos(theta) * np.cos(phi)
    y = np.cos(theta) * np.sin(phi)
    z = np.sin(theta)
    return np.array([x, y, z])

def is_point_in_fov_fixed(point_query, pan_scanner, tilt_scanner):
    """Fixed implementation"""
    LIDAR_FOV = 240.0 * np.pi / 180.0
    
    R_z_inv = rotation_z(-pan_scanner)
    R_y_inv = rotation_y(-tilt_scanner)
    point_lidar = R_y_inv @ R_z_inv @ point_query
    
    x_lidar = point_lidar[0]
    y_lidar = point_lidar[1]
    z_lidar = point_lidar[2]
    
    # Calculate angle in the XY scanning plane
    alpha = np.arctan2(y_lidar, x_lidar)
    
    # Check if within ±120° FOV in the horizontal plane
    fov_limit = LIDAR_FOV / 2.0
    in_horizontal_fov = np.abs(alpha) <= fov_limit
    
    # Check if point is roughly in the scanning plane
    elevation_in_lidar_frame = np.arctan2(z_lidar, np.sqrt(x_lidar**2 + y_lidar**2))
    max_elevation_error = 2.0 * np.pi / 180.0  # ±2° tolerance
    in_scanning_plane = np.abs(elevation_in_lidar_frame) <= max_elevation_error
    
    return in_horizontal_fov and in_scanning_plane

# Test multiple scanner poses
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
test_cases = [
    (0.0, 0.0, "Pan=0°, Tilt=0°"),
    (0.0, 30.0 * np.pi/180, "Pan=0°, Tilt=30°"),
    (45.0 * np.pi/180, 0.0, "Pan=45°, Tilt=0°"),
    (45.0 * np.pi/180, 30.0 * np.pi/180, "Pan=45°, Tilt=30°"),
]

resolution = 100
phi_range = np.linspace(-np.pi, np.pi, resolution)
theta_range = np.linspace(-np.pi/2, np.pi/2, resolution)

for idx, (pan, tilt, title) in enumerate(test_cases):
    ax = axes[idx // 2, idx % 2]
    
    visibility_map = np.zeros((resolution, resolution))
    
    for i, theta in enumerate(theta_range):
        for j, phi in enumerate(phi_range):
            point = spherical_to_cartesian(phi, theta)
            if is_point_in_fov_fixed(point, pan, tilt):
                visibility_map[i, j] = 1
    
    im = ax.imshow(visibility_map, extent=[-180, 180, -90, 90], 
                   origin='lower', cmap='RdYlGn', aspect='auto')
    ax.set_xlabel('Pan Angle (degrees)', fontsize=10)
    ax.set_ylabel('Tilt Angle (degrees)', fontsize=10)
    ax.set_title(f'Fixed FOV: {title}\n(Should show horizontal band)', fontsize=11)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/tmp/fov_test_fixed.png', dpi=150)
print("Fixed FOV test saved to /tmp/fov_test_fixed.png")
print("You should now see horizontal bands (not squares) in the visualizations!")
plt.show()
