import open3d as o3d
import numpy as np
import cv2
import yaml
import os
from ament_index_python.packages import get_package_share_directory

pkg_dir = "/home/stringer/b2_ws/src/b2_thesis_fusion/maps"

pcd_file = os.path.join(pkg_dir, "fastlio2_maps", "map.pcd")        
map_name = os.path.join(pkg_dir, "2d_maps", "2d_map")
resolution = 0.05  # 5 cm per pixel
z_min = 0.2        
z_max = 1.0


print(f"1. Loading {pcd_file}...")
pcd = o3d.io.read_point_cloud(pcd_file)
points = np.asarray(pcd.points)

print(f"2. Height slice (Z from {z_min}m to {z_max}m)...")
mask = (points[:, 2] > z_min) & (points[:, 2] < z_max)
points_2d = points[mask][:, :2]

print("3. Projection on 2D grid...")
min_x, max_x = np.min(points_2d[:, 0]), np.max(points_2d[:, 0])
min_y, max_y = np.min(points_2d[:, 1]), np.max(points_2d[:, 1])

width = int(np.ceil((max_x - min_x) / resolution))
height = int(np.ceil((max_y - min_y) / resolution))

# 254 - free, 0 - occupied
grid = np.full((height, width), 254, dtype=np.uint8)

idx_x = ((points_2d[:, 0] - min_x) / resolution).astype(int)
idx_y = ((points_2d[:, 1] - min_y) / resolution).astype(int)

# Solve dismatch between ROS origin (bottom-left) and open3d origin (top-left)
# Invert Y for correct display 
idx_y_img = height - 1 - idx_y

# display obstacles
grid[idx_y_img, idx_x] = 0

# Thicken walls by 1 pixel (so AMCL and laser rays can better latch onto them)
kernel = np.ones((3,3), np.uint8)
grid = cv2.erode(grid, kernel, iterations=1) 

print("4. Saving PGM and YAML...")
cv2.imwrite(f"{map_name}.pgm", grid)

yaml_data = {
    "image": f"{map_name}.pgm",
    "resolution": resolution,
    "origin": [float(min_x), float(min_y), 0.0],
    "negate": 0,
    "occupied_thresh": 0.65,
    "free_thresh": 0.196
}

with open(f"{map_name}.yaml", 'w') as f:
    yaml.dump(yaml_data, f, default_flow_style=False)

print(f"Success! Files {map_name}.pgm and {map_name}.yaml created")