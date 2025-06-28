import numpy as np
import cv2
import math
import os

# ----- Configuration Parameters -----
# Input binary floor plan image
input_image_path = "../maps/floor_plans/maze_01_binary.png"

# Output filenames and location
output_filename_base = "maze_01_binary"
output_directory = "../maps/"

# Real-world width of the map in meters (adjustable)
map_width_meters = 40.0
map_resolution = 0.05  # meters per pixel (standard ROS occupancy grid resolution)

# Map origin coordinates for ROS (adjustable)
origin_x = -1.0
origin_y = -1.0
origin_theta = 0.0

# ----- Image Loading -----
original_image = cv2.imread(input_image_path)

if original_image is None:
    raise FileNotFoundError(f"Cannot load image: {input_image_path}")

original_height, original_width, _ = original_image.shape

# ----- Calculate Scaling Factors -----
scale_x = (map_width_meters / (original_width * map_resolution))
scale_y = scale_x  # Maintain aspect ratio based on width

# Resize image with calculated scaling factors
resized_image = cv2.resize(
    original_image, None, fx=scale_x, fy=scale_y, interpolation=cv2.INTER_CUBIC
)

# ----- Convert Image to Grayscale -----
gray_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

# ----- Save as .pgm (ROS-compatible format) -----
output_map_path = os.path.join(output_directory, f"{output_filename_base}.pgm")
cv2.imwrite(output_map_path, gray_image)

# ----- Generate ROS YAML Configuration File -----
output_yaml_path = os.path.join(output_directory, f"{output_filename_base}.yaml")

yaml_content = f"""image: {output_filename_base}.pgm
resolution: {map_resolution}
origin: [{origin_x}, {origin_y}, {origin_theta}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

with open(output_yaml_path, "w") as yaml_file:
    yaml_file.write(yaml_content)

print(f"Map and YAML files saved successfully:\n- {output_map_path}\n- {output_yaml_path}")
