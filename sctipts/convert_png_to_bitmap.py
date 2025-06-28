import numpy as np
import cv2
import math
import os

# ----- Configuration Parameters -----
# Input binary floor plan image
input_image_path = "../maps/mapa100-binary_raster_res0.0744m_org-start600x320pix_inv.png"

# Output filenames and location
output_filename_base = "mapa100-binary_raster_res0.0744m_org-start600x320pix_inv"
output_directory = "../maps/"

# Map resolution in meters/pixel (from filename or metadata)
map_resolution = 0.0744

# Robot's start pixel (this will be aligned to world origin [0, 0])
start_pixel_x = 600
start_pixel_y = 320

# ----- Image Loading -----
original_image = cv2.imread(input_image_path)

if original_image is None or original_image.size == 0:
    raise FileNotFoundError(f"Cannot load image: {input_image_path}")

original_height, original_width, _ = original_image.shape

# ----- Compute Map Size in Meters -----
map_width_meters = original_width * map_resolution
map_height_meters = original_height * map_resolution

print(f"Map size: {original_width}x{original_height} px → {map_width_meters:.2f}m x {map_height_meters:.2f}m")

# ----- Compute Map Origin -----
# This shifts the (600, 320) pixel to (0, 0) in world coordinates
origin_x = -start_pixel_x * map_resolution
origin_y = -start_pixel_y * map_resolution
origin_theta = 0.0

# ----- Resize (if needed for ROS or visualization, otherwise skip) -----
# Keeping 1.0 scaling means no resize here; adjust if needed
scale_x = 1.0
scale_y = 1.0

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
