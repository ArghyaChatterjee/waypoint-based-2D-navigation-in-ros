import cv2

# Load the TIFF image
img = cv2.imread('../maps/mapa100-binary_raster_res0.0744m_org-start600x320pix.tiff')

# Check if image is loaded correctly
if img is None:
    raise FileNotFoundError("Input file not found or the path is incorrect.")

# Convert the image to grayscale
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply binary thresholding
# Adjust threshold value (e.g., 240) as needed
_, bw_img = cv2.threshold(gray_img, 240, 255, cv2.THRESH_BINARY)

# Save the binary image as PNG
cv2.imwrite("../maps/mapa100-binary_raster_res0.0744m_org-start600x320pix.png", bw_img)
