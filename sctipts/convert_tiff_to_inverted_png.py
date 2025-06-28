import cv2

# Load the TIFF image
img = cv2.imread('../maps/mapa100-binary_raster_res0.0744m_org-start600x320pix.tiff')

# Check if image is loaded correctly
if img is None:
    raise FileNotFoundError("Input file not found or the path is incorrect.")

# Convert the image to grayscale
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply binary thresholding
_, bw_img = cv2.threshold(gray_img, 240, 255, cv2.THRESH_BINARY)

# Invert the colors (black to white, white to black)
inverted_img = cv2.bitwise_not(bw_img)

# Save the inverted binary image as PNG
cv2.imwrite("../maps/mapa100-binary_raster_res0.0744m_org-start600x320pix.png", inverted_img)
