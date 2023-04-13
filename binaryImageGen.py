import matplotlib.pyplot as plt
import numpy as np
import cv2

# Read the image
image = cv2.imread("map1.jpg")

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply thresholding to convert to binary image
_, binary_image = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Convert the binary image to a numpy array
binary_array = np.array(binary_image)

# Save the numpy array to .npy file
np.save("map1.npy", binary_array)

# Load the .npy image
image_array = np.load("map1.npy")

# Display the image using matplotlib
plt.imshow(image_array, cmap='gray')
plt.axis('off')
plt.show()
