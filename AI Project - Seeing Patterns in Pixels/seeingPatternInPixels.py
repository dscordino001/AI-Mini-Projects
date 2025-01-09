# Dominic Scordino
# 11/15/2021
# SFE 340 AI & ML
# Assignment 9
# This program uses advanced image processing techniques to detect fish shapes in images.

# Note: This code is based on the OpenCV library, which is a powerful library for image processing.
# I also could not get the code to fully remove all spots in fish 3, but I got close :)

import cv2 # run <pip install opencv-python> to get this module before running the code
import numpy as np
from matplotlib import pyplot as plt

# Constants
KERNEL_SIZE = 5
THRESHOLD_VALUE = 127
MAX_BINARY_VALUE = 255
CANNY_THRESHOLD1 = 50
CANNY_THRESHOLD2 = 150
MIN_CONTOUR_AREA = 40
ASPECT_RATIO_MIN = 1.5
ASPECT_RATIO_MAX = 5.0
CONTOUR_THICKNESS = 2


def find_all_fish_shapes(image_path):
    # Load image as grayscale
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # Step 1: Apply Median Blurring for noise reduction
    blurred_image = cv2.medianBlur(image, KERNEL_SIZE)

    # Step 2: Thresholding to create a binary image
    _, binary_image = cv2.threshold(blurred_image, THRESHOLD_VALUE, MAX_BINARY_VALUE, cv2.THRESH_BINARY_INV)

    # Step 3: Denoise using Morphological Opening
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)) # Elliptical kernel for better noise removal
    denoised_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel) # Erosion followed by dilation

    # Step 4: Edge detection using Canny
    edges = cv2.Canny(denoised_image, CANNY_THRESHOLD1, CANNY_THRESHOLD2) # Canny edge detection

    # Step 5: Contour detection
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # External contours only

    # Step 6: Shape matching using Hu Moments
    fish_shapes = []
    for contour in contours:
        # Filter small or irrelevant contours by area
        if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
            continue

        # Calculate bounding box and aspect ratio
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h

        # Heuristic for fish shapes: Aspect ratio and area
        if ASPECT_RATIO_MIN < aspect_ratio < ASPECT_RATIO_MAX and cv2.contourArea(contour) > MIN_CONTOUR_AREA:
            fish_shapes.append(contour)

    # Step 7: Create output image with the fish shapes highlighted
    output_image = np.zeros_like(image)
    cv2.drawContours(output_image, fish_shapes, -1, 255, thickness=CONTOUR_THICKNESS) # Draw all fish contours

    return output_image, fish_shapes


# Refine the process for all five images
def process_all_images_fully(image_paths):
    outputs = []
    for path in image_paths:
        fish_output, fish_contours = find_all_fish_shapes(path)
        outputs.append((fish_output, fish_contours))
    return outputs


# Process images with the advanced method
file_paths = [
    "fish1.png",
    "fish3.png"
]
all_fish_detected_outputs = process_all_images_fully(file_paths)

# Display results for all images
plt.figure(figsize=(6, 4))
for i, (fish_output, _) in enumerate(all_fish_detected_outputs):
    plt.subplot(1, 5, i + 1)
    plt.title(f"Fish {i + 1}")
    plt.imshow(fish_output, cmap='gray')
    plt.axis("off")

plt.tight_layout()
plt.show()