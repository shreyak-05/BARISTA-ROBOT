import cv2
import numpy as np

def detect_cups(image):
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Blur the image to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)
    cv2.imwrite("edges_image.png", edges)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Number of contours detected: {len(contours)}")

    # Filter contours based on their area and shape
    for contour in contours:
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)

        # Adjust thresholds if necessary
        if area > 200 and 0.3 < (4 * np.pi * area) / (perimeter ** 2) < 2.0:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return image

# Read the image
image_path = '/home/manas/Pictures/cup_image.png'
input_image = cv2.imread(image_path)

if input_image is None:
    print(f"Error: Unable to load image from {image_path}")
    exit()

# Detect cups in the image
result = detect_cups(input_image)

# Save the result
cv2.imwrite("output_image.png", result)
print("Output saved to output_image.png")

# import cv2
# import numpy as np

# def detect_cups(image):
#     """
#     Detect cups in the given image using contour detection.
#     """
#     # Step 1: Convert the image to grayscale
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     # Step 2: Apply Gaussian Blur to reduce noise
#     blurred = cv2.GaussianBlur(gray, (5, 5), 0)

#     # Step 3: Detect edges using the Canny Edge Detector
#     edges = cv2.Canny(blurred, 50, 150)

#     # Step 4: Find contours from the edges
#     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     # Step 5: Iterate through contours and filter based on size and circularity
#     for contour in contours:
#         # Calculate area and perimeter
#         area = cv2.contourArea(contour)
#         perimeter = cv2.arcLength(contour, True)

#         # Skip small contours to remove noise
#         if area < 1000:
#             continue

#         # Circularity = (4 * π * Area) / (Perimeter²)
#         circularity = (4 * np.pi * area) / (perimeter ** 2)
        
#         # Check for circularity (cups are roughly circular at the top view)
#         if 0.7 < circularity < 1.3:  # Adjust range as necessary
#             # Draw the contour
#             cv2.drawContours(image, [contour], -1, (0, 0, 255), 3)

#     return image

# # Read the input image
# input_image = cv2.imread('/home/manas/Pictures/cup_image.png')

# # Check if the image was loaded
# if input_image is None:
#     print("Error: Image not found or could not be read.")
# else:
#     # Detect cups in the image
#     result_image = detect_cups(input_image)

#     # Display the resulting image with detected cups
#     cv2.imshow("Cup Detection", result_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
