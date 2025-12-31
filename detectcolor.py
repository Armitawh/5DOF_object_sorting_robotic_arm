import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import os
from PIL import Image
import time

# Initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
raw_capture = PiRGBArray(camera)

# Function to detect circle
def detect_circle(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.medianBlur(gray, 5)

    # Perform Hough Circle Transform
    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            # Draw the circumference of the circle
            cv2.circle(image, center, radius, (0, 255, 0), 2)
            # Draw a small circle to mark the center
            cv2.circle(image, center, 2, (0, 0, 255), 3)
            return True, center, radius
    return False, None, None

# Function to detect color
def detect_color(image, center, radius):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros_like(hsv[:, :, 0])
    cv2.circle(mask, center, radius, (255, 255, 255), -1)

    # Define color ranges in HSV
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])

    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    mask_red = cv2.bitwise_and(red_mask, red_mask, mask=mask)
    mask_blue = cv2.bitwise_and(blue_mask, blue_mask, mask=mask)

    red_count = np.count_nonzero(mask_red)
    blue_count = np.count_nonzero(mask_blue)

    if red_count > blue_count:
        return "Red"
    else:
        return "Blue"

# Capture the image from the camera
camera.start_preview()
time.sleep(2) # Allow camera to warm up
camera.capture(raw_capture, format='bgr')
image = raw_capture.array
camera.stop_preview()

# Save the image to the Desktop in the 'pics' folder
desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
pics_folder = os.path.join(desktop, 'pics')
if not os.path.exists(pics_folder):
    os.makedirs(pics_folder)
image_path = os.path.join(pics_folder, 'ping_pong_ball.jpg')
cv2.imwrite(image_path, image)

# Detect the circle
detected, circle_center, circle_radius = detect_circle(image)

if detected:
    color = detect_color(image, circle_center, circle_radius)
    print(f"The detected circle is {color}.")
    # Optional: save the image with the detected circle
    cv2.imwrite(os.path.join(pics_folder, 'detected_circle.jpg'), image)
else:
    print("No circle detected.")

# If you want to display the image with the detected circle uncomment the following lines
# cv2.imshow('Detected Circle', image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
