import warnings
import time
from adafruit_servokit import ServoKit
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import os
from PIL import Image

# Ignore specific RuntimeWarnings about I2C frequency
warnings.filterwarnings("ignore", category=RuntimeWarning)

# Initialize a ServoKit instance for the PCA9685
kit = ServoKit(channels=16)

# Function to maintain servo position
def maintain_position(channels, angles, duration):
    # Set each servo to the specified angle
    for channel, angle in zip(channels, angles):
        kit.servo[channel].angle = angle
    # Maintain position for the specified duration
    time.sleep(duration)

# Function to capture image and detect features
def capture_and_process_image():
    # Initialize the camera
    camera = PiCamera()
    camera.resolution = (640, 480)
    raw_capture = PiRGBArray(camera)

    # Capture the image from the camera
    camera.start_preview()
    time.sleep(2)  # Allow camera to warm up
    camera.capture(raw_capture, format='bgr')
    camera.stop_preview()
    image = raw_capture.array
    image = cv2.rotate(image, cv2.ROTATE_180)
    # Save the image to the Desktop in the 'pics' folder
    desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
    pics_folder = os.path.join(desktop, 'pics')
    if not os.path.exists(pics_folder):
        os.makedirs(pics_folder)
    image_path = os.path.join(pics_folder, 'ping_pong_ball.jpg')
    cv2.imwrite(image_path, image)

    # Detect circles and color
    detected, circle_center, circle_radius = detect_circle(image)
    if detected:
        color = detect_color(image, circle_center, circle_radius)
        print(f"The detected circle is {color}.")
        cv2.imwrite(os.path.join(pics_folder, 'detected_circle.jpg'), image)
    else:
        print("No circle detected.")

def detect_circle(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            cv2.circle(image, center, radius, (0, 255, 0), 2)
            cv2.circle(image, center, 2, (0, 0, 255), 3)
            return True, center, radius
    return False, None, None

def detect_color(image, center, radius):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros_like(hsv[:, :, 0])
    cv2.circle(mask, center, radius, (255, 255, 255), -1)
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

# Zero state positioning
servo_channels = [9,11, 12, 13, 14, 10]
servo_angles = [20,10, 20, 170, 120, 80]  # Angles for zero state
maintain_position(servo_channels, servo_angles, 5)  # Hold positions for 2 seconds

# Set specific servo to 90 degrees for the operation
kit.servo[11].angle = 90  # Adjust servo at channel 11
time.sleep(5)  # Hold this position

# Capture and process image
capture_and_process_image()

print("Operation completed successfully.")
