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

# Initial servo positions
servo_channels = [9, 11, 12, 13, 14, 10]
servo_angles = [20, 10, 20, 170, 120, 80]  # Angles for zero state
maintain_position(servo_channels, servo_angles, 3)  # Hold positions for 3 seconds

# Set specific servo to 90 degrees for the operation
kit.servo[11].angle = 90  # Adjust servo at channel 11

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

# Function to move servo smoothly to target angle
def smooth_move(servo, start_angle, end_angle, step=1, delay=0.01):
    start_angle = int(round(start_angle))
    end_angle = int(round(end_angle))
    if start_angle < end_angle:
        for angle in range(start_angle, end_angle + 1, step):
            kit.servo[servo].angle = angle
            time.sleep(delay)
    else:
        for angle in range(start_angle, end_angle - 1, -step):
            kit.servo[servo].angle = angle
            time.sleep(delay)

# Capture the image from the camera
camera.start_preview()
time.sleep(2)  # Allow camera to warm up
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
    
    # If the detected color is red, perform the movements
    if color == "Red":
        # Smoother movements
        smooth_move(9, kit.servo[9].angle, 70)
        time.sleep(1)
        smooth_move(14, kit.servo[14].angle, 100)
        time.sleep(1)
        smooth_move(13, kit.servo[13].angle, 80)
        time.sleep(1)
        smooth_move(12, kit.servo[12].angle, 50)
        time.sleep(1)
        smooth_move(12, 50, 70)
        time.sleep(1)
        smooth_move(9, 70, 20)
        time.sleep(2)
else:
    print("No circle detected.")

print("Operation completed successfully.")
