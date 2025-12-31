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
servo_channels = [9,11, 12, 13, 14, 10]
servo_angles = [20,10, 20, 170, 120, 80]  # Angles for zero state
maintain_position(servo_channels, servo_angles, 3)  # Hold positions for 2 seconds

# Set specific servo to 90 degrees for the operation
kit.servo[11].angle = 90  # Adjust servo at channel 11

kit.servo[9].angle = 70
time.sleep(2)
kit.servo[14].angle = 100
time.sleep(1)
kit.servo[13].angle = 80
time.sleep(1)
kit.servo[12].angle = 50
time.sleep(1)
kit.servo[12].angle = 70
time.sleep(1)
kit.servo[9].angle = 20
time.sleep(2)
print("Operation completed successfully.")


