import warnings
import time
from adafruit_servokit import ServoKit

# Ignore specific RuntimeWarnings about I2C frequency
warnings.filterwarnings("ignore", category=RuntimeWarning)

# Initialize a ServoKit instance for the PCA9685
kit = ServoKit(channels=16)  # Assuming you are using a 16-channel driver

# Define the channels that the servos are connected to
servo_channels = [9,11,12,13,14,10]  # Replace with the channels you're using

# Define the target angles for each servo (converted from PWM values if necessary)
servo_angles = [20,10, 20, 170, 120, 80]  # These values should be in degrees

# Function to maintain servo position
def maintain_position(channels, angles, duration):
    # Set each servo to the specified angle
    for channel, angle in zip(channels, angles):
        kit.servo[channel].angle = angle

    # Maintain position for the specified duration
    time.sleep(duration)

    # Optionally adjust this to reset servos or turn them off as needed
    for channel in channels:
        kit.servo[channel].angle = None  # Disables the PWM signal

# Set and maintain each servo position for 5 seconds
maintain_position(servo_channels, servo_angles, 5)

print("All servo signals have been maintained, then disabled.")