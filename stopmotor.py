import warnings
from adafruit_servokit import ServoKit

# Ignore specific RuntimeWarnings about I2C frequency
warnings.filterwarnings("ignore", category=RuntimeWarning)

# Initialize a ServoKit instance for the PCA9685
kit = ServoKit(channels=16)  # Assuming you are using a 16-channel driver

# Define the channels that the servos are connected to
servo_channels = [9,14,13,12,11,10]  # Example channels, replace with your actual channels

# Function to stop all servos by setting their PWM to 0
def stop_all_servos(channels):
    for channel in channels:
        kit.servo[channel].fraction = None  # This disables the PWM signal

# Stop all the servos
stop_all_servos(servo_channels)

print("All servo signals have been disabled.")
