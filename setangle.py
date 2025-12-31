from adafruit_servokit import ServoKit
import time


# Initialize a ServoKit instance for the PCA9685
kit = ServoKit(channels=16)  # Specify the number of channels on your PCA9685 board

# Function to move the servo on a specified channel to a given angle
def move_servo(channel, angle):
    kit.servo[channel].angle = angle
    print(f"Moved servo on channel {channel} to {angle} degrees")
    time.sleep(1)  # Wait for 1 second to let the servo reach the position

# Move the servo to the desired angle
move_servo(13, 75)

print("Servo movement complete and program has ended.")
