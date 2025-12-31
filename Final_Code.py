import warnings
import time
import board
import busio
from adafruit_servokit import ServoKit
import adafruit_tcs34725
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import os
from PIL import Image
from collections import Counter

warnings.filterwarnings("ignore", category=RuntimeWarning)

image_counter = 0 

i2c = busio.I2C(board.SCL, board.SDA)
kit = ServoKit(channels=16, i2c=i2c)
sensor = adafruit_tcs34725.TCS34725(i2c)

def stop_all_servos(channels):
    for channel in channels:
        kit.servo[channel].fraction = None 

def maintain_position(channels, angles, duration):
    for channel, angle in zip(channels, angles):
        kit.servo[channel].angle = angle
    time.sleep(duration)

def smooth_move(servo, start_angle, end_angle, step=1, delay=0.01):
    if start_angle is None or end_angle is None:
        return 
    if start_angle < end_angle:
        angle = start_angle
        while angle <= end_angle:
            kit.servo[servo].angle = angle
            time.sleep(delay)
            angle += step
    else:
        angle = start_angle
        while angle >= end_angle:
            kit.servo[servo].angle = angle
            time.sleep(delay)
            angle -= step

servo_channels = [9, 11, 12, 13, 14, 10]
servo_angles = [20, 125, 20, 170, 120, 80] 

def detect_circle(image, min_radius_threshold=20, max_radius_threshold=100, save_path=None):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if save_path:
        cv2.imwrite(os.path.join(save_path, 'step1_grayscale.jpg'), gray)

    gray_blurred = cv2.medianBlur(gray, 5)
    if save_path:
        cv2.imwrite(os.path.join(save_path, 'step2_median_blur.jpg'), gray_blurred)

    circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    if save_path and circles is not None:
        circles_img = image.copy()
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            if radius < min_radius_threshold or radius > max_radius_threshold:
                continue
            cv2.circle(circles_img, center, radius, (0, 255, 0), 2)
            cv2.circle(circles_img, center, 2, (0, 0, 255), 3)
        cv2.imwrite(os.path.join(save_path, 'step3_hough_circles.jpg'), circles_img)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]
            if radius < min_radius_threshold or radius > max_radius_threshold:
                print("No Ball Detected: Detected circle is out of the specified size range.")
                continue
            cv2.circle(image, center, radius, (0, 255, 0), 2)
            cv2.circle(image, center, 2, (0, 0, 255), 3)
            return True, center, radius
    return False, None, None

def detect_color(image, center, radius, save_path):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imwrite(os.path.join(save_path, 'step4_hsv.jpg'), hsv)
    mask = np.zeros_like(hsv[:, :, 0])
    cv2.circle(mask, center, radius, (255, 255, 255), -1)
    cv2.imwrite(os.path.join(save_path, 'step5_mask.jpg'), mask)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    red_mask = cv2.bitwise_and(red_mask, red_mask, mask=mask)
    blue_mask = cv2.bitwise_and(blue_mask, blue_mask, mask=mask)
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    cv2.imwrite(os.path.join(save_path, 'step6_red_mask.jpg'), red_mask)
    cv2.imwrite(os.path.join(save_path, 'step6_blue_mask.jpg'), blue_mask)
    red_count = np.count_nonzero(red_mask)
    blue_count = np.count_nonzero(blue_mask)
    if red_count > blue_count:
        return "Red"
    else:
        return "Blue"

def detect_color_with_sensor():
    color_counts = Counter()
    for _ in range(5):
        r, g, b = sensor.color_rgb_bytes 
        if r > g and r > b:
            color_counts['Red'] += 1
        elif b > r and b > g:
            color_counts['Blue'] += 1
        else:
            color_counts['Unknown'] += 1
        time.sleep(0.1) 
    return color_counts.most_common(1)[0][0]

camera = PiCamera()
camera.resolution = (640, 480)

desktop = os.path.join(os.path.expanduser('~'), 'Desktop')
pics_folder = os.path.join(desktop, 'pics')
if not os.path.exists(pics_folder):
    os.makedirs(pics_folder)

def inverse_kinematics(px, py, pz, a1=9, a2=10, a3=10):
    theta1 = np.arctan2(py, px)
    r = np.sqrt(px**2 + py**2)
    s = pz - a1
    d = np.sqrt(r**2 + s**2)
    cos_theta3 = (d**2 - a2**2 - a3**2) / (2 * a2 * a3)
    cos_theta3 = np.clip(cos_theta3, -1, 1) 
    theta3 = np.arccos(cos_theta3)
    k1 = a2 + a3 * cos_theta3
    k2 = a3 * np.sin(theta3)
    theta2 = np.arctan2(s, r) - np.arctan2(k2, k1)
    theta1_deg = np.degrees(theta1)
    theta2_deg = np.degrees(theta2)
    theta3_deg = np.degrees(theta3)
    if theta2_deg < 0:
        theta2_deg += 180
    return np.array([theta1_deg, theta2_deg, theta3_deg])

def calculate_diameter_in_cm(radius_pixels, pixel_to_cm_ratio):
    diameter_pixels = 2 * radius_pixels
    diameter_cm = diameter_pixels * pixel_to_cm_ratio
    return diameter_cm

try:
    while True:
        maintain_position(servo_channels, servo_angles, 3)
        kit.servo[11].angle = 125  
        while True:
            raw_capture = PiRGBArray(camera) 
            camera.start_preview()
            time.sleep(2) 
            camera.capture(raw_capture, format='bgr')
            image = raw_capture.array
            camera.stop_preview()
            image_counter += 1
            image_path = os.path.join(pics_folder, f'captured_{image_counter}.jpg')
            cv2.imwrite(image_path, image)
            height, width, _ = image.shape
            middle_x = width // 2
            cv2.line(image, (middle_x, 0), (middle_x, height), (255, 0, 0), 2)
            min_radius_threshold = 20  
            max_radius_threshold = 100
            detected, circle_center, circle_radius = detect_circle(image, min_radius_threshold, max_radius_threshold, pics_folder)
            if detected:
                while True:
                    ball_x = circle_center[0]
                    if kit.servo[11].angle is None:
                        kit.servo[11].angle = 90 
                    if ball_x < middle_x - 10: 
                        kit.servo[11].angle -= 5  
                        time.sleep(0.2)  
                    elif ball_x > middle_x + 10:
                        kit.servo[11].angle += 5  
                        time.sleep(0.2)  
                    else:
                        break  
                    raw_capture = PiRGBArray(camera) 
                    camera.capture(raw_capture, format='bgr')
                    image = raw_capture.array
                    cv2.line(image, (middle_x, 0), (middle_x, height), (255, 0, 0), 2)
                    detected, circle_center, circle_radius = detect_circle(image, min_radius_threshold, max_radius_threshold, pics_folder)
                    if detected:
                        cv2.circle(image, circle_center, circle_radius, (0, 255, 0), 2)
                        cv2.circle(image, circle_center, 2, (0, 0, 255), 3)
                    image_counter += 1
                    image_path = os.path.join(pics_folder, f'captured_{image_counter}.jpg')
                    cv2.imwrite(image_path, image)
                    if not detected:
                        break
                break
            else:
                print("No ball detected, retrying in 5 seconds...")
                time.sleep(5)
        pixel_to_cm_ratio = 6 / (2 * circle_radius) 
        diameter_cm = calculate_diameter_in_cm(circle_radius, pixel_to_cm_ratio)
        print(f"The diameter of the ball is: {diameter_cm:.2f} cm")
        camera_detected_color = detect_color(image, circle_center, circle_radius, pics_folder)
        print(f"Camera detected color: {camera_detected_color}")
        print("Ball detected")
        cv2.imwrite(os.path.join(pics_folder, 'detected_circle.jpg'), image)
        sensor_detected_color = detect_color_with_sensor()
        print(f"Sensor detected color: {sensor_detected_color}")
        if camera_detected_color == sensor_detected_color:
            print(f"The output of the sensor and camera match. The color of the ball is {camera_detected_color}.")
            color = camera_detected_color
        else:
            print("The output of the sensor and camera do not match.")
            print(f"Camera: {camera_detected_color}, Sensor: {sensor_detected_color}")
            color = "Unknown"
        target_px = 0
        target_py = 12
        target_pz = 1
        thetas = inverse_kinematics(target_px, target_py, target_pz)
        print(f"Inverse Kinematics Angles: Theta1={thetas[0]:.2f} degrees, Theta2={thetas[1]:.2f} degrees, Theta3={thetas[2]:.2f} degrees")
        smooth_move(9, kit.servo[9].angle, 50)
        time.sleep(1)
        smooth_move(14, kit.servo[14].angle, thetas[2])
        time.sleep(1)
        smooth_move(13, kit.servo[13].angle, thetas[1])
        time.sleep(1)
        smooth_move(12, kit.servo[12].angle, thetas[0])
        time.sleep(1)
        smooth_move(9, 50, 20)
        time.sleep(0.5)
        stop_all_servos(servo_channels)
        time.sleep(0.5)
        kit.servo[9].angle = 20
        time.sleep(0.5)
        kit.servo[12].angle = 40
        time.sleep(0.5)
        kit.servo[12].angle = 20
        time.sleep(0.5)
        kit.servo[13].angle = 170
        time.sleep(0.5)
        kit.servo[14].angle = 120
        time.sleep(0.5)
        kit.servo[10].angle = 80
        time.sleep(3)
        if color == "Red":
            kit.servo[11].angle = 20
            smooth_move(14, kit.servo[14].angle, 100)
            time.sleep(1)
            smooth_move(13, kit.servo[13].angle, 80)
            time.sleep(1)
            smooth_move(12, kit.servo[12].angle, 30)
            time.sleep(1)
            kit.servo[9].angle = 70
            time.sleep(0.5)
        elif color == "Blue":
            kit.servo[11].angle = 180
            smooth_move(14, kit.servo[14].angle, 100)
            time.sleep(1)
            smooth_move(13, kit.servo[13].angle, 80)
            time.sleep(1)
            smooth_move(12, kit.servo[12].angle, 30)
            time.sleep(1)
            kit.servo[9].angle = 70
            time.sleep(1)
        smooth_move(14, kit.servo[14].angle, 120)
        smooth_move(13, kit.servo[13].angle, 170)
        smooth_move(12, kit.servo[12].angle, 20)
        smooth_move(11, kit.servo[11].angle, 125)
        smooth_move(9, kit.servo[9].angle, 20)
        maintain_position(servo_channels, servo_angles, 1)
        kit.servo[10].angle = 0
        time.sleep(1)
        kit.servo[10].angle = 180
        time.sleep(1)
        kit.servo[10].angle = 0
        time.sleep(1)
        kit.servo[10].angle = 80
        time.sleep(1)
        print("Operation completed successfully. Restarting...")
finally:
    camera.close()  
