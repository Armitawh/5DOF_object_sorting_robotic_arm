import board
import busio
import adafruit_tcs34725

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create a TCS34725 sensor object
sensor = adafruit_tcs34725.TCS34725(i2c)

# Set integration time in milliseconds (e.g., 50ms)
sensor.integration_time = 50

# Calibration values for your specific setup
calibration_values = {
    "red": (100, 0, 0),
    "green": (0, 100, 0),
    "blue": (0, 0, 100),
    "yellow": (100, 100, 0),
    "white": (100, 100, 100),
}

def get_color_name(rgb):
    closest_color = None
    min_distance = float('inf')

    for color, (r_c, g_c, b_c) in calibration_values.items():
        # Calculate Euclidean distance
        distance = ((rgb[0] - r_c) ** 2 + (rgb[1] - g_c) ** 2 + (rgb[2] - b_c) ** 2) ** 0.5

        if distance < min_distance:
            min_distance = distance
            closest_color = color

    return closest_color

try:
    while True:
        # Read RGB values
        r, g, b = sensor.color_rgb_bytes

        # Print RGB values
        print(f"Red: {r}, Green: {g}, Blue: {b}")

        # Get and print detected color
        detected_color = get_color_name((r, g, b))
        print(f"Detected Color: {detected_color}")

except KeyboardInterrupt:
    pass
