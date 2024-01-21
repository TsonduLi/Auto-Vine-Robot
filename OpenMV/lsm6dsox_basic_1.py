import time
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI

# Initialize the sensor
lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))

# Constants
GRAVITY = 9.800  # Earth's gravity in m/s^2

# Threshold values
ACCEL_THRESHOLD = 0.07  # 0.07 g for accelerometer
GYRO_THRESHOLD = 0.4    # 0.4 degrees per second for gyroscope

def apply_thresholds(accel_data, gyro_data):
    ax, ay, az = accel_data
    gx, gy, gz = gyro_data

    # Apply thresholds to accelerometer data
    ax = ax if abs(ax) > ACCEL_THRESHOLD else 0
    ay = ay if abs(ay) > ACCEL_THRESHOLD else 0
    az = az if abs(az - 1) > ACCEL_THRESHOLD else 1  # Assuming 1g in the Z-axis when stationary

    # Apply thresholds to gyroscope data
    gx = gx if abs(gx) > GYRO_THRESHOLD else 0
    gy = gy if abs(gy) > GYRO_THRESHOLD else 0
    gz = gz if abs(gz) > GYRO_THRESHOLD else 0

    return (ax, ay, az), (gx, gy, gz)


while True:
    # Read raw data from the sensor
    raw_accel_data = lsm.accel()
    raw_gyro_data = lsm.gyro()

    # Apply thresholds
    filtered_accel_data, filtered_gyro_data = apply_thresholds(raw_accel_data, raw_gyro_data)

    # Convert filtered accelerometer data to m/s^2
    filtered_accel_data_ms2 = tuple(value * GRAVITY for value in filtered_accel_data)

    # Print filtered data with six decimal places
    print("Accelerometer (m/s^2, filtered): x:{:>12.6f} y:{:>12.6f} z:{:>12.6f}".format(*filtered_accel_data_ms2))
    print("Gyroscope (dps, filtered):        x:{:>12.6f} y:{:>12.6f} z:{:>12.6f}".format(*filtered_gyro_data))
    print("")

    time.sleep_ms(500)
