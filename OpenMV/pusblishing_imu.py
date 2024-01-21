import time
import network
from mqtt import MQTTClient
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI
import ujson


# Initialize the sensor
lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))

# Constants
GRAVITY = 9.800  # Earth's gravity in m/s^2
ACCEL_THRESHOLD = 0.07  # Accelerometer threshold
GYRO_THRESHOLD = 0.4    # Gyroscope threshold

SSID = "jinpin"  # Network SSID
KEY = "xijinpin"  # Network key

# Define a function to apply thresholds to IMU data

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

# Init wlan module and connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, KEY)

# Wait for connection to the network
while not wlan.isconnected():
    print('Trying to connect to the network...')
    time.sleep_ms(1000)
print('Connection successful')

# Setup MQTT client and connect
client = MQTTClient("openmv", "test.mosquitto.org", port=1883)
client.connect()

# Main loop to read from the IMU and publish over MQTT
# Main loop to read from the IMU and publish over MQTT
while True:
    # Read raw data from the sensor
    raw_accel_data = lsm.accel()
    raw_gyro_data = lsm.gyro()

    # Apply thresholds to raw data
    filtered_accel_data, filtered_gyro_data = apply_thresholds(raw_accel_data, raw_gyro_data)

    # Convert filtered accelerometer data to m/s^2
    filtered_accel_data_ms2 = tuple(value * GRAVITY for value in filtered_accel_data)

    # Create a dictionary of the data
    imu_data_dict = {
        "accel": {
            "x": filtered_acce_data_ms2[0],
            "y": filtered_accel_data_ms2[1],
            "z": filtered_accel_data_ms2[2]
        },
        "gyro": {
            "x": filtered_gyro_data[0],
            "y": filtered_gyro_data[1],
            "z": filtered_gyro_data[2]
        }
    }

    # Convert the dictionary to a JSON string
    imu_data_json = ujson.dumps(imu_data_dict)

    # Publish the IMU data in JSON format
    client.publish("openmv/imu", imu_data_json)
    print("Published:", imu_data_json)
    time.sleep_ms(1000)

