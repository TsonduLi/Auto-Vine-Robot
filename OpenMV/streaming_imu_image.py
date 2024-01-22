# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE

# Combined MJPEG Streaming and IMU Data Publishing
import sensor
import time
import network
import socket
from mqtt import MQTTClient
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI, I2C
from vl53l1x import VL53L1X
import ujson


# Constants
GRAVITY = 9.800  # Earth's gravity in m/s^2
ACCEL_THRESHOLD = 0.07  # Accelerometer threshold
GYRO_THRESHOLD = 0.4    # Gyroscope threshold

# Network Credentials
SSID = "jinpin"  # Network SSID
KEY = "xijinpin"  # Network key

# MJPEG Streaming Settings
HOST = ""  # Use first available interface
PORT = 8080  # Arbitrary non-privileged port

# Initialize the sensor
lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
tof = VL53L1X(I2C(2))   # Initialize VL53L1X Time of Flight (ToF) sensor

# Init sensor
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)

# Init wlan module and connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, KEY)

# Wait for connection to the network
while not wlan.isconnected():
    print('Trying to connect to "{:s}"...'.format(SSID))
    time.sleep_ms(1000)
print('Connection Successful')

# Setup MQTT client and connect
client = MQTTClient("openmv", "test.mosquitto.org", port=1883)
client.connect()

# Create server socket for MJPEG Streaming
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
s.bind([HOST, PORT])
s.listen(5)
s.setblocking(True)

# Define a function to apply thresholds to IMU data
def apply_thresholds(accel_data, gyro_data):
    ax, ay, az = accel_data
    gx, gy, gz = gyro_data

    # Apply thresholds to accelerometer data
    ax = ax if abs(ax) > ACCEL_THRESHOLD else 0
    ay = ay if abs(ay) > ACCEL_THRESHOLD else 0
    az = az if abs(az - GRAVITY) > ACCEL_THRESHOLD else GRAVITY

    # Apply thresholds to gyroscope data
    gx = gx if abs(gx) > GYRO_THRESHOLD else 0
    gy = gy if abs(gy) > GYRO_THRESHOLD else 0
    gz = gz if abs(gz) > GYRO_THRESHOLD else 0

    return (ax, ay, az), (gx, gy, gz)

def start_streaming(s):
    print("Waiting for connections..")
    client_socket, addr = s.accept()
    client_socket.settimeout(5.0)
    print("Connected to " + addr[0] + ":" + str(addr[1]))

    # Send multipart header
    client_socket.sendall(
        "HTTP/1.1 200 OK\r\n"
        "Server: OpenMV\r\n"
        "Content-Type: multipart/x-mixed-replace;boundary=openmv\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n\r\n"
    )

    # FPS clock
    clock = time.clock()

    # Start streaming images and publishing IMU data
    while True:
        clock.tick()

        # MJPEG Streaming
        frame = sensor.snapshot()
        cframe = frame.compressed(quality=35)
        header = (
            "\r\n--openmv\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length:" + str(cframe.size()) + "\r\n\r\n"
        )
        client_socket.sendall(header)
        client_socket.sendall(cframe)

        # IMU Data Publishing
        raw_accel_data = lsm.accel()
        raw_gyro_data = lsm.gyro()
        filtered_accel_data, filtered_gyro_data = apply_thresholds(raw_accel_data, raw_gyro_data)
        filtered_accel_data_ms2 = tuple(value * GRAVITY for value in filtered_accel_data)

        imu_data_dict = {
            "accel": {
                "x": filtered_accel_data_ms2[0],
                "y": filtered_accel_data_ms2[1],
                "z": filtered_accel_data_ms2[2]
            },
            "gyro": {
                "x": filtered_gyro_data[0],
                "y": filtered_gyro_data[1],
                "z": filtered_gyro_data[2]
            }
        }

        imu_data_json = ujson.dumps(imu_data_dict)
        client.publish("openmv/imu", imu_data_json)
        print("Published:", imu_data_json)
        distance_mm = tof.read()

        # Publish Distance Data via MQTT
        distance_data_json = ujson.dumps({"distance": distance_mm})
        client.publish("openmv/distance", distance_data_json)
        print("Distance data published:", distance_data_json)


        print(clock.fps())
        time.sleep(1.0 / 10)

while True:
    try:
        start_streaming(s)
    except OSError as e:
        print("socket error: ", e)
        # Reinitialize the socket if an error occurs
        s.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
        s.bind([HOST, PORT])
        s.listen(5)
        s.setblocking(True)
