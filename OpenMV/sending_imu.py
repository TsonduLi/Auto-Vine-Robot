import time
import network
import urequests
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI

# Network setup
WIFI_SSID = 'YourWiFiSSID'
WIFI_PASSWORD = 'YourWiFiPassword'
HTTP_SERVER = 'YourHttpServerAddress'

# Connect to Wi-Fi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

# Wait for connection
while not wlan.isconnected():
    pass

print("Connected to WiFi")

# Initialize the LSM6DSOX sensor
lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))

while True:
    accel_data = lsm.accel()
    gyro_data = lsm.gyro()

    # Create a payload string
    payload = "Accel: x:{:.3f}, y:{:.3f}, z:{:.3f}; Gyro: x:{:.3f}, y:{:.3f}, z:{:.3f}".format(*accel_data, *gyro_data)

    # Send data via HTTP POST request
    try:
        response = urequests.post(HTTP_SERVER, data=payload)
        if response.status_code == 200:
            print("Data sent successfully")
        else:
            print("Failed to send data, status code:", response.status_code)
        response.close()
    except Exception as e:
        print("Error:", e)

    time.sleep_ms(100)
