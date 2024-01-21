import pyb
import time

# Initialize I2C
# Adjust the I2C bus number (2) and pins if necessary for your OpenMV model
i2c = pyb.I2C(2, pyb.I2C.MASTER, baudrate=400000)

def scan_i2c():
    print("Scanning I2C bus...")
    devices = []
    for address in range(0x08, 0x78):  # I2C addresses range from 0x08 to 0x77
        try:
            i2c.mem_read(1, address, 0x00)  # Attempt to read from the address
            print("Found I2C device at address 0x{:02X}".format(address))
            devices.append(address)
        except OSError:
            pass  # No device at this address
    return devices

# Perform the I2C scan
found_devices = scan_i2c()

if not found_devices:
    print("No I2C devices found.")
else:
    print("Found {} I2C device(s):".format(len(found_devices)))
    for device in found_devices:
        print("  - 0x{:02X}".format(device))




