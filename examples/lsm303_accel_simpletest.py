# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Display accelerometer data once per second"""

import time

import board

import adafruit_lsm303_accel

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)

while True:
    acc_x, acc_y, acc_z = sensor.acceleration

    print(f"Acceleration (m/s^2): ({acc_x:10.3f}, {acc_y:10.3f}, {acc_z:10.3f})")
    print("")
    time.sleep(1.0)
