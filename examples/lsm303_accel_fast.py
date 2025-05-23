# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Read data from the accelerometer and print it out, ASAP!"""

import board

import adafruit_lsm303_accel

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)

while True:
    accel_x, accel_y, accel_z = sensor.acceleration
    print(f"{accel_x:10.3f} {accel_y:10.3f} {accel_z:10.3f}")
