""" Display inclination data five times per second """

import time
from math import atan2, degrees
import board
import busio
import adafruit_lsm303_accel


i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm303_accel.LSM303_Accel(i2c)


def vector_2_degrees(x, y):
    radians = atan2(y, x)
    degrees_calc = degrees(radians)
    if degrees_calc < 0:
        degrees_calc = 360 + degrees_calc
    return degrees_calc


def get_inclination(_sensor):
    return get_inclination_respect_x(_sensor), get_inclination_respect_y(_sensor)


def get_inclination_respect_x(_sensor):
    accel_axis_data = _sensor.acceleration
    return vector_2_degrees(accel_axis_data[0], accel_axis_data[2])


def get_inclination_respect_y(_sensor):
    accel_axis_data = _sensor.acceleration
    return vector_2_degrees(accel_axis_data[1], accel_axis_data[2])


while True:
    print("inclination: (%s, %s)" % (get_inclination(sensor)))
    time.sleep(0.2)
