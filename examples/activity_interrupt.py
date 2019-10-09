import board
i2c = board.I2C()

import adafruit_lsm303_accel
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

accel._act_threshold = 20
accel._act_duration = 1
accel._int2_activity_enable = True

# toggle pins, defaults to False
accel._int_pin_active_low = True