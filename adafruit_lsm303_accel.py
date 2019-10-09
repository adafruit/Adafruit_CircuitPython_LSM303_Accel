# The MIT License (MIT)
#
# Copyright (c) 2019 Bryan Siepert for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_lsm303_accel`
====================================================


CircuitPython driver for the accelerometer in LSM303 sensors.

* Author(s): Dave Astels, Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* Adafruit `Triple-axis Accelerometer+Magnetometer (Compass) Board - LSM303
  <https://www.adafruit.com/product/1120>`_ (Product ID: 1120)
* Adafruit `FLORA Accelerometer/Compass Sensor - LSM303 - v1.0
  <https://www.adafruit.com/product/1247>`_ (Product ID: 1247)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

try:
    import struct
except ImportError:
    import ustruct as struct
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import UnaryStruct
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_LSM303_Accel.git"

# pylint: disable=bad-whitespace
_ADDRESS_ACCEL             = const(0x19)  # (0x32 >> 1)       // 0011001x
_ADDRESS_MAG               = const(0x1E)  # (0x3C >> 1)       // 0011110x
_ID                        = const(0xD4)  # (0b11010100)

# Accelerometer registers
_REG_ACCEL_CTRL_REG1_A     = const(0x20)
_REG_ACCEL_CTRL_REG2_A     = const(0x21)
_REG_ACCEL_CTRL_REG3_A     = const(0x22)
_REG_ACCEL_CTRL_REG4_A     = const(0x23)
_REG_ACCEL_CTRL_REG5_A     = const(0x24)
_REG_ACCEL_CTRL_REG6_A     = const(0x25)
_REG_ACCEL_REFERENCE_A     = const(0x26)
_REG_ACCEL_STATUS_REG_A    = const(0x27)
_REG_ACCEL_OUT_X_L_A       = const(0x28)
_REG_ACCEL_OUT_X_H_A       = const(0x29)
_REG_ACCEL_OUT_Y_L_A       = const(0x2A)
_REG_ACCEL_OUT_Y_H_A       = const(0x2B)
_REG_ACCEL_OUT_Z_L_A       = const(0x2C)
_REG_ACCEL_OUT_Z_H_A       = const(0x2D)
_REG_ACCEL_FIFO_CTRL_REG_A = const(0x2E)
_REG_ACCEL_FIFO_SRC_REG_A  = const(0x2F)
_REG_ACCEL_INT1_CFG_A      = const(0x30)
_REG_ACCEL_INT1_SOURCE_A   = const(0x31)
_REG_ACCEL_INT1_THS_A      = const(0x32)
_REG_ACCEL_INT1_DURATION_A = const(0x33)
_REG_ACCEL_INT2_CFG_A      = const(0x34)
_REG_ACCEL_INT2_SOURCE_A   = const(0x35)
_REG_ACCEL_INT2_THS_A      = const(0x36)
_REG_ACCEL_INT2_DURATION_A = const(0x37)
_REG_ACCEL_CLICK_CFG_A     = const(0x38)
_REG_ACCEL_CLICK_SRC_A     = const(0x39)
_REG_ACCEL_CLICK_THS_A     = const(0x3A)
_REG_ACCEL_TIME_LIMIT_A    = const(0x3B)
_REG_ACCEL_TIME_LATENCY_A  = const(0x3C)
_REG_ACCEL_TIME_WINDOW_A   = const(0x3D)
_REG_ACCEL_ACT_THS_A       = const(0x3E)
_REG_ACCEL_ACT_DUR_A       = const(0x3F)
_REG_ACCEL_WHO_AM_I        = const(0x0F)

# Conversion constants
_LSM303ACCEL_MG_LSB        = 16704.0
_GRAVITY_STANDARD          = 9.80665      # Earth's gravity in m/s^2
# pylint: enable=bad-whitespace

class LSM303_Accel:
    """Driver for the LSM303's accelerometer."""

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _chip_id = UnaryStruct(_REG_ACCEL_WHO_AM_I, "B")
    _int2_int1_enable = RWBit(_REG_ACCEL_CTRL_REG6_A, 6, 1)
    _int2_int2_enable = RWBit(_REG_ACCEL_CTRL_REG6_A, 5, 1)

    _int2_activity_enable = RWBit(_REG_ACCEL_CTRL_REG6_A, 3, 1)
    _int_pin_active_low = RWBit(_REG_ACCEL_CTRL_REG6_A, 1, 1)

    _act_threshold = UnaryStruct(_REG_ACCEL_ACT_THS_A, "B")
    _act_duration = UnaryStruct(_REG_ACCEL_ACT_DUR_A, "B")

    _data_rate = RWBits(4, _REG_ACCEL_CTRL_REG1_A, 4, 1)

    _BUFFER = bytearray(6)

    def __init__(self, i2c):
        self._accel_device = I2CDevice(i2c, _ADDRESS_ACCEL)
        self.i2c_device = self._accel_device
        self._write_u8(self._accel_device, _REG_ACCEL_CTRL_REG1_A, 0x27)  # Enable the accelerometer

    @property
    def raw_acceleration(self):
        """The raw accelerometer sensor values.
        A 3-tuple of X, Y, Z axis values that are 16-bit signed integers.
        """
        self._read_bytes(self._accel_device, _REG_ACCEL_OUT_X_L_A | 0x80, 6, self._BUFFER)
        return struct.unpack_from('<hhh', self._BUFFER[0:6])

    @property
    def acceleration(self):
        """The processed accelerometer sensor values.
        A 3-tuple of X, Y, Z axis values in meters per second squared that are signed floats.
        """
        raw_accel_data = self.raw_acceleration
        return tuple([n / _LSM303ACCEL_MG_LSB * _GRAVITY_STANDARD for n in raw_accel_data])


    def _read_u8(self, device, address):
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            i2c.write_then_readinto(self._BUFFER, self._BUFFER,
                                    out_end=1, in_end=1)
        return self._BUFFER[0]

    def _write_u8(self, device, address, val):
        with device as i2c:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            i2c.write(self._BUFFER, end=2)

    @staticmethod
    def _read_bytes(device, address, count, buf):
        with device as i2c:
            buf[0] = address & 0xFF
            i2c.write_then_readinto(buf, buf, out_end=1, in_end=count)
