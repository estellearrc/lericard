#!/usr/bin/python

import smbus
import time
import numpy as np


def merge(lower_byte, upper_byte):
    """merge 2 bytes (2*8 bits) to form a 16-bit long binary integer"""
    res = lower_byte + upper_byte*256
    return res


def bin2decs(x):
    """Convert a binary string into a signed decimal integer"""
    if x > 32767:
        x = x - 65536
    return x


def convert(tab):
    """ convert a table of 6 bytes into 3 decimal integers"""
    three_values = [0, 0, 0]
    for i in range(0, 6, 2):
        two_bytes = merge(tab[i], tab[i+1])
        three_values[int(i/2)] = bin2decs(two_bytes)
    return three_values


def retrieve_compass_values():
    """
    Read the current value of the magnetic field on the compass
    """
    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
    bus = smbus.SMBus(1)

    # 7 bit address (will be left shifted to add the read write bit)
    DEVICE_ADDRESS = 0x1e
    CTRL_REG3 = 0x22
    OUT_X_L = 0x28  # first data register to read

    # Write a single register
    # Set continuous-conversion mode to (MD1=0,MD0=0)
    bus.write_byte_data(DEVICE_ADDRESS, CTRL_REG3, 0b00000000)
    six_values = bus.read_i2c_block_data(DEVICE_ADDRESS, OUT_X_L, 6)
    x, y, z = convert(six_values)
    return x, y, z


def apply_calibration(x1, x_1, x2, x3):
    # # Calibrage de la boussole
    # x1 = np.array([[864.02], [-4670.3], [5298.5]])
    # x_1 = np.array([[7030.86], [-4034.02], [4805.73]])
    # x2 = np.array([[4137.86], [-7385.4], [5082.37]])
    # x3 = np.array([[4066.86], [-4540.8], [1934.98]])

    b = -1/2*(x1 + x_1)
    beta = 46000
    v1 = 1/beta*(x1+b)
    v2 = 1/beta*(x2 + b)
    v3 = 1/beta*(x3 + b)
    M = np.zeros([3, 3])
    M[:, 0] = v1.flatten()
    M[:, 1] = v2.flatten()
    M[:, 2] = v3.flatten()
    return M, b
