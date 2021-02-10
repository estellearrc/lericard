#!/usr/bin/python

import numpy as np


def byteToNumber(val_1, val_2):
    number = 256 * val_2 + val_1
    if number >= 32768:
        number = number - 65536
    return number


class AcceleroGyro:
    def __init__(self, bus):
        self.bus = bus
        self.DEVICE_ADDRESS = 0x6b
        self.CTRL1_XL = 0x10
        self.CTRL5_C = 0x14
        ctrl = [0x57, 0x50]  # register 0x10 and 0x11
        self.bus.write_i2c_block_data(self.DEVICE_ADDRESS, self.CTRL1_XL, ctrl)
        ctrl = [0x64, 0x20, 0x00, 0xa5, 0x38, 0x3d]  # registers 0x14 to 0x19
        self.bus.write_i2c_block_data(self.DEVICE_ADDRESS, self.CTRL5_C, ctrl)

    def read_sensor_values(self):
        # Read acc x   0x28 & 0x29

        AX = byteToNumber(self.bus.read_byte_data(
            self.DEVICE_ADDRESS, 0x28), self.bus.read_byte_data(self.DEVICE_ADDRESS, 0x29))

        # Read acc y   0x2a & 0x2b
        AY = byteToNumber(self.bus.read_byte_data(
            self.DEVICE_ADDRESS, 0x2A), self.bus.read_byte_data(self.DEVICE_ADDRESS, 0x2B))

        # Read acc z   0x2c & 0x2d
        AZ = byteToNumber(self.bus.read_byte_data(
            self.DEVICE_ADDRESS, 0x2C), self.bus.read_byte_data(self.DEVICE_ADDRESS, 0x2D))

        # Read gyro x   0x22 & 0x23
        GX = byteToNumber(self.bus.read_byte_data(
            self.DEVICE_ADDRESS, 0x22), self.bus.read_byte_data(self.DEVICE_ADDRESS, 0x23))

        # Read gyro y   0x24 & 0x25
        GY = byteToNumber(self.bus.read_byte_data(
            self.DEVICE_ADDRESS, 0x24), self.bus.read_byte_data(self.DEVICE_ADDRESS, 0x25))

        # Read gyro z   0x26 & 0x27
        GZ = byteToNumber(self.bus.read_byte_data(
            self.DEVICE_ADDRESS, 0x26), self.bus.read_byte_data(self.DEVICE_ADDRESS, 0x27))
        return np.array([AX, AY, AZ, GX, GY, GZ])
