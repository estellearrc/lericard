#!/usr/bin/python

import numpy as np
import smbus
import time


def sawtooth(x):
    return (x+np.pi) % (2*np.pi)-np.pi


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


class Compass:
    # 7 bit address (will be left shifted to add the read write bit)
    DEVICE_ADDRESS = 0x1e
    CTRL_REG1 = 0x20
    CTRL_REG2 = 0x21
    CTRL_REG3 = 0x22
    CTRL_REG4 = 0x23
    CTRL_REG5 = 0x24
    OUT_X_L = 0x28  # first data register to read

    def __init__(self, bus, x1, x_1, x2, x3):
        self.bus = bus
        self.A = np.zeros([3, 3])
        self.b = 0
        self.compute_calibration(x1, x_1, x2, x3)

        # Write a single register
        # Set continuous-conversion mode to (MD1=0,MD0=0)
        self.bus.write_byte_data(Compass.DEVICE_ADDRESS, Compass.CTRL_REG1, 0b01010000)
        self.bus.write_byte_data(Compass.DEVICE_ADDRESS, Compass.CTRL_REG2, 0b00000000)
        self.bus.write_byte_data(Compass.DEVICE_ADDRESS, Compass.CTRL_REG3, 0b00000001)
        self.bus.write_byte_data(Compass.DEVICE_ADDRESS, Compass.CTRL_REG4, 0b00000000)
        self.bus.write_byte_data(Compass.DEVICE_ADDRESS, Compass.CTRL_REG5, 0b00000000)

    def compute_calibration(self, x1, x_1, x2, x3):
        """ Compute A and b -> y = inv(A)@(x+b)
        y ideal magnetic field
        x measured magnetic field
        A matrix
        b bias"""
        self.b = -1/2*(x1 + x_1)
        beta = 4600
        y1 = 1/beta*(x1+self.b)
        y2 = 1/beta*(x2 + self.b)
        y3 = 1/beta*(x3 + self.b)
        self.A[:, 0] = y1.flatten()
        self.A[:, 1] = y2.flatten()
        self.A[:, 2] = y3.flatten()

    def read_sensor_values(self):
        """Read the current value of the magnetic field on the compass"""

        six_values = self.bus.read_i2c_block_data(
            Compass.DEVICE_ADDRESS, Compass.OUT_X_L, 6)
        x, y, z = convert(six_values)
        X = np.array([[x], [y], [z]])
        B = np.linalg.inv(self.A)@(X + self.b)
        return B

    def compute_heading(self, Bx, By):
        """Magic formula"""
        return np.arctan2(By, Bx)


def test():
    """ Retrieve compass measures x1, x_1, x2, x3 for earth magnetic field"""
    pass
    # while True:
    #     bus = smbus.SMBus(1)
    #     DEVICE_ADDRESS = 0x1e
    #     CTRL_REG3 = 0x22
    #     OUT_X_L = 0x28
    #     bus.write_byte_data(DEVICE_ADDRESS, CTRL_REG3, 0b00000000)
    #     six_values = bus.read_i2c_block_data(DEVICE_ADDRESS, OUT_X_L, 6)
    #     x, y, z = convert(six_values)
    #     print("[Bx, By, Bz] = [{}, {}, {}] = ".format(x, y, z))
    #     print("cap = ", sawtooth(np.arctan2(x, y) + np.pi/2))
    #     time.sleep(1)


if __name__ == "__main__":
    test()
