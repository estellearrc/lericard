#!/usr/bin/python

import smbus
import time
import numpy as np
from roblib import *


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


def write_compass_values():
    """
    CALIBRATION
    Write all magnetic values retrieved for calibration into a CSV file
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

    # Read an array of registers
    six_values = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
    three_values = [0, 0, 0]
    dt = 0.1
    fichier = open("data_compass.csv", "w")
    for t in range(1000):
        six_values = bus.read_i2c_block_data(DEVICE_ADDRESS, OUT_X_L, 6)
        # print(six_values)
        three_values = convert(six_values)
        fichier.write(
            str(three_values[0])+";"+str(three_values[1])+";"+str(three_values[2]) + "\n")
        # print(three_values)
        time.sleep(dt)
    fichier.close()


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


def tranform_compass_data(x, y, z):
    """
    correct one point to fit a sphere instead of an ellipsoid
    """
    point = np.array([[x], [y], [z]])
    center = np.array([[334.], [-2022.], [3758.]])
    point_trans = translate(point, center)
    point_norm = normalize_1_point(point_trans)
    # point = correct_manually(x, y, z)
    return point_norm


def read_compass_values():
    """
    CALIBRATION
    read the file of magnetic values for compass calibration
    """
    X = []
    Y = []
    Z = []
    fichier = open("data_compass.csv", "r")
    for elt in fichier.readlines():
        line = elt.strip("\n").split(";")
        X.append(int(line[0]))
        Y.append(int(line[1]))
        Z.append(int(line[2]))
    return X, Y, Z


def display_compass_values(points, center=np.array([[0], [0], [0]])):
    """
    Display in a 3d figure the compass values (display ellipsoid or sphere)
    """
    print(points)
    fig = figure()
    ax = Axes3D(fig)
    plot3D(ax, points)
    R = eye(3, 3)
    draw_axis3D(ax, 0, 0, 0, R, zoom=50)
    draw_axis3D(ax, center[0, 0], center[1, 0], center[2, 0], R, zoom=50)
    # X = points[0, :]
    # Y = points[1, :]
    # plot(Y/X, np.arctan2(Y, X)*180/pi)
    show()


def computer_ellipse_center(X, Y, Z):
    """
    CALIBRATION
    Compute the center of the ellipsoid
    """
    minX, minY, minZ = min(X), min(Y), min(Z)
    maxX, maxY, maxZ = max(X), max(Y), max(Z)
    center = np.array(
        [[(maxX+minX)/2], [(maxY+minY)/2], [(maxZ+minZ)/2]])
    return center


def translate(points, p):
    """
    CALIBRATION OR NOT
    translate all points of the vector p
    """
    return points - p


def normalize_1_point(point):
    """
    Normalize one point to match the coordinates of a sphere.
    Transform the ellipsoid into a sphere
    """
    x, y, z = point[0, 0], point[1, 0], point[2, 0]
    minX, minY, minZ = -3292.0, -3004.0, -3198.5
    maxX, maxY, maxZ = 3292.0, 3004.0, 3198.5
    a, b, c = (maxX-minX)/2, (maxY-minY)/2, (maxZ-minZ)/2
    x_sphere = 3000*x/a
    y_sphere = 3000*y/b
    z_sphere = 3000*z/c
    return np.array([[x_sphere], [y_sphere], [z_sphere]])


def normalize(points):
    """
    CALIBRATION
    Normalize all points to match the coordinates of a sphere.
    Transform the ellipsoid into a sphere
    """
    X = points[0, :]
    Y = points[1, :]
    Z = points[2, :]
    minX, minY, minZ = min(X), min(Y), min(Z)
    maxX, maxY, maxZ = max(X), max(Y), max(Z)
    print([minX, minY, minZ])
    print([maxX, maxY, maxZ])
    a, b, c = (maxX-minX)/2, (maxY-minY)/2, (maxZ-minZ)/2
    X_sphere = 3000*X/a
    Y_sphere = 3000*Y/b
    Z_sphere = 3000*Z/c
    minX, minY, minZ = min(X_sphere), min(Y_sphere), min(Z_sphere)
    maxX, maxY, maxZ = max(X_sphere), max(Y_sphere), max(Z_sphere)
    # print([minX, minY, minZ])
    # print([maxX, maxY, maxZ])
    return np.array([X_sphere, Y_sphere, Z_sphere])


def calibrate():
    """
    CALIBRATION
    """
    # write_compass_values()
    X, Y, Z = read_compass_values()
    points = np.array([X, Y, Z])
    # display_compass_values(points)
    center = computer_ellipse_center(X, Y, Z)
    print(center)
    points_trans = translate(points, center)
    # display_compass_values(points_trans, center)

    points_norm = normalize(points_trans)
    display_compass_values(points_norm, center)


def test():
    """
    Display the values of the magnetic field into the terminal
    """
    x, y, z = retrieve_compass_values()
    print("Bx = %d G, By = %d G, Bz = %d G", x, y, z)
    time.sleep(1)


def correct_manually(Bx, By, Bz):
    """
    CALIBRATION
    Compute the center of the ellipsoid and apply the sphere deformation with values found experimentally (b, A)
    """
    beta = 0.46  # magnetic field in Brest into Gauss
    xb1 = np.array([[-2884], [-2032], [3993]])
    xb2 = np.array([[3420], [-1736], [3437]])
    x1 = xb1
    x2 = np.array([[408], [-5190], [3750]])
    x3 = np.array([[438], [-2120], [6747]])
    b = -(xb1+xb2)/2
    a1 = (x1+b)/beta
    a2 = (x2+b)/beta
    a3 = (x3+b)/beta
    A = np.hstack((a1, a2, a3))
    B = np.array([Bx, By, Bz])
    print("b", B)
    B_corrected = np.linalg.pinv(A).dot(B + b)
    return B_corrected


def calibrate_manually():
    """
    CALIBRATION
    Calibrate with values (b,A) found experimentally (inclination)
    """
    # second way to calibrate the compass, doesn't work... the ellipsoid is even more ellipsoid than before...
    Bx, By, Bz = read_compass_values()
    points = correct_manually(Bx, By, Bz)
    display_compass_values(points)


if __name__ == "__main__":
    # calibrate()
    # retrieve_compass_values()
    calibrate_manually()
    # retrieve_compass_values()
    # while(1):
    #     test()
