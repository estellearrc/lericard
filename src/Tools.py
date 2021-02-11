import numpy as np
from numpy import pi
from smbus import SMBus
import Drivers.arduino_driver_py3 as ardudrv
from Drivers.arduino_driver_py3 import clip_cmd
import Drivers.gps_driver_py3 as gps_drv
import time


x1 =  np.array([[  608.4], [-3196.2], [ 5461.7]])
x2 =  np.array([[ 4005.6], [-6290. ], [ 4947.3]])
x3 =  np.array([[ 3945.2], [-4062.2], [ 8548.8]])
x_1 =  np.array([[ 7150.2], [-3191.6], [ 5072.3]])

b = -0.5*(x1+x_1)
A = 1/46* np.concatenate((x1 + b, x2 + b, x3 +b ), axis=1)

def init_compass():
    bus = SMBus(1)
    address = 0x1e
    bus.write_byte_data(address, 0x20, 0b00010000)
    bus.write_byte_data(address, 0x21, 0b00000000)
    bus.write_byte_data(address, 0x22, 0b00000000)
    bus.write_byte_data(address, 0x23, 0b00000000)
    bus.write_byte_data(address, 0x24, 0b00000000)
    print("Compass initialized")
    return bus


def get_compass1(bus):
    xl = bus.read_byte_data(0x1e, 0x28)
    xh = bus.read_byte_data(0x1e, 0x29)
    yl = bus.read_byte_data(0x1e, 0x2A)
    yh = bus.read_byte_data(0x1e, 0x2B)
    zl = bus.read_byte_data(0x1e, 0x2C)
    zh = bus.read_byte_data(0x1e, 0x2D)
    X_raw = xl + 256 * xh
    if X_raw > 32767:
        X_raw -= 65536
    Y_raw = yl + 256 * yh
    if Y_raw > 32767:
        Y_raw -= 65536
    Z_raw = zl + 256 * zh
    if Z_raw > 32767:
        Z_raw -= 65536

    true_direction = np.array([[X_raw], [Y_raw], [Z_raw]])

    return true_direction


def get_heading(bus):
    psi0 = get_compass1(bus)
    true_direction = np.linalg.inv(A)@(psi0+b)
    direction = np.array([true_direction[0], true_direction[1]])
    X = true_direction[0, 0] / np.linalg.norm(direction)
    Y = true_direction[1, 0] / np.linalg.norm(direction)

    return np.arctan2(Y, X)



def init_accel():
    bus = SMBus(1)
    adresse = 0x6b
    bus.write_byte_data(adresse, 0x10, 0b01010111)  # CTRL 1
    bus.write_byte_data(adresse, 0x11, 0x01010000)  # CTRL 2
    bus.write_byte_data(adresse, 0x14, 0b01100100)  # CTRL 5
    bus.write_byte_data(adresse, 0x15, 0b00100000)  # CTRL 6
    bus.write_byte_data(adresse, 0x16, 0x00000000)  # CTRL 7
    bus.write_byte_data(adresse, 0x17, 0b10100101)  # CTRL 8
    bus.write_byte_data(adresse, 0x18, 0b00111000)  # CTRL 9
    bus.write_byte_data(adresse, 0x19, 0b00111101)  # CTRL 10
    print("Accelerometer initialized")
    return bus


def get_accel(bus):
    xl = bus.read_byte_data(0x6b, 0x28)
    xh = bus.read_byte_data(0x6b, 0x29)
    yl = bus.read_byte_data(0x6b, 0x2A)
    yh = bus.read_byte_data(0x6b, 0x2B)
    zl = bus.read_byte_data(0x6b, 0x2C)
    zh = bus.read_byte_data(0x6b, 0x2D)

    X = xl + 256 * xh
    if X > 32767:
        X -= 65536
    Y = yl + 256 * yh
    if Y > 32767:
        Y -= 65536
    Z = zl + 256 * zh
    if Z > 32767:
        Z -= 65536

    return X, Y, Z


def init_motors():
    serial_arduino, data_arduino = ardudrv.init_arduino_line()
    print("Motors initialized")
    return serial_arduino


def set_motor_speed(serial, l, r):
    ardudrv.send_arduino_cmd_motor(serial, l, r)


def stop_motors(serial):
    ardudrv.send_arduino_cmd_motor(serial, 0, 0)


def regul_cap(psi0, psi, speed):

    e = psi - psi0

    A = np.array([[0.5, 0.5], [-0.5, 0.5]])
    B = np.array([[(e + np.pi) % (2 * np.pi) - np.pi],
                  [1]])

    U = A@B * speed

    return U


def init_GPS():
    gps_serial = gps_drv.init_line()
    return gps_serial


def get_GPS(ser):
    val = gps_drv.read_gll(ser)
    t = val[1:-1].split(",")
    ly = t[0]
    ly = int(ly[0:2]) + float(ly[2:])/60
    lx = t[2]
    print(t[2])
    lx = int(lx[0:3])+float(lx[3:])/60
    return lx, ly


rho = 6371000


def GPS_2_cartesian(lx, ly):
    lx = -lx
    x = rho * ((lx + 3.01473333)*np.pi/180)
    y = rho * np.cos(lx*np.pi/180)*((ly - 48.19906500)*np.pi/180)
    return x, y

def sawtooth(x):
    return (x+pi) % (2*pi)-pi