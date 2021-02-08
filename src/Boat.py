import numpy as np
from numpy.linalg import norm
from Compass import *
from Motors import *
from GPS import *
import smbus


def sawtooth(x):
    return (x+2*np.pi) % (2*np.pi)-np.pi


class Boat:
    k = 1.15
    lx_home = 48.199129
    ly_home = -3.014017

    def __init__(self):
        # Compass calibration
        # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        self.bus = smbus.SMBus(1)
        # Calibrage de la boussole
        x1 = np.array([[864.02], [-4670.3], [5298.5]])
        x_1 = np.array([[7030.86], [-4034.02], [4805.73]])
        x2 = np.array([[4137.86], [-7385.4], [5082.37]])
        x3 = np.array([[4066.86], [-4540.8], [1934.98]])
        self.compass = Compass(self.bus, x1, x_1, x2, x3)
        self.motors = Motors()
        self.gps = GPS()

    def follow_heading(self, heading_obj, vmin, vmax, f_stop, arg):
        """heading_obj instruction
        vmin minimum speed
        vmax maximum speed
        f_stop stopping condition
        arg argument of f_stop """

        while f_stop(arg):
            vx, vy, vz = self.compass.read_sensor_values().flatten()
            X = np.array([vx, vy, vz]).reshape((3, 1))

            heading = np.arctan2(X[0, 0], X[1, 0])

            e = sawtooth(heading - heading_obj)

            v = ((abs(e)*(vmax - vmin)) / np.pi) + vmin

            u_left = int(0.5*v*(1 + Boat.k*e))
            u_right = int(0.5*v*(1 - Boat.k*e))
            self.motors.command(u_left, u_right)

    def reach_point(self, point):
        """Return if a certain point has been reached
        point is a 2d-array"""
        xy_tilde = self.gps.read_cart_coord()
        return norm(point-xy_tilde) <= 5  # 5m of accuracy

    def compute_heading(self, target_point):
        """ Return heading to go to target point
        target_point array"""
        actual_pos = self.gps.read_cart_coord()
        return np.arctan2(target_point[1, 0]-actual_pos[1, 0], target_point[0, 0]-actual_pos[0, 0])

    def back_to_home(self):
        """Bring back the DDBoat home"""
        home = np.array([[Boat.lx_home], [Boat.ly_home]])
        heading = self.compute_heading(home)
        self.follow_heading(heading, 80, 120, self.reach_point, home)
