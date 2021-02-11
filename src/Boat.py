import numpy as np
from numpy.linalg import norm, det
from Compass import *
from Motors import *
from GPS import *
from AcceleroGyro import *
import smbus
import time


def sawtooth(x):
    return (x+np.pi) % (2*np.pi)-np.pi


class Boat:
    Kp = 1
    lx_home, ly_home = convert_longlat_to_rad(48.199129, -3.014017)
    coef_left_motor = 0.8

    def __init__(self):
        # Compass calibration
        # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        self.bus = smbus.SMBus(1)
        # # Calibrage de la boussole boat 14
        # x1 = np.array([[-2388], [-1892], [1668]])
        # x_1 = np.array([[2870], [-2650], [1404]])
        # x2 = np.array([[-50], [-5430], [3447]])
        # x3 = np.array([[300], [-2020], [6780]])
        # Calibrage de la boussole boat 15
        x1 = np.array([[-5494], [1018], [-215]])
        x_1 = np.array([[499], [268], [-2538]])
        x2 = np.array([[-1518], [-2066], [-2431]])
        x3 = np.array([[-1692], [626], [2735]])
        self.compass = Compass(self.bus, x1, x_1, x2, x3)
        self.motors = Motors()
        self.gps = GPS()
        self.last_error = 0
        self.x_home, self.y_home = self.gps.convert_rad_to_cart(
            Boat.lx_home, Boat.ly_home)

    def follow_heading(self, heading, heading_obj, v_obj):
        """Returns motors commands from an heading to follow"""

        # increase the range of the bearing angle
        e = 0.1*(heading_obj - heading)

        M = np.array([[Boat.coef_left_motor, -1], [Boat.coef_left_motor, 1]])
        b = np.array([[(sawtooth(e))], [1]])

        M_1 = np.linalg.pinv(M)  # resolution of the system
        u = M_1.dot(b)  # command motor array

        u_left = v_obj*u[0, 0]
        u_right = v_obj*u[1, 0]  # command right motor
        print("e=", sawtooth(e))
        print("u_left, u_right=", u_left, u_right)
        return u_left, u_right

    def follow_line_potential(self, a, b, t, t0, p):
        """
        Return motos commands from a direction vector
        a: departure point
        b: target point
        t: current time
        t0: departure time
        p: boat postion array([[x],[y]])
        """
        d = (b-a)/norm(b-a)
        v0 = 100*d  # Arbitraire -> a adapter
        n = np.array([[-d[1, 0]], [d[0, 0]]])  # normal vector of the line ab
        # moving attractive point
        phat = a + v0*(t-t0)
        # vector field
        w = -n @ n.T @ (p-a) + v0 + 0.1*(p-phat)
        v_obj = norm(w)
        theta_obj = np.arctan2(w[1, 0], w[0, 0])

        return theta_obj, v_obj

    # def follow_line_heading(self, pointB, vmin, vmax):

    #     pointA = self.gps.read_cart_coord()
    #     # Starting point of the robot in the line following towards pointB
    #     phi = np.arctan2(pointB[1, 0]-pointA[1, 0], pointB[0, 0]-pointA[0, 0])

    #     while self.reach_point(pointB):
    #         x = self.gps.read_cart_coord()
    #         x = x.flatten()
    #         mag_field = self.compass.read_sensor_values()
    #         # boat actual heading
    #         theta = self.compass.compute_heading(
    #             mag_field[0, 0], mag_field[1, 0])
    #         m = np.array([[x[0]], [x[1]]])  # position GPS (x,y)
    #         # error to the following line
    #         e_dist = det(
    #             np.hstack(((pointB-pointA)/norm(pointA-pointB), m-pointA)))
    #         # heading of the line to follow

    #         if abs(e_dist) > self.gps.range:
    #             if e_dist > 0:
    #                 q = 1
    #             else:
    #                 q = -1
    #             # q = sign(e_dist)

    #         theta_bar = phi - np.arctan(e_dist/self.gps.range)

    #         e_heading = sawtooth(theta - theta_bar)

    #         v = ((abs(e_heading)*(vmax - vmin)) / np.pi) + vmin

    #         if q > 0:
    #             # Il faut peut etre intervertir
    #             u_left = int(0.5*v*(1 + Boat.k*e_heading))
    #             u_right = int(0.5*v*(1 - Boat.k*e_heading))
    #         else:
    #             u_right = int(0.5*v*(1 + Boat.k*e_heading))
    #             u_left = int(0.5*v*(1 - Boat.k*e_heading))
    #         self.motors.command(u_left, u_right)

    def reach_point(self, point):
        """Return false when a certain point has been reached
        point is a 2d-array"""
        data = self.gps.read_sensor_values()
        state_vector = self.gps.convert_to_cart_coord(data)
        xy_tilde = np.array([[state_vector[1, 0]], [state_vector[2, 0]]])
        dist = norm(point-xy_tilde)
        print("dist pos to target point = ", dist)
        return dist <= 1

    def compute_heading(self, target_point, actual_pos):
        """ Return heading to go to target point
        target_point array"""
        print('gps : ', actual_pos[0, 0], actual_pos[1, 0])
        return np.arctan2(target_point[1, 0]-actual_pos[1, 0], target_point[0, 0]-actual_pos[0, 0])

    def back_to_home(self):
        """Bring back the DDBoat home"""
        home = np.array([[self.x_home], [self.y_home]])
        heading = self.compute_heading(home)
        self.follow_heading(heading, 100, self.reach_point, home)


def test():
    """ Retrieve earth magnetic field"""
    b = Boat()
    while True:
        B = b.compass.read_sensor_values()
        # print(B)
        print("cap = ", np.arctan2(B[1, 0], B[0, 0]))
        time.sleep(1)


if __name__ == "__main__":
    test()
