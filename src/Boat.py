import numpy as np
from numpy.linalg import norm, det
from Compass import *
from Motors import *
from GPS import *
import smbus
import time


def sawtooth(x):
    return (x+np.pi) % (2*np.pi)-np.pi


class Boat:
    k = 0.1/np.pi
    lx_home = 48.199129
    ly_home = -3.014017
    coef_left_motor = 1

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

    def follow_heading(self, heading, heading_obj, v_obj):
        """Returns motors commands from an heading to follow"""
        
        # increase the range of the bearing angle
        e = 0.2*(heading_obj - heading)
        u = self.motors.compute_command(e)
        print("error: ",e)
        print("heading: ",heading)
        u_left = v_obj*Boat.coef_left_motor*u[0, 0]
        u_right = v_obj*u[1, 0]  # command right motor
        print("uleft = ", u_left)
        print("uright = ", u_right)
        return u_left, u_right

    def follow_line_potential(self, b):
        gpgll_a = self.gps.read_sensor_values()
        a = self.gps.convert_to_cart_coord(gpgll_a[0, 0], gpgll_a[1, 0])
        t0 = gpgll_a[4]
        d = (b-a)/norm(b-a)  # direction vector of the line ab
        v0 = 100*d
        n = np.array([[-d[1, 0]], [d[0, 0]]])  # normal vector of the line ab
        gpgll = self.gps.read_sensor_values()
        p = self.gps.convert_to_cart_coord(gpgll[0, 0], gpgll[1, 0])
        while norm(p - b) > 1:  # while the boat hasn't reached point b
            gpgll = self.gps.read_sensor_values()
            p = self.gps.convert_to_cart_coord(gpgll[0, 0], gpgll[1, 0])
            B = self.compass.read_sensor_values()
            heading = self.compass.compute_heading(B[0, 0], B[1, 0])
            t = gpgll[4]
            # moving attractive point
            phat = a + v0*(t-t0)
            # vector field
            w = -n@n.T@(p-a)+v0+0.1*(p-phat)
            v_obj = norm(w)
            thetabar = np.arctan2(w[1, 0], w[0, 0])

            # commande proportionnelle
            e = sawtooth(thetabar-heading)
            u_right = int(0.5*v_obj*(1 + Boat.k*e))
            u_left = int(0.5*v_obj*(1 - Boat.k*e))
            self.motors.command(u_left, u_right)
            time.sleep(0.2)

    def follow_line_heading(self, pointB, vmin, vmax):

        pointA = self.gps.read_cart_coord()
        # Starting point of the robot in the line following towards pointB
        phi = np.arctan2(pointB[1, 0]-pointA[1, 0], pointB[0, 0]-pointA[0, 0])

        while self.reach_point(pointB):
            x = self.gps.read_cart_coord()
            x = x.flatten()
            mag_field = self.compass.read_sensor_values()
            # boat actual heading
            theta = self.compass.compute_heading(
                mag_field[0, 0], mag_field[1, 0])
            m = np.array([[x[0]], [x[1]]])  # position GPS (x,y)
            # error to the following line
            e_dist = det(
                np.hstack(((pointB-pointA)/norm(pointA-pointB), m-pointA)))
            # heading of the line to follow

            if abs(e_dist) > self.gps.range:
                if e_dist > 0:
                    q = 1
                else:
                    q = -1
                # q = sign(e_dist)

            theta_bar = phi - np.arctan(e_dist/self.gps.range)

            e_heading = sawtooth(theta - theta_bar)

            v = ((abs(e_heading)*(vmax - vmin)) / np.pi) + vmin

            if q > 0:
                # Il faut peut etre intervertir
                u_left = int(0.5*v*(1 + Boat.k*e_heading))
                u_right = int(0.5*v*(1 - Boat.k*e_heading))
            else:
                u_right = int(0.5*v*(1 + Boat.k*e_heading))
                u_left = int(0.5*v*(1 - Boat.k*e_heading))
            self.motors.command(u_left, u_right)

    def reach_point(self, point):
        """Return false when a certain point has been reached
        point is a 2d-array"""
        xy_tilde = self.gps.read_cart_coord()
        return norm(point-xy_tilde) <= 1

    def compute_heading(self, target_point):
        """ Return heading to go to target point
        target_point array"""
        actual_pos = self.gps.read_cart_coord()
        return np.arctan2(target_point[1, 0]-actual_pos[1, 0], target_point[0, 0]-actual_pos[0, 0])

    def back_to_home(self):
        """Bring back the DDBoat home"""
        home = np.array([[Boat.lx_home], [Boat.ly_home]])
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
