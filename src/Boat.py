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
    Kp = 0.6
    lx_home, ly_home = convert_longlat_to_rad(48.199129, -3.014017)
    coef_left_motor = 0.6

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

    def follow_heading(self, heading_gps, heading_compasssss, heading_obj, v_obj):
        """Returns motors commands from an heading to follow"""

        # increase the range of the bearing angle
        e = 0.35*(heading_obj - heading_gps)
        print("e : ", abs(sawtooth(e)))
        
        # while abs(sawtooth(e)) > 0.6:
        #    self.motors.stop()
        #    headings = []
        #    for k in range(5):
        #        mag_field = self.compass.read_sensor_values().flatten().reshape((3, 1))
        #        headings.append(self.compass.compute_heading(mag_field[0, 0], mag_field[1, 0]))
        #        time.sleep(0.2)
        #    heading_compass = np.mean(np.array(headings))
        #    e = 0.35*(heading_obj - heading_compass)
        #    print("e : ", abs(sawtooth(e)))
        #    if e > 0:
        #        self.motors.command(0, 150)
        #        time.sleep(0.5)
        #        self.motors.stop()
        #    else:
        #        self.motors.command(150, 0)
        #        time.sleep(0.5)
        #        self.motors.stop()
        #    time.sleep(2)

        M = np.array([[Boat.coef_left_motor, -1], [Boat.coef_left_motor, 1]])
        b = np.array([[Boat.Kp*sawtooth(e)], [1]])

        M_1 = np.linalg.pinv(M)  # resolution of the system
        u = M_1.dot(b)  # command motor array

        u_left = v_obj*u[0, 0]
        u_right = v_obj*u[1, 0]  # command right motor
        return u_left, u_right

    def follow_line_potential(self, a, b, p, phat, v0):
        """
        Return motos commands from a direction vector
        a: departure point
        b: target point
        p: boat postion array([[x],[y]])
        phat: moving attractive point a + v0*(t-t0)
        v0: vitesse du point atractif   
        """
        d = (b-a)/norm(b-a)
        n = np.array([[-d[1, 0]], [d[0, 0]]])  # normal vector of the line ab
        # vector field
        w = -n @ n.T @ (p-a) + v0 + 0.1*(phat-p)
        v_bar = norm(w)
        theta_bar = np.arctan2(w[1, 0], w[0, 0])
        return theta_bar, v_bar


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
