import numpy as np
from Compass import *


class Boat:
    def __init__(self):
        # Compass calibration
        x1 = np.array([[864.02], [-4670.3], [5298.5]])
        x_1 = np.array([[7030.86], [-4034.02], [4805.73]])
        x2 = np.array([[4137.86], [-7385.4], [5082.37]])
        x3 = np.array([[4066.86], [-4540.8], [1934.98]])
        self.compass = Compass(x1, x_1, x2, x3)
    
    
    def follow_heading(heading_obj, vmin, vmax, f_stop):
        """heading_obj instruction
        vmin minimum speed
        vmax maximum speed
        f_stop stopping condition """
        
        while f_stop():
            vx, vy, vz = self.compass.read_sensor_values()
            X = np.array([vx, vy, vz]).reshape((3, 1))

            cap = np.arctan2(X[0, 0], X[1, 0])

            e = sawtooth(cap - cap_bar)

            v = ((abs(e)*(vmax - vmin)) / np.pi) + vmin

            u1 = int(0.5*v*(1 + K*e))
            u2 = int(0.5*v*(1 - K*e))
            
            # ard.send_arduino_cmd_motor(arduino, u1, u2)
    
    
    def reach_point():
        """Return if a certain point has been reached"""
        return 3<4
        


def sawtooth(x):
    return (x+2*np.pi) % (2*np.pi)-np.pi
