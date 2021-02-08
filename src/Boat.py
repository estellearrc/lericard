import numpy as np
from Compass import *
from Motors import *
from GPS import *
import smbus


class Boat:
    def __init__(self):
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
