import numpy as np


class Compass:
    def __init__(self, x1, x_1, x2, x3):
        self.A = np.zeros([3, 3])
        self.b = 0
        self.compute_calibration(x1, x_1, x2, x3)

    def compute_calibration(self, x1, x_1, x2, x3):
        self.b = -1/2*(x1 + x_1)
        beta = 46000
        y1 = 1/beta*(x1+self.b)
        y2 = 1/beta*(x2 + self.b)
        y3 = 1/beta*(x3 + self.b)
        self.A[:, 0] = y1.flatten()
        self.A[:, 1] = y2.flatten()
        self.A[:, 2] = y3.flatten()
