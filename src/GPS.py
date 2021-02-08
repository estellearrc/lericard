import drivers.gps_driver_py3 as gpsdrv
import numpy as np
from math import cos, sin
import time


class GPS:
    rho = 6371000  # earth radius
    # precision de 1.9m au bout du ponton
    lx0 = 48.19906500
    ly0 = -3.01473333

    def __init__(self):
        self.range = 5
        self.gps_com = gpsdrv.init_line()

    def read_sensor_values(self):
        data = gpsdrv.read_gll(self.gps_com)
        # divide by 100 to be in degrees
        data[0] = data[0]/100  # North
        data[2] = -data[2]/100  # West = -Est$
        return data

    def destroy(self):
        gpsdrv.close(self.gps_com)

    def read_cart_coord(self):
        data_array = self.read_sensor_values()
        lx = data_array[0]  # North / longitude
        ly = data_array[2]  # West / latitude
        x_tilde = GPS.rho*cos(ly)*(lx-GPS.lx0)
        y_tilde = GPS.rho*(ly-GPS.ly0)
        return np.array([[x_tilde], [y_tilde]])


def test():
    """ Try GPS """
    gps = GPS()
    while True:
        data = gps.read_sensor_values()
        print("[long, lat] = [{}, {}]".format(data[0], data[2]))
        data = gps.read_cart_coord()
        print("[xtilde, ytilde] = [{}, {}]".format(data[0, 0], data[1, 0]))
        time.sleep(1)


if __name__ == "__main__":
    test()
