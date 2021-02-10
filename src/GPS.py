import drivers.gps_driver_py3 as gpsdrv
import numpy as np
from math import cos, sin
import time


class GPS:
    rho = 6371000  # earth radius
    # precision de 1.9m au bout du ponton
    lx0 = 48.19906500
    ly0 = -3.01473333
    file_name = "GPS_traceback.csv"

    def __init__(self):
        self.range = 5
        self.gps_com = gpsdrv.init_line()
        f = open(GPS.file_name, "w")
        f.close()

    def read_sensor_values(self):
        data = gpsdrv.read_gll(self.gps_com)
        # divide by 100 to be in degrees
        # ly = pi*(DD+mm.mm/60)/100
        DDx = int(data[0]/100)
        data[0] = np.pi*(DDx + (data[0]-DDx*100)/60)/180  # North
        DDy = int(data[2]/100)
        data[2] = -np.pi*(DDy + (data[2]-DDy*100)/60)/180  # West = -Est
        self.write_coordinates(data[0], data[2])
        return data

    def destroy(self):
        gpsdrv.close(self.gps_com)

    def convert_to_cart_coord(self, lx, ly):
        x_tilde = GPS.rho*cos(ly)*(lx-GPS.lx0)
        y_tilde = GPS.rho*(ly-GPS.ly0)
        return np.array([[x_tilde], [y_tilde]])

    def read_cart_coord(self):
        data_array = self.read_sensor_values()
        lx = data_array[0]  # North / longitude
        ly = data_array[2]  # West / latitude
        return self.convert_to_cart_coord(lx, ly)

    def write_coordinates(self, lon, lat):
        with open(GPS.file_name, "a") as f:
            f.write(str(lon) + "," + str(lat) + "\n")


def test():
    """ Try GPS """
    gps = GPS()
    while True:
        data = gps.read_sensor_values()
        print("[long, lat] = [{}, {}]".format(data[0], data[2]))
        data = gps.read_cart_coord()
        # print("[xtilde, ytilde] = [{}, {}]".format(data[0, 0], data[1, 0]))
        time.sleep(0.1)


if __name__ == "__main__":
    test()
