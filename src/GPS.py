import drivers.gps_driver_py3 as gpsdrv
import numpy as np
from math import cos, sin
import time


def convert_DDmm_to_rad(lx, ly):
    DDx = int(lx/100)
    # North  / longitude in radian
    lx = np.pi*(DDx + (lx-DDx*100)/60) / 180
    DDy = int(ly/100)
    # West = -Est / latitude in radian
    ly = -np.pi*(DDy + (ly-DDy*100)/60) / 180
    return lx, ly


def convert_longlat_to_rad(lx, ly):
    lx = np.pi*lx/180
    ly = np.pi*ly/180
    return lx, ly


def convert_deg_to_rad(theta):
    rad = (theta*np.pi)/180
    return (rad+np.pi) % (2*np.pi)-np.pi


def convert_knot_to_meterpersec(speed):
    return speed*0.514


class GPS:
    rho = 6371000  # earth radius in meters
    # precision de 1.9m au bout du ponton
    # lx0, ly0 = convert_longlat_to_rad(48.19906500, -3.01473333)
    lx0, ly0 = convert_DDmm_to_rad(4811.9447, 300.8921)
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
        # self.write_coordinates(data[4], data[0], data[2])
        return data

    def destroy(self):
        gpsdrv.close(self.gps_com)

    def convert_rad_to_cart(self, lx, ly):
        x_tilde = GPS.rho*cos(ly)*(lx-GPS.lx0)
        y_tilde = GPS.rho*(ly-GPS.ly0)
        return x_tilde, y_tilde

    def convert_to_cart_coord(self, data):
        lx, ly = convert_DDmm_to_rad(data[0], data[2])
        t = data[4]
        x_tilde, y_tilde = self.convert_rad_to_cart(lx, ly)
        self.write_coordinates(t, x_tilde, y_tilde)
        return t, np.array([[x_tilde], [y_tilde]])

    def write_coordinates(self, t, lon, lat):
        with open(GPS.file_name, "a") as f:
            f.write(str(t) + "," + str(lon) + "," + str(lat) + "\n")


def test():
    """ Try GPS """
    gps = GPS()
    while True:
        data = gps.read_sensor_values()
        print("[long, lat] = [{}, {}]".format(data[0], data[2]))
        t, p = gps.convert_to_cart_coord(data)
        print("[xtilde, ytilde] = [{}, {}]".format(p[0, 0], p[1, 0]))
        time.sleep(0.1)


if __name__ == "__main__":
    test()
