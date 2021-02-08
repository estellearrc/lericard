import drivers.gps_driver_py3 as gpsdrv
from math import cos, sin


class GPS:
    rho = 6371000  # earth radius
    # precision de 1.9m au bout du ponton
    lx0 = 48.19906500
    ly0 = -3.01473333

    def __init__(self):
        self.gps_com = gpsdrv.init_line()

    def read_sensor_values(self):
        return gpsdrv.read_gll(self.gps_com)  # TO TEST

    def destroy(self):
        gpsdrv.close(self.gps_com)

    def convert_to_cart_coord(self):
        data_array = self.read_sensor_values()
        lx = data_array[2]  # North / longitude
        ly = data_array[4]  # West / latitude
        x_tilde = GPS.rho*cos(ly)*(lx-GPS.lx0)
        y_tilde = GPS.rho*(ly-GPS.ly0)
        return x_tilde, y_tilde
