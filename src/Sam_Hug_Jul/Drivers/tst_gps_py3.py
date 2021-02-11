import time
import sys
import gps_driver_py3 as gpsdrv

gps = gpsdrv.init_line()

for i in range(1000):
    print (gpsdrv.read_gll(gps))

gpsdrv.close(gps)
