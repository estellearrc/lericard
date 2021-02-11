import time
import sys
import gps_driver_py3 as gpsdrv

flog = open ("gps_guerledan_1.log","w")

gps = gpsdrv.init_line()

for i in range(500):
    val,ok = gpsdrv.read_gll(gps)
    print (val)
    st = "{:f}".format(val[0])
    st = st+" "+val[1]
    st = st+" "+"{:f}".format(val[2])
    st = st+" "+val[3]
    st = st+" "+"{:f}".format(val[4])
    flog.write(st+"\n")

flog.close()
gpsdrv.close(gps)
