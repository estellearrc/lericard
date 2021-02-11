import serial
import os
import time
from threading import Thread
from numpy import cos, pi


class GPS(Thread):

    def __init__(self):
        Thread.__init__(self)

        self.rho = 6371000

        self.number_of_satellites = 0
        self.long_lat = [0, 0]
        self.location = [0, 0]

        self.heading = 0
        self.speed = 0

        self.ser = serial.Serial('/dev/ttyS0', timeout=1.0)

        self.end = False

        print("Initialysing GPS...")        
        self.read_gll()
        print("GPS initialized")

    def close(self):
        self.end = True

    def read_gll(self, nmax=20):
        val = "0000.000, N, 00000.000, W"
        for i in range(nmax):
            v = self.ser.readline().decode("utf-8")
            if str(v[0:6]) == "$GPRMC":
                vv = v.split(",")
                self.speed = float(vv[7])
                self.heading = float(vv[8])
            if str(v[0:6]) == "$GPGGA":
                vv = v.split(",")
                if int(vv[7]) > 0:
                    val = ""
                    val += vv[2] + ","
                    val += vv[3] + ","
                    val += vv[4] + ","
                    val += vv[5]
                    self.number_of_satellites = int(vv[7])
                    break
        return val

    def get_GPS(self):
        val = self.read_gll()
        t = val.split(",")
        ly = t[0]
        ly = int(ly[0:2]) + float(ly[2:])/60
        lx = t[2]
        lx = int(lx[0:3])+float(lx[3:])/60
        self.long_lat = [-lx, ly]

        x = self.rho * ((self.long_lat[0] + 3.01473333)*pi/180)
        y = self.rho * cos(self.long_lat[0]*pi/180) * \
            ((self.long_lat[1] - 48.19906500)*pi/180)
        self.location = [x, y]

    def run(self):
        while not self.end:
            self.get_GPS()
        self.ser.close()
