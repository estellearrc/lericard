import serial
import os
import time


def init_line():
    ser = serial.Serial('/dev/ttyS0', timeout=1.0)
    time.sleep(1.0)
    print(ser)
    return ser


def read_gll(ser, nmax=20):
    val = [0., 'N', 0., 'W', 0.]
    for i in range(nmax):
        v = ser.readline().decode("utf-8")
        print(v)
        if str(v[0:6]) == "$GPGLL":
            vv = v.split(",")
            if len(vv[1]) > 0:
                val[0] = float(vv[1])
            if len(vv[2]) > 0:
                val[1] = vv[2]
            if len(vv[3]) > 0:
                val[2] = float(vv[3])
            if len(vv[4]) > 0:
                val[3] = vv[4]
            if len(vv[5]) > 0:
                val[4] = float(vv[5])
            break
    return val


def read_gprmc(ser, nmax=20):
    # $GPRMC,111200.000,A,4811.9520,N,00300.8641,W,0.94,254.28,110221,,,A*70
    val = [0., 'A', 0., 'N', 0., 'W', 0., 0., 0., 0., 0., 0., 'A*70']
    for i in range(nmax):
        v = ser.readline().decode("utf-8")
        print(v)
        if str(v[0:6]) == "$GPRMC":
            vv = v.split(",")
            if len(vv[1]) > 0:
                val[0] = float(vv[1])  # time
            if len(vv[2]) > 0:
                val[1] = float(vv[2])  # North = latitude
            if len(vv[4]) > 0:
                val[3] = float(vv[4])  # West = longitude
            if len(vv[4]) > 0:
                val[3] = vv[4]
            if len(vv[5]) > 0:
                val[4] = float(vv[5])
            break
    return val


def close(ser):
    ser.close()
