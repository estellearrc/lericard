import serial
import os
import time
import struct
from threading import Thread

# encoder frame
#  0     : sync 0xFF
#  1     : sync 0x0D
#  2-5   : timer MSByte first
#  6     : direction 1 (left)
#  7     : direction 2 (right)
#  8-9   : encoder 1 (left)
#  10-11 : encoder 2 (right)
#  12-13 : Hall voltage 1 (left)
#  14-15 : Hall voltage 2 (right)
#  16    : sync 0xAA

# data = [timer,dirLeft,dirRight,encLeft,encRight,voltLeft,voltRight]


def delta_odo(odo1, odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


class Encoders(Thread):

    def __init__(self):
        Thread.__init__(self)

        set_baudrate()

        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)

        self.old_time = 0
        self.time = 0

        self.old_encoder_left = 0
        self.encoder_left = 0
        self.old_encoder_right = 0
        self.encoder_right = 0

        self.speed_left = 0
        self.speed_right = 0

        self.end = False

    def get_sync(self):
        while True:
            c1 = self.serial.read(1)
            c1 = struct.unpack('B', c1)[0]
            if c1 == 0xff:
                c2 = self.serial.read(1)
                c2 = struct.unpack('B', c2)[0]
                if c2 == 0x0d:
                    v = self.serial.read(15)
                    break

    def read_encoders(self, debug=False):
        self.old_time = self.time
        self.old_encoder_left = self.encoder_left
        self.old_encoder_right = self.encoder_right

        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)

        self.get_sync()

        sync = True
        data = []
        v = self.serial.read(17)
        c1 = v[0]
        c2 = v[1]
        if (c1 != 0xff) or (c2 != 0x0d):
            if debug:
                print("sync lost, exit")
            sync = False
        else:
            self.time = (v[2] << 32)
            self.time += (v[3] << 16)
            self.time += (v[4] << 8)
            self.time += v[5]
            sensLeft = v[6]
            sensRight = v[7]
            self.encoder_right = v[8] << 8
            self.encoder_right += v[9]
            self.encoder_left = v[10] << 8
            self.encoder_left += v[11]
            voltLeft = v[12] << 8
            voltLeft += v[13]
            voltRight = v[14] << 8
            voltRight += v[15]
            c3 = v[16]
            stc3 = "%2.2X" % (c3)
            data.append(self.time)
            data.append(sensLeft)
            data.append(sensRight)
            data.append(self.encoder_left)
            data.append(self.encoder_right)
            data.append(voltLeft)
            data.append(voltRight)
            if debug:
                print(self.time, sensLeft, sensRight, self.encoder_left,
                      self.encoder_right, voltLeft, voltRight, stc3)

        self.serial.close()

        return sync, data

    def get_speed(self):
        self.read_encoders()

        self.speed_left = (delta_odo(self.encoder_left,
                                     self.old_encoder_left))/(self.time - self.old_time)
        self.speed_right = -(delta_odo(self.encoder_right, self.old_encoder_right))/(self.time - self.old_time)

        return self.speed_left, self.speed_right

    def run(self):
        while not self.end:
            self.read_encoders()
            self.get_speed()

    def close(self):
        self.end = True


def set_baudrate(baudrate=115200):
    st = os.system("stty -F /dev/ttyUSB0 %d" % (baudrate))
    print(st)
    st = os.system("stty -F /dev/ttyUSB0")
    print(st)


def init_line():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
    time.sleep(1.0)
    return ser


def convertHexToString(char):
    return "\\x" + str(hex(ord(char)))[2:]


def sync(ser):
    while True:
        c1 = ser.read(1)
        c1 = struct.unpack('B', c1)[0]
        if c1 == 0xff:
            c2 = ser.read(1)
            c2 = struct.unpack('B', c2)[0]
            if c2 == 0x0d:
                v = ser.read(15)
                break


def read_packet(ser, debug=True):
    sync = True
    data = []
    v = ser.read(17)
    c1 = v[0]
    c2 = v[1]
    if (c1 != 0xff) or (c2 != 0x0d):
        if debug:
            print("sync lost, exit")
        sync = False
    else:
        timer = (v[2] << 32)
        timer += (v[3] << 16)
        timer += (v[4] << 8)
        timer += v[5]
        sensLeft = v[6]
        sensRight = v[7]
        posLeft = v[8] << 8
        posLeft += v[9]
        posRight = v[10] << 8
        posRight += v[11]
        voltLeft = v[12] << 8
        voltLeft += v[13]
        voltRight = v[14] << 8
        voltRight += v[15]
        c3 = v[16]
        stc3 = "%2.2X" % (c3)
        data.append(timer)
        data.append(sensLeft)
        data.append(sensRight)
        data.append(posLeft)
        data.append(posRight)
        data.append(voltLeft)
        data.append(voltRight)
        if debug:
            print(timer, sensLeft, sensRight, posLeft,
                  posRight, voltLeft, voltRight, stc3)
    return sync, data
