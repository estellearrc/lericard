import serial
import os
import time
import struct

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

def set_baudrate(baudrate=115200):
    st = os.system ("stty -F /dev/ttyUSB0 %d"%(baudrate))
    print (st)
    st = os.system ("stty -F /dev/ttyUSB0")
    print (st)

def init_line():
    ser = serial.Serial('/dev/ttyUSB0',115200,timeout=1.0)
    time.sleep(1.0)
    print (ser)
    return ser

def close_line(ser):
    ser.close()

def get_sync(ser):
    while True:
        c1 = ser.read(1)
        if ord(c1) == 0xff:
            c2 = ser.read(1)
            if ord(c2) == 0x0d:
                v = ser.read(15)
                break

def read_single_packet(debug=True):
    ser = init_line()
    get_sync(ser)
    sync,data = read_packet(ser,debug=debug)
    close_line(ser)
    timeAcq = data[0]
    sensLeft = data[1]
    sensRight = data[2]
    posLeft = data[3]
    posRight = data[4]
    return sync, timeAcq, sensLeft, sensRight, posLeft, posRight

def read_packet(ser,debug=True):
    sync = True
    data = []
    v=ser.read(17)
    #print (type(v))
    #st=""
    #for i in range(len(v)):
    #  st += "%2.2x"%(ord(v[i]))
    #print st
    c1 = v[0]
    c2 = v[1]
    if (c1 != 0xff) or (c2 != 0x0d):
      if debug:
          print ("sync lost, exit")
      sync = False
    else:
      timer = (v[2] << 32)
      timer += (v[3] << 16)
      timer += (v[4] << 8)
      timer += v[5]
      sensLeft = v[7]
      sensRight= v[6]
      posLeft = v[10] << 8
      posLeft += v[11]
      posRight = v[8] << 8
      posRight += v[9]
      voltLeft = v[14] << 8
      voltLeft += v[15]
      voltRight = v[12] << 8
      voltRight += v[13]
      c3 = v[16]
      stc3 = "%2.2X"%(c3)
      data.append(timer)
      data.append(sensLeft)
      data.append(sensRight)
      data.append(posLeft)
      data.append(posRight)
      data.append(voltLeft)
      data.append(voltRight)
      if debug:
          print (timer,sensLeft,sensRight,posLeft,posRight,voltLeft,voltRight,stc3)
    return sync,data

    
