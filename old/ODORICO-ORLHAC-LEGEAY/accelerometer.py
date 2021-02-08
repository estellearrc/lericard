import smbus
import time
import numpy as np

"""Programme de lecture de l'accéléromètre"""


bus = smbus.SMBus(1)

ACCEL_ADRESS = 0X6b
#fifo_ctrl
FIFO_CTRL1 = 0x06
FIFO_CTRL2 = 0x07
FIFO_CTRL3 = 0x08
FIFO_CTRL4 = 0x09
FIFO_CTRL5 = 0x0A

WHO_AM_I = 0x0F
#CTRL
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
CTRL4_C = 0x13
CTRL5_C = 0x14
CTRL6_C = 0x15
CTRL7_G = 0x16
CTRL8_XL = 0x17
CTRL9_XL = 0x18
CTRL10_C = 0x19

#adresseaccel
OUTX_L_XL = 0x28
OUTX_H_XL = 0x29
OUTY_L_XL = 0x2A
OUTY_H_XL = 0x2B
OUTZ_L_XL = 0x2C
OUTZ_H_XL = 0x2D

def read_accelerometer():
	ax = bus.read_byte_data(ACCEL_ADRESS,OUTX_L_XL) + 256*bus.read_byte_data(ACCEL_ADRESS,OUTX_H_XL)
	if ax > 32767:
		ax -= 65536
	ay = bus.read_byte_data(ACCEL_ADRESS,OUTY_L_XL) + 256*bus.read_byte_data(ACCEL_ADRESS,OUTY_H_XL)
	if ay > 32767:
		ay -= 65536

	return np.array([[ax], [ay]])
