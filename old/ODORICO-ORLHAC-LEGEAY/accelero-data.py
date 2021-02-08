import smbus
import time
import arduino_driver_py3 as ard

"""Permet de récolter des données d'accéléromètre pour traitement et mise en place d'une
détection des chocs"""


bus = smbus.SMBus(1)
ACCEL_ADRESS = 0X6b
#fifo_ctrl
FIFO_CTRL1 = 0x06
FIFO_CTRL2 = 0x07
FIFO_CTRL3 = 0x08
FIFO_CTRL4 = 0x09
FIFO_CTRL5 = 0x0A

WHO_AM_I = 0x0F
#CTRL32
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

arduino = ard.init_arduino_line()[0]


f = open("Datasheetacc.txt","w")
for i in range(5000):
	ard.send_arduino_cmd_motor(arduino, (50+(i*120)/5000)*1.1, 50+(i*120)/5000)
	chaine = ""
	chaine = chaine + " " + str(bus.read_byte_data(ACCEL_ADRESS,OUTX_L_XL))
	chaine = chaine + " " + str(bus.read_byte_data(ACCEL_ADRESS,OUTX_H_XL))
	chaine = chaine + " " + str(bus.read_byte_data(ACCEL_ADRESS,OUTY_L_XL))
	chaine = chaine + " " + str(bus.read_byte_data(ACCEL_ADRESS,OUTY_H_XL))
	chaine = chaine + " " + str(bus.read_byte_data(ACCEL_ADRESS,OUTZ_L_XL))
	chaine = chaine + " " + str(bus.read_byte_data(ACCEL_ADRESS,OUTZ_H_XL))
	f.write(chaine+"\n")
	#time.sleep(0.01)

ard.send_arduino_cmd_motor(arduino, 0, 0)


print("execution Completed")
