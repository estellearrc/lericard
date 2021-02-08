"""Acquisition de données de la boussole pour analyse"""

import smbus
import time

bus = smbus.SMBus(1)   
DEVICE_ADDRESS = 0x1e  
WHO_AM_I = 0x0F
print(bus.read_byte_data(DEVICE_ADDRESS,WHO_AM_I))
dic_reg = {
"CTRL_REG1": 0x20,     
"CTRL_REG2": 0x21,
"CTRL_REG3": 0x22,
"CTRL_REG4": 0x23,
"CTRL_REG5": 0x24
}

bus.write_byte_data(DEVICE_ADDRESS, dic_reg["CTRL_REG3"],0b00000000)
bus.write_byte_data(DEVICE_ADDRESS, dic_reg["CTRL_REG1"],0b00010000)

for key in dic_reg:
	print(str(key)+ " " +str(bus.read_byte_data(DEVICE_ADDRESS, dic_reg[key])))


OUT_X_L = 0x28
OUT_X_H = 0x29
OUT_Y_L = 0x2A
OUT_Y_H = 0x2B
OUT_Z_L = 0x2C
OUT_Z_H = 0x2D


dic_val_bouss = {
"OUT_X_L": 0,
"OUT_X_H": 0,
"OUT_Y_L": 0,
"OUT_Y_H": 0,
"OUT_Z_L": 0,
"OUT_Z_H": 0}



x,y,z = 0,0,0
f = open("Datasheet.txt","w")
#for key in dic_val_bouss:
#	chaine = chaine +" "+ key
#f.write(chaine+"\n")
for i in range(500):
	chaine = ""
	X_L = bus.read_byte_data(DEVICE_ADDRESS, OUT_X_L)
	X_H = bus.read_byte_data(DEVICE_ADDRESS, OUT_X_H)
	Y_L = bus.read_byte_data(DEVICE_ADDRESS, OUT_Y_L)
	Y_H = bus.read_byte_data(DEVICE_ADDRESS, OUT_Y_H)
	Z_L = bus.read_byte_data(DEVICE_ADDRESS, OUT_Z_L)
	Z_H = bus.read_byte_data(DEVICE_ADDRESS, OUT_Z_H)
	f.write(str(X_L) + " " + str(X_H) + " " + str(Y_L) + " " + str(Y_H) + " " + str(Z_L) + " " + str(Z_H) +"\n")
	time.sleep(0.1)


print("Execution completed")
