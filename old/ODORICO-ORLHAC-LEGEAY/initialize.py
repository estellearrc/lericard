import smbus

"""Permet d'initialiser les capteurs avec les paramètres choisis (voir la doc)"""

bus = smbus.SMBus(1)

COMPASS_ADDRESS = 0x1e  

bus.write_byte_data(COMPASS_ADDRESS, 0x22,0b00000000)
bus.write_byte_data(COMPASS_ADDRESS, 0x20,0b00010000)


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

#basicoffset
bus.write_byte_data(ACCEL_ADRESS,FIFO_CTRL1, 0b00000000)
bus.write_byte_data(ACCEL_ADRESS,FIFO_CTRL2, 0b00000000)
bus.write_byte_data(ACCEL_ADRESS,FIFO_CTRL3, 0b00000000)
bus.write_byte_data(ACCEL_ADRESS,FIFO_CTRL4, 0b00000000)
bus.write_byte_data(ACCEL_ADRESS,FIFO_CTRL5, 0b00000000)
#capteurregl

bus.write_byte_data(ACCEL_ADRESS,CTRL1_XL, 0b01010111)
bus.write_byte_data(ACCEL_ADRESS,CTRL2_G, 0b01010000)
bus.write_byte_data(ACCEL_ADRESS,CTRL3_C, 0b00000100)
bus.write_byte_data(ACCEL_ADRESS,CTRL4_C, 0b00000000)
bus.write_byte_data(ACCEL_ADRESS,CTRL5_C, 0b01100100)
bus.write_byte_data(ACCEL_ADRESS,CTRL6_C, 0b00100000)
bus.write_byte_data(ACCEL_ADRESS,CTRL7_G, 0b00000000)
bus.write_byte_data(ACCEL_ADRESS,CTRL8_XL, 0b10100101)
bus.write_byte_data(ACCEL_ADRESS,CTRL9_XL, 0b00111000)
bus.write_byte_data(ACCEL_ADRESS,CTRL10_C, 0b00111101)
