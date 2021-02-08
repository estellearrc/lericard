import smbus
import time

bus = smbus.SMBus(1)
COMPASS_ADDRESS = 0x1e  

OUT_X_L = 0x28
OUT_X_H = 0x29
OUT_Y_L = 0x2A
OUT_Y_H = 0x2B
OUT_Z_L = 0x2C
OUT_Z_H = 0x2D

def read_compass():
	"""Renvoie les données de la boussole"""
	return [bus.read_byte_data(COMPASS_ADDRESS, OUT_X_L), bus.read_byte_data(COMPASS_ADDRESS, OUT_X_H),
	bus.read_byte_data(COMPASS_ADDRESS, OUT_Y_L), bus.read_byte_data(COMPASS_ADDRESS, OUT_Y_H), bus.read_byte_data(COMPASS_ADDRESS, OUT_Z_L),
	bus.read_byte_data(COMPASS_ADDRESS, OUT_Z_H)]


