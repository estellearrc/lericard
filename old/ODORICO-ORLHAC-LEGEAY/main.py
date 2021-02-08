import compass
import regul_cap
import time
import arduino_driver_py3 as ard
import numpy as np
import accelerometer 
from collision import detect_collision
from filter import low_pass_filter

"""Code principal à executer"""


arduino = ard.init_arduino_line()[0]


def heading_follow(vmin, vmax, alpha):
	"""Suivre un cap"""
	for k in range(500):
		regul_cap.regulCap(alpha, 80, 120, compass.read_compass())
		time.sleep(0.1)


def cycle_stable(vmin, vmax, start_angle):
	"""Faire un cycle stable"""
	on_edge = False
	i = 0
	for k in range(6000):
		if on_edge == False:
			regul_cap.regulCap(start_angle, 80, 100, compass.read_compass())
		else:
			regul_cap.regulCap(follow_edge_angle, 80, 100, compass.read_compass())
		if detect_collision(accelerometer.read_accelerometer()):
			print("boom")
			ard.send_arduino_cmd_motor(arduino, 0, 0)
			time.sleep(1)
			if on_edge == False:
				on_edge = True
				follow_edge_angle = -(np.pi)/3
			else:
				if i%3 == 0:
					follow_edge_angle = -(np.pi)/3
				elif i%3 == 1:
					follow_edge_angle = -(2*np.pi)/3
				else:
					follow_edge_angle = np.pi/2
				follow_edge_angle = (follow_edge_angle + np.pi/6 - np.pi)%(2*np.pi) - np.pi
			i+=1
		time.sleep(0.04)


mode = int(input("Heading_follow {0} Cycle stable {1}"))
if mode == 0:
	vmin = int(input("vmin vitesse de croisière"))
	vmax = int(input("vmax vitesse maximale en régulation"))
	alpha = (np.pi*(int(input("Angle en degre"))-180))/180
	heading_follow(vmin,vmax,alpha)
elif mode == 1:
	vmin = int(input("vmin vitesse de croisière"))
	vmax = int(input("vmax vitesse maximale en régulation"))
	alpha = (np.pi*(int(input("Angle en degre"))-180))/180
	cycle_stable(vmin, vmax, alpha)

ard.send_arduino_cmd_motor(arduino, 0, 0)

