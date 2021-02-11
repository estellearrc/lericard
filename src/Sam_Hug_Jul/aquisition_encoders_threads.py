from Tools import init_motors, set_motor_speed, stop_motors
from time import time, sleep
from Drivers.encoders_driver_py3 import Encoders

encoders = Encoders()
encoders.start()

serial = init_motors()

speed0 = int(input("Speed ? --> "))

cmdl = 50
cmdr = 50

t0 = time()
while time() - t0 < 20:
	speed_L = encoders.speed_left
	speed_R = encoders.speed_right

	errL = speed0-speed_L
	errR = speed0-speed_R
	kp = 0.1
	cmdl = cmdl + kp*errL
	cmdr = cmdr + kp*errR
	if cmdl>255:
		cmdl=255
	if cmdr>255:
		cmdr=255
	if cmdl<0:
		cmdl=0
	if cmdr<0:
		cmdr=0
	
	set_motor_speed(serial, cmdl, cmdr)
	
	print("speed_L =", speed_L, "speed_R =", speed_R, "cmd =", [int(cmdl), int(cmdr)])
	sleep(0.1)

stop_motors(serial)
encoders.close()