from threading import Thread
from encoders_driver_py3 import Encoders
import arduino_driver_py3 as ardudrv

class Speed_driver(Thread):

	def __init__(self):
		Thread.__init__(self)

		self.motor_serial, data = ardudrv.init_arduino_line()

		self.encoders = Encoders()
		self.encoders.start()

		self.command_l = 0
		self.command_r = 0

		self.desired_speed_l = 0
		self.desired_speed_r = 0

		self.end = False

	def set_speed(self, l, r):  # l et r en ticks/sec

		self.desired_speed_l = l
		self.desired_speed_r = r

		self.command_l = 80
		self.command_r = 80

	def run(self):

		while not self.end:
			errL = self.desired_speed_l - self.encoders.speed_left
			errR = self.desired_speed_r - self.encoders.speed_right
			
			kp = 0.2
			self.command_l = self.command_l + kp*errL
			self.command_r = self.command_r + kp*errR

			if self.command_l>255:
				self.command_l=255
			if self.command_r>255:
				self.command_r=255
			if self.command_l<0:
				self.command_l=0
			if self.command_r<0:
				self.command_r=0
			
			ardudrv.send_arduino_cmd_motor(self.motor_serial, self.command_l, self.command_r)