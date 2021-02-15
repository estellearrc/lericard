from threading import Thread
import numpy as np
import time
from smbus import SMBus
from Tools import init_compass, get_heading


class Compass(Thread):

	def __init__(self):
		Thread.__init__(self)
		self.heading = 0
		self.end = False

	def run(self):
		bus = init_compass()

		while not self.end:
			self.heading = get_heading(bus)

			time.sleep(0.05)

	def close(self):
		self.end = True