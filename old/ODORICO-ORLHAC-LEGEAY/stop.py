import arduino_driver_py3 as ard

"""Arret d'urgence"""

arduino = ard.init_arduino_line()[0]
ard.send_arduino_cmd_motor(arduino, 0, 0)
