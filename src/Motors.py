import drivers.arduino_driver_py3 as ardudrv
from time import time, sleep
import numpy as np
from Tools import init_motors, set_motor_speed, stop_motors
from drivers.encoders_driver_py3 import Encoders


def sawtooth(x):
    return (x+np.pi) % (2*np.pi)-np.pi


class Motors:
    def __init__(self):
        self.serial_arduino, self.data_arduino = ardudrv.init_arduino_line()
        print("data:", self.data_arduino[0:-1])
        print("... done")
        print("get status ...")
        timeout = 1.0
        data_arduino = ardudrv.get_arduino_cmd_motor(
            self.serial_arduino, timeout)
        print("data:", data_arduino[0:-1])
        print("... done")
        self.encoders = Encoders()
        self.encoders.start()

    def command(self, speedl, speedr):
        """ compute command for left and right motor to reach speed0 in ticks per seconds"""
        cmdl = speedl
        cmdr = speedr

        speed_L = self.encoders.speed_left
        speed_R = self.encoders.speed_right

        errL = speedl-speed_L
        errR = speedr-speed_R
        kp = 0.1
        cmdl = cmdl + kp*errL
        cmdr = cmdr + kp*errR
        if cmdl > 255:
            cmdl = 255
        if cmdr > 255:
            cmdr = 255
        if cmdl < 0:
            cmdl = 0
        if cmdr < 0:
            cmdr = 0

        print("speed_L =", speed_L, "speed_R =",
              speed_R, "cmd =", [int(cmdl), int(cmdr)])
        self.set_pwm(cmdl, cmdr)

    def set_pwm(self, cmdl, cmdr):
        ardudrv.send_arduino_cmd_motor(self.serial_arduino, cmdl, cmdr)

    def stop(self):
        ardudrv.send_arduino_cmd_motor(self.serial_arduino, 0, 0)


def test():
    """ Try command motors """
    motors = Motors()
    # while True:
    #     motors.command(100, 50)
    #     time.sleep(1)
    #     motors.stop()
    #     motors.command(50, 100)
    #     time.sleep(1)
    #     motors.stop()
    speed0 = int(input("Speed ? --> "))
    t0 = time()
    while time() - t0 < 20:
        motors.command(speed0, speed0)
        sleep(0.3)

    motors.stop()
    motors.encoders.close()


if __name__ == "__main__":
    test()
