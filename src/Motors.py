import drivers.arduino_driver_py3 as ardudrv
import time
import numpy as np


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

    def command(self, cmdl, cmdr):
        ardudrv.send_arduino_cmd_motor(self.serial_arduino, cmdl, cmdr)

    def stop(self):
        ardudrv.send_arduino_cmd_motor(self.serial_arduino, 0, 0)

    def compute_command(self, e):
        # linearization loop
        M = np.array([[1, -1], [1, 1]])
        b = np.array([[sawtooth(e)], [1]])
        M_1 = np.linalg.pinv(M)  # resolution of the system
        u = M_1.dot(b)  # command motor array
        return u


def test():
    """ Try command motors """
    motors = Motors()
    motors.command(50, 50)
    time.sleep(10)
    motors.stop()


if __name__ == "__main__":
    test()
