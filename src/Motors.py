import drivers.arduino_driver_py3 as ardudrv


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
