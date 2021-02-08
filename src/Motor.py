import drivers.arduino_driver_py3 as ardudrv


class Motors:
    def __init__(self):
        self.serial_arduino, self.data_arduino = ardudrv.init_arduino_line()
        ACC_ADDRESS = 0x6b
        bus = smbus.SMBus(1)
