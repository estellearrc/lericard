import time
import encoders_driver_py3 as encodrv

encodrv.set_baudrate(baudrate=115200)

serial_encoders = encodrv.init_line()

while True:
    encodrv.sync(serial_encoders)
    while True:
        sync,data_encoders = encodrv.read_packet(serial_encoders,debug=True)
        if not sync:
            break
