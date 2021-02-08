import time
import sys
import arduino_driver_py3 as ardudrv

cmdl = 40
cmdr = 40
duration = 2.0
try:
    cmdl = int(sys.argv[1])
    cmdr = cmdl
except:
    pass
try:
    cmdr = int(sys.argv[2])
except:
    pass
try:
    duration = float(sys.argv[3])
except:
    pass


print ("init arduino ...")
serial_arduino, data_arduino = ardudrv.init_arduino_line()
print ("data:",data_arduino[0:-1])
print ("... done")

print ("get status ...")
timeout = 1.0
data_arduino = ardudrv.get_arduino_status(serial_arduino,timeout)
print ("data:",data_arduino[0:-1])
print ("... done")

print ("set motors to L=%d R=%d ..."%(cmdl,cmdr))
ardudrv.send_arduino_cmd_motor(serial_arduino,cmdl,cmdr)
print ("... done")

print ("get decoded data (debug) ...")
timeout = 1.0
data_arduino = ardudrv.get_arduino_decode_cmd (serial_arduino,timeout)
print ("data:",data_arduino[0:-1])
print ("... done")

print ("wait {:.1f} seconds ...".format(duration))
t0 = time.time()
while (time.time()-t0) < duration:
    data_arduino = ardudrv.get_arduino_status(serial_arduino,timeout)
    print ("status:",data_arduino[0:-1])
    data_arduino = ardudrv.get_arduino_cmd_motor(serial_arduino,timeout)
    print ("motors:",data_arduino[0:-1])
    data_arduino = ardudrv.get_arduino_rc_chan(serial_arduino,timeout)
    print ("RC ch1-4:",data_arduino[0:-1])
    time.sleep (0.5)
    ardudrv.send_arduino_cmd_motor(serial_arduino,cmdl,cmdr)

print ("set motors to L=0 R=0 ...")
cmdl = 0
cmdr = 0
ardudrv.send_arduino_cmd_motor(serial_arduino,cmdl,cmdr)
print ("... done")

print ("get decoded data (debug) ...")
timeout = 1.0
data_arduino = ardudrv.get_arduino_decode_cmd (serial_arduino,timeout)
print ("data:",data_arduino[0:-1])
print ("... done")
