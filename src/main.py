import time
import fsm
import numpy as np
import sys
import Boat
import Logs
from GPS import convert_longlat_to_rad


remaining_points = []


def doWait():
    print("Waiting 1 sec ...")
    time.sleep(1)
    return "go"


def doMainMenu():
    print(
        " Go North [1] \n Do a triangle [2] \n Going to a point in duration [3] \n Leave [4]")
    a = input()
    try:
        int(a)
    except:
        print("Unknown command")
        event = 'wait'
        return event
    if int(a) == 1:
        event = 'north'
        print("Going North ...")
        return event
    elif int(a) == 2:
        event = 'triangle'
        print("Initializing triangle mode ...")
        return event
    elif int(a) == 3:
        event = 'timedline'
        return event
    elif int(a) == 4:
        event = 'stop'
        return event
    else:
        print("Unknown command")
        event = 'wait'
        return event


def doTriangle():
    logs = Logs.Logs('triangle', 't', 'u_L', 'u_R', 'heading', 'heading_obj', 'pos_x', 'pos_y')

    global remaining_points
    print("Points to follow : ")
    # x_1 = 48.199482
    # y_1 = -3.014891
    x_1 = 48.198971
    y_1 = -3.014399
    x_2 = 48.199295
    y_2 = -3.016188
    x_3 = 48.200187
    y_3 = -3.015764
    x_1, y_1 = convert_longlat_to_rad(x_1, y_1)
    x_1, y_1 = boat.gps.convert_rad_to_cart(x_1, y_1)
    x_2, y_2 = convert_longlat_to_rad(x_2, y_2)
    x_2, y_2 = boat.gps.convert_rad_to_cart(x_2, y_2)
    x_3, y_3 = convert_longlat_to_rad(x_3, y_3)
    x_3, y_3 = boat.gps.convert_rad_to_cart(x_3, y_3)
    print("A : x=", x_1, " y=", y_1)
    print("B : x=", x_2, " y=", y_2)
    print("C : x=", x_3, " y=", y_3)
    remaining_points = [(x_1, y_1), (x_2, y_2), (x_3, y_3), (x_1, y_1)]
    x_target, y_target = remaining_points.pop(0)
    target_point = np.array([[x_target], [y_target]])
    print("Going to point x=", x_target, " y=", y_target)
    mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))

    while len(remaining_points) > 0:
        # Nav block
        heading = boat.compass.compute_heading(mag_field[0, 0], mag_field[1, 0])
        logs.update('heading', heading)
        if boat.reach_point(target_point):
            x_target, y_target = remaining_points.pop(0)
            target_point = np.array([[x_target], [y_target]])
            print("Going to point x=", x_target, " y=", y_target)

        # Guide block
        data = boat.gps.read_sensor_values()
        actual_pos = boat.gps.convert_to_cart_coord(data)[0:2]
        t = boat.gps.convert_to_cart_coord(data)[2, 0]
        logs.update('t', t)
        logs.update('pos_x', actual_pos[0, 0])
        logs.update('pos_y', actual_pos[1, 0])
        
        heading_obj = boat.compute_heading(target_point, actual_pos)
        logs.update('heading_obj', heading_obj)
        print("obj : ", heading_obj)
        print("heading : ", heading)

        v_obj = 40

        # Control block
        u_L, u_R = boat.follow_heading(heading, heading_obj, v_obj)
        logs.update('u_L', u_L)
        logs.update('u_R', u_R)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))

        logs.write_data()
        
    print("End of the triangle ...")
    return "stop"


def doGoNorth():
    logs = Logs.Logs('goNorth', 'u_L', 'u_R', 'heading')
    mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))
    t0 = time.time()
    t = time.time()
    while t - t0 < 1000:
        # Nav block
        heading = boat.compass.compute_heading(mag_field[0, 0], mag_field[1, 0])
        logs.update("heading", heading)

        # Guide block
        heading_obj = 0
        v_obj = 40
        t = time.time()

        # Control block
        u_L, u_R = boat.follow_heading(heading, heading_obj, v_obj)
        logs.update("u_L", u_L)
        logs.update("u_R", u_R)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        # boat.motors.command(40, 40)
        mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))

        logs.write_data()

    event = "stop"
    print("Stop following North")
    return event


def doGoPointInTime():
    print("Please enter the desired time (in seconds) :")
    input(t_total)
    try:
        int(t_total)
    except:
        print("Unknown command")
        return 'wait'

    x_1 = 48.199482
    y_1 = -3.014891
    t, target_point = boat.gps.convert_to_cart_coord(x_1, y_1)
    mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))

    while boat.reach_point(target_point) == False:
        # Nav block
        heading = boat.compass.compute_heading(
            mag_field[0, 0], mag_field[1, 0])

        # Guide block
        t = time.time()
        heading_obj, v_obj = boat.follow_line_potential(
            a, target_point, t, t0, p)

        # Control block
        u_L, u_R = boat.follow_heading(heading, heading_obj, v_obj)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))
        data = boat.GPS.read_sensor_values()
        p = boat.GPS.convert_to_coordinates(data)

    event = "stop"
    print("Arrived !")
    return event


def doStop():
    print("Stopping ...")
    time.sleep(0.1)
    event = None
    return event


if __name__ == "__main__":
    f = fsm.fsm()
    f.load_fsm_from_file("fsm_boat_cmd.txt")
    run = True
    while run:
        #try:
        funct = f.run()
        if f.curState != f.endState:
            newEvent = funct()
            if newEvent is None:
                break
            else:
                f.set_event(newEvent)
        else:
            funct()
            run = False
        #except:
         #   print("I'm coming home bitches!")
            # boat = Boat.Boat()
          #  boat.back_to_home()
           # boat.stop()
else:
    boat = Boat.Boat()
