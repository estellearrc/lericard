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


def doInitSpeed():
    print("Going forward ...")
    boat.motors.command(100, 100)

    while boat.gps.read_sensor_values()[1] != 'a':
        time.sleep(1)
    print('GPS ready ! ')
    time.sleep(4)
    boat.motors.command(50, 50)
    return 'go'


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
    logs = Logs.Logs('triangle', 't', 'u_L', 'u_R', 'heading_gps',
                     'heading_obj', 'pos_x', 'pos_y')

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
        heading_compass = boat.compass.compute_heading(mag_field[0, 0], mag_field[1, 0])
        data = boat.gps.read_sensor_values()
        state_vector = boat.gps.convert_to_cart_coord(data)
        actual_pos = np.array([[state_vector[1, 0]], [state_vector[2, 0]]])
        heading_gps = state_vector[4, 0]
        t = state_vector[0, 0]

        logs.update('heading_gps', heading_gps)
        if boat.reach_point(target_point):
            x_target, y_target = remaining_points.pop(0)
            target_point = np.array([[x_target], [y_target]])
            print("Going to point x=", x_target, " y=", y_target)

        # Guide block
        logs.update('t', t)
        logs.update('pos_x', actual_pos[0, 0])
        logs.update('pos_y', actual_pos[1, 0])

        heading_obj = boat.compute_heading(target_point, actual_pos)
        logs.update('heading_obj', heading_obj)

        v_obj = 100

        # Control block
        u_L, u_R = boat.follow_heading(heading_gps, heading_compass, heading_obj, v_obj)
        logs.update('u_L', u_L)
        logs.update('u_R', u_R)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))

        logs.write_data()

    print("End of the triangle ...")
    return "stop"


def doGoNorth():
    logs = Logs.Logs('goNorth', 'u_L', 'u_R', 'heading_gps')
    mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))
    data = boat.gps.read_sensor_values()
    state_vector = boat.gps.convert_to_cart_coord(data)
    actual_pos = np.array([[state_vector[1, 0]], [state_vector[2, 0]]])
    heading_gps = state_vector[4, 0]
    t = state_vector[0, 0]

    t0 = t
    while t - t0 < 120:
        print(t)
        print(t-t0)
        # Nav block
        heading_compass = boat.compass.compute_heading(mag_field[0, 0], mag_field[1, 0])
        data = boat.gps.read_sensor_values()
        state_vector = boat.gps.convert_to_cart_coord(data)
        actual_pos = np.array([[state_vector[1, 0]], [state_vector[2, 0]]])
        heading_gps = state_vector[4, 0]
        t = state_vector[0, 0]
        logs.update("heading_gps", heading_gps)

        # Guide block
        heading_obj = 0
        v_obj = 100

        # Control block
        u_L, u_R = boat.follow_heading(heading_gps, heading_compass, heading_obj, v_obj)
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
    logs = Logs.Logs('goPointInTime', 't', 'u_L', 'u_R', 'pos_x',
                     'pos_y', 'heading_gps', 'heading_obj', 'v', 'v_obj')

    x_1 = 48.199482
    y_1 = -3.014891
    data = boat.gps.read_sensor_values()
    state_vector = boat.gps.convert_to_cart_coord(data)
    p = np.array([[state_vector[1, 0]], [state_vector[2, 0]]])
    t = state_vector[0, 0]
    v = state_vector[3, 0]
    heading_gps = state_vector[4, 0]

    a = p[:, :]
    t0 = t
    v0 = 140  # A adapter
    mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))

    while boat.reach_point(target_point) == False:
        # Nav block
        heading_compass = boat.compass.compute_heading(
            mag_field[0, 0], mag_field[1, 0])
        logs.update("heading_gps", heading_gps)

        # Guide block
        phat = a + v0*(t-t0)
        heading_obj, v_obj = boat.follow_line_potential(
            a, target_point, p, phat, v0)
        logs.update("heading_obj", heading_obj)
        logs.update("v_obj", v_obj)

        # Control block
        u_L, u_R = boat.follow_heading(heading_gps, heading_compass, heading_obj, v_obj)
        logs.update("u_L", u_L)
        logs.update("u_R", u_R)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        mag_field = boat.compass.read_sensor_values().flatten().reshape((3, 1))
        data = gps.read_sensor_values()
        state_vector = gps.convert_to_cart_coord(data)
        p = np.array([[state_vector[1, 0]], [state_vector[2, 0]]])
        t = state_vector[0, 0]
        v = state_vector[3, 0]
        heading_gps = state_vector[4, 0]

        logs.update("t", t)
        logs.update("pos_x", p[0, 0])
        logs.update("pos_y", p[1, 0])
        logs.update("v", v)
        logs.write_data()

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
        # try:
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
        # except:
         #   print("I'm coming home bitches!")
            # boat = Boat.Boat()
          #  boat.back_to_home()
           # boat.stop()
else:
    boat = Boat.Boat()
