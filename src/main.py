import time
import fsm
import numpy as np
import sys
import Boat
from GPS import convert_DDmm_to_rad


remaining_points = []


def doWait():
    print("Waiting 1 sec ...")
    time.sleep(1)
    return "go"


def doMainMenu():
    print(" Go North [1] \n Do a triangle [2] \n Leave [3]")
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
    else:
        print("Unknown command")
        event = 'wait'
        return event


def doTriangle():
    global remaining_points
    print("Points to follow : ")
    x_1 = 48.199482
    y_1 = -3.014891
    x_2 = 48.199295
    y_2 = -3.016188
    x_3 = 48.200187
    y_3 = -3.015764
    x_1, y_1 = convert_DDmm_to_rad(x_1, y_1)
    x_2, y_2 = convert_DDmm_to_rad(x_2, y_2)
    x_3, y_3 = convert_DDmm_to_rad(x_3, y_3)
    print("A : x=", x_1, " y=", y_1)
    print("B : x=", x_2, " y=", y_2)
    print("C : x=", x_3, " y=", y_3)
    remaining_points = [(x_1, y_1), (x_2, y_2), (x_3, y_3), (x_1, y_1)]
    x_target, y_target = remaining_points.pop(0)
    target_point = np.array([[x_target], [y_target]])
    print("Going to point x=", x_target, " y=", y_target)
    X = boat.compass.read_sensor_values().flatten().reshape((3, 1))

    while len(remaining_points) > 0:
        # Nav block
        heading = boat.compass.compute_heading(X[0, 0], X[1, 0])
        if boat.reach_point(target_point):
            x_target, y_target = remaining_points.pop(0)
            target_point = np.array([[x_target], [y_target]])
            print("Going to point x=", x_target, " y=", y_target)

        # Guide block
        heading_obj = boat.compute_heading(target_point)
        print("obj : ", heading_obj)
        print("heading : ", heading)

        v_obj = 120

        # Control block
        u_L, u_R = boat.follow_heading(heading, heading_obj, v_obj)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        X = boat.compass.read_sensor_values().flatten().reshape((3, 1))

    print("End of the triangle ...")
    return "stop"


def doGoNorth():
    X = boat.compass.read_sensor_values().flatten().reshape((3, 1))
    t0 = time.time()
    t = time.time()
    while t - t0 < 1000:
        # Nav block
        heading = boat.compass.compute_heading(X[0, 0], X[1, 0])

        # Guide block
        heading_obj = np.pi/2
        v_obj = 160
        t = time.time()

        # Control block
        u_L, u_R = boat.follow_heading(heading, heading_obj, v_obj)

        # DDBoat command
        boat.motors.command(u_L, u_R)
        # boat.motors.command(40, 40)
        X = boat.compass.read_sensor_values().flatten().reshape((3, 1))

        with open("test_heading_following.csv", "a") as f:
            f.write(str(t) + "," + str(heading) + "," +
                    str(u_L) + "," + str(u_R) + "\n")

    event = "stop"
    print("Stop following North")
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
        try:
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
        except:
            print("I'm coming home bitches!")
            # boat = Boat.Boat()
            boat.back_to_home()
            boat.stop()
else:
    boat = Boat.Boat()
