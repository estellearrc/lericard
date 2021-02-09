import time
import fsm
import numpy as np
import sys
import Boat


remaining_points = []


def doWait():
    print("Waiting 1 sec ...")
    time.sleep(1)
    return "go"


def doMainMenu():
    print(" Do a triangle [1] \n Leave [2]")
    a = input()
    try:
        int(a)
    except:
        print("Non reconnu")
        event = 'wait'
        return event
    if int(a) == 1:
        event = 'triangle'
        print("Initializing triangle mode ...")
        return event
    elif int(a) == 2:
        event = 'stop'
        return event
    else:
        print("Non reconnu")
        event = 'wait'
        return event


def doTriangleInitialize():
    global remaining_points
    print("Points to follow : ")
    x_1 = 48.199482
    y_1 = -3.014891
    x_2 = 48.199295
    y_2 = -3.016188
    x_3 = 48.200187
    y_3 = -3.015764
    X1 = boat.gps.convert_to_cart_coord(x_1, y_1)
    X2 = boat.gps.convert_to_cart_coord(x_2, y_2)
    X3 = boat.gps.convert_to_cart_coord(x_3, y_3)
    print("A : x=", x_1, " y=", y_1)
    print("B : x=", x_2, " y=", y_2)
    print("C : x=", x_3, " y=", y_3)
    remaining_points = [(X1[0, 0], X1[1, 0]), (X2[0, 0], X2[1, 0]),
                        (X3[0, 0], X3[1, 0]), (X1[0, 0], X1[1, 0])]
    return "go"


def doFollowHeading():
    global remaining_points
    if len(remaining_points) > 0:
        x_target, y_target = remaining_points.pop(0)
        target_point = np.array([[x_target], [y_target]])
        print("Going to point x=", x_target, " y=", y_target)
        boat.follow_heading(target_point, 120,
                            boat.reach_point, target_point)
        #boat.follow_line(self, pointA, pointB, 80, 120)
        # boat.follow_line_potential(target_point)
        event = "go"
    else:
        print("End of the triangle ...")
        event = "stop"
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
