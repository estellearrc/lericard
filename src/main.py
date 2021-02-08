import time
import fsm
import time
import sys


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


def doTriangle():
    #boat.triangle()
    print("oui")
    event = "wait"
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
