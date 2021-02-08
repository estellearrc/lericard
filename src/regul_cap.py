import numpy as np
import arduino_driver_py3 as ard
from filter import low_pass_filter
from tst_compass import retrieve_compass_values

# # Calibrage de la boussole
# x1 = np.array([[864.02],[-4670.3],[5298.5]])
# x_1 = np.array([[7030.86],[-4034.02],[4805.73]])
# x2 = np.array([[4137.86],[-7385.4],[5082.37]])
# x3 = np.array([[4066.86],[-4540.8],[1934.98]])

# b = -1/2*(x1 + x_1)
# beta = 46000
# v1 = 1/beta*(x1+b)
# v2 = 1/beta*(x2 + b)
# v3 = 1/beta*(x3 + b)
# M  = np.zeros([3,3])
# M[:,0]=v1.flatten()
# M[:,1]=v2.flatten()
# M[:,2]=v3.flatten()

K = 1.15

# arduino = ard.init_arduino_line()[0]


def sawtooth(x):
    return (x+2*np.pi) % (2*np.pi)-np.pi


def regulCap(cap_bar, vmin, vmax, bouss):
    """cap_bar consigne en cap
    vmin vitesse de croisière
    vmax vitesse maximale en régulation
    bouss = [X_H, X_L, ...]"""
    # vx = bouss[0] + 256*bouss[1]
    # if vx > 32767:
    #     vx -= 65536
    # vy = bouss[2] + 256*bouss[3]
    # if vy > 32767:
    #     vy -= 65536
    # vz = bouss[4] + 256*bouss[5]
    # if vz > 32767:
    #     vz -= 65536
    vx, vy, vz = retrieve_compass_values()
    X = np.array([vx, vy, vz]).reshape((3, 1))

    P = np.linalg.inv(M)@(X + b)
    cap = np.arctan2(P[0, 0], P[1, 0])

    e = sawtooth(cap - cap_bar)

    v = ((abs(e)*(vmax - vmin)) / np.pi) + vmin

    u1 = int(0.5*v*(1 + K*e))
    u2 = int(0.5*v*(1 - K*e))
    ard.send_arduino_cmd_motor(arduino, u1, u2)
