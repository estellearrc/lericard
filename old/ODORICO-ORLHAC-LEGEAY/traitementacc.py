import roblib
from numpy import array
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from numpy import array
import filter

nom = "Datasheetacc.txt"
X, Y, Z, T = [], [], [], []
f = open(nom,"r")
dt=0
i = 0
for line in f: 
	T.append(0.1*dt)
	dt += 1
	acq= f.readline().split(" ")
	X.append(int(acq[1])+int(acq[2])*255)
	if X[i] >32767:
		X[i] = X[i] - 65536
	Y.append(int(acq[3])+int(acq[4])*255)
	if Y[i] >32767:
		Y[i] = Y[i] - 65536
	number =""
	for elem in acq[6]:
		if elem.isdigit():
			number = number + elem
		acq[6] = number
	Z.append(int(acq[5])+int(acq[6])*255)
	if Z[i] >32767:
		Z[i] = Z[i] - 65536
	i += 1

X = array(X)
Y = array(Y)
Z = array(Z)

N = [(X[0]**2+Y[0]**2)**(1/2)]
for i in range(1, len(X)):
	N = filter.low_pass_filter(N, (X[i]**2+Y[i]**2)**(1/2))


plt.figure(1)
plt.plot(T,(X**2+Y**2)**(1/2), '.b')
plt.figure(2)
plt.plot(T, N, '.b')
plt.show()
