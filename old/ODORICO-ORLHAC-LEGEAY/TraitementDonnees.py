from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt


fich = "Datasheet.txt"


M = np.loadtxt(fich)
print(M)

X = []
Y = []
Z = []

for i in range(len(M)):
	vx = M[i, 0] + 256*M[i, 1]
	if vx > 32767:
		vx -= 65536
	vy = M[i, 2] + 256*M[i, 3]
	if vy > 32767:
		vy -= 65536
	vz = M[i, 4] + 256*M[i, 5]
	if vz > 32767:
		vz -= 65536
	X.append(vx)
	Y.append(vy)
	Z.append(vz)

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

ax.scatter(X, Y, Z)
ax.scatter(0, 0, 0, '.r')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')


# Translation
p1 = np.mean(X) # 0
p2 = np.mean(Y) #-1200
p3 = np.mean(Z) #4200

# Dilatation
p4 = 2/(np.max(X)-np.min(X))
p5 = 2/(np.max(Y)-np.min(Y))
p6 = 2/(np.max(Z)-np.min(Z))

# Rotation
p7 = 0
p8 = 0
p9 = 0

nX = []
nY = []
nZ = []

for i in range(len(X)):
	P = np.array([X[i], Y[i], Z[i]]).reshape((3, 1))
	nP = np.array([[p4, p7, p9], [p7, p5, p8], [p9, p8, p6]])@(P - np.array([p1, p2, p3]).reshape((3, 1)))
	nX.append(nP[0, 0])
	nY.append(nP[1, 0])
	nZ.append(nP[2, 0])

fig = plt.figure(2)
ax = fig.add_subplot(111, projection='3d')

ax.scatter(nX, nY, nZ)
ax.scatter(0, 0, 0, '.r')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')


print("p1 :", p1, "\n p2 :", p2, "\n p3 :", p3, "\n p4 :", p4, "\n p5 :", p5, "\n p6 :", p6, "\n p7 :", p7, "\n p8 :", p8, "\n p9 :", p9)
plt.show()


