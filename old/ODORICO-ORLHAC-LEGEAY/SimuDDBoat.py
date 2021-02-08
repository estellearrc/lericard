from roblib import *

"""Code de simulation du DDBoat"""

p1 = pi/(2*255)
p2 = 2/510
p3 = 1.1
p4 = 1
K = 0.8
# Limites de la piscine
xmin = 0
xmax = 20
ymin = 0
ymax = 10


X0=[(3*xmax-xmin)/4,(3*ymax-ymin)/4, pi/2, 0]


def fsimu(X,u):
    x1d = X[3]*cos(X[2])
    x2d = X[3]*sin(X[2])
    x3d = p1*(u[0] - u[1])
    x4d = p2*(u[0] + u[1]) - p3*X[3]*abs(X[3])

    Y = np.array([x1d, x2d, x3d, x4d])
    return Y

psibar = 0

def commandecap(psibar, psi, vmin, vmax):
    e = sawtooth(psi - psibar)
    v = ((abs(e)*(vmax - vmin)) / np.pi) + vmin
    u1 = int(0.5*v*(1 - K*e))
    u2 = int(0.5*v*(1 + K*e))
    return [u1, u2]

q1 = 0

def fcorde(alpha,beta,u):
    return(np.array([q1*sin(beta)*(u[0]+u[1]),p1*(commandecap(psibar,alpha+beta))-q1*sin(beta)*(u[0]+u[1])]))


fig = figure(0)
ax=fig.add_subplot(111)

ax.set_xlim(xmin-5, xmax+5)
ax.set_ylim(ymin-5, ymax+5)


X = X0
dt = 0.1
t = 0

X1 = []
X2 = []
X3 = []
X4 = []
T = []
on_edge = False
follow_edge_angle = (13*pi)/12
collision = False

while t < 300:
    X1.append(X[0])
    X2.append(X[1])
    X3.append(X[2])
    X4.append(X[3])
    T.append(t)
    pause(0.001)
    cla()
    draw_segment(array([[xmin],[ymin]]),array([[xmin],[ymax]]))
    draw_segment(array([[xmin],[ymin]]),array([[xmax],[ymin]]))
    draw_segment(array([[xmax],[ymin]]),array([[xmax],[ymax]]))
    draw_segment(array([[xmin],[ymax]]),array([[xmax],[ymax]]))

    if on_edge:
        u = commandecap(follow_edge_angle, X[2], 150, 300)
    else:
        u = commandecap(7*pi/8, X[2], 150, 300)

    dX = fsimu(X, u)
    X = X + dX*dt
    t = t + dt

    if (X[0] < xmin+0.2 or X[0] > xmax-0.2):
        X[3] = -1.2*X[3]
        collision = True
    if (X[1] < ymin+ 0.2 or X[1] > ymax-0.2):
        X[3] = -1.2*X[3]
        collision = True

    if collision:
        collision = False
        if on_edge == False:
            on_edge = True
        else:
            follow_edge_angle = (follow_edge_angle + np.pi/2 - np.pi)%(2*np.pi) - np.pi

    draw_tank(array([[X[0]],[X[1]],[X[2]]]),r=0.25,w=0.5)




figure(1)
plot(X1, X2, '.b', label="pos")
legend()
figure(2)
plot(T, np.array(X3)%(2*pi), '.b', label="theta(t)")
legend()
figure(3)
plot(T, X4, '.b', label="v(t)")
legend()
show()

