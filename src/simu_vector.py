from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

p1, p2 = 0.714, 0.1


# Point triangle
a = array([[-3], [1]])
b = array([[9], [-5]])
c = array([[4], [5]])
triangle_points = [a, b, c]

t0 = 0
dt = 0.01
s = 10
ax = init_figure(-s, s, -s, s)


def lissajou(t):
    A = array([[0], [0]])
    B = array([[1], [1]])
    delta = pi / 2
    x = A * sin(a * t + delta)
    y = B * sin(b * t)
    return array([[x], [y]])


# Field function
def f1(x1, x2):
    # calcul du champ de vecteurs attractif/répulsif dans l'environnement
    d = (b - a) / norm(b - a)
    n = array([[-d[1, 0]], [d[0, 0]]])  # normal vector of the line ab
    f1 = -(n @ n.T)[0, 0] * (x1 - a[0, 0]) - (x1 - phat[0, 0])
    f2 = -(n @ n.T)[1, 0] * (x2 - a[1, 0]) - (x2 - phat[1, 0])
    return f1, f2


# State function
def fsimu(X, u):
    x = X[2, 0] * cos(X[3, 0])
    y = X[2, 0] * sin(X[3, 0])
    v = p1 * u[0, 0] - p2 * X[2, 0] ** 2
    theta = u[1, 0]
    return array([[x], [y], [v], [theta]])


def reach_point(x, point):
    """Return false when a certain point has been reached
    point is a 2d-array"""
    return norm(point - x[0:2, 0]) <= 3


def follow_line_potential(n, p, phat, v0, i):
    """
    Return motos commands from a direction vector
    a: departure point
    b: target point
    t: current time
    t0: departure time
    p: boat postion array([[x],[y]])
    i: index of triangle_points only for simulation
    """
    p = p[0:3, 0]  # Specifique simu
    # vector field
    w = -n @ n.T @ (p - triangle_points[i]) + v0 + 1 * (phat - p)
    # w =  v0 + 0.1*(phat-p)
    v_bar = norm(w)
    theta_bar = arctan2(w[1, 0], w[0, 0])
    return theta_bar, v_bar


X = array([[-3], [0], [0], [pi / 2]])  # State vector x,y,v,theta
X1 = []
X2 = []
X3 = []
X4 = []
T = []
t = t0
i = 0  # indice departure point

while t < 20:
    # Guide bloc
    d = (triangle_points[(i + 1) % 2] - triangle_points[i % 2]) / norm(
        triangle_points[(i + 1) % 2] - triangle_points[i % 2]
    )  # vecteur unitaire
    # d = (b - a) / norm(b - a)  # vecteur unitaire
    v0 = 10 * d  # Arbitraire -> a adapter
    n = array([[-d[1, 0]], [d[0, 0]]])  # normal vector of the line ab
    phat = triangle_points[i % 2] + v0 * (t - t0)  # moving attractive point t-t0
    theta_bar, v_bar = follow_line_potential(n, X, phat, v0, i % 2)
    print("thetabar: ", theta_bar)
    # Commande prop
    u = array(
        [[3 * (v_bar - X[2, 0])], [10 * sawtooth(theta_bar - X[3, 0])]]
    )  # velocity, theta
    # Euler
    dX = fsimu(X, u)
    X = X + dX * dt
    t = t + dt
    print("theta: ", X[3, 0])
    # Draw field
    X1.append(X[0])
    X2.append(X[1])
    X3.append(X[2])
    X4.append(X[3])
    T.append(t)
    pause(0.001)
    cla()
    draw_field(ax, f1, -s, s, -s, s, 0.4)
    draw_disk(a, 0.3, ax, "magenta")
    draw_disk(b, 0.3, ax, "magenta")
    draw_disk(c, 0.3, ax, "magenta")
    draw_disk(phat, 0.3, ax, "red")
    draw_tank(X[[0, 1, 3]], r=0.25, w=0.5)

    if reach_point(X, triangle_points[(i + 1) % 2]):
        print("reach point !")
        i += 1
