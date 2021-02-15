from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py

p1, p2 = 0.714, 0.1
#p1, p2 = 2, 0.1


# Point triangle
a = array([[4], [5]])
b = array([[9], [-5]])
c = array([[-3], [1]])
triangle_points = [a, b, c, a]

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


def reach_point(p, target):
    """Return false when a certain point has been reached
    point is a 2d-array"""
    return norm(p - target) <= 1
    
    
def follow_line_potential(a, b, p, phat, v0):
    """
    Return motos commands from a direction vector
    a: departure point
    b: target point
    p: boat postion array([[x],[y]])
    phat: moving attractive point a + v0*(t-t0)
    v0: vitesse du point atractif   
    """
    d = (b-a)/norm(b-a)
    n = np.array([[-d[1, 0]], [d[0, 0]]])  # normal vector of the line ab
    # vector field
    w = -n @ n.T @ (p-a) + v0 + (phat-p)
    v_bar = norm(w)
    theta_bar = np.arctan2(w[1, 0], w[0, 0])
    return theta_bar, v_bar


def control(heading, heading_obj, v_obj):
    """Returns motors commands from an heading to follow"""

    # increase the range of the bearing angle
    e = 0.35*sawtooth(heading_obj - heading)
    print("e = ", e)
    M = np.array([[1, -1], [1, 1]])
    b = np.array([[2*e], [1]])

    M_1 = np.linalg.pinv(M)  # resolution of the system
    u = M_1.dot(b)  # command motor array
    
    u_left = v_obj*u[0, 0]
    u_right = v_obj*u[1, 0]  # command right motor

    return u_left, u_right


X = array([[0], [0], [0], [pi / 2]])  # State vector x,y,v,theta
X1 = []
X2 = []
X3 = []
X4 = []
T = []
t = t0
i = 0  # indice departure point

while i < 3:
    p = array([[X[0, 0]], [X[1, 0]]])
    v0 = 5 * (b-a)/norm(b-a)
    
    # Guide bloc
    a = triangle_points[i]
    b = triangle_points[i+1]
    phat = a + (t - t0) * v0
    
    theta_bar, v_bar = follow_line_potential(a, b, p, phat, v0)
    
    
    # Control bloc
    u_left, u_right = control(X[3, 0], theta_bar, v_bar)
    u = array([[u_left + u_right], [u_left - u_right]])
    
    
    # DDBoat bloc
    # Euler
    dX = fsimu(X, u)
    X = X + dX * dt
    t = t + dt
    
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

    if reach_point(p, triangle_points[i+1]):
        print("reach point !")
        i += 1
        t0 = t
