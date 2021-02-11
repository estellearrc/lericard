import numpy as np

rho = 6371000


def convert_DDmm_to_rad(lx, ly):
    DDx = int(lx/100)
    # North  / longitude in radian
    lx = np.pi*(DDx + (lx-DDx*100)/60) / 180
    DDy = int(ly/100)
    # West = -Est / latitude in radian
    ly = -np.pi*(DDy + (ly-DDy*100)/60) / 180
    return lx, ly


def convert_longlat_to_rad(lx, ly):
    lx = np.pi*lx/180
    ly = np.pi*ly/180
    return lx, ly


def convert_rad_to_cart(lx, ly):
    x_tilde = rho*np.cos(ly)*(lx-lx0)
    y_tilde = rho*(ly-ly0)
    return x_tilde, y_tilde


lx0, ly0 = convert_DDmm_to_rad(4811.9447, 300.8921)
x_0, y_0 = convert_rad_to_cart(lx0, ly0)
print(lx0, ly0)
print(x_0, y_0)
x_1 = 48.198971
y_1 = -3.014399
x_2 = 48.199295
y_2 = -3.016188
x_3 = 48.200187
y_3 = -3.015764
x_1, y_1 = convert_longlat_to_rad(x_1, y_1)
x_1, y_1 = convert_rad_to_cart(x_1, y_1)
x_2, y_2 = convert_longlat_to_rad(x_2, y_2)
x_2, y_2 = convert_rad_to_cart(x_2, y_2)
x_3, y_3 = convert_longlat_to_rad(x_3, y_3)
x_3, y_3 = convert_rad_to_cart(x_3, y_3)
print(x_1, y_1)
print(x_2, y_2)
print(x_3, y_3)
