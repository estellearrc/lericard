from numpy import array,zeros
import numpy as np

"""Calibrage de boussole (usage unique)"""

x1 = array([[864.02],[-4670.3],[5298.5]])
x_1 = array([[7030.86],[-4034.02],[4805.73]])
x2 = array([[4137.86],[-7385.4],[5082.37]])
x3 = array([[4066.86],[-4540.8],[1934.98]])

b = -1/2*(x1 + x_1)
print(b)
def f(xi):
	return np.linalg.inv(M)@(xi +b) 
beta = 46000
v1 = 1/beta*(x1+b)
v2 = 1/beta*(x2 + b)
v3 = 1/beta*(x3 + b)
M  = zeros([3,3])
M[:,0]=v1.flatten()
M[:,1]=v2.flatten()
M[:,2]=v3.flatten()
print(f(x_1))
print(M)
