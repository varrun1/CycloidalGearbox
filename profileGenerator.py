import matplotlib.pyplot as plt
import numpy as np

# x_points = np.array([1, 8])
# y_points = np.array([3, 10])

# plt.plot(x_points,y_points)
# plt.show()

# Define cycloidal gearbox parameters 
D = 100 # mm
N = 30
n = N - 1
d_fp = 5 # mm
e = 4 # mm

for phi in np.linspace(0, 2*np.pi, 100):
    argNum = np.sin(n*phi)
    argDen = np.cos(n*phi) - (D/(2*e*N))
    gamma = np.arctan2(argNum/argDen)
    x_val = (D/2)*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
    y_val = -(D/2)*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)
