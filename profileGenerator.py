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
e = 0.8 # mm

x_val = []
y_val = []
#phi_val = np.radians(np.arange(0,360))
phi_val = np.linspace(0, 2*np.pi, 10000, endpoint=False)

for phi in phi_val:
    argNum = np.sin(-n*phi)
    argDen = (D/(2*e*N)) -np.cos(-n*phi)
    gamma = np.arctan2(argNum,argDen)
    x_current = (D/2)*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
    y_current = -(D/2)*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)
    x_val.append(x_current)
    y_val.append(y_current)


plt.plot(x_val, y_val)
plt.grid(True)
plt.axis('equal')
plt.show()