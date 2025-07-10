import matplotlib.pyplot as plt
import numpy as np

# x_points = np.array([1, 8])
# y_points = np.array([3, 10])

# plt.plot(x_points,y_points)
# plt.show()

#Prints out string equation for SW equation-driven curve
def swEquation(D,dr,n,e,N):
    R = D/2
    Rr = dr/2 
    n = -n
    X_eq = "({}*cos(t)) - ({}*cos(t+arctan( sin({}*t)/(({}) - cos({}*t))))) - ({}*cos({}*t)) ".format(R,Rr,n,R/(e*N),n,e,N)
    Y_eq = "(-{}*sin(t)) + ({}*sin(t+arctan( sin({}*t)/(({}) - cos({}*t))))) + ({}*sin({}*t)) ".format(R,Rr,n,R/(e*N),n,e,N) 
    print(X_eq)
    print(Y_eq)

# MAIN CODE
# Define cycloidal gearbox parameters 
D = 100 # mm
N = 30
n = N - 1
d_fp = 5 # mm
e = 0.8 # mm

x_val = []
y_val = []
phi_val = np.linspace(0, 2*np.pi, 10000, endpoint=False)

for phi in phi_val:
    argNum = np.sin(-n*phi)
    argDen = (D/(2*e*N)) -np.cos(-n*phi)
    gamma = np.arctan2(argNum,argDen)
    x_current = (D/2)*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
    y_current = -(D/2)*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)
    x_val.append(x_current)
    y_val.append(y_current)

swEquation(D, d_fp,n,e,N)

plt.plot(x_val, y_val)
plt.grid(True)
plt.axis('equal')
plt.show()