import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np

#Prints out string equation for SW equation-driven curve
def swEquation(D,dr,n,e,N):
    R = D/2
    Rr = dr/2 
    n = -n
    X_eq = "({}*cos(t)) - ({}*cos(t+arctan( sin({}*t)/(({}) - cos({}*t))))) - ({}*cos({}*t)) ".format(R,Rr,n,R/(e*N),n,e,N)
    Y_eq = "(-{}*sin(t)) + ({}*sin(t+arctan( sin({}*t)/(({}) - cos({}*t))))) + ({}*sin({}*t)) ".format(R,Rr,n,R/(e*N),n,e,N) 
    print(X_eq)
    print(Y_eq)

def plotFullStatic(N, D, d_fp, x_value, y_value, e):
    fig1, ax1, = plt.subplots()

    x_val_offset = [x-e for x in x_value]

    # Plot fixed ring pins
    thetaRing = (2*np.pi)/N
    for pinNum in range(N):
        pin_cx = (D/2)*np.cos(pinNum*thetaRing)
        pin_cy = (D/2)*np.sin(pinNum*thetaRing)
        pin = plt.Circle((pin_cx, pin_cy), radius = d_fp/2, color='black')
        ax1.add_patch(pin)
    ax1.plot(x_val_offset, y_value)
    shaft = plt.Circle((0, 0), 0.5, color='black')  # Fixed input shaft center
    ax1.add_patch(shaft)
    ax1.set_title('Static Gearbox Diagram')
    plt.grid(True)
    plt.axis('equal')
    plt.show()
    #eccenPath = plt.Circle((0,0), e, fill=False, edgecolor='brown',linestyle='--')


#-----------------------------
# MAIN CODE
#-----------------------------

#|||||||||||||||||||||||||||||
# Define profile parameters
#|||||||||||||||||||||||||||||
D = 100 # mm
N = 30
n = N - 1
d_fp = 5 # mm
e = 0.85 # mm

x_val = []
y_val = []
phi_val = np.linspace(0, 2*np.pi, 10000, endpoint=False)

#|||||||||||||||||||||||||||||
# Generate disc profile points
#|||||||||||||||||||||||||||||
for phi in phi_val:
    argNum = np.sin(-n*phi)
    argDen = (D/(2*e*N)) -np.cos(-n*phi)
    gamma = np.arctan2(argNum,argDen)
    x_current = (D/2)*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
    y_current = -(D/2)*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)
    x_val.append(x_current)
    y_val.append(y_current)

#swEquation(D,d_fp,n,e,N) # function call for equation generator

#|||||||||||||||||||||||||||||
# Plot static disc profile (at origin)
#|||||||||||||||||||||||||||||
fig, ax, = plt.subplots()
ax.plot(x_val, y_val)
shaft = plt.Circle((0, 0), 0.5, color='black')  # Fixed input shaft center
ax.add_patch(shaft)
ax.set_title('Static Cycloidal Disk Profile')
plt.grid(True)
plt.axis('equal')
plt.show()

plotFullStatic(N, D, d_fp, x_val,y_val, e)