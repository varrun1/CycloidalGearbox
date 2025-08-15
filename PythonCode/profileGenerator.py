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

#Plot static disc profile (at origin)
def plotStaticProf(x_val, y_val, D): 
    fig, ax, = plt.subplots()
    ax.plot(x_val, y_val)
    shaft = plt.Circle((0, 0), 0.5, color='black')  # Fixed input shaft center
    ax.add_patch(shaft)
    base_circle = plt.Circle((0,0),radius = (D/2),fill=False,linestyle='--',edgecolor='red',linewidth=2) # PCD outline plot
    ax.add_patch(base_circle)
    ax.set_title('Static Cycloidal Disk Profile')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

#Plot full static cycloidal gearbox profile(s)
def plotFullStatic(N, D, d_fp, x_value, y_value, e, outputPCD, d_outH, numOutPin, outPinD):
    fig1, ax1, = plt.subplots()
    disc2Color = 'GoldenRod'
    # Static 1st disc, offset @ +e
    x_val_offset = [x+e for x in x_value]
    y_val_offset = y_value

    # Static 2nd disc, 180 deg OoP
    x_val_180 = [-x for x in x_value]
    y_val_180 = [-y for y in y_value]
    x_val_180_shifted = [x - e for x in x_val_180] 
    
    # Plot fixed ring pins
    thetaRing = (2*np.pi)/N
    for pinNum in range(N):
        pin_cx = (D/2)*np.cos(pinNum*thetaRing)
        pin_cy = (D/2)*np.sin(pinNum*thetaRing)
        pin = plt.Circle((pin_cx, pin_cy), radius = (d_fp/2), color='black')
        ax1.add_patch(pin)

    # Plot output pin holes on disc #1
    thetaOutput = (2*np.pi)/numOutPin
    for pinNum in range(numOutPin):
        pin_cx = (outputPCD/2)*np.cos(pinNum*thetaOutput) + e
        pin_cy = (outputPCD/2)*np.sin(pinNum*thetaOutput)
        pin = plt.Circle((pin_cx, pin_cy), radius = (d_outH/2), fill = False, color = 'blue')
        ax1.add_patch(pin)

    # Plot output pin holes on disc #2
    for pinNum in range(numOutPin):
        pin_cx = (outputPCD/2)*np.cos(pinNum*thetaOutput) - e
        pin_cy = (outputPCD/2)*np.sin(pinNum*thetaOutput)
        pin = plt.Circle((pin_cx, pin_cy), radius = (d_outH/2), fill = False, color = disc2Color)
        ax1.add_patch(pin)    

    # Plot output disc pins
    for pinNum in range(numOutPin):
        pin_cx = (outputPCD/2)*np.cos(pinNum*thetaOutput)
        pin_cy = (outputPCD/2)*np.sin(pinNum*thetaOutput)
        pin = plt.Circle((pin_cx, pin_cy), radius = (outPinD/2), fill = False, color = 'red')
        ax1.add_patch(pin)       

    # Plot both static discs
    ax1.plot(x_val_offset, y_val_offset, color = 'blue', linewidth = 2)
    ax1.plot(x_val_180_shifted,y_val_180, color = disc2Color, linewidth = 2)

    #Plot static outlines
    shaft = plt.Circle((0, 0), 0.5, color='black')  # Fixed input shaft center
    eccenPath = plt.Circle((0,0), e, fill=False, edgecolor='brown',linestyle='--') # Eccentric path outline
    ringPin_PCD = plt.Circle((0,0),radius = (D/2),fill=False,linestyle='--',edgecolor='red',linewidth=1) #Fixed ring pin PCD outline
    output_PCD_circ1 = plt.Circle((e,0),radius = (outputPCD/2),fill=False,linestyle='--',edgecolor='DarkRed',linewidth=1) #Disc #1 output ring holes
    output_PCD_circ2 = plt.Circle((-e,0),radius = (outputPCD/2),fill=False,linestyle='--',edgecolor='DarkRed',linewidth=1) #Disc #2 output ring holes
    ax1.add_patch(shaft)
    ax1.add_patch(eccenPath)
    ax1.add_patch(ringPin_PCD)
    ax1.add_patch(output_PCD_circ1)
    ax1.add_patch(output_PCD_circ2)
    ax1.set_title('Static Gearbox Diagram')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

    

#-----------------------------
# MAIN CODE
#-----------------------------

#|||||||||||||||||||||||||||||
# Define profile parameters
D = 80 # mm
N = 30
n = N - 1
d_fp = 5 # mm
e = 1.0 # mm
output_PCD = 40
d_outPin = 4
d_outputPinHole = d_outPin + 2*e 
N_output = 6

x_val = []
y_val = []
phi_val = np.linspace(0, 2*np.pi, 10000, endpoint=False)

#|||||||||||||||||||||||||||||
# Generate disc profile points
for phi in phi_val:
    argNum = np.sin(-n*phi)
    argDen = (D/(2*e*N)) -np.cos(-n*phi)
    gamma = np.arctan2(argNum,argDen)
    x_current = (D/2)*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
    y_current = -(D/2)*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)
    x_val.append(x_current)
    y_val.append(y_current)

#|||||||||||||||||||||||||||||
#Function Calls
plotStaticProf(x_val, y_val, D)
#plotFullStatic(N, D, d_fp, x_val,y_val, e, output_PCD, d_outputPinHole, N_output, d_outPin)
swEquation(D,d_fp,n,e,N) # function call for equation generator