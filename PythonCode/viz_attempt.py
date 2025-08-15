import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

D = 100 # mm
N = 30
n = N - 1
d_fp = 5 # mm
e = 0.85 # mm

x_val = []
y_val = []
phi_val = np.linspace(0, 2*np.pi, 10000, endpoint=False)
#phase_shift = np.pi / N  # Half the lobe pitch
#phi_val = np.linspace(0, 2*np.pi, 10000, endpoint=False) + phase_shift

for phi in phi_val:
    argNum = np.sin(-n*phi)
    argDen = (D/(2*e*N)) -np.cos(-n*phi)
    gamma = np.arctan2(argNum,argDen)
    x_current = (D/2)*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
    y_current = -(D/2)*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)
    x_val.append(x_current)
    y_val.append(y_current)


x_profile = np.array(x_val)
y_profile = np.array(y_val)

#Center Pointer initial profile
lineX_profile = np.array([0,-e*4]) #ACTION: make unit vector? 
lineY_profile = np.array([0,0])

# -----------------------------
# Gear parameters
# -----------------------------
#e = 4                # Eccentricity (mm)
#N = 30               # Number of lobes / ring pins
total_frames = 360   # One full orbit (2π)
R = 60              # Arbitrary profile scale reference (used for plot limits)

# -----------------------------
# Set up the plot
# -----------------------------
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-')
centerLine, = ax.plot([],[],'r-')
discCenter, = ax.plot([],[],'go',markersize=5)
shaft = plt.Circle((0, 0), 0.5, color='black')  # Fixed input shaft center
eccenPath = plt.Circle((0,0), e, fill=False, edgecolor='brown',linestyle='--')


# Plot fixed ring pins
thetaRing = (2*np.pi)/N
for pinNum in range(N):
    pin_cx = (D/2)*np.cos(pinNum*thetaRing)
    pin_cy = (D/2)*np.sin(pinNum*thetaRing)
    pin = plt.Circle((pin_cx, pin_cy), radius = d_fp/2, color='black')
    ax.add_patch(pin)

base_circle = plt.Circle((0, 0), radius=D/2, edgecolor='blue', fill=False, linestyle='--')
ax.add_patch(base_circle)

ax.add_patch(shaft)
ax.add_patch(eccenPath)
ax.set_aspect('equal')
ax.set_xlim(-R, R)
ax.set_ylim(-R, R)
ax.set_title('Cycloidal Disc with Physical Rotation')

rpm = 0.5
omega = 2 * np.pi * rpm / 60  # rad/s
fps = 60
dt = 1 / fps



# -----------------------------
# Frame update function
# -----------------------------
def update(frame):
    #speed_factor = 8  # slows it to 1/4 speed
    #theta = 0
    #theta = (2 * np.pi * frame) / (total_frames * speed_factor)
    t = frame * dt
    theta = omega * t
    #theta = 2 * np.pi * frame / total_frames        # Motor shaft angle (0 → 2π)
    theta_offset = np.pi/N
    spin_angle = (1 - N) * theta + theta_offset             # Actual disc rotation
    
    #spin_angle = 0 

    # Orbiting center of the disc
    cx = -e * np.cos(theta)
    cy = -e * np.sin(theta)

    # Rotation of the disc shape about its center
    cos_spin = np.cos(spin_angle)
    sin_spin = np.sin(spin_angle)
    x_rot = cos_spin * x_profile - sin_spin * y_profile
    y_rot = sin_spin * x_profile + cos_spin * y_profile

    # Rotation of motor shaft tick (i.e theta)
    cos_lineSpin = np.cos(theta)
    sin_lineSpin = np.sin(theta)
    xLine_rot = cos_lineSpin * lineX_profile - sin_lineSpin * lineY_profile
    yLine_rot = sin_lineSpin * lineX_profile + cos_lineSpin * lineY_profile

    # Translate rotated disc to its orbiting position
    x_orbit = x_rot + cx
    y_orbit = y_rot + cy

    # Update plot
    line.set_data(x_orbit, y_orbit)
    centerLine.set_data(xLine_rot, yLine_rot)
    discCenter.set_data([cx],[cy])
    return line, centerLine, discCenter

# -----------------------------
# Run animation
# -----------------------------
ani = FuncAnimation(fig, update, frames=total_frames, interval=30, blit=True)
plt.show()
