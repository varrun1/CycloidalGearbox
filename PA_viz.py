import numpy as np
import matplotlib.pyplot as plt

# This file is used to visualize the pressure angle variation along the cycloid disk's singular tooth profile

# — Parameters
D, N, d_fp, e = 100, 30, 5, 0.85
R, n = D/2, N-1

# — Generate full profile centered at origin
phi = np.linspace(0, 2*np.pi, 1000, endpoint=False)
gamma = np.arctan2(np.sin(-n*phi), (D/(2*e*N)) - np.cos(-n*phi))
x = R*np.cos(phi) - (d_fp/2)*np.cos(phi+gamma) - e*np.cos(N*phi)
y = -R*np.sin(phi) + (d_fp/2)*np.sin(phi+gamma) + e*np.sin(N*phi)

# — Key rotation‐angles (in radians)
phi_pts = [0, np.pi/3, np.pi, 5*np.pi/3, 2*np.pi]
labels = ['1 (0°)','2 (60°)','3 (180°)','4 (300°)','5 (360°)']
xp, yp = [], []

# — Compute profile points at each key phi
for p in phi_pts:
    g = np.arctan2(np.sin(-n*p), (D/(2*e*N)) - np.cos(-n*p))
    xp.append(R*np.cos(p) - (d_fp/2)*np.cos(p+g) - e*np.cos(N*p))
    yp.append(-R*np.sin(p) + (d_fp/2)*np.sin(p+g) + e*np.sin(N*p))

# — Plot
fig, ax = plt.subplots(figsize=(6,6))

# Cycloidal profile
ax.plot(x, y, linewidth=1.5, color='blue')

# Pitch circle
ax.add_patch(plt.Circle((0,0), R, edgecolor='gray', ls='--', fill=False))

# All fixed pins around pitch circle
for k in range(N):
    angle_k = 2*np.pi * k / N
    pin_x = R * np.cos(angle_k)
    pin_y = R * np.sin(angle_k)
    ax.add_patch(plt.Circle((pin_x, pin_y), radius=d_fp/2, color='black'))

# Highlight the "active" pin at 0°
ax.add_patch(plt.Circle((R,0), radius=d_fp/2, edgecolor='red', fill=False, linewidth=2))

# Mark and label the five key lobes
ax.scatter(xp, yp, color='red', s=60)
for xi, yi, lab in zip(xp, yp, labels):
    ax.text(xi, yi, lab, color='red', ha='center', va='bottom')

ax.set_aspect('equal')
ax.set_xlim(-R-15, R+15)
ax.set_ylim(-R-15, R+15)
ax.set_title('Cycloidal Lobe & All Pin Locations')
ax.axis('off')
plt.show()
