import matplotlib.pyplot as plt
import numpy as np

# === Gear Parameters ===
D = 100       # pitch circle diameter (mm)
N = 30        # number of ring pins
n = N - 1     # number of lobes
d_fp = 5.0    # diameter of fixed pins (rollers)
e = 0.6      # eccentricity (mm)

# === Sampling ===
num_points = 5000
phi_vals = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

# === Step 1: Compute gamma array ===
numerator = np.sin(n * phi_vals)
denominator = np.cos(n * phi_vals) - (D / (2 * e * N))
gamma_vals = np.arctan2(numerator, denominator)

# === Step 2: Unwrap gamma (critical) ===
gamma_vals = np.unwrap(gamma_vals)

# === Step 3: Evaluate x, y with unwrapped gamma ===
x_vals = (D / 2) * np.cos(phi_vals) \
         - (d_fp / 2) * np.cos(phi_vals + gamma_vals) \
         - e * np.cos(N * phi_vals)

y_vals = -(D / 2) * np.sin(phi_vals) \
         + (d_fp / 2) * np.sin(phi_vals + gamma_vals) \
         + e * np.sin(N * phi_vals)

# === Plotting ===
plt.figure(figsize=(7, 7))
plt.plot(x_vals, y_vals)
plt.axis('equal')
plt.grid(True)
plt.title("Cycloidal Disc Profile — Fixed with Unwrapped Gamma")
plt.xlabel("x [mm]")
plt.ylabel("y [mm]")
plt.show()