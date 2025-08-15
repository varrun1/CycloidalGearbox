import numpy as np
import matplotlib.pyplot as plt

def pressure_angle(theta, e, r_p, r_pcd, z_p, z_c):
    theta = np.asarray(theta)
    i_h = z_p / z_c
    k1 = e * z_p / r_pcd
    S = 1 + k1**2 - 2*k1 * np.cos(theta)
    sqrtS = np.sqrt(S)
    nc_x = -np.cos((i_h - 1) * theta) + k1 * np.cos(i_h * theta)
    nc_y =  np.sin((i_h - 1) * theta) - k1 * np.sin(i_h * theta)
    vc_x = (
        -r_pcd * np.sin((1 - i_h) * theta)
        - e    * np.sin(i_h * theta)
        + r_p  * (np.sin((1 - i_h) * theta) + k1 * np.sin(i_h * theta)) / sqrtS
    )
    vc_y = (
         r_pcd * np.cos((i_h - 1) * theta)
        - e    * np.cos(i_h * theta)
        + r_p  * (-np.cos((i_h - 1) * theta) + k1 * np.cos(i_h * theta)) / sqrtS
    )
    dot = (vc_x * nc_x + vc_y * nc_y) / sqrtS
    cos_alpha = dot / np.hypot(vc_x, vc_y)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)
    return np.degrees(alpha)

def main():
    # Parameters
    e    = 0.85  # eccentricity
    r_p   = 5/2    # roller radius
    r_pcd  = 80/2   # distribution-circle radius
    z_p   = 30     # number of pins
    z_c   = z_p - 1 # lobes

    # Angle array
    theta_deg = np.linspace(0, 360, 361)
    theta_rad = np.radians(theta_deg)

    # Compute pressure angles
    phi = pressure_angle(theta_rad, e, r_p, r_pcd, z_p, z_c)
    print('PA computed by vector-form')
    print('Maximum Pressure Angle (°):',np.max(phi))
    print('Minimum Pressure Angle (°):',np.min(phi))

    # Plot
    plt.figure()
    plt.plot(theta_deg, phi)
    plt.xlabel("Rotation angle θ (degrees)")
    plt.ylabel("Pressure angle φ (degrees)")
    plt.title("Pressure Angle vs. Rotation Angle")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
