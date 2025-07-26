import numpy as np

def force_analysis(
    thetas: np.ndarray,
    T_in: float,
    e: float,
    r_pin: float,
    r_pcd: float,
    z_p: int,
    z_out: int,
    r_out: float,
    pressure_angle_fn
):

    Nθ = thetas.size
    # gather all per‐lobe pressure angles alphaₖ(θ)
    alpha = np.vstack([pressure_angle_fn(thetas, e, r_pin, r_pcd, z_p, k+1)
        for k in range(z_p)
    ])  # shape (z_p, Nθ)

   # S1: cam reaction force (constant)
    F_cam = T_in / e

    # S2: distribute the radial force 
    mask   = (alpha <= (np.pi/2))                 # active‐lobe mask
    counts = mask.sum(axis=0)                # N_active(θ) for each θ_j
    #print('Active lobes: ',counts) #debugging statement

    # radial force per lobe:
    Fr = np.zeros_like(alpha)
    nonzero = counts > 0
    Fr[:, nonzero] = (F_cam / counts[nonzero]) * mask[:, nonzero] #Fr(k) = Fcam/Nout

    # Compute Fn and Ft (Ft = transmitted force)
    Fn = Fr / np.sin(alpha)                       
    Ft = Fn * np.cos(alpha)

    # S3: output torque from the sum of tangential forces at radius r_pcd
    T_out = np.sum(Ft * r_pcd, axis=0)    # shape (Nθ,)
    F_out = T_out / (z_out * r_out)       # per‐pin radial force

    return F_cam, Fn, Ft, Fr, T_out, F_out

def pressure_angle_closed(theta, e, r_pin, r_pcd, zp, k):
        lam = r_pcd / (e * zp)
        theta_k = theta + 2 * np.pi * (k - 1) / zp

        A = 1 + lam**2 - 2 * lam * np.cos(theta_k)
        num = -np.sin(theta_k) * A**(-0.5)

        B = (
            1
            - 2 * (r_pin / r_pcd) * (lam - np.cos(theta_k)) * A**(-0.5)
            + (r_pin / r_pcd)**2
        )
        den = np.sqrt(B)

        return np.arccos(num / den)

def shaftForces(T_input, r_shaft, Sy):
    conversion = 1e-6
    J = (np.pi/2)*(r_shaft**4) #polar moment of inertia, solid CS
    tau_max = ((T_input*r_shaft)/J)*conversion
    print('Expected shaft load is (MPa):',np.round(tau_max,4))
    t_allow_VM = Sy/np.sqrt(3)
    t_allow_Tresca = Sy/2
    SF = 2 # allowing for print defects, etc
    FOS_tres = (t_allow_Tresca/(tau_max*SF))
    FOS_VM = (t_allow_VM/(tau_max*SF))
    print('VM Allowable Stress (MPa): ', np.round(t_allow_VM, 4))
    print('VM FOS:', np.round(FOS_VM,4))
    print('Tresca Allowable Stress (MPa): ', np.round(t_allow_Tresca,4))
    print('Tresca FOS:', np.round(FOS_tres,4))
    

    #Backward calculation
    print('Backward Calculation')
    T_in_back = (((t_allow_Tresca/SF)/conversion)*J)/r_shaft #applied SF
    print('Allowable T_in (Nm):', np.round(T_in_back,4))

     

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # parameters
    thetas = np.linspace(0, 2*np.pi, 361)
    T_in   =  0.16      # Maximum motor torque
    e      =  0.85/1000     # m
    r_pin  =  5/1000    # m
    r_pcd  =  80/1000     # m
    z_p    = 30
    z_out  =  8
    r_out  =  55/1000     # m
    r_shaft = 10/1000

    # Material parameters
    Sy_pla = 35 # MPA 
    Sy_petg = 34 # MPA

    F_cam, Fn, Ft, Fr, T_out, F_out = force_analysis(thetas, T_in, e, r_pin, r_pcd, z_p, z_out, r_out, pressure_angle_closed)
    shaftForces(T_in, r_shaft, Sy_pla)


    plt.figure(figsize=(8,4))
    plt.plot(np.degrees(thetas), Fr[0], label="Fr (radial)")
    plt.plot(np.degrees(thetas), Ft[0], label="Ft (tangential)")
    plt.plot(np.degrees(thetas), Fn[0], label="Fn (normal)")
    plt.xlabel("θ (°)")
    plt.ylabel("Force (N)")
    plt.title("Lobe #1 Force Components")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # plot cam force & per‐pin output force
    plt.figure(figsize=(6,3))
    plt.plot(np.degrees(thetas), F_cam * np.ones_like(thetas), "--", label="F_cam")
    plt.plot(np.degrees(thetas), F_out,   "-", label="F_out_pin")
    plt.xlabel("θ (°)")
    plt.ylabel("Force (N)")
    plt.title("Cam Reaction vs. Per Output‐Pin Load")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    """
    plt.figure(figsize=(6,4))
    plt.imshow(Fn, 
           aspect='auto', 
           extent=[0, 360, z_p, 1],
           cmap='plasma',
           origin='upper')
    plt.colorbar(label='Fn (N)')
    plt.xlabel("θ (°)")
    plt.ylabel("Lobe index k")
    plt.title("Normal Force on Each Lobe Over One Revolution")
    plt.show()
    

    #Force dist across lobes at Theta 
    θ0 = np.deg2rad(120)
    j0 = np.argmin(np.abs(thetas - θ0))
    plt.figure()
    plt.bar(np.arange(1, z_p+1) - 0.2, Fr[:,j0], width=0.2, label='Fr')
    plt.bar(np.arange(1, z_p+1)      , Ft[:,j0], width=0.2, label='Ft')
    plt.bar(np.arange(1, z_p+1) + 0.2, Fn[:,j0], width=0.2, label='Fn')
    plt.xlabel("Lobe k")
    plt.ylabel("Force (N)")
    plt.title(f"Force Distribution at θ={np.degrees(thetas[j0]):.1f}°")
    plt.legend()
    plt.grid(True)
    plt.show()
   """

