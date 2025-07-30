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
    pressure_angle_fn, 
    N_disc: float
):
    """Performs Force Analysis on Disc(s)'s based on gearbox configuration

    Args:
        thetas (np.ndarray): range of input rotation angle
        T_in (float): input torque, max torque of motor
        e (float): eccentricity
        r_pin (float): fixed roller pin radius
        r_pcd (float): PCD radius
        z_p (int): fixed roller pin count
        z_out (int): output pin count 
        r_out (float): output pin radius
        pressure_angle_fn (string): pressure angle function name
        N_disc (float): disc number
    """

    Nθ = thetas.size
    # gather all per‐lobe pressure angles alphaₖ(θ)
    alpha = np.vstack([pressure_angle_fn(thetas, e, r_pin, r_pcd, z_p, k+1)
        for k in range(z_p)
    ])  # shape (z_p, Nθ) - 2D array of z_p rows and theta (N) columns

   # 1: cam reaction force (constant)
    F_cam_total = T_in / e
    F_cam = F_cam_total/N_disc

    # 2: distribute the radial force 
    mask   = (alpha <= (np.pi/2))                 # active‐lobe mask
    counts = mask.sum(axis=0)                # N_active(θ) for each θ_j

    # radial force per lobe:
    Fr = np.zeros_like(alpha)
    nonzero = counts > 0
    Fr[:, nonzero] = (F_cam / counts[nonzero]) * mask[:, nonzero] #Fr(k) = Fcam/Nout

    # Compute Fn and Ft (Ft = transmitted force)
    Fn = Fr / np.sin(alpha)                       
    Ft = Fn * np.cos(alpha)

    # 3: output torque from the sum of tangential forces at radius r_pcd
    T_out = np.sum(Ft * r_pcd, axis=0)    
    F_out = T_out / (z_out * r_out)       # per‐pin radial force

    return F_cam, Fn, Ft, Fr, T_out, F_out, alpha


def pressure_angle_closed(theta, e, r_pin, r_pcd, zp, k):
    """Pressue angle calculator based on other code

    Args:
        theta (list): range of input rotation angle
        e (float): eccentricity
        r_pin (float): fixed roller pin radius
        r_pcd (float): PCD radius
        zp (int): fixed roller pin count
        k (int): lobe number

    Returns:
        float: pressure angle across theta for each pin
    """
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
    """Perform force analysis on shaft

    Args:
        T_input (_type_): input torque from motor
        r_shaft (_type_): shaft radius
        Sy (_type_): yield strength of material 
    """
    print('\n--------------------------')
    print('Shaft Force Analysis')
    #compute torsional stress 
    conversion = 1e-6
    SF = 2 # allowing for print defects, etc
    J = (np.pi/2)*(r_shaft**4) #polar moment of inertia, solid CS
    tau_max = ((T_input*r_shaft)/J)*conversion #converted to MPa
    print('Expected torsional shaft load is (MPa):',np.round(tau_max,4))

    print('Safety Factor (in all proceeding cals):',SF)
    #compute allowable stress using Von mises & Tresca critera
    t_allow_VM = Sy/np.sqrt(3)
    t_allow_Tresca = Sy/2
    FOS_tres = (t_allow_Tresca/(tau_max*SF))
    FOS_VM = (t_allow_VM/(tau_max*SF))
    print('VM Allowable Stress (MPa): ', np.round(t_allow_VM, 4))
    print('VM FOS:', np.round(FOS_VM,4))
    print('Tresca Allowable Stress (MPa): ', np.round(t_allow_Tresca,4))
    print('Tresca FOS:', np.round(FOS_tres,4))
    
    #Backward calculation for max input motor torque 
    print('Backward Calculation')
    T_in_back = (((t_allow_Tresca/SF)/conversion)*J)/r_shaft #applied SF
    print('Allowable T_in (Nm):', np.round(T_in_back,4))
    print('----------------------------------\n')


def plot_static_disc_forces_numbered(
    thetas: np.ndarray,
    Fr:      np.ndarray,    # shape (z_p, Nθ)
    Ft:      np.ndarray,    # shape (z_p, Nθ)
    Fn:      np.ndarray,    # shape (z_p, Nθ)
    alpha:   np.ndarray,    # shape (z_p, Nθ)
    r_pcd:   float,         # pitch‑circle radius
    j:       int,           # index into thetas
    prof_x:  np.ndarray,    # disc profile x,y centered at (0,0)
    prof_y:  np.ndarray,
    scale:   float = 0.002  # scaling for force vectors
):
    """ Plots force vectors on active lobes at input rotation angle, theta """
    θ0 = thetas[j]
    z_p = Fr.shape[0]

    # compute disc spin‐angle
    psi = - (z_p - 1)/z_p * θ0
    cψ, sψ = np.cos(psi), np.sin(psi)

    # rotate the raw profile by psi around (0,0)
    xp =  prof_x * cψ - prof_y * sψ
    yp =  prof_x * sψ + prof_y * cψ

    # plot static disc profile 
    fig, ax = plt.subplots(figsize=(6,6))
    ax.plot(prof_x, prof_y, '-', color='0.3', lw=1, label='disc profile')

    #Plot PCD circle
    pcd_circle = plt.Circle((0, 0), radius = r_pcd, fill = False,  color='black', ls = '--')
    ax.add_patch(pcd_circle)

    # Scaling-related stuff
    max_force = max(Fr.max(), Ft.max(), Fn.max())
    arrow_scale = r_pcd / max_force * 0.5

    for k in range(z_p):
        Fr0 = Fr[k, j]
        if Fr0 <= 0:
            continue  # only label active lobes

        Ft0 = Ft[k, j]
        Fn0 = Fn[k, j]
        α0  = alpha[k, j]

        φk = θ0 + 2*np.pi*k / z_p
        xk, yk = r_pcd*np.cos(φk), r_pcd*np.sin(φk)

        #unit vectors for radial, tangential and normal force
        ur = np.array([ np.cos(φk),  np.sin(φk)])
        ut = np.array([-np.sin(φk),  np.cos(φk)])
        un = np.array([ np.cos(φk + np.pi/2 + α0),
                        np.sin(φk + np.pi/2 + α0)])

        #Plot vectors using quiver function
        ax.quiver(xk, yk,
                  ur[0]*Fr0*arrow_scale, ur[1]*Fr0*arrow_scale,
                  angles='xy', scale_units='xy', scale=2.5,color='C0', width=0.005)
        ax.quiver(xk, yk,
                  ut[0]*Ft0*arrow_scale, ut[1]*Ft0*arrow_scale,
                  angles='xy', scale_units='xy', scale=2,color='C1', width=0.005)
        ax.quiver(xk, yk,
                  un[0]*Fn0*arrow_scale, un[1]*Fn0*arrow_scale,
                  angles='xy', scale_units='xy', scale=2.5,color='C2', width=0.005)

        # lobe numbering offset for each vector. change 0.20 to control offset
        num_off = un * (r_pcd * 0.20)
        ax.text(xk + num_off[0],
                yk + num_off[1],
                str(k+1),
                color='k',
                fontsize= 8,
                fontweight='bold',
                ha='center',
                va='center',
               )

    ax_padding = 0.01
    ax.set_aspect('equal', 'box')
    ax.set_title(f'Forces at θ={np.degrees(θ0):.1f}°')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_xlim(-r_pcd - ax_padding, r_pcd+ax_padding)
    ax.set_ylim(-r_pcd - ax_padding, r_pcd+ax_padding)

    ax.plot([], [], color='C0', label='Fr (radial)')
    ax.plot([], [], color='C1', label='Ft (tangential)')
    ax.plot([], [], color='C2', label='Fn (normal)')
    ax.legend(loc='upper right')
    ax.grid(alpha=0.3)
    plt.show()


def genProfile(e, r_pin, r_pcd, z_p):
    """Generate cycloidal disc profile (used from other script)

    Args:
        e (float): eccentricity
        r_pin (float): fixed roller pin radius
        r_pcd (float): PCD radius
        z_p (int): fixed roller pin count

    Returns:
        list: list of x and y profile points
    """
    D    = 2*r_pcd
    N    = z_p
    d_fp = r_pin
    n    = N - 1

    phi = np.linspace(0, 2*np.pi, 10_000, endpoint=False)
    # pre‐allocate
    x = np.empty_like(phi)
    y = np.empty_like(phi)

    for i, φ in enumerate(phi):
        γ = np.arctan2( np.sin(-n*φ),
                        D/(2*e*N) - np.cos(-n*φ) )
        x[i] = (D/2)*np.cos(φ) \
               - (d_fp/2)*np.cos(φ+γ) \
               - e*np.cos(N*φ)
        y[i] = -(D/2)*np.sin(φ) \
               + (d_fp/2)*np.sin(φ+γ) \
               + e*np.sin(N*φ)
    return x, y

def lobe_root_stress_from_profile(
    Ft:             np.ndarray | float,  # tangential force at the pitch circle of kth lobe
    r_pcd:          float,               # pitch‑circle radius
    x_prof:         np.ndarray,          # raw cycloid outline x coords
    y_prof:         np.ndarray,          # raw cycloid outline y coords
    r_shaft:        float,               # inner bore/shaft radius
    web_thickness:  float,               # b, width of the web section
    allowable:      float                # allowable stress (Pa)
) -> tuple[np.ndarray, np.ndarray, float, float, np.ndarray, np.ndarray,np.ndarray,np.ndarray]:
    """Computes VM stress on given lobe through root stress method, using bending moment + shear analysis
    """
    Ft = np.atleast_1d(Ft).astype(float)

    # 1) find root radius & web height
    rs     = np.hypot(x_prof, y_prof)
    r_base = rs.min()
    h      = r_base - r_shaft
    if h <= 0:
        raise ValueError(f"Computed web height h={h:.4g} m is ≤ 0.")

    # 2) section properties (assume rectangular properties)
    b = web_thickness
    I = (b * h**3) / 12
    c = h / 2

    # 3) moment arm from root of lobe to pitch circle
    L = r_pcd - r_base

    # 4) gen output arrays, mask out Ft==0
    active   = Ft != 0.0
    sigma_b  = np.full_like(Ft, np.nan)
    sigma_vm = np.full_like(Ft, np.nan)
    FOS      = np.full_like(Ft, np.nan)
    tau_max_array  = np.full_like(Ft, np.nan)
    sigma_vm_total = np.full_like(Ft, np.nan)
    FOS_total= np.full_like(Ft, np.nan)

    # 4.5) compute bending moment and shear stresses for active rotation angles of input lobe
    if np.any(active):
        S                = I / c
        M_active         = Ft[active] * L          # bending moment [N·m]
        sigma_b_active  = M_active / S              # bending stress [Pa]
        sigma_vm_bend = np.abs(sigma_b_active)    # von Mises for pure bending
        FOS_active      = allowable / (sigma_vm_bend * (1e-6))

        tau_max = ((3*Ft[active])/(2*b*h))
        sigma_vm_full = np.sqrt(sigma_b_active**2 + 3*(tau_max**2))
        FOS_full = allowable/ (sigma_vm_full * (1e-6))

        # storing current values into respective arrays
        sigma_b [active] = sigma_b_active
        sigma_vm[active] = sigma_vm_bend
        FOS     [active] = FOS_active
        tau_max_array[active] = tau_max
        sigma_vm_total[active] = sigma_vm_full
        FOS_total[active] = FOS_full

    print('Min Bending FOS:',np.round(np.nanmin(FOS),2))
    print('Min Total FOS:',np.round(np.nanmin(FOS_total),2))
        
    return sigma_b, sigma_vm, c, I, FOS, tau_max_array,sigma_vm_total,FOS_total
     

# MAIN CODE
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # design parameters
    thetas = np.linspace(0, 2*np.pi, 361)
    T_in   =  0.16      # Maximum motor torque
    e      =  1/1000     # m
    r_pin  =  2.5/1000    # m
    r_pcd  =  40/1000     # m
    z_p    = 30
    z_out  =  8
    r_out  =  55/1000     # m
    r_shaft = 4/1000 #m
    r_bearing = (28/2)/1000 #m
    disc_t = 8/1000 #m

    # Material parameters
    Sy_pla = 35 # MPA 
    Sy_petg = 34 # MPA

    #disc configuration (1 = single disc FA, 2 = dual disc FA)
    disc_Config = 1
    if disc_Config == 1:
        print('Running single disc configuration')
    else: 
        print('Running dual disc configuration')

    #Force Analysis Function calls
    F_cam, Fn, Ft, Fr, T_out, F_out, alpha_total = force_analysis(thetas, T_in, e, r_pin, r_pcd, z_p, z_out, r_out, pressure_angle_closed, disc_Config)
    shaftForces(T_in, r_shaft, Sy_pla)
    profile_x, profile_y = genProfile(e, r_pin,r_pcd,z_p)

    #Plot lobe-specific force analysis over input shaft angle
    lobeNum = 15 #based on 0-indexing
    plt.figure(figsize=(8,4))
    plt.plot(np.degrees(thetas), Fr[lobeNum], label="Fr (radial)")
    plt.plot(np.degrees(thetas), Ft[lobeNum], label="Ft (tangential)")
    plt.plot(np.degrees(thetas), Fn[lobeNum], label="Fn (normal)")
    plt.xlabel("Input Angle, θ (°)")
    plt.ylabel("Force (N)")
    plt.title(f'Lobe {lobeNum+1} Force Components')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    j = 0  # index corresponding to theta (input angle)

    """
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
    θ0 = np.deg2rad(j)
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
    
    #Function call for force vector plot function
    plot_static_disc_forces_numbered(thetas, Fr, Ft, Fn, alpha_total, r_pcd, j, np.array(profile_x), np.array(profile_y),)

    # Lobe root stress analysis
    Ft_lobe = Ft[lobeNum,:]
    sigma_b, σ_vm, c, I, FOS, tau_m, sigmavm_total, FOS_t = lobe_root_stress_from_profile(Ft_lobe,r_pcd,np.array(profile_x), np.array(profile_y),r_bearing,web_thickness=disc_t, allowable=Sy_pla)
    active = ~np.isnan(sigma_b) # mask out the inactive θ’s (where Ft==0 → sigma_b is NaN)

    plt.figure(figsize=(15,6))
    plt.subplot(3,1,1)
    plt.plot(np.degrees(thetas[active]), sigma_b[active] * 1e-6, 'o-', markersize=4)
    plt.xlabel('Rotation θ (°)')
    plt.ylabel('Bending Stress (MPa)')
    plt.title(f'Lobe {lobeNum+1} Bending Stress vs θ')
    plt.xlim(0,360)
    plt.grid(True)
    plt.tight_layout()

    plt.subplot(3,1,2)
    plt.plot(np.degrees(thetas[active]), tau_m[active] * 1e-6, 'o-', markersize=4)
    plt.xlabel('Rotation θ (°)')
    plt.ylabel('Shear Stress (MPa)')
    plt.title(f'Lobe {lobeNum+1} Shear Stress vs θ')
    plt.xlim(0,360)
    plt.grid(True)
    plt.tight_layout()

    plt.subplot(3,1,3)
    plt.plot(np.degrees(thetas[active]), sigmavm_total[active] * 1e-6, 'o-', markersize=4)
    plt.xlabel('Rotation θ (°)')
    plt.ylabel('VM Stress (MPa)')
    plt.title(f'Lobe {lobeNum+1} VM vs θ')
    plt.xlim(0,360)
    plt.grid(True)
    plt.tight_layout()
    plt.show()
