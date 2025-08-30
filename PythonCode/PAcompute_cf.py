import numpy as np
import matplotlib.pyplot as plt

def pressure_angle_closed(theta, e, r_pin, r_pcd, zp, k):
    """Computes pressure angle for kth lobe across range of input rotation angle (theta), using closed-form solution

    Args:
        theta (list): range of input rotation angle
        e (float): eccentricity
        r_pin (float): fixed roller pin radius
        r_pcd (float): PCD radius
        zp (int): fixed roller pin count
        k (int): lobe number

    Returns:
        float: pressure angle for kth lobe at range of theta
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

def lobePlot (thetas, e, r_pin, r_pcd, zp):
    """
    Computes and plots all pressure angles across range of theta
    """
    plt.figure(figsize=(8, 6))
    alpha_list = []

    plt.subplot(2, 1, 1)
    for k in range(1, zp+1):
        αk = pressure_angle_closed(thetas, e, r_pin, r_pcd, zp, k)
        alpha_list.append(αk)
        plt.plot(np.degrees(thetas),np.degrees(αk),label=f'k={k}')
    #plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left')
    plt.title("Pressure Angles – Closed Form")
    plt.xlabel("Rotation Angle θ (°)")
    plt.ylabel("Pressure Angle αₖ (°)")
    #plt.tight_layout()
    plt.grid(True)

    alpha_arr = np.vstack(alpha_list)  # radians, 2D array holding alpha values for each lobe

    #include only PA ≤ 90° (π/2), except skip the j=0,k=0 corner if it's exactly π/2 (i.e 90 deg)
    mask = alpha_arr <= (np.pi/2)
    if np.isclose(alpha_arr[0, 0], np.pi/2):
        mask[0, 0] = False

    # COmpute instantaneous average at each θ_j:
    # avg_inst[j] = ( sum_k α_k(θ_j) * mask ) / (count of mask) 
    sums   = np.where(mask, alpha_arr, 0.0).sum(axis=0) #replace every False entry with 0, then sum 
    counts = mask.sum(axis=0) #essentially store active lobes
    avg_inst = np.zeros_like(thetas) #create array with same struct as thetas
    nonzero = counts > 0 #find and store actual location of alpha
    avg_inst[nonzero] = (sums[nonzero] / counts[nonzero]) / 2.0 #compute mean of all PA, divide by 2 to change scale

    #remove outlier produced at theta = 0
    thetas = thetas[1:]
    avg_inst = avg_inst[1:]
    avg_all_inst = np.degrees(np.mean(avg_inst))
    print('Average Pressure Angle (°):',np.round(avg_all_inst,4))

    plt.subplot(2, 1, 2)
    plt.plot(np.degrees(thetas),np.degrees(avg_inst),'o-', markersize=3)
    plt.axhline(avg_all_inst,color = 'red', ls = '--')
    plt.title("Instantaneous Multi‑Tooth Average Pressure Angle")
    plt.xlabel("Rotation Angle θ (°)")
    plt.ylabel("Avg Pressure Angle φ̄(θ) (°)")
    plt.grid(True)
    plt.ylim(np.degrees(avg_inst).min() - 0.05, np.degrees(avg_inst).max() + 0.05)
    plt.tight_layout()
    

def comparParam(thetas, e, r_pin, r_pcd, zp):
    """
    Compares effect of disc parameters on pressure angle
    """
    plt.figure(figsize=(6,6))
    s = 0.05 #spacing value for eccentricity
    s2 = 5 #spacing value for PCD
    center_e = e
    center_pcd = r_pcd
    eccen_test = [center_e-2*s, center_e-1*s,center_e,center_e+s, center_e+2*s]
    pcd_test = [center_pcd-1*s2,center_pcd,center_pcd+s2, center_pcd+2*s2]

    plt.subplot(2,1,1)
    for i in range(len(eccen_test)):
        current_e = np.round(eccen_test[i], 2)
        alpha = pressure_angle_closed(thetas, current_e, r_pin, r_pcd, zp, 1)
        plt.plot(np.degrees(thetas), np.degrees(alpha), label=f'e={current_e}')
    plt.legend(bbox_to_anchor=(1.02,1), loc='upper left', ncol=1)
    plt.xlabel('Rotation Angle θ (°)')
    plt.ylabel('Pressure Angle αₖ (°)')
    plt.title('Pressure Angles (CF) - Eccentricity Comparison')
    plt.tight_layout()
    plt.ylim(20, 160)
    plt.grid(True)

    plt.subplot(2,1,2)
    for i in range(len(pcd_test)):
        current_pcd = np.round(pcd_test[i], 2)
        alpha = pressure_angle_closed(thetas, e, r_pin, current_pcd, zp, 1)
        plt.plot(np.degrees(thetas), np.degrees(alpha), label=f'r_PCD={current_pcd}')
    plt.legend(bbox_to_anchor=(1.02,1), loc='upper left', ncol=1)
    plt.xlabel('Rotation Angle θ (°)')
    plt.ylabel('Pressure Angle αₖ (°)')
    plt.title('Pressure Angles (CF) - Ring Pin Diameter Comparison')
    plt.tight_layout()
    plt.ylim(20, 160)
    plt.grid(True)

def compareProfile(thetas, e, r_pin, r_pcd, zp):
    #Profile 2 Specification
    #disk1 = [e, r_pin,r_pcd,zp]
    disk2 = [1.0,r_pin,r_pcd,zp] 
    
    plt.figure(figsize=(10,5))
    alphaD1 = pressure_angle_closed(thetas, e, r_pin, r_pcd, zp, 1)
    alphaD2 = pressure_angle_closed(thetas, disk2[0],disk2[1],disk2[2],disk2[3],1)

    theta_max1, theta_min1, max_PA1, min_PA1 = maxMinPA(alphaD1, thetas)
    theta_max2, theta_min2, max_PA2, min_PA2 = maxMinPA(alphaD2, thetas)

    #"""
    p1_label = (
        "Profile 1\n"
        f"    e = {e}\n"
        f"    r_pin = {r_pin}\n"
        f"    r_pcd = {r_pcd}\n"
        f"    zp = {zp}"
    )
    p2_label = (
        "Profile 2\n"
        f"    e = {disk2[0]}\n"
        f"    r_pin = {disk2[1]}\n"
        f"    r_pcd = {disk2[2]}\n"
        f"    zp = {disk2[3]}"
    )
    #"""

    plt.plot(np.degrees(thetas), np.degrees(alphaD1), label=p1_label)
    plt.plot(np.degrees(thetas), np.degrees(alphaD2), label=p2_label)
    plt.axvline(theta_max1, color = 'red', ls = '--')
    plt.axvline(theta_min1, color = 'green', ls = '--')
    plt.plot(theta_max1,max_PA1,marker = 'o', markersize = '5', markeredgecolor = 'red', markerfacecolor = 'none')
    plt.plot(theta_min1,min_PA1,marker = 'o', markersize = '5', markeredgecolor = 'green', markerfacecolor = 'none')

    plt.axvline(theta_max2, color = 'red', ls = '--')
    plt.axvline(theta_min2, color = 'green', ls = '--')
    plt.plot(theta_max2,max_PA2,marker = 'o', markersize = '5', markeredgecolor = 'red', markerfacecolor = 'none')
    plt.plot(theta_min2,min_PA2,marker = 'o', markersize = '5', markeredgecolor = 'green', markerfacecolor = 'none')

    plt.legend(bbox_to_anchor=(1.02,1), loc='upper left', handletextpad = 0.5, labelspacing = 1.2)
    plt.xlabel('Rotation Angle θ (°)')
    plt.ylabel('Pressure Angle αₖ (°)')
    plt.title('Pressure Angles (CF) - Profile Comparison')
    plt.tight_layout()
    plt.ylim(20, 160)
    plt.grid(True)


def computePA(thetas, e, r_pin, r_pcd, zp):
    """
    Computes and plots single-lobe pressure angle
    """
    plt.figure(figsize=(8,5))
    alpha = pressure_angle_closed(thetas, e, r_pin, r_pcd, zp, 1)
    alpha = np.array(alpha) # convert to numpy array
    
    theta_max, theta_min, max_PA, min_PA = maxMinPA(alpha, thetas)

    plt.plot(np.degrees(thetas), np.degrees(alpha), label=f'k={1}')
    plt.axvline(theta_max, color = 'red', ls = '--')
    plt.axvline(theta_min, color = 'green', ls = '--')
    plt.plot(theta_max,max_PA,marker = 'o', markersize = '5', markeredgecolor = 'red')
    plt.plot(theta_min,min_PA,marker = 'o', markersize = '5', markeredgecolor = 'green')
    plt.xlabel('Rotation Angle θ (°)')
    plt.ylabel('Pressure Angle αₖ (°)')
    plt.title('Pressure Angles - Closed Form')
    plt.tight_layout()
    plt.ylim(20, 160)
    plt.grid(True)

    print('PA computed by Closed-form')
    print('Maximum Pressure Angle (°):',np.round(max_PA, 4), '@ θ=',np.round(theta_max, 2),'°')
    print('Minimum Pressure Angle (°):',np.round(min_PA, 4), '@ θ=',np.round(theta_min, 2),'°')


def maxMinPA(alpha,thetas):
    """Returns max & min pressure angle
    Args:
        alpha (list): list of pressure angles
        thetas (list): list of input angle from 0 -> 360 
    """
    #Compute minimum and maximum PA
    max_PA = np.degrees(alpha.max())
    theta_max = np.degrees(thetas[alpha.argmax()])
    min_PA = np.degrees(alpha.min())
    theta_min = np.degrees(thetas[alpha.argmin()])

    return theta_max, theta_min, max_PA, min_PA



def main():
    # Profile parameters
    e    = 1.0  # eccentricity
    r_pin   = 5/2    # roller radius
    r_pcd  = 80/2   # distribution-circle radius
    zp   = 30     # number of pins

    # Generate input angle data points from 0 to 360°
    thetas = np.linspace(0, 2*np.pi, 1000)
    
    #Available functions
    computePA(thetas, e, r_pin, r_pcd, zp) #compute single-lobe Pressure angle for profile 
    lobePlot(thetas, e, r_pin, r_pcd, zp) # plots all lobes' pressure angle variation 
    #comparParam(thetas, e, r_pin, r_pcd, zp) #plots parameter comparison 
    #compareProfile(thetas, e, r_pin, r_pcd, zp) 

    plt.show()

if __name__ == '__main__':
    main()
