import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np

# Constants
leverArm = 0.15 # m
T_in = 0.16 # Nm
m_bottle = 0.107 # kg
m_cap = 0.047 # kg
g = 9.81
r_shaft = 8/1000 # m
conversion = 1e-6

mass_tested = np.array([0.170, 0.202,0.228,0.257,0.304,
               0.351,0.377,0.402,0.452,0.551,0.576]) # mass of water + bottle (no cap)

#Compute resultant torque acting on shaft
total_mass = mass_tested + m_cap
force_r = total_mass*g
torque = force_r*leverArm

#Compute torque-based SF
max_torque = torque[-1]
SF = max_torque/T_in
print('Achieved FOS:', np.round(SF,3))

#Compute torsion curve
J = (np.pi/2)*(r_shaft**4)
tau = ((torque*r_shaft)/J)*conversion
print(tau)

#Plotting torque vs mass
plt.figure(figsize=(8,4))
plt.subplot(2,1,1)
plt.plot(total_mass,torque, marker='o', linestyle='-', markersize=5)
plt.xlabel('Total Mass (kg)')
plt.ylabel('Resultant Torque (Nm)')
plt.title('Resultant Torque on Shaft')
plt.grid(True)
plt.tight_layout()

plt.subplot(2,1,2)
plt.plot(total_mass,tau, marker='o', linestyle='-', markersize=5)
plt.xlabel('Total Mass (kg)')
plt.ylabel('Resultant Torsional Stress (MPa)')
plt.title('Torsional Shaft Stress')
plt.grid(True)
plt.tight_layout()
plt.show()


