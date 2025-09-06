import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np
import pandas as pd

# Constants
fullROT = 360.0 #degrees
reduction = 29.0
arcmin_conversion = 60
L = 0.150 # in meters
cycles = 10

def mm_to_deg(d_mm):
    deg = (d_mm/(L*1000))*(180/np.pi)
    return deg

def metrics_row(y_land):
    return dict(
        mean_arcmin   = y_land.mean(),                    # bias
        sigma_arcmin  = np.std(y_land, ddof=1),           # sample std (precision)
        MAE_arcmin    = np.mean(np.abs(y_land)),
        RMSE_arcmin   = np.sqrt(np.mean(y_land**2)),      # vs zero
        MaxAbs_arcmin = np.max(np.abs(y_land)),
        N             = y_land.size
    )

#Measured datasets
set1_A = np.array([-0.05,-0.06,-0.06,-0.05,-0.05,-0.05,-0.05,-0.06,-0.06,-0.06]) 
set1_B = np.array([-0.04,-0.05,-0.04,-0.04,-0.04,-0.04,-0.04,-0.05,-0.05,-0.05]) 
trials = np.arange(1, set1_A.size+1)

#Convert measured data
set1_A_arcmin = mm_to_deg(set1_A)*arcmin_conversion
set1_B_arcmin = mm_to_deg(set1_B)*arcmin_conversion
diff = set1_A-set1_B
arcmin_measured = mm_to_deg(diff)*arcmin_conversion
arcmin_measured_abs = mm_to_deg(abs(diff))*arcmin_conversion
print('\nMeasured data')
print('A (arcmin):',set1_A_arcmin) 
print('B (arcmin):',set1_B_arcmin) 
print('Measured Backlash:',arcmin_measured) 
print('Measured Abs Backlash:',arcmin_measured_abs) 
labels  = ["Set 1"]

# 1) Paired points with connectors ( trial’s A vs B)
plt.figure()
plt.plot(trials, set1_A_arcmin, marker="o", linestyle="none", label="CCW landing (A)")
plt.plot(trials, set1_B_arcmin, marker="s", linestyle="none", label="CW landing (B)")
for i in range(set1_A.size):
    plt.plot([trials[i], trials[i]], [set1_A_arcmin[i], set1_B_arcmin[i]], linestyle="--", color = 'grey')  # vertical connector
plt.axhline(0, linestyle="--", linewidth=1, color = 'red')
plt.xlabel("Trial #"); plt.ylabel("Dial reading (arcmin)")
plt.title("Backlash pairs: CCW (A) vs CW (B) per trial")
plt.grid(True); plt.legend(); plt.tight_layout()

# 2) Backlash magnitude per trial (|A-B| in arcmin)
plt.figure()
plt.scatter(trials, arcmin_measured_abs)
plt.axhline(arcmin_measured_abs.mean(), linestyle="--", label=f"Mean = {arcmin_measured_abs.mean():.3f}'")
plt.xlabel("Trial #"); plt.ylabel("Backlash |A−B| (arcmin)")
plt.title("Backlash magnitude per trial")
plt.grid(True); plt.legend(); plt.tight_layout()

print('\nQuick Summary')
print(f"N={set1_A.size}, mean={arcmin_measured_abs.mean():.3f}', median={np.median(arcmin_measured_abs):.3f}', "
      f"p95={np.percentile(arcmin_measured_abs,95):.3f}', max={arcmin_measured_abs.max():.3f}'") #using abs arcmin to rep lost motion
print(f"bias = {np.mean(arcmin_measured):.3f}',  sigma = {np.std(arcmin_measured, ddof=1):.3f}'") #showing 1std 
plt.show()