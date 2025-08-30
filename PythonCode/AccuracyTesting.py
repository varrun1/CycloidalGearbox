import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np
import pandas as pd

# Constants
fullROT = 360.0 #degrees
reduction = 29.0
arcmin_conversion = 60
L = 0.145 # in meters
outputA_ideal_deg = np.array([1,2,3,4,5])

def mm_to_deg(d_mm):
    deg = (d_mm/(L*1000))*(180/np.pi)
    return deg

def metrics_row(y_deg, x_deg=outputA_ideal_deg):
    m, b = np.polyfit(x_deg, y_deg, 1)          # slope, intercept (deg)
    err = (y_deg - x_deg) * 60.0                # arcmin
    return dict(
        slope=m, scale_pct=(m-1)*100,
        intercept_deg=b, R2=1 - np.sum((y_deg-(m*x_deg+b))**2)/np.sum((y_deg-y_deg.mean())**2),
        RMSE_arcmin=np.sqrt(np.mean(err**2)),
        MAE_arcmin=np.mean(np.abs(err)),
        MaxAbs_arcmin=np.max(np.abs(err)),
        sigma_arcmin=np.std(err, ddof=1),
        cal_factor=1.0/m
    )

#Theoretical Data
 
outputA_ideal_rad = outputA_ideal_deg * (np.pi/180.0)
d_ideal = L*np.sin(outputA_ideal_rad)
arcmin_ideal = outputA_ideal_deg*arcmin_conversion
print('Commanded output angle (deg):',outputA_ideal_deg)
print('Commanded Output angle (rad):',outputA_ideal_rad)
print('Ideal displacement (mm):',d_ideal*1000)
print('Ideal Arcmin:',arcmin_ideal)

#Measured datasets
outputA_set1 = np.array([2.65,5.08,7.33,9.81,12.53]) 
outputA_set2 = np.array([2.38,4.68,7.32,10.04,12.66]) 
outputA_set3 = np.array([2.61,5.05,7.24,9.85,12.62]) 

#Convert measured data
outputA_measured_deg = mm_to_deg(np.vstack([outputA_set1, outputA_set2, outputA_set3]))
arcmin_measured = outputA_measured_deg*arcmin_conversion
error_arcmin = (outputA_measured_deg-outputA_ideal_deg)*arcmin_conversion
print('\nMeasured data')
#print(outputA_measured_deg)         # shape (3,5)
print('Measured arcmin:',arcmin_measured) 
print('Error arcmin:',error_arcmin) 

labels  = ["Set 1", "Set 2", "Set 3"]

plt.figure()
for i, measured_deg in enumerate(outputA_measured_deg):
    plt.plot(outputA_ideal_deg, measured_deg, marker="o", linestyle="-", label=labels[i])
plt.plot(outputA_ideal_deg, outputA_ideal_deg, linestyle="--", label="Ideal y=x")
plt.xlabel("Commanded (deg)")
plt.ylabel("Measured (deg)")
plt.title("Measured vs Commanded (overlay)")
plt.grid(True); plt.legend(); plt.tight_layout()


plt.figure()
for i, error in enumerate(error_arcmin):
    plt.plot(outputA_ideal_deg, error, marker="o", linestyle="-", label=labels[i])
plt.axhline(0, linestyle="--", linewidth=1)
plt.xlabel("Commanded (deg)")
plt.ylabel("Error (arcmin)")
plt.title("Residuals vs Commanded")
plt.grid(True); plt.legend(); plt.tight_layout()


print('\nSummary Table')
summary = pd.DataFrame([metrics_row(row) for row in outputA_measured_deg], index=labels)
print(summary.round(4))

plt.show()

# Most important in this table would be RSME_arcmin and MaxAbs_arcmin (most relevant to EDS)




