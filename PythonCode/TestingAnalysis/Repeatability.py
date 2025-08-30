import matplotlib.pyplot as plt
import matplotlib.patches as ptc
import numpy as np
import pandas as pd

# Constants
fullROT = 360.0 #degrees
reduction = 29.0
arcmin_conversion = 60
L = 0.145 # in meters
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
set1 = np.array([-0.01,-0.01,0,-0.01,0,0,0,-0.01,-0.01,-0.01]) 
set2 = np.array([-0.01,0,0,-0.02,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01]) 
#set3 = np.array([0,0,-0.01,-0.01,-0.02,-0.05,-0.05,-0.07,-0.08,-0.10]) 

#Convert measured data
repeatability_deg = mm_to_deg(np.concatenate([set1,set2])) #converted to 1D array (1x20)
arcmin_measured = repeatability_deg*arcmin_conversion
print('\nMeasured data')
print('Measured arcmin:',arcmin_measured) 
labels  = ["Set 1"]


plt.figure()
spread = 0.03 #contrl spacing of each datapoint
arr = np.asarray(arcmin_measured, dtype=float)
if arr.ndim == 1:                 # works for both 1D and 2D array
    arr = arr[None, :]
S, N = arr.shape
if labels is None:
    labels = [f"Set {i+1}" for i in range(S)]

xpos = np.arange(S)
palette = plt.rcParams['axes.prop_cycle'].by_key()['color']

for i, row in enumerate(arr):
    vals, cnts = np.unique(np.round(row, 9), return_counts=True)
    xs_all, ys_all = [], []
    for y, c in zip(vals, cnts): #offset of x-pos for each group of duplicates
        xs = xpos[i] + (np.arange(c) - (c-1)/2)*spread 
        xs_all.append(xs)
        ys_all.append(np.full(c, y))
    xs_all = np.concatenate(xs_all)
    ys_all = np.concatenate(ys_all)
    color = palette[i % len(palette)]
    plt.scatter(xs_all, ys_all, s=40, color=color, label=labels[i])

    # mean and ±1σ plotting
    mu  = row.mean()
    sig = row.std(ddof=1) if row.size > 1 else np.nan
    plt.plot([xpos[i]-0.15, xpos[i]+0.15], [mu, mu], color=color, linewidth=2)   # mean
    plt.plot([xpos[i], xpos[i]], [mu - sig, mu + sig], color=color, linewidth=2) # ±1σ
    plt.scatter([xpos[i]], [mu], marker="D", color=color, edgecolor="k", zorder=3)

plt.axhline(0, linestyle="--", linewidth=1, color = 'red') #target (i.e no error = 0)
plt.xticks(xpos, labels)
plt.ylabel("Landing error (arcmin)")
if S == 1:
    plt.xlim(-0.5, 0.5)           # center a single group nicely
plt.title("Repeatability Distribution")
plt.grid(True, axis="y")
plt.tight_layout()


summary = pd.DataFrame([metrics_row(row) for row in arr], index=labels)
print("\nSummary Table")
print(summary.round(4))
plt.show()







