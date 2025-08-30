# Cycloidal Gearbox — Firmware (STM32 Nucleo) + Python Analysis

This repo contains two complementary parts:

1. **STM32 Nucleo firmware** to drive a stepper motor through my cycloidal reducer and run motion test routines (accuracy, repeatability, backlash, ratio).
2. **Python analysis tools** for my pre‑design calculations/visualization and post‑test data analysis.

It’s meant as a complete loop: **command motions → measure → analyze → iterate**.

---

## Repository structure

```
.
├─ CycloidGearbox/                     # MCU firmware (STM32 HAL)  ← put your CubeIDE project here
│  ├─ main.c / main.h
│  ├─ motor_hal.c / motor_hal.h
│  └─ ... (timer ISR, GPIO dir, constants, config)
│
├─ TestingAnalysis/            # Python analysis (post‑processing)
│  ├─ AccuracyTesting.py       # absolute accuracy vs commanded angle
│  ├─ Backlash.py              # bidirectional backlash / lost‑motion analysis
│  ├─ RatioTesting.py          # speed/ratio comparisons (measured vs theoretical)
│  ├─ Repeatability.py         # same‑direction landing repeatability
│  └─ ShaftTesting.py          # shaft/runout checks (optional)
│
├─ FA.py                       # pre‑design: force analysis helpers
├─ PA_viz.py                   # pre‑design: profile/geometry visualization
├─ PAcompute_cf.py             # pre‑design: compute calibration factor(s)
├─ PAcompute_vec.py            # pre‑design: vectorized profile computations
├─ profileGenerator.py         # pre‑design: generate cycloidal profiles
├─ viz_attempt.py              # scratch viz utilities
└─ README.md
```
> Tip: add screenshots/figures in `docs/` and reference them here.

Example screenshots (replace with your own):
- `docs/repo-structure.png`
- `docs/testing-scripts.png`

---

## Firmware overview (STM32 Nucleo)

- Tooling: **STM32CubeIDE** (or GCC/Make + HAL).
- Direction convention used across code & analysis: **+angle = CCW**, **−angle = CW**.
- Key routines (in `motor_hal.c`):
  - `MoveByAngleConst(motor, motor_angle_rad, motor_rpm)` — constant step rate at the **motor**.
  - `MoveOutputByDeg_FixedMotorRPM(motor, out_deg, motor_rpm)` — output‑space move using the gear ratio.
  - **Test helpers** (used during measurements):
    - `Landing_FromCW(...)`, `Landing_FromCCW(...)` — retreat then re‑approach from a given side.
    - `BacklashTest_FixedRPM(...)` — orchestrates CW/CCW landings at a constant motor RPM.
- Safety / setup:
  - Mount the dial indicator **tangentially** to the arm’s path (perpendicular to radius).
  - Start **touching the probe**, dial **zeroed**, before running a landing routine.
  - Use a modest retreat (**2–3°**) and a constant, gentle **motor RPM**.

---

## Python analysis quick start

Create a virtual environment and install deps:

```bash
python -m venv .venv
# Windows
.venv\Scripts\activate
# macOS/Linux
# source .venv/bin/activate

pip install numpy pandas matplotlib
```

Run any script from the root of the repo, e.g.

```bash
python TestingAnalysis/Repeatability.py
python TestingAnalysis/Backlash.py
python TestingAnalysis/AccuracyTesting.py
python TestingAnalysis/RatioTesting.py
```

---

## What the analysis scripts do

### Repeatability (`TestingAnalysis/Repeatability.py`)
**Question:** How tightly do repeated landings cluster when approaching from the **same direction**?

- Input: one or more arrays of dial readings (mm) or pre‑converted arcminutes.
- Conversion (if using mm at radius `r_mm`):
  $\theta_{\text{arcmin}} = s_{\text{mm}}\cdot\frac{10800}{\pi\,r_{\text{mm}}}$
- Metrics reported (arcmin): **mean (bias)**, **σ (sample std, ddof=1)**, **MAE**, **RMSE**, **Max|err|**, **N**.
- Plots:
  - **Strip (dot) plot** per set with **mean ±1σ** (stacked duplicates, no random jitter).

### Backlash (`TestingAnalysis/Backlash.py`)
**Question:** What’s the bidirectional **lost motion** at a target when approaching from CW vs CCW?

- Provide paired arrays `A` (CW landing) and `B` (CCW landing) in **mm** or **arcmin**.
- Signed difference: `d = A − B` (arcmin) → **directional bias** (which side lands deeper).
- Backlash magnitude: `|A − B|` (arcmin) → the spec number.
- Outputs:
  - Summary: mean / median / p95 / max of **|A−B|**.
  - Signed bias: **mean ± σ** of `d`.
  - Plots: paired A/B with **light‑grey connectors**; scatter of **|A−B|** per trial with mean line.

### Accuracy (`TestingAnalysis/AccuracyTesting.py`)
**Question:** How accurate is the output angle vs command (e.g., 1–5°)?

- Convert dial travel (mm) at radius `r_mm` → degrees:
  \f[\theta_{\deg} = \frac{s}{r}\cdot\frac{180}{\pi}\f]
- Fit measured vs commanded. Report:
  - **slope** (scale error), **intercept** (offset), **R²**,
  - **RMSE/MAE/Max|err|** (arcmin),
  - **calibration factor** = `1/slope` (multiply commands to correct scale).
- Plots: **Measured vs Commanded** (with ideal `y=x`) and **residuals** (points only).

### Ratio (`TestingAnalysis/RatioTesting.py`)
**Question:** How close is measured output speed to theoretical (nominal ratio × input speed)?

- Report % differences and **MAPE**; slope of measured vs theoretical gives a **ratio calibration factor**.
- ⚠️ Note: this is **not efficiency**. Efficiency needs torque as well (power in/out). See below.

---

## Handy conversions used throughout

- **mm → degrees / arcminutes**
  ```python
  theta_deg    = (mm / r_mm) * (180.0/np.pi)
  theta_arcmin = mm * (10800.0 / (np.pi * r_mm))  # direct mm→arcmin
  ```

- **Backlash from paired A/B (mm)**
  ```python
  d_arcmin        = (A - B) * (10800.0 / (np.pi * r_mm))   # signed bias
  backlash_arcmin = np.abs(A - B) * (10800.0 / (np.pi * r_mm))  # magnitude
  ```

- **Repeatability metrics (array `e` in arcmin)**
  ```python
  mean  = e.mean()
  sigma = e.std(ddof=1)           # sample std
  mae   = np.mean(np.abs(e))
  rmse  = np.sqrt(np.mean(e**2))  # vs zero
  maxabs= np.max(np.abs(e))
  ```

---


## Dial‑indicator setup notes

- Probe axis **tangential** to the arm motion (perpendicular to radius).
- Use the correct radius `r_mm` from shaft center to contact point.
- Typical retreat for landing tests: **2–3°** (at 150 mm ⇒ 5.2–7.9 mm travel).
- With a **0.01 mm** dial at **150 mm**, one tick = **0.229 arcmin** → measurements may be **resolution‑limited**.

---

---

## Acknowledgements

Thanks to open‑source STM32 HAL examples and the scientific Python ecosystem (NumPy, Pandas, Matplotlib).
