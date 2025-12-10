# Assignment 5 EKF + LQR Experiment Plan

Workflow to collect the data requested by `specs_assignment5.md`, process it in MATLAB, and fill the report/template. Paths assume this repo layout and the Arduino/QRC channel map below.

---

## 0. Preparation

1. **Measure geometry (update code + MATLAB variables)**
   - `a` (half wheelbase if needed for motor mapping) is already given by `WHEELBASE/2 = 0.083 m` in `mecotron.h`.
   - Measure and record in meters (update both `extended_kalman_filter.cpp` and `assignment5_solution.m`):
     - `alpha` – center to front IR along X'.
     - `beta` – center to lateral IR along X'.
     - `gamma` – center to lateral IR along Y' (positive to the sensor side).
   - Measure the wall equations: default corner at `x=0`, `y=0` with robot starting in `x<0, y<0`. Adjust `(p,q,r)` if the arena differs.

2. **Firmware setup**
   - Flash `arduino_files/ass5_ino/CT-EKF-Swivel.ino` after updating the constants above, `Q/R` guesses, and `Kfb` (LQR gains).
   - Confirm PI velocity gains (in `robot.cpp`) remain as in Assignment 2 unless you intentionally retune.
   - Initial state in `resetKalmanFilter()`: `(-0.30, -0.20, 0.0)` m/rad (per spec). Update if you start elsewhere.

3. **Buttons and modes**
   - Button 0: enable velocity loop + trajectory feedforward + state-feedback (if `Kfb ≠ 0`).
   - Button 1: enable/reset EKF.
   - Button 2: start/stop trajectory playback.
   - Button 3: reset trajectory pointer to the start.

4. **QRC channel map (log all during runs)**
   - ch0: `v_ff` (trajectory.v), ch1: `omega_ff`
   - ch2–4: `x_ref, y_ref, theta_ref`
   - ch5: `hasMeasurements` flag
   - ch6–7: wheel speeds A/B (rad/s)
   - ch8–9: IR measurements `z1` (front), `z2` (side)
   - ch10–11: motor voltages A/B
   - ch12–14: state estimates `xhat, yhat, thetahat`
   - ch15–16: innovations `nu1, nu2`
   - ch17–19: covariance diagonals `Pxx, Pyy, Ptt`
   - Sampling: 100 Hz (Ts = 10 ms) to match firmware.

5. **Safety**
   - Keep sensors in range (<80 cm). Verify the turn is obstacle-free. Clamp wheel-speed references to ±12 rad/s (already in code).

---

## 1. EKF Q/R Sweep (Spec 3b)
Goal: feedforward-only trajectory with different Q/R ratios; analyze convergence and bias.

1. **Firmware parameters (per run)**
   - Set `kQx/kQy/kQtheta` and `kRz1/kRz2` in `extended_kalman_filter.cpp` according to the planned combo.
   - Keep `Kfb` = zeros (or very small) if you want pure feedforward; otherwise the default gains are mild.
   - Reflash after each change.

2. **Execution per run**
   - Place cart at `(-0.30, -0.20)` m facing +X.
   - Button 1 (EKF on/reset) → Button 0 (control on) → Button 2 (start trajectory).
   - Let the full straight–turn–straight profile finish; Button 2 to stop or Button 3 to reset.

3. **Files and labels (match MATLAB script)**
   - `matlab_assign5/data/ekf_Q1_R1.csv` (nominal Q_proc_nom, R_meas_nom).
   - `ekf_Q5_R1.csv`, `ekf_Q1_R5.csv`, `ekf_Q5_R5.csv` for other ratios.

4. **Notes to capture**
   - Innovation spikes near sensor loss (turn start), any drift after sensors are disabled, and heading estimate smoothness.

---

## 2. EKF Uncertainty & Measurement Plots (Spec 3c)
1. Choose the “best” combo from Section 1 (small bias, stable covariance); keep it as `ekfNominalIdx` in MATLAB.
2. Run once more and save as `ekf_Q1_R1.csv` (or the chosen best filename).
3. Verify channels 17–19 are nonzero so the 95% CI can be plotted.
4. Export state + measurement plots in MATLAB:
   - `ekf_states_QR_sweep.pdf`
   - `ekf_uncertainty_95ci.pdf`

Record observations on:
   - Covariance drop while both sensors are active vs. growth when `hasMeasurements=0`.
   - Behaviour if sensors stayed on during the turn (describe qualitatively for the report).

---

## 3. LQR Tracking Experiments (Spec 4c)
Goal: closed-loop tracking of the same trajectory with varying Q/R (max 4 combos), **without feedforward** for these tests.

1. **Compute K in MATLAB**
   - Update `Q_lqr`, `R_lqr` in `assignment5_solution.m`, run to get `K_lqr`.
   - Copy the numerical `K_lqr` into `robot.cpp` (matrix `arrayKfbInit`). For “no feedforward” tests, temporarily set `uff.Fill(0)` in `control()`.

2. **Runs and filenames**
   - `lqr_A.csv`, `lqr_B.csv`, `lqr_C.csv`, `lqr_D.csv` aligned with the Q/R sets in the script.

3. **Execution**
   - Button 1 (EKF) → Button 0 (controller) → Button 2 (trajectory).
   - Ensure feedforward is disabled for Section 4(c) runs; restore it for normal operation afterward.

4. **Signals to inspect**
   - Tracking errors `e_x, e_y, e_theta` over time.
   - Control signals (log `v` and `omega` if you add channels, else proxy with ch0–1 feedforward values).
   - Convergence speed vs. oscillations as Q/R changes; voltage saturation events.

---

## 4. Data Processing in MATLAB
1. Update measured `alpha, beta, gamma`, wall equations, and any non-default Q/R in `assignment5_solution.m`.
2. Ensure data filenames in `ekfRuns` and `lqrRuns` match the recorded CSVs.
3. Run the script; it will export to `tex_control/ass5_tex/images/`:
   - `ekf_states_QR_sweep.pdf`
   - `ekf_uncertainty_95ci.pdf`
   - `lqr_tracking_errors.pdf`
   - `lqr_control_signals.pdf`
4. Note the winning Q/R (EKF) and Q/R (LQR) along with chosen `P0` for the report tables.

---

## 5. Report Integration Checklist

- Fill measured values for `a, alpha, beta, gamma` and wall equations.
- Insert exported figures into `tex_control/ass5_tex/assignment5.tex` placeholders.
- Document chosen `Q`, `R`, `P0`, and qualitative trends (drift during dead-reckoning, effect of higher R, etc.).
- Summarize the selected `K` matrix from MATLAB and the rationale for final Q/R in LQR.

---

## Quick Status Checklist
- [ ] Geometry measured and code constants updated (`extended_kalman_filter.cpp`, `assignment5_solution.m`)
- [ ] EKF logs: `ekf_Q1_R1.csv`, `ekf_Q5_R1.csv`, `ekf_Q1_R5.csv`, `ekf_Q5_R5.csv`
- [ ] EKF nominal CI log captured
- [ ] LQR logs: `lqr_A.csv`, `lqr_B.csv`, `lqr_C.csv`, `lqr_D.csv` (feedforward disabled)
- [ ] MATLAB figures regenerated into `tex_control/ass5_tex/images/`
- [ ] Report placeholders filled with numbers/plots and final Q/R/K choices documented
