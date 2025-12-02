# Assignment 3 Data Acquisition & Analysis Plan

This plan outlines the exact order of operations to gather the measurements required by `specs_assignment3.md`, process the data, and update the report/codebase. Follow the steps sequentially; note which channels/buttons correspond to each action.

---

## 0. Preparation

1. **Firmware & wiring**
   - Flash `arduino_files/ass3_ino/CT-StateEstimation_StateFeedback.ino` to the cart.
   - Verify motors, IR sensor, and wheel encoders are working; keep the cart within the 5–30 cm sensor range.

2. **Telemetry mapping (QRC)**
   - Confirm channel assignments in `robot.cpp`:  
     `ch1`: measured distance, `ch2`: x̂, `ch3`: v\_ref, `ch4–5`: wheel speeds, `ch6–7`: voltages, `ch8`: innovation, `ch9`: x\_ref, `ch12`: avg speed, `ch13`: active K, `ch14`: active L.
   - Configure QRC logging templates to capture **time + channels (1–9,12–14)** at ≥100 Hz.

3. **Runtime overrides**
   - Reference input (distance setpoint) via `writeValue(0)` (meters).
   - Controller gain override via `writeValue(10)` (rad/(s·m)).
   - Estimator gain override via `writeValue(11)` (dimensionless).
   - Forced estimator initial guess via `writeValue(5)` (meters, negative in front of wall).

4. **Buttons**
   - Button 0: controller enable.
   - Button 1: estimator enable/reset.
   - Button 2: full MECOtron reset (use if states misbehave).

---

## 1. Estimator-Only Experiments (Spec §2(a))

Goal: Study convergence vs. L with wrong initial estimate, controller OFF.

1. **Setup**
   - Controller disabled (button 0 off). Estimator enabled (button 1 on).
   - Place cart stationary around −0.25 m (distance ≈ 0.25 m).

2. **L sweep**
   - Run three experiments with `L ∈ {−0.05, −0.18, −0.35}`. For each:
     1. Press button 1 to reset estimator.
     2. Immediately write `readValue(5) = -0.40` (or desired wrong estimate).
     3. Override estimator gain via channel 11 with the selected L.
     4. Record ≥10 s of data.

3. **Logging**
   - Export each run to CSV:  
     `matlab_assign3/data/est_only_L-005.csv`, `…L-018.csv`, `…L-035.csv`.
   - Ensure columns: `time`, `distance`, `xhat`, `innovation`.

4. **Notes**
   - Verify no drift (u=0). If innovation saturates, recheck the distance limit.

---

## 2. Controller-Only Experiments (Spec §2(b))

Goal: Study position control vs. K with estimator OFF.

1. **Setup**
   - Button 1 OFF (estimator disabled). Button 0 ON (controller active).
   - Command step reference via channel 0 (e.g., 0.15 m) to keep cart inside IR range.

2. **K sweep**
   - Gains `K ∈ {80, 120, 250}` rad/(s·m).
   - For each K:
     1. Override via channel 10 before enabling controller.
     2. Apply the same step reference magnitude.
     3. Record until response settles (≈6 s).

3. **Collected signals**
   - time, reference (ch0 stored separately), measured distance (ch1), motor voltages (ch6–7), wheel speeds (ch4–5), v\_ref (ch3).

4. **File naming**
   - `matlab_assign3/data/ctrl_only_K080.csv`, `…K120.csv`, `…K250.csv`.

5. **Observations to capture**
   - Rise time, overshoot, saturation events, steady-state error.

---

## 3. Combined Estimator + Controller (Spec §2(c))

Goal: Demonstrate separation principle and effect of slow estimator.

1. **Setup**
   - Enable both controller (button 0) and estimator (button 1).
   - Set position reference (e.g., 0.15 m) same as Section 2(b).

2. **Gain selection**
   - Controller: keep `K = 120` rad/(s·m).
   - Estimator: set slow pole (`L = -0.004`) via channel 11 (true 10× slower than control pole).
   - **Verification:** p_est = 1 + L = 0.996 ≈ 10× slower settling than p_cl = 0.96.

3. **Runs**
   - **Good initial estimate**: before enabling estimator, write actual distance to channel 5 (use measurement).
   - **Wrong initial estimate**: set channel 5 to -0.40 m before enabling.
   - For each, log at least 10 s covering the step response.

4. **Files**
   - `matlab_assign3/data/est_ctrl_good.csv`
   - `matlab_assign3/data/est_ctrl_bad.csv`

5. **Metrics**
   - Compare measured vs. estimated distance.
   - Note whether controller performance degrades when estimator starts wrong (should be minimal).

---

## 4. Data Processing (MATLAB)

1. Update `matlab_assign3/assignment3_solution.m`:
   - Set `estOnlyRuns`, `ctrlOnlyRuns`, `estCtrlRuns` arrays to the CSV paths.
2. Run the script; it will:
   - Regenerate theory plots (pole maps, simulated responses).
   - Produce experimental plots (`estimator_L_sweep.pdf`, `controller_K_response.pdf`, `controller_K_voltage.pdf`, `est_ctrl_good.pdf`, `est_ctrl_bad.pdf`) in `tex_control/ass3_tex/images/`.
3. Extract key metrics (settling times, RMS innovation) and jot them down for the report narrative.

---

## 5. Report Integration

1. Replace placeholder figure boxes in `tex_control/ass3_tex/assignment3.tex` with `\includegraphics{images/...}` references.
2. Insert measured numerical results into Sections 2(a/b/c), referencing the figures and commenting on convergence/overshoot/saturation.
3. Document final chosen gains (K and L) with justification from the experiments and MATLAB analysis.

---

## 6. Validation & Backup

1. Re-run critical experiments if any anomalies appear (e.g., IR clipping, actuator saturation).
2. Commit updated Arduino code, MATLAB figures, and LaTeX changes once satisfied.
3. Archive raw CSV files for traceability (keep in `matlab_assign3/data/` and optionally version-control large files if policy allows).

---

Following this plan ensures every bullet in `specs_assignment3.md` has corresponding data, plots, and written analysis, while keeping the workflow reproducible.

---

## 7. Remaining Checklist

### Code Fixes
- [x] **Fix controller-only mode** in `robot.cpp` (Issue 1 above) ✓
- [x] Update `L_slow` constant → now -0.004 in MATLAB and LaTeX ✓

### Data Collection (all CSVs in `matlab_assign3/data/`)
- [ ] `est_only_L-005.csv` (L = -0.05)
- [ ] `est_only_L-018.csv` (L = -0.18)
- [ ] `est_only_L-035.csv` (L = -0.35)
- [ ] `ctrl_only_K080.csv` (K = 80)
- [ ] `ctrl_only_K120.csv` (K = 120)
- [ ] `ctrl_only_K250.csv` (K = 250)
- [ ] `est_ctrl_good.csv` (correct initial estimate)
- [ ] `est_ctrl_bad.csv` (wrong initial estimate)

### MATLAB Processing
- [ ] Populate `estOnlyRuns`, `ctrlOnlyRuns`, `estCtrlRuns` arrays with CSV paths
- [ ] Generate figures: `estimator_L_sweep.pdf`, `controller_K_response.pdf`, `controller_K_voltage.pdf`, `est_ctrl_good.pdf`, `est_ctrl_bad.pdf`
- [ ] Update MATLAB pole map for L_slow = -0.004 in Section 2(c) discussion

### LaTeX Report (`assignment3.tex`)
- [ ] Replace figure placeholders with `\includegraphics{images/...}`
- [ ] Fill in TODO placeholders:
  - Settling time for L_nom (Section 2.1 observations)
  - RMS innovation for L_nom
  - Settling time for K_nom (Section 2.2 observations)
- [ ] Update $L_\text{slow}$ value from -0.02 to -0.004 in Section 2.3
- [ ] Final proofreading and consistency check

## 8. Review vs. `specs_assignment3.md`

### 8.1 Theoretical design (Task 1)

- **State and measurement models (1a, 1b)**  
  - `assignment3.tex` derives the continuous-time model, applies forward Euler, and states $A_d = 1$, $B_d = T_s r = 3.3\times10^{-4}$ m/rad, matching `specs_assignment3.md` and `assignment3_solution.m` (`A = 1; B = Ts * r;`).  
  - The measurement equation $y = -x$ with $C = -1$, $D = 0$ is used consistently in LaTeX, MATLAB (`C = -1; D = 0;`), and Arduino (`frontDistance` is positive, `feedbackPos = -frontDistance` when the estimator is disabled).

- **State feedback gain K (1c)**  
  - LaTeX correctly derives $p_\text{cl}(K) = 1 - T_s r K$ and discusses stability/response vs K.  
  - MATLAB implements the required pole map and simulated step responses: `p_cl = 1 - B * K_sweep;` with `K_sweep = [80, 120, 250]` and exports `pole_map_K.pdf` and `step_K_sweep.pdf`, as requested.  
  - The chosen nominal gain $K_\text{nom} = 120$ rad/(s·m) is documented and used consistently in MATLAB and in the plan (K-sweep section).

- **Estimator gain L (1d)**  
  - LaTeX derives $p_\text{est}(L) = 1 + L$ and explains stability and noise/convergence trade-offs in line with the spec.  
  - MATLAB sweeps `L_sweep = [-0.05, -0.18, -0.35]` and exports `pole_map_L.pdf`; $L_\text{nom} = -0.18$ and the sweep set are explicitly stated in LaTeX.  
  - Overall, the analytical part of 1(a)–1(d) is complete and consistent across LaTeX and MATLAB; remaining gaps are mainly numerical values to be filled from experiments.

### 8.2 Implementation and experiments (Task 2)

- **Estimator only (2a)**  
  - Spec requires: controller off, estimator on, wrong initial $\hat x[0]$, multiple L values, plots of measured vs estimated distance, linked to 1(d).  
  - Arduino: `resetStateEstimator()` reads an initial guess from `readValue(5)` (falling back to `kDefaultX0`), allowing both “good” and “wrong” initializations. With button 0 off and button 1 on, motors are held at zero voltage while the estimator runs, matching an "estimator only" mode.  
  - Plan: Section 1 prescribes an L-sweep with $L \in \{-0.05, -0.18, -0.35\}$, uses `writeValue(5)` for a wrong initial estimate, and logs distance, $\hat x$, and innovation to CSV files.  
  - MATLAB: `estOnlyRuns` + `plotEstimatorOnly` are ready to generate the required plots once CSV paths are provided.  
  - **Minor issue:** In `robot.cpp`, the estimator gain override (`lOverride`) is only applied inside `if(controlEnabled())`. For pure estimator-only experiments (controller disabled), runtime L-sweeps via channel 11 will not take effect unless this logic is moved outside the controller block or gains are changed offline.

- **Controller only (2b)**  
  - Spec requires: estimator off, controller uses raw position, step reference for several K values, plots of responses and control signals, explanation vs 1(c).  
  - Arduino: when `StateEstimationEnabled()` is false and `controlEnabled()` is true, `feedbackPos = -frontDistance`, so the outer loop uses the raw measurement without the estimator, as required. Channel 10 (`kOverride`) cleanly implements the K-sweep.  
  - Plan: Section 2 defines a K-sweep with $K \in \{80,120,250\}$, a safe step reference, and the correct set of logged signals.  
  - MATLAB: `ctrlOnlyRuns` + `plotControllerOnly` generate the measured step responses and corresponding motor voltages (`controller_K_response.pdf`, `controller_K_voltage.pdf`).  
  - LaTeX: Section 2.2 already reflects the qualitative trends; the remaining TODOs are the numerical settling times and overshoot numbers to be pulled from the experimental figures.

- **Estimator + controller (2c)**  
  - Spec requires: estimator pole 10× slower than controller pole, good/wrong initial estimate experiments under a step, and analysis of whether estimator performance affects control performance (separation principle).  
  - Design: `assignment3_solution.m` sets `K_nom = 120`, `L_slow = -0.004`, and `assignment3.tex` explains the 10× slower pole placement and the resulting poles $\{p_\text{cl}(K_\text{nom}), p_\text{est}(L_\text{slow})\}$.  
  - Arduino: runtime overrides (channels 10 and 11) allow implementing $(K_\text{nom}, L_\text{slow})$ while both controller and estimator are enabled.  
  - Plan: Section 3 prescribes good/wrong initial estimate runs and the corresponding data files, aligned with the spec.  
  - MATLAB/LaTeX: `estCtrlRuns` + `plotEstimatorController` generate `est_ctrl_good.pdf` and `est_ctrl_bad.pdf` for the LaTeX placeholders; the LaTeX text already articulates the separation principle, and only minor expansion is needed if more explicit reference to the closed-loop transfer function is desired.

### 8.3 Consistency notes and remaining work

- **Firmware path**  
  - The plan refers to `arduino_files/ass3_ino/CT-StateEstimation_StateFeedback.ino`, whereas the repository currently contains `arduino_files/CT-StateEstimation_StateFeedback/CT-StateEstimation_StateFeedback.ino`. These should be aligned (either by updating the path in the plan or by adding the `ass3_ino` wrapper used in other assignments).

- **Channel 5 usage**  
  - The plan uses channel 5 as an input for a "forced estimator initial guess" (`writeValue(5)`), while `robot.cpp` both reads `readValue(5)` in `resetStateEstimator()` and writes `writeValue(5, speedB)`. Using the same channel as both input and telemetry can be confusing or error-prone; separating these roles (e.g. a dedicated read channel for the initial estimate, and a different channel for speed logging) would make the experiments more robust.

- **Estimator gain override in estimator-only mode**  
  - As noted above, the L override is currently only applied when the controller is enabled. If strict compliance with the 2(a) spec (estimator only, controller truly off) is important, consider allowing `lOverride` to update the estimator gain even when `controlEnabled() == false`.

- **Experiment and report completion**  
  - All analytical pieces required by `specs_assignment3.md` are in place and consistent across Arduino, MATLAB, and LaTeX. The remaining work is operational: collect the CSVs listed in Section 7, populate `estOnlyRuns`, `ctrlOnlyRuns`, and `estCtrlRuns`, regenerate figures, and replace the LaTeX placeholders/TODOs with measured metrics and final wording.
