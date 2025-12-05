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

3. **Gain configuration (hardcoded in `init()`)**
   - Reference input (distance setpoint) via `writeValue(0)` (meters) **when the estimator is enabled**.
   - **K and L are now hardcoded** in `Robot::init()`. To change gains, edit the file and re-flash:
     ```cpp
     // 2(a) L sweep: uncomment one
     // setEstimatorGain(-0.05f);   // L_slow
     // setEstimatorGain(-0.18f);   // L_nom
     // setEstimatorGain(-0.35f);   // L_fast
     
     // 2(b) K sweep: uncomment one
     // kPosGain = 20.0f;           // K_slow
     kPosGain = 40.0f;              // K_nom  <-- default
     // kPosGain = 80.0f;           // K_fast
     
     // 2(c): use kPosGain = 40.0f + setEstimatorGain(-0.00132f)
     setEstimatorGain(-0.00132f);   // L_slow active for spec 2(c)
     ```
   - **Initial estimate is hardcoded** to −0.40 m in `resetStateEstimator()` (no runtime override).

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
     1. Edit `init()` in `robot.cpp` to uncomment the desired L value.
     2. Re-flash the Arduino.
     3. Press button 1 to reset estimator (starts at hardcoded −0.40 m).
     4. Record ≥10 s of data.

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
   - **Step reference is hardcoded in `robot.cpp` to 0.25 m distance to the wall** for controller-only runs (no need to command it via channel 0).

2. **K sweep**
   - Gains `K ∈ {20, 40, 80}` rad/(s·m).
   - For each K:
     1. Edit `init()` in `robot.cpp` to uncomment the desired K value.
     2. Re-flash the Arduino.
     3. Ensure the cart starts from a distance smaller than 0.25 m so that the hardcoded 0.25 m step is inside the IR sensor range.
     4. Record until response settles (≈6 s).

3. **Collected signals**
   - time, reference (ch0 stored separately), measured distance (ch1), motor voltages (ch6–7), wheel speeds (ch4–5), v\_ref (ch3).

4. **File naming**
   - `matlab_assign3/data/ctrl_only_K020.csv`, `…K040.csv`, `…K080.csv`.

5. **Observations to capture**
   - Rise time, overshoot, saturation events, steady-state error.

---

## 3. Combined Estimator + Controller (Spec 2(c))

Goal: Demonstrate separation principle and effect of slow estimator.

1. **Setup**
   - Enable both controller (button 0) and estimator (button 1).
   - Set position reference (e.g., 0.25 m) via channel 0; this is independent of the hardcoded controller-only reference.

2. **Gain selection**
   - Edit `init()` in `robot.cpp`:
     ```cpp
     kPosGain = 40.0f;             // K_nom
     setEstimatorGain(-0.00132f);  // L_slow: 10x slower than controller (p_est ~= 0.9987)
     ```
   - Re-flash the Arduino.
   - **Verification:** p_cl = 0.9868 and p_est = 0.9987 so (1 - p_est) = (1 - p_cl)/10.

3. **Runs**
   - **Wrong initial estimate**: press button 1 to reset estimator (starts at hardcoded -0.40 m).
   - For the "good" run, you may temporarily change `kWrongInitialEstimate` in `robot.cpp` to match the actual cart position, or simply accept that the hardcoded value is always used.
   - Log at least 10 s covering the step response.

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
- [x] **Fix controller-only mode** in `robot.cpp` (Issue 1 above) 
- [x] Update `L_slow` constant -> now -0.00132 in Arduino, MATLAB, and LaTeX 

### Data Collection (all CSVs in `matlab_assign3/data/`)
- [ ] `est_only_L-005.csv` (L = -0.05)
- [ ] `est_only_L-018.csv` (L = -0.18)
- [ ] `est_only_L-035.csv` (L = -0.35)
- [ ] `ctrl_only_K020.csv` (K = 20)
- [ ] `ctrl_only_K040.csv` (K = 40)
- [ ] `ctrl_only_K080.csv` (K = 80)
- [ ] `est_ctrl_good.csv` (correct initial estimate)
- [ ] `est_ctrl_bad.csv` (wrong initial estimate)

### MATLAB Processing
- [ ] Populate `estOnlyRuns`, `ctrlOnlyRuns`, `estCtrlRuns` arrays with CSV paths
- [ ] Generate figures: `estimator_L_sweep.pdf`, `controller_K_response.pdf`, `controller_K_voltage.pdf`, `est_ctrl_good.pdf`, `est_ctrl_bad.pdf`
- [ ] Update MATLAB pole map for L_slow = -0.00132 in Section 2(c) discussion

### LaTeX Report (`assignment3.tex`)
- [ ] Replace figure placeholders with `\includegraphics{images/...}`
- [ ] Fill in TODO placeholders:
  - Settling time for L_nom (Section 2.1 observations)
  - RMS innovation for L_nom
  - Settling time for K_nom (Section 2.2 observations)
- [ ] Update $L_\text{slow}$ value from -0.02 to -0.00132 in Section 2.3
- [ ] Final proofreading and consistency check

## 8. Review vs. `specs_assignment3.md`

### 8.1 Theoretical design (Task 1)

- **State and measurement models (1a, 1b)**  
  - `assignment3.tex` derives the continuous-time model, applies forward Euler, and states $A_d = 1$, $B_d = T_s r = 3.3\times10^{-4}$ m/rad, matching `specs_assignment3.md` and `assignment3_solution.m` (`A = 1; B = Ts * r;`).  
  - The measurement equation $y = -x$ with $C = -1$, $D = 0$ is used consistently in LaTeX, MATLAB (`C = -1; D = 0;`), and Arduino (`frontDistance` is positive, `feedbackPos = -frontDistance` when the estimator is disabled).

- **State feedback gain K (1c)**  
  - LaTeX correctly derives $p_\text{cl}(K) = 1 - T_s r K$ and discusses stability/response vs K.  
  - MATLAB implements the required pole map and simulated step responses: `p_cl = 1 - B * K_sweep;` with `K_sweep = [20, 40, 80]` and exports `pole_map_K.pdf` and `step_K_sweep.pdf`, as requested.  
  - The chosen nominal gain $K_\text{nom} = 40$ rad/(s·m) is documented and used consistently in MATLAB and in the plan (K-sweep section).

- **Estimator gain L (1d)**  
  - LaTeX derives $p_\text{est}(L) = 1 + L$ and explains stability and noise/convergence trade-offs in line with the spec.  
  - MATLAB sweeps `L_sweep = [-0.05, -0.18, -0.35]` and exports `pole_map_L.pdf`; $L_\text{nom} = -0.18$ and the sweep set are explicitly stated in LaTeX.  
  - Overall, the analytical part of 1(a)–1(d) is complete and consistent across LaTeX and MATLAB; remaining gaps are mainly numerical values to be filled from experiments.

### 8.2 Implementation and experiments (Task 2)

- **Estimator only (2a)**  
  - Spec requires: controller off, estimator on, wrong initial $\hat x[0]$, multiple L values, plots of measured vs estimated distance, linked to 1(d).  
  - Arduino: `resetStateEstimator()` uses a hardcoded wrong initial estimate of −0.40 m. With button 0 off and button 1 on, motors are held at zero voltage while the estimator runs, matching an "estimator only" mode.  
  - Plan: Section 1 prescribes an L-sweep with $L \in \{-0.05, -0.18, -0.35\}$, uses the hardcoded wrong initial estimate (−0.40 m), and logs distance, $\hat x$, and innovation to CSV files.  
  - MATLAB: `estOnlyRuns` + `plotEstimatorOnly` are ready to generate the required plots once CSV paths are provided.  
  - **Resolved:** K and L are now hardcoded in `init()` with clear comments. Runtime override via QRC channels 10/11 has been removed for reproducibility. Change gains by editing `init()` and re-flashing.

- **Controller only (2b)**  
  - Spec requires: estimator off, controller uses raw position, step reference for several K values, plots of responses and control signals, explanation vs 1(c).  
  - Arduino: when `StateEstimationEnabled()` is false and `controlEnabled()` is true, `feedbackPos = -frontDistance`, so the outer loop uses the raw measurement without the estimator, as required. K is hardcoded in `init()`.  
  - Plan: Section 2 defines a K-sweep with $K \in \{20,40,80\}$, a safe step reference, and the correct set of logged signals.  
  - MATLAB: `ctrlOnlyRuns` + `plotControllerOnly` generate the measured step responses and corresponding motor voltages (`controller_K_response.pdf`, `controller_K_voltage.pdf`).  
  - LaTeX: Section 2.2 already reflects the qualitative trends; the remaining TODOs are the numerical settling times and overshoot numbers to be pulled from the experimental figures.

- **Estimator + controller (2c)**  
  - Spec requires: estimator pole 10× slower than controller pole, good/wrong initial estimate experiments under a step, and analysis of whether estimator performance affects control performance (separation principle).  
  - Design: `assignment3_solution.m` sets `K_nom = 40`, computes `L_slow = -(1 - p_cl)/10 = -0.00132`, and `assignment3.tex` explains the 10× slower pole placement and the resulting poles $\{p_\text{cl}(K_\text{nom}), p_\text{est}(L_\text{slow})\}` (0.9868, 0.9987).  
  - Arduino: K and L are hardcoded in `init()`. For 2(c), set `kPosGain = 40.0f` and `setEstimatorGain(-0.00132f)`, then re-flash.  
  - Plan: Section 3 prescribes good/wrong initial estimate runs and the corresponding data files, aligned with the spec.  
  - MATLAB/LaTeX: `estCtrlRuns` + `plotEstimatorController` generate `est_ctrl_good.pdf` and `est_ctrl_bad.pdf` for the LaTeX placeholders; the LaTeX text already articulates the separation principle, and only minor expansion is needed if more explicit reference to the closed-loop transfer function is desired.

### 8.3 Consistency notes and remaining work

- **Firmware path**  
  - The plan refers to `arduino_files/ass3_ino/CT-StateEstimation_StateFeedback.ino`, whereas the repository currently contains `arduino_files/CT-StateEstimation_StateFeedback/CT-StateEstimation_StateFeedback.ino`. These should be aligned (either by updating the path in the plan or by adding the `ass3_ino` wrapper used in other assignments).

- **Channel 5 usage**  
  - Channel 5 is now used exclusively for `writeValue(5, speedB)` telemetry. The initial estimate is hardcoded to −0.40 m in `resetStateEstimator()`, eliminating the previous read/write conflict.

- **Experiment and report completion**  
  - All analytical pieces required by `specs_assignment3.md` are in place and consistent across Arduino, MATLAB, and LaTeX. The remaining work is operational: collect the CSVs listed in Section 7, populate `estOnlyRuns`, `ctrlOnlyRuns`, and `estCtrlRuns`, regenerate figures, and replace the LaTeX placeholders/TODOs with measured metrics and final wording.
