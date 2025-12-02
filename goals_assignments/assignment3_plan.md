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
   - Estimator: set slow pole (`L = -0.02`) via channel 11 (≈10× slower than control pole).

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

Following this plan ensures every bullet in `specs_assignment3.md` has corresponding data, plots, and written analysis, while keeping the workflow reproducible.*** End Patch
