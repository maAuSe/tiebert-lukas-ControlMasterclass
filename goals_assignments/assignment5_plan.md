# Assignment 5 EKF + LQR Experiment Plan

Workflow to collect the data requested by `specs_assignment5.md`, process it in MATLAB, and fill the report/template. This plan focuses on the experimental EKF and LQR parts (Sections 3 and 4 of `specs_assignment5.md`); the modelling and derivation questions (Sections 1 and 2, 3a, 4a, 4b) are handled directly in `matlab_assign5/assignment5_solution.m` and `tex_control/ass5_tex/assignment5.tex`. Paths assume this repo layout and the Arduino/QRC channel map below.

---

## 0. Preparation

1. **Geometry (once per cart, already measured)**
   - `a` (half wheelbase) is `WHEELBASE/2 = 0.083 m` in `mecotron.h`.
   - Measured offsets for our cart (must match `assignment5_solution.m`):
     - `alpha = 0.075 m` - center to front IR along X'.
     - `beta  = 0.065 m` - center to lateral IR along X'.
     - `gamma = 0.078 m` - center to lateral IR along Y' (positive to the sensor side).
   - Arena geometry: default corner at `x=0`, `y=0` with robot starting in `x<0, y<0`. If the real walls are not exactly at `x=0` or `y=0`, measure and update the wall parameters `(p,q,r)` consistently in MATLAB and firmware.

2. **Mark the world frame on the lab floor**
   - Use tape or markers to define:
     - The corner where the two walls meet as `(0,0)`.
     - The positive `X` axis along the wall used for the front sensor (`y = 0`).
     - The positive `Y` axis along the wall used for the lateral sensor (`x = 0`).
   - Mark the **start pose** of the cart center at `(-0.30, -0.20)` m:
     - From the corner, measure `30 cm` along `-X` and `20 cm` along `-Y` and place a cross.
     - When placing the cart, align the **geometric center** of the axle with this cross and point the cart along `+X` (front sensor facing the `y=0` wall).
   - Optionally mark the **target pose** near `(-0.15, -0.35)` m to visually check end position.

3. **Firmware focus (Section 3b only)**
   - Firmware is currently *feedforward-only* with the EKF (no LQR/state feedback). For the four Q/R sweeps, edit `arduino_files/CT-EKF-Swivel/extended_kalman_filter.cpp` and set:
     - `kQScale` and `kRScale` to `1` or `5` per run (multipliers on the diagonal Q/R values).
   - If you later need LQR experiments (spec 4c), restore the LQR feedback code before running those tests.

4. **QRC channel map (configure once, log for every run)**
   - Current firmware (`arduino_files/CT-EKF-Swivel/robot.cpp`) streams **12** general-purpose outputs (`ValueIn0`…`ValueIn11`), which is what `matlab_assign5/assignment5_solution.m` parses:
     - ch0: `v_ff` = `trajectory.v()`
     - ch1: `omega_ff` = `trajectory.omega()`
     - ch2: `z1` (front IR distance), `NaN` when sensors are off
     - ch3: `z2` (side IR distance), `NaN` when sensors are off
     - ch4: `xhat` (EKF)
     - ch5: `yhat` (EKF)
     - ch6: `thetahat` (EKF)
     - ch7: `Pxx` (EKF covariance diagonal)
     - ch8: `Pyy` (EKF covariance diagonal)
     - ch9: `Ptt` (EKF covariance diagonal)
     - ch10: `v` (applied input), `0` when control is disabled
     - ch11: `omega` (applied input), `0` when control is disabled
   - In MATLAB, `hasMeas` is inferred from `isfinite(z1) & isfinite(z2)`, and `x_ref, y_ref, theta_ref` are reconstructed by integrating `v_ff, omega_ff` from the known start pose (no dedicated QRC channels for them).
   - In QRC, set the default log export folder to `matlab_assign5/data/` so that CSVs can be saved directly with the filenames expected by `assignment5_solution.m`.

5. **Buttons and modes (verify behaviour once)**
   - Button 0: toggles the velocity loop + trajectory feedforward (state-feedback is currently disabled).
   - Button 1: toggles and resets the EKF (reinitialises `_xhat` and `_Phat`).
   - Button 2: starts/stops trajectory playback.
   - Button 3: resets the trajectory pointer to the start.
   - With the cart on a stand or with wheels lifted, briefly press each button and confirm in QRC that the expected messages/behaviour occur before running real experiments.

## 1. EKF Q/R Sweep (Spec 3b)
Goal: collect four EKF data sets (different Q/R ratios) for the **same feedforward-only trajectory**, then compare convergence and bias.

1. **Choose and program the Q/R combination for this run**
   - In the firmware (`arduino_files/CT-EKF-Swivel/extended_kalman_filter.cpp`), set the scalar multipliers `kQScale` and `kRScale` so they match one of the four combinations used in `assignment5_solution.m`:
     - Run `Q1_R1`:   `Q = Q_proc_nom`,   `R = R_meas_nom` (nominal).
     - Run `Q5_R1`:   `Q = 5 * Q_proc_nom`, `R = R_meas_nom`.
     - Run `Q1_R5`:   `Q = Q_proc_nom`,   `R = 5 * R_meas_nom`.
     - Run `Q5_R5`:   `Q = 5 * Q_proc_nom`, `R = 5 * R_meas_nom`.
   - Control is already pure feedforward; no K gains are applied. Compile and **reflash the firmware** after setting the desired Q/R for the current run.

2. **Physical execution and logging (repeat once for each of the four Q/R combinations)**
   - Before the first sweep run of the session:
     - Power the cart off, place it by hand with wheels free to roll, and power it on.
     - In QRC, connect to the cart and confirm channels and buttons respond.
   - For each combination (`Q1_R1`, `Q5_R1`, `Q1_R5`, `Q5_R5`), perform the following steps **in order**:
     1. In QRC, create or open a log and set the experiment name (e.g. `ekf_Q1_R1`).
        - Configure the log to export a CSV to `matlab_assign5/data/ekf_Q1_R1.csv` (or the matching
          name for the current run).
     2. Physically place the cart at the **start mark** `(-0.30, -0.20)` m, facing exactly along `+X`:
        - Align the axle center with the floor mark made in Section 0.2.
        - Ensure the front IR sensor points straight at the `y = 0` wall and the lateral sensor toward the `x = 0` wall.
     3. Make sure the area along the straight-turn-straight path is clear and both IR sensors see their walls.
     4. In QRC, **start logging** (so the entire run is captured, including the button presses).
     5. On the cart, press **Button 1** once to reset and enable the EKF; wait ~1-2 seconds.
     6. Press **Button 0** to enable the velocity loop and controller.
     7. Press **Button 2** to start the trajectory playback.
        - Do **not** touch the cart during the run; it will execute the built-in straight-turn-straight profile.
     8. Wait until the cart has completed the turn and comes to a halt near the target region
        (around `(-0.15, -0.35)` m) and QRC shows the trajectory finished.
     9. Press **Button 2** again to stop the trajectory if needed, or **Button 3** to reset the pointer.
    10. In QRC, **stop logging** and export the CSV using the exact filename expected by MATLAB:
        - `matlab_assign5/data/ekf_Q1_R1.csv` for the nominal run.
        - `ekf_Q5_R1.csv`, `ekf_Q1_R5.csv`, `ekf_Q5_R5.csv` for the other three runs.
    11. Write a short note (in your lab notebook or comments) about what you observed for this Q/R
        combination: overshoot, bias during dead reckoning, smoothness of heading estimate, etc.

3. **Files and labels (must match MATLAB script)**
   - After completing all four runs, verify that the following files exist in `matlab_assign5/data/`:
     - `ekf_Q1_R1.csv` (nominal `Q_proc_nom`, `R_meas_nom`).
     - `ekf_Q5_R1.csv`, `ekf_Q1_R5.csv`, `ekf_Q5_R5.csv` for the scaled Q/R ratios.

4. **Notes to capture for the report**
   - For each run, note:
     - How quickly the estimate locks onto the wall-based measurements.
     - Behaviour at the moment when sensors are disabled (transition to dead reckoning).
     - Any visible drift or bias during the sensor-off phase.
     - Whether larger `Q` or larger `R` gives faster convergence versus noisier estimates.

---

## 2. EKF Uncertainty & Measurement Plots (Spec 3c)
1. **Select the nominal Q/R based on Section 1 plots**
   - After processing the four Q/R runs in MATLAB (Section 4), decide which Q/R gives:
     - Small bias in position and heading.
     - Reasonable noise level and stable covariance.
   - Set `ekfNominalIdx` in `assignment5_solution.m` to the index of this chosen run.

2. **Record one clean nominal EKF run for uncertainty analysis**
   - Reprogram the firmware with the **chosen** Q/R combination (if it is not the nominal `Q1_R1`).
   - Repeat the exact physical procedure from Section 1.2 for a single run, with the following details:
     - Use the same **start pose** `(-0.30, -0.20)` m and button sequence (1 -> 0 -> 2).
     - In QRC, log to the filename that corresponds to the chosen nominal run (e.g. `ekf_Q1_R1.csv`).
     - Confirm during the run that channels 7-9 (`Pxx, Pyy, Ptt`) are non-zero and evolving.

3. **Generate state and measurement plots in MATLAB**
   - In MATLAB, run `assignment5_solution.m` with the updated `ekfExps` and `ekfNominalIdx`.
   - Check that the following figures are produced in `tex_control/ass5_tex/images/`:
     - `ekf_state{1,2,3}_QR_sweep.pdf` (state overlays across Q/R choices).
     - `ekf_state{1,2,3}_95ci_plotstates.pdf` (95% confidence bands for the chosen nominal run).
     - `ekf_measurement{1,2}_95ci.pdf` (95% confidence bands for the two IR measurements).

4. **Observations to write down for the report**
   - How the uncertainty (confidence intervals) behaves:
     - While both sensors are active.
     - During the turn when `hasMeasurements = 0` (prediction only).
     - After measurements resume.
   - How the plots would change if the sensors were **not** disabled during the turn:
     - Discuss qualitatively what would happen to the measurement equations and residuals.

---

## 3. LQR Tracking Experiments (Spec 4c)
Goal: closed-loop tracking of the **same reference trajectory** as in the EKF experiment, with different LQR weightings `Q_\ell, R_\ell`, **without feedforward** for these tests.

> Current firmware is feedforward-only. To run this section, restore the LQR feedback path (K matrix and body-frame error computation) in `robot.cpp` and reintroduce the feedforward/feedback split before flashing.

1. **Compute K for each LQR setting in MATLAB**
   - In `assignment5_solution.m`, choose a set of diagonal weights for the error states and inputs:
     - `Q_lqr = diag([q_x, q_y, q_theta])` and `R_lqr = diag([r_v, r_omega])`.
   - For each of up to four combinations (A-D):
     1. Set `Q_lqr`, `R_lqr` in the script.
     2. Run the script to compute `K_lqr = dlqr(Ad, Bd, Q_lqr, R_lqr)`.
     3. Copy the numerical matrix `K_lqr` into `robot.cpp` by updating the `Kfb` initialization (e.g. `Kfb = {k11, k12, k13, k21, k22, k23};`).
        - The current default is approximately `[[-3.1127, 0, 0]; [0, 3.1393, -1.4483]]`.
   - For the **no-feedforward** tests required in Spec 4(c):
     - In `robot.cpp`, temporarily set the feedforward vector to zero in `control()` (e.g. `uff.Fill(0)`), so the cart is driven only by the LQR feedback.

2. **Runs and filenames**
   - Plan up to four runs with different `(Q_\ell, R_\ell)` choices and consistent file naming:
     - `lqr_A.csv`, `lqr_B.csv`, `lqr_C.csv`, `lqr_D.csv` in `matlab_assign5/data/`.
   - Ensure each run uses a **different** weighting according to the combinations defined in the MATLAB script.

3. **Physical execution for each LQR run (no feedforward)**
   - Compile and **reflash the firmware** after updating `Kfb` and disabling feedforward.
   - For each LQR weighting (A-D):
     1. In QRC, configure the log to export to the corresponding file:
        - `matlab_assign5/data/lqr_A.csv`, etc.
     2. Place the cart at the same **start pose** `(-0.30, -0.20)` m, facing `+X`.
     3. Start logging in QRC.
     4. Press **Button 1** to enable/reset the EKF (so the controller uses a consistent estimate).
     5. Press **Button 0** to enable the LQR controller.
     6. Press **Button 2** to start the trajectory.
        - The cart now tracks the reference using **state feedback only** (no feedforward contribution).
     7. Let the full straight-turn-straight profile finish; do not interfere unless safety requires it.
     8. Stop logging in QRC when the cart has stopped near the expected end region.
   - After completing all LQR experiments, **restore** the original feedforward behaviour in `robot.cpp` for normal operation (undo the `uff.Fill(0)` change).

4. **Signals to inspect (for each LQR run)**
   - In MATLAB, using `assignment5_solution.m`:
     - World-frame tracking errors `e_x, e_y, e_\theta` over time.
     - Body-frame tracking errors (after rotation), which are the ones actually penalised by the LQR design.
     - Control signals `v` and `\omega` (either directly or via the logged feedforward channels as proxies).
   - Compare across Q/R combinations:
     - Convergence speed vs. oscillations.
     - Whether wheel voltages approach saturation.

---

## 4. Data Processing in MATLAB
1. **Check geometry and Q/R settings in the script**
   - Open `matlab_assign5/assignment5_solution.m` and verify:
     - `a, alpha, beta, gamma` and wall parameters match the measured values and arena.
     - `Q_proc_nom`, `R_meas_nom`, and the LQR weights `Q_lqr`, `R_lqr` correspond to the experiments you actually ran.

2. **Verify filenames in `ekfRuns` and `lqrRuns`**
   - Ensure the `ekfRuns` array lists exactly the four EKF CSV filenames you recorded in Section 1.
   - Ensure the `lqrRuns` array lists the LQR CSV filenames (`lqr_A.csv`-`lqr_D.csv`) you recorded in Section 3.

3. **Run the script and generate all figures**
   - In MATLAB, run `assignment5_solution.m` from the project root so that paths resolve correctly.
   - Confirm that the script completes without errors and exports the following figures to `tex_control/ass5_tex/images/`:
     - `ekf_state{1,2,3}_QR_sweep.pdf`.
     - `ekf_state{1,2,3}_95ci_plotstates.pdf`.
     - `ekf_measurement{1,2}_95ci.pdf`.
     - `lqr_tracking_errors_world.pdf`.
     - `lqr_tracking_errors_body.pdf`.
     - `lqr_control_signals.pdf`.

4. **Decide on final Q/R and K values for the report**
   - From the EKF plots, pick the Q/R combination that gives the best trade-off between noise and bias and record the numerical `Q`, `R`, and `\hat{P}_{0|0}`.
   - From the LQR plots, pick the `(Q_\ell, R_\ell)` that yields acceptable tracking performance without excessive control effort and record the final `K` matrix.

---

## 5. Report Integration Checklist

- Fill measured values for `a, alpha, beta, gamma` and wall equations.
- Insert exported figures into `tex_control/ass5_tex/assignment5.tex` placeholders.
- Document chosen `Q`, `R`, `P0`, and qualitative trends (drift during dead-reckoning, effect of higher R, etc.).
- Summarize the selected `K` matrix from MATLAB and the rationale for final Q/R in LQR.
