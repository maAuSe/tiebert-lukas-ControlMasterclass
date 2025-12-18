# Assignment 5 LQR Experiment Plan

Workflow to collect LQR tracking data for `specs_assignment5.md` Section 4c, process in MATLAB, and fill the report. EKF experiments are complete; this plan covers only the remaining LQR tracking experiments.

---

## 0. Preparation

1. **Geometry (already measured)**
   - `a = 0.083 m`, `alpha = 0.075 m`, `beta = 0.065 m`, `gamma = 0.078 m`
   - Arena corner at `(0,0)`, cart starts in `x<0, y<0`

2. **Start pose**: `(-0.30, -0.20)` m, heading along `+X`

3. **QRC channel map (channels 0-11)**
   | Ch | Signal | Ch | Signal |
   |----|--------|----|--------|
   | 0 | `x_ref` | 6 | `e'_x` (body error) |
   | 1 | `y_ref` | 7 | `e'_y` (body error) |
   | 2 | `theta_ref` | 8 | `e'_theta` (body error) |
   | 3 | `xhat` | 9 | `v_applied` |
   | 4 | `yhat` | 10 | `omega_applied` |
   | 5 | `thetahat` | 11 | `z1` (debug) |

4. **Buttons**
   - Button 0: toggle velocity loop + controller
   - Button 1: reset/enable EKF
   - Button 2: start/stop trajectory
   - Button 3: reset trajectory pointer

---

## 1. LQR Tracking Experiments (Spec 4c)
Goal: closed-loop tracking of the **same reference trajectory** as in the EKF experiment, with different LQR weightings `Q_lqr, R_lqr`, **without feedforward** for these tests.

> **FIRMWARE STATUS**: LQR feedback is now fully implemented in `robot.cpp`. All configuration is centralized in `extended_kalman_filter.cpp`.

### 1.1 Firmware Configuration

The LQR gains are set directly in `arduino_files/CT-EKF-Swivel/robot.cpp` in `resetLqrController()`:

```cpp
// LQR gain matrix K (2x3) from dlqr(Ad, Bd, Q_lqr, R_lqr)
_Klqr = {-3.1127f, 0.0f, 0.0f,
          0.0f, 3.1393f, -1.4483f};
```

The firmware is **feedback-only** (no feedforward). This matches spec 4c requirements.

### 1.2 Compute K Matrix in MATLAB (For Each Q/R Combination)

1. Open `matlab_assign5/assignment5_solution.m`
2. Set `Q_lqr` and `R_lqr` for the current run:
   ```matlab
   % Run A: baseline
   Q_lqr = diag([4, 4, 0.8]);      % penalize position/heading errors
   R_lqr = diag([0.4, 0.4]);       % penalize control effort
   K_dlqr = dlqr(Ad, Bd, Q_lqr, R_lqr);
   K_firmware = -K_dlqr;           % NEGATE! See note below.
   disp(K_firmware);               % Copy these values to firmware
   ```
3. **⚠️ CRITICAL SIGN CONVENTION**: `dlqr` returns K for `u = -K*x`, but firmware uses `u = K*e'`.
   You must **NEGATE** the K matrix when copying to Arduino!
4. Copy `K_firmware` (negated!) to `robot.cpp` → `resetLqrController()`:
   ```cpp
   _Klqr = {K_firmware(1,1), K_firmware(1,2), K_firmware(1,3),
            K_firmware(2,1), K_firmware(2,2), K_firmware(2,3)};
   ```

### 1.3 LQR Q/R Sweep Combinations

| Run   | Q_lqr                    | R_lqr              | Expected Behavior                    |
|-------|--------------------------|--------------------|------------------------------------- |
| **A** | `diag([4, 4, 0.8])`      | `diag([0.4, 0.4])` | Baseline: balanced tracking          |
| **B** | `diag([8, 8, 1.6])`      | `diag([0.4, 0.4])` | Higher state penalty → faster conv.  |
| **C** | `diag([4, 4, 0.8])`      | `diag([0.8, 0.8])` | Higher input penalty → slower/smooth |
| **D** | `diag([16, 16, 3.2])`    | `diag([0.2, 0.2])` | Aggressive: fast but may overshoot   |

### 1.4 Physical Execution Checklist (Per LQR Run)

For **each** of the four runs (A, B, C, D):

1. **In MATLAB**: compute `K_lqr` for the current Q/R combination
2. **In firmware** (`robot.cpp` → `resetLqrController()`):
   - Update `_Klqr = {k11, k12, k13, k21, k22, k23};` with the new K values
3. **Compile and flash** the firmware to the cart
4. **In QRC**:
   - Create a new log, set export filename: `matlab_assign5/data/lqr_A.csv` (or B/C/D)
   - Configure logging for channels 0-11
5. **Place the cart** at start pose `(-0.30, -0.20)` m, heading along `+X`
6. **Start logging** in QRC
7. **Press Button 1** to reset and enable the EKF
8. **Press Button 0** to enable the controller
9. **Press Button 2** to start the trajectory
10. **Wait** for the trajectory to complete (cart will track: straight → turn → straight)
11. **Stop logging** in QRC, verify CSV was saved
12. **Note observations**: overshoot, oscillations, final position error

### 1.5 QRC Channel Map

| Channel | Signal         | Description                          |
|---------|----------------|--------------------------------------|
| 0       | `x_ref`        | Reference x from trajectory          |
| 1       | `y_ref`        | Reference y from trajectory          |
| 2       | `theta_ref`    | Reference θ from trajectory          |
| 3       | `xhat`         | EKF estimated x [m]                  |
| 4       | `yhat`         | EKF estimated y [m]                  |
| 5       | `thetahat`     | EKF estimated θ [rad]                |
| 6       | `e'_x`         | Body-frame error (longitudinal)      |
| 7       | `e'_y`         | Body-frame error (lateral)           |
| 8       | `e'_theta`     | Body-frame error (heading)           |
| 9       | `v_applied`    | LQR feedback v [m/s]                 |
| 10      | `omega_applied`| LQR feedback ω [rad/s]               |
| 11      | `z1`           | Front IR (debugging)                 |

### 1.6 Expected Output Files

After completing all runs, verify:
- `matlab_assign5/data/lqr_A.csv`
- `matlab_assign5/data/lqr_B.csv`
- `matlab_assign5/data/lqr_C.csv`
- `matlab_assign5/data/lqr_D.csv`

### 1.7 Signals to Inspect in MATLAB

1. **World-frame tracking errors**: `e_x = x_ref - xhat` (ch0 - ch3), `e_y` (ch1 - ch4), `e_θ` (ch2 - ch5)
2. **Body-frame tracking errors**: `e'_x` (ch6), `e'_y` (ch7), `e'_θ` (ch8)
3. **Control signals**: `v_applied` (ch9), `ω_applied` (ch10)
4. **Comparisons across runs**:
   - Convergence speed (how fast errors go to zero)
   - Overshoot and oscillations
   - Control effort magnitude (saturation risk)

---

## 2. Data Processing in MATLAB

1. **Verify `lqrRuns` filenames** in `assignment5_solution.m` match your CSV files (`lqr_A.csv`–`lqr_D.csv`)

2. **Run the script** and confirm these figures are exported to `tex_control/ass5_tex/images/`:
   - `lqr_tracking_errors_world.pdf`
   - `lqr_tracking_errors_body.pdf`
   - `lqr_control_signals.pdf`

3. **Select final LQR gains**: pick the `(Q_lqr, R_lqr)` that yields acceptable tracking without excessive control effort; record the final `K` matrix

---

## 3. Report Integration

- Insert LQR figures into `tex_control/ass5_tex/assignment5.tex`
- Summarize the selected `K` matrix and rationale for final Q/R choice
- Note qualitative observations: convergence speed, overshoot, oscillations across runs
