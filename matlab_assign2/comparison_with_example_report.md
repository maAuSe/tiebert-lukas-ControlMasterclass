# Comparison: `assignment2_solution.m` vs Example Report Scripts

This document compares the MATLAB implementation in `matlab_assign2/assignment2_solution.m` with the example report scripts (`Example reports/Assigment 2/Matlab/ass2.m` and `graphs_2c.m`).

---

## 0. Step-by-Step Workflow in Our Solution (`assignment2_solution.m`)

The script executes the following steps in order:

### Step 1: Configuration (Lines 14–48)
1. Clear workspace, close figures, set format.
2. Define sampling time `Ts = 0.01 s` (100 Hz).
3. Create **nominal design spec** struct: phase margin 55°, phase-lag reserve 15°, target crossover 30 rad/s.
4. Create **low-bandwidth design spec** struct: same margins, target crossover ≈ 3.14 rad/s (0.5 Hz).
5. Set `exportPlots` flag and define paths for data files and figure output directory.

### Step 2: Load Plant Models from Assignment 1 (Lines 50–76)
1. Define discrete-time transfer functions for **Wheel A** and **Wheel B** (2nd-order with z⁻² delay).
2. For each motor:
   - Set input/output names (`"Voltage"`, `"Velocity"`).
   - Convert discrete TF to continuous using Tustin (`d2c(..., 'tustin')`).
   - Store both representations in `motorModels` struct.

### Step 3: Design PI Controllers (Lines 78–108)
1. For each motor, call `designPiController()` twice:
   - Once with **nominal spec** (high bandwidth).
   - Once with **low-bandwidth spec**.
2. `designPiController()` internally:
   - Computes Bode response of continuous plant over 0.5–500 rad/s.
   - Calculates integrator time constant: `Ti = tan(90° - phaseLagReserve) / ωc`.
   - Builds continuous PI: `C(s) = K * (Ti*s + 1) / (Ti*s)`.
   - Evaluates open-loop magnitude at ωc and sets `K = 1 / |L(jωc)|`.
   - Forms open-loop `L(s)`, closed-loop `T(s)`, sensitivity `S(s)`, and control TF.
   - Computes gain/phase margins via `margin()`.
   - Discretizes controller with Tustin and generates difference-equation text.
3. Print summary table: target/achieved crossover, Kp, Ki, Ti, discrete coefficients, difference equation.

### Step 4: Frequency-Response Visualization (Lines 110–122)
1. For each motor:
   - Plot **plant Bode** (magnitude and phase vs frequency).
   - For each controller (nominal, low-BW):
     - Plot **open-loop Bode** with crossover frequency and phase-margin annotations.

### Step 5: Simulated Step Responses (Lines 124–132)
1. For each motor:
   - Simulate closed-loop step response for both controllers over 0–3 s.
   - Overlay nominal vs low-bandwidth in one figure.
   - Simulate control signal (voltage) step response and overlay both controllers.

### Step 6: Load Experimental Data (Lines 134–148)
1. Define scenario list: flat-ground step (2a), incline nominal (2b).
2. For each scenario, call `loadQRCLog()`:
   - Read CSV, convert timestamps ms → s.
   - Map columns to named fields: `reference`, `speedA`, `speedB`, `errorA`, `errorB`, `controlA`, `controlB`.
   - Warn if file missing or empty.
3. Separately load incline low-bandwidth data (2c).

### Step 7: Experimental Validation Plots — Sections 2(a) & 2(b) (Lines 149–165)
1. For each scenario (flat step, incline nominal):
   - For each motor:
     - Simulate closed-loop, sensitivity, and control TF responses using `lsim()`.
     - Plot **closed-loop response**: reference, measured, simulated.
     - Plot **tracking error**: measured vs simulated.
     - Plot **control effort**: measured vs simulated.

### Step 8: Incline Comparison — Section 2(c) (Lines 167–179)
1. If both incline datasets exist:
   - For each motor:
     - Overlay **nominal vs low-bandwidth** measured and simulated responses.
     - Overlay tracking errors.
     - Overlay control efforts.
2. Warn if data missing.

### Step 9: Disturbance Block Diagram (Lines 181–183)
1. Call `plotDisturbanceBlockDiagram()` to programmatically draw a feedback loop diagram with disturbance input `Fd`, controller `C(z)`, plant `P(z)`, and output `y = ω`.

### Step 10: Finish (Line 184)
1. Print completion message with figure directory path.

---

### Local Functions Summary

| Function | Purpose |
|----------|---------|
| `designPiController()` | Frequency-response PI design, returns gains, TFs, margins, discrete coefficients |
| `discreteDifferenceEquation()` | Converts discrete TF to human-readable difference equation string |
| `plotPlantBode()` | Bode plot of plant only |
| `plotOpenLoopBode()` | Bode plot of open-loop with crossover/PM annotations |
| `plotClosedLoopStepComparison()` | Overlays nominal vs low-BW closed-loop step |
| `plotControlSignalStepComparison()` | Overlays nominal vs low-BW control effort |
| `loadQRCLog()` | Parses QRC CSV, validates, returns struct with named fields |
| `plotExperimentalValidation()` | Plots measured vs simulated for one scenario/motor |
| `plotScenarioComparison()` | Overlays two scenarios (nominal vs low-BW) for incline comparison |
| `plotDisturbanceBlockDiagram()` | Draws annotated block diagram |
| `persistFigure()` | Saves figure to PNG if `exportPlots` is true |

---

## 1. Code Structure & Organization

| Aspect | Your Solution | Example Report |
|--------|---------------|----------------|
| **File count** | Single unified script (~500 lines) | Two separate scripts (`ass2.m` + `graphs_2c.m`, ~470 lines total) |
| **Functions** | Modular with 10+ local functions | Monolithic procedural code, no functions |
| **Configuration** | Centralized `struct` for specs, paths, and design parameters | Hard-coded values scattered throughout |
| **Comments/Documentation** | Extensive header block + inline comments | Minimal comments |

**Verdict:** Your solution is significantly more maintainable and reusable.

---

## 2. Plant Models

| Parameter | Your Solution | Example Report |
|-----------|---------------|----------------|
| **Wheel A** | `tf([0.6309], [1, -0.6819, 0], Ts)` | `tf([0.6594, -0.3973], [1, -0.8789, 0.006421, 0], Ts)` |
| **Wheel B** | `tf([0.6488], [1, -0.6806, 0], Ts)` | `tf([0.6845, -0.4205], [1, -0.8804, 0.00909, 0], Ts)` |
| **Model order** | 2nd-order (simplified) | 3rd-order |
| **Numerator structure** | Single coefficient (no extra delay) | Two coefficients |
| **Source** | From your own Assignment 1 identification | From example Assignment 1 data |

**Key difference:** Your models are simpler (2nd-order) while the example uses 3rd-order ARX fits with an additional pole near zero. Both include the integrator (pole at z=0).

**Model form:** `H(z) = b1 / (z^2 + a1*z)` which corresponds to `tf([b1], [1, a1, 0], Ts)` in MATLAB.

---

## 3. Controller Design Approach

### 3.1 Nominal Controller

| Parameter | Your Solution | Example Report |
|-----------|---------------|----------------|
| **Target crossover** | 30 rad/s (configurable) | 30.3 rad/s (hard-coded) |
| **Phase margin** | 55° (configurable) | ~55° (implicit) |
| **Phase-lag reserve** | 15° (explicit parameter) | 15° (hard-coded in `tand(90-15)`) |
| **Gain K** | Computed from `evalfr()` at ωc | Hard-coded `K = 0.8395` |
| **Ti calculation** | `Ti = tand(90 - phaseLagReserve) / wc` | Same formula |

### 3.2 Low-Bandwidth Controller

| Parameter | Your Solution | Example Report |
|-----------|---------------|----------------|
| **Target crossover** | `2*pi*0.5` ≈ 3.14 rad/s | 3.1415 rad/s |
| **Gain K** | Computed automatically | Hard-coded `K = 0.4741` |

**Key difference:** Your solution computes gains programmatically via frequency-response evaluation; the example uses manually tuned constants.

---

## 4. Discretization

| Aspect | Your Solution | Example Report |
|--------|---------------|----------------|
| **Method** | Tustin (bilinear) via `c2d(..., 'tustin')` | Same |
| **Output** | Stores `num`, `den`, and generates difference-equation text | Only stores discrete TF object |

Your script additionally exports the difference equation in human-readable form for direct implementation on the microcontroller.

---

## 5. Data Loading & Parsing

| Aspect | Your Solution | Example Report |
|--------|---------------|----------------|
| **Function** | Dedicated `loadQRCLog()` with validation | Inline `dlmread` calls |
| **Time conversion** | Converts ms → s, checks sample interval | Manual indexing |
| **Column mapping** | Named fields (`data.speedA`, `data.controlA`, etc.) | Raw column indices (`plot_data(:,9)`) |
| **Error handling** | Warns if file missing or empty | None (will crash) |

---

## 6. Visualization

| Feature | Your Solution | Example Report |
|---------|---------------|----------------|
| **Plant Bode** | Dedicated `plotPlantBode()` | Uses built-in `margin()` |
| **Open-loop Bode** | Custom plot with crossover/PM annotations | `margin()` only |
| **Step response** | Overlays nominal vs low-BW in one figure | Separate figures per scenario |
| **Experimental validation** | Overlays reference, measured, simulated | Same approach |
| **Control effort** | Separate figure with sim vs measured | Same |
| **Figure export** | Configurable `exportPlots` flag with `exportgraphics()` | No export capability |
| **Block diagram** | Programmatic disturbance block diagram | Not included |

---

## 7. Scenario Coverage

| Scenario | Your Solution | Example Report |
|----------|---------------|----------------|
| **Flat ground step (2a)** | ✅ | ✅ |
| **Incline nominal (2b)** | ✅ | ✅ |
| **Incline low-BW (2c)** | ✅ with overlay comparison | ✅ (in `graphs_2c.m`) |
| **Disturbance analysis** | Block diagram included | Not included |

---

## 8. Code Quality Metrics

| Metric | Your Solution | Example Report |
|--------|---------------|----------------|
| **Reusability** | High (functions, structs) | Low (copy-paste) |
| **Extensibility** | Easy to add motors/scenarios | Requires duplication |
| **Hardcoded paths** | Centralized in config section | Scattered absolute paths |
| **Magic numbers** | Named constants | Inline literals |
| **DRY principle** | Followed | Violated (repeated plotting code) |

---

## 9. Summary of Improvements in Your Solution

1. **Modular architecture** — Local functions for each task (design, plotting, data loading).
2. **Parameterized design** — Change specs in one place; gains recompute automatically.
3. **Robust data handling** — Graceful warnings instead of crashes.
4. **Richer output** — Difference-equation text, margin printouts, optional figure export.
5. **Cleaner plots** — Annotated Bode diagrams, overlays for direct comparison.
6. **Documentation** — Header block explains workflow; inline comments clarify intent.

---

## 10. Potential Enhancements

- Add unit tests for `designPiController()` edge cases.
- Parameterize CSV column order to handle different QRC log formats.
- Include Nyquist plots for additional stability insight.
- Automate LaTeX table generation for the report.
