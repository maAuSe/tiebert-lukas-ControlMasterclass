Internal consistency check of assignment reports vs. supplied specifications.

Scope: `tex_control/ass1_tex/assignment1.tex`, `tex_control/ass2_tex/assignment2.tex`, `tex_control/ass3_tex/assignment3.tex`, `tex_control/ass5_tex/assignment5.tex` against `goals_assignments/specs_assignment{1,2,3,5}.md`.

## Assignment 1 (Identification)
- Structure mostly matches prompts: continuous-time DC motor TF with units, ZOH discretisation, recursion + LS criterion, excitation rationale/plot, model validation both motors, filtering experiment.
- Internal issues:
  - Filtering redo (spec 2(c)) is only reported for motor A; the spec asks to “redo the identification after filtering your data” and to validate “for both motors.” No filtered-model parameters/plots or comparisons are given for motor B (`tex_control/ass1_tex/assignment1.tex`, Filtering section).
  - Claimed third-order discrete model (due to microOS delay) conflicts with the earlier two-delay narrative: difference equation uses inputs at k-2 and k-3, but the base ZOH model is second order; the exposition does not cleanly reconcile where each delay enters (`assignment1.tex`, Section 1a/1b). Not fatal but leaves ambiguity about the exact model order used in estimation.
  - Page-length/structure guidance (max half/one page) is exceeded in several subsections; derivation/explanations are much longer than the “brief” limits specified in `specs_assignment1.md`.

## Assignment 2 (Velocity control)
- Controller choice, frequency-domain design, time/frequency verification, disturbance tests, and low-bandwidth redesign are all addressed with the requested plots and discussions.
- Deviations/gaps:
  - The plant models used here (`G_{s,A}(s)`, `G_{s,B}(s)` in `tex_control/ass2_tex/assignment2.tex`) are continuous-time, second-order with zeros, but Assignment 1 selected a discrete second-order model with identified coefficients. The report never explains how these continuous-time numerators/denominators were obtained from the Assignment 1 identification. This breaks the traceability the spec expects (“Use the identified models ... selected in part 2”).
  - Minor figure/caption inconsistencies (e.g., Figure set `Figures_part_1b/Verification-Open-loop-B.png` captioned as compensated loop) make it unclear where design targets are marked on the plots, whereas the spec asks to indicate on the Bode plot the characteristics used to choose parameters.
  - Degree symbols render as `Aø` throughout; not critical for content but indicates encoding issues.

## Assignment 3 (State feedback & estimation)
- Broadly follows the required experiment flow (estimator-only, controller-only, combined) and supplies pole maps, step sweeps, and discussion of trade-offs.
- Consistency/requirements issues:
  - Full closed-loop eigenvalues are reported inconsistently: earlier, the controller pole is defined as \(z_{cl}=1-T_s r K\) (`assignment3.tex`, Section 1c), but later the eigenvalue is written as \(\lambda_1 = 1 - T_s K\) (`assignment3.tex`, Section 2c, “Full closed-loop system poles”), dropping the wheel-radius factor. This contradicts the state equation and changes stability limits.
  - The “Design guideline” sentence in Section 1d is truncated (“faster than the :”), leaving the intended estimator speed criterion unclear.
  - The separation-principle discussion asserts the controller and estimator poles “appear separately and are not coupled” and that the combined eigenvalues equal the individual poles; however, with the stated \(A_{\text{full}}\) matrix the eigenvalues depend on both \(K\) and \(L\) unless the plant is exactly the integrator assumed. The text should reconcile this with the actual model or explicitly justify why the cross-terms do not shift poles.
  - Units/parameter consistency: \(K_{\max} = 6060\) rad/(s·m) follows from \(2/(T_s r)\), but the later pole expressions omit \(r\), causing conflicting stability claims.
  - The estimator-slower-than-controller design in 2(c) uses \(L=-0.0013\) (very slow), but the text does not confirm that this choice respects the “10× slower” requirement relative to the earlier selected \(K\) when including the \(r\) factor.

## Assignment 5 (2WD EKF + LQR)
- Largely aligned with the specification: kinematic model and wheel/velocity relations, general + scenario-specific measurement equations, forward Euler discretisation, Jacobians, EKF vs. linear KF discussion, noise sources, EKF tuning with four \(Q/R\) combinations, uncertainty evolution, rotation matrix, LQR structure/tuning with four \(Q_{\text{lqr}},R_{\text{lqr}}\) sets, and chosen gains with units.
- Minor gaps:
  - The discretisation of the measurement equation is stated as “requires no discretisation” (`tex_control/ass5_tex/assignment5.tex`, Section 2a); the spec asks to “clearly show which formulas you use” for discretisation. Even if algebraic, an explicit discrete-time form matching the continuous equation would satisfy the prompt more directly.
  - The implementation narrative omits explicit mention of using the provided Arduino template/QRC import steps (`specs_assignment5.md` asks for this context); the plots are referenced but the linkage to the prescribed tooling is implicit rather than stated.

## Cross-assignment consistency
- Traceability break: Assignment 2 claims to use “unfiltered simplified continuous-time models” but no such continuous-time model was produced in Assignment 1; only discrete models with numerical coefficients were identified. The leap from `assignment1.tex` to `assignment2.tex` leaves the controller design foundation undocumented.
- Sampling/loop assumptions are mostly consistent (100 Hz in Assignments 1–3), and the “ideal velocity loop” assumption in Assignments 3 and 5 aligns with the velocity-control design of Assignment 2.

## Key fixes to reach spec fidelity
1) Provide filtered identification results (parameters + validation plots) for motor B in Assignment 1 Section 2(c), or state explicitly that both motors were checked and behaved identically.  
2) Document how the continuous-time plant models in Assignment 2 were derived from the discrete models of Assignment 1 (method, parameter mapping, any refits).  
3) Correct the inconsistent pole formulas/eigenvalues in Assignment 3 (ensure \(r\) appears consistently) and clarify the estimator-speed guideline text.  
4) Make the measurement discretisation explicit in Assignment 5 and note the use of the provided EKF template/QRC data path as requested in the spec.
