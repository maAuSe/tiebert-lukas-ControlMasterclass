# Assignment Rating Report

## Assignment 1 — Identification of the cart
- Quality score: 7.5/10
- Specs compliance score: 6.5/10
- Assessment: Technically detailed and well-motivated model selection, clear equations, and thorough comparison of filtered/unfiltered identification. Figures cover requested overlays and errors. However, the write-up is long and sometimes unfocused relative to the tight space limits; some notation drifts from the physical DC-motor model into generic coefficients without reconnecting to the physics.
- Weaknesses:
  - Exceeds multiple stated page limits (1a, 1b, 2a, 2b, 2c) with multi-paragraph derivations instead of the requested brief 5‑line explanations and half-page discussions.
  - Declaration section contains garbled characters (“ƒ?”) and typos.
  - Continuous-time model is reduced to generic a/(bs²+cs+d) and never mapped back to motor parameters, so physical interpretability is weakened.
  - Model-derivation narrative is verbose and repeats concepts (e.g., multiple descriptions of delay and ZOH), crowding out concise motivation.
  - No explicit numeric link between the discrete coefficients and the continuous parameters, despite the spec emphasis on presenting the CT transfer with units.
- Spec match: Partially met; all requested items are present (CT and DT models, recursion, excitation rationale, validation plots, filtering attempt), but conciseness and structure requirements are not respected, and the CT–DT linkage is only partially addressed.

## Assignment 2 — Velocity control of the cart
- Quality score: 7/10
- Specs compliance score: 6/10
- Assessment: Controller choice and design are justified, and the report includes frequency/time-domain verification plus disturbance and low-bandwidth comparisons. Experiments and simulations are shown for both wheels. The narrative is readable but drifts from the spec formatting and brevity.
- Weaknesses:
  - Section 1(a) far exceeds the 1/4-page limit and mixes requirements, options, and selection across several paragraphs.
  - Degree symbols are mangled (“Aø”), which obscures clarity in phase-margin discussions.
  - Some Bode figures lack clearly marked crossover/PM points as demanded; captions and subcaptions have inconsistencies (e.g., repeated or mismatched labels).
  - Uses d2c on the discrete models to design the PI without reconciling the physically unrealistic CT numerator/denominator (second-order numerator), risking inaccurate gains.
  - Trade-off discussion is generic; numerical values are not tied to specific annotated plot features as requested.
- Spec match: Partially met; all major deliverables (plots, disturbance tests, low-bandwidth redesign) are included, but brevity/structure constraints and plot-annotation requirements are not fully satisfied, and clarity suffers from encoding errors.

## Assignment 3 — State feedback and state estimation
- Quality score: 8/10
- Specs compliance score: 7.5/10
- Assessment: Clean derivations of A, B, C, D with forward Euler, clear pole expressions for K and L, and coherent explanations of pole movement and stability. Experiments/plots for estimator-only, controller-only, and combined cases align with the requested scenarios; separation principle is discussed explicitly.
- Weaknesses:
  - Minor truncation/typo (“the estimator should be 3–6 times faster than the :”) and occasional wording roughness.
  - Assumes ideal inner velocity loop and sensor behavior without discussing practical deviations, which weakens experimental credibility.
  - Plots are referenced but not explicitly tied to units or sample counts; unclear whether they come from real experiments or simulations.
  - Estimator design section gives a nominal L=-0.18 but later switches to L≈-0.0013 for the 10× slower requirement without reconciling the choices.
- Spec match: Largely met; all structured requests are addressed (derivations, pole maps, step sweeps, wrong-initial-estimate tests, slow-estimator experiment, full closed-loop poles). Minor clarity and consistency issues prevent a perfect spec hit.

## Assignment 5 — Estimation and control of a two-wheel driven cart
- Quality score: 8.5/10
- Specs compliance score: 8/10
- Assessment: Comprehensive coverage of modeling, EKF derivation, Jacobians, and LQR design. Provides general and scenario-specific measurement equations, forward-Euler discretization, clear discussion of linear vs EKF, detailed noise-source analysis, explicit Q/R choices with units, and multi-case Q/R sweeps plus CI plots. LQR section includes rotation matrix, K structure rationale, tuning sweeps, and selected gains.
- Weaknesses:
  - Narrative density risks exceeding implied page expectations; some explanations could be tighter.
  - EKF tuning/results are described but not clearly tied to Arduino implementation details (e.g., data acquisition steps), leaving reproducibility assumptions implicit.
  - Measurement discretization is asserted as “no discretization needed” rather than explicitly showing the discrete measurement equation, which the spec asks to present.
  - Chosen Q/R units are given, but intermediate sweep configurations omit units, and feedforward/trajectory specifics are referenced but not summarized in the text.
- Spec match: Strong overall; all major structured items are present (modeling, discretization, linearization, KF choice, Q/R tuning with plots, uncertainty discussion, LQR design/tuning). Minor omissions in implementation detail and conciseness keep it from a perfect score.
