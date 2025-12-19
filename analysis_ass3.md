# Analysis of Assignment 3 LaTeX Report

**Document:** `tex_control/ass3_tex/assignment3.tex` (670 lines)  
**Reference:** `goals_assignments/specs_assignment3.md`

---

## Executive Summary

The report is well-structured and technically correct but suffers from **significant redundancy** and **over-explanation**. Several derivations and trade-off discussions are repeated nearly verbatim across sections. At graduate level, certain elementary explanations can be assumed known. Estimated reduction potential: **25–35%** of body content.

---

## Section-by-Section Analysis

### 1. Preamble & Title Page (Lines 1–110)
**Status:** Acceptable  
No changes needed. Standard boilerplate.

---

### 2. Section 1.1 — Discrete-time State Equation (Lines 125–163)

**Issues:**
- Line 162 adds a "Note" about the average wheel speed that is implementation detail, not required by specs.

**Recommendation:**  
Delete line 162 entirely. The specs ask only for the state equation and matrices.

**Savings:** ~2 lines

---

### 3. Section 1.2 — Measurement Equation (Lines 165–184)

**Issues:**
- Line 183 restates that $C = [-1]$ "because the sensor measures the negation of the state" — this is already obvious from line 171.

**Recommendation:**  
Delete the explanatory sentence on line 183.

**Savings:** ~2 lines

---

### 4. Section 1.3 — Design of $K$ (Lines 186–292)

**Major Redundancy:**
- The closed-loop pole formula $z_\text{cl}(K) = 1 - T_s r K$ appears **4 times** (lines 213, 222, 243, 286).
- Stability analysis (lines 219–229) repeats what the pole-zero map already shows.
- The paragraph "Relationship between pole location, $K$, and time response" (lines 266–272) restates concepts already in Table 1 and the preceding paragraph.
- "Choice of $K$" paragraph (lines 274–291) partially duplicates content that reappears in Section 2.2.

**Recommendations:**
1. State the closed-loop pole formula **once** with a boxed equation, then reference it.
2. Merge the stability analysis into the pole-behavior paragraph — one sentence suffices: "For stability, $|z_\text{cl}| < 1 \Rightarrow 0 < K < 6060$ rad/(s·m)."
3. Delete the bullet list at lines 267–272; the table already conveys this.
4. The "practical constraints" list (lines 277–280) can be reduced to a single sentence referencing actuator limits, noise, and sensor range.

**Savings:** ~30–40 lines

---

### 5. Section 1.4 — Design of $L$ (Lines 296–384)

**Major Redundancy:**
- The estimator pole formula $z_\text{est}(L) = 1 + L$ appears **3 times** (lines 327, 341, 381).
- The "Trade-offs in pole placement" nested list (lines 354–368) is verbose. A single sentence per trade-off suffices at graduate level.
- Lines 370–373 repeat trade-offs just stated.
- Line 383 previews experimental $L$ values — belongs in Section 2, not design.

**Recommendations:**
1. State the estimator pole formula once, then reference.
2. Collapse the trade-off lists into a concise 2–3 sentence paragraph.
3. Move the $L$ sweep values to Section 2.1.

**Savings:** ~20–25 lines

---

### 6. Section 2.1 — Estimator Only (Lines 390–436)

**Major Redundancy:**
- Lines 407–436 re-derive and re-explain the estimator pole behavior that was **already fully covered in Section 1.4**.
- The interpretation restates verbatim: "As shown analytically, the closed-loop pole is $z_\text{est}(L) = 1 + L$" — this was already boxed in Section 1.4.
- The three bullet blocks (small/moderate/large $|L|$) mirror the trade-off discussion from Section 1.4 almost word-for-word.

**Recommendations:**
1. **Delete lines 407–436 entirely** or replace with: "The observed convergence rates match the theoretical predictions from Section 1.4: larger $|L|$ yields faster convergence but amplifies sensor noise."
2. Keep only the figure and a 2–3 sentence caption-style interpretation.

**Savings:** ~25–30 lines

---

### 7. Section 2.2 — Controller Only (Lines 439–513)

**Major Redundancy:**
- Lines 465–492 repeat the closed-loop pole analysis from Section 1.3 almost verbatim.
- The formula $z_\text{cl}(K) = 1 - T_s r K$ is restated again.
- The interpretation bullet lists (lines 470–491) duplicate Table 1 and the discussion in Section 1.3.
- "Choice of $K$" paragraph (lines 494–512) is **nearly identical** to lines 274–291 in Section 1.3.

**Recommendations:**
1. Replace lines 465–492 with: "The step responses confirm the theoretical analysis from Section 1.3: higher $K$ produces faster response but risks saturation."
2. **Delete the entire "Choice of $K$" subsection** (lines 494–512) — it is a verbatim repeat. A single sentence referencing Section 1.3 suffices.

**Savings:** ~40–45 lines

---

### 8. Section 2.3 — Combined Estimator and Controller (Lines 518–663)

**Issues:**
- Lines 524–529 ("Recap of closed-loop poles") restates what is already known.
- The derivation of $L_\text{slow}$ (lines 534–550) is appropriate but could be more concise.
- Lines 618–652 ("Full closed-loop system poles") are excessively verbose. The separation principle can be stated in 3–4 sentences, not 35 lines.
- The conclusion (lines 655–662) restates points already made in the preceding paragraphs.

**Recommendations:**
1. Delete the "Recap" paragraph — simply reference Sections 1.3 and 1.4.
2. Condense the separation principle discussion (lines 618–652) to: "By the separation principle, the combined system poles are $\{z_\text{cl}, z_\text{est}\}$, designed independently. However, a slow estimator with wrong initialization degrades transient performance because the controller acts on $\hat{x}[k]$, not $x[k]$."
3. The conclusion can be merged into the preceding paragraph — no separate "Conclusion" needed.

**Savings:** ~35–40 lines

---

## Summary of Redundant Patterns

| Pattern | Occurrences | Recommended Action |
|---------|-------------|-------------------|
| $z_\text{cl}(K) = 1 - T_s r K$ formula | 5+ | State once, reference thereafter |
| $z_\text{est}(L) = 1 + L$ formula | 4+ | State once, reference thereafter |
| Stability bound derivation | 3 | State once in Section 1 |
| Trade-off lists (noise vs. speed) | 4 | Consolidate into one paragraph |
| "Choice of $K$" discussion | 2 (verbatim) | Delete duplicate |
| Separation principle explanation | 2 | State once concisely |

---

## Specific Line-by-Line Edits

### High Priority (Remove/Rewrite)

| Lines | Action | Rationale |
|-------|--------|-----------|
| 162 | Delete | Implementation note not in specs |
| 183 | Delete | Redundant restatement |
| 267–272 | Delete | Covered by Table 1 |
| 370–373 | Delete | Repeats lines 354–368 |
| 383 | Move to Sec 2.1 | Wrong location |
| 407–436 | Replace with 2–3 sentences | Verbatim repeat of Sec 1.4 |
| 465–492 | Replace with 2–3 sentences | Verbatim repeat of Sec 1.3 |
| 494–512 | Delete | Exact duplicate of lines 274–291 |
| 524–529 | Delete | Unnecessary recap |
| 618–652 | Condense to ~5 lines | Over-explained separation principle |
| 655–662 | Merge into preceding text | Redundant conclusion |

### Medium Priority (Tighten)

| Lines | Action |
|-------|--------|
| 219–229 | Merge stability analysis into pole-behavior paragraph |
| 354–368 | Collapse nested lists into prose |
| 534–550 | Streamline derivation (remove step-by-step commentary) |
| 586–616 | Reduce bullet verbosity |

---

## Estimated Final Length

| Section | Current | After Edits |
|---------|---------|-------------|
| Sec 1.1 | 38 lines | 36 lines |
| Sec 1.2 | 20 lines | 18 lines |
| Sec 1.3 | 107 lines | 70 lines |
| Sec 1.4 | 89 lines | 60 lines |
| Sec 2.1 | 47 lines | 20 lines |
| Sec 2.2 | 75 lines | 35 lines |
| Sec 2.3 | 146 lines | 90 lines |
| **Total body** | **522 lines** | **~330 lines** |

**Net reduction: ~37%** while preserving all required content per specs.

---

## Style Notes for Graduate-Level Writing

1. **Assume reader competence:** Do not re-explain what a closed-loop pole is or why stability requires $|z| < 1$.
2. **One derivation, many references:** Derive key results once; cite equation numbers thereafter.
3. **Figures speak:** If a figure shows convergence behavior, a lengthy prose explanation is redundant.
4. **Avoid hedging:** "This matches the formula..." can be "Consistent with Eq. (X)."
5. **Bullet lists ≠ prose:** Convert repetitive bullet lists to concise paragraphs.

---

## Recommended Revision Order

1. **Pass 1:** Delete all duplicate "Choice of $K$" and "Interpretation" blocks in Section 2.
2. **Pass 2:** Consolidate pole formulas into single boxed equations with labels.
3. **Pass 3:** Rewrite trade-off discussions as compact paragraphs.
4. **Pass 4:** Tighten Section 2.3 separation principle discussion.
5. **Pass 5:** Final polish — remove filler phrases ("as shown analytically", "this means that", etc.).

---

*Analysis generated for Team 47, Assignment 3 revision.*
