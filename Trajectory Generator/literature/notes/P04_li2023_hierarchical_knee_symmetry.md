---
id: P04
title: "Hierarchical Optimization for Control of Robotic Knee Prostheses Toward Improved Symmetry of Propulsive Impulse"
authors: "Minhan Li, Wentao Liu, Jennie Si, Jonathan W. Stallrich, He Huang"
year: 2023
venue: "IEEE Transactions on Biomedical Engineering (TBME), vol. 70, no. 5, pp. 1634-1642"
doi_or_url: "10.1109/TBME.2022.3224026"
topics: [hierarchical_control, prosthesis_knee, action_space, symmetry, bayesian_optimization, reward_design, human_in_loop]
keywords: [hierarchical, Bayesian-optimization, approximate-policy-iteration, FSM impedance control, propulsive-impulse-symmetry, target-kinematics-design]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/2 - Protesi robotiche di ginocchio-transfemorali/Hierarchical Optimization for Control of Robotic Knee Prostheses Toward Improved Symmetry of Propulsive Impulse.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\GPT\paper\2 - Protesi robotiche di ginocchio-transfemorali\Hierarchical Optimization for Control of Robotic Knee Prostheses Toward Improved Symmetry of Propulsive Impulse.pdf"
pages_read: "1-9 (full; refs read)"
extraction_confidence: high  # equations clean; figures/tables not interpreted
related: [P01, P02, P03, P05]
---

# P04 — Hierarchical BO+RL for Robotic Knee Prostheses (propulsion symmetry)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [hierarchical_control](../TOPICS.md) · [action_space](../TOPICS.md) · [symmetry](../TOPICS.md) · [bayesian_optimization](../TOPICS.md)

## TL;DR
A **two-level (hierarchical) controller**: the **outer loop = Bayesian Optimization
(human-in-the-loop)** that *designs the target knee kinematics* to minimize
propulsive-impulse asymmetry; the **inner loop = RL (approximate policy iteration)**
that tunes the 12 impedance parameters to track that target. This is the closest
match to the **"high-level picks a kinematic target, low-level tracks it"** pattern.
Key finding: the kinematics giving best symmetry are **not** the normative profile.

## Problem & contribution
- HIL optimization scales poorly to high-dim control; RL has mostly optimized
  *kinematic* targets, not overall gait performance (p.1).
- Contribution: a hierarchical framework that connects HIL **BO (high level: choose
  target kinematics)** and **RL (low level: hit the target via 12 impedance params)**
  to improve **propulsive-impulse symmetry** (p.2).
- Demonstrated finding: best-symmetry kinematics ≠ normative → **restoring normative
  joint biomechanics doesn't necessarily optimize gait performance** (abstract, p.7).
  *(Notable for any approach that uses a normative/IK reference as the goal.)*

## Method / architecture
Two nested loops (Fig.1, p.3):
- **Outer (high level) — Bayesian Optimization** chooses target knee feature `s` (4
  peak angles, one per phase) to minimize asymmetry `h(s)`:
  - Objective = propulsive-impulse **Symmetry Index** `SI = (I_PI − I_PP)/(0.5(I_PI+I_PP))`
    → PDF p.2, eq.(1); propulsive impulse `I_Pλ = ∫_TDS Fλ dt + ∫_SS ReLU(Fλ) dt`
    → PDF p.2, eq.(2).
  - GP prior, squared-exponential kernel → PDF p.4, eq.(12); posterior mean/var
    → eq.(13); **Expected Improvement** acquisition → eq.(14). Terminate on SI/
    hyperparam convergence or 20 iters (p.5).
- **Inner (low level) — RL approximate policy iteration** tunes impedance to hit `s`:
  - Plant `x(t+1)=f(x(t),u(t))`, policy `u=π(x)`; state `x = ΔP` (measured−target
    feature), action `u = [ΔK, Δθe, ΔC]ᵀ` → PDF p.3, eq.(4).
  - Quadratic stage cost `g = xᵀ R_s x + uᵀ R_a u` → PDF p.3, eq.(5); value `Q`
    eq.(6); Bellman optimality eq.(7); linear basis-function approx `Q̂=φ(x,u)ᵀr`
    eq.(8); policy evaluation eq.(9); improvement `π_{i+1}=argmin_u Q̂` eq.(10).
    Full RL algorithm in ref [22].
  - **A single pre-trained impedance-tuning policy is reused for ALL participants**
    (robust to user changes, from [22]) (p.6).
- Low-level torque law `τ = K(θe − θ) − Cω` → PDF p.3, eq.(3); knee feature = 4 peak
  angle magnitudes (timings dropped: lower SNR) (p.3).

## Experimental setup
- 2 able-bodied (L-adapter) + 1 transfemoral; treadmill 0.6 m/s; slider-crank knee,
  ≤80 N·m, potentiometer + ATI load cell + Bertec; LabVIEW+MATLAB (p.5).
- Impedance updated every **4 gait cycles**; target reached when all phase
  deviations < **2°** for 3 consecutive updates (p.5).
- BO: 6 pseudo-random init targets + up to 14 sampled; baseline = passive devices
  (Total-Knee / C-Leg) (p.5-6).

## Key results
- Optimal vs worst target: SI improved on average **39.2±10.2%**; vs passive
  baseline **~50.2%** (p.7).
- Optimization total time ~**14.1±4.5 min** (clinic-feasible) (p.6).
- **Best-symmetry kinematics ≠ normative** for all subjects; shared feature = a
  terminal-stance **plateau / under-extension** of the prosthetic knee (p.7).
- Optimal condition increased prosthetic-side peak propulsive GRF and impulse,
  driving the symmetry gain (p.8).

## Code / data availability
- **No public code/dataset** (human-subject hardware study, same group).
- RL inner-loop algorithm: ref [22] "Toward expedited impedance tuning ... by RL
  control" (Li et al., IEEE T-RO 2022) — not in this corpus.

## Notable claims (page-anchored)
- "**restoration of normative joint biomechanics in walking does not necessarily
  optimize the gait performance** of human-prosthesis systems" (abstract; p.7).
- High level produces a **functional target (kinematics)**, low level a **tracking
  policy** — explicit two-level decomposition (p.2, Fig.1).
- Single RL policy generalizes across users → high-level objective can change
  without retraining the low level (p.6).
- Chose **symmetry** (not metabolic cost) as objective; HIL metabolic-cost
  optimization has mostly worked only for non-disabled users (p.2).

## Related notes
- [P02 — Online RL personalization (Wen et al. 2020)](P02_wen2020_online_rl_personalization.md) — the RL inner-loop lineage (ref [20]); here the fixed normative target is *replaced by a BO-chosen target*.
- [P01 — RL Impedance Echo Control (Wu et al. 2022)](P01_wu2022_rl_impedance_echo_control.md) — alternative high-level target (intact knee) vs BO-optimized target here.
- [P03 — Wearer-Prosthesis Interaction for Symmetrical Gait (Wen et al. 2020)](P03_wen2020_wearer_prosthesis_symmetry.md) — established the impulse↔symmetry link this paper *optimizes* (ref [30]).
- [P05 — Human-Robotic Prosthesis as Collaborating Agents (Wu et al. 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — same group; multi-agent take on human-prosthesis co-optimization.

## Caveats (not verified / limits)
- Figures (Fig.1 schematic; Figs.2-5 results) and Table I (target domain) **not
  interpreted**.
- The RL inner loop is the same per-phase, gait-cycle-rate impedance tuner as
  P01/P02 (not a control-rate/trajectory policy). The novelty is the **BO outer
  loop choosing the target**.
- Small N (3), single speed (0.6 m/s), treadmill only.
