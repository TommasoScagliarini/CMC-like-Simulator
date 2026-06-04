---
id: P01
title: "Reinforcement Learning Impedance Control of a Robotic Prosthesis to Coordinate With Human Intact Knee Motion"
authors: "Ruofan Wu, Minhan Li, Zhikai Yao, Wentao Liu, Jennie Si, He Huang"
year: 2022
venue: "IEEE Robotics and Automation Letters (RA-L), vol. 7, no. 3, pp. 7014-7020"
doi_or_url: "10.1109/LRA.2022.3179420"
topics: [prosthesis_knee, impedance_control, hierarchical_control, action_param_tuning, reward_tracking, safety_bounds, human_in_loop, online_rl, symmetry]
keywords: [echo-control, FSM impedance control, PICE, actor-critic, policy-iteration, transfemoral, intact-knee-mirroring]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/2 - Protesi robotiche di ginocchio-transfemorali/Reinforcement Learning Impedance Control of a Robotic Prosthesis to Coordinate With Human Intact Knee Motion.pdf"
pages_read: "1-7 (full)"
extraction_confidence: medium  # prose reliable; some equations broken by PDF layout; figures not interpreted
related: [P02, P03, P04, P05]
---

# P01 — RL Impedance Control of a Robotic Prosthesis (Echo Control)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use this KB](../README_FOR_LLM.md)
**Topics:** [prosthesis_knee](../TOPICS.md) · [impedance_control](../TOPICS.md) · [hierarchical_control](../TOPICS.md) · [safe (safety_bounds)](../TOPICS.md)

## TL;DR
Model-free RL that tunes the **impedance parameters** (not torque, not a trajectory)
of a powered knee prosthesis online so the robotic knee **mirrors the contralateral
intact knee profile** ("echo control"), instead of tracking a fixed prescribed
target. Validated in real time on 2 able-bodied + 1 transfemoral amputee, continuous
treadmill walking, with tracking error converging inside safety tolerances.

## Problem & contribution
- Manual tuning of a powered prosthesis' impedance parameters is hard and
  subject/task-specific; the authors' prior work tracked a **fixed, prescribed**
  knee profile (p.1).
- Limitation: prescribing a profile per person/task is unrealistic, and a fixed
  robotic profile hinders human-robot co-adaptation (p.1).
- Contribution: first real-time RL tuning of impedance parameters to track a
  **time-varying** intact-knee profile during human co-adaptation, demonstrated
  on human subjects (p.2).

## Method / architecture
Two-level hierarchy (relevant as a control pattern):

- **Low level — FSM Impedance Control (FSM-IC).** The gait cycle is split into 4
  phases (STF, STE, SWF, SWE). Each phase has 3 impedance parameters: stiffness K,
  damping B, equilibrium position θe (p.2).
  - Parameter vector (simple eq.): `I_k = [K_k, B_k, θe_k]` → PDF p.2, eq.(1).
  - Knee torque (simple eq.): `T_k = K_k·(θ − θe_k) + B_k·ω` → PDF p.2, eq.(2).
- **High level — RL tracking controller** emits parameter **increments** (NOT a
  trajectory): `u_k = [ΔK_k, ΔB_k, Δθe_k]`, applied as `I_{k+1} = I_k + u_k`
  → PDF p.2, eq.(3). Total **12 parameters** (3 × 4 phases).
- **RL state** = kinematic-feature errors between prosthetic and intact knee: peak
  angle `ΔP_k = P^p_k − P^i_k` and phase duration `ΔD_k = D^p_k − D^i_k`, so
  `x_k = [ΔP_k, ΔD_k]` → PDF p.2, eqs.(4)-(5).
- **Policy** (simple eq.): `u_k = h(x_k)` → PDF p.2, eq.(6).
- **Instantaneous cost** — *heavy/garbled in extraction*: a quadratic form in state
  and control, `U(x_k, u_k) = x_kᵀ R_x x_k + u_kᵀ R_u u_k`, with R_x (2×2), R_u (3×3)
  positive-definite; optimized as an infinite-horizon discounted cost. The
  superscript transposes garbled on extraction — **verify exact form → PDF p.2,
  eq.(7)**.
- **Algorithm:** PICE — *Policy Iteration with Constraint Embedded* (actor-critic
  solving a QP per update), taken from ref [9]. **Four separate controllers**, one
  per gait phase, identical structure (p.2-3). The PICE update rule itself is NOT
  in this paper (see [9]); → PDF p.3 references "eq.(14) in [9]".
- **Update cadence:** every gait cycle, measure [P,D] and compute x_k; PICE updates
  (QP) and outputs ΔI. Impedance parameters are applied **every 4 gait cycles** (p.3).
- **Intact-knee feature extraction** — *heavy/garbled*: peak/trough features; a
  running average over L=10 cycles produces the target; a hysteresis threshold
  ε=1.5° decides whether to update the target; tracking has a 1-cycle delay. The
  running-average and hysteresis equations broke across lines on extraction —
  **read them directly → PDF p.3-4, eqs.(8)-(9)**.

## Experimental setup
- Subjects: 2 able-bodied (via "L"-shaped socket adapter) + 1 transfemoral amputee;
  treadmill at preferred speed (0.65 / 0.7 / 0.8 m/s) (p.4).
- Hardware: powered slider-crank knee (DC motor + ball screw), potentiometer for
  knee kinematics, load cell for phase transitions, instrumented Bertec treadmill
  GRF at 1000 Hz, **goniometer** for the intact knee, LabVIEW+MATLAB at 100 Hz (p.3).
- Pre-processing: 20 Hz low-pass on GRF and kinematics; heel-strike/toe-off from
  vertical GRF with a **30 N** threshold; Dempster-Shafer phase-transition rule (p.3-4).
- Convergence: peak error < 2°, duration error < 3%, for **8 of 10** consecutive
  updates (p.4).
- **Safety bound** on state/action: 1.5× the standard deviation of per-phase knee
  peak values → peak-error bounds `[10.5, 7.5, 9, 6]°` and duration bounds `[12% ×4]`
  (p.4).

## Key results
- All subjects converge: the robotic knee profile approaches the intact one;
  starting from random ("far") parameters, it closes in (p.5).
- Peak-error reductions (e.g., AB1: SWF 13.8° → −0.9°; STF −4.6° → −1.7°; amputee:
  from [−3.1, −5.6, 8.76, 2.4]° to [−0.8, 0.1, −0.5, −0.9]°) (p.5).
- t-test: peak errors decrease significantly (p<0.01); duration errors already
  within tolerance (p>0.01 for AB1/TF) (p.5).
- Amputee reports more stability/comfort than the prescribed profile; temporal
  symmetry index 20% → 10% (improved, **anecdotal**, not an explicit objective) (p.6-7).

## Code / data availability
- **No public code or dataset reported** (human-subject hardware study; ASU / NC
  State / UNC). 
- The core learning algorithm (**PICE**) is detailed in ref [9]: M. Li, Y. Wen, X.
  Gao, J. Si, H. Huang, "Toward expedited impedance tuning ... by reinforcement
  learning control," IEEE T-Robotics 38(1), 2022 (not in this corpus).
- Simulation precursor in ref [15]: Wu et al., IEEE/CAA J. Automatica Sinica 9(1), 2022.

## Notable claims (page-anchored)
- "all the above studies utilized **position control** at low level joint operation.
  This may be of **safety concerns**. Instead, the **finite state impedance control**
  strategy is a more feasible approach ... to meet compliance requirements for user
  safety" (p.2). → impedance vs position control as a safety argument.
- The RL problem is cast as **tracking on feature errors** (peak, duration), not as
  full-trajectory generation (p.2).
- "A reinforcement learning solution framework is a **more appropriate** approach
  **than supervised learning**" — collecting SL training pairs would be prohibitive
  and potentially unsafe (p.6).
- **Safety bounds** are set from non-amputee kinematic standard deviations: safety is
  an explicit constraint, not only reward (p.4).
- Exploration: critic/actor weights randomly initialized (or pretrained policy +
  local exploitation); small learning rate (<0.01) to encourage exploration (p.6).

## Related notes
- [P02 — Online RL personalization of a robotic knee prosthesis (Wen et al. 2020)](P02_wen2020_online_rl_personalization.md) — same group; the **fixed-profile** predecessor that P01 generalizes to a time-varying target.
- [P03 — Wearer-Prosthesis Interaction for Symmetrical Gait (Wen et al. 2020)](P03_wen2020_wearer_prosthesis_symmetry.md) — same group; symmetry framing; cited as [7].
- [P04 — Hierarchical Optimization for Robotic Knee Prostheses (Li et al. 2023)](P04_li2023_hierarchical_knee_symmetry.md) — hierarchical impedance/target optimization, same lineage as the PICE work.
- [P05 — Human-Robotic Prosthesis as Collaborating Agents (Wu et al. NeurIPS 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — MARL extension modelling human adaptation explicitly.

## Caveats (not verified / limits)
- **Equations** extracted with line breaks: eq.(7) (transpose superscripts),
  eq.(8) (running average / sums) and eq.(9) (hysteresis condition) were
  reconstructed from sense — use the PDF pointers above if exact forms are needed.
- **Figures not interpreted** (Fig.1 schematic, Fig.2 gait phases/features,
  Fig.3-6 results): only text and captions here.
- The **PICE** update rule (its eq.(14)) is NOT in this paper; it lives in ref [9].
- Important framing: the "RL" here is a **per-phase actor-critic updating at the
  gait-cycle rate** (every 4 cycles) over a 2-feature state — NOT a deep-RL policy
  acting at the control rate, nor a sample-by-sample trajectory generator.
