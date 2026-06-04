---
id: P03
title: "Wearer-Prosthesis Interaction for Symmetrical Gait: A Study Enabled by Reinforcement Learning Prosthesis Control"
authors: "Yue Wen, Minhan Li, Jennie Si, He (Helen) Huang"
year: 2020
venue: "IEEE Transactions on Neural Systems and Rehabilitation Engineering (TNSRE), vol. 28, no. 4, pp. 904-913"
doi_or_url: "10.1109/TNSRE.2020.2979033"
topics: [prosthesis_knee, symmetry, human_in_loop, online_rl, impedance_control, reward_design]
keywords: [gait-symmetry, anteroposterior-impulse, dHDP, FSM impedance control, transfemoral, wearer-prosthesis-interaction]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/2 - Protesi robotiche di ginocchio-transfemorali/Wearer-Prosthesis Interaction for Symmetrical Gait - A Study Enabled by Reinforcement Learning Prosthesis Control.pdf"
pages_read: "1-13 (full; refs skimmed)"
extraction_confidence: high  # clean text and equations; figures/tables not interpreted
related: [P01, P02, P04, P05]
---

# P03 — Wearer-Prosthesis Interaction for Symmetrical Gait (RL-enabled study)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [prosthesis_knee](../TOPICS.md) · [symmetry](../TOPICS.md) · [human_in_loop](../TOPICS.md) · [online_rl](../TOPICS.md)

## TL;DR
Not a new RL method — it **uses the dHDP RL auto-tuner** (the one from
[P02](P02_wen2020_online_rl_personalization.md)) as a *tool* to safely explore the
12-D impedance-parameter space with a human in the loop, then studies how prosthesis
**mechanics affect gait symmetry** and **anteroposterior (AP) impulses**. Key finding:
**net inter-limb AP impulse** is the consistent correlate of stance-time symmetry;
tuning prosthesis mechanics alone is **insufficient** to maximize symmetry — it needs
coordination with the wearer's intact limb.

## Problem & contribution
- Powered prostheses are usually tuned to restore *normative knee kinematics*, but
  the wearer's overall gait performance (symmetry, stability, energy) doesn't
  consistently improve (p.2).
- Hard to study because safely exploring 12+ control parameters with a human is
  impractical manually. The RL tuner enables it (p.3).
- Contribution: quantifies (1) effect of prosthesis mechanics on GRF impulse and
  temporal-spatial symmetry, and (2) the relationship between inter-limb impulses
  and symmetry (p.3).

## Method / architecture
- **Same low-level + RL tuner as P02**: FSM-IC, 4 phases, torque
  `τ_m = k_m(θ−θe_m) + b_mω` → PDF p.4, eq.(1); 12 impedance params.
- **dHDP actor-critic auto-tuner** (ANN policy `U = φ(W_a2 φ(W_a1 X))` → PDF p.5,
  eq.(4); CNN cost-to-go → eq.(6); ΔI = β⊙U → eq.(5)); state = scaled
  [feature error, feature change] → PDF p.4, eq.(3). Update every **7 strides**;
  safety reinforcement/bounds inherited from P02 [18] (p.5).
- **Outcome metrics** (the actual contribution):
  - Symmetry index `SI_V = (V_i − V_p) / (½(V_i + V_p))` for stance time and step
    length → PDF p.7, eq.(10).
  - Braking/propulsive impulses per leg from AP GRF, split over double/single-limb
    support bins → PDF p.8, eq.(11); braking/propulsion SI → PDF p.8, eq.(12).
  - **Net inter-limb impulse** = prosthetic-leg braking + intact-leg propulsion (p.8).

## Experimental setup
- 6 subjects: 3 able-bodied (L-adapter) + 3 transfemoral amputees; treadmill 0.6 m/s;
  ≥6 training sessions (p.6).
- 4 sessions/subject, each a fresh naïve RL tuner + random initial impedance set;
  ~155±10 parameter combinations tested per subject (p.6, p.9).
- GRF 1000 Hz (Bertec); 8-camera VICON 100 Hz for calcaneus markers; 20 Hz LPF;
  heel-strike/toe-off via 30 N vertical-GRF threshold (p.7).
- Stats: one-way ANOVA (p<0.01) for mechanics effects; Pearson correlation (p<0.01)
  for impulse↔symmetry (p.9).

## Key results
- Prosthesis mechanics significantly affect knee kinematics, AP impulses, and
  temporal-spatial symmetry (all p<0.01); parameter-induced variation ≫ intrinsic
  human variation (p.9).
- **Net inter-limb AP impulse** is consistently and significantly correlated with
  **stance-time SI** across all subjects (R = 0.7±0.08, p<0.01) — the strongest,
  most consistent correlate (p.10).
- Braking SI and propulsion SI correlate with **step-length SI** for all subjects;
  net impulse does not (p.10).
- Cause of asymmetry is individual: TF2 weak prosthetic propulsion; TF3 weak
  prosthetic braking — both yield temporal asymmetry (p.10).
- Tuning mechanics alone yielded step-length symmetry in some subjects but **never
  stance-time symmetry** → likely needs combined rehab + intact-limb coordination
  (p.11).
- **Early-stance knee flexion** appears necessary for best stance-time symmetry
  (shock absorption ↔ braking impulse) (p.11-12).

## Code / data availability
- **No public code/dataset** (human-subject hardware study; same group/hardware as P01/P02).

## Notable claims (page-anchored)
- "achieving gait symmetry may require **coordination between the wearer's motor
  control of the intact limb and adaptive control of the prosthetic joints**"
  (abstract, p.1) — prosthesis tuning alone is insufficient.
- The RL tuner "ensures a human-like locomotion pattern and the wearer's upright
  walking stability and safety by **maintaining the basic knee kinematic pattern**
  and allowing variations within a range" (p.3) — safety via structured exploration.
- Net inter-limb impulse ∝ change of CoM forward velocity at limb transition →
  balancing it gives symmetric mid-stance CoM velocity (p.11). Biomechanical
  grounding for a **symmetry-oriented objective/reward**.
- RL-based tuning proposed as a **tool to study wearer-robot interaction**, beyond
  exoskeletons too (p.12).

## Related notes
- [P02 — Online RL personalization (Wen et al. 2020)](P02_wen2020_online_rl_personalization.md) — the **RL tuner this study uses** as an exploration tool.
- [P01 — RL Impedance Echo Control (Wu et al. 2022)](P01_wu2022_rl_impedance_echo_control.md) — same group; later targets the intact knee (a symmetry-adjacent goal).
- [P04 — Hierarchical Optimization for Robotic Knee Prostheses (Li et al. 2023)](P04_li2023_hierarchical_knee_symmetry.md) — optimizes **propulsive-impulse symmetry** directly (this paper's metric becomes that paper's objective).
- [P05 — Human-Robotic Prosthesis as Collaborating Agents (Wu et al. 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — formalizes the "needs intact-limb coordination" finding as multi-agent.

## Caveats (not verified / limits)
- Figures (Figs.1-6) and Tables I-II **not interpreted** (correlation coefficients
  read from text where stated).
- This is a **biomechanics study**, not an RL-method paper: the RL details are
  identical to P02; the novelty is the impulse↔symmetry analysis.
- Only treadmill, 0.6 m/s, small N (6); authors flag generalization limits (p.13).
