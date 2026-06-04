---
id: P02
title: "Online Reinforcement Learning Control for the Personalization of a Robotic Knee Prosthesis"
authors: "Yue Wen, Jennie Si, Andrea Brandt, Xiang Gao, He (Helen) Huang"
year: 2020
venue: "IEEE Transactions on Cybernetics, vol. 50, no. 6, pp. 2346-2356"
doi_or_url: "10.1109/TCYB.2019.2890974"
topics: [prosthesis_knee, online_rl, impedance_control, human_in_loop, safety_bounds, adp_dhdp, action_param_tuning]
keywords: [dHDP, ADP, adaptive-critic, actor-critic, FSM impedance control, transfemoral, personalization, OpenSim-precursor]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/2 - Protesi robotiche di ginocchio-transfemorali/Online Reinforcement Learning Control for the Personalization of a Robotic Knee Prosthesis.pdf"
pages_read: "1-11 (full; refs skimmed)"
extraction_confidence: high  # prose + key eqs clean; the dHDP NN-update eqs (9-22) partly garbled → PDF pointers
related: [P01, P03, P04, P05, P11]
---

# P02 — Online RL (dHDP/ADP) for Personalizing a Robotic Knee Prosthesis

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [prosthesis_knee](../TOPICS.md) · [online_rl](../TOPICS.md) · [impedance_control](../TOPICS.md) · [safety_bounds](../TOPICS.md)

## TL;DR
First online Approximate Dynamic Programming (ADP) controller — specifically
**direct Heuristic Dynamic Programming (dHDP)**, an actor-critic — to auto-tune the
**12 impedance parameters** of a powered knee prosthesis, with a human in the loop,
to reach a **fixed normative** target knee profile. This is the **predecessor** of
[P01](P01_wu2022_rl_impedance_echo_control.md) (which later replaced the fixed
target with the time-varying intact-knee "echo" target). Tested on 1 able-bodied +
1 transfemoral amputee; converges in ~300 gait cycles (~10 min).

## Problem & contribution
- Powered prostheses expose many control parameters (12–15 for knees) that must be
  personalized; manual clinic tuning is slow/labor-intensive (p.1).
- Contribution: first online ADP (dHDP) learning controller applied to a clinical
  human-subject problem; model-free tuning of 12 impedance parameters online;
  shown feasible and (toward) safe (p.2).
- A **simulation precursor** validated dHDP on an **OpenSim** amputee gait model
  before human trials (ref [44]); dHDP beat NFQCA there (p.2). *(Directly relevant
  as prior art for OpenSim + prosthesis RL.)*

## Method / architecture
Three time scales (p.2, Fig.1): real-time impedance controller @100 Hz; FSM at gait
frequency (4 phases); dHDP parameter update every few gaits.
- **Low level — FSM Impedance Control**, 4 phases (STF, STE, SWF, SWE). Per phase
  `I_m = [k_m, θe_m, b_m]ᵀ` → PDF p.3, eq.(1); torque `τ_m = k_m(θ−θe_m) + b_mω`
  → PDF p.3, eq.(2). 12 parameters total.
- **Knee kinematics features**: peak angle P and duration D per phase, `[P_m, D_m]`;
  target = normative features `[P̄_m, D̄_m]` (p.3).
- **Control input / output**: `I(n)=[k,θe,b]ᵀ`, output features `x(n)=[P,D]ᵀ`
  → PDF p.3, eq.(3).
- **Action update** (increments): `I(n) = I(n−1) + β ⊙ U(n−1)`, β scaling factors,
  ⊙ Hadamard → PDF p.3, eq.(4). Updated **every 7 gait cycles**.
- **RL state** (4-dim per phase, scaled to [−1,1]):
  `X(n) = γ ⊙ [ x(n)−x̄ , x(n)−x(n−1) ]ᵀ` — i.e. **feature error + feature change
  rate** → PDF p.3, eq.(5).
- **Reward / reinforcement signal** (sparse, safety-shaped) → PDF p.4, eq.(6):
  `r(n) = −1` if `x(n) ∉ [Bl,Bu]` (safety bounds → params reset to default);
  `−0.8` if penalty score `S⁻ > 4` (tuning in an unfavorable direction);
  `0` otherwise.
- **Cost-to-go** `J(n)=r(n+1)+αJ(n+1)` (discounted, α∈(0,1)) → PDF p.4, eqs.(7)-(8).
- **Algorithm — dHDP** (adaptive critic): Critic NN (7-7-1) predicts cost-to-go Ĵ;
  Action NN (4-7-3) outputs U; tan-sigmoid activations; gradient weight updates.
  **Four identical parallel dHDP blocks**, one per gait phase. The NN forward/update
  equations are *heavy/garbled in extraction* → PDF p.4-5, eqs.(9)-(22) and
  Algorithm 1 (p.5). Weights init uniform random in [−1,1]; dHDP has uniformly-
  ultimately-bounded property under mild conditions (p.5).

## Experimental setup
- 1 AB (L-socket adapter) + 1 transfemoral amputee; Bertec treadmill @0.6 m/s; ≥5
  training sessions; 100 Hz kinematics (p.6).
- Robust feature extraction against sensor noise (window ±10 samples, eq.(23) p.6);
  outlier rejection (>1.5 SD over 7 cycles) (p.6).
- **Safety bounds**: feature errors 1.5×SD of non-amputee kinematics
  (STF 10.5°, STE 7.5°, SWF 9°, SWE 6°); ROM bound [−5°, 60°]; on violation reset to
  defaults + r=−1 (p.6).
- Naïve ADP (random init) vs **experienced** ADP (weights transferred from a prior
  successful session). Termination: ≤70 iterations (=490 gaits) or success
  (errors within tolerance for 3 of last 5 iterations) (p.6).

## Key results
- RMSE knee angle (vs target): 5.83±0.85° → 3.99±0.62° after tuning (p.7).
- Angle feature errors decrease consistently; **duration errors inconsistent**
  (partly human-controlled) (p.7).
- Data/time efficiency: ~43±10 iterations ≈ 300 gait cycles ≈ 10±2 min;
  **experienced ADP faster**: 28 iterations ≈ 7 min ≈ 210 cycles, no extra
  reinforcement signals (p.7-8).
- Multiple impedance combinations give similar kinematics (torque underdetermined
  by 3 params, eq.2) (p.8). Knee peaks driven by impedance params; phase duration
  largely by the human user (p.8).

## Code / data availability
- **No public code/dataset** (human-subject hardware study; NC State / ASU / UNC).
- OpenSim **simulation precursor** in ref [44]: Gao et al. (dHDP vs NFQCA on an
  OpenSim amputee model) — the closest prior art to "prosthesis RL in OpenSim".

## Notable claims (page-anchored)
- "For weight-bearing prostheses, **safety is the primary concern**" → constraints
  encoded as hard reset + −1 reinforcement, bounds from non-amputee SDs (p.6).
- State = **feature error + feature change rate** (not raw trajectory) (p.3, eq.5).
- The controller has **no plant model and no prior tuning knowledge**; only observes
  knee-angle features + safety reinforcement (p.9).
- "questions regarding the **appropriate control objective remain open**" — knee
  kinematics chosen as a reasonable first step, but symmetry/stability/preference
  are flagged as better future objectives (p.8, p.10).

## Related notes
- [P01 — RL Impedance Echo Control (Wu et al. 2022)](P01_wu2022_rl_impedance_echo_control.md) — **direct successor**: same FSM-IC + per-phase RL, but tracks the time-varying intact knee instead of this fixed normative target.
- [P03 — Wearer-Prosthesis Interaction for Symmetrical Gait (Wen et al. 2020)](P03_wen2020_wearer_prosthesis_symmetry.md) — same group; studies the human side of this loop.
- [P04 — Hierarchical Optimization for Robotic Knee Prostheses (Li et al. 2023)](P04_li2023_hierarchical_knee_symmetry.md) — extends to hierarchical high-level objective.
- [P05 — Human-Robotic Prosthesis as Collaborating Agents (Wu et al. 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — MARL view of the same human-prosthesis system.
- [P11 — DRL Physics-Based Musculoskeletal Transfemoral (De Vree et al.)](P11_devree_drl_transfemoral.md) — OpenSim-based transfemoral RL (different style: full simulation, not hardware tuning).

## Caveats (not verified / limits)
- The **dHDP neural-network equations (9)-(22)** and Algorithm 1 garbled on text
  extraction — read exact forms at PDF p.4-5.
- Figures (Fig.1 block diagram; Fig.2 feature def.; Figs.3-8 results; Table I final
  params) **not interpreted**.
- Framing (same as P01): RL here is a **per-phase adaptive-critic tuner updating
  every 7 gait cycles** over a low-dim feature state — NOT a control-rate deep-RL
  policy nor a trajectory generator.
