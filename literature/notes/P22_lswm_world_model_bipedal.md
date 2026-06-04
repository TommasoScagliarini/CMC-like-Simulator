---
id: P22
title: "LSWM: A Long–Short History World Model for Bipedal Locomotion via Reinforcement Learning"
authors: "Jie Xue, Zhiyuan Liang, Haiming Mou, Qingdu Li, Jianwei Zhang"
year: 2026
venue: "Biomimetics 2026, 11(1), 40 (MDPI, open access)"
doi_or_url: "10.3390/biomimetics11010040"
topics: [pomdp, memory, world_model, bipedal, locomotion_rl, reward_design, sim_to_real, action_space]
keywords: [POMDP, state-reconstruction, future-state-prediction, privileged-information, asymmetric-actor-critic, teacher-student, DreamWaQ, PPO, history-window]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/5 - Osservabilità Parziale (POMDP) e Reti Ricorrenti (Memoria)/LSWM - A Long–Short History World Model for Bipedal Locomotion via Reinforcement Learning.pdf"
pages_read: "1-8 of 20 (intro/related/method/reward; experiments skimmed)"
extraction_confidence: high  # clean text, equations and reward table clean; figures not interpreted
related: [P05, P19, P13]
---

# P22 — LSWM: Long–Short History World Model (POMDP / memory)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [pomdp](../TOPICS.md) · [memory / world_model](../TOPICS.md) · [reward_design](../TOPICS.md) · [bipedal](../TOPICS.md)

## TL;DR
Treats bipedal locomotion as a **POMDP** and tackles partial observability with two
modules: a **State Reconstruction Module (SRM)** that reconstructs noise-free
**short-term privileged** state from **long-term noisy history**, and a **State
Prediction Module (SPM)** that forecasts **future short-term** privileged state.
Both feed an **asymmetric actor-critic** (PPO), trained in a single phase. The most
directly relevant paper in this corpus for the **observation / memory** question.

## Problem & contribution
- Sim policies degrade in the real world due to sensor noise, unobservable states,
  disturbances (sim2real gap) (p.1-2).
- Prior work reconstructs only *current* single-frame privileged state from
  long-term history, underusing **short-term history** (fast response) and **future
  prediction** (anticipating risk) (p.2).
- Contributions: (1) reconstruct short-term privileged info from long-term history
  (robust to noise, better observability); (2) predict short-term future privileged
  info; (3) extensive indoor/outdoor real tests (p.2).

## Method / architecture
- **POMDP** `M = {S, O, A, T, Z, R, γ}` → PDF p.4, eq.(1); objective
  `J(π) = E[Σ γ^t r_t]` → eq.(2).
- **Observation** `o_t = [phase, cmd, q_t, q̇_t, Φ(Euler), ω]` → PDF p.5, eq.(3);
  full state `s_t = [o_t, v_t, h_t]` with `v_t` linear velocity and `h_t` privileged
  (elevation map, friction) → eq.(4).
- **SRM** (state reconstruction): encoder `E_SRM(ṽ_t, z_t | o_{H1})` from long-term
  history → estimated linear velocity ṽ_t + latent z_t; decoder reconstructs
  short-term privileged `s̃_r`. Loss `L_SRM = MSE(ṽ_t,v_t) + MSE(s̃_r,s_H)`
  → PDF p.5, eq.(5).
- **SPM** (future prediction): encoder `E_SPM(z^p_t | o_{H3})` → latent; decoder
  predicts short-term future privileged `s̃_p`. Loss `L_SPM = MSE(s̃_p, s_H)`
  → PDF p.6, eq.(6). Lets the robot anticipate hyperextension/slip/height
  instability (p.6).
- **Asymmetric actor-critic** (PPO): critic sees privileged `s̃_H`; policy
  `π(a | z_t, z^p_t, ṽ_t)` uses only the SRM/SPM latents + estimated velocity
  → PDF p.7, PPO loss eq.(7), value loss eq.(8). **Joint single-phase** total loss
  `L_LSWM = L_SRM + L_SPM + L_π + L_v` → eq.(9).
- **Action space**: incremental target joint positions (10 DoF) → PD controller
  (same "position-deviation" pattern as P19) (p.7).

## Experimental setup
- Robot X02 (Shanghai DroidUp): 1.7 m, 32 kg, 20 DoF, controlling 10 lower-body DoF
  (p.7-8).
- Domain randomization (Table 1): joint pos noise ±0.05 rad, joint vel ±2 rad/s,
  Euler ±0.05, ang.vel ±0.5, COM ±7.5 cm, payload ±5 kg, friction [0.1,2], Kp/Kd
  ±15% (p.6).
- Reward (Table 2, useful menu): lin/ang velocity tracking (exp), lin-vel-z,
  ang-vel-xy, orientation, base height, **joint torque ‖τ‖²**, **joint accel q̈²**,
  **joint power |τq̇|**, action rate, feet distance/stance/swing, hip deviation,
  joint-limit, collision penalties (p.7-8).

## Key results
- Indoor stair-climbing: **94% success**, ≥34% above SOTA baselines (abstract, p.2).
  *(Detailed comparisons in §5, not extracted.)*

## Code / data availability
- Open-access (MDPI, CC-BY). No explicit code repo noted in the read pages.
  Compares to DreamWaQ, DWL, Dreamer-family, teacher-student distillation [13].

## Notable claims (page-anchored)
- Bipedal locomotion is modeled as a **POMDP**; the core challenge is using **history
  observation sequences** to reconstruct full state (p.2, p.4).
- **Short-term history** is critical for fast state response; **long-term history**
  for adaptation — they should be combined (p.2).
- Naive fixes have downsides: filtering breaks the **Markov property**; penalizing
  policy jumps in the reward is hard to tune and can cause oscillations (p.2).
- **Future-state prediction** enables proactive risk avoidance, not just reactive
  reward following (p.6).
- Asymmetric actor-critic (privileged critic) avoids a separate distillation phase
  (p.7).

## Related notes
- [P05 — Human-Robotic Prosthesis Collaborating Agents (Wu et al. 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — also frames partial observability (CTDE/POMDP) in a coupled system.
- [P19 — ECO (Huang et al. 2026)](P19_eco_energy_constrained.md) — shares the asymmetric actor-critic + privileged critic + position-deviation action + torque/power reward terms.
- [P13 — Learning Torque Control for Quadrupedal Locomotion](P13_learning_torque_control_quadruped.md) — action-space (position vs torque) comparison.

## Caveats (not verified / limits)
- Read pages 1-8 of 20: **experiments/ablations (§4-5) and architecture figure
  (Fig.1) not extracted** — read PDF for quantitative comparisons and exact net
  sizes / history lengths H1,H2,H3.
- Domain is a **20-DoF humanoid in sim+real** (not OpenSim / not a prosthesis); the
  transferable ideas are the **POMDP handling (history reconstruction + future
  prediction)** and the reward/observation menus.
- No spiking-network content (the project's SNN choice is orthogonal); LSWM uses
  MLP encoders/decoders + PPO.
