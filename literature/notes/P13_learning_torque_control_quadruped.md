---
id: P13
title: "Learning Torque Control for Quadrupedal Locomotion"
authors: "Shuxiao Chen, Bike Zhang, Mark W. Mueller, Akshara Rai, Koushil Sreenath (UC Berkeley / Meta AI)"
year: 2023
venue: "IEEE-RAS Humanoids 2023 (arXiv:2203.05194)"
doi_or_url: "arXiv:2203.05194"
topics: [action_space, torque_vs_position, locomotion_rl, sim_to_real]
keywords: [torque-control, position-control, PD-controller, action-space-design, residual-position, learned-PD-gains, IsaacGym, query-frequency]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/2 - Controllo Gerarchico, Spazio delle Azioni e Locomozione/Learning Torque Control for Quadrupedal Locomotion.pdf"
pages_read: "1-3 of 8 (intro/related/formulation; experiments skimmed)"
extraction_confidence: high  # clean text; figures not interpreted
related: [P12, P19, P22]
---

# P13 — Learning Torque Control (action-space: torque vs position)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [action_space](../TOPICS.md) · [torque_vs_position](../TOPICS.md) · [locomotion_rl](../TOPICS.md) · [sim_to_real](../TOPICS.md)

## TL;DR
The reference on the **position-vs-torque action-space** question. Conventional RL
locomotion is **position-based** (policy outputs target joint positions at low freq →
high-freq **PD** produces torque); this paper learns **torque control directly** at
high freq, no PD. First sim-to-real end-to-end torque RL for quadrupeds; achieves
higher reward and more disturbance robustness than position control. Its discussion
of PD-gain effects is directly relevant to the project's knee high-gain instability.

## Problem & contribution
- Model-based control shifted position→torque for more dynamic/robust behavior; RL
  still mostly uses position + PD. They ask whether RL should too (p.1).
- Contributions: sim-to-real **torque-based RL** (policy streams joint torques, no
  PD); validated on terrain + disturbances; **higher reward** and **more robust to
  large disturbances** than position control; implementation guidance (p.2).

## Method / architecture
- **RL formulation**: standard MDP, maximize `J(π)=E[Σ γ^t r_t]` (p.2).
- **Position-based (baseline)**: policy → target joint positions (low freq) → PD
  (`τ = K_p(q_des−q) + K_d(−q̇)`, high freq) → torque (Fig.2, p.3).
- **Torque-based (proposed)**: policy → joint torques **directly at high frequency**,
  no PD (Fig.2, p.3).
- Sim: **IsaacGym**, 4096 parallel envs, rough-terrain curriculum (p.2).

## Action-space design literature (the useful part) — p.2
- **Joint position** + PD: PD treated as low-level tracker, gains tuned manually;
  variant = **residual position** (heuristic/optimization base + learned residual).
- **High P-gains → instability**; **low P-gains → behaves like a torque controller,
  more robust** but larger tracking error. *(Mirrors the project's knee limit-cycle:
  high inner gain → instability.)*
- **Learning PD gains** as part of the action → adaptive stiffness/damping, but
  gain switching → **nonsmooth torques / instability**.
- **Torque** needs a **higher query frequency** than position to match performance;
  torque improves as query rate increases. Position control = quicker early learning
  but can converge to **lower-reward** behaviors and hinder fast adaptation.

## Experimental setup / key results
- Quadruped on varied terrain + external pushes, following user commands; torque RL
  resists larger disturbances and reaches higher reward than position RL; first
  sim-to-real torque RL for quadrupeds (abstract). *(Quantitative results §IV-V not
  extracted.)*

## Code / data availability
- IsaacGym; experiment video linked; no code repo noted in read pages.

## Notable claims (page-anchored)
- **High proportional gains cause instability** on the robot; low gains → torque-like,
  more robust but worse tracking (p.2). *(Directly echoes the project's finding that
  the high knee inner gain drives a saturating limit-cycle.)*
- Learning PD gains can cause **nonsmooth torque / instability** at gain switches (p.2)
  — caution against adaptive-gain action designs.
- Torque control needs **high query frequency**; position+PD is gentler to learn but
  may cap performance/adaptivity (p.2).

## Related notes
- [P12 — GLiDE](P12_glide_quadrupedal_centroidal.md) — abstract (centroidal-acceleration) action + QP, vs raw position/torque here.
- [P19 — ECO](P19_eco_energy_constrained.md), [P22 — LSWM](P22_lswm_world_model_bipedal.md) — both use position-deviation actions + PD (the paradigm this paper contrasts with torque).

## Caveats (not verified / limits)
- Read 3 of 8 pages: **observation/network details, reward, and quantitative results**
  not extracted.
- Figures (Fig.2 architecture) **not interpreted**.
- Domain = quadruped (IsaacGym), not OpenSim/prosthesis. Transferable: the
  **action-space tradeoffs (position+PD vs torque vs learned-gains)** and the
  **high-gain instability** caution — relevant to the project's SEA cascade tuning.
