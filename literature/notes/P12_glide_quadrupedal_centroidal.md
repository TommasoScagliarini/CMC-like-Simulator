---
id: P12
title: "GLiDE: Generalizable Quadrupedal Locomotion in Diverse Environments with a Centroidal Model"
authors: "Zhaoming Xie, Xingye Da, Buck Babich, Animesh Garg, Michiel van de Panne (UBC / NVIDIA / U Toronto)"
year: 2022
venue: "(conference; arXiv) — quadrupedal locomotion"
doi_or_url: "GLiDE (Xie et al.)"
topics: [hierarchical_control, action_space, locomotion_rl, model_based_rl, sim_to_real, reward_design]
keywords: [centroidal-model, reduced-order-model, QP, ground-reaction-forces, Jacobian-transpose, Raibert-foot-placement, hierarchical, simple-reward, sim-to-real]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/2 - Controllo Gerarchico, Spazio delle Azioni e Locomozione/Generalizable Quadrupedal Locomotion in Diverse Environments with a Centroidal Model (GLiDE).pdf"
pages_read: "1-5 of 16 (intro/related/method start; experiments skimmed)"
extraction_confidence: high  # clean text; figures not interpreted
related: [P13, P04, P19, P22]
---

# P12 — GLiDE: RL on a centroidal (reduced-order) model + QP

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [hierarchical_control](../TOPICS.md) · [action_space](../TOPICS.md) · [model_based_rl](../TOPICS.md) · [sim_to_real](../TOPICS.md)

## TL;DR
Train the RL policy on a **highly-abstracted centroidal model** (single rigid body +
massless legs) instead of full-fidelity dynamics. The **high-level policy outputs
desired body linear/angular accelerations**; a **QP** converts them to ground
reaction forces; the full robot realizes them via Jacobian-transpose (stance) +
trajectory tracking (swing). Result: **simple 6-term reward**, **1-2 h training**
(vs 3-58 h), robust sim-to-real. A clean example of **"high-level abstract action +
low-level QP/controller realizes it."**

## Problem & contribution
- Model-free RL usually needs an accurate full-body simulator + complex reward
  (9-12 terms) + millions of samples; MPC uses reduced-order models but generalizes
  poorly (p.1, Table 1).
- Contribution: use a **centroidal model for RL** → anticipatory locomotion/balance
  with simple reward, efficient training, flexible/robust control; a **new action
  space** (centroidal accelerations) with QP + Jacobian-transpose transcription to
  joint space (p.2).

## Method / architecture
- **Centroidal dynamics model**: single rigid body (mass m, inertia I) + 4 massless
  legs, each with a phase φ; state `s = [p, ṗ, R, ω]` (p.4-5).
- **Action** (high level): desired body acceleration `a_d = [p̈_d, ω̇_d]`
  (linear + angular) (p.4).
- **QP**: transcribes `a_d` → ground reaction forces `f_{1..4}`, enforcing no-slip /
  leg-length constraints (p.2, p.5).
- **Realization on full model**: stance legs via **Jacobian transpose** (τ = Jᵀf),
  swing legs via **trajectory tracking**; **Raibert-style foot placement**; assumes a
  given gait pattern. All at **100 Hz** (Fig.2, p.4).
- **Hierarchy**: learned high-level policy (centroidal accelerations) + MPC/QP
  low-level realization → simple task reward at the high level (p.4-5).

## Experimental setup
- Trained purely on the cheap centroidal sim (no full-body simulator / no robot
  data for rollouts); validated on Laikago/A1 sim + **sim-to-real on physical A1**
  (p.2). Tasks: flat, stepping stone, two-legged balance, balance beam.

## Key results (Table 1, p.3)
- **6-term reward** (vs 9-12 for full-physics RL); **1-2 h** training (vs Anymal RL
  3-4 h, DeepGait 58 h); robust **sim-to-real**; solves stepping-stone / balance-beam
  that MPC struggles with.

## Code / data availability
- Video referenced; robots Laikago/A1 (Unitree). No code repo noted in read pages.

## Notable claims (page-anchored)
- Using a **reduced-order (centroidal) model for RL** gives **simple reward + cheap,
  parallelizable training** without a GPU full-physics simulator (p.2-3).
- A **high-level learned policy in an abstract action space** (accelerations) +
  **low-level QP** realization generalizes better than full-model RL or MPC alone
  (p.4-5). *(Directly parallels the project's high-level-reference / low-level-SEA
  split, and the idea of training on a cheaper model.)*
- Easily adapted to different robots by changing the compact centroidal parameters
  (mass, inertia) (p.4).

## Related notes
- [P13 — Learning Torque Control for Quadrupedal Locomotion](P13_learning_torque_control_quadruped.md) — action-space study (torque vs position/abstract) on quadrupeds.
- [P04 — Hierarchical Optimization (Li et al. 2023)](P04_li2023_hierarchical_knee_symmetry.md) — another high-level/low-level split (BO target + RL tracking).
- [P19 — ECO](P19_eco_energy_constrained.md), [P22 — LSWM](P22_lswm_world_model_bipedal.md) — locomotion-RL with the position-deviation/abstract action + low-level realization.

## Caveats (not verified / limits)
- Read 5 of 16 pages: the **exact reward terms, QP formulation, training algorithm,
  and quantitative task/sim-to-real results** not extracted.
- Figures (Fig.1 centroidal model, Fig.2 system) **not interpreted**.
- Domain = **quadruped**, not biped/prosthesis/OpenSim. Transferable ideas: the
  hierarchical abstract-action + QP realization, reduced-order training for speed,
  simple reward — not the specific dynamics.
