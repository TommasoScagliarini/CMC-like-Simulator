---
id: P06
title: "Learning to Run challenge: Synthesizing physiologically accurate motion using deep reinforcement learning"
authors: "Łukasz Kidziński, Sharada P. Mohanty, Carmichael Ong, Jennifer L. Hicks, Sean F. Carroll, Sergey Levine, Marcel Salathé, Scott L. Delp"
year: 2018
venue: "NIPS 2017 Competition track (Springer); arXiv:1804.00198"
doi_or_url: "arXiv:1804.00198 ; code: github.com/stanfordnmbl/osim-rl"
topics: [drl_opensim, musculoskeletal, reward_design, env_api]
keywords: [osim-rl, OpenSim, muscle-excitation-action, Learning-to-Run, TRPO, DDPG, fall-termination, ligament-penalty, delayed-actuation]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/1 - OpenSim-DRL-locomozione/Learning to Run challenge Synthesizing physiologically accurate motion using deep reinforcement learning.pdf"
pages_read: "1-8 of 16 (overview/model/task/reward/protocol; results & participant solutions skimmed)"
extraction_confidence: high  # clean text; muscle-force equation captured; figures not interpreted
related: [P07, P08, P09, P10, P11, D02, P05]
---

# P06 — "Learning to Run" challenge (the founding OpenSim+RL benchmark)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [drl_opensim](../TOPICS.md) · [musculoskeletal](../TOPICS.md) · [reward_design](../TOPICS.md) · [env_api](../TOPICS.md)

## TL;DR
The NIPS 2017 competition that established **OpenSim as a (D)RL environment**: a
muscle-driven musculoskeletal model where the policy outputs **18 muscle
excitations** and must run an obstacle course as far as possible in 10 s. Ships the
`osim-rl` Gym-style env. This is the **ancestor of the whole project's approach**
(OpenSim + RL, muscle-driven, Gym wrapper, distance+effort reward, fall termination).

## Problem & contribution
- Synthesizing physiologically accurate motion is hard; classic controllers are
  movement-specific or torque-driven (not muscle-driven) (p.1).
- Posed as a competition: build a controller (RL encouraged) for a muscle-driven
  OpenSim human to run an *unknown* obstacle course; no experimental data allowed
  (p.1-2). Showed DRL can synthesize physiologically feasible motion in
  high-dim biomechanical systems (abstract).
- Three RL challenges emphasized: **large action dim, delayed actuation, robustness**
  (p.2).

## Method / architecture (env + model)
- **Model** (p.5): 7 bodies, **9 DoF** (3 pelvis-ground, 1-dof hip/knee/ankle per
  leg), **18 musculotendon actuators** (9/leg). Muscle force:
  `F_muscle = F_max_iso · (a·f_active(l)·f_velocity(v) + f_passive(l))`; tendon
  `F_tendon = F_muscle·cos(α)`; first-order **excitation→activation** dynamics
  (delay). Ligaments = rotational springs at joint limits; Hunt-Crossley foot
  contact (2 spheres/foot) (p.5-6).
- **Observation** `s ∈ R^41`: joint angles/velocities (pelvis, hip, knee, ankle);
  positions/velocities (pelvis, CoM, head, torso, toes, talus); distance + radius +
  height of next obstacle (p.6). Step = 10 ms, 1000 steps over 10 s.
- **Action** `v ∈ [0,1]^18`: muscle **excitations** (not torque) (p.6).
- **Reward** `reward(T) = X(T) − λ ∫₀ᵀ √(L(t)) dt`, X = pelvis x-distance, L = sum
  of squared ligament forces, λ = 1e-7 (tiny) → PDF p.7. **Fall termination**: pelvis
  below **0.65 m** (p.7). *(cf. the project's `pelvis_min_height` truncation.)*
- **`osim-rl`**: OpenSim wrapped as a Gym env (`reset(difficulty,seed)`,
  `step(activations)`), cross-platform (Win/Mac/Linux via Anaconda) → PDF p.7,
  github.com/stanfordnmbl/osim-rl.

## Experimental setup
- Baselines: **TRPO** and **DDPG** (provided in the tutorial) (p.6-7).
- Two-round competition; scored on hidden obstacle environments to force robustness/
  prevent trajectory-tracking "cheating" (p.7-8).

## Key results
- DRL successfully synthesized physiologically feasible running despite high compute
  cost (abstract). *(Top-solution analysis in §5, not extracted; see refs [13,14].)*

## Code / data availability
- **`osim-rl`** (github.com/stanfordnmbl/osim-rl), free/open, cross-platform; OpenSim
  + Simbody backend. *(See also [D02](D02_osim_rl_musculoskeletal.md).)*

## Notable claims (page-anchored)
- Muscle (vs torque) actuation + delayed activation makes the control↔torque map far
  more complex; more actuators needed than torque models (p.3).
- Scoring on an **unknown** environment was explicitly designed so a pure
  trajectory-**tracking** controller would fail → forces generalizable policies (p.7-8).
- Reward = forward progress with a small **effort/ligament penalty**; episode ends on
  fall (pelvis height) (p.7).

## Related notes
- [P07 — DRL for Modeling Human Locomotion in Neuromechanical Simulation (Song et al. 2021)](P07_song2021_drl_neuromechanical_locomotion.md) — the review/position paper of this OpenSim+DRL line (Learn-to-Move successor).
- [P08 — Stairs and Ramps (Adriaenssens et al. 2022)](P08_adriaenssens2022_stairs_ramps.md), [P09 — MBRL walking (Su 2023)](P09_su2023_mbrl_walking.md), [P10 — KINESIS motion imitation](P10_simos_kinesis_motion_imitation.md) — build on this OpenSim+RL paradigm.
- [P11 — DRL Transfemoral (De Vree)](P11_devree_drl_transfemoral.md) — extends OpenSim DRL to transfemoral prosthesis users.
- [D02 — osim-rl docs](D02_osim_rl_musculoskeletal.md) — the environment's documentation.
- [P05 — Collaborating Agents (Wu et al. 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — also validates in OpenSim.

## Caveats (not verified / limits)
- Read 8 of 16 pages: **results, biomechanical analysis of top controllers (§5), and
  organizational details** not fully extracted.
- Figures (Fig.1-4) **not interpreted**.
- This is a **muscle-excitation** action space (full body), opposite end from the
  project's *prosthetic trajectory-generation* action — relevant for OpenSim env
  design, reward/termination, and muscle modeling, not the action contract.
