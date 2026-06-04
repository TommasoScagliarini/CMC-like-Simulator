---
id: P09
title: "Simulating human walking: a model-based reinforcement learning approach with musculoskeletal modeling"
authors: "Binbin Su, Elena M. Gutierrez-Farewik (KTH)"
year: 2023
venue: "Frontiers in Neurorobotics 17:1244417"
doi_or_url: "10.3389/fnbot.2023.1244417"
topics: [drl_opensim, model_based_rl, musculoskeletal, reward_design, reflex_control]
keywords: [CMA-ES, reflex-based-control, OpenSim-RL, OpenAI-Gym, Hill-type-muscle, no-reference-data, pathological-gait, pelvis-reward]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/1 - OpenSim-DRL-locomozione/Simulating human walking a model-based reinforcement learning approach with musculoskeletal modeling.pdf"
pages_read: "1-3 of 12 (intro/methods/reward start; results skimmed)"
extraction_confidence: high  # clean text; reward formula partially captured; figures not interpreted
related: [P06, P07, P08, P10, P20]
---

# P09 — Simulating human walking: reflex policy + CMA-ES in OpenSim

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [drl_opensim](../TOPICS.md) · [model_based_rl](../TOPICS.md) · [reflex_control](../TOPICS.md) · [reward_design](../TOPICS.md)

## TL;DR
Generates human walking in an OpenSim musculoskeletal model **without reference motion
data**, using a **reflex-based controller as the policy** (Song & Geyer lineage)
optimized with **CMA-ES** (derivative-free). Reward = survival + footstep terms with a
**novel pelvis-tilt component** for naturalness. Reproduces hip/knee kinematics (ankle
less well) at self-selected (1.45 m/s) and imposed speeds; can predict pathological
(muscle-weakness) gait.

## Problem & contribution
- Pure deep-RL torque controllers look unnatural (large upper-body motion, abnormal
  posture); a biomechanical muscle model improves realism (p.1-2).
- Reviews the **3 musculoskeletal control frameworks**: trajectory tracking (can't
  predict novel motion), optimal control (needs hand-crafted cost, expensive),
  **reflex-based** (no reference, naturalistic, adapts) (p.2).
- Contribution: RL (MDP + CMA-ES) to solve the parameters of a **reflex-based**
  controller in a complex musculoskeletal model; reward design that yields stable
  gaits similar to able-bodied across speeds and under muscle weakness (p.2).

## Method / architecture
- Platform: **OpenSim-RL** ([P06]) embedding OpenSim + OpenAI Gym (p.3).
- Model: simplified **2D**, 6 internal DoF (hip/knee/ankle flex-ext), **18 Hill-type
  muscles** (9/leg, non-compliant tendons), forward dynamics; first-order activation
  dynamics (Zajac) (p.3).
- **MDP** `<S, A, R>`: S = joint pos/vel, ground contact; A = muscle excitations;
  deterministic policy π:S→A (p.3).
- **Policy = reflex-based gait controller** (Song & Geyer 2015) mapping body state →
  muscle excitations; its parameters optimized by **CMA-ES** (derivative-free) — this
  is the "model-based RL" framing (p.2-3).
- **Reward** `r = R_alive + R_steps` (survival + footstep); high when walking at
  desired velocity with **minimum muscle effort and pelvis tilt** → PDF p.3. **Novel
  pelvis term** added for natural pelvis tilt (p.2). dt = 0.01 s; **fall = pelvis
  below 0.6 m** (p.3). *(cf. project's pelvis-height truncation.)*

## Experimental setup
- No imposed speed → model settles at **1.45 m/s**; then a range of imposed speeds.
  All without MoCap reference; experimental data used for comparison only (p.3).

## Key results
- Hip and knee kinematics agree well with experiment; **ankle less well predicted**
  (abstract).
- Framework can model/predict **pathological gait from muscle weakness** (abstract).

## Code / data availability
- OpenSim-RL platform; experimental data for comparison only. Open access (CC-BY).

## Notable claims (page-anchored)
- Without a biomechanical muscle model, RL gaits look unnatural; muscles + reflex
  structure restore realism (p.1-2).
- **Reflex-based control** needs no predefined movement and adapts to slopes/
  disturbances without parameter intervention (Geyer & Herr) (p.2). Cited for powered
  ankle-foot prostheses (Eilenberg, Thatte) — reflex control transferred to protheses.
- Optimal-control predictive sims hinge on a cost criterion (energy vs muscle
  activity) and "which best represents reality remains unclear" (p.2) — caution on
  reward/objective choice.

## Related notes
- [P06 — Learning to Run (Kidziński 2018)](P06_kidzinski2018_learning_to_run.md) — the OpenSim-RL platform used here.
- [P07 — DRL neuromechanical review (Song 2021)](P07_song2021_drl_neuromechanical_locomotion.md) — reflex-control vs traj-opt vs DRL landscape (Song & Geyer).
- [P08 — Stairs and Ramps (Adriaenssens 2022)](P08_adriaenssens2022_stairs_ramps.md) — OpenSim+PPO+imitation (vs reflex+CMA-ES here).
- [P10 — KINESIS motion imitation](P10_simos_kinesis_motion_imitation.md) — imitation-based OpenSim locomotion.
- [P20 — Powered Ankle-Foot Prosthesis (Au et al.)](P20_au2009_powered_ankle_foot.md) — reflex control applied to ankle prostheses.

## Caveats (not verified / limits)
- Read 3 of 12 pages: full reward formula (R_alive, R_steps terms), CMA-ES settings,
  and quantitative kinematics comparisons **not extracted**.
- Figures **not interpreted**.
- 2D, muscle-excitation action — relevant for OpenSim reward/termination design and
  the reflex-control + derivative-free-optimization alternative, not the prosthetic
  action contract. "Model-based RL" here ≈ reflex-controller params via CMA-ES.
