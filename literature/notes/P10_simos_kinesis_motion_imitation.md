---
id: P10
title: "KINESIS: Motion Imitation for Human Musculoskeletal Locomotion"
authors: "Merkourios Simos, Alberto Chiappa, Alexander Mathis (EPFL)"
year: 2026
venue: "arXiv:2503.14637 (v3, Mar 2026)"
doi_or_url: "arXiv:2503.14637 ; code: github.com/amathislab/Kinesis"
topics: [motion_imitation, musculoskeletal, warm_start, pomdp, reward_design]
keywords: [motion-imitation, DeepMimic-reward, MyoSuite, MyoLeg, MuJoCo, EMG-validation, KIT-Locomotion, 290-muscles, model-free, negative-mining, text-to-control]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/1 - OpenSim-DRL-locomozione/KINESIS - Motion Imitation for Human Musculoskeletal Locomotion.pdf"
pages_read: "1-3 of 10 (intro/related/methods/reward start; experiments skimmed)"
extraction_confidence: high  # clean text; reward equation captured; figures not interpreted
related: [P06, P07, P08, P09, P22]
---

# P10 — KINESIS: muscle-driven motion imitation (DeepMimic-style)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [motion_imitation](../TOPICS.md) · [musculoskeletal](../TOPICS.md) · [warm_start](../TOPICS.md) · [pomdp](../TOPICS.md)

## TL;DR
**Model-free motion-imitation** RL for **muscle-driven** locomotion: track MoCac
reference motion with a musculoskeletal model, scaling to **290 muscles**. Trained on
1.8 h of curated MoCap (KIT-Locomotion). Produces EMG-correlated muscle activity and
supports downstream tasks (text-to-control, target reaching, penalty kicks). Runs in
**MuJoCo/MyoSuite** (faster than OpenSim). Code is public.

## Problem & contribution
- Torque-controlled humanoids miss biomechanical joint constraints + overactuated
  musculotendon control; muscle-driven locomotion RL lags behind torque RL (p.1).
- Contribution: single-stage, model-free imitation framework, applied with ~no
  hyperparameter tuning to 3 models of increasing complexity (80/86/290 muscles);
  **EMG-validation benchmark**; downstream task transfer; ~3.8× real-time on laptop
  CPU (no distillation) (p.1-2).

## Method / architecture
- **Models**: MyoLeg variants from **MyoSuite** (MuJoCo), biologically validated
  against an OpenSim full-body model: *Legs* (80 muscles), *Legs+Abs* (86),
  *Legs+Back* (290) (p.2). Body dims match SMPL → seamless MoCap use.
- **Data**: KIT-Locomotion (AMASS/SMPL), 1054 clips, 5 skills (walk, gradual turn,
  turn-in-place, walk-backwards, run); 30 fps; IK to align with model (p.2-3).
- **POMDP** `⟨S, O, A, T, R, γ⟩` (p.3). **Observation** (R^309/363/453): pelvis
  height/tilt/velocity, per-joint kinematics (pos/vel/ang pos/ang vel), feet contact
  forces, **+ target reference pose at t+1** (absolute & relative). **Action** =
  per-muscle control signal `a ∈ R^M` (p.3).
- **Reward** (DeepMimic-style imitation): `r^pos_t = exp(−k_pos Σ_i ‖p^i_t − p̂^i_{t+1}‖²)`
  — negative Euclidean distance between body joint positions and target pose
  → PDF p.3. Plus (later) velocity/effort terms (not extracted). **Negative mining**
  for robust priors (p.1).

## Experimental setup
- MuJoCo/MyoSuite; reference body set {head, pelvis, knees, ankles, toes};
  random-start initialization from any reference frame (p.3).

## Key results
- Strong imitation on **unseen** trajectories; scales seamlessly to 290 muscles;
  muscle activity **correlates with human EMG**; zero-shot text-conditioned motion +
  fine-tuned target reaching / football kicks (abstract, p.1).

## Code / data availability
- **Code/videos/benchmarks**: github.com/amathislab/Kinesis. Data: KIT-Locomotion
  (AMASS). Models: MyoSuite (open).

## Notable claims (page-anchored)
- OpenSim is "**computationally costly** compared to physics engines used in
  robotics/RL" → they use MyoSuite/MuJoCo ports (p.2). *(Supports the S01 idea of a
  faster surrogate for pretraining.)*
- Observation explicitly includes the **target reference pose at t+1** — i.e. the
  policy sees the upcoming reference (p.3) — relevant to reference-tracking policies.
- Single-stage model-free imitation generalizes across body models with ~no tuning
  (p.1-2).

## Related notes
- [P06 — Learning to Run](P06_kidzinski2018_learning_to_run.md), [P07 — Song review](P07_song2021_drl_neuromechanical_locomotion.md), [P08 — Stairs/Ramps](P08_adriaenssens2022_stairs_ramps.md), [P09 — MBRL walking](P09_su2023_mbrl_walking.md) — same muscle-driven-locomotion family (OpenSim/MuJoCo).
- [P22 — LSWM](P22_lswm_world_model_bipedal.md) — also a POMDP locomotion formulation (state reconstruction vs imitation here).

## Caveats (not verified / limits)
- Read 3 of 10 pages: full reward (velocity/effort terms), training algorithm
  (PPO?), negative-mining details, and quantitative imitation/EMG results **not
  extracted**.
- Runs in **MuJoCo (MyoSuite)**, not OpenSim directly (MyoLeg is OpenSim-validated).
- Full-body muscle imitation, not a prosthetic generator — relevant for
  **imitation-reward design**, **observation with look-ahead reference**, and the
  **OpenSim→MuJoCo speed** argument.
