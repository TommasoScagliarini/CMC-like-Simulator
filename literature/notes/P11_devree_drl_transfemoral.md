---
id: P11
title: "Deep Reinforcement Learning for Physics-Based Musculoskeletal Simulations of Healthy Subjects and Transfemoral Prostheses' Users During Normal Walking"
authors: "Leanne de Vree, Raffaella Carloni (University of Groningen)"
year: 2021
venue: "IEEE TNSRE, vol. 29, pp. 607-618"
doi_or_url: "10.1109/TNSRE.2021.3063015"
topics: [drl_opensim, transfemoral, musculoskeletal, motion_imitation, reward_design]
keywords: [PPO, imitation-learning, OpenSim, transfemoral-amputee-model, muscle-like-actuators, prosthesis, MyLeg, level-ground-walking]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/1 - Deep Reinforcement Learning per Protesi e OpenSim/Deep_Reinforcement_Learning_for_Physics-Based_Musculoskeletal_Simulations_of_Healthy_Subjects_and_Transfemoral_Prostheses_Users.pdf"
pages_read: "1-4 of 12 (intro/background; method & results skimmed)"
extraction_confidence: high  # clean text; figures/tables not interpreted
related: [P08, P06, P07, P09, P02, P05]
---

# P11 — DRL + OpenSim for healthy & transfemoral-amputee walking

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [drl_opensim](../TOPICS.md) · [transfemoral](../TOPICS.md) · [musculoskeletal](../TOPICS.md) · [motion_imitation](../TOPICS.md)

## TL;DR
**The closest paper to the project's domain**: DRL (**PPO + imitation learning**) on
**OpenSim** musculoskeletal models of both a healthy subject and a **transfemoral
amputee**, for level-ground walking. Introduces a generic OpenSim transfemoral model
with **muscle-like actuators on the prosthesis** (2 agonist/antagonist at knee + 2 at
ankle). Same group as [P08]. Paves the way for DRL-based transfemoral prosthesis
control design.

## Problem & contribution
- Bring DRL to OpenSim gait analysis for healthy + impaired subjects; propose **PPO +
  imitation learning** for natural gait with reduced training time (p.1).
- First OpenSim **transfemoral amputee model with muscle-like actuators on a
  prosthetic leg**, designed for DRL (p.1-2).
- Apply DRL to control sound-leg muscles + prosthesis actuator forces to reach a gait
  comparable to healthy; analyze required actuator forces (p.2).

## Method / architecture
- **Healthy model**: 18 muscles (9/leg), **10 DoF** (NIPS'17 lineage model) (p.1).
- **Transfemoral model** (new): **19 muscles, 12 DoF** = 11 sound-leg muscles + 4 at
  the amputated-leg hip + **4 muscle-like actuators in the prosthesis** (2 ago/antag
  at knee, 2 at ankle) (p.1-2).
- **Agent** (Fig.1): DNN receives reward (objective + **imitation-learning term**) +
  observed muscle/joint states; outputs **action = muscle activations + prosthesis
  muscle-like-actuator forces**; sensory feedback used (p.2).
- **Algorithm**: PPO vs **PPO + imitation learning** (proposed); validated on healthy
  vs public dataset, then applied to the amputee model (p.1-2).
- Background reviews **evolutionary optimizers** (CMA-ES, BBO, PSO) vs DRL for gait
  (Table I) — useful comparison; CMA-ES/BBO/PSO used elsewhere for prosthesis control
  (p.2-4).

## Experimental setup
- OpenSim; public healthy-gait dataset for imitation + validation; normal-speed
  level-ground walking; EU H2020 "MyLeg" project (p.1-2).

## Key results
- Transfemoral model achieves **stable gait with forward dynamics comparable to
  healthy**, but using **higher muscle forces** (abstract).
- Computed muscle forces **can't be used directly** as control inputs for muscle-like
  linear actuators (pattern issues), but the approach "paves the way" for DRL-based
  transfemoral prosthesis control (abstract).

## Code / data availability
- OpenSim models; public healthy dataset; **supplementary material** at the DOI. No
  code repo noted in read pages. MyLeg (H2020 #780871).

## Notable claims (page-anchored)
- **PPO + imitation learning** guarantees a natural gait while reducing training time
  vs plain PPO (abstract, p.1).
- A muscle-like-actuator prosthesis can be modeled in OpenSim and driven by DRL
  (p.1-2) — conceptually parallel to the project's **SEA** prosthesis (the project
  uses SEA actuators + a high-level reference, not muscle-like actuators).
- Evolutionary algorithms (CMA-ES/BBO/PSO) are common alternatives to DRL for gait
  optimization (p.2-4).

## Related notes
- [P08 — Stairs and Ramps (Adriaenssens 2022)](P08_adriaenssens2022_stairs_ramps.md) — same group (Carloni/Groningen); PPO + imitation in OpenSim (this is the earlier, walking-only + transfemoral work).
- [P06 — Learning to Run](P06_kidzinski2018_learning_to_run.md), [P07 — Song review](P07_song2021_drl_neuromechanical_locomotion.md), [P09 — MBRL walking](P09_su2023_mbrl_walking.md) — OpenSim+RL musculoskeletal family.
- [P02 — Online RL personalization (Wen 2020)](P02_wen2020_online_rl_personalization.md), [P05 — Collaborating Agents (Wu 2022)](P05_wu2022_human_prosthesis_collaborating_agents.md) — prosthesis RL (hardware / OpenSim-sim respectively).

## Caveats (not verified / limits)
- Read 4 of 12 pages: exact **reward formula, network/PPO details, and quantitative
  gait/force results** not extracted (only abstract).
- Figures (Fig.1) and Table I **not interpreted**.
- The prosthesis here uses **muscle-like actuators**, not SEA + high-level reference
  (the project's setup). Relevant as the closest OpenSim+DRL+transfemoral precedent
  and for the PPO+imitation (warm-start) idea — not as a matching control architecture.
