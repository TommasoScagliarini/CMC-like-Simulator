---
id: P08
title: "Learning to Ascend Stairs and Ramps: Deep Reinforcement Learning for a Physics-Based Human Musculoskeletal Model"
authors: "Aurelien J. C. Adriaenssens, Vishal Raveendranathan, Raffaella Carloni"
year: 2022
venue: "Sensors 2022, 22(21), 8479 (MDPI, open access)"
doi_or_url: "10.3390/s22218479"
topics: [drl_opensim, musculoskeletal, motion_imitation, reward_design, curriculum]
keywords: [PPO, imitation-learning, OpenSim, muscle-excitation, stair-ascent, ramp-ascent, elastic-foundation-contact, CMU-mocap, prosthesis-goal]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/1 - OpenSim-DRL-locomozione/Learning to Ascend Stairs and Ramps Deep Reinforcement Learning for a Physics-Based Human Musculoskeletal Model.pdf"
pages_read: "1-4 of 16 (intro/model/environment; reward details & results skimmed)"
extraction_confidence: high  # clean text; figures/tables not interpreted
related: [P06, P07, P09, P10, P11]
---

# P08 — Learning to Ascend Stairs & Ramps (PPO + imitation in OpenSim)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [drl_opensim](../TOPICS.md) · [musculoskeletal](../TOPICS.md) · [motion_imitation](../TOPICS.md) · [reward_design](../TOPICS.md)

## TL;DR
DRL (**PPO + imitation learning**) teaches an OpenSim musculoskeletal model to
**ascend stairs and ramps** with muscle forces/forward-dynamics comparable to healthy
subjects, trained on public MoCap data. Explicitly motivated by the **long-term goal
of lower-limb prosthesis control**. Also shows how to extend OpenSim contact (elastic
foundation) for non-level terrain.

## Problem & contribution
- Learning complex, physiologically plausible motions in musculoskeletal sim is hard;
  prior OpenSim+DRL focused on level walking (p.1).
- Contributions: (1) PPO + imitation learning teaches stair/ramp ascent in OpenSim
  (toward prosthesis control); (2) replaces the level-only **Hunt–Crossley** contact
  with the **elastic foundation** contact model + adds stair/ramp object meshes to
  study non-level terrain (p.2).

## Method / architecture
- **Model** (from the NIPS'17 / [P06] model): 7 bodies, **18 Hill-type muscles**
  (9/leg: 6 uniarticular + 3 biarticular), **14 DoF** (6 pelvis: tilt/list/rot + XYZ;
  2/hip flex+ad/abduction; 1/knee; 1/ankle) (p.3). First-order excitation→activation.
  Max isometric force +80% for the stair task (p.3).
- **Observation**: joint kinematics + muscle forces; **action** = muscle excitations
  (p.1, Fig.1).
- **Reward** = objective function + **imitation-learning term** (track MoCap reference)
  (p.1-2). *(Imitation/warm-start relevant to the project's hybrid-training option.)*
- **Environment changes** (Table 1): added stair/ramp meshes (BodySet); spherical foot
  contact meshes (ContactGeometrySet); **elastic foundation** force model replacing
  Hunt–Crossley (ForceSet) (p.3-4).

## Experimental setup
- OpenSim 3.3; CMU Graphics Lab Motion Capture Database as imitation + validation data;
  pelvis-torso angle fixed −15° (stairs) / −5° (ramp) (p.3).

## Key results
- Muscle forces comparable to healthy subjects; forward-dynamics correlation with
  experimental data **0.82 (stair ascent)** and **0.58 (ramp ascent)** across knee &
  ankle (abstract).

## Code / data availability
- OpenSim 3.3 .osim model (NIPS'17 lineage); CMU MoCap dataset (public). MDPI open
  access (CC-BY). No code repo noted in read pages.

## Notable claims (page-anchored)
- **PPO + imitation learning** can teach physiologically plausible complex locomotion
  in OpenSim, explicitly *toward lower-limb prosthesis control* (p.1-2).
- To go beyond level ground in OpenSim you must swap **Hunt–Crossley → elastic
  foundation** contact and add object meshes (p.3-4) — a practical OpenSim modeling
  note.

## Related notes
- [P06 — Learning to Run (Kidziński 2018)](P06_kidzinski2018_learning_to_run.md) — source of the musculoskeletal model and the OpenSim+RL paradigm.
- [P07 — DRL neuromechanical review (Song 2021)](P07_song2021_drl_neuromechanical_locomotion.md) — the broader landscape (imitation vs traj-opt vs reflex).
- [P09 — MBRL walking (Su 2023)](P09_su2023_mbrl_walking.md), [P10 — KINESIS motion imitation](P10_simos_kinesis_motion_imitation.md) — related OpenSim+RL / imitation methods.
- [P11 — DRL Transfemoral (De Vree)](P11_devree_drl_transfemoral.md) — the prosthesis target this paper aims toward.

## Caveats (not verified / limits)
- Read 4 of 16 pages: exact **reward formula, PPO hyperparameters, and quantitative
  results** not extracted (only the abstract correlations).
- Figures/Table 1 partially captured; muscle/feet geometry tables not fully read.
- Muscle-excitation action (full body), not a prosthetic trajectory generator —
  relevant for OpenSim modeling + imitation-reward, not the action contract.
