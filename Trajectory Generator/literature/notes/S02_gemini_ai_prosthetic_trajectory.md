---
id: S02
title: "Exploratory Literature Review and Feasibility Study: Artificial Intelligence Trajectory Generation for Transfemoral Robotic Prostheses (Gemini deep-research)"
authors: "(Gemini deep-research synthesis)"
year: 2026
venue: "Internal deep-research report (Gemini)"
doi_or_url: "@/Gemini/AI for Prosthetic Trajectory Generation.pdf"
topics: [feasibility_review, hierarchical_control, action_space, reward_design, pomdp, safe_rl, software_integration]
keywords: [synthesis, transfemoral, SEA, hierarchical-control, spline-action-space, position-vs-torque, imitation-warmstart, recurrent-memory, LibTorch, ONNX]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/AI for Prosthetic Trajectory Generation.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\AI for Prosthetic Trajectory Generation.pdf"
pages_read: "1-4 of 22 (intro/architecture/DRL/hierarchical/action-space; reward/POMDP/software/safety sections skimmed)"
extraction_confidence: medium  # synthesis; only first ~4 pages read in detail; mirrors S01
related: [S01, P11, P13, P22, P19, P16]
---

# S02 — Gemini feasibility synthesis (companion to S01)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [feasibility_review](../TOPICS.md) · [hierarchical_control](../TOPICS.md) · [action_space](../TOPICS.md) · [reward_design](../TOPICS.md)

## TL;DR
The **Gemini** deep-research feasibility report — the twin of the GPT one
(`GPT/deep-research-report.md`, summarized elsewhere). Same project: replace the
experimental kinematic reference with an **AI trajectory generator** for a
**transfemoral SEA prosthesis** in a **C++/OpenSim** simulator, under a strict
hierarchy (AI high-level → outer position/impedance loop → inner SEA driver).
Notably it **independently flags issues the project later hit** (SEA oscillations from
erratic references; position-RL ≫ torque-RL; spline action spaces for C¹/C² smoothness).

## System architecture it assumes (matches the project exactly)
- Hybrid asymmetric model: healthy contralateral limb = biological muscles tracking
  MoCap; amputated side = **knee + ankle driven by Series Elastic Actuators (SEAs)**
  (p.1).
- **SEAs**: spring in series with motor → shock tolerance, energy storage, compliance
  (mimics tendons), **but "highly non-linear and susceptible to oscillations if
  subjected to erratic control signals"** (p.1). *(This is exactly the ~35 Hz knee
  limit-cycle the project diagnosed.)*
- Hierarchy: **High-level AI = dynamic trajectory planner** (kinematic references
  instant-by-instant from local state) → **Outer loop** (position/impedance control,
  computes torque) → **Inner loop** (high-freq motor driver / SEA) (p.1-2).

## Key positions (from the read pages)
- **DRL in OpenSim**: PPO/DDPG on healthy + transfemoral models; **imitation learning
  / behavioral cloning warm-start** to guarantee a plausible initial gait and cut
  sample cost (p.2).
- **End-to-end torque/excitation control is "systematically problematic"**: sample-
  inefficient, sim-to-real-fragile, **jerky high-frequency oscillations** that are
  inefficient and dangerous → drives the field to **hierarchical** abstractions (p.2-3).
- **Position-based RL converges faster and to higher reward than torque-based**; the
  PD/impedance outer loop regularizes the action space and absorbs disturbances
  between the AI's lower-frequency decisions (p.3). *(Matches [P13].)*
- **Action-space parameterization table** (p.3-4): 
  - *Direct joint position targets* — simplest, aligns with PD; but divergent
    successive commands → **step-functions / torque spikes**.
  - *Task-space waypoints* — intuitive for foot placement; needs real-time IK.
  - **Spline coefficients (Bézier / B-spline)** — outer loop evaluates the curve;
    **guarantees C¹/C² continuity, eliminates step-discontinuities → smooth velocity/
    acceleration**. *(Directly the project's segment-spline + 6 Hz reference-filter
    design rationale.)*
- (Beyond p.4, per the TOC/abstract, it also covers multi-objective reward design,
  **recurrent memory for localized/POMDP state**, and **Python↔C++ software** bridging
  — overlapping with S01; not fully extracted here.)

## Code / data availability
- N/A (internal synthesis report).

## Notable claims (page-anchored)
- SEA dynamics "susceptible to oscillations if subjected to erratic control signals"
  (p.1) — independent prediction of the project's knee instability.
- Spline/Bézier action spaces give **mathematically smooth** references and remove
  step discontinuities (p.3-4) — the strongest literature support for the project's
  smooth-reference + filter approach.
- Hierarchical separation (AI plans kinematics; classical loops realize) is the
  recommended antidote to end-to-end torque instability (p.2-3).

## Related notes
- **S01** (GPT synthesis, `GPT/deep-research-report.md`) — the companion report (fully
  read); same conclusions (memory, action parametrization, safety-as-constraint,
  hierarchy).
- [P13 — Learning Torque Control](P13_learning_torque_control_quadruped.md) — position-vs-torque evidence.
- [P22 — LSWM](P22_lswm_world_model_bipedal.md) — recurrent/POMDP memory.
- [P19 — ECO](P19_eco_energy_constrained.md), [P16 — NN Repair](P16_majd_nn_repair_assistive.md) — reward-constraint / safety routes.
- [P11 — DRL Transfemoral](P11_devree_drl_transfemoral.md) — OpenSim transfemoral DRL precedent.

## Caveats (not verified / limits)
- Only **4 of 22 pages** read in detail (intro/architecture/DRL/hierarchical/action
  space). The **reward-design, POMDP/memory, software, and safety** sections were not
  fully extracted → read the PDF for those (they largely mirror S01).
- It's a synthesis (secondary source), like S01 — useful for *design rationale + which
  primary papers to read*, not as primary evidence. Citations are to the primary
  papers (P01-P25).
