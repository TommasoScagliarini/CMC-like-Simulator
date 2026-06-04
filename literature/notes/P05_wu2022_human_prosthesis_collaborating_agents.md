---
id: P05
title: "Human-Robotic Prosthesis as Collaborating Agents for Symmetrical Walking"
authors: "Ruofan Wu, Junmin Zhong, Brent Abraham Wallace, Xiang Gao, He Huang, Jennie Si"
year: 2022
venue: "NeurIPS 2022 (36th Conference on Neural Information Processing Systems)"
doi_or_url: "NeurIPS 2022 proceedings"
topics: [multi_agent_rl, human_in_loop, prosthesis_knee, symmetry, drl_opensim, pomdp]
keywords: [cMARL, human-prosthesis-collaboration, dHDP, actor-critic, MADDPG, COMA, CTDE, shared-cost, OpenSim, intact-knee-target]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/2 - Protesi robotiche di ginocchio-transfemorali/Human-Robotic Prosthesis as Collaborating Agents for Symmetrical Walking.pdf"
pages_read: "1-15 (main text; appendices A-E not extracted)"
extraction_confidence: high  # equations/Table 1 read cleanly; figures not interpreted
related: [P01, P02, P03, P04]
---

# P05 — Human-Robotic Prosthesis as Collaborating Agents (cMARL)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [multi_agent_rl](../TOPICS.md) · [human_in_loop](../TOPICS.md) · [symmetry](../TOPICS.md) · [drl_opensim](../TOPICS.md)

## TL;DR
First RL prosthesis controller that **explicitly models the human as a collaborating
agent**: a **collaborative multi-agent RL (cMARL)** formulation of human-prosthesis
collaboration (HPC), with a **shared (non-factorized) cost** and an *estimated human
control signal* fed to the robot controller. Validated **in OpenSim** (not yet on
humans). Beats single-agent (no-human), MADDPG, and COMA on cost, symmetry,
convergence speed, and success rate.

## Problem & contribution
- Prior prosthesis RL is single-agent and ignores **human influence**; the human
  and robot are *physically coupled* with little reaction time — unlike typical
  pHRI/shared-autonomy where a third object mediates and there is a clear endpoint
  goal (p.1-3).
- There is **no clear endpoint target** in HPC → defining the performance goal is
  itself a challenge; they use **symmetry** + intact-knee matching (p.3).
- Contributions: (1) treat the human user as a collaborating agent; (2) a cMARL
  method that existing MARL (COMA/MADDPG) cannot solve well due to tight coupling;
  (3) first to explicitly include human influence in prosthesis control design (p.3).

## Method / architecture
- **Low level — FS-IC**, 4 phases, torque `T_k = K_k(θ − θe_k) + B_kω` → PDF p.4,
  eq.(1); impedance update `I_{k+1} = I_k + u_k` → eq.(2); 12 params, 4 controllers.
- **Two agents**:
  - Robot state `x_k = [Δτ, Δλ, ΔP, ΔD]ᵀ` (stance-time diff, step-length diff, peak
    knee-angle error, duration error) → PDF p.5, eqs.(3)-(4); robot action
    `u_k = [ΔK, ΔB, Δθe]ᵀ` → eq.(5).
  - Human state `z_k = [Δτ, Δλ]ᵀ`; human "control" `v_k = λ_d − λ_o` (step-length
    deviation from a reference cue) → PDF p.5, eq.(6). The human's first two state
    vars are **shared** with the robot; the robot's kinematic features are **not**
    available to the human (realistic partial observability).
- **Shared stage cost** (key design) → PDF p.5, eq.(7):
  `U = xᵀR_x x + R_v v² + uᵀR_u u + μ h²`, with `h = v − v̄` the human
  estimation-error term (v̄ = actual human step-length deviation). Couples kinematic
  matching + symmetry + control effort + human-estimation error.
- **cMARL solution**: shared infinite-horizon discounted `Q`; Bellman optimality
  → PDF p.6, eq.(8); iterative actor-critic → eq.(9); critic MLP (6 hidden, linear
  out) `Q̂` → eq.(12); actor MLPs (6 hidden, tanh) for robot `u` and human `v`
  → eq.(15); dHDP weight updates eqs.(13)-(17). The Q is learned on the **shared**
  cost, NOT factorized per-agent (their argument vs CTDE methods).

## Experimental setup
- **OpenSim** biomechanical simulation (not human tested). Sensor/actuator **noise
  injected from real human data** (p.7).
- Safety constraints: impedance reset to initial if state exceeds safety bounds
  (App. A.4) (p.7).
- Tasks: level ground 1 m/s (benchmark + ablation); ramp 11.5° and faster 1.12 m/s
  (reliability) (p.7-9).
- Baselines: single-agent (wout/human), COMA, MADDPG (tailored to HPC).

## Key results (Table 1, p.8)
- cMARL (w/human) best on all metrics: stage cost 0.002 vs 0.008 (no-human) /
  0.004 (COMA) / 0.38 (MADDPG); peak error 0.003; symmetry 0.001; converge steps
  **97** vs 187 (no-human) vs 136 (COMA); success rate **0.7** vs 0.58 vs 0.5.
- ≥30% fewer environment samples than baselines (p.8).
- Ablation: removing the human-influence terms (h, v) degrades performance → the
  estimated human control gives a useful predictive signal (p.8-9).
- Reliability: same trends on ramp and faster walking → using the **intact-knee
  trajectory as the robot target** removes a control-design barrier across tasks (p.9).

## Code / data availability
- **No public code** noted; OpenSim simulation study. Appendices A-E (FS-IC details,
  noise extraction, cMARL details, metrics) **not extracted** here.

## Notable claims (page-anchored)
- "robot trajectory tracking using **position control may preclude any dynamic
  interaction** of the robot with its human user" → motivates impedance over
  position control (p.3).
- HPC has **no clear endpoint goal** (unlike upper-limb pHRI) → performance goal
  must be defined indirectly (symmetry + intact-knee matching) (p.3).
- Factorized CTDE methods (MADDPG/COMA) **struggle** because of the intrinsic
  human-robot coupling → a *shared* cost works better (p.10).
- Convergence/optimality/closed-loop stability are "still needed" but "very likely
  provable" (p.10) — i.e. not yet guaranteed.

## Related notes
- [P01 — RL Impedance Echo Control (Wu et al. 2022)](P01_wu2022_rl_impedance_echo_control.md) — the single-agent echo-control predecessor (ref [17]); P05 adds the human agent.
- [P02 — Online RL personalization (Wen et al. 2020)](P02_wen2020_online_rl_personalization.md) — dHDP lineage used here.
- [P03 — Wearer-Prosthesis Interaction for Symmetrical Gait (Wen et al. 2020)](P03_wen2020_wearer_prosthesis_symmetry.md) — symmetry metrics motivate the shared goal.
- [P04 — Hierarchical Optimization (Li et al. 2023)](P04_li2023_hierarchical_knee_symmetry.md) — alternative way (BO outer loop) to optimize symmetry in the same human-prosthesis system.

## Caveats (not verified / limits)
- **Simulation only (OpenSim)**; not human-tested (authors flag this, p.10).
- Figures (Figs.1-5 learning curves) **not interpreted**; Table 1 numbers read from
  text. Appendices not read.
- The "human agent" is an *estimated* control signal informing the robot, not a
  literal model of human neurocontrol (p.5).
- Same per-phase, gait-cycle-rate dHDP impedance tuner family as P01-P04.
