---
id: P19
title: "ECO: Energy-Constrained Optimization with Reinforcement Learning for Humanoid Walking"
authors: "Weidong Huang, Jingwen Zhang, Jiongye Li, Shibowen Zhang, Jiayang Wu, Jiayi Wang, Hangxin Liu, Yaodong Yang, Yao Su (BIGAI)"
year: 2026
venue: "IEEE Transactions on Automation Science and Engineering (preprint, accepted Feb 2026); arXiv:2602.06445"
doi_or_url: "arXiv:2602.06445 ; project: https://sites.google.com/view/eco-humanoid"
topics: [reward_design, cost_of_transport, constrained_optimization, lagrangian, safe_rl, action_space, locomotion_rl, symmetry, sim_to_real]
keywords: [CMDP, PPO-Lagrangian, energy-constraint, mirror-loss, IsaacGym, BRUCE-humanoid, constrained-RL]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/4 - Progettazione della Reward Function e Costo del Trasporto (COT)/ECO - Energy-Constrained Optimization with Reinforcement Learning for Humanoid Walking.pdf"
pages_read: "1-5 of 16 (intro/related/preliminaries/method start; results skimmed)"
extraction_confidence: high  # prose clean; some math glyphs (`pS,Aq`) artifact-y but readable; figures not interpreted
related: [P14, P17, P18, P20, P21, P13]
---

# P19 — ECO: Energy as a *constraint* (not a reward term) for humanoid walking

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [reward_design](../TOPICS.md) · [cost_of_transport](../TOPICS.md) · [constrained_optimization](../TOPICS.md) · [safe_rl](../TOPICS.md)

## TL;DR
Instead of folding energy into a weighted reward (which needs painful tuning and
trades off against stability), **ECO reformulates energy as an explicit inequality
constraint** in a CMDP, enforced via the **Lagrangian (PPO-Lagrangian)**. Reward
keeps task terms (velocity tracking, stability); energy + reference-motion become
constraints with physically meaningful thresholds. Sim-to-real on the BRUCE kid
humanoid: ~6× lower energy than MPC, 2.3× lower than PPO.

## Problem & contribution
- RL energy terms in the reward (joint torque/accel/contact penalties) require
  extensive, non-intuitive weight tuning; conflicting objectives → suboptimal or
  unstable gaits (energy-vs-stability) (p.1-2).
- Contribution: (1) treat energy cost as a **constraint** with clear physical
  meaning, tuned by a simple linear search on the threshold; (2) benchmark 4
  constrained-RL methods and pick **PPO-Lag**; (3) first energy-efficient humanoid
  walking via constrained RL on real hardware (p.2).

## Method / architecture
- **CMDP** = MDP + cost functions `C_i ≤ b_i` → PDF p.3, eq.(1)-(4). Two constraint
  forms: **discounted-sum** `E[Σ γ^t C_i] ≤ b_i` → eq.(5) (good for cumulative
  energy); **average-sum** `E[(1/T)Σ C_i] ≤ b_i` → eq.(6) (good for style/symmetry).
- **Constrained-RL methods reviewed** (useful menu) → PDF p.3-4:
  - **PPO-Lagrangian**: `L = J_R − Σ λ_i (J_{C_i} − b_i)`, primal-dual → eq.(7)-(8). *(chosen)*
  - **CRPO** (alternate reward max / cost min, pick a violated constraint) → eq.(9).
  - **IPO** (log-barrier) → eq.(10); **P3O** (clamped penalty) → eq.(11)-(12).
  - Also cites **CPO** ([P14]) as second-order but poor multi-constraint scalability.
- **Two constraints chosen**: energy minimization + reference-motion (Mirror Loss)
  for natural/symmetric gait. More constraints hurt convergence for humanoids
  (smaller feasible domain than quadrupeds) (p.4).
- **Policy I/O**: observation history of K_f frames (proprioception: clock sin/cos,
  joint pos/vel, body angular vel, last action, body Euler XY; + velocity commands).
  Action `a ∈ R^n` = **deviation from nominal joint positions**, fed to a PD
  controller `τ̂ = K_p(a + q_nominal − q) + K_d(0 − q̇)` @1kHz; policy @100 Hz
  → PDF p.4-5, eq.(13). *(Same "policy emits position setpoint → low-level PD"
  pattern relevant to the project.)*
- **Asymmetric actor-critic**: reward critic uses **privileged** observations (body
  lin. vel., push forces, friction, mass, phase, contacts), trained in IsaacGym;
  policy deployed directly (p.4-5).

## Experimental setup
- IsaacGym training; sim-to-sim + sim-to-real on **BRUCE** kid-sized humanoid (p.1).
- Baselines: MPC, standard PPO (reward shaping), and 4 constrained-RL methods
  (PPO-Lag, CRPO, IPO, P3O) (p.2).

## Key results
- ECO ≈ **6× lower energy than MPC**, **2.3× lower than PPO**, while keeping robust
  walking (p.2, abstract).
- Emergent behaviors: more **extended knees**, **lighter steps**, **reduced body
  shaking** (p.2, Fig.1) — beneficial for loco-manipulation.
- PPO-Lag gave lowest energy with fast/stable convergence among constrained methods
  (p.2). Adding too many constraints destabilizes humanoid training (p.4).

## Code / data availability
- Project website with demos: https://sites.google.com/view/eco-humanoid . Trained
  in **IsaacGym**; hardware = BRUCE. No explicit code repo noted in the read pages.

## Notable claims (page-anchored)
- "adjusting the reward weights alone for PPO **does not lead to convergence within
  the feasible domain**" for energy → motivates the constraint formulation (p.4).
- Energy-as-constraint gives a "**clear, interpretable physical meaning**", enabling
  intuitive threshold tuning by linear search (p.2).
- Constraint *form matters*: discounted-sum for cumulative energy, average-sum for
  style/symmetry (p.3, eqs.5-6).
- Humanoids have a **smaller feasible domain** than quadrupeds → fewer constraints
  converge better (p.4).

## Related notes
- [P14 — Constrained Policy Optimization (Achiam et al. 2017)](P14_achiam2017_cpo.md) — the CMDP/constrained-RL foundation ECO builds on and compares to.
- [P17 — Safe MBRL with Stability Guarantees (Berkenkamp et al.)](P17_berkenkamp2017_safe_mbrl_lyapunov.md) — another safe-RL route (Lyapunov vs Lagrangian constraints).
- [P18 — Statistical Model Predictive Shielding](P18_smps_shielding.md) — alternative safety mechanism (shielding vs constraint).
- [P20 — Powered Ankle-Foot Prosthesis Metabolic Economy (Au et al.)](P20_au2009_powered_ankle_foot.md) — physical energy/metabolic grounding.
- [P21 — Predicting Metabolic Cost (Silder et al. 2012)](P21_silder2012_metabolic_cost_incline.md) — surrogate energy from mechanics.
- [P13 — Learning Torque Control for Quadrupedal Locomotion](P13_learning_torque_control_quadruped.md) — action-space (torque vs position) comparison; ECO uses position-deviation + PD.

## Caveats (not verified / limits)
- Read pages 1-5 of 16: the **exact ECO objective (eq.20)**, full constraint
  definitions (energy motor model, eqs after p.5), and quantitative result tables
  were NOT extracted → read PDF §IV-VI for specifics.
- Some math glyphs garbled by extraction (e.g., `pS, A, ...q` for `(S, A, ...)`).
- Figures (Fig.1 comparison, Fig.2 pipeline) **not interpreted**.
- Domain is a **humanoid robot in IsaacGym**, not a prosthesis/OpenSim — transfer of
  the *constraint formulation* is the relevant takeaway, not the exact setup.
