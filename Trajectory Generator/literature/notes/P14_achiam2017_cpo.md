---
id: P14
title: "Constrained Policy Optimization (CPO)"
authors: "Joshua Achiam, David Held, Aviv Tamar, Pieter Abbeel"
year: 2017
venue: "ICML 2017 (PMLR); arXiv:1705.10528"
doi_or_url: "arXiv:1705.10528"
topics: [safe_rl, cmdp, constrained_optimization, trust_region, policy_gradient]
keywords: [CPO, CMDP, trust-region, TRPO, constraint-satisfaction, policy-search, surrogate-bounds]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/3 - Safe RL-vincoli-sicurezza/Constrained Policy Optimization.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\GPT\paper\3 - Safe RL-vincoli-sicurezza\Constrained Policy Optimization.pdf"
pages_read: "1-4 of 18 (abstract/related/CMDP/theory start; full derivation & experiments skimmed)"
extraction_confidence: medium  # concepts clean; heavy theorem math approximate in extraction
related: [P19, P15, P16, P17, P18]
---

# P14 — Constrained Policy Optimization (CPO)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [safe_rl](../TOPICS.md) · [cmdp](../TOPICS.md) · [constrained_optimization](../TOPICS.md) · [trust_region](../TOPICS.md)

## TL;DR
The **first general-purpose policy-search algorithm for constrained RL (CMDP)** with
**near-constraint-satisfaction guarantees at every iteration** of training (not just
at convergence). A trust-region method (TRPO-style) that maximizes reward subject to
keeping auxiliary cost-returns below limits, with a worst-case constraint-violation
bound. Foundational reference for the "safety as a constraint, not a reward term"
approach.

## Problem & contribution
- For systems interacting with/around humans, it's more natural to specify **reward
  + constraints** than to hand-craft behavior through reward shaping (p.1).
- Prior CMDP methods don't scale to high-dim continuous control with neural policies,
  or can't guarantee feasibility *throughout* training (p.1-2).
- Contribution: CPO — first policy search for CMDPs that (1) guarantees constraint
  satisfaction throughout training and (2) works for arbitrary (NN) policy classes;
  plus a new policy-improvement bound of independent interest (p.2).

## Method / architecture
- **CMDP**: MDP augmented with auxiliary cost functions `C_1..C_m` and limits
  `d_1..d_m`; constraint return `J_{C_i}(π) = E[Σ γ^t C_i]`; feasible set
  `Π_C = {π : ∀i, J_{C_i}(π) ≤ d_i}`; objective `π* = argmax_{π∈Π_C} J(π)`
  → PDF p.2, §4.
- **Policy performance bound** (Theorem 1, the theoretical core): bounds the
  difference in (reward or cost) returns between two policies `π', π` by an average
  divergence (TV/KL) between them → PDF p.3, eqs.(1),(4). This tightens TRPO/Kakade-
  Langford bounds and bounds *worst-case constraint violation* per update.
- **CPO update** (heavy/derivation): local trust-region policy search over
  `Π_θ ∩ Π_C` — maximize a surrogate reward subject to surrogate cost constraints
  and a KL trust region `D(π,π_k) ≤ δ` → PDF p.3, eqs.(2)-(3). Solved via a
  Taylor/convex approximation with duality; a **backtracking line search** recovers
  feasibility if a step violates constraints (later pages, not fully extracted).

## Experimental setup
- Simulated robot **locomotion** tasks (MuJoCo-style) with safety-motivated
  constraints (e.g., speed limits, stay out of regions). Neural policies with
  thousands of parameters (p.2). *(Details in §6-7, not extracted.)*

## Key results
- CPO trains high-dim NN policies while **enforcing constraints throughout
  training**, unlike unconstrained policy search (abstract, p.1-2). *(Quantitative
  task results in §7, not extracted.)*

## Code / data availability
- Classic ICML'17 method; reference implementations exist in the safe-RL ecosystem
  (historically OpenAI "safety-starter-agents"/Safety Gym) — **not stated in the
  paper itself**; verify before citing a specific repo.

## Notable claims (page-anchored)
- "it can be more convenient to specify both a **reward function and constraints**,
  rather than trying to design behavior through the reward function" (abstract, p.1)
  — the core motivation shared by [P19](P19_eco_energy_constrained.md).
- CPO guarantees **near-constraint satisfaction at each iteration**, enabling safe
  exploration during training (p.1-2).
- The performance-difference bound (Thm 1) links trust-region theory to practice and
  bounds worst-case degradation/violation per step (p.3).

## Related notes
- [P19 — ECO (Huang et al. 2026)](P19_eco_energy_constrained.md) — uses CMDP framing; compares CPO vs PPO-Lagrangian (and finds CPO scales poorly to multi-constraint humanoids).
- [P15 — Safe RL for Legged Locomotion](P15_yang_safe_rl_legged.md) — applies safe-RL to legged robots.
- [P16 — NN Repair for Assistive Devices](P16_majd_nn_repair_assistive.md) — post-hoc verifiable safety vs in-training constraints.
- [P17 — Safe MBRL with Stability Guarantees](P17_berkenkamp2017_safe_mbrl_lyapunov.md) — Lyapunov-based safety alternative.
- [P18 — Statistical Model Predictive Shielding](P18_smps_shielding.md) — runtime shielding alternative.

## Caveats (not verified / limits)
- Read 4 of 18 pages: the **full CPO update derivation, the convex/dual solution,
  the line-search feasibility recovery, and all experiments** were NOT extracted →
  read PDF §5.2-§7 for the algorithm and results.
- Theorem statements partially garbled in extraction (divergence terms, ε bounds) —
  treat the math here as conceptual; read PDF p.3 for exact forms.
- Domain is generic simulated locomotion, not prosthesis/OpenSim — CPO is the
  *method*, transferable as the constrained-RL backbone.
