---
id: P18
title: "Safe Reinforcement Learning via Statistical Model Predictive Shielding (SMPS)"
authors: "Osbert Bastani, Shuo Li, Anton Xu (University of Pennsylvania)"
year: 2021
venue: "Robotics: Science and Systems (RSS) 2021"
doi_or_url: "RSS 2021"
topics: [safe_rl, shielding, backup_policy, runtime_verification, lyapunov]
keywords: [shielding, model-predictive-shielding, backup-policy, recoverable-set, LQR, statistical-verification, invariant-set, simplex-architecture]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/3 - Safe RL, Shielding e Garanzie di Stabilità/Statistical Model Predictive Shielding (SMPS).pdf"
pages_read: "1-3 of 13 (abstract/intro/background/problem; algorithm & experiments skimmed)"
extraction_confidence: high  # clean text; figure not interpreted
related: [P17, P14, P19, P16]
---

# P18 — Statistical Model Predictive Shielding (SMPS)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [safe_rl](../TOPICS.md) · [shielding / backup_policy](../TOPICS.md) · [runtime_verification](../TOPICS.md)

## TL;DR
A **runtime safety shield**: run the learned policy π̂ only where a **backup policy**
can provably keep the system safe; otherwise switch to the backup. SMPS decides this
**on-the-fly** by simulating the backup for N steps to check **recoverability**
(reach a safe equilibrium set while staying in the safe set), using **statistical
verification** (sampling + concentration inequalities) + robust control for the LQR.
Guarantees ϵ-safety with high probability; >100× faster than prior robust MPS.

## Problem & contribution
- DRL policies (DNNs) are powerful but unsafe (robot may fall, car may crash);
  ahead-of-time verification scales exponentially with state dimension (p.1).
- Compose π̂ with a backup policy πbackup; use π̂ inside the region where πbackup is
  safe, switch near the boundary → only πbackup must be verified (p.1).
- Contribution: SMPS checks the backup's safety **on-the-fly** via (i) robust control
  + Taylor bounds for the LQR invariant set, (ii) **statistical verification** for
  the recovery policy; proven ϵ-safe w.h.p. under Lipschitz dynamics + bounded
  disturbance; >100× faster than RMPS (p.1-2).

## Method / architecture
- **Shield policy**: `π_shield(x) = π̂(x) if f(x,π̂(x)) ∈ X_inv else π_backup`
  → PDF p.2. If `X_inv` is invariant for πbackup, it's invariant for π_shield.
- **Backup** = LQR (`π_LQR`, keeps system near a safe equilibrium set `X_eq`) +
  recovery policy `π_rec` (drives the system to `X_inv`) (p.1-2).
- **Recoverable set** (Def II.1): state x is recoverable if simulating πbackup for N
  steps reaches `X_eq` while staying in `X_safe`. **Theorem II.2**: the recoverable
  set `X_rec` is invariant for π_shield (p.2-3). MPS checks `x ∈ X_rec` on-the-fly
  instead of precomputing the invariant set.
- **ϵ-safety** (Def III.1): `P(trajectory ⊆ X_safe) ≥ 1−ϵ`; the sampling-based check
  holds with prob ≥1−δ (p.3). Assumes Lipschitz `f, ∇f` and bounded disturbance
  (Assumption III.2).
- Invariant sets relate to **Lyapunov functions / control barrier functions** (p.2).

## Experimental setup
- Benchmarks include cart-pole, autonomous car (obstacle avoidance), walking robot
  (don't fall). Nominal dynamics model + stochastic disturbance (p.1-2). *(Algorithm
  1 and quantitative results in §IV-VI, not extracted.)*

## Key results
- ϵ-safety w.h.p.; **>100× faster than RMPS** at the same safety confidence (p.2).
  *(Per-benchmark numbers not extracted.)*

## Code / data availability
- UPenn (Bastani group); no repo noted in read pages.

## Notable claims (page-anchored)
- Only the **backup policy** needs verification — it can be simple since it only must
  be *safe*, not performant (p.1).
- Checking recoverability **on-the-fly** makes time complexity ~independent of state/
  action dimension (except through π̂, πbackup) (p.1).
- Three flavors of safe RL: heuristic / prove-before-deploy (their focus) / safe-
  during-learning (reducible to prove-before-deploy via a GP dynamics model) (p.2).

## Related notes
- [P17 — Safe MBRL with Stability Guarantees (Berkenkamp)](P17_berkenkamp2017_safe_mbrl_lyapunov.md) — closely related: safe set / region of attraction / recovery; Lyapunov + GP uncertainty.
- [P14 — Constrained Policy Optimization (Achiam et al. 2017)](P14_achiam2017_cpo.md) — alternative: in-training CMDP constraints vs runtime shielding here.
- [P19 — ECO (Huang et al. 2026)](P19_eco_energy_constrained.md) — constrained-RL (Lagrangian) safety vs shielding.
- [P16 — NN Repair for Assistive Devices](P16_majd_nn_repair_assistive.md) — another post-hoc safety mechanism (repair the network vs shield at runtime).

## Caveats (not verified / limits)
- Read 3 of 13 pages: **Algorithm 1, the LQR/statistical-verification details (§IV-V),
  and experiments (§VI)** not extracted.
- Figure 1 (SMPS overview / recoverable vs irrecoverable) **not interpreted**.
- Needs a **nominal dynamics model** + Lipschitz/bounded-disturbance assumptions →
  for the project, the "shield/backup on the generated reference" idea is the
  takeaway (correct a reference that would leave the safe/recoverable set), not the
  exact algorithm.
