---
id: P15
title: "Safe Reinforcement Learning for Legged Locomotion"
authors: "Tsung-Yen Yang, Tingnan Zhang, Linda Luu, Sehoon Ha, Jie Tan, Wenhao Yu (Princeton / Google Research / Georgia Tech)"
year: 2022
venue: "IEEE/RSJ IROS 2022"
doi_or_url: "10.1109/IROS47612.2022.9982038"
topics: [safe_rl, recovery_policy, cmdp, shielding, legged_locomotion]
keywords: [safe-recovery-policy, learner-policy, safety-trigger-set, reachability, approximate-dynamics, centroidal-model, CMDP, safe-during-training]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/3 - Safe RL-vincoli-sicurezza/Safe Reinforcement Learning for Legged Locomotion.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\GPT\paper\3 - Safe RL-vincoli-sicurezza\Safe Reinforcement Learning for Legged Locomotion.pdf"
pages_read: "1-3 of 8 (abstract/intro/background/setup; algorithm & experiments skimmed)"
extraction_confidence: high  # clean text; figures not interpreted
related: [P18, P17, P14, P12]
---

# P15 — Safe RL for Legged Locomotion (recovery policy + reachability switch)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [safe_rl](../TOPICS.md) · [recovery_policy](../TOPICS.md) · [cmdp](../TOPICS.md) · [shielding](../TOPICS.md)

## TL;DR
Safe RL that **switches between a learner policy (task) and a safe recovery policy**
to keep a quadruped safe **during training** (no manual resets from falls). When the
learner enters a **safety trigger set**, the recovery policy takes over; control is
handed back only when an **approximate dynamics model** rollout predicts future states
stay safe (reachability criterion). 48.6% fewer falls in sim; near-zero falls on
hardware.

## Problem & contribution
- RL exploration visits unsafe states; quadrupeds are unstable; learning/fine-tuning
  in the real world needs safety during training (p.1).
- Contribution: a **two-policy switching** safe-RL framework using **model-based**
  (not learned-critic) switching via an approximate dynamics model; theoretical
  **regret bound** under dynamics error; strong empirical safety (p.1-2).

## Method / architecture
- **CMDP** `<S, A, T, R, C>`; cost C = failure (e.g., fall); maximize reward s.t.
  `J_C(π) ≤ h` **throughout training** (p.2).
- **Two policies**: `π_learner` (task) + `π_safe` (recovery). Sets (Def.1, p.3):
  - **Safety trigger set** `C_tri` — near-violation states still saveable by π_safe;
  - **Safe set** `C_safe`; **Failure set** `C_failure` (e.g., robot height too low).
- **Switch logic**: enter `C_tri` → switch to `π_safe`; **hand back** to learner only
  if an **approximate dynamics model** (e.g., **centroidal model**) rollout predicts
  all future states safe (**reachability criterion**) → reduces frequent switching and
  keeps the learner exploring near the boundary (p.1-2, Fig.2).
- Assumes approximate dynamics `T̂` with bounded error `‖T − T̂‖ ≤ ε` (Assumption 1,
  p.3). Contrast with **Recovery RL** (learned safety critic) — they argue learned
  critics mis-estimate real safety and don't generalize to unseen states (p.2).

## Experimental setup
- Four quadruped tasks (sim + real): efficient gait, catwalk, two-leg balance, pacing
  (p.1-2). Compared to SOTA safe-RL baselines.

## Key results
- **48.6% fewer falls** in simulation; **near-zero falls** real-world; **<5 falls in
  115 min** hardware (abstract, p.1).
- Real-world gains: **34% energy efficiency** (efficient gait), 40.9% narrower feet
  (catwalk), 2× jump duration (two-leg balance) (abstract).
- Fewer recovery-policy invocations than prior work → faster learning (p.2).

## Code / data availability
- Project site: sites.google.com/view/saferlleggedlocomotion ; Google Research. No
  code repo noted in read pages.

## Notable claims (page-anchored)
- Model-free safe-RL (cost-in-reward, action modification, learned safety critics)
  **still violates constraints during early random exploration** (p.2).
- Exploiting **known approximate dynamics** for safe planning beats black-box safety
  critics for verification/generalization (p.2).
- Safety must hold for the **entire training process**, not just at convergence (p.2).

## Related notes
- [P18 — Statistical Model Predictive Shielding](P18_smps_shielding.md) — same family (backup policy + on-the-fly recoverability check); SMPS uses statistical verification, this uses an approximate dynamics rollout.
- [P17 — Safe MBRL with Stability Guarantees](P17_berkenkamp2017_safe_mbrl_lyapunov.md) — safe set / region of attraction with GP uncertainty.
- [P14 — Constrained Policy Optimization](P14_achiam2017_cpo.md) — the CMDP backbone (in-training constraints).
- [P12 — GLiDE](P12_glide_quadrupedal_centroidal.md) — the centroidal dynamics model used here for the safety reachability check.

## Caveats (not verified / limits)
- Read 3 of 8 pages: **the switching algorithm details (§IV), regret-bound theorem,
  and quantitative tables** not extracted.
- Figure 2 (safe/trigger/failure sets) **not interpreted**.
- Quadruped, not OpenSim/prosthesis — transferable idea: a **recovery/backup
  reference + reachability switch** on the generated reference (fall = pelvis height),
  using an approximate model for the safety check.
