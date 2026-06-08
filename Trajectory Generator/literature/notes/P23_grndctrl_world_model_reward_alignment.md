---
id: P23
title: "GrndCtrl: Grounding World Models via Self-Supervised Reward Alignment"
authors: "Haoyang He, Jay Patrikar, Dong-Ki Kim, Max Smith, Daniel McGann, Ali-akbar Agha-mohammadi, Shayegan Omidshafiei, Sebastian Scherer (CMU / FieldAI)"
year: 2026
venue: "arXiv:2512.01952 (v2, Feb 2026)"
doi_or_url: "arXiv:2512.01952 ; project: rlwg-grndctrl.github.io"
topics: [rlvr, world_model, verifiable, robotics]
keywords: [RLWG, RLVR, GRPO, verifiable-rewards, geometric-grounding, video-world-model, pose-cycle-consistency, self-supervised-post-training]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/6 - Reinforcement Learning from Verifiable Results (RLVR) in Robotica/Grounding World Models via Self-Supervised Reward Alignment.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\paper\6 - Reinforcement Learning from Verifiable Results (RLVR) in Robotica\Grounding World Models via Self-Supervised Reward Alignment.pdf"
pages_read: "1-3 of 15 (abstract/intro/related; method/results skimmed)"
extraction_confidence: medium  # clean text; domain (video world models) far from project; figures not interpreted
related: [P24, P22, P19]
---

# P23 — GrndCtrl / RLWG: verifiable-reward grounding of world models

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [rlvr](../TOPICS.md) · [world_model](../TOPICS.md) · [verifiable](../TOPICS.md)

## TL;DR
Extends **RLVR (verifiable-reward RL)** from language to **video world models**:
post-train a pretrained world model with **automatically-verifiable geometric/temporal
rewards** (pose cycle-consistency, depth reprojection, temporal coherence) instead of
reconstruction loss, using **GRPO**. Far from the prosthesis domain — relevant only as
a concrete instance of the **"verifiable rewards"** idea (and GRPO as an algorithm).

## Problem & contribution
- Video world models look realistic but **lack geometric grounding** (pose drift,
  depth wobble) → unusable for closed-loop navigation (p.1-2).
- Contribution: **RLWG** — self-supervised post-training aligning world models to
  physically verifiable spatial/temporal invariants via rewards from **frozen
  evaluators** (no labels, no external simulator); **GrndCtrl** instantiates it with
  GRPO over stochastic rollouts (p.1-2).

## Method / architecture
- **RLWG**: treat the pretrained world model as a **policy** generating multiple
  candidate rollouts from the same context; score each with **verifiable grounding
  rewards** (pose cycle-consistency, depth reprojection agreement, action adherence)
  that measure *physical correctness*, not pixel error (p.1-2).
- **GrndCtrl = GRPO**: generate a group of rollouts, **rank by grounding reward**,
  compute relative advantages within the group, update the latent transition operator
  with a **clipped policy gradient regularized toward the pretrained model** (p.2).
  Rewards: Translation, Rotation, Depth Temporal Reprojection Inlier ratio, perceptual
  quality (p.2, contribution 2).
- Explicit analogy: "**where RLVR grounds language in logic, RLWG grounds world models
  in geometry**" (p.2).

## Experimental setup / results
- Evaluated on multiple navigation/outdoor datasets; reduced pose-error mean/variance;
  gains under counterfactual rollouts; generalization to unseen inputs (p.2,
  contribution 3). *(Quantitative results not extracted.)*

## Code / data availability
- Project page: rlwg-grndctrl.github.io (CMU / FieldAI).

## Notable claims (page-anchored)
- **Verifiable rewards** measure physical correctness (geometry/time), unlike
  reconstruction/pixel losses (p.1). *(The transferable idea for the project: reward
  terms that are automatically + objectively checkable — e.g., no-fall, joint limits,
  contact correctness — i.e. "RL from verifiable simulator results".)*
- **GRPO** (group-relative, ranking rollouts) is the optimization mechanism — an
  alternative to PPO for verifiable-reward settings (p.2).
- RLVR (LLM) → RLWG (world models): the "verifiable feedback" paradigm transfers
  across domains by swapping the verifier (logic → geometry) (p.1-2).

## Related notes
- [P24 — RLVR for Reasoning LMs](P24_rlvr_reasoning_lm.md) — the origin of the verifiable-reward paradigm this generalizes.
- [P22 — LSWM](P22_lswm_world_model_bipedal.md) — world model for locomotion (different: state reconstruction/prediction for control, not video generation).
- [P19 — ECO](P19_eco_energy_constrained.md) — verifiable physical *constraints* (energy) vs verifiable *rewards* here.

## Caveats (not verified / limits)
- Read 3 of 15 pages: GRPO details, reward formulas, datasets, results **not
  extracted**.
- **Domain mismatch**: video world models for navigation, not prosthesis/OpenSim
  control. Per the S01 synthesis, RLVR is not standard in prosthesis/locomotion; the
  only transferable takeaways are the **verifiable-reward principle** and **GRPO**.
- Figures **not interpreted**.
