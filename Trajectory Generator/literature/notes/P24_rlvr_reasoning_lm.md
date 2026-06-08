---
id: P24
title: "100 Days After DeepSeek-R1: A Survey on Replication Studies and More Directions for Reasoning Language Models (RL from Verifiable Rewards)"
authors: "Chong Zhang, Yue Deng, Xiang Lin, Bin Wang, et al. (MiroMind / Fudan / NUS / SUTD / NTU)"
year: 2025
venue: "arXiv:2505.00551 (v3, May 2025)"
doi_or_url: "arXiv:2505.00551"
topics: [rlvr, llm, verifiable_rewards]
keywords: [RLVR, DeepSeek-R1, reasoning-language-models, SFT, GRPO, reward-modeling, KL-loss, survey]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/6 - Reinforcement Learning from Verifiable Results (RLVR) in Robotica/Reinforcement Learning from Verifiable Rewards for Reasoning Language Models.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\paper\6 - Reinforcement Learning from Verifiable Results (RLVR) in Robotica\Reinforcement Learning from Verifiable Rewards for Reasoning Language Models.pdf"
pages_read: "1-3 of 36 (abstract + table of contents only)"
extraction_confidence: low  # only abstract/TOC read; LLM-reasoning domain, peripheral to the project
related: [P23]
---

# P24 — RLVR for Reasoning Language Models (survey)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [rlvr](../TOPICS.md) · [llm](../TOPICS.md) · [verifiable_rewards](../TOPICS.md)

## TL;DR
A **survey** of post-DeepSeek-R1 replication work on **Supervised Fine-Tuning (SFT)**
and **Reinforcement Learning from Verifiable Rewards (RLVR)** for **reasoning language
models**. This is the *origins/landscape* reference for the RLVR paradigm. **Domain is
LLMs, not prosthesis/robotics** — included only because the corpus has an "RLVR" theme;
the transferable concept is the **verifiable-reward principle** + RL algorithms (GRPO).

## Problem & contribution
- DeepSeek-R1 sparked interest in explicit-reasoning LLMs but didn't fully open-source
  details; many replication studies emerged (p.1).
- Contribution: survey SFT + RLVR (data construction, method design, training),
  distilling key findings to guide future RLM research (abstract).

## Method / architecture (survey scope, from TOC)
- **SFT** (§2): datasets (OpenThoughts-114k, OpenR1-Math-220k, LIMO, s1K, …),
  curation, training/performance.
- **RLVR** (§3): RL datasets; components = **Algorithms** (GRPO and PPO variants),
  **Rewards** (verifiable correctness signals), **Sampling strategies**; analysis of
  data recipes, RL algorithm design, model size, context length, **reward modeling**,
  **KL loss** (p.2 TOC).
- **More directions** (§4): reasoning enhancement, generalizability, **safety**,
  multimodal/multilingual.

## Key results
- Not extracted (survey; only abstract + TOC read). High level: SFT + RLVR with
  open data can approach DeepSeek-R1-level reasoning (abstract).

## Code / data availability
- Survey; references many open datasets/models (OpenThoughts, OpenR1, Light-R1,
  Bespoke-Stratos, SYNTHETIC-1, s1.1, LIMO).

## Notable claims (from abstract/TOC)
- **RLVR** = RL where rewards come from **automatically verifiable correctness** (e.g.,
  a math answer is right/wrong) rather than a learned/human reward model — the core
  idea the project's "RL from verifiable simulator results" TODO alludes to.
- **GRPO** features prominently as the RLVR optimization algorithm; **KL loss** and
  **reward modeling** are recurring design axes (TOC §3).

## Related notes
- [P23 — GrndCtrl / RLWG](P23_grndctrl_world_model_reward_alignment.md) — extends this RLVR paradigm from language to (video) world models.

## Caveats (not verified / limits)
- **Only abstract + TOC read** (3/36 pages) → low confidence; the algorithm/reward
  details (§3.2) were not extracted.
- **Domain mismatch**: LLM reasoning, not robotics/prosthesis/OpenSim. Per the S01
  synthesis, RLVR is *not* a prosthesis-control method; treat this purely as
  background on "verifiable rewards" and the GRPO algorithm family.
