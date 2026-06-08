---
id: P17
title: "Safe Model-Based Reinforcement Learning with Stability Guarantees (within: 'Safe Exploration in RL: Theory and Applications in Robotics', PhD thesis)"
authors: "Felix Berkenkamp (thesis; advisors A. Krause, A. Schoellig). NeurIPS'17 paper co-authors: Matteo Turchetta, Angela Schoellig, Andreas Krause"
year: 2019
venue: "PhD thesis, ETH Zurich, 2019 (contains the NeurIPS 2017 paper 'Safe Model-Based RL with Stability Guarantees')"
doi_or_url: "ETH thesis; NeurIPS 2017 paper arXiv:1705.08551"
topics: [safe_rl, lyapunov, stability_guarantees, model_based_rl, bayesian_optimization, region_of_attraction]
keywords: [Lyapunov-function, region-of-attraction, safe-set, GP-dynamics, SafeOpt, safe-MPC, backup-policy, epistemic-uncertainty]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/3 - Safe RL, Shielding e Garanzie di Stabilità/Safe Model-Based Reinforcement Learning with Stability Guarantees.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\paper\3 - Safe RL, Shielding e Garanzie di Stabilità\Safe Model-Based Reinforcement Learning with Stability Guarantees.pdf"
pages_read: "1-10 of 205 (front matter: abstract + table of contents only)"
extraction_confidence: low  # only abstract/TOC read; this is the 205-page thesis, not just the NeurIPS paper — content below is conceptual
related: [P14, P18, P19, P15]
---

# P17 — Safe Model-Based RL with Stability Guarantees (Lyapunov / region of attraction)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [safe_rl](../TOPICS.md) · [lyapunov / stability_guarantees](../TOPICS.md) · [model_based_rl](../TOPICS.md) · [bayesian_optimization](../TOPICS.md)

> **Heads-up on the file:** the PDF is the **205-page ETH PhD thesis** "Safe
> Exploration in RL: Theory and Applications in Robotics" (Berkenkamp, 2019), which
> *contains* the well-known NeurIPS 2017 paper of the INDEX title as one chapter.
> Only the front matter (abstract + TOC) was read here; the note is **conceptual**.

## TL;DR
Safe-exploration RL that gives **rigorous safety guarantees during learning** by
explicitly modeling **uncertainty** about the environment. The flagship idea
(NeurIPS'17): use a **Lyapunov function** + a (Gaussian-process) model of the
dynamics with confidence intervals to certify a **region of attraction / safe set**,
and **only explore states from which a safe backup policy can provably return** to
the safe set. Conservative when uncertainty is large; more confident as data
accrues; **safe at all times**.

## Problem & contribution
- RL finds good controllers but usually gives **no safety guarantees during
  learning** → unusable in safety-critical systems (abstract, p.3).
- Contributions (from abstract/TOC):
  1. **Direct policy optimization** — extend **Bayesian Optimization** (SafeOpt-style)
     to **multiple safety constraints**; safe **knowledge transfer across contexts**;
     validated on a flying robot (p.3).
  2. **Model-based RL** — prove you can learn safely by **restricting exploration to
     states where a safe backup strategy exists** (Lyapunov region of attraction);
     a **safe MPC** exploration algorithm that only collects data where it can always
     recover to the safe region (p.3-4).
  3. **Regret bounds** for an optimistic exploration scheme, and how to combine it
     with the safety-constrained setting (p.4).

## Method / architecture (conceptual)
- **Safe set via Lyapunov stability**: a Lyapunov function `V` whose decrease
  condition, certified under the GP model's confidence bounds, defines a region of
  attraction that is provably safe. Exploration is allowed only where the certified
  decrease holds (TOC §2.4 "Safe Control: Lyapunov Stability and Regions of
  Attraction", §2.4.3 "Safety-constrained MDP").
- **Epistemic-uncertainty model** (GP) shrinks with data → the certified safe set
  grows over time (TOC §2.6).
- **Safe MPC**: plan actions that keep the system inside the recoverable region
  (backup policy always available).

## Experimental setup
- Flying robotic vehicle (BO part); simulated dynamical systems for the model-based
  part (from abstract; details not extracted).

## Key results
- Not extracted (front matter only). The headline guarantee: **safety maintained
  throughout the learning process**, with performance improving as uncertainty
  shrinks (abstract).

## Code / data availability
- Berkenkamp released open-source libraries for this line of work (e.g.,
  `safe_learning` / `SafeOpt` on GitHub) — **not confirmed from the read pages**;
  verify before citing.

## Notable claims (from abstract)
- Algorithms are "**conservative in the beginning**, when uncertainties are large,
  but become more confident over time ... **remain safe at all times**" (p.3).
- Model-based safety = "restricting exploration to the set of states where a **safe
  backup strategy is available**" (p.3).

## Related notes
- [P14 — Constrained Policy Optimization (Achiam et al. 2017)](P14_achiam2017_cpo.md) — alternative safe-RL route: *constraint satisfaction (CMDP)* vs *Lyapunov stability* here.
- [P18 — Statistical Model Predictive Shielding](P18_smps_shielding.md) — closely related (MPC-based recovery / shielding to a safe region).
- [P19 — ECO (Huang et al. 2026)](P19_eco_energy_constrained.md) — constrained-RL (Lagrangian) take on safety/energy.
- [P15 — Safe RL for Legged Locomotion](P15_yang_safe_rl_legged.md) — applies safe-RL ideas to legged robots.

## Caveats (not verified / limits)
- **Only abstract + TOC read** (10/205 pages). Method/equations/experiments are
  inferred from the abstract and chapter titles → low confidence; read the relevant
  chapter (or the NeurIPS'17 paper directly) for the Lyapunov/GP theorems and the
  exact safe-set construction.
- The INDEX title says "NeurIPS paper"; the file is the **thesis** superset.
- Domain: control-theoretic / robotics, not prosthesis/OpenSim — relevant as the
  **Lyapunov-stability safe-exploration** reference cited by the S01 synthesis.
