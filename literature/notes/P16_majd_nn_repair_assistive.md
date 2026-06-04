---
id: P16
title: "Safe Robot Learning in Assistive Devices through Neural Network Repair"
authors: "Keyvan Majd, Geoffrey Clark, Tanmay Khandait, Siyu Zhou, Sriram Sankaranarayanan, Georgios Fainekos, Heni Ben Amor (ASU / CU Boulder / Toyota)"
year: 2022
venue: "Conference on Robot Learning (CoRL) 2022"
doi_or_url: "CoRL 2022 (PMLR)"
topics: [safe_rl, nn_repair, assistive_devices, verifiable, prosthesis_ankle]
keywords: [NN-repair, NNRepLayer, MIQP, formal-safety-constraints, ReLU, control-rate-bounds, lower-leg-prosthesis, PD-controller, imitation-learning]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/3 - Safe RL-vincoli-sicurezza/Safe Robot Learning in Assistive Devices through Neural Network Repair.pdf"
pages_read: "1-4 of 11 (abstract/intro/problem/formulation; experiments skimmed)"
extraction_confidence: high  # clean text; MIQP constraint equations partially captured; figures not interpreted
related: [P14, P18, P17, P02]
---

# P16 — Safe assistive-device learning via Neural Network Repair

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [safe_rl](../TOPICS.md) · [nn_repair / verifiable](../TOPICS.md) · [assistive_devices](../TOPICS.md) · [prosthesis_ankle](../TOPICS.md)

## TL;DR
**Post-hoc repair** of a trained NN controller's weights so its outputs **provably
satisfy formal safety constraints** (max control, velocity/joint-angle limits, control-
*rate* bounds), while preserving the original task performance. Formulated as a **Mixed
Integer Quadratic Program (MIQP)** on a ReLU network (NNRepLayer). Demonstrated on a
**real lower-leg (ankle) prosthesis** where the NN predicts control parameters driving
a **PD controller** — architecturally close to the project (network → reference/params
→ low-level PD).

## Problem & contribution
- ML adapts assistive devices to individuals, but NN controllers can output unsafe
  values on unseen data; adoption is limited by safety (p.1).
- Contribution: an algorithm that **updates a trained NN to satisfy formal safety
  predicates** while minimizing the original loss; **repairs any layer** (larger
  feasible space than output-only methods); **theoretical guarantee** of repairing all
  discovered unsafe samples; **first NN-repair on a real physical system** (powered
  prosthesis) (p.1-2).

## Method / architecture
- Controller: NN policy `π_θ` predicts ankle control parameters from sensors → drives
  the prosthesis via a **PD controller** (p.2). `π_θ` may be trained by imitation or
  RL (p.3).
- **Repair Problem**: given trained `π_θ` and a predicate `Ψ(y, x0)` constraining
  outputs for inputs of interest `Xr`, modify weights/biases so `π_θr` satisfies `Ψ`
  while preserving behavior (p.3).
- **NNRepLayer**: repair a **single layer** (the full multi-layer problem is non-convex
  due to ReLU + weight products). For ReLU nets, encode as **MIQP**: minimize
  `E(θ_w, θ_b) = Σ‖y_n − t_n‖²` (deviation from original targets) subject to the network
  forward constraints + the safety predicate `Ψ` → PDF p.4, eqs.(1)-(5) (partially
  captured). Deterministic global optimization → optimality guarantee (no gradient
  descent, no counterexample relabeling) (p.2-4).
- Safety predicates for prosthesis: control magnitude threshold, velocity limits, joint
  angle limits, **bounds on control-input change (rate)** (p.1). *(Control-rate /
  velocity bounds are exactly the kind of constraints relevant to the project's knee
  saturation / qddot issues.)*

## Experimental setup
- Real powered **lower-leg (ankle) prosthesis**; validation data reveals violations;
  NNRepLayer removes them (Fig.2) (p.1-3). *(Quantitative results §5 not extracted.)*

## Key results
- Produces **safe NN policies** for the lower-leg prosthesis satisfying a variety of
  constraints, in real-world robot experiments (abstract). Repairing the **output layer
  alone is often infeasible/insufficient** → repairing arbitrary layers helps (p.3).

## Code / data availability
- CoRL 2022 (ASU Ben Amor group). MIQP solver (e.g., Gurobi) implied; no repo noted in
  read pages.

## Notable claims (page-anchored)
- Retraining/fine-tuning on counterexamples has pitfalls: needs labels, and **gradient
  descent gives no constraint-satisfaction guarantee** (p.2).
- **Minimal weight deviation ≠ preserved performance** (subtle weight changes can
  greatly change behavior) → must minimize the original training loss during repair
  (p.3).
- Safety predicate may **override the original control reference** for unsafe input
  states — "a natural constraint in many applications" (p.3).

## Related notes
- [P14 — Constrained Policy Optimization](P14_achiam2017_cpo.md) — in-training constraints vs this post-hoc repair.
- [P18 — Statistical Model Predictive Shielding](P18_smps_shielding.md), [P17 — Safe MBRL](P17_berkenkamp2017_safe_mbrl_lyapunov.md) — runtime/safe-set alternatives vs weight repair here.
- [P02 — Online RL personalization (Wen 2020)](P02_wen2020_online_rl_personalization.md) — prosthesis NN control (different safety approach: bounds + reset).

## Caveats (not verified / limits)
- Read 4 of 11 pages: the **full MIQP constraints (eqs.2-5), the repair algorithm, and
  experimental results** not fully extracted.
- Figures (Fig.1 prosthesis, Fig.2 repair overview) **not interpreted**.
- Most relevant transferable idea: **formal output constraints** (control magnitude,
  velocity, joint angle, **control-rate**) on a network→PD prosthesis controller, and
  the option of **repairing** vs retraining to guarantee them.
