---
id: P21
title: "Predicting the Metabolic Cost of Incline Walking from Muscle Activity and Walking Mechanics"
authors: "Amy Silder, Thor Besier, Scott L. Delp (Stanford / Auckland)"
year: 2012
venue: "Journal of Biomechanics 45(10): 1842-1849"
doi_or_url: "10.1016/j.jbiomech.2012.03.032"
topics: [metabolic_cost, surrogate_energy, biomechanics]
keywords: [metabolic-cost-prediction, EMG, regression-model, joint-kinematics-kinetics, incline-walking, sum-squared-activation, surrogate]
pdf: "/Users/tommy/Desktop/report opensim+rl/GPT/paper/4 - Energia-Costo metabolico/Predicting the Metabolic Cost of Incline Walking from Muscle.pdf"
pages_read: "1-5 of 17 (abstract/intro/methods; full regression results skimmed)"
extraction_confidence: high  # clean text; figures/tables not interpreted
related: [P19, P20, P09]
---

# P21 — Predicting metabolic cost from muscle activity & gait mechanics

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [metabolic_cost](../TOPICS.md) · [surrogate_energy](../TOPICS.md) · [biomechanics](../TOPICS.md)

## TL;DR
Biomechanics study identifying which signals best **predict the metabolic cost** of
incline walking. A regression on **incline + squared integrated EMG of soleus & vastus
lateralis** explains **96%** of the variance; a model using **only joint
kinematics/kinetics** (a few peak angles/moments) explains **89%**. → You can build a
**metabolic-cost surrogate** from muscle activation² or from kinematics/kinetics —
directly useful for an energy/COT reward term computable in simulation.

## Problem & contribution
- Classic metabolic models (Pandolf: mass, speed, incline) ignore muscle activation,
  body composition, and gait features → inaccurate when gait is altered by assistive
  tech (p.1-2).
- Contribution: regression models predicting net metabolic cost from EMG / physical
  characteristics / gait features during 0/5/10% incline walking (16 subjects) (p.2-3).

## Method / architecture
- 16 subjects; treadmill at 0%, 5%, 10% incline, preferred speed (~1.29 m/s);
  O₂/CO₂ metabolic; DXA body composition; VICON 100 Hz; GRF 2000 Hz; 29-DoF, 12-segment
  model (Delp 1990); inverse kinematics + inverse dynamics; EMG of 8 lower-limb muscles
  (p.3-4).
- **Net normalized metabolic cost** = (walking − standing metabolic) / body mass (p.3).
- Multivariate linear regression to predict metabolic cost from subsets of variables
  (p.4-5).

## Key results
- **Incline + Σ(integrated EMG²) of soleus + vastus lateralis → R² ≈ 0.96** (abstract).
- **Kinematics/kinetics only** (peak stance knee-flexion angle, peak knee-extension
  moment, peak ankle-plantarflexion moment, peak hip-flexion moment) → **R² ≈ 0.89**
  (abstract). → mechanics alone predict metabolic cost well.

## Code / data availability
- Stanford/Delp lab (OpenSim ecosystem); hardware/MoCap study. No code/dataset noted.

## Notable claims (page-anchored)
- Musculoskeletal simulations often minimize **Σ squared muscle activations** as a
  proxy for metabolic cost, but "**this relationship has not been rigorously tested**"
  (p.2). *(Caveat for using activation² as an energy reward.)*
- A handful of **peak joint angles/moments** carry most of the metabolic-cost signal
  (R²≈0.89) → a cheap kinematic/kinetic **energy surrogate** is feasible (abstract).
- Body fat / distribution and gait features (step width, flexed-knee walking, arm
  swing) affect metabolic cost beyond mass/speed/incline (p.2).

## Related notes
- [P19 — ECO (Huang et al. 2026)](P19_eco_energy_constrained.md) — uses energy/COT as an RL constraint; this gives the *surrogate* one could constrain/penalize.
- [P20 — Powered Ankle-Foot Prosthesis (Au et al. 2009)](P20_au2009_powered_ankle_foot.md) — measured metabolic COT improvement (the quantity this paper predicts).
- [P09 — MBRL walking (Su 2023)](P09_su2023_mbrl_walking.md), [P07 — Song review](P07_song2021_drl_neuromechanical_locomotion.md) — muscle-effort minimization as objective.

## Caveats (not verified / limits)
- Read 5 of 17 pages: the **exact regression coefficients, full variable lists, and
  validation** not extracted (only abstract R² values).
- Figures/tables **not interpreted**.
- Not RL; it's a biomechanics regression. Relevant only as a **metabolic-cost
  surrogate** recipe (activation² and/or kinematics-kinetics) for a possible energy
  reward term — and the caveat that activation²↔metabolic is unvalidated.
