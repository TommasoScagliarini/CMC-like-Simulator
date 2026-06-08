---
id: P20
title: "Powered Ankle–Foot Prosthesis Improves Walking Metabolic Economy"
authors: "Samuel K. Au, Jeff Weber, Hugh Herr (MIT Media Lab)"
year: 2009
venue: "IEEE Transactions on Robotics, vol. 25, no. 1, pp. 51-66"
doi_or_url: "10.1109/TRO.2008.2008747"
topics: [metabolic_cost, sea_elastic, prosthesis_ankle, impedance_control]
keywords: [powered-ankle-foot-prosthesis, series-elastic-actuator, parallel-elasticity, cost-of-transport, FSM, impedance-control, net-positive-work, transtibial]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/4 - Progettazione della Reward Function e Costo del Trasporto (COT)/Powered Ankle-Foot Prosthesis Improves Walking Metabolic Economy.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\paper\4 - Progettazione della Reward Function e Costo del Trasporto (COT)\Powered Ankle-Foot Prosthesis Improves Walking Metabolic Economy.pdf"
pages_read: "1-3 of 16 (intro/challenges/ankle biomechanics; design & results skimmed)"
extraction_confidence: high  # clean text; figures not interpreted
related: [P19, P21, P09]
---

# P20 — Powered Ankle-Foot Prosthesis (SEA) improves metabolic economy

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [metabolic_cost](../TOPICS.md) · [sea_elastic](../TOPICS.md) · [prosthesis_ankle](../TOPICS.md) · [impedance_control](../TOPICS.md)

## TL;DR
Foundational hardware paper: a powered ankle-foot prosthesis built with a **series-
elastic actuator (SEA)** + parallel spring delivers human-like **net positive ankle
work**, and **reduces amputees' metabolic Cost of Transport (COT) by ~14%** vs passive
prostheses (3 transtibial subjects), despite being 2× heavier. The canonical "why SEA"
+ "why energy matters" reference; the project uses SEA actuators.

## Problem & contribution
- Passive prostheses can't do net positive ankle work → transtibial amputees use
  **20-30% more metabolic power** and prefer 30-40% slower speeds; gait is asymmetric
  (p.1).
- Contribution: design + build a powered ankle-foot prosthesis with **series +
  parallel elasticity** that mimics human ankle dynamics; test whether it lowers
  metabolic COT (p.1-2).

## Method / architecture
- **Actuator**: unidirectional parallel spring + a **force-controllable actuator with
  series elasticity (SEA)** — delivers high mechanical power / net positive work (p.1).
- **Control**: a **finite-state machine** giving **impedance control during stance**
  and **position control during swing**, mimicking the human ankle (which is believed
  to be impedance-controlled in stance, position-controlled in swing) (p.2).
- **Local sensing only** preferred on the prosthesis (a control constraint) (p.2).
- **Human ankle biomechanics** (target behavior): stance (60%) + swing (40%); stance
  subphases CP (controlled plantarflexion, linear spring), CD (controlled
  dorsiflexion, nonlinear stiffening spring storing energy), PP (powered
  plantarflexion, net positive work). Torque-angle loop area W = net work (Fig.2, p.2-3).
- **Specs**: peak ankle power ~350 W, torque ~140 N·m; shank-ankle-foot ~2 kg (78 kg
  person) (p.2).

## Experimental setup
- 3 unilateral transtibial amputees; level-ground self-selected speed; O₂/CO₂ to
  estimate metabolic rate; vs Flex-Foot Ceterus & Freedom Innovations Sierra passive
  feet (p.1).

## Key results
- **~14% lower metabolic COT** with the powered prosthesis vs passive, despite >2×
  the mass (abstract). Delivers the high power / net positive work of normal walking
  (abstract).

## Code / data availability
- Hardware study (MIT / VA-funded). No code/dataset.

## Notable claims (page-anchored)
- The human ankle does **net positive work** at moderate/fast speeds (~0.10 J/kg net,
  ~3.5 W/kg peak power at 1.25 m/s); passive prostheses can't → the hypothesized cause
  of amputees' metabolic/speed/symmetry deficits (p.1-2).
- A powered prosthesis must be **position- and impedance-controllable**; human ankle
  = impedance (stance) + position (swing) (p.2). *(Supports phase-dependent control;
  the project's SEA cascade is a position-reference + impedance/torque loop.)*
- **Series + parallel elasticity** is how to meet the demanding human-like power/torque
  specs in a wearable package (p.2). *(Rationale for SEA.)*

## Related notes
- [P19 — ECO (Huang et al. 2026)](P19_eco_energy_constrained.md) — energy/COT as an RL objective/constraint; this paper is the physical/metabolic grounding.
- [P21 — Predicting Metabolic Cost (Silder et al. 2012)](P21_silder2012_metabolic_cost_incline.md) — surrogate metabolic-cost estimation.
- [P09 — MBRL walking (Su 2023)](P09_su2023_mbrl_walking.md) — reflex control applied to powered ankle prostheses (Eilenberg/Thatte lineage).

## Caveats (not verified / limits)
- Read 3 of 16 pages: the **detailed mechanical/SEA design, FSM control law, and
  metabolic results tables** not extracted (only the abstract's 14%).
- Figures (Fig.1-3 ankle biomechanics/model) **not interpreted**.
- 2009 hardware/biomechanics paper, **transtibial ankle** (the project is
  transfemoral knee+ankle SEA). Relevant for SEA rationale, COT, and phase-dependent
  impedance/position control — not an RL method.
