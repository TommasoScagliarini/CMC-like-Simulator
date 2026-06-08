# INDEX — "opensim + rl" paper corpus

Router for the knowledge base. PDFs live on the Desktop (not in the repo),
abbreviated `@/` below (see **PDF locations** appendix). The same corpus exists on
both machines:
- macOS: `/Users/tommy/Desktop/report opensim+rl/`
- Windows: `C:\Users\tomma\Desktop\report opensim+rl\`

Usage protocol: [README_FOR_LLM.md](README_FOR_LLM.md).
Topic map: [TOPICS.md](TOPICS.md).

`note` column links the canonical note filename (stable even when `pending`, so
cross-links resolve once written).

## Prosthesis / robotic knee control (RL)
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P01 | RL Impedance Control of a Robotic Prosthesis to coordinate w/ intact knee (echo control) — Wu et al. | 2022 | IEEE RA-L | prosthesis_knee, impedance_control, hierarchical_control, action_param_tuning, reward_tracking, safety_bounds, human_in_loop | ✅ [P01](notes/P01_wu2022_rl_impedance_echo_control.md) |
| P02 | Online RL Control for Personalization of a Robotic Knee Prosthesis — Wen et al. | 2020 | IEEE T-Cyb | prosthesis_knee, online_rl, impedance_control, human_in_loop, safety_bounds | ✅ [P02](notes/P02_wen2020_online_rl_personalization.md) |
| P03 | Wearer-Prosthesis Interaction for Symmetrical Gait — Wen et al. | 2020 | IEEE TNSRE | prosthesis_knee, symmetry, reward_design, human_in_loop | ✅ [P03](notes/P03_wen2020_wearer_prosthesis_symmetry.md) |
| P04 | Hierarchical Optimization for Control of Robotic Knee Prostheses (propulsive symmetry) — Li et al. | 2023 | IEEE TBME | hierarchical_control, prosthesis_knee, action_space, symmetry | ✅ [P04](notes/P04_li2023_hierarchical_knee_symmetry.md) |
| P05 | Human-Robotic Prosthesis as Collaborating Agents for Symmetrical Walking — Wu et al. | 2022 | NeurIPS | multi_agent_rl, human_in_loop, prosthesis_knee, symmetry | ✅ [P05](notes/P05_wu2022_human_prosthesis_collaborating_agents.md) |

## DRL on OpenSim / musculoskeletal locomotion
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P06 | Learning to Run challenge: physiologically accurate motion w/ DRL — Kidzinski et al. | 2018 | NeurIPS comp. | drl_opensim, musculoskeletal, reward_design | ✅ [P06](notes/P06_kidzinski2018_learning_to_run.md) |
| P07 | DRL for Modeling Human Locomotion Control in Neuromechanical Simulation — Song et al. | 2021 | J NeuroEng Rehab | drl_opensim, musculoskeletal, review | ✅ [P07](notes/P07_song2021_drl_neuromechanical_locomotion.md) |
| P08 | Learning to Ascend Stairs and Ramps: DRL for Musculoskeletal Model — Adriaenssens et al. | 2022 | Sensors | drl_opensim, musculoskeletal, curriculum | ✅ [P08](notes/P08_adriaenssens2022_stairs_ramps.md) |
| P09 | Simulating Human Walking: model-based RL w/ musculoskeletal modeling — Su & Gutierrez-Farewik | 2023 | Front. Neurorobot. | drl_opensim, model_based_rl, compute_cost | ✅ [P09](notes/P09_su2023_mbrl_walking.md) |
| P10 | KINESIS — Motion Imitation for Human Musculoskeletal Locomotion — Simos et al. | 2025? | preprint | motion_imitation, musculoskeletal, warm_start | ✅ [P10](notes/P10_simos_kinesis_motion_imitation.md) |
| P11 | DRL for Physics-Based Musculoskeletal Sims of Healthy & Transfemoral Prosthesis Users — De Vree et al. | 2023? | IEEE TNSRE | drl_opensim, transfemoral, musculoskeletal | ✅ [P11](notes/P11_devree_drl_transfemoral.md) |

## Hierarchical control / action space (quadruped/humanoid)
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P12 | GLiDE — Generalizable Quadrupedal Locomotion w/ a Centroidal Model | 2022? | ICRA? | hierarchical_control, action_space, locomotion_rl | ✅ [P12](notes/P12_glide_quadrupedal_centroidal.md) |
| P13 | Learning Torque Control for Quadrupedal Locomotion | 2023? | Humanoids? | action_space, torque_vs_position, locomotion_rl | ✅ [P13](notes/P13_learning_torque_control_quadruped.md) |

## Safe RL / shielding / stability
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P14 | Constrained Policy Optimization — Achiam et al. | 2017 | ICML | safe_rl, cmdp, constrained_optimization | ✅ [P14](notes/P14_achiam2017_cpo.md) |
| P15 | Safe Reinforcement Learning for Legged Locomotion — Yang et al. | 2022? | IROS? | safe_rl, legged_locomotion, recovery_policy | ✅ [P15](notes/P15_yang_safe_rl_legged.md) |
| P16 | Safe Robot Learning in Assistive Devices through Neural Network Repair — Majd et al. | 2023? | — | safe_rl, nn_repair, assistive_devices, verifiable | ✅ [P16](notes/P16_majd_nn_repair_assistive.md) |
| P17 | Safe Model-Based RL with Stability Guarantees — Berkenkamp et al. | 2017 | NeurIPS | safe_rl, lyapunov, stability_guarantees, model_based_rl | ✅ [P17](notes/P17_berkenkamp2017_safe_mbrl_lyapunov.md) |
| P18 | Statistical Model Predictive Shielding (SMPS) | 2021? | — | safe_rl, shielding, backup_policy | ✅ [P18](notes/P18_smps_shielding.md) |

## Reward design / energy / Cost of Transport
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P19 | ECO — Energy-Constrained Optimization w/ RL for Humanoid Walking — Huang et al. | 2024? | — | reward_design, cost_of_transport, constrained_optimization, lagrangian | ✅ [P19](notes/P19_eco_energy_constrained.md) |
| P20 | Powered Ankle-Foot Prosthesis Improves Walking Metabolic Economy — Au et al. | 2009 | — | metabolic_cost, sea_elastic, prosthesis_ankle | ✅ [P20](notes/P20_au2009_powered_ankle_foot.md) |
| P21 | Predicting the Metabolic Cost of Incline Walking from Muscle Activity & Mechanics — Silder et al. | 2012 | J Biomech | metabolic_cost, surrogate_energy, biomechanics | ✅ [P21](notes/P21_silder2012_metabolic_cost_incline.md) |

## POMDP / memory (recurrent / world model)
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P22 | LSWM — A Long–Short History World Model for Bipedal Locomotion via RL | 2024? | — | pomdp, memory, world_model, bipedal | ✅ [P22](notes/P22_lswm_world_model_bipedal.md) |

## RLVR (verifiable rewards)
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P23 | Grounding World Models via Self-Supervised Reward Alignment (GrndCtrl) | 2025? | — | rlvr, world_model, verifiable, robotics | ✅ [P23](notes/P23_grndctrl_world_model_reward_alignment.md) |
| P24 | RL from Verifiable Rewards for Reasoning Language Models | 2024? | — | rlvr, llm, verifiable_rewards | ✅ [P24](notes/P24_rlvr_reasoning_lm.md) |

## Software / C++↔Python integration
| id | title (abbrev.) | year | venue | topics | note |
|----|-----------------|------|-------|--------|------|
| P25 | Wrapyfi — Python Wrapper for Integrating Robots/Sensors across Middleware | 2023? | — | software_integration, middleware, zeromq | ✅ [P25](notes/P25_wrapyfi_middleware.md) |
| D01 | PyTorch C++ Frontend (LibTorch) — tutorial | — | docs (HTML) | software_integration, libtorch, cpp_inference | ✅ [D01](notes/D01_pytorch_cpp_frontend.md) |
| D02 | osim-rl — RL with musculoskeletal models | — | docs (HTML) | drl_opensim, osim_rl, env_api | ✅ [D02](notes/D02_osim_rl_musculoskeletal.md) |

## Deep-research syntheses
| id | title | type | status | source |
|----|-------|------|--------|--------|
| S01 | Exploratory feasibility review (GPT) | md + pdf | read (md) | `@/GPT/deep-research-report.md` (≡ `@/GPT/Revisione...pdf`) |
| S02 | AI for Prosthetic Trajectory Generation (Gemini) | pdf | ✅ read | `@/Gemini/AI for Prosthetic Trajectory Generation.pdf` |

Legend: ✅ extracted · ⏳ pending.
Years/venues with `?` are uncertain (inferred from index/filename, not verified in
the PDF) — confirm when extracting each note.

---

## Appendix — PDF locations

`@/` resolves to the corpus root on either machine:
- macOS: `@/` = `/Users/tommy/Desktop/report opensim+rl/`
- Windows: `@/` = `C:\Users\tomma\Desktop\report opensim+rl\` (use `\` separators)

- P01 `@/GPT/paper/2 - Protesi robotiche di ginocchio-transfemorali/Reinforcement Learning Impedance Control of a Robotic Prosthesis to Coordinate With Human Intact Knee Motion.pdf`
- P02 `@/GPT/paper/2 - .../Online Reinforcement Learning Control for the Personalization of a Robotic Knee Prosthesis.pdf`
- P03 `@/GPT/paper/2 - .../Wearer-Prosthesis Interaction for Symmetrical Gait - A Study Enabled by Reinforcement Learning Prosthesis Control.pdf`
- P04 `@/GPT/paper/2 - .../Hierarchical Optimization for Control of Robotic Knee Prostheses Toward Improved Symmetry of Propulsive Impulse.pdf`
- P05 `@/GPT/paper/2 - .../Human-Robotic Prosthesis as Collaborating Agents for Symmetrical Walking.pdf`
- P06 `@/GPT/paper/1 - OpenSim-DRL-locomozione/Learning to Run challenge Synthesizing physiologically accurate motion using deep reinforcement learning.pdf`
- P07 `@/GPT/paper/1 - .../Deep reinforcement learning for modeling human locomotion control in neuromechanical simulation.pdf`
- P08 `@/GPT/paper/1 - .../Learning to Ascend Stairs and Ramps Deep Reinforcement Learning for a Physics-Based Human Musculoskeletal Model.pdf`
- P09 `@/GPT/paper/1 - .../Simulating human walking a model-based reinforcement learning approach with musculoskeletal modeling.pdf`
- P10 `@/GPT/paper/1 - .../KINESIS - Motion Imitation for Human Musculoskeletal Locomotion.pdf`
- P11 `@/Gemini/paper/1 - Deep Reinforcement Learning per Protesi e OpenSim/Deep_Reinforcement_Learning_for_Physics-Based_Musculoskeletal_Simulations_of_Healthy_Subjects_and_Transfemoral_Prostheses_Users.pdf`
- P12 `@/Gemini/paper/2 - Controllo Gerarchico, Spazio delle Azioni e Locomozione/Generalizable Quadrupedal Locomotion in Diverse Environments with a Centroidal Model (GLiDE).pdf`
- P13 `@/Gemini/paper/2 - .../Learning Torque Control for Quadrupedal Locomotion.pdf`
- P14 `@/GPT/paper/3 - Safe RL-vincoli-sicurezza/Constrained Policy Optimization.pdf`
- P15 `@/GPT/paper/3 - .../Safe Reinforcement Learning for Legged Locomotion.pdf`
- P16 `@/GPT/paper/3 - .../Safe Robot Learning in Assistive Devices through Neural Network Repair.pdf`
- P17 `@/Gemini/paper/3 - Safe RL, Shielding e Garanzie di Stabilità/Safe Model-Based Reinforcement Learning with Stability Guarantees.pdf`
- P18 `@/Gemini/paper/3 - .../Statistical Model Predictive Shielding (SMPS).pdf`
- P19 `@/Gemini/paper/4 - Progettazione della Reward Function e Costo del Trasporto (COT)/ECO - Energy-Constrained Optimization with Reinforcement Learning for Humanoid Walking.pdf`
- P20 `@/Gemini/paper/4 - .../Powered Ankle-Foot Prosthesis Improves Walking Metabolic Economy.pdf`
- P21 `@/GPT/paper/4 - Energia-Costo metabolico/Predicting the Metabolic Cost of Incline Walking from Muscle.pdf`
- P22 `@/Gemini/paper/5 - Osservabilità Parziale (POMDP) e Reti Ricorrenti (Memoria)/LSWM - A Long–Short History World Model for Bipedal Locomotion via Reinforcement Learning.pdf`
- P23 `@/Gemini/paper/6 - Reinforcement Learning from Verifiable Results (RLVR) in Robotica/Grounding World Models via Self-Supervised Reward Alignment.pdf`
- P24 `@/Gemini/paper/6 - .../Reinforcement Learning from Verifiable Rewards for Reasoning Language Models.pdf`
- P25 `@/Gemini/paper/7 - Architettura Software e Interfacciamento (C++ e Python)/Wrapyfi A Python Wrapper for Integrating Robots, Sensors, and Applications across Multiple Middleware.pdf`
- D01 `@/Gemini/paper/7 - .../Using the PyTorch C++ Frontend — PyTorch Tutorials 2.11.0+cu130 documentation.html`
- D02 `@/Gemini/paper/1 - .../Reinforcement learning with musculoskeletal models.html`
