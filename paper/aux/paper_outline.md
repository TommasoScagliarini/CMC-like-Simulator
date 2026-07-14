# Annotated Paper Outline — RL Trajectory Generation for a Series-Elastic Prosthesis

**Format:** IEEE conference, double-column, 10 pages.

**Thesis:** Reinforcement learning generates the *kinematic reference trajectory* that drives a robotic (series-elastic) transfemoral prosthesis, inside a muscle-driven CMC-like OpenSim simulation.

**Protagonist:** Pillar 4 (RL Trajectory Generator). Pillars 1–3 (SEA, cascade/PI, CMC-like simulator) are the validated *testbed* that makes the learned trajectory physically honest.

**Subject:** AB06 (single subject — state explicitly). Simulation-only (OpenSim + MuJoCo/MJX twin).

> STATUS legend per section: **[READY]** writable now from existing material · **[PARTIAL]** writable but needs one missing number/figure · **[BLOCKED]** needs experiments that do not yet exist.

---

## Title (candidates)
1. *Learning Kinematic Reference Trajectories for a Series-Elastic Transfemoral Prosthesis in a Muscle-Driven Simulation*
2. *Reinforcement-Learned Trajectory Generation for Series-Elastic Prosthetic Gait*
3. *From Normative Kinematics to Learned References: RL Trajectory Generation for an SEA Knee–Ankle Prosthesis*

🇮🇹 Nota: il titolo deve contenere "trajectory generation/reference" + "series-elastic prosthesis" + "reinforcement learning". Evita "control" come parola guida: il tuo contributo è la *generazione del riferimento*, non il controllo low-level.

---

## Abstract (~0.5 col) — **[BLOCKED on results]**
Structure (7 sentences):
1. Motivation: active transfemoral prostheses need references that are both physiologically valid and feasible for the actuator (cite P20).
2. Gap: prior OpenSim+RL prosthesis work uses muscle-like actuators / muscle excitations (P11) or tunes impedance increments (P01–P04); none *learn the kinematic reference* for a real SEA prosthesis in a muscle-driven loop.
3. Approach: an RL policy emits the absolute knee+ankle reference; a band-limited reference governor is the action interface; a cascade/PI driver tracks it on a two-SEA prosthesis; the contralateral side stays muscle-driven.
4. Testbed validity (one number): muscle-recruitment ~96%, plugin-vs-Python SEA agreement ~5e-7 N·m.
5. **Key method result:** the reference governor removes motor-torque saturation (33.9%→0%) without retuning the validated plant.
6. **[BLOCKED] Headline RL result:** ⟪learned-trajectory tracking RMSE in deg vs IK baseline; symmetry/feasibility number; multi-seed mean±SD⟫.
7. Takeaway: RL can generate feasible prosthetic kinematic references when the action is band-limited to the actuator's bandwidth.

🇮🇹 Nota: lascia la frase 6 come placeholder esplicito finché non hai la baseline ladder. NON scrivere numeri di return/reward qui.

---

## I. Introduction (~0.75 col) — **[READY]**
- **¶1 Motivation.** Powered prostheses must restore net-positive joint work; passive devices raise metabolic cost; compliant/SEA actuation enables dynamic interaction and shock tolerance (P20, P05, P01). Position-only control precludes interaction → motivates SEA + a *reference* the actuator can realize.
- **¶2 Gap.** OpenSim+RL prosthesis work either drives *muscle-like linear actuators* with RL excitations (P11, P08) or learns *impedance increments* on a fixed reference (P01–P04). Fixed normative IK references are provably sub-optimal (P04: best kinematics ≠ normative). **No prior work learns the full kinematic reference for a series-elastic prosthesis in a muscle-driven simulation.**
- **¶3 Challenge.** A naive learned reference is acceleration-discontinuous and out-of-band → it rails the SEA motor regardless of controller gains. The reference must respect the actuator's physical bandwidth.
- **¶4 Contributions (numbered):**
  1. An RL trajectory generator that outputs the absolute knee+ankle kinematic reference for an SEA prosthesis (action = trajectory, not torque, not impedance).
  2. A **band-limited reference governor** as the policy's action interface, which makes the learned reference physically feasible (saturation 33.9%→0%) without retuning the plant.
  3. A muscle-driven CMC-like OpenSim testbed with two integrated SEAs and a *reserve-as-fidelity-gauge* methodology, plus a MuJoCo/MJX twin for GPU-batched training.
  4. ⟪**[BLOCKED]** Quantitative evaluation of the learned trajectories vs an IK-tracking baseline (deg, symmetry, multi-seed).⟫
- End ¶4 with forward pointer to **Fig. 1** (system block diagram).

Citations: P20, P05, P01, P11, P08, P04, P06.

---

## II. Related Work (~0.75 col) — **[READY]**
- **¶1 RL prosthesis control lineage:** echo/impedance RL (P01), online ADP personalization (P02, OpenSim precursor), symmetry/wearer-interaction (P03), hierarchical high-level target + low-level tracker (P04), multi-agent validated in OpenSim (P05). *State the distinction:* these tune impedance/targets; **we generate the kinematic reference.**
- **¶2 OpenSim+RL musculoskeletal & action space:** founding benchmark (P06), control taxonomy (P07), PPO+imitation transfemoral with muscle-like actuators (P11 — closest precedent, state why we differ), position-vs-torque action choice (P13), reduced-order high-level + classical low-level (P12), the project's own feasibility synthesis (S02). Imitation reward / look-ahead reference (P10).
- **Table I (P11-style comparison):** our method vs {muscle-like-actuator DRL, impedance-tuning RL, MPC/optimization} across columns {actuator model, RL output, low-level controller, biological side, validation engine}.

Citations: P01–P13, S02, D02, P10.

---

## III. Simulation Testbed (~1.75 col) — **[READY]** *(compressed: SEA + cascade + CMC-like)*
🇮🇹 Nota: questa sezione fonde i tuoi Pillar 1+2+3 in *infrastruttura*. Tieni le equazioni fitte, niente derivazioni lunghe; i dettagli del driver vanno in appendice.

### A. CMC-like muscle-driven model (the body)
- Pipeline (one block sub-diagram or fold into Fig. 1): IK spline (6 Hz zero-phase Butterworth) → biological outer-loop computed-torque PD (bio DOFs; Kp=100/Kd=20, pelvis 25/10) → zero-actuator inverse dynamics with mass-matrix projection (avoids `realizeAcceleration`) → muscle-first static-optimization QP with penalized reserves → SEA integration (RK4-bypass, dt=1 ms).
- AB06 model build (one sentence + **Table IV**): mass-preserving graft 78.099 kg; two SEAs (knee, ankle); contralateral muscle-driven.
- Equations: ID projection (Eq. 15), outer computed-torque (Eq. 16), SO QP objective (Eq. 17).
- **Reserve-as-fidelity-gauge** (one paragraph — genuine methodological point): reserves penalized (w_res=1e6); floating-base reserves track the instantaneous external-force deficit (corr up to 0.9999) → reserves are a *quantitative readout of physical validity*.

### B. Series-elastic actuator and low-level driver
- Two-state rotor (θ_m, ω_m) + integral state ξ. **Use the LIVE plugin law** (Eqs. 1–6). One sentence each on the feed-forward `+tau_ref` (removes a structural torque bias) and the integral term.
- Frequency characterization (Eqs. 7–9): mechanical resonance (knee 28.5 Hz, ankle 35.6 Hz, nearly undamped — all damping from the controller); motor-driver band ~125 Hz (ζ≈0.71). *This defines the band the learned reference must respect.*
- **Table II:** SEA parameters (K, Jm, Bm, Kp, Kd, Ki, F_opt, clamp) knee & ankle. **Fig. 2:** SEA two-body schematic. **Fig. 3:** driver pole map + Bode (sim vs analytic).

### C. Cascade + PI reference-tracking controller
- Three-level cascade: outer position-P → inner velocity-PI → normalized u (Eqs. 10–13). One sentence on anti-windup. **Table III:** cascade gains + governor limits.
- One-sentence sub-result: the inner PI cancels a velocity-dependent torque bias `−(Kd+Bm)ω_j/(1+Kp)` (Eq. 14); cascade is ~10% cleaner than single-loop PID.

🇮🇹 Nota: la driver-isolation suite, il pole-placement dettagliato e il chattering 88 Hz → *appendice o una frase*. Sono ottimi ma non sono la storia.

---

## IV. RL Trajectory Generator (~2.0 col) — **[READY]** *(the core method)*
- **¶1 MDP/POMDP formulation** (Eqs. 20, 22; P09/P10/P22 style). ⟨S, A, R, γ⟩. **Action contract (state explicitly):** absolute reference knots for (pros_knee, pros_ankle), policy_knots=1, one q_cmd per 0.01 s segment (~100 Hz), bounds knee [−1.5, 0] / ankle [−0.7, 0.7] rad. The policy generates an *absolute ex-novo reference*, not a delta from IK. Cite P13/S02 (reference over torque), P10 (look-ahead reference in obs).
- **¶2 Reference governor as action interface** (Eqs. 18–19). The same band-limiting object from §III-C sits between policy and cascade: 2nd-order form (ω_n=2π·6 → effective −3 dB at 3.86 Hz) and the production 3rd-order C2 jerk-limited model (butterworth3) with hard velocity/accel/jerk governors. **Key message:** this is the single element that converts an unstable, saturating loop into a feasible one — and it is the policy's action interface.
- **¶3 Asymmetric actor-critic.** Realistic actor obs (encoders, SEA motor states, foot load, gait-phase clock) vs privileged critic obs (full biological context, IK reference, sound-foot load); ~54% of the naive observation is non-sensorizable → motivates the split (P22). Deployability-by-construction argument. Network 2×256 tanh.
- **¶4 Reward** (Eq. 21; **Table V** with term/form/weight). Anti-phase sound-leg imitation + served-reference + SEA-physical penalties (saturation, torque-error, motor speed/accel/power) + out-of-band + smoothness/command-rate; all dimensionless / per-coordinate-normalized. DeepMimic-style score 1/(1+w·loss) (relate to exp(−k‖·‖²), P10).
  - 🇮🇹 Nota CRITICA: nel testo NON definire il successo come "reward alta". Una policy non addestrata segna già ~0.90. La reward è il segnale di training, non la metrica di risultato.
- **¶5 Gait-phase clock + hybrid GRF.** Sound-leg pacemaker locked to prescribed heel strikes; hybrid GRF (prescribed sound side, online Hunt-Crossley contact on the prosthetic foot). One honest sentence: pure online contact is structurally infeasible (memoryless contact cannot re-phase the wrench) → hybrid.
- **¶6 MuJoCo/MJX twin + training.** Static parity (CoM 2.78e-6 m, 96.7% moment arms <10%); Gymnasium contract (D02); PPO/RLlib (batch 4096, γ=0.99, λ=0.9, 10 epochs, 13 env-runners). One honest sentence: dynamic/recruitment parity not yet achieved.
- **Fig. 5:** RL agent-environment + reward diagram with actor/critic obs split and the governor (P11 Fig.1 style).

Citations: P09, P10, P13, P22, P19, S02, D02, P11.

---

## V. Experimental Setup (~0.5 col) — **[PARTIAL]**
- Subject AB06; OpenSim 4.5.2; model AB06_SEASEA_stiff321_500_pi; IK window 11.99–21.0 s; dt 1 ms; hybrid GRF. RL: 2×256 tanh MLP, 100 iters, 5 s episodes, N seeds.
- **The baseline ladder (define here, report in §VI):**
  - **B0 "Oracle":** cascade tracking the prescribed experimental IK → quality ceiling (knee 0.18°, ankle 1.22°).
  - **B1 "RL, no governor":** raw policy reference → SEA saturates (the problem).
  - **B2 "RL + governor":** the method.
  - *(optional B3)* RL absolute vs RL delta-from-IK (does it learn beyond copying IK?).
- Metrics convention: knee/ankle RMSE [deg, mean±SD], SEA tau_spring RMS [N·m], reserve norm [N·m], SEA saturation [%], success/fall rate, ≥1 symmetry/impulse index. **Never return as an outcome metric.**

🇮🇹 Nota: questa sezione è scrivibile *adesso come protocollo*; i numeri B1/B2 arrivano dagli esperimenti.

---

## VI. Results (~2.25 col — the centerpiece) — **[BLOCKED]**
Order so the RL payoff lands last. Each subsection leads with a number.

- **A. Testbed validity (short, ~0.4 col) — [READY].** Muscle-recruitment ~96% (residual ≤4e-8 N·m); plugin-vs-Python SEA agreement ~5e-7 N·m (anti-tautology — the SEA is genuinely integrated); driver clean in isolation (step 4.1% overshoot, Bode −3 dB ~125 Hz). **Table VI** (driver metrics vs analytic). One paragraph — establishes the testbed is trustworthy.
- **B. The governor makes RL feasible (~0.6 col) — [PARTIAL].** Reference governor: knee internal saturation 33.9%→0%, torque-error 41.89→0.96 N·m, motor power 15.9 kW→15 W, at the *same* validated gain Kp=18. 6 Hz filter: command saturation 59%→0%, divergent short-segment cases rescued. **Fig. 6** (governor ON/OFF time-series + bar). *This is the strongest currently-defensible RL-relevant result.*
- **C. Learned trajectory vs baseline (~0.8 col) — [BLOCKED, the money result].** ⟪Baseline ladder table B0/B1/B2 × all metrics in physical units; learned-trajectory tracking RMSE in deg vs the IK oracle; the symmetry/impulse number.⟫ **Table VII** (baseline ladder — the centerpiece). **Fig. 9** (learned reference vs IK overlay — "learned ≠ normative", P04 style).
- **D. Multi-seed RL learning (~0.45 col) — [BLOCKED].** ⟪Learning curves with mean±SD over ≥3 seeds; reward-term decomposition; clean asym-vs-sym A/B if run.⟫ **Fig. 8** (learning curves), **Table VIII** (multi-seed metrics).
- **E. MuJoCo/MJX twin & efficiency (~0.3 col) — [READY].** Static parity table; reusable reset ~1800× cheaper than rebuild; honest dynamic-parity gap. **Table IX**.

---

## VII. Discussion (~0.75 col) — **[PARTIAL]**
- ¶1 The unifying insight: the same band-limit (governor) that stabilizes tracking is what makes the policy trainable.
- ¶2 Learned reference ≠ normative IK (P04); modularity of high-level reference vs low-level tracker (policy needs no plant retuning; P04/P12).
- ¶3 Honest open problems: kinematics doesn't determine dynamics (ankle torque-sign flip at corr 0.954); reserves as fidelity detector; structural limit of memoryless contact (P02, P03).
- ¶4 SEA vs muscle-like-actuator framing (P11) — our novelty vs the closest precedent.

---

## VIII. Limitations & Future Work (~0.4 col) — **[READY]**
One sentence each: simulation-only; single subject AB06; ⟪N seeds⟫; no formal stability guarantee (P05); MuJoCo dynamic/recruitment parity incomplete; activation²↔metabolic-cost caveat (P21); imitation target coordinate mismatch (in-progress fix); residual chattering; mtp_angle_r baseline failure. Path to hardware (P08/P11), energy-as-constraint (P19), multi-subject (AB07/AB08 pipeline exists).

---

## IX. Conclusion (~0.25 col) — **[BLOCKED on results]**
Restate the four contributions and the thesis. Closing principle: *a learned high-level reference must respect the low-level actuator's bandwidth; making that constraint explicit (a reference governor) serves both controller stability and RL trainability.*

---

## Equation register (live forms — use these, NOT AGENTS.md)
| # | Equation | Where |
|---|----------|-------|
| 1 | `tau_spring = K(θ_m − θ_j)` | III-B |
| 2 | `tau_ref = u·F_opt` | III-B |
| 3 | `tau_input = tau_ref + Kp(tau_ref − tau_spring) + clamp(Ki·ξ, ±L) − Kd·ω_m` | III-B |
| 4 | `ξ̇ = tau_ref − tau_spring` (anti-windup freeze at clamp) | III-B |
| 5 | `θ̇_m = ω_m ; ω̇_m = (tau_input − tau_spring − Bm·ω_m)/Jm` | III-B |
| 6 | `tau_input = clamp(·, ±500)` N·m | III-B |
| 7 | `H(s) = K((1+Kp)s + Ki) / (Jm s³ + (Kd+Bm)s² + (1+Kp)K s + Ki K)` | III-B |
| 8 | `ω_mech = √(K/Jm)` (knee 179.16, ankle 223.61 rad/s) | III-B |
| 9 | `Kp = Jm ω_n²/K − 1 ; Kd = 2ζ Jm ω_n − Bm ; Ki = Jm p ω_n²/K, p=0.2ω_n` | III-B |
| 10 | `qdot_cas = qdot_ref + Kp_outer(q_ref − q)` | III-C |
| 11 | `e_v = qdot_cas − qdot` | III-C |
| 12 | `tau_cmd = Kp_inner·e_v + Ki_inner·∫e_v dt` | III-C |
| 13 | `u = clip(tau_cmd/F_opt, ±1)` | III-C |
| 14 | `e_ss = −(Kd+Bm)ω_j/(1+Kp)` (knee −0.584, ankle −0.902) | III-C |
| 15 | `tau = M(q)(q̈_des − q̈_0)` (zero-actuator ID projection) | III-A |
| 16 | `q̈_des = q̈_ref + Kp_bio(q_ref − q) + Kd_bio(q̇_ref − q̇)` | III-A |
| 17 | `min Σ a_m² + w_res Σ r_j²  s.t. moment balance` (SO QP) | III-A |
| 18 | `q̈_f = ω_n²(q_tgt − q_f) − 2ζω_n q̇_f`, ω_n=2π·6 | IV-2 |
| 19 | 3rd-order C2 jerk-limited governor (|q̇|,|q̈|,|⃛q| limits) | IV-2 |
| 20 | MDP ⟨S,A,R,γ⟩; A = absolute reference knots (pros_knee, pros_ankle) | IV-1 |
| 21 | `R = Σ w_i r_i ; r_imit = 1/(1+w·loss)` (DeepMimic-style) | IV-4 |
| 22 | gait phase `φ(t)` locked to sound-side heel strikes | IV-5 |

## Figure list
| Fig | Content | Source | Status |
|-----|---------|--------|--------|
| 1 | System block diagram: RL → governor → cascade(P→PI) → SEA → CMC body, frequencies annotated | new (anchor figure) | READY |
| 2 | SEA two-body schematic (θ_m, ω_m, ξ) | new | READY |
| 3 | Driver pole map + Bode of H(s), sim vs analytic | `plot/05_16_2026_3_driver_isolation/`, `plot/05_18_2026_motor_driver_poles/` | READY |
| 4 | Frequency ladder (reference < cascade < SEA resonance < driver) | `2026-06-14` freq analysis | READY |
| 5 | RL agent-environment + reward, actor/critic obs split + governor | new | READY |
| 6 | Governor ON/OFF: knee tau_input + saturation time-series + bar | `plot/06_12_2026_1/` | PARTIAL |
| 7 | Gait-cycle work-loop torque-angle-power, prosthetic vs AB06-healthy | `plot/.../03_gaitcycle_torque_angle_power.png` | PARTIAL (replot cleanly) |
| 8 | RL learning curves + reward decomposition, multi-seed | TensorBoard | BLOCKED |
| 9 | Learned reference vs IK overlay (learned ≠ normative) | `plot/.../07_mlp_policy_vs_sound_leg_error.png` | BLOCKED (post-fix run) |
| 10 *(opt)* | Hybrid-GRF: prosthetic SEA load + pelvis-reserve collapse | `2026-06-08` grf_ibrida | READY |

## Table list
| Tbl | Content | Status |
|-----|---------|--------|
| I | Related-work comparison (method × {actuator, RL output, low-level ctrl, bio side, engine}) | READY |
| II | SEA plant parameters (knee/ankle) | READY |
| III | Cascade gains + governor limits | READY |
| IV | OpenSim model components + AB06 mass | READY |
| V | Reward terms (name, form, weight) | READY |
| VI | Driver-isolation metrics vs analytic | READY |
| VII | **Baseline ladder B0/B1/B2 × all metrics (CENTERPIECE)** | BLOCKED |
| VIII | RL multi-seed metrics (mean±SD) | BLOCKED |
| IX | OpenSim↔MuJoCo parity | READY |

## Data-integrity checklist (resolve before drafting numbers)
- [ ] Use the LIVE plugin law (Eq. 3, with +tau_ref and +Ki·ξ), not AGENTS.md.
- [ ] Use live `.osim`/config params: knee K=321/Kp=18/Kd=11/Ki=190/F_opt=100; ankle K=500/Kp=11.3/Kd=11/Ki=123/F_opt=250.
- [ ] Never conflate actual-vs-IK RMSE with served-vs-target RMSE (different quantities).
- [ ] Fix ankle target mapping (`ankle_angle_r`→`pros_ankle_angle`) before any RL result.
- [ ] Convert all RL metrics to degrees + mean±SD over ≥3 seeds.
- [ ] Compute ≥1 symmetry/impulse metric on a final rollout.
