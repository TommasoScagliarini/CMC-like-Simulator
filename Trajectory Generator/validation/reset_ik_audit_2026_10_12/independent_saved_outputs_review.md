# Review indipendente degli output del reset IK — 2026-10-12

Verifica in sola lettura: 102 STO (17 per caso), ciascuno con 5000 righe numeriche finite e griglia comune di 1 ms. I sei episodi registrano 500 step/5 s. Il nominale riproduce byte per byte tutti i 17 STO del controllo 6 ottobre e dello storico 2 settembre. Nessuna nuova simulazione o modifica degli input.

L'intervento zero cambia esattamente quattro variabili iniziali: velocità knee/ankle protesiche e le due motor_speed SEA. Le altre variabili, la posa, le derivate IK originali e la reference servita iniziale sono identiche al nominale. È una perturbazione di uno stato di cammino, con biologia presente; non un avvio da fermo.

| Caso | Return | Δ nominale | Cicli | Invalidi | RMSE knee/ankle actual–served [°] | Pen. 1 ms [mm] |
|---|---:|---:|---:|---:|---:|---:|
| nominal | 27.423921 | +0.000000 | 2 | 0 | 0.570515/0.505732 | 24.649120 |
| trained_minus020 | 53.939081 | +26.515160 | 3 | 0 | 0.650694/0.449660 | 23.507630 |
| trained_plus020 | 38.062224 | +10.638303 | 2 | 0 | 0.681844/0.529546 | 22.444600 |
| heldout_phase_plus040 | 11.989258 | -15.434663 | 2 | 0 | 0.799378/0.572493 | 27.770900 |
| heldout_next_left_hs | -11.713349 | -39.137270 | 2 | 0 | 0.798234/0.508694 | 23.698660 |
| zero_prosthetic_velocity | 27.451473 | +0.027552 | 2 | 0 | 0.570091/0.506210 | 24.672980 |

RMSE ricalcolati con parser e somme indipendenti: accordo con le metriche registrate entro 1e-12°. Resync e HS cancellati finali sono zero in tutti i casi; i flag per-step, incluse eventuali attese soft, sono conservati nel JSON e non confusi con invalidazioni.

Ispezionati visivamente via view_image i seguenti 18 file (01 mostra SEA/controlli/reserve; la cinematica è nella 02):

- [nominal/01_time_sea_control_reserve.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/nominal/01_time_sea_control_reserve.png>)
- [nominal/02_time_joint_motor_states.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/nominal/02_time_joint_motor_states.png>)
- [nominal/07_mlp_policy_vs_sound_leg_error.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/nominal/07_mlp_policy_vs_sound_leg_error.png>)
- [trained_minus020/01_time_sea_control_reserve.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/trained_minus020/01_time_sea_control_reserve.png>)
- [trained_minus020/02_time_joint_motor_states.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/trained_minus020/02_time_joint_motor_states.png>)
- [trained_minus020/07_mlp_policy_vs_sound_leg_error.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/trained_minus020/07_mlp_policy_vs_sound_leg_error.png>)
- [trained_plus020/01_time_sea_control_reserve.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/trained_plus020/01_time_sea_control_reserve.png>)
- [trained_plus020/02_time_joint_motor_states.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/trained_plus020/02_time_joint_motor_states.png>)
- [trained_plus020/07_mlp_policy_vs_sound_leg_error.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/trained_plus020/07_mlp_policy_vs_sound_leg_error.png>)
- [heldout_phase_plus040/01_time_sea_control_reserve.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/heldout_phase_plus040/01_time_sea_control_reserve.png>)
- [heldout_phase_plus040/02_time_joint_motor_states.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/heldout_phase_plus040/02_time_joint_motor_states.png>)
- [heldout_phase_plus040/07_mlp_policy_vs_sound_leg_error.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/heldout_phase_plus040/07_mlp_policy_vs_sound_leg_error.png>)
- [heldout_next_left_hs/01_time_sea_control_reserve.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/heldout_next_left_hs/01_time_sea_control_reserve.png>)
- [heldout_next_left_hs/02_time_joint_motor_states.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/heldout_next_left_hs/02_time_joint_motor_states.png>)
- [heldout_next_left_hs/07_mlp_policy_vs_sound_leg_error.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/heldout_next_left_hs/07_mlp_policy_vs_sound_leg_error.png>)
- [zero_prosthetic_velocity/01_time_sea_control_reserve.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/zero_prosthetic_velocity/01_time_sea_control_reserve.png>)
- [zero_prosthetic_velocity/02_time_joint_motor_states.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/zero_prosthetic_velocity/02_time_joint_motor_states.png>)
- [zero_prosthetic_velocity/07_mlp_policy_vs_sound_leg_error.png](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/reset_ik_audit_2026_10_12/zero_prosthetic_velocity/07_mlp_policy_vs_sound_leg_error.png>)

**nominal:** 01/02 show repeated finite-looking control/kinematic patterns with motor-speed ripples. 07 shows substantial phase displacement from the shifted sound-leg diagnostic; it is an ex-novo run, so that trace is not an active imitation objective.

**trained_minus020:** Different initial knee pose/velocity produces a visible initial transient; repeated motion follows. 01 prosthetic reserves are flat zero. 07 diagnostic phase displacement persists.

**trained_plus020:** Initial ankle velocity/motor-speed spikes are visible, followed by repeated bounded motion over the plotted interval. 07 remains displaced from the shifted sound-leg diagnostic.

**heldout_phase_plus040:** 01 has a positive ankle-torque burst in the first portion and 02 has a sharp initial knee motor-speed transient. Later motion repeats. No broken or missing plot panel was observed; 07 diagnostic mismatch remains.

**heldout_next_left_hs:** 01 has deeper negative knee torque than nominal and 02 has repeated motor-speed ripples. 07 knee swings appear out of phase with the shifted sound-leg comparator. Completion does not resolve these shape/contact questions.

**zero_prosthetic_velocity:** 01/02/07 are visually close to nominal after the brief initial transient. Saved-data arithmetic, rather than the visual similarity, determines the measured differences.

Non è espresso un verdetto di qualificazione scientifica. Completamento, ritorno, tracking del controller, contatto e forma delle traiettorie sono criteri distinti; il confronto fra start diversi conserva anche cambiamenti di riferimento biologico e GRF prescritte. Le figure 07 confrontano una curva sound-leg diagnostica in ex-novo, senza farne un obiettivo imitativo attivo.

Tutti i file letti e i loro SHA sono elencati nel JSON; hash invariati dopo la review. Non sono state lette conclusioni Claude o la sintesi consolidata per produrre queste verifiche.
