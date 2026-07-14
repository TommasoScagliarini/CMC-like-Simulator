# Piano - Validazione reward ex-novo

Data: 2026-07-02

Instruction check token: CMC_AGENT_OK_2026

## Obiettivo

Validare la reward ex-novo prima, durante e dopo il training, separando in modo
esplicito:

- test offline, eseguibili senza allenare una rete;
- test con policy e training PPO, necessari per verificare apprendibilita' e
assenza di nuove scorciatoie.

Il criterio finale non e' solo "reward alta", ma un ordinamento robusto:

```text
camminata prescribed corretta
> progressi parziali corretti
> comportamento stabile ma inutile
> comportamento patologico o contrario alla sequenza HS-TO-HS
```

La validazione termina quando la reward premia una camminata corretta, penalizza
i controesempi principali, e un training progressivo dimostra che la policy puo'
imparare almeno la sequenza protesica `HS -> TO -> HS`.

## Perimetro e vincoli

- Tutto il lavoro reward/policy/training resta dentro `Trajectory Generator/` o
  `validation/`.
- Non modificare il plugin C++ SEA.
- Non modificare la semantica del comando SEA.
- La reward prescribed deve usare dati prescribed in tutto:
  - cinematica prescribed;
  - GRF prescribed;
  - eventi/FSM derivati dalle GRF prescribed.
- `onlineGRF` puo' restare diagnostico, ma non deve generare falsi negativi nel
  sanity check prescribed.
- Per il training deterministico iniziale partire da un HS sinistro reale:

```yaml
simulation:
  episode_start_offset_s: 1.956870983805102  # t_start 11.99 -> 13.946870983805102 s
```

## Evidenza gia' disponibile

### Prescribed misaligned, 12.99 -> 17.99

Output:

```text
validation/prescribed_reward_probe_runs/prescribed_full_12p99_17p99/
```

Risultato:

```text
ok: true
steps: 501
episode_return: 25.3309
reward_mean: 0.05056
reward_min: -1.55
grf_slip_loss_mean: 0.0
valid_hs_count: 3
valid_to_count: 2
valid_cycle_count: 2
```

Interpretazione: lo slip e' risolto, ma la finestra parte a meta' stance e
genera un timeout iniziale artificiale. Questo run non va usato come gate di
qualita' della reward, ma come test di robustezza su finestra non allineata.

### Prescribed aligned, 13.946870983805102 -> 17.99

Output:

```text
validation/prescribed_reward_probe_runs/prescribed_clean_left_hs_13p946870984_17p99/
```

Risultato:

```text
ok: true
steps: 405
episode_return: 136.289
reward_mean: 0.336516
reward_min: -0.00011
reward_max: 1.0
grf_slip_loss_mean: 0.0
phase_timeout_loss_mean: 0.0
invalid_event_count: 0
valid_hs_count: 3
valid_to_count: 2
valid_cycle_count: 2
```

Interpretazione: con prescribed GRF + prescribed kinematics, su finestra
allineata, la reward e' logicamente coerente.

## Struttura dei risultati

Ogni test deve produrre almeno:

```text
summary.json
summary.md
trace.csv
online_events.csv, se applicabile
```

Directory consigliata:

```text
validation/reward_audit_runs/<YYYY-MM-DD>_<test_id>/
```

Ogni `summary.json` deve contenere un campo sintetico:

```json
{
  "test_id": "...",
  "status": "PASS|FAIL|BLOCKED",
  "pass_criteria": {},
  "metrics": {},
  "notes": []
}
```

## Fase A - Preparazione strumenti offline

### A1 - Congelare config e contratti

Obiettivo: verificare che la config usata dai test sia quella attesa.

Comandi:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/validate_training_config.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
```

Metriche:

```text
validate_training_config: PASS
test_reward_function: PASS
reward_mode: ex_novo
blend_tracking: 0.0
blend_reference: 0.0
blend_bio: 0.0
grf_slip_weight: 0.0
morphology_weight: 0.0
```

Gate:

```text
PASS se tutti i validatori passano e i flag critici corrispondono.
FAIL se un validatore fallisce o se un termine prescribed/imitativo rientra
nel blend reward ex-novo.
```

Stato: PASS eseguito il 2026-07-03.

### A2 - Standardizzare reward audit suite

Obiettivo: creare uno script unico per lanciare i controesempi offline.

Output richiesto:

```text
validation/reward_audit_suite.py
```

Lo script deve supportare almeno:

```bash
--scenario prescribed_aligned
--scenario prescribed_long
--scenario prescribed_misaligned
--scenario static_leg
--scenario missing_to
--scenario missing_second_hs
--scenario swing_load
--scenario joint_oob
--scenario slip_injection
--scenario morphology_corridor
--scenario fake_cycle_ankle_only
--start-time <float>
--end-time <float>
--output-dir <path>
```

Gate:

```text
PASS se ogni scenario produce summary.json, summary.md e trace.csv.
PASS se lo script puo' essere eseguito senza training e senza checkpoint.
FAIL se un test richiede una policy addestrata.
```

Stato: PASS iniziale implementato il 2026-07-03 in
`validation/reward_audit_suite.py`. Gli scenari non ancora implementati
producono artefatti standardizzati con stato `BLOCKED`.

## Fase B - Test offline positivi

### B1 - Prescribed aligned: camminata corretta

Scopo: dimostrare che una camminata corretta viene premiata.

Input:

```text
start_time = 13.946870983805102
end_time   = 17.99
contract   = prescribed_pure
```

Metriche minime:

```text
ok == true
reward_mean >= 0.25
episode_return > 0
reward_min >= -0.01
grf_slip_loss_mean == 0.0
prosthetic_slip_speed_m_s_mean == 0.0
phase_timeout_loss_mean == 0.0
phase_timeout_penalty_term_mean == 0.0
invalid_event_count_final == 0
valid_hs_count_final >= 3
valid_to_count_final >= 2
valid_cycle_count_final >= 2
prosthetic_joint_range_loss_mean == 0.0
oob_term_mean == 0.0
```

Gate:

```text
PASS se tutti i criteri sono soddisfatti.
FAIL se una camminata prescribed corretta riceve reward media bassa, slip,
timeout o invalid event.
```

Stato: PASS rieseguito il 2026-07-03 con la suite:
`validation/reward_audit_runs/2026-07-03_173444_prescribed_aligned/`.

### B2 - Prescribed piu' lungo

Scopo: verificare che la reward resti coerente su una finestra piu' lunga.

Input consigliato:

```text
start_time = 13.946870983805102
end_time   = 21.0
contract   = prescribed_pure
```

Metriche:

```text
reward_mean >= 0.20
episode_return > 0
grf_slip_loss_mean == 0.0
phase_timeout_loss_mean == 0.0
invalid_event_count_final == 0
valid_cycle_count_final >= 4
last_period_s in [0.90, 2.20]
last_stance_fraction in [0.50, 0.80]
```

Gate:

```text
PASS se la reward resta positiva e la FSM chiude piu' cicli senza timeout.
FAIL se emergono timeout/eventi invalidi su cicli prescribed completi.
```

Stato: PASS eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_prescribed_long_13p946870984_21p0/`.
Metriche chiave: `reward_mean == 0.3581`, `valid_cycle_count_final == 4`,
`invalid_event_count_final == 0`, `phase_timeout_loss_mean == 0.0`,
`last_period_s == 1.571`, `last_stance_fraction == 0.675`.

### B3 - Finestra misaligned come controllo di bordo

Scopo: documentare il comportamento su partenza a meta' fase.

Input:

```text
start_time = 12.99
end_time   = 17.99
contract   = prescribed_pure
```

Metriche:

```text
grf_slip_loss_mean == 0.0
valid_cycle_count_final >= 2
reward_mean < reward_mean_prescribed_aligned
phase_timeout_penalty_term_mean > 0.0 oppure note=edge_start
```

Gate:

```text
PASS se lo script identifica il caso come edge_start e non lo usa come gate
principale della reward.
FAIL se il test viene interpretato come fallimento fisiologico della camminata.
```

Stato: PASS diagnostico gia' ottenuto.

## Fase C - Test offline negativi / controesempi

### C1 - Gamba protesica ferma

Scopo: verificare che una configurazione stabile ma inutile non sia premiata.

Scenario:

```text
static_leg: knee/ankle protesici costanti, GRF/eventi prescribed o coerenti
con mancato ciclo protesico.
```

Metriche:

```text
reward_mean <= 0.10
valid_cycle_count_final == 0
phase_timeout_penalty_term_mean > 0.0
phase_event_progress_score_mean <= 0.05
contact_load_score_mean non sufficiente a compensare il timeout
```

Gate:

```text
PASS se la reward resta bassa o negativa.
FAIL se la gamba ferma ottiene reward comparabile al prescribed aligned.
```

Stato: PASS sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/static_leg/`.
Metriche chiave: `reward_mean == -0.4656`,
`valid_cycle_count_final == 0`, `phase_timeout_penalty_term_mean > 0`.

### C2 - Missing TO

Scopo: penalizzare una stance che non produce toe-off.

Scenario:

```text
missing_to: mascherare o ritardare il TO sinistro oltre il timeout.
```

Metriche:

```text
valid_hs_count_final >= 1
valid_to_count_final == 0
valid_cycle_count_final == 0
phase_stance_timeout_loss_mean > 0.0
phase_timeout_penalty_term_mean > 0.0
reward_mean <= 0.15
reward_min < 0.0
```

Gate:

```text
PASS se il mancato TO e' chiaramente peggiore del prescribed aligned.
FAIL se la policy potrebbe restare in stance e ottenere reward alta.
```

Stato: PASS sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/missing_to/`.
Metriche chiave: `reward_mean == -0.2675`,
`valid_to_count_final == 0`, `valid_cycle_count_final == 0`,
`phase_stance_timeout_loss_mean == 1.075`.

### C3 - Missing second HS

Scopo: penalizzare la sequenza `HS -> TO` senza landing successiva.

Scenario:

```text
missing_second_hs: mantenere il TO ma rimuovere/ritardare il secondo HS.
```

Metriche:

```text
valid_hs_count_final == 1
valid_to_count_final >= 1
valid_cycle_count_final == 0
phase_swing_timeout_loss_mean > 0.0
phase_timeout_penalty_term_mean > 0.0
landing_window_contact_score_mean basso
reward_mean <= 0.05
reward_min < 0.0
```

Gate:

```text
PASS se lo swing senza landing diventa molto meno conveniente del ciclo chiuso.
FAIL se `HS -> TO` senza nuovo HS resta competitivo.
```

Stato: PASS sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/missing_second_hs/`.
Metriche chiave: `reward_mean == -0.0344`, `valid_hs_count_final == 1`,
`valid_to_count_final == 1`, `valid_cycle_count_final == 0`,
`phase_swing_timeout_loss_mean == 0.2545`.

### C4 - Carico in swing

Scopo: verificare la penalizzazione del carico protesico durante swing.

Scenario:

```text
swing_load: inserire forza verticale prescribed sulla protesi durante swing.
```

Metriche:

```text
swing_unloading_loss_mean > 0.0
contact_load_score_mean non deve compensare il carico errato
reward_mean <= reward_mean_prescribed_aligned - 0.15
```

Gate:

```text
PASS se il carico in swing riduce chiaramente la reward.
FAIL se il carico in swing viene premiato come supporto utile.
```

Stato: PASS sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/swing_load/`.
Metriche chiave: `reward_mean == 0.1531`,
`swing_unloading_loss_mean == 0.6721`, sotto il margine rispetto al
prescribed aligned.

### C5 - Fuori range articolare

Scopo: verificare che posture protesiche non fisiologiche siano penalizzate.

Scenario:

```text
joint_oob: perturbare knee/ankle oltre `prosthetic_joint_q_min/max`.
```

Metriche:

```text
prosthetic_joint_range_loss_mean > 0.0
oob_loss_mean > 0.0 se il comando/reference e' fuori banda
reward_mean <= reward_mean_prescribed_aligned - 0.15
```

Gate:

```text
PASS se il fuori range e' separato dal prescribed aligned.
FAIL se la reward lascia competitivo un ginocchio/caviglia fuori banda.
```

Stato: PASS sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/joint_oob/`.
Metriche chiave: `reward_mean == -2.1010`,
`prosthetic_joint_range_loss_mean == 1.0`, `oob_loss_mean > 0`.

### C6 - Slip injection

Scopo: testare il termine slip senza confonderlo con i dati prescribed reali.

Scenario:

```text
slip_injection: forzare `prosthetic_slip_speed_m_s > 0` in input reward.
```

Due sotto-test:

```text
C6a: config attuale con grf_slip_weight = 0.0
C6b: config temporanea diagnostica con grf_slip_weight > 0.0
```

Metriche:

```text
C6a: grf_slip_loss diagnostico puo' essere > 0, ma grf_slip_term == 0
C6a: reward_without_grf_slip == reward
C6b: grf_slip_term > 0
C6b: reward_mean <= reward_mean_C6a - 0.10
```

Gate:

```text
PASS se lo slip e' spento nella config attuale ma funziona quando abilitato
esplicitamente.
FAIL se lo slip penalizza ancora il sanity check prescribed.
```

Stato: PASS diagnostico sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/slip_injection/`.
Metriche chiave: con config attuale `grf_slip_term == 0`; con
`grf_slip_weight = 1.0` diagnostico, `reward_mean` scende da `0.0` a `-0.25`.

### C7 - Morphology corridor diagnostico

Scopo: verificare che il corridor sia pronto anche se ora ha peso zero.

Due sotto-test:

```text
C7a: prescribed aligned con morphology_weight = 0.0
C7b: prescribed aligned con morphology_weight temporaneo > 0
C7c: perturbazione fuori corridor con morphology_weight temporaneo > 0
```

Metriche:

```text
C7a: morphology_term == 0
C7a: morphology_loss tracciato e finito
C7b: morphology_loss_mean basso su prescribed aligned
C7c: morphology_loss_mean > C7b + margine
C7c: reward_mean < reward_mean_C7b
```

Gate:

```text
PASS se il corridor e' diagnostico nella config attuale e penalizzante solo
quando viene attivato.
FAIL se il corridor prescribed aligned produce falsi positivi forti.
```

Stato: PASS diagnostico sintetico eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/morphology_corridor/`.
Nota: verifica solo che `morphology_weight == 0.0` sia non penalizzante e che
un peso temporaneo positivo penalizzi una loss alta. Non sostituisce una futura
validazione prescribed/OpenSim del corridor con cinematica perturbata.

### C8 - Fake gait cycle ankle-only

Scopo: verificare esplicitamente la scappatoia osservata il 2026-07-03: la
policy tiene il ginocchio quasi fermo, fa oscillare soprattutto la caviglia e
genera eventi online `HS -> TO -> HS` che la vecchia FSM contava come gait
cycle validi pur senza camminata reale.

Scenario:

```text
fake_cycle_ankle_only:
  knee protesico quasi statico;
  ankle protesica oscillante;
  eventi GRF/eventi online tali da produrre una sequenza HS -> TO -> HS;
  carico stance insufficiente o non coerente con supporto reale.
```

Metriche:

```text
phase_cycle_knee_excursion_rad < phase_min_cycle_knee_excursion_rad
phase_stance_contact_fraction < phase_min_stance_contact_fraction
  oppure phase_stance_load_integral_bw_s < phase_min_stance_load_bw_s
phase_cycle_rejected_this_step > 0 almeno una volta
valid_cycle_count_final == 0
phase_cycle_complete_bonus_mean == 0.0
phase_regular_score_mean basso o nullo
reward_mean <= 0.05
```

Gate:

```text
PASS se la sequenza HS -> TO -> HS fasulla viene rifiutata dalla FSM e non
ottiene bonus di ciclo completo.
FAIL se la reward/FSM conta un ciclo valido con ginocchio statico e solo
oscillazione di caviglia.
```

Stato: PASS eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/fake_cycle_ankle_only/`.
Metriche chiave: `valid_cycle_count_final == 0`,
`cycle_complete_bonus_max == 0.0`, `cycle_rejected_max == 1.0`,
`loaded_knee_excursion_max_rad == 0.015`, `reward_mean == 0.10`.

## Fase D - Matrice di separazione offline

Obiettivo: confrontare tutti gli scenari con una tabella unica.

Output richiesto:

```text
validation/reward_audit_runs/<date>_matrix/reward_separation_matrix.csv
validation/reward_audit_runs/<date>_matrix/reward_separation_matrix.md
```

Metriche minime per scenario:

```text
test_id
status
reward_mean
reward_min
episode_return
valid_hs_count
valid_to_count
valid_cycle_count
phase_timeout_penalty_term_mean
invalid_event_count_final
contact_load_score_mean
swing_unloading_loss_mean
grf_slip_loss_mean
prosthetic_joint_range_loss_mean
oob_loss_mean
morphology_loss_mean
phase_cycle_rejected_this_step_count
phase_cycle_knee_excursion_rad_min
phase_stance_contact_fraction_mean
phase_stance_load_integral_bw_s_mean
```

Gate globale offline:

```text
PASS se:
- prescribed_aligned e prescribed_long sono PASS;
- static_leg, missing_to, missing_second_hs, swing_load, joint_oob e
  fake_cycle_ankle_only sono chiaramente peggiori del prescribed aligned;
- nessun controesempio patologico ha reward_mean >= 70% del prescribed aligned;
- nessun controesempio patologico chiude valid_cycle_count comparabile al
  prescribed aligned;
- il caso fake_cycle_ankle_only produce almeno un reject diagnostico e non
  riceve `phase_cycle_complete_bonus`;
- slip e morphology non generano falsi negativi nella config attuale.
```

Stato: PASS iniziale eseguito il 2026-07-03:
`validation/reward_audit_runs/2026-07-03_reward_separation_matrix.md`.
Tutti i controesempi negativi sintetici sono sotto il 70% del prescribed
aligned; `prescribed_aligned` e `prescribed_long` restano positivi.

## Fase E - Training readiness gate

Il training puo' partire solo se:

```text
A1 PASS
B1 PASS
B2 PASS
C1 PASS
C2 PASS
C3 PASS
C4 PASS
C5 PASS
D PASS
```

Eccezioni ammesse:

```text
C6 e C7 possono restare diagnostici se grf_slip_weight == 0.0 e
morphology_weight == 0.0 nella config di training.
```

Se il gate fallisce:

```text
non lanciare training lungo;
correggere pesi/logica reward;
ripetere solo i test offline impattati.
```

## Fase F - Test con policy senza training

### F1 - Rollout policy random/untrained

Scopo: misurare il livello base da cui parte PPO.

Input:

```text
config ex-novo attuale
episode_start_offset_s = 1.956870983805102
episode_duration = 5.0
policy random o checkpoint non addestrato
```

Metriche:

```text
reward_mean_random
episode_return_random
valid_cycle_count_random
phase_timeout_penalty_term_mean_random
terminated/truncated reason
nan_count == 0
```

Gate:

```text
PASS se random e' peggiore del prescribed aligned e non produce NaN/crash.
FAIL se random ottiene reward simile al prescribed aligned.
```

Stato: PASS eseguito il 2026-07-03:
`validation/reward_policy_runs/2026-07-03_random_policy_baseline/`.
Metriche chiave: `reward_mean == -0.2085`, `episode_return == -2.0848`,
`valid_cycle_count_final == 0`, `nan_count == 0`, terminazione per
`grf_penetration`.

### F2 - Rollout checkpoint precedente fallito

Scopo: verificare che la reward attuale separi il vecchio failure mode.

Input:

```text
ultimo/best checkpoint del training 100 iter precedente, se disponibile.
```

Metriche:

```text
valid_hs_count
valid_to_count
valid_cycle_count
phase_timeout_penalty_term_mean
reward_mean_failed_checkpoint
prosthetic_joint_range_loss_mean
contact_load_score_mean
```

Gate:

```text
PASS se il failure mode `HS -> TO -> no second HS` riceve reward chiaramente
inferiore al prescribed aligned.
FAIL se il vecchio checkpoint resta competitivo.
```

Stato: PASS eseguito il 2026-07-03:
`Trajectory Generator/runs/rollout/2026-07-03_old_100iter_checkpoint_current_reward/`.
Il vecchio checkpoint 100 iter, rivalutato con `--no-auto-config` e la reward
attuale, ha `reward_mean == -0.0246` ed e' quindi non competitivo rispetto al
prescribed aligned.

## Fase G - Smoke training

### G1 - Training 2 iterazioni

Scopo: validare infrastruttura, logging e checkpoint senza cercare performance.

Config:

```yaml
simulation:
  iterations: 2
  episode_start_offset_s: 1.956870983805102
  episode_duration: 5.0
```

Metriche:

```text
training_exit_code == 0
checkpoint_last esiste
checkpoint_best esiste oppure best metric registrata
loss finite
nan_count == 0
sample_timeout == false
iteration_timeout == false
```

Gate:

```text
PASS se il training completa e il checkpoint e' caricabile.
FAIL se crasha, produce NaN o non salva checkpoint.
```

Stato: PASS eseguito il 2026-07-03:
`Trajectory Generator/runs/training/2026-07-03_smoke2_reward_validation/`.
Metriche chiave: `iterations_completed == 2`, `timed_out == false`,
`best_episode_return_mean == -2.663`, `checkpoint_last`, `checkpoint_best`,
`rl_module_last` e `rl_module_best` presenti.

### G2 - Rollout post-smoke

Scopo: verificare che un checkpoint appena prodotto sia valutabile.

Metriche:

```text
rollout_exit_code == 0
summary.json esiste
trace.csv esiste
reward terms completi
FSM terms completi
nan_count == 0
```

Gate:

```text
PASS se rollout e report vengono prodotti.
FAIL se il checkpoint non e' caricabile o il rollout non genera diagnostica.
```

Stato: PASS eseguito il 2026-07-03:
`Trajectory Generator/runs/rollout/2026-07-03_smoke2_reward_validation_rollout_recorded/`.
Nota: lo strumento genera `rollout_policy_trace.json`, non `trace.csv`.
Metriche chiave: `ok == true`, `steps == 13`, `reward_mean == -0.2074`,
`record_outputs == true`, output `.sto` e `rollout_policy_trace.json`
prodotti.

## Fase H - Training diagnostico breve

### H1 - Training 10 iterazioni

Scopo: verificare apprendibilita' iniziale e direzione del gradiente.

Config:

```yaml
simulation:
  iterations: 10
  episode_start_offset_s: 1.956870983805102
  episode_duration: 5.0
```

Metriche da TensorBoard / progress / rollout best:

```text
episode_return_mean non collassa
reward_mean_best > reward_mean_random
phase_valid_hs_count_best >= 1
phase_valid_to_count_best >= 1 oppure trend crescente
phase_timeout_penalty_term_best <= random baseline
prosthetic_joint_range_loss_best == 0 oppure in diminuzione
policy_action_clip_fraction_best basso
nan_count == 0
```

Gate:

```text
PASS se il best checkpoint migliora random e mostra progresso FSM.
FAIL se la policy torna a stabilizzare la gamba senza TO/HS e senza penalita'
sufficiente.
```

Stato: FAIL diagnostico eseguito il 2026-07-03:
`Trajectory Generator/runs/training/2026-07-03_diag10_reward_validation/`.
La pipeline di training e' funzionante (`ok == true`, `iterations_completed == 10`,
`timed_out == false`, checkpoint best/last e moduli RLlib prodotti), ma il gate
di apprendibilita' non e' superato. Il best `episode_return_mean` resta negativo
(`-2.4803`, iterazione 9), l'ultima iterazione resta negativa (`-2.6836`) e le
terminazioni per `grf_penetration` dominano la raccolta (`23` a fine run).
Esito operativo: infrastruttura PASS, learnability FAIL; non procedere a
training 20-50/100 iterazioni senza prima analisi del rollout best.

Rerun dopo target slew limiter eseguito il 2026-07-07:
`Trajectory Generator/runs/training/2026-07-07_H1_diag10_reward_validation_limiter/`.
Il training completa regolarmente 10/10 iterazioni (`ok == true`,
`timed_out == false`) con best checkpoint all'iterazione 10. Il return medio
migliora in modo monotono da `1.3801` a `1.8265` e non compaiono NaN nelle
metriche PPO principali. La lunghezza episodio resta corta (`31.51 -> 34.31`
step medi) e le terminazioni cumulative per `grf_penetration` restano dominanti
(`1250` a fine run). Esito operativo: infrastruttura PASS e gradiente/reward
trend migliorato; validazione comportamentale ancora vincolata al nuovo H2 sul
best checkpoint.

Rerun dopo gate del premio di contatto sulla penetrazione eseguito il
2026-07-08:
`Trajectory Generator/runs/training/2026-07-08_H1_contact_penetration_gate/`.
Il training completa regolarmente 10/10 iterazioni (`ok == true`) con best
checkpoint all'iterazione 10. Il return medio parte negativo e diventa positivo
solo nelle ultime iterazioni (`-0.0633 -> 0.1410`), come atteso dopo aver spento
il premio del carico penetrato. La lunghezza media aumenta poco (`31.51 ->
33.40` step) e le terminazioni per `grf_penetration` restano dominanti
(`1265` cumulative a fine run). Esito operativo: infrastruttura PASS e reward
piu' selettiva, ma gate comportamentale ancora da H2.

### H2 - Rollout best 10 iterazioni

Scopo: leggere il comportamento effettivo, non solo la metrica PPO.

Metriche:

```text
event_sequence
time_HS_to_TO
time_TO_to_HS
valid_cycle_count
phase_timeout_side
phase_timeout_penalty_term_mean
contact_load_score_mean
swing_unloading_loss_mean
prosthetic_joint_range_loss_mean
reward_mean
reward_min
```

Gate:

```text
PASS se compare almeno una sequenza coerente parziale `HS -> TO` e la reward
del rollout e' superiore alla random baseline.
FAIL se nessun evento utile compare e la reward resta ottenuta da contatto
statico.
```

Stato: FAIL diagnostico eseguito il 2026-07-03:
`Trajectory Generator/runs/rollout/2026-07-03_diag10_reward_validation_best_rollout/`.
Il checkpoint best e' caricabile e produce diagnostica completa, ma non mostra
progresso FSM sufficiente: `steps == 10`, `episode_return == -2.3826`,
`reward_mean == -0.2383`, `terminated == true`, `truncated == false`,
`action_clipped_fraction == 0.0`. Gli eventi online contengono solo gli
heel-strike iniziali left/right confermati a `13.997870983805074 s`; non compare
nessun toe-off. Nel trace il massimo/finale e' `phase_valid_hs_count == 1`,
`phase_valid_to_count == 0`, `phase_valid_cycle_count == 0`, con terminazione
su `grf_penetration_loss == 1.4031`. Esito operativo: rollout caricabile PASS,
gate comportamentale FAIL.

Rerun dopo target slew limiter eseguito il 2026-07-07:
`Trajectory Generator/runs/rollout/2026-07-07_H2_diag10_reward_validation_limiter_recorded/`.
Il limiter risolve la terminazione immediata del vecchio H2 (`steps` passa da
10 a 27 e la reward media diventa positiva: `episode_return == 0.9166`,
`reward_mean == 0.0339`), ma il gate resta FAIL. Gli eventi online mostrano
heel-strike iniziale left/right e un `toe_off` solo sul lato right; non compare
un `toe_off` valido della protesi/left. Nel trace il massimo/finale resta
`phase_valid_hs_count == 1`, `phase_valid_to_count == 0`,
`phase_valid_cycle_count == 0`. La terminazione avviene ancora per
`grf_penetration` al passo 27 (`grf_penetration_m == 0.01745`,
`grf_penetration_loss == 1.1881`, `safety_loss == 1.0`). I termini
`target_slew_*` confermano che il comando raw e' continuamente limitato
(`target_slew_limited_fraction` spesso `1.0`, `served_delta == 0.025 rad`).
Esito operativo: target jump mitigato, ma H2 non sbloccato; serve nuova analisi
su mancato scarico/TO protesico e penetrazione progressiva in stance.

Rerun sul best del nuovo H1 con target slew limiter eseguito il 2026-07-08:
`Trajectory Generator/runs/rollout/2026-07-08_H2_H1_limiter_best_recorded/`.
Il rollout e' tecnicamente valido (`ok == true`, `record_outputs == true`) e
migliora la durata rispetto al vecchio H2 (`steps == 45`, `episode_return ==
3.7272`, `reward_mean == 0.0828`). Il gate comportamentale pero' resta FAIL:
gli eventi online contengono HS left/right iniziali e un solo TO destro, ma
nessun TO left/protesico. Nel trace finale `phase_valid_hs_count == 1`,
`phase_valid_to_count == 0`, `phase_valid_cycle_count == 0`; la terminazione e'
ancora `grf_penetration`/safety con `grf_penetration_m == 0.01714` e
`grf_penetration_loss == 1.05794`. Esito operativo: non procedere a 20-50/100
iter; serve revisione reward/dinamica early-stance per impedire il caricamento
progressivo senza scarico/TO.

Rerun sul best del nuovo H1 dopo gate del premio di contatto sulla penetrazione
eseguito il 2026-07-08:
`Trajectory Generator/runs/rollout/2026-07-08_H2_contact_penetration_gate_best_recorded/`.
Il rollout e' tecnicamente valido (`ok == true`, `record_outputs == true`) ma il
gate comportamentale resta FAIL: `steps == 36`, `episode_return == 0.8193`,
`reward_mean == 0.02276`, `terminated == true`, `truncated == false`. Gli eventi
online contengono HS left/right iniziali e un solo TO destro; non compare nessun
TO left/protesico. Nel trace finale la FSM e' ancora in `STANCE_AFTER_HS`
(`phase_fsm_state_id == 1`, `phase_valid_hs_count == 1`,
`phase_valid_to_count == 0`, `phase_valid_cycle_count == 0`). La modifica reward
funziona: a `10.757 mm` la qualita' del premio scende a `0.622`, a `13.007 mm`
`contact_load_score == 0.0` nonostante `contact_load_raw_score == 0.703`, e
all'ultimo step `reward_base == 0.0` mentre la terminazione e' data da
`safety_term == 2.0` + `grf_penetration_term == 0.611`. Esito operativo: la
scappatoia reward del carico penetrato e' chiusa, ma la policy/dinamica resta
bloccata in stance compressiva senza scarico/TO; non procedere a 20-50/100 iter.

## Fase I - Training intermedio

### I1 - Training 20-50 iterazioni

Scopo: verificare se la reward guida verso cicli completi.

Metriche:

```text
reward_mean_best > reward_mean_10iter_best
valid_cycle_count_best >= 1
valid_hs_count_best >= 2
valid_to_count_best >= 1
phase_timeout_penalty_term_mean in diminuzione
phase_regular_score_mean in aumento
contact_load_score_mean non nullo
swing_unloading_loss_mean controllato
prosthetic_joint_range_loss_mean == 0 o basso
```

Gate:

```text
PASS se il best rollout produce almeno un ciclo `HS -> TO -> HS`.
FAIL se dopo 20-50 iterazioni resta il failure mode senza secondo HS.
```

Stato: BLOCCATO da H1/H2. Non eseguire finche' il best 10 iter non mostra almeno
una transizione `HS -> TO` senza terminazione precoce per `grf_penetration`.

### I2 - Analisi failure mode se I1 fallisce

Scopo: capire quale termine reward non separa abbastanza.

Diagnostiche obbligatorie:

```text
reward component breakdown
event timeline
GRF/contact timeline
joint range timeline
served reference vs actual q
FSM state timeline
comparison vs offline matrix
```

Gate:

```text
PASS se viene identificata una causa dominante e proposta una modifica mirata.
BLOCKED se non e' possibile distinguere bug, pesi reward o limiti dinamici.
```

Stato: DA ESEGUIRE solo se necessario.

## Fase J - Training lungo

### J1 - Training 100 iterazioni

Scopo: validare la config candidata su scala comparabile ai run precedenti.

Prerequisito:

```text
Fasi A-D PASS
Fase G PASS
Fase H PASS
Fase I PASS o modifica reward documentata e rivalidata offline
```

Metriche minime:

```text
training completato
best checkpoint caricabile
reward_mean_best stabile o crescente
valid_cycle_count_best >= 2 su rollout da 5 s
valid_hs_count_best >= 3
valid_to_count_best >= 2
phase_timeout_penalty_term_mean == 0 o molto basso
phase_regular_score_mean migliorato rispetto a 20-50 iter
prosthetic_joint_range_loss_mean == 0 o basso
policy_action_clip_fraction basso
nessun NaN/Inf
```

Gate:

```text
PASS se il best rollout chiude almeno due cicli e non usa scorciatoie evidenti.
FAIL se ricompare stabilizzazione statica, no-second-HS, out-of-range o
contatto patologico con reward alta.
```

Stato: BLOCCATO da H1/H2. L'analisi e' necessaria prima di aumentare il budget
di training.

### J2 - Rollout e plot standard

Scopo: produrre evidenza leggibile e confrontabile.

Output:

```text
rollout summary.json
trace.csv
event timeline
plot standard in stile plot/
report in reports/user/
```

Metriche:

```text
plot generati
summary/report generati
event sequence leggibile
reward components leggibili
```

Gate:

```text
PASS se i plot permettono di verificare visualmente HS, TO, HS, GRF e joint
ranges.
FAIL se mancano segnali chiave o i plot sono ambigui.
```

Stato: DA ESEGUIRE.

## Fase K - Decisione finale

La reward/config viene promossa a baseline candidata solo se:

```text
offline prescribed aligned PASS
offline negative scenarios PASS
offline fake_cycle_ankle_only PASS
random/untrained baseline chiaramente peggiore PASS
smoke training PASS
10 iter training PASS
20-50 iter training PASS
100 iter training PASS
rollout/plot/report finali PASS
```

Decisioni possibili:

```text
PROMOTE:
  la reward e' coerente e allenabile; usare come baseline ex-novo.

REVISE_REWARD:
  offline PASS ma training FAIL; rivedere pesi o termini di shaping.

REVISE_LOGIC:
  offline FAIL; correggere FSM, eventi, input GRF o implementazione reward.

BLOCKED_DYNAMIC:
  reward coerente ma dinamica/SEA/OpenSim impedisce il comportamento richiesto;
  aprire analisi separata fuori dalla reward.
```

## Registro esecuzione

| Fase | Test | Stato | Criterio principale | Output |
| --- | --- | --- | --- | --- |
| A1 | Config validators | PASS | validatori PASS | `validation/` |
| A2 | Reward audit suite | PASS iniziale | script unico pronto | `validation/reward_audit_suite.py` |
| B1 | Prescribed aligned | PASS | reward positiva, slip 0, FSM corretta | `validation/reward_audit_runs/2026-07-03_173444_prescribed_aligned/` |
| B2 | Prescribed lungo | PASS | >= 4 cicli senza timeout | `validation/reward_audit_runs/2026-07-03_prescribed_long_13p946870984_21p0/` |
| B3 | Prescribed misaligned | PASS diagnostico | edge start documentato | `prescribed_full_12p99_17p99/` |
| C1 | Static leg | PASS sintetico | reward bassa, timeout alto | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/static_leg/` |
| C2 | Missing TO | PASS sintetico | stance timeout penalizzato | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/missing_to/` |
| C3 | Missing second HS | PASS sintetico | swing timeout penalizzato | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/missing_second_hs/` |
| C4 | Swing load | PASS sintetico | carico in swing penalizzato | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/swing_load/` |
| C5 | Joint OOB | PASS sintetico | range loss > 0, reward ridotta | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/joint_oob/` |
| C6 | Slip injection | PASS diagnostico | spento ora, attivo solo se abilitato | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/slip_injection/` |
| C7 | Morphology corridor | PASS diagnostico sintetico | diagnostico ora, penalizzante se abilitato | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/morphology_corridor/` |
| C8 | Fake gait cycle ankle-only | PASS | HS-TO-HS fasullo rifiutato, no cycle bonus | `validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/fake_cycle_ankle_only/` |
| D | Separation matrix | PASS iniziale | patologici < 70% prescribed aligned | `validation/reward_audit_runs/2026-07-03_reward_separation_matrix.*` |
| F1 | Random/untrained rollout | PASS | peggiore del prescribed aligned | `validation/reward_policy_runs/2026-07-03_random_policy_baseline/` |
| F2 | Old failed checkpoint | PASS | failure mode penalizzato | `Trajectory Generator/runs/rollout/2026-07-03_old_100iter_checkpoint_current_reward/` |
| G1 | Training 2 iter | PASS | completa, no NaN, checkpoint | `Trajectory Generator/runs/training/2026-07-03_smoke2_reward_validation/` |
| G2 | Rollout smoke checkpoint | PASS | checkpoint caricabile | `Trajectory Generator/runs/rollout/2026-07-03_smoke2_reward_validation_rollout_recorded/` |
| H1 | Training 10 iter | PASS tecnico / H2 richiesto | contact gate produce return positivo finale, ma episodi corti e `grf_penetration` ancora dominante | `Trajectory Generator/runs/training/2026-07-08_H1_contact_penetration_gate/` |
| H2 | Rollout best 10 iter | FAIL diagnostico | gate contatto-penetrazione funziona, ma no left TO/cycle e termina `grf_penetration` a 36 step | `Trajectory Generator/runs/rollout/2026-07-08_H2_contact_penetration_gate_best_recorded/` |
| I1 | Training 20-50 iter | BLOCCATO | richiede H2 PASS | run dir |
| I2 | Failure analysis | TODO | causa identificata | report |
| J1 | Training 100 iter | BLOCCATO | richiede 20-50 iter PASS | run dir |
| J2 | Plot/report finali | BLOCCATO | richiede rollout valido | plot/report |
| K | Decisione finale | REVISE_REWARD/BLOCKED_DYNAMIC | offline PASS, training diagnostico FAIL | report finale |
