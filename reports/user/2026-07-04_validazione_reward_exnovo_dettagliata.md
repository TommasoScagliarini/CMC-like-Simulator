# Validazione reward ex-novo dopo rilevazione gate cycle fasullo

Data report: 2026-07-04

## Sintesi

E' stata avviata una validazione estesa della reward ex-novo dopo
l'osservazione visuale di un failure mode: la policy del rollout da 100
iterazioni sembrava ottenere cicli di gait validi senza eseguire un vero ciclo
biomeccanico. In particolare il ginocchio protesico tendeva a rimanere quasi
fermo mentre la caviglia oscillava; questa oscillazione poteva produrre una
sequenza apparente `heel strike -> toe off -> heel strike` accettata dalla FSM e
premiata come ciclo valido.

La correzione logica ha reso la reward/FSM piu' selettiva: il ciclo non e' piu'
validato solo dall'ordine temporale degli eventi, ma richiede anche contatto e
carico in stance, durata minima e una escursione minima del ginocchio durante il
ciclo. Le validazioni offline sono passate, incluso il controesempio specifico
`fake_cycle_ankle_only`. Tuttavia il training diagnostico da 10 iterazioni non
ha superato il gate comportamentale: il checkpoint migliore non arriva a
`toe_off`, non chiude cicli e termina precocemente per `grf_penetration`.

Decisione operativa: la reward attuale non va promossa a baseline lunga. Le
fasi di training 20-50 e 100 iterazioni restano bloccate finche' il rollout
diagnostico non mostra almeno una transizione coerente `HS -> TO` senza
terminazione precoce.

## Problema

Il rollout del checkpoint:

```text
Trajectory Generator/runs/training/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter/rl_module_best
```

produceva metriche apparentemente positive, ma visualizer e grafici indicavano
un comportamento non valido. La reward/FSM contava cicli validi anche quando il
movimento era dominato da oscillazione della caviglia e ginocchio quasi statico.

Il problema tecnico individuato era nella validazione del ciclo:

```text
HS -> TO -> HS
```

era sufficiente rispettare ordine eventi e durate minime, ma non era richiesto
che il ciclo avesse:

- stance realmente caricata;
- contatto sufficiente durante stance;
- escursione minima del ginocchio;
- rifiuto esplicito del ciclo ankle-only.

Questo permetteva alla policy di sfruttare una scorciatoia della macchina a
stati, ottenendo credito di fase senza produrre un gait cycle plausibile.

## Strategia adottata

La validazione e' stata strutturata in tre livelli.

1. Correzione della logica FSM/reward per impedire cicli fasulli.
2. Audit offline con scenari prescritti e controesempi sintetici.
3. Verifica di allenabilita' con random baseline, vecchio checkpoint fallito,
   smoke training e training diagnostico breve.

Il principio usato e' stato separare due domande:

- la reward discrimina correttamente un ciclo valido da failure mode sintetici?
- la reward rimane allenabile in OpenSim/RLlib senza convergere subito a una
  terminazione patologica?

La prima domanda ha esito positivo. La seconda, al momento, ha esito negativo.

## Modifiche principali

### FSM protesica

File:

```text
Trajectory Generator/prosthetic_phase_fsm.py
```

La FSM e' stata estesa per tracciare e validare:

- frazione di stance con contatto;
- integrale del carico in stance in BW*s;
- escursione del ginocchio durante il ciclo;
- rifiuto del ciclo quando i requisiti minimi non sono soddisfatti;
- diagnostiche `phase_cycle_rejected_this_step`,
  `phase_stance_contact_fraction`, `phase_stance_load_integral_bw_s`,
  `phase_cycle_knee_excursion_rad` e segnali collegati.

Nuovi gate configurabili:

```yaml
phase_min_stance_contact_fraction: 0.20
phase_min_stance_load_bw_s: 0.04
phase_min_cycle_knee_excursion_rad: 0.12
```

### Reward e ambiente

File:

```text
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

La reward ora riceve ed espone i nuovi termini diagnostici della FSM. Il ciclo
completo non riceve bonus se non supera i requisiti minimi di contatto/carico e
cinematica del ginocchio. Il logging TensorBoard e i wrapper di ambiente sono
stati aggiornati per propagare i nuovi campi.

### Suite di validazione

File nuovi/aggiornati:

```text
validation/reward_audit_suite.py
validation/random_policy_reward_probe.py
validation/test_reward_function.py
validation/validate_training_config.py
reports/plans/2026-07-02_piano_validazione_reward_exnovo.md
```

La suite `reward_audit_suite.py` esegue scenari prescritti e sintetici:

```text
prescribed_aligned
prescribed_long
prescribed_misaligned
static_leg
missing_to
missing_second_hs
swing_load
joint_oob
slip_injection
morphology_corridor
fake_cycle_ankle_only
```

Il probe `random_policy_reward_probe.py` crea una baseline random/untrained
confrontabile con gli scenari positivi prescritti.

## Risultati offline

### Prescribed aligned

Output:

```text
validation/reward_audit_runs/2026-07-03_173444_prescribed_aligned/
```

Metriche:

```text
status: PASS
reward_mean: 0.33651620954243927
episode_return: 136.2890648646879
valid_hs: 3
valid_to: 2
valid_cycle: 2
invalid_event_count: 0
phase_timeout_loss: 0
grf_slip_loss: 0
```

Interpretazione: un replay positivo allineato continua a ricevere reward
positiva e chiude cicli validi.

### Prescribed lungo

Output:

```text
validation/reward_audit_runs/2026-07-03_prescribed_long_13p946870984_21p0/
```

Metriche:

```text
status: PASS
reward_mean: 0.35809702780398467
episode_return: 252.81650162961316
valid_hs: 5
valid_to: 4
valid_cycle: 4
invalid_event_count: 0
phase_timeout_loss: 0
last_period_s: 1.5709874864018296
last_stance_fraction: 0.6751950880539738
```

Interpretazione: la reward non penalizza indebitamente un replay multi-ciclo
coerente.

### Matrice di separazione

Output:

```text
validation/reward_audit_runs/2026-07-03_reward_separation_matrix.md
validation/reward_audit_runs/2026-07-03_reward_separation_matrix.csv
```

Risultato:

```text
status: PASS
prescribed_aligned reward_mean: 0.336516
soglia 70%: 0.235561
```

Tutti i controesempi negativi sono sotto il 70% del prescribed aligned.

Metriche principali:

```text
static_leg: reward_mean -0.465625, valid_cycle 0
missing_to: reward_mean -0.267500, valid_cycle 0
missing_second_hs: reward_mean -0.034375, valid_cycle 0
swing_load: reward_mean 0.153074, valid_cycle 0
joint_oob: reward_mean -2.101003, valid_cycle 0
fake_cycle_ankle_only: reward_mean 0.100000, valid_cycle 0
```

Il caso `fake_cycle_ankle_only` e' il controesempio direttamente legato
all'osservazione dell'utente: ginocchio quasi fermo, caviglia oscillante,
sequenza evento fasulla. La nuova logica lo rifiuta:

```text
valid_cycle_count: 0
cycle_complete_bonus_max: 0
cycle_rejected_max: 1
loaded_knee_excursion_max_rad: 0.015
phase_min_cycle_knee_excursion_rad: 0.12
```

## Baseline policy

### Random/untrained

Output:

```text
validation/reward_policy_runs/2026-07-03_random_policy_baseline/
```

Metriche:

```text
status: PASS
episode_return: -2.084803129813263
reward_mean: -0.20848031298132633
reward_min: -2.3175162285919035
reward_max: 0.08175493627576333
steps: 10
valid_cycle_count_final: 0
nan_count: 0
```

Interpretazione: la policy random resta chiaramente peggiore del prescribed
aligned e non chiude cicli.

### Vecchio checkpoint da 100 iterazioni

Output:

```text
Trajectory Generator/runs/rollout/2026-07-03_old_100iter_checkpoint_current_reward/
```

Metriche:

```text
ok: true
steps: 500
episode_return: -12.279588615968445
reward_mean: -0.024559177231936888
reward_min: -0.7749999999974229
reward_max: 0.8241950724670792
terminated: false
truncated: true
action_clipped_fraction: 0.002
record_outputs: false
```

Interpretazione: il vecchio comportamento che prima sembrava promettente non e'
piu' competitivo con la reward attuale. Questo e' coerente con l'obiettivo di
eliminare la scorciatoia dei cicli fasulli.

## Training e rollout diagnostici

### Smoke training 2 iterazioni

Output:

```text
Trajectory Generator/runs/training/2026-07-03_smoke2_reward_validation/
```

Esito: PASS infrastrutturale.

Metriche principali:

```text
iterations_completed: 2
timed_out: false
best_episode_return_mean: -2.66337240099042
```

Interpretazione: la pipeline RLlib, la config e i checkpoint funzionano.

### Smoke rollout

Output:

```text
Trajectory Generator/runs/rollout/2026-07-03_smoke2_reward_validation_rollout_recorded/
```

Esito: PASS infrastrutturale.

Metriche principali:

```text
ok: true
steps: 13
reward_mean: -0.20739874196386138
record_outputs: true
```

Interpretazione: il checkpoint e' caricabile e produce trace/output `.sto`.

### Training diagnostico 10 iterazioni

Output:

```text
Trajectory Generator/runs/training/2026-07-03_diag10_reward_validation/
```

Esito: FAIL diagnostico comportamentale.

Metriche:

```text
ok: true
iterations_completed: 10
timed_out: false
best_episode_return_mean: -2.4803382930046234
best_iteration: 9
final_episode_return_mean: -2.683589172082808
grf_penetration_final: 23
```

Interpretazione: l'infrastruttura di training e' stabile, ma il segnale di
apprendimento non supera il gate. Le terminazioni per penetrazione GRF dominano
la raccolta e impediscono progresso di fase.

### Rollout best 10 iterazioni

Output:

```text
Trajectory Generator/runs/rollout/2026-07-03_diag10_reward_validation_best_rollout/
```

Metriche:

```text
ok: true
steps: 10
episode_return: -2.382635642832604
reward_mean: -0.2382635642832604
reward_min: -2.639740762239603
reward_max: 0.18093515066656082
terminated: true
truncated: false
action_clipped_fraction: 0.0
record_outputs: true
```

Eventi online:

```text
13.946870983805102 -> heel_strike left/right
13.997870983805074 -> conferma heel_strike left/right
nessun toe_off
```

Trace reward/FSM:

```text
phase_valid_hs_count: 1
phase_valid_to_count: 0
phase_valid_cycle_count: 0
grf_penetration_loss max/finale: 1.4030641310396434
terminated: 1
```

Interpretazione: il checkpoint best e' caricabile e tracciabile, ma non produce
una sequenza utile `HS -> TO`. Il fallimento non e' piu' il conteggio falso del
ciclo ankle-only; il blocco attuale e' una terminazione precoce per
`grf_penetration`.

## Verifiche eseguite

Comandi/verifiche principali:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/validate_training_config.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/reward_audit_suite.py --scenario prescribed_aligned
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/reward_audit_suite.py --scenario prescribed_long ...
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/reward_audit_suite.py --scenario static_leg --scenario missing_to --scenario missing_second_hs --scenario swing_load --scenario joint_oob --scenario slip_injection --scenario morphology_corridor --scenario fake_cycle_ankle_only ...
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/random_policy_reward_probe.py ...
/opt/anaconda3/envs/envCMC-rllib/bin/python Trajectory Generator/baseline_MLP/rollout_eval.py ... old_100iter_checkpoint_current_reward
/opt/anaconda3/envs/envCMC-rllib/bin/python Trajectory Generator/baseline_MLP/train_ppo_mlp.py ... smoke2_reward_validation
/opt/anaconda3/envs/envCMC-rllib/bin/python Trajectory Generator/baseline_MLP/train_ppo_mlp.py ... diag10_reward_validation
/opt/anaconda3/envs/envCMC-rllib/bin/python -m py_compile validation/reward_audit_suite.py validation/random_policy_reward_probe.py
git diff --check
rg -n "[[:blank:]]$" ...
```

Esito delle verifiche finali:

```text
py_compile: PASS
git diff --check: PASS
trailing whitespace diretto: PASS
```

## File modificati o aggiunti

Reward/FSM/config:

```text
Trajectory Generator/prosthetic_phase_fsm.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Validazione:

```text
validation/reward_audit_suite.py
validation/random_policy_reward_probe.py
validation/test_reward_function.py
validation/validate_training_config.py
validation/_env_timing.py
validation/_hybrid_env_smoke.py
```

Piano e artefatti:

```text
reports/plans/2026-07-02_piano_validazione_reward_exnovo.md
validation/reward_audit_runs/2026-07-03_reward_separation_matrix.md
validation/reward_audit_runs/2026-07-03_reward_separation_matrix.csv
```

Output principali prodotti:

```text
validation/reward_audit_runs/2026-07-03_173444_prescribed_aligned/
validation/reward_audit_runs/2026-07-03_prescribed_long_13p946870984_21p0/
validation/reward_audit_runs/2026-07-03_synthetic_negative_matrix/
validation/reward_policy_runs/2026-07-03_random_policy_baseline/
Trajectory Generator/runs/rollout/2026-07-03_old_100iter_checkpoint_current_reward/
Trajectory Generator/runs/training/2026-07-03_smoke2_reward_validation/
Trajectory Generator/runs/rollout/2026-07-03_smoke2_reward_validation_rollout_recorded/
Trajectory Generator/runs/training/2026-07-03_diag10_reward_validation/
Trajectory Generator/runs/rollout/2026-07-03_diag10_reward_validation_best_rollout/
```

## Decisione

La reward/FSM attuale supera i test offline necessari per il problema specifico
dei gate cycle fasulli. Il controesempio ankle-only viene rifiutato e il vecchio
checkpoint da 100 iterazioni viene penalizzato.

La reward non e' pero' ancora validata come baseline di training lunga. Il
training diagnostico breve mostra un blocco diverso: la policy termina presto
per `grf_penetration` e non arriva a `toe_off`.

Classificazione finale del piano:

```text
offline reward validation: PASS
fake-cycle exploit guard: PASS
training infrastructure: PASS
short-horizon learnability: FAIL
decision: REVISE_REWARD/BLOCKED_DYNAMIC
```

## Azioni aperte

Prima di lanciare training 20-50 o 100 iterazioni serve una failure analysis del
rollout:

```text
Trajectory Generator/runs/rollout/2026-07-03_diag10_reward_validation_best_rollout/
```

Priorita' tecniche:

1. Isolare perche' il rollout termina per `grf_penetration` entro 10 step.
2. Verificare se la causa e' reward shaping, dinamica OpenSim/GRF, reset state,
   azioni iniziali della policy o interazione SEA/reference governor.
3. Produrre plot dedicati del rollout H2 con GRF, penetrazione, coordinate
   protesiche, azioni, `phase_valid_*` e reward terms.
4. Solo dopo almeno una transizione `HS -> TO` coerente riprendere il gate I1
   da 20-50 iterazioni.

