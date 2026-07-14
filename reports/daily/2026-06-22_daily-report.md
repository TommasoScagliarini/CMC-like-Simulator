# 2026-06-22 daily report

Instruction check token: CMC_AGENT_OK_2026

## Sintesi

La giornata e' stata centrata sulla diagnosi del burst positivo della coppia
ankle in `asym100`, sulla validazione di `online_grf` come segnale reward e
sull'implementazione di un pacchetto di contact-validity per il training MLP.

Conclusione principale: `sym60` non sembra un errore di plot. Il suo
comportamento favorevole nasce da una reference ankle che in early stance non
anticipa lo stato reale; `asym100`, invece, produce un burst positivo perche'
la contact patch online ribalta il braccio COP-caviglia e genera un momento GRF
alla caviglia di segno sospetto.

Il nuovo confronto da lanciare va formulato cosi':

```text
asym100
vs
asym100 + GRF contact-validity package
```

Non come `asym100 + solo tau_GRF penalty`.

## Report utente inclusi

Report del 22/06 inclusi:

- `reports/user/2026-06-22_test_controfattuale_fattore_indiretto_sym60_asym100.md`
- `reports/user/2026-06-22_online_grf_affidabilita_reward_debole_forte.md`
- `reports/user/2026-06-22_reward_biomeccanica_senza_target_torque.md`
- `reports/user/2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md`
- `reports/user/2026-06-22_reward_guard_momento_grf_caviglia.md`
- `reports/user/2026-06-22_confronto_asym100_contact_validity_grf.md`

## Diagnosi sym60/asym100

Il test controfattuale ha isolato il fattore indiretto che rende `sym60`
particolare: l'errore relativo visto dal controller ankle in early stance.

Test eseguito:

```text
policy base: asym100
finestra: 15.5-16.0 s
override in memoria:
q_ref_asym100_cf     = q_asym100     + (q_ref_sym60     - q_sym60)
qdot_ref_asym100_cf  = qdot_asym100  + (qdot_ref_sym60  - qdot_sym60)
```

Risultato:

```text
case                                  tau_at_16   tau_mean 15.5-16.0
asym100 baseline                       +20.78 Nm       +3.59 Nm
sym60 baseline                          -7.00 Nm       -6.36 Nm
asym100 controfattuale errore sym60      -7.94 Nm       -6.75 Nm
```

Interpretazione:

- `sym60` ha trovato un bacino funzionale reale;
- il comportamento favorevole dipende dal segno di `q_ref - q` e
  `qdot_ref - qdot` nella finestra critica;
- `asym100` ottimizza meglio la reward globale, ma lascia la reference ankle
  andare davanti allo stato reale in early stance, caricando positivo il PI del
  cascade.

## Affidabilita di online_grf

L'audit storico e numerico ha confermato che `online_grf` e' numericamente
coerente ma non validata come oracolo dinamico forte.

Evidenze chiave:

- audit plugin/Python macOS: errore numerico massimo circa `1.7e-12 N`;
- rollout completi con sottostima dell'impulso verticale lato sinistro:
  - `sym60`: `Fy left full` circa `0.212x`;
  - `asym100`: `Fy left full` circa `0.296x`;
- run CMC-like prescribed vs online:
  - `pelvis_ty` reserve passa da `42.78` a `475.59`, circa `11.12x`;
  - il problema decisivo non e' solo la magnitudo verticale, ma wrench, COP,
    timing e momento complessivo.

Verdetto:

```text
online_grf non e' adatta come segnale reward forte.
Puo' essere usata solo come weak contact-confidence / diagnostica,
con gating esterno e pesi bassi.
```

Gli output temporanei della nuova run CMC-like sono stati prodotti in
`/private/tmp/cmc_grf_compare_20260622/` e rimossi dopo l'estrazione delle
metriche.

## Causa del burst positivo asym100

La scomposizione del momento alla caviglia ha mostrato che il burst positivo e'
compatibile con il bilancio dinamico locale quando la contact patch online
produce un momento GRF di segno opposto.

Formula diagnostica:

```text
tau_GRF_about_ankle = ankle_axis · (M_ground - ankle_center x F_GRF)
```

Numeri chiave nel burst:

```text
asym100, t = 15.974 s:
tau_SEA          = +21.83 Nm
tau_no_left      = +0.52 Nm
tau_GRF_online   = -21.31 Nm
penetrazione     = 13.73 mm
COP_z - ankle_z  = +31 mm

sym60, stesso istante:
tau_SEA          = -8.56 Nm
tau_no_left      = +0.43 Nm
tau_GRF_online   = +9.00 Nm
penetrazione     = 11.41 mm
COP_z - ankle_z  = -45 mm
```

Verdetto: la penetrazione e' un trigger/amplificatore, ma la causa meccanica
diretta e' il ribaltamento del braccio COP-caviglia. Il residual/free moment
amplifica la magnitudine, ma il cambio di segno esiste gia' guardando il solo
braccio COP-GRF.

## Reward biomeccanica

La strada "reward torque-aware" diretta e' stata scartata come troppo legata a
target di coppia, morfologia AB06 o controller specifici.

Vincoli emersi:

- non usare target torque prescribed o shape della coppia;
- non penalizzare genericamente smoothness/rate perche' `asym100` non presenta
  chattering/spike evidenti nella `tau_spring`;
- non usare `online_grf` come segnale forte;
- evitare termini controller-specifici come `outer_i_cmd`, `cascade_xi` o stati
  integrali;
- restare su segnali fisici osservabili e modulari: stato SEA, contatto valido,
  tracking e costo dinamico composito.

Reward candidate rimaste:

- contact-load confidence debole;
- uso meccanico funzionale del SEA;
- costo dinamico composito non basato su morfologia torque.

## Implementazione contact-validity GRF

E' stato implementato un nuovo termine di guardia sul momento GRF alla caviglia.

Formula:

```text
tau_GRF_about_ankle = ankle_axis · (M_ground - ankle_center x F_GRF)
excess = max(0, -tau_GRF_about_ankle - tau_tol)
raw_loss_nm2 = excess^2
loss = raw_loss_nm2 / tau_tol^2
```

Config scelta:

```yaml
grf_ankle_moment_flip_weight: 0.25
grf_ankle_moment_flip_tau_tol_nm: 8.0
grf_ankle_moment_flip_force_threshold_n: 50.0
```

Il termine e' attivo solo per online GRF applicata sul lato protesico sinistro e
solo quando `normal_force >= 50 N`.

Oltre alla nuova penalty, il pacchetto contact-validity include i threshold di
penetrazione:

```yaml
grf_penetration_penalty_threshold_m: 0.012
grf_penetration_termination_m: 0.017
```

Interpretazione del confronto:

```text
The learning/control setup is identical to asym100; the only intentional
changes are contact-validity terms associated with the online GRF model.
```

## Stato training_cfg.yaml

`Trajectory Generator/baseline_MLP/training_cfg.yaml` e' stato allineato al setup
`asym100` per learning/control e reward imitation, con differenze intenzionali
solo nella famiglia GRF contact-validity.

Punti chiave:

```yaml
model:
  asymmetric_actor_critic: true
  seed: 123

parallelism:
  num_env_runners: 13
  ray_num_cpus: 14

simulation:
  iterations: 100
  episode_duration: 5.0
  episode_start_offset_s: 1.0

reward:
  reward_mode: imitation
  imitation_knee_velocity_weight: 0.02
  imitation_ankle_velocity_weight: 0.04
  blend_served_imitation: 0.80
  blend_imitation: 0.20
  blend_imitation_tracking: 0
  sea_tau_spring_effort_weight: 0.0
  sea_tau_spring_rate_weight: 0.0
  policy_action_clip_weight: 0.0
  grf_penetration_weight: 5.0
  grf_ankle_moment_flip_weight: 0.25
  grf_ankle_moment_flip_tau_tol_nm: 8.0
  grf_ankle_moment_flip_force_threshold_n: 50.0
```

Il confronto con
`Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_asym_actor_critic_100/training_cfg.resolved.yaml`
ha confermato che `model`, `ppo`, `parallelism`, `simulation`, `grf`,
`supervision`, `logging` e i vecchi pesi reward coincidono semanticamente. Le
chiavi nuove a peso zero (`sea_tau_spring_*`, `policy_action_clip_weight`) sono
non operative.

## File modificati oggi

Codice/config:

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/README.md
```

Report creati:

```text
reports/user/2026-06-22_test_controfattuale_fattore_indiretto_sym60_asym100.md
reports/user/2026-06-22_online_grf_affidabilita_reward_debole_forte.md
reports/user/2026-06-22_reward_biomeccanica_senza_target_torque.md
reports/user/2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md
reports/user/2026-06-22_reward_guard_momento_grf_caviglia.md
reports/user/2026-06-22_confronto_asym100_contact_validity_grf.md
reports/daily/2026-06-22_daily-report.md
```

Gia presenti nel worktree prima o comunque non oggetto di revert:

```text
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.md
paper/
reports/daily/2026-06-19_daily-report.md
reports/daily/2026-06-21_daily-report.md
reports/user/2026-06-19_*.md
reports/user/2026-06-21_*.md
```

## Verifiche eseguite

- Lettura e confronto del resolved config storico `asym100`.
- Verifica YAML:
  - `imitation_ankle_velocity_weight = 0.04`;
  - `grf_ankle_moment_flip_weight = 0.25`;
  - `grf_ankle_moment_flip_tau_tol_nm = 8.0`;
  - `grf_ankle_moment_flip_force_threshold_n = 50.0`.
- `py_compile` su:
  - `Trajectory Generator/osim_trj_cmc_like.py`;
  - `Trajectory Generator/baseline_MLP/reward_function.py`;
  - `Trajectory Generator/baseline_MLP/env_factory.py`;
  - `Trajectory Generator/baseline_MLP/tb_logging.py`.
- Test numerico reward:

```text
grf_ankle_moment_flip_loss = 1.0
grf_ankle_moment_flip_weight = 0.25
reward base = 1.0
reward finale = 0.75
```

- Pulizia dei `.pyc` generati dai check.
- Audit rollout con `rollout_episode_online_grf.sto`.
- Audit CMC-like prescribed vs online e rimozione degli output temporanei in
  `/private/tmp`.

## TODO chiusi o avanzati il 22/06

- [x] Verificare se `sym60` fosse un errore di plot o un comportamento reale.
      Esito: reale.
- [x] Isolare il fattore indiretto di `sym60` con test controfattuale. Esito:
      errore relativo ankle early stance.
- [x] Verificare se `online_grf` puo' essere usata come segnale forte. Esito:
      no, weak-only/diagnostic-only.
- [x] Individuare la causa meccanica del burst positivo `asym100`. Esito:
      ribaltamento COP-caviglia e momento GRF di segno sospetto.
- [x] Implementare guard reward sul momento GRF alla caviglia.
- [x] Esporre `tau_tol` e `force_threshold` nel config e portarli a
      `8 Nm` / `50 N`.
- [x] Allineare `training_cfg.yaml` ad `asym100` piu' pacchetto GRF
      contact-validity.
- [x] Formalizzare il confronto corretto:
      `asym100` vs `asym100 + GRF contact-validity package`.

## TODO aperti e propagati

### Nuovi / aggiornati dal 22/06

- [ ] Lanciare il training A/B:
      `asym100 + GRF contact-validity package`.
- [ ] Dopo il training, eseguire rollout deterministico, plot MLP e confronto
      con:
  - `asym100`;
  - `sym60`;
  - eventuale run `05_23`/CMC-like di riferimento.
- [ ] Nel confronto post-training monitorare:
  - `tau_spring` ankle;
  - `grf_ankle_moment_flip_loss`;
  - `grf_ankle_moment_flip_cop_z_minus_ankle_z_m`;
  - `grf_penetration_m`;
  - `served-reference tracking`;
  - reserve root, soprattutto `pelvis_ty`;
  - eventuali terminazioni/failure per `grf_penetration`.
- [ ] Verificare se il pacchetto contact-validity riduce o elimina il burst
      positivo senza peggiorare tracking, penetrazione o reserve.
- [ ] Se il burst resta, provare `grf_ankle_moment_flip_weight: 0.50`; se la
      policy diventa troppo conservativa o perde tracking, provare `0.10`.
- [ ] Progettare un candidato reward debole di contact confidence che fonda fase
      gait, eventi, cinematica del piede e `online_grf`, senza usare
      `online_grf` come segnale forte.
- [ ] Definire una versione quantitativa minimale dei termini di uso meccanico
      SEA senza target torque, morfologia della coppia o dettagli controller.
- [ ] Stabilire una baseline/floor per reserve e residual che tenga conto delle
      asimmetrie sane prescribed.
- [ ] Valutare offline, sui rollout disponibili, se i candidati reward 2/3/4
      distinguono `sym60` da `asym100` senza codificare la shape della coppia.
- [ ] Solo dopo la verifica offline, introdurre eventuali nuovi termini in
      `Trajectory Generator/baseline_MLP/reward_function.py` con pesi deboli e
      ablation dedicata.
- [ ] Chiarire perche' `online_grf_left_basis_03` prende carico dominante in
      `asym100` durante il burst e confrontare COP online vs prescribed/oracle
      nella stessa fase.

### Propagati dal 21/06 e ancora rilevanti

- [ ] Nei confronti futuri loggare/confrontare sempre `qdot_ref`, `qdot_cas`,
      `cascade_velocity_error`, contributo P, contributo I e `tau_spring`.
- [ ] Aggiungere ai plot diagnostici MLP i termini interni del cascade, almeno
      per knee e ankle.
- [ ] Separare esplicitamente nei report futuri:
  - qualita' della reference servita;
  - tracking SEA della reference;
  - segno/scala della coppia prodotta dal cascade.
- [ ] Verificare ancora il nodo del target imitation ankle:
      `ankle_angle_r` sound-leg anti-phase vs `pros_ankle_angle` / offset-range
      protesico.
- [ ] Non interpretare una curva `tau_spring` piu simile all'healthy come
      successo globale se reserve, scala della coppia e lavoro ankle restano
      lontani dal riferimento.
- [ ] Valutare una testa di output bounded/squashed, ad esempio `tanh` scalato
      ai bound fisici, per ridurre la dipendenza dal clipping hard dell'azione.
- [ ] Se la velocity imitation migliora ma l'accelerazione/reference diventa
      aggressiva, valutare una loss target-vs-served su `qddot`.

### Propagati dai report precedenti

- [ ] Eseguire confronti piu robusti simmetrico/asimmetrico:
  - training simmetrico da zero `1 -> 100` con stessa config;
  - piu seed;
  - stesso OS e stesso numero di env runner.
- [ ] Monitorare in TensorBoard:
  - served/imitation/tracking losses;
  - `policy_action_clip_loss`;
  - `policy_action_clip_fraction`;
  - reward components;
  - value metrics;
  - nuovi diagnostici `grf_ankle_moment_flip_*`.
- [ ] Valutare una ablation separata su `vf_clip_param` solo dopo i confronti
      actor-critic e reward.
- [ ] Non introdurre nella reward profili prescribed di coppia protesica o
      termini controller-specifici come `outer_i_cmd`.
- [ ] Continuare a monitorare ankle torque, knee torque, command-rate, action
      clipping, reserve/root load e GRF penetration.
- [ ] Validare heel-strike online, `in_contact`, rocker/COP push-off e contatto
      protesico.
- [ ] Ridurre carico/penetrazione del piede protesico senza perdere contatto.
- [ ] Investigare reserve biologiche elevate e richieste sui DOF non attuati,
      incluso il FAIL di tracking biologico `mtp_angle_r`.
- [ ] Progettare e validare la reward ex-novo task-based prima del warm-start.
- [ ] Implementare prima le nuove metriche ex-novo in modalita diagnostica.
- [ ] Validare auto-recovery durante un crash nativo Ray reale.
- [ ] Verificare su macOS arm64 cleanup, resume, RLModule e reward mode.
- [ ] Pulire launcher/log temporanei e artefatti di smoke quando non servono piu.
- [ ] Proseguire i TODO SNN/skrl propagati e i TODO storici SEA ancora aperti.
- [ ] Per la linea MuJoCo/MJX: verificare stato VCS, creare matrice
      cross-platform, produrre oracle OpenSim canonici, chiudere gate statici e
      integrare ambiente JAX/MJX batched prima di PPO JAX.
