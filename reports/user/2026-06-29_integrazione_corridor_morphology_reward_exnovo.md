# Integrazione corridor morphology nella reward ex-novo

Data: 2026-06-29

## Problema

La reward `ex_novo` doveva includere un nuovo termine di morfologia cinematica
protesica basato sul corridoio sperimentale AB06:

```text
mean(phi) +/- K * std(phi) +/- margin
```

Il termine doveva agire sulla traiettoria generata dalla policy e servita al
controllore SEA, non sulla cinematica prescritta IK ne sulla sound leg. La
configurazione doveva esporre solo i parametri utili al corridor:

- profilo morphology AB06;
- peso del termine;
- moltiplicatori `K` separati per knee e ankle;
- margini separati per knee e ankle, esposti in gradi.

In una prima versione era stato aggiunto anche `morphology_signal: served_ref`,
ma il campo e stato rimosso perche la modalita deve essere fissa: solo
`served_ref`.

## Soluzione

E stato aggiunto un profilo JSON versionato in:

```text
Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_mean_std_corridor.json
```

Il profilo contiene:

- `phase_grid` normalizzata `0..1`;
- `mean_rad` e `std_rad` per `pros_knee_angle`;
- `mean_rad` e `std_rad` per `pros_ankle_angle`;
- metadati della pipeline AB06, inclusi `n_cycles: 123`,
  `grf_threshold_n: 20.0` e `mean_to_phase: 0.6223299989`.

In `RewardConfig` sono stati aggiunti i campi:

```yaml
morphology_profile: ""
morphology_weight: 0.0
morphology_std_multiplier_knee: 1.0
morphology_std_multiplier_ankle: 1.0
morphology_margin_knee_deg: 0.0
morphology_margin_ankle_deg: 0.0
```

Il termine morphology:

- carica il profilo una sola volta in `RewardShapingWrapper`;
- usa la fase `online_gait["sides"]["left"]["gait_phase"]`;
- interpola il corridoio AB06 alla fase corrente;
- legge sempre e solo:
  - `pros_knee_angle_served_ref`;
  - `pros_ankle_angle_served_ref`;
- calcola una loss normalizzata solo sull'escursione fuori banda;
- espone diagnostiche su disponibilita, fase, valore corrente e limiti min/max
  del corridor.

La penalita viene sottratta dopo il clip:

```text
reward = reward_base - ... - morphology_weight * morphology_loss
```

Con `morphology_weight: 0.0` il termine e diagnostico e non altera il training.

## Strategia

La scelta tecnica e stata mantenere il corridor come guardrail della
generazione cinematica ex-novo, non come penalita sull'esecuzione fisica del
giunto. Questo separa i ruoli:

- `served_ref` morphology: controlla che la traiettoria richiesta dalla policy
  sia biomeccanicamente plausibile;
- tracking/saturazione/SEA terms: controllano se la protesi riesce a eseguire
  quella reference;
- `prosthetic_joint_range_*`: continua a monitorare il q simulato reale con
  limiti globali.

La modalita configurabile `morphology_signal` e stata rimossa per evitare
ambiguita nel file ex-novo: il termine e definito semanticamente come
served-reference morphology.

## File modificati

- `Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_mean_std_corridor.json`
  - nuovo profilo AB06 mean/std in radianti;
- `Trajectory Generator/baseline_MLP/reward_function.py`
  - caricamento profilo;
  - interpolazione del corridor;
  - calcolo `morphology_loss`, `morphology_knee_loss`,
    `morphology_ankle_loss`;
  - diagnostiche min/max/phase/value;
  - sottrazione post-clip tramite `morphology_weight`;
  - rimozione del selettore `morphology_signal`;
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - aggiunto il gruppo 7 `Prosthetic kinematic morphology`;
  - esposti solo profilo, peso, K e margini;
- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - logging dei termini morphology;
- `validation/test_reward_function.py`
  - test su profilo, interpolazione, peso nullo, penalita post-clip e margini;
- `validation/validate_training_config.py`
  - validazione dei nuovi campi nel config ex-novo.

## Test e verifiche

Sono state eseguite queste verifiche:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
python3 validation/validate_training_config.py
/opt/anaconda3/envs/envCMC-rllib/bin/python -m py_compile ...
```

Risultati:

- `test_reward_function.py`: 9 test passati;
- `validate_training_config.py`: tutti i controlli passati;
- `py_compile`: passato;
- smoke env breve con `training_exnovo_cfg.yaml`: passato.

Nello smoke env il termine e risultato disponibile e diagnostico:

```text
morphology_available = 1.0
morphology_loss = finite
morphology_term = 0.0
```

La verifica conferma che il profilo viene caricato correttamente, la fase online
viene usata per interpolare il corridor e il peso nullo lascia invariata la
reward effettiva.

## Stato finale

Il termine morphology e integrato nella reward ex-novo come guardrail
diagnostico sulla reference servita. Per attivarlo nel training basta impostare
un valore positivo di `morphology_weight` e regolare:

- `morphology_std_multiplier_knee`;
- `morphology_std_multiplier_ankle`;
- `morphology_margin_knee_deg`;
- `morphology_margin_ankle_deg`.
