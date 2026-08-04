# 2026-06-26 Daily Report

## Sintesi

Giornata centrata sulla transizione ex-novo della baseline MLP: validazione della
fase protesica da GRF online, confronto dinamico tra profili GRF, design della
reward task-based e audit finale di readiness del training.

Risultato principale: il `Trajectory Generator/baseline_MLP` e' ora pronto per
un training ex-novo reale con reward task-based punti 1-6, actor senza target
imitativo sano, actor senza gait clock prescritto sano, e doppio canale GRF:

- `grf_correct_magnitude` per il supporto fisico della simulazione;
- `grf_detector_HS-TO` come detector sensor-only per HS/TO e fase protesica.

## Report utente creati oggi

- `reports/user/2026-06-26_validazione_eventi_grf_fase_protesica_reward.md`
- `reports/user/2026-06-26_confronto_ibrido_profili_grf_online.md`
- `reports/user/2026-06-26_decisione_profili_grf_supporto_fase_reward.md`
- `reports/user/2026-06-26_design_reward_core_exnovo_timing_grf.md`
- `reports/user/2026-06-26_readiness_training_exnovo_reward_dual_grf.md`

## 1. Validazione eventi GRF online

E' stata costruita una pipeline dedicata:

```text
validation/validate_online_grf_events.py
validation/test_online_grf_event_matching.py
```

Obiettivo: validare `online_grf` non come magnitudo fisica assoluta, ma come
segnale per forma, contatto e timing HS/TO.

Reference:

- file: `models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot`;
- soglia reference: `20 N`;
- durata minima contatto: `0.05 s`;
- ciclo minimo: `0.30 s`;
- finestra setup: `11.99-21.0 s`;
- `sample_dt = 0.001 s`.

Gate strict lato sinistro/protesico:

- HS precision/recall = `1.0 / 1.0`;
- TO precision/recall = `1.0 / 1.0`;
- HS max error <= `50 ms`;
- TO max error <= `80 ms`.

Risultato:

- `grf_correct_magnitude`: FAIL come detector HS/TO;
- `grf_detector_HS-TO`: PASS su IK replay e forward states.

Detector raccomandato:

```text
low_threshold = 15 N
confirmation_threshold = 120 N
min_contact_duration = 0.03 s
causal_smoothing = 0.10 s
```

Esiti principali:

```text
IK replay grf_detector_HS-TO:
  HS precision/recall = 1.0 / 1.0
  HS max abs error = 0.013 s
  TO precision/recall = 1.0 / 1.0
  TO max abs error = 0.026 s

Forward states:
  HS max abs error = 0.013 s
  TO max abs error = 0.027 s
```

## 2. Confronto dinamico A/B tra profili GRF

Eseguito confronto CMC-like ibrido su finestra `11.99-13.99 s`.

Setup:

- lato sinistro/protesico: online GRF applicata;
- lato destro: prescribed GRF mantenuta;
- profilo A: `grf_correct_magnitude`;
- profilo B: `grf_detector_HS-TO`.

Entrambe le run hanno completato la simulazione.

Risultati chiave:

| metrica | grf_correct_magnitude | grf_detector_HS-TO |
|---|---:|---:|
| TO max error | 53 ms | 2 ms |
| Contact IoU | 0.925 | 0.980 |
| impulso verticale sinistro | 659.5 Ns | 488.4 Ns |
| impulso / prescribed | 0.869 | 0.643 |
| tau reserve norm p95 | 325.6 | 433.3 |
| pelvis ty reserve p95 | 304.7 N | 400.1 N |
| penetration max sinistra | 0.0175 m | 0.0394 m |

Conclusione:

- `grf_correct_magnitude` resta migliore per sostenere fisicamente la protesi;
- `grf_detector_HS-TO` resta migliore come proxy sensoriale per HS/TO/fase;
- non usare la magnitudo GRF event-tuned come target fisico di reward.

Artifact:

```text
results/hybrid_profile_ab_grf_correct_magnitude/
results/hybrid_profile_ab_grf_detector_HS-TO/
results/hybrid_profile_ab_comparison/report.md
results/hybrid_profile_ab_comparison/summary.json
results/hybrid_profile_ab_comparison/left_fy_events.png
results/hybrid_profile_ab_comparison/dynamics_comparison.png
```

## 3. Decisione architetturale GRF/fase

Decisione consolidata:

```text
supporto dinamico sim    -> grf_correct_magnitude
detector HS/TO/fase      -> grf_detector_HS-TO
```

Implicazioni reward:

- punti 1-2 non devono inseguire una GRF prescribed quantitativa;
- stance/load va interpretato come contatto/carico non nullo phase-gated;
- swing unloading va interpretato come assenza di contatto/carico in swing;
- penetration, slip e moment flip restano guardrail/diagnostiche, non target
  cinematico o wrench prescribed.

## 4. Reward ex-novo punti 1-6

Implementata reward task-based nel perimetro `Trajectory Generator/baseline_MLP`.

Blocchi:

1. `stance contact/load`;
2. `swing unloading`;
3. `contact validity`;
4. `phase sequence + periodicity + timing`;
5. `prosthetic joint range`;
6. `SEA usability`.

Scelte temporali:

- `T_nom` fisso per primo pass;
- model-based AB06 treadmill;
- `phase_period_nominal_s = 1.58`;
- niente stima dinamica autoreferenziale da periodo HS->HS prodotto dalla policy.

Punto 7 rimandato:

- `prosthetic kinematic morphology`;
- da progettare in un secondo pass senza introdurre imitazione prescribed.

## 5. Readiness training ex-novo

Durante l'audit finale sono stati trovati e corretti due blocchi:

1. reward/fase ancora agganciata allo stesso profilo GRF usato per la dinamica;
2. actor ex-novo ancora dotato di gait clock sano prescritto come pacemaker
   interim.

Correzioni:

- introdotto profilo detector separato:

```yaml
online_grf_profile: online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
online_grf_detector_profile: online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
```

- disattivato gait clock sano nel cfg ex-novo:

```yaml
gait_clock_enable: false
```

Probe env ex-novo:

```text
n_actor = 31
n_obs = 76
gait_clock available = False
sound_clock_features = 0.0, 1.0
actor_has_imitation_target = False
profile = AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
detector_profile = AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
force_counts = 20 physical/sensor contacts, 4 detector contacts
```

Smoke PPO minimale:

```text
output_dir = Trajectory Generator/runs/training/_exnovo_smoke_audit
ok = true
stop_reason = completed
iterations_completed = 1
num_env_runners = 0
sampled env steps = 2
checkpoint_last scritto
```

Verdetto: la pipeline MLP e' pronta per un training ex-novo reale.

Comando base:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml"
```

## File modificati o creati oggi

Codice/config principali:

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `config.py`
- `path_resolver.py`
- `online_grf.py`
- `model_loader.py`
- `simulation_runner.py`
- `validation/test_reward_function.py`
- `validation/validate_training_config.py`
- `validation/validate_online_grf_events.py`
- `validation/test_online_grf_event_matching.py`

Profili GRF:

- creato/rinominato `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json`;
- creato/rinominato `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`;
- rimossi/sostituiti i nomi storici `residual_tangent_v2` e
  `preliminary_calibrated`.

Artifact principali:

- `results/online_grf_event_validation_preliminary_fine/`
- `results/online_grf_event_validation_forward_states_recommended/`
- `results/hybrid_profile_ab_comparison/`
- `Trajectory Generator/runs/training/_exnovo_smoke_audit/`

## Test e verifiche

Eseguiti e passati:

```text
validation/test_online_grf_event_matching.py: PASS
unittest validation.test_online_grf_event_matching + validation.test_online_grf_core: 14/14 PASS
validation/test_reward_function.py: PASS
validation/validate_training_config.py: PASS
py_compile sui Python operativi toccati: PASS
git diff --check sui file toccati: PASS
env ex-novo reset + step: PASS
PPO smoke 1 iterazione: PASS
```

## TODO chiusi o avanzati il 26/06

- [x] Validare eventi HS/TO online su lato protesico.
- [x] Eseguire confronto A/B dinamico ibrido tra `grf_correct_magnitude` e
  `grf_detector_HS-TO`.
- [x] Decidere il ruolo dei due profili GRF.
- [x] Implementare separazione esplicita tra profilo dinamico e detector di fase
  nella pipeline RL.
- [x] Rinominare i profili GRF con nomi semantici.
- [x] Rimuovere dall'actor ex-novo il pacemaker sano prescritto tramite
  `gait_clock_enable: false`.
- [x] Esporre all'actor ex-novo solo fase/eventi protesici online per la parte
  phase-based.
- [x] Implementare reward core ex-novo punti 1-6 con pesi nel cfg.
- [x] Inserire in config i parametri temporali del primo pass:
  `phase_period_nominal_s`, margine soft, hard range, timeout stance/swing.
- [x] Aggiornare test reward e config wiring.
- [x] Eseguire smoke PPO end-to-end del training ex-novo.

## TODO aperti e propagati

### Training ex-novo

- [ ] Lanciare training ex-novo reale con `training_exnovo_cfg.yaml`.
- [ ] Monitorare nel primo training reale:
  - `contact_load_score`;
  - `phase_regular_score`;
  - `phase_period_loss`;
  - `phase_timeout_loss`;
  - `swing_unloading_loss`;
  - `grf_penetration_loss`;
  - `grf_ankle_moment_flip_loss`;
  - termini SEA di saturazione, torque error, motor speed/accel/power.
- [ ] Verificare se i pesi dei sei blocchi reward sono bilanciati o se un
  termine domina/deprime il learning.
- [ ] Controllare che la policy non sfrutti il detector/fase in modo
  degenerato, per esempio generando contatti spuri per manipolare HS/TO.

### Reward ex-novo secondo pass

- [ ] Disegnare il punto 7 `prosthetic kinematic morphology` senza imitazione
  prescribed diretta.
- [ ] Possibili sottotermini futuri:
  - knee flexion minima in swing;
  - knee extension prima di HS;
  - ankle stance non collassata;
  - toe clearance;
  - smoothness;
  - assenza di oscillazioni ad alta frequenza.
- [ ] Preparare in futuro una stima dinamica di `T_nom` basata sulla velocita di
  cammino stimata da sensori protesici, lenta, filtrata e clampata in range
  clinico/biomeccanico.
- [ ] Tenere nel primo training `T_nom` fisso/model-based AB06 treadmill.

### Baseline imitativa GRF half, propagata dal 25/06

- [ ] Eseguire rollout deterministico del best checkpoint
  `MLP_imitation_training_06-25-2026_grfhalf_40iter`, best logical iteration 37.
- [ ] Generare plot/diagnostica MLP per la run 40 iter.
- [ ] Ripetere il confronto in fase critica `0.15-0.40` con Fy > `50 N`.
- [ ] Promuovere `grfhalf_40iter` solo se:
  - stance protesica valida;
  - knee RMSE target-served migliore o non peggiore della soft `1.0/0.10`;
  - ankle tau ancora negativa in early/mid stance;
  - root reserve non peggiore della baseline soft matura.
- [ ] Confrontare contro baseline soft GRF `1.0/0.10`, jerk2000, hard/original
  se utile, ed eventuale `sym60` come anti-chattering baseline.

### Diagnostica ginocchio e GRF, propagata dal 25/06

- [ ] Fare diagnosi data-driven stance/swing del degrado al ginocchio.
- [ ] Incrociare knee target, served, tau, action, clipping, COP e GRF.
- [ ] Separare errore di policy/reference da errore SEA tracking.
- [ ] Decidere se intervenire con ulteriore riduzione GRF, penalty
  knee-specific, bounded action head o weak contact-confidence term.
- [ ] Non adottare ancora la variante acceleration knee `45/30` senza diagnosi
  dedicata.

### Migrazione imitation -> ex-novo, propagata dal 25/06

- [ ] Scegliere sorgente warm-start:
  - baseline `grfsoft_knee1_ankle2_100iter`;
  - oppure `grfhalf_40iter` dopo rollout e validazione.
- [ ] Creare `Trajectory Generator/baseline_MLP/training_imitation_cfg.yaml`.
- [ ] Decidere il ruolo legacy di `training_cfg.yaml`:
  - alias temporaneo;
  - deprecazione;
  - copia del config imitativo.
- [ ] Aggiornare CLI training con scorciatoie `--imitation` e `--exnovo` o altra
  convenzione non ambigua.
- [ ] Implementare `Trajectory Generator/baseline_MLP/transfer_imitation_to_exnovo.py`.
- [ ] Il tool deve:
  - caricare `rl_module_best` imitativo;
  - ricostruire schema osservazioni sorgente da `training_cfg.resolved.yaml`;
  - ricostruire schema target da `training_exnovo_cfg.yaml`;
  - copiare pesi actor per nome feature;
  - gestire le feature healthy target rimosse;
  - re-inizializzare critic nel primo esperimento;
  - salvare initializer compatibile ex-novo;
  - scrivere `actor_transplant_report.json`.
- [ ] Implementare modalita `--removed-feature-mode drop` e
  `--removed-feature-mode mean-bias`.
- [ ] Stimare `mean(x_removed)` dai dati normalizzati usati in imitation.
- [ ] Validare iteration 0:
  - source imitation;
  - ex-novo random;
  - ex-novo transplant drop;
  - ex-novo transplant mean-bias.
- [ ] Eseguire ablation random vs transplant su training ex-novo.

### Metodo e documentazione

- [ ] Nei report futuri distinguere sempre:
  - target imitation;
  - reference IK/prescribed;
  - served reference;
  - tracking SEA;
  - reward functional/task-based.
- [ ] Propagare i TODO non chiusi nei daily successivi.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
