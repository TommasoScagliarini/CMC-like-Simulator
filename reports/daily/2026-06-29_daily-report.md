# Daily report - 2026-06-29

## Sintesi

Il lavoro del 29/6 si e' concentrato sul punto 7 della reward ex-novo:

```text
prosthetic kinematic morphology
```

L'obiettivo era trasformare l'idea del morphology term da proposta concettuale a
guardrail operativo basato su un corridoio cinematico AB06 phase-dependent:

```text
mean(phi) +/- K * std(phi) +/- margin
```

Il termine e' stato progettato per agire sulla reference protesica servita al
controllore SEA (`served_ref`), non sulla sound leg, non sulla bio reference e
non sulla vicinanza alla traiettoria media. La logica resta ex-novo: il centro
del corridoio non viene premiato, viene penalizzata solo l'uscita dalla banda.

## Report utente consolidati

Report del 29/6 inclusi in questo daily:

- `reports/user/2026-06-29_reward_exnovo_prosthetic_kinematic_morphology.md`
- `reports/user/2026-06-29_integrazione_corridor_morphology_reward_exnovo.md`

## Decisioni tecniche

- Il morphology term deve essere un guardrail, non un tracking imitativo.
- Il corridoio operativo viene costruito da AB06 sinistro/protesico.
- La segmentazione dei cicli usa la sequenza `HS -> TO -> HS` da GRF verticale.
- Per il primo pass non si usa una envelope sulle velocita' articolari.
- La fase runtime viene dal detector online GRF lato sinistro/protesico.
- Il termine agisce sulla reference servita:
  - `pros_knee_angle_served_ref`;
  - `pros_ankle_angle_served_ref`.
- La modalita configurabile `morphology_signal` e' stata rimossa: il termine e'
  sempre `served_ref`.
- Il peso iniziale resta:

```yaml
morphology_weight: 0.0
```

Quindi il termine e' diagnostico nel primo run e non modifica ancora la reward
effettiva.

## Corridoio AB06

E' stato creato il profilo:

```text
Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_mean_std_corridor.json
```

Contenuto principale:

- `phase_grid` normalizzata `0..1`;
- `mean_rad` e `std_rad` per `pros_knee_angle`;
- `mean_rad` e `std_rad` per `pros_ankle_angle`;
- metadati della pipeline:
  - `n_cycles: 123`;
  - `grf_threshold_n: 20.0`;
  - `mean_to_phase: 0.6223299989`;
  - `mean_period_s: 1.1282657419`.

La pipeline usata concettualmente:

```text
IK_results_AB06_SEASEA.mot
+ AB06_SEASEA_GRF_FullSpan.mot
-> ground_force1_vy > 20 N
-> HS crossing up, TO crossing down
-> cicli completi HS_i -> TO_i -> HS_{i+1}
-> resampling su %GC
-> mean(phi), std(phi)
```

## Plot e visualizzazione

Sono stati generati/aggiornati plot nella cartella:

```text
results/ab06_prosthetic_morphology_corridor_k1_margin0/
```

Plot statici prodotti:

- corridoio `mean +/- 1 std`;
- corridoio `mean +/- 2 std`;
- corridoio phase-dependent `min/max`;
- corridoio phase-dependent `max abs`;
- variante `min/max` con margine 5 gradi.

Plot interattivi:

- HTML basato su `phase_minmax` con slider per i margini;
- HTML basato su `mean +/- K*std` con slider per margini;
- aggiunto slider `K`;
- in seguito separato `K` in:
  - `K_knee`;
  - `K_ankle`.

Il file interattivo corrente per il corridoio `mean +/- K*std` e':

```text
results/ab06_prosthetic_morphology_corridor_k1_margin0/
ab06_prosthetic_morphology_corridor_k1std_interactive_margin.html
```

## Integrazione reward

Modifiche principali in:

```text
Trajectory Generator/baseline_MLP/reward_function.py
```

Sono stati aggiunti:

- campi morphology in `RewardConfig`;
- caricamento del profilo morphology una sola volta in `RewardShapingWrapper`;
- interpolazione del corridoio alla fase online;
- calcolo delle loss fuori banda per knee e ankle;
- diagnostiche:
  - `morphology_available`;
  - `morphology_phase`;
  - `morphology_loss`;
  - `morphology_knee_loss`;
  - `morphology_ankle_loss`;
  - valore corrente knee/ankle;
  - min/max corridor knee/ankle;
- sottrazione post-clip:

```text
reward -= morphology_weight * morphology_loss
```

Il termine e' robusto al caso in cui profilo, fase o served reference non siano
disponibili: in quel caso ritorna diagnostiche a zero e `morphology_available=0`.

## Config ex-novo

In:

```text
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

e' stato aggiunto il gruppo:

```yaml
# 7) Prosthetic kinematic morphology. AB06 mean +/- K*std corridor on the
# policy's served reference; weight 0.0 keeps it diagnostic in this first pass.
morphology_profile: morphology_profiles/ab06_prosthetic_mean_std_corridor.json
morphology_weight: 0.0
morphology_std_multiplier_knee: 1.0
morphology_std_multiplier_ankle: 1.0
morphology_margin_knee_deg: 0.0
morphology_margin_ankle_deg: 0.0
```

Il campo `morphology_signal` era stato introdotto provvisoriamente, poi rimosso:
la reward morphology e' solo `served_ref`.

## Logging

In:

```text
Trajectory Generator/baseline_MLP/tb_logging.py
```

sono stati aggiunti i termini morphology ai log TensorBoard/diagnostici, in
particolare:

- `morphology_loss`;
- `morphology_knee_loss`;
- `morphology_ankle_loss`;
- diagnostiche con prefisso `morphology_`.

## Test e verifiche

Verifiche eseguite e passate:

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

Nello smoke env morphology:

```text
morphology_available = 1.0
morphology_loss = finite
morphology_term = 0.0
```

Per l'HTML interattivo e' stato fatto un controllo statico dei riferimenti DOM e
della separazione `K_knee` / `K_ankle`. Non e' stato possibile eseguire un test
browser headless perche' `node`, `playwright` e `selenium` non erano disponibili
nell'ambiente.

## Stato training

A fine giornata il training ex-novo e' stato considerato avviabile con:

```text
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Il training e' stato poi avviato con la configurazione corrente. Il morphology
term e' diagnostico perche' `morphology_weight: 0.0`, quindi il run serve a
osservare:

- frequenza e ampiezza delle uscite dal corridoio;
- differenza tra knee e ankle;
- disponibilita' della fase online;
- relazione tra morphology loss, phase regularity e contact load.

## File modificati o creati

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_mean_std_corridor.json`
- `validation/test_reward_function.py`
- `validation/validate_training_config.py`
- `results/ab06_prosthetic_morphology_corridor_k1_margin0/*`
- `reports/user/2026-06-29_reward_exnovo_prosthetic_kinematic_morphology.md`
- `reports/user/2026-06-29_integrazione_corridor_morphology_reward_exnovo.md`

## TODO propagati

TODO originari dal report utente
`2026-06-29_reward_exnovo_prosthetic_kinematic_morphology.md`:

- [x] Costruire offline i cicli AB06 da `IK_results_AB06_SEASEA.mot` e
      GRF/eventi `HS -> TO -> HS`.
- [ ] Generare `q_min(phi), q_max(phi)` per `pros_knee_angle` e
      `pros_ankle_angle`, con smoothing e `std_floor`.
      - Stato parziale: il corridor `mean +/- K*std +/- margin` e' generato runtime dal
        profilo mean/std; smoothing e `std_floor` restano da valutare.
- [ ] Validare il corridoio su dataset AB06 e rollout esistenti.
      - Stato parziale: validato caricamento profilo, `n_cycles=123`, interpolazione e
        smoke env; resta da validare sistematicamente su rollout esistenti.
- [x] Aggiungere logging diagnostico del morphology corridor senza peso in
      reward.
- [ ] Definire le finestre HS/loading/TO/swing/pre-HS e le relative inequality.
- [x] Integrare il termine in `training_exnovo_cfg.yaml` solo dopo validazione
      diagnostica iniziale, con `morphology_weight: 0.0`.

TODO ancora aperti da propagare:

- [ ] Valutare se introdurre smoothing e/o `std_floor` nel profilo morphology.
- [ ] Validare la morphology loss su rollout esistenti e confrontarla con
      `phase_regular_score` e `contact_load_score`.
- [ ] Definire eventuali inequality biomeccaniche per finestre
      HS/loading/TO/swing/pre-HS.
- [ ] Dopo il primo training diagnostico, decidere se e quanto aumentare
      `morphology_weight`.
