# Config training GRF soft con knee jerk 2000 - 2026-06-24

Instruction check token: CMC_AGENT_OK_2026

## Problema

Dopo l'analisi del rollout GRF soft, il file sorgente
`Trajectory Generator/baseline_MLP/training_cfg.yaml` non era ancora
completamente allineato al setup del run migliore recente:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter
```

In particolare, il config sorgente era rimasto sui pesi hard-GRF:

```text
grf_penetration_weight: 5.0
grf_ankle_moment_flip_weight: 0.25
```

In parallelo, l'analisi del rollout soft-GRF ha mostrato che il knee resta il
canale critico per chattering/high-frequency. La domanda operativa era se fosse
sensato abbassare i limiti della reference knee prima del prossimo training.

## Soluzione

Il config sorgente e' stato portato al setup GRF soft e preparato per una nuova
ablation knee-specific sul jerk:

```yaml
grf_penetration_weight: 1.0
grf_ankle_moment_flip_weight: 0.10
pros_knee_ref_acceleration_limit_rad_s2: 60.0
pros_knee_ref_jerk_limit_rad_s3: 2000.0
```

La scelta e' intenzionalmente conservativa:

- il peso GRF soft replica il setup del rollout migliore recente;
- il limite di accelerazione knee resta a `60.0 rad/s^2`, perche' nel rollout
  soft-GRF non era il vincolo dominante;
- il limite di jerk knee passa da `3000.0` a `2000.0 rad/s^3`, perche' il jerk
  limit risultava spesso attivo e plausibilmente collegato al chattering knee.

Il resto della configurazione training resta allineato al run soft-GRF:

```yaml
iterations: 100
num_env_runners: 13
ray_num_cpus: 14
pros_ankle_ref_acceleration_limit_rad_s2: 55.0
pros_ankle_ref_jerk_limit_rad_s3: 2750.0
imitation_knee_position_weight: 1.0
imitation_knee_velocity_weight: 0.02
imitation_ankle_position_weight: 2.0
imitation_ankle_velocity_weight: 0.04
```

## Strategia

L'analisi prima del cambio ha confrontato `training_cfg.yaml` con il resolved
config del run soft-GRF e ha verificato che, sui campi condivisi, la nuova
differenza intenzionale fosse solo:

```text
simulation.pros_knee_ref_jerk_limit_rad_s3:
  current   = 2000.0
  reference = 3000.0
```

Per decidere sul limite di accelerazione/jerk sono stati letti i diagnostics del
rollout soft-GRF:

```text
knee served acceleration:
  mean max per step = 19.7 rad/s^2
  p95               = 36.0 rad/s^2
  max               = 42.5 rad/s^2
  current limit     = 60.0 rad/s^2

knee served jerk:
  max               = 3000 rad/s^3
  jerk_limit_fraction medio = 0.475
```

Da qui la scelta: non abbassare subito l'accelerazione a `30`, per evitare una
reference troppo lenta o troppo laggata, e testare prima solo il jerk knee a
`2000`.

## File modificati

Modificato:

```text
Trajectory Generator/baseline_MLP/training_cfg.yaml
```

Creato:

```text
reports/user/2026-06-24_config_grf_soft_knee_jerk2000.md
```

Nessuna modifica al plugin C++, alla semantica SEA o al simulatore root.

## Verifiche eseguite

- Parsing YAML di `Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- Verifica valori correnti:

```text
knee_acc   60.0
knee_jerk  2000.0
ankle_acc  55.0
ankle_jerk 2750.0
grf_pen    1.0
grf_flip   0.1
iterations 100
```

- `git diff --check -- Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- Confronto programmatico con:

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter/training_cfg.resolved.yaml
```

Esito del confronto sui campi condivisi: unica differenza intenzionale
`pros_knee_ref_jerk_limit_rad_s3: 2000.0` invece di `3000.0`.

## TODO

- [ ] Lanciare un nuovo training da zero con config attuale:
      GRF soft + `pros_knee_ref_jerk_limit_rad_s3: 2000.0`.
- [ ] Dopo il training, eseguire rollout deterministico del `rl_module_best` e
      generare plot MLP.
- [ ] Confrontare il nuovo run contro `soft_grf_latest` su:
  - return;
  - clipping totale e per giunto;
  - knee `tau_spring` RMS e high-pass ratio;
  - knee `tau_input_plugin` high-pass ratio;
  - served imitation loss;
  - knee target -> served -> actual;
  - ankle `tau_spring` area positiva/negativa;
  - online left Fy medio;
  - reserve root RMS;
  - GRF penetration e flip loss.
- [ ] Se il knee chattering migliora senza peggiorare ankle e imitation,
      considerare una seconda ablation con acceleration knee moderata
      (`45 rad/s^2`), non `30 rad/s^2` come primo passo.
- [ ] Se clipping o lag peggiorano molto, tornare a jerk `3000` e valutare
      mitigazioni alternative: bounded action head o penalty knee-specific.
