# Validazione onlineGRF per training e inference

## Problema

La baseline MLP usava come default `online_sensor_basis`: il profilo riproduceva
bene la GRF prescribed, ma la validazione fisica ha misurato circa `102 mm` di
penetrazione massima a sinistra. Inoltre `rollout_eval.py` non aveva un
supervisore esterno e poteva bloccarsi durante un reset o uno step OpenSim.

## Soluzione

- Promosso come default per nuovi training/inference in modalita
  `online_sensor` il profilo:
  `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_physical_basis_10mm_balanced.json`.
- Mantenuta bloccata la modalita `online` attiva: il profilo fallisce ancora il
  gate fisico completo per reserve `pelvis_ty` active/sensor p95 `5.94x`, limite
  `1.5x`.
- Aggiunto `process_watchdog.py`: heartbeat atomico, timeout startup/stallo/run,
  terminazione dell'albero processi Windows/macOS/Linux e self-test con figlio
  volutamente bloccato.
- Reso `rollout_eval.py` supervisionato. Gli import Torch/RLlib/OpenSim avvengono
  nel figlio e ogni reset/step pubblica un heartbeat.
- Rafforzato il supervisore training: heartbeat atomico, rimozione heartbeat
  residuo, timeout se il figlio non pubblica il primo stato e sonda di stallo.
- Aggiunto un gate profilo-specifico:
  `validate_online_grf_train_inference.py`.

## Strategia di validazione

1. Verificare che il watchdog termini un processo volutamente bloccato.
2. Bloccare senza simulazione ogni uso active il cui acceptance report non sia
   `PASS`.
3. Validare il contratto `online_sensor` con azioni random da training e azioni
   zero deterministiche da inference.
4. Eseguire un tiny PPO reale e il rollout deterministico del checkpoint,
   entrambi supervisionati e con lo stesso profilo.

## Risultati

- Self-test watchdog condiviso: PASS, stallo rilevato in `1.77 s`.
- Sonda watchdog training: PASS, fase intenzionalmente bloccata terminata al
  timeout `1.5 s`; nessun processo figlio residuo.
- Gate `online` active: `BLOCKED` prima di costruire l'ambiente OpenSim.
- Gate `online_sensor`: PASS, `9/9` criteri plugin/sensore superati.
- Prova negativa profilo precedente: `online_sensor_basis` correttamente
  `BLOCKED` per mismatch rispetto al profilo fisicamente validato.
- Probe contratto training: `4/4` step, massimo `3.84 s/step`, fine regolare per
  `episode_time_limit`.
- Probe contratto inference: `4/4` step, massimo `2.33 s/step`, fine regolare
  per `episode_time_limit`.
- Schema osservativo stabile: `70` feature totali, incluse `12` feature
  onlineGRF.
- Tiny PPO: PASS, `1` iterazione, `4` step, return medio `2.5008`, nessun
  timeout.
- Rollout checkpoint: PASS, `4` step, return `3.8857`, `pelvis_ty_min=0.9766`,
  nessuna terminazione unsafe; watchdog inference completato senza timeout.
- Durante probe e tiny training e comparso il warning gia noto della Static
  Optimization: QP non convergente con fallback bounded least-squares. Non ha
  bloccato la pipeline, ma resta un rischio diagnostico da monitorare nei run
  piu lunghi.

## File modificati

- `Trajectory Generator/baseline_MLP/process_watchdog.py`
- `Trajectory Generator/baseline_MLP/validate_online_grf_train_inference.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/env_factory.py`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `validation/online_grf_acceptance.py`
- `online_grf_profiles/README.md`

## Artifact principali

- `results/online_grf_training_inference_validation_sensor.json`
- `results/online_grf_training_inference_validation_active.json`
- `results/online_grf_sensor_train_inference_watchdog_summary.json`
- `results/watchdog_self_test_online_grf/watchdog_self_test.json`
- `runs/_online_grf_sensor_validation_train_tiny/summary.json`
- `runs/_online_grf_sensor_validation_rollout_tiny/rollout_summary.json`
- `runs/_online_grf_sensor_validation_rollout_tiny/watchdog_summary.json`

## TODO

- Ridurre la reserve `pelvis_ty` active/sensor p95 da `5.94x` a `<=1.5x`.
- Solo dopo un acceptance report active `PASS`, ripetere questa pipeline con
  `--grf-mode online` e autorizzare rollout/training progressivamente piu lunghi.
