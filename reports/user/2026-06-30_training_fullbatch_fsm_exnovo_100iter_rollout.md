# 2026-06-30 - Training full-batch FSM ex-novo 100 iter + rollout

## Problema

Dopo lo smoke test FSM da 10 iterazioni restava aperta una domanda: il mancato
ciclo completo `HS -> TO -> HS` dipendeva dalla batch size ridotta, dal training
troppo breve, oppure da un limite della reward/control landscape.

Per isolare il fattore training e' stato lanciato un run piu lungo con batch
storica:

```text
iterations: 100
train_batch_size: 4096
morphology_weight: 0.0
grf_mode: online_sensor
online_grf_applied_sides: [left]
detector: AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO
```

## Strategia

Il test ha usato la pipeline ex-novo aggiornata con FSM protesica nell'env:

- actor observation deployable minimale, con campi FSM;
- reward progressione evento FSM;
- landing window contact score;
- timeout hard swing/stance terminale;
- morphology ancora solo diagnostica/spenta.

Comando training:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --iterations 100 \
  --train-batch-size 4096 \
  --checkpoint-every 10 \
  --output-dir MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter
```

Comando rollout:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/rollout_eval.py" \
  --checkpoint "Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter/rl_module_best" \
  --output-dir MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout \
  --max-steps 1000
```

## Risultati training

Run:

```text
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter
```

Esito:

```text
ok: true
stop_reason: completed
iterations_completed: 100
sampled steps: 409600
elapsed_wall_time_s: 46370.78
elapsed_wall_time: circa 12h 52m 51s
started_at: 2026-06-30T18:47:38+02:00
finished_at: 2026-07-01T11:27:31+02:00
best_episode_return_mean: 40.951685
best checkpoint logical_iteration: 99
last_episode_return_mean: 40.937618
last_episode_len_mean: 141.307692
```

Trend sintetico:

| iter | return mean | len mean | termination principali |
|---:|---:|---:|---|
| 1 | -6.037315 | 71.116667 | grf_penetration=55 |
| 5 | 14.836921 | 177.282051 | grf_penetration=186, phase_timeout_swing=12 |
| 10 | 25.007879 | 142.307692 | grf_penetration=197, phase_timeout_swing=131 |
| 20 | 29.881832 | 142.000000 | grf_penetration=197, phase_timeout_swing=418 |
| 50 | 38.025788 | 142.333333 | grf_penetration=197, phase_timeout_swing=1286 |
| 80 | 39.772409 | 141.769231 | grf_penetration=200, phase_timeout_swing=2152 |
| 99 | 40.951685 | 140.974359 | grf_penetration=200, phase_timeout_swing=2703 |
| 100 | 40.937618 | 141.307692 | grf_penetration=200, phase_timeout_swing=2732 |

Gli ultimi 10 episodi medi restano nello stesso regime:

```text
last10_episode_return_mean: 40.598903
last10_episode_len_mean: 141.152564
```

Quindi il return cresce, ma la durata episodio non supera stabilmente il
failure mode noto intorno a 140-142 step.

## Risultati rollout

Run:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout
```

Sommario:

```text
steps: 140
episode_return: 43.474741
reward_mean: 0.310534
reward_min: -0.436591
reward_max: 0.417147
terminated: true
truncated: false
end reason: phase_timeout:swing
pelvis_ty_min: 0.957156
```

Azioni:

```text
raw action abs max: 2.873095
applied action abs max: 1.000000
action_clipped_steps: 140 / 140
action_clipped_fraction: 0.567857
```

La policy produce ancora comandi raw fuori bound; tutti gli step hanno almeno un
canale clippato e circa il 56.8% delle componenti viene saturato.

## Analisi FSM nel rollout

Transizioni osservate:

```text
step 1   t=13.000  WAIT_HS
step 6   t=13.050  STANCE_AFTER_HS
step 10  t=13.090  SWING_AFTER_TO
step 140 t=14.390  TIMEOUT
```

Eventi validi:

```text
valid_HS_count: 1
valid_TO_count: 1
valid_cycle_count: 0
invalid_event_count: 0
cycle_progress_credit: 0.5
phase_event_progress_score: 0.5
```

Landing window:

```text
active steps: 55
first active step: 65, swing_elapsed_s=0.553
last active step: 119, swing_elapsed_s=1.093
landing_window_contact_score_min/max/mean: 0.0 / 0.0 / 0.0
```

Timeout:

```text
step: 140
time: 14.390
phase_swing_elapsed_s: 1.303
phase_swing_hard_timeout_s: 1.300
phase_timeout_loss: 1.515
phase_timeout_side: 2.0
```

Il detector online salva gli eventi left:

```text
12.990 -> left heel_strike, confirmed 13.041
13.087 -> left toe_off
```

Non compare un secondo left heel strike. La FSM non sta generando falsi invalid
event; sta diagnosticando correttamente il mancato recupero del contatto
protesico.

## Interpretazione

Questo run riduce molto la probabilita' che il problema sia solo batch size o
numero di iterazioni. Con 100 iterazioni e batch 4096 la policy migliora il
return medio, ma resta nel medesimo attrattore:

```text
HS valido -> TO valido -> swing prolungato -> phase_timeout:swing
```

La FSM appare sana come diagnostica e come terminazione: vede la progressione
parziale, non segnala invalid event, attiva la landing window e termina quando
lo swing supera l'hard timeout.

Il problema residuo e' nella forma del segnale di apprendimento: il bonus
landing/contact e la progressione evento non sono ancora abbastanza efficaci per
rendere conveniente generare il nuovo HS. Inoltre la policy continua a usare
azioni raw saturate, quindi va considerato anche il controllo del clipping.

## File e artifact

Codice:

```text
Nessuna modifica di codice in questa fase.
```

Artifact training:

```text
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter/summary.json
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter/train_iterations.jsonl
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter/rl_module_best
Trajectory Generator/runs/training/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter/checkpoint_best
```

Artifact rollout:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout/rollout_summary.json
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout/rollout_policy_trace.json
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter_rollout/sim_outputs/
```

## Verifiche eseguite

- Training PPO completato per 100 iterazioni con batch 4096.
- Checkpoint migliore esportato come `rl_module_best`.
- Rollout deterministico eseguito dal best checkpoint.
- Analisi di `summary.json`, `train_iterations.jsonl`, `rollout_summary.json`,
  `rollout_policy_trace.json` e `rollout_episode_gait_events_online.csv`.
- Verifica conteggi FSM: `HS=1`, `TO=1`, `cycle=0`, `invalid=0`.
- Verifica landing window: attiva ma con `landing_window_contact_score=0`.
- Verifica terminazione: coerente con `phase_timeout:swing`.

## Decisione operativa

Non accendere ancora morphology. Prima bisogna modificare/tarare la parte GRF
landing/event progress in modo che compaiano cicli `HS -> TO -> HS` affidabili.

Le prossime leve tecniche sono:

- aumentare o rendere piu denso il segnale `landing_window_contact_score`;
- verificare se il detector richiede una soglia/profilo piu adatto al recupero
  contatto in training;
- valutare `policy_action_clip_weight > 0` o una strategia anti-saturazione;
- mantenere il timeout hard come guardia, ma non come principale segnale di
  apprendimento.
