# Training ex-novo 100 iterazioni con Morphology Corridor

Data report: 2026-07-02

Nota: il training e' stato eseguito materialmente tra il 2026-07-03
01:26:14 e il 2026-07-03 13:45:16, ma il report e' datato 2026-07-02 per
continuita' con la campagna sperimentale richiesta.

## Problema

Dopo lo smoke test da 10 iterazioni con start corretto su HS sinistro, la
sequenza `HS -> TO -> HS` era finalmente osservabile, ma il rollout terminava
ancora per `grf_penetration`:

```text
startHS_slipoff_smoke_10iter best:
steps: 204
episode_return: -1.4807
terminated: true
end_reason: grf_penetration
valid cycles finali: 2
invalid events finali: 4
```

Il test successivo doveva verificare se piu' iterazioni e un Morphology
Corridor leggero potessero ridurre configurazioni protesiche fragili e
migliorare la persistenza del rollout.

## Soluzione

E' stato lanciato un training da 100 iterazioni con la configurazione ex-novo
attuale e Morphology Corridor attivo tramite override runtime:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --iterations 100 \
  --checkpoint-every 1 \
  --reward-json '{"morphology_weight": 0.05}' \
  --output-dir "MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter"
```

Configurazione reward rilevante:

```text
episode_start_offset_s: 1.956870983805102
grf_slip_weight: 0.0
morphology_weight: 0.05
morphology_std_multiplier_knee: 1.6
morphology_std_multiplier_ankle: 0.6
morphology_margin_knee_deg: 7.5
morphology_margin_ankle_deg: 7.5
```

## Strategia

Sono stati valutati:

- summary del training;
- andamento per iterazione da `train_iterations.jsonl`;
- rollout del checkpoint `rl_module_best`;
- rollout del checkpoint `rl_module_last`;
- trace reward/FSM dei rollout;
- eventi online `rollout_episode_gait_events_online.csv`.

Il checkpoint `best` e' il criterio principale per giudicare se il training ha
trovato una policy utile. Il checkpoint `last` e' stato usato come controllo
per capire se la policy si e' stabilizzata o degradata durante le ultime
iterazioni.

## Risultati training

Run:

```text
Trajectory Generator/runs/training/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter/
```

Esito:

```text
ok: true
stop_reason: completed
iterations_completed: 100
best_episode_return_mean: -3.1365
best_iteration: 28
elapsed_wall_time_s: 44339.35
```

Sintesi iterazioni:

| Iterazione | episode_return_mean | episode_len_mean | entropy |
|---:|---:|---:|---:|
| 1 | `-3.1767` | `20.15` | `2.6440` |
| 28, best | `-3.1365` | `293.62` | `2.3528` |
| 65, worst | `-31.1789` | `432.45` | `2.0337` |
| 100, last | `-9.7007` | `390.14` | `1.5560` |

Le metriche di training indicano che il miglior checkpoint e' stato trovato
all'iterazione 28, non alla fine. Quindi il training non e' convergente in modo
monotono: dopo il best c'e' una fase di degradazione e oscillazione.

Termination metrics cumulative all'iterazione 100:

```text
grf_penetration: 2036
phase_timeout_swing: 114
episode_time_limit: 278
phase_timeout_stance: 48
```

Interpretazione: il training produce anche episodi lunghi, ma i failure mode
restano presenti. La presenza del corridor non elimina da sola penetrazione GRF
e timeout di fase.

## Rollout checkpoint best

Run:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter_best_rollout_07-03-2026/
```

Esito sintetico:

```text
steps: 500
episode_return: 20.0684
reward_mean: 0.04014
reward_min: -0.2581
reward_max: 0.9956
terminated: false
truncated: true
end_reason: episode_time_limit
action_clipped_fraction: 0.002
pelvis_ty_min: 0.9521 m
```

Diagnostica FSM/reward finale:

```text
valid_hs_count_final: 4
valid_to_count_final: 4
valid_cycle_count_final: 3
invalid_event_count_final: 7
phase_regular_score_final: 0.2469
phase_timeout_loss_final: 0.0
grf_penetration_m_max: 0.00272
grf_penetration_loss_max: 0.0
morphology_loss_mean: 0.1334
morphology_loss_final: 0.3262
reserve_norm_nm_mean: 419.99
reserve_norm_nm_max: 1041.12
```

Eventi online principali:

```text
left HS: 13.9469
left TO: 14.1649
left HS: 15.0719
left TO: 15.1729
left HS: 15.4669
left TO: 15.9179
left HS: 16.5309
left TO: 16.7289
left HS: 17.1219
left TO: 17.3149
```

Il best checkpoint e' il primo risultato chiaramente positivo in rollout:
raggiunge il time limit, non termina per penetrazione GRF, chiude 3 cicli
validi e ottiene return positivo.

Il limite e' che non e' ancora una camminata pulita: compaiono 7 eventi
invalidi, alcuni contatti sono brevi o ripetuti, e il supporto delle reserve
resta alto.

## Rollout checkpoint last

Run:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter_last_rollout_07-03-2026/
```

Esito sintetico:

```text
steps: 319
episode_return: -57.8582
reward_mean: -0.1814
reward_min: -1.7372
reward_max: 0.8949
terminated: true
truncated: false
end_reason: phase_timeout:stance
action_clipped_fraction: 0.00157
pelvis_ty_min: 0.9519 m
```

Diagnostica FSM/reward finale:

```text
valid_hs_count_final: 2
valid_to_count_final: 1
valid_cycle_count_final: 1
invalid_event_count_final: 1
phase_timeout_loss_final: 3.30
phase_cycle_failed_this_step_final: 1
morphology_loss_mean: 0.3814
morphology_loss_final: 0.8465
reserve_norm_nm_mean: 419.30
```

Il checkpoint `last` e' nettamente peggiore del `best`: termina per timeout di
stance e ha reward cumulata molto negativa. Questo conferma che il training da
100 iterazioni non va interpretato usando automaticamente l'ultima policy.

## Confronto con smoke 10 iterazioni

| Run | Checkpoint | Corridor | Step | Return rollout | Reward media | Fine episodio | Cicli validi finali |
|---|---|---:|---:|---:|---:|---|---:|
| smoke startHS 10iter | best | no | 204 | `-1.4807` | `-0.0073` | `grf_penetration` | 2 |
| smoke startHS 10iter | last | no | 188 | `-2.4201` | `-0.0129` | `grf_penetration` | 2 |
| corridor 100iter | best | si | 500 | `20.0684` | `0.0401` | `episode_time_limit` | 3 |
| corridor 100iter | last | si | 319 | `-57.8582` | `-0.1814` | `phase_timeout:stance` | 1 |

Il miglior checkpoint del training con corridor e' molto migliore dello smoke:

- arriva al time limit invece di terminare per `grf_penetration`;
- passa da return negativo a return positivo;
- aumenta i cicli validi finali da 2 a 3;
- mantiene `grf_penetration_loss = 0.0`;
- non mostra clipping significativo delle azioni.

La parte negativa e' la degradazione del checkpoint finale. Il corridor aiuta
il best rollout, ma il training non ha ancora una dinamica di apprendimento
robusta fino alla fine.

## Conclusione

Il test e' passato come smoke esteso: esiste una policy, salvata come
`rl_module_best`, che completa l'episodio fino al time limit con reward positiva
e chiude piu' cicli `HS -> TO -> HS`.

Non e' passato come validazione definitiva della camminata. Restano tre limiti
tecnici:

- il training degrada dopo l'iterazione best;
- gli eventi online non sono ancora puliti, con `invalid_event_count_final = 7`;
- le reserve restano alte, con media circa `420 Nm` e picco circa `1041 Nm`.

La policy da usare per analisi successive e' quindi `rl_module_best`, non
`rl_module_last`.

## File generati

- `Trajectory Generator/runs/training/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter_best_rollout_07-03-2026/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter_last_rollout_07-03-2026/`
- `reports/user/2026-07-02_training_corridor_100iter_rollout.md`

## Verifiche eseguite

- Training 100 iterazioni completato con `ok: true`.
- Rollout checkpoint `rl_module_best` completato: `500` step, time limit.
- Rollout checkpoint `rl_module_last` completato: `319` step, timeout stance.
- Lettura `summary.json` del training.
- Lettura `train_iterations.jsonl` per best/last/worst iteration.
- Lettura `rollout_summary.json` per entrambi i rollout.
- Lettura `rollout_policy_trace.json` per FSM, GRF penetration, morphology,
  reserve norm e joint range.
- Lettura `rollout_episode_gait_events_online.csv` per sequenza eventi HS/TO.

## TODO

- Usare `rl_module_best` come checkpoint di riferimento per i prossimi plot e
  per eventuale visualizzazione, non `rl_module_last`.
- Analizzare perche' il training degrada dopo l'iterazione 28.
- Ridurre gli eventi invalidi nel best rollout, in particolare contatti brevi e
  ripetuti.
- Capire se le reserve alte indicano ancora una dinamica protesica non
  sostenibile o un problema di scaling della reward/reserve diagnostic.
- Completare la suite offline negativa della reward pianificata in
  `reports/plans/2026-07-02_piano_validazione_reward_exnovo.md`.
