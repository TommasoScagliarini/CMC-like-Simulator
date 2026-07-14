# Confronto smoke test 10 iterazioni: anti-halfcycle vs start HS corretto

## Problema

Lo smoke test precedente da 10 iterazioni era stato eseguito con uno start temporale non allineato a un heel strike sinistro utile per la sequenza protesica HS-TO-HS. Il dubbio era che una parte del comportamento patologico della policy non fosse dovuta solo alla reward, ma anche a una partenza sfavorevole rispetto alla FSM.

L'obiettivo di questo controllo era confrontare:

- smoke precedente `anti_halfcycle_smoke_10iter`, con `episode_start_offset_s = 1.0`, `grf_slip_weight = 0.02`;
- smoke nuovo `startHS_slipoff_smoke_10iter`, con `episode_start_offset_s = 1.956870983805102`, `grf_slip_weight = 0.0`.

Il Morphology Corridor e rimasto disattivato in entrambi i rollout di confronto (`morphology_weight = 0.0`).

## Strategia

Sono stati analizzati training summary, `train_iterations.jsonl`, rollout summary, trace della reward/FSM e file eventi online.

Per ogni smoke sono stati considerati sia il checkpoint best sia il checkpoint last. Il confronto principale usa pero il checkpoint best, perche rappresenta il miglior modello secondo il criterio PPO salvato durante le 10 iterazioni.

## Risultati training

| Run | Best train return medio | Iterazione best | Last train return medio | Last episode length medio |
|---|---:|---:|---:|---:|
| `anti_halfcycle_smoke_10iter` | `-16.0568` | 1 | `-17.9120` | `100.24` step |
| `startHS_slipoff_smoke_10iter` | `-2.9996` | 7 | `-12.3143` | `167.39` step |

Il nuovo smoke migliora molto il best return medio gia in training. La last del nuovo smoke e peggiore della sua best, ma mantiene episodi piu lunghi: questo indica che in 10 iterazioni l'entropia/esplorazione e ancora alta e che la policy non e convergente.

## Risultati rollout

| Run rollout | Checkpoint | Step | Return | Reward media | Reward max | Terminazione | Cicli validi FSM finali |
|---|---|---:|---:|---:|---:|---|---:|
| `anti_halfcycle_smoke_10iter_best_rollout` | best | 9 | `-7.1919` | `-0.7991` | `-0.3952` | `grf_penetration` | 0 |
| `anti_halfcycle_smoke_10iter_last_rollout` | last | 99 | `-19.4658` | `-0.1966` | `~0.0` | `grf_penetration` | 0 |
| `startHS_slipoff_smoke_10iter_best_rollout` | best | 204 | `-1.4807` | `-0.0073` | `0.9372` | `grf_penetration` | 2 |
| `startHS_slipoff_smoke_10iter_last_rollout` | last | 188 | `-2.4201` | `-0.0129` | `0.8411` | `grf_penetration` | 2 |

## Diagnostica FSM

Nel vecchio smoke `last`, la sequenza online osservata era:

- left HS a `12.990 s`;
- right HS a `13.048 s`;
- left TO a `13.107 s`, troppo precoce rispetto al vincolo di stance minima;
- left HS a `13.503 s`, invalidato;
- left TO a `13.634 s`.

Il trace chiudeva con:

- `phase_valid_hs_count = 1`;
- `phase_valid_to_count = 1`;
- `phase_valid_cycle_count = 0`;
- `invalid_event_count = 2`;
- `phase_regular_score = 0`.

Nel nuovo smoke `best`, la sequenza online parte invece da HS sinistro allineato:

- left HS a `13.946870984 s`;
- left TO a `14.131870984 s`;
- successivi HS/TO sinistri fino a `15.963870984 s`.

Il trace chiude con:

- `phase_valid_hs_count = 3`;
- `phase_valid_to_count = 2`;
- `phase_valid_cycle_count = 2`;
- `invalid_event_count = 4`;
- `phase_regular_score finale = 0.211`.

Quindi la sequenza HS-TO-HS viene finalmente osservata nel rollout, ma gli eventi sono ancora irregolari e troppo brevi rispetto alla morfologia/temporizzazione attesa.

## Diagnostica fisica

Entrambi gli smoke continuano a terminare per `grf_penetration`. Nel nuovo smoke `best`:

- `grf_penetration_m` finale: `0.01806 m`;
- `grf_penetration_loss` finale: `1.4673`;
- `contact_load_score` finale: `0.6739`;
- `prosthetic_joint_range_loss`: `0.0`;
- azioni non clippate: `action_clipped_fraction = 0.0`.

Questo indica che il nuovo start non ha risolto la stabilita fisica, ma ha rimosso una causa importante di fallimento logico della FSM. Il problema residuo non e piu solo "non arriva a HS-TO-HS"; ora arriva a cicli validi, ma con dinamica GRF non ancora stabile.

## Conclusione

Il confronto e favorevole allo start corretto:

- reward training molto meno negativa;
- rollout molto piu lungo;
- reward massima positiva;
- `2` cicli HS-TO-HS validi contro `0`;
- nessun clipping delle azioni nel nuovo smoke.

Non si puo ancora dichiarare la policy stabile. La terminazione per `grf_penetration` rimane il failure mode principale, e gli eventi online mostrano ancora contatti troppo brevi/irregolari. Il prossimo test ha senso con `100` iterazioni e Morphology Corridor attivo, per verificare se il vincolo cinematico aiuta a evitare configurazioni protesiche dinamicamente fragili.

## File generati

- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_slipoff_smoke_10iter_best_rollout_07-03-2026/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_slipoff_smoke_10iter_last_rollout_07-03-2026/`
- `reports/user/2026-07-02_confronto_smoke_startHS_vs_anti_halfcycle.md`

## Verifiche eseguite

- Rollout checkpoint best del nuovo smoke: completato, `204` step.
- Rollout checkpoint last del nuovo smoke: completato, `188` step.
- Lettura `rollout_summary.json` per vecchio e nuovo smoke.
- Lettura `rollout_policy_trace.json` per conteggi FSM, GRF penetration, contact load, range articolare e reserve norm.
- Lettura `rollout_episode_gait_events_online.csv` per sequenza eventi HS/TO.

## TODO

- Eseguire training da `100` iterazioni con config corrente e Morphology Corridor attivo.
- Eseguire rollout del checkpoint best e, se utile, del checkpoint last del training con corridor.
- Verificare se il corridor riduce eventi invalidi, contatti brevi e terminazioni per `grf_penetration`.
- Completare in seguito la suite offline negativa della reward, separata dal training.
