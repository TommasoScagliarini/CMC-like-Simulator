# Daily report - 2026-07-02

Nota: il report e' datato 2026-07-02 come richiesto. Alcune esecuzioni
materiali, in particolare il training da 100 iterazioni con corridor, sono
terminate il 2026-07-03.

## Sintesi

Il lavoro del 2/7 ha validato la reward ex-novo corrente su dati prescribed,
ha corretto lo start del training rispetto a un HS sinistro reale, ha confrontato
i due smoke test da 10 iterazioni e ha completato un training da 100 iterazioni
con Morphology Corridor attivo.

Risultato principale: la reward offline positiva e' coerente, lo start corretto
risolve una parte del problema logico della FSM, e il checkpoint `best` del
training con corridor arriva al time limit con reward positiva e piu' cicli
`HS -> TO -> HS`.

Il risultato non e' ancora una validazione definitiva della camminata: il
checkpoint finale degrada, gli eventi online restano irregolari e le reserve
rimangono alte.

## Report utente consolidati

Report e piani del 2/7 inclusi in questo daily:

- `reports/user/2026-07-02_validazione_reward_prescribed_e_config_exnovo.md`
- `reports/user/2026-07-02_validazione_offline_reward_exnovo_attuale.md`
- `reports/plans/2026-07-02_piano_validazione_reward_exnovo.md`
- `reports/user/2026-07-02_confronto_smoke_startHS_vs_anti_halfcycle.md`
- `reports/user/2026-07-02_training_corridor_100iter_rollout.md`

## Reward prescribed

E' stato preparato e validato lo script:

```text
validation/prescribed_reward_probe.py
```

Lo scopo era verificare che la reward ex-novo premiasse una camminata prescribed
coerente usando dati prescribed in modo completo: cinematica, GRF ed eventi.

Risultato del test principale allineato su HS sinistro reale:

```text
finestra: 13.946870983805102 -> 17.99 s
steps: 405
episode_return: 136.289
reward_mean: 0.336516
reward_max: 1.0
grf_slip_loss_mean: 0.0
phase_timeout_loss_mean: 0.0
invalid_event_count_final: 0
valid_hs_count_final: 3
valid_to_count_final: 2
valid_cycle_count_final: 2
last_period_s: 1.5480
last_stance_fraction: 0.6797
```

Conclusione: la reward ex-novo attuale passa il sanity check positivo:

```text
prescribed kinematics + prescribed GRF -> reward positiva e FSM coerente
```

Il termine slip e' stato mantenuto disattivato nella config corrente:

```text
grf_slip_weight: 0.0
```

Motivo: lo slip online non e' direttamente comparabile al caso prescribed e
generava falsi negativi.

## Config ex-novo

La configurazione validata usa:

```text
reward_mode: ex_novo
blend_tracking: 0.0
blend_reference: 0.0
blend_bio: 0.0
blend_contact_load: 0.35
blend_phase_regular: 0.25
blend_phase_event_progress: 1.00
blend_landing_window_contact: 0.25
grf_slip_weight: 0.0
```

Lo start episodio corretto e':

```text
episode_start_offset_s: 1.956870983805102
t_start assoluto: 13.946870983805102 s
```

Questo corrisponde a un HS sinistro reale nella finestra IK/GRF.

## Piano reward

E' stato scritto il piano operativo:

```text
reports/plans/2026-07-02_piano_validazione_reward_exnovo.md
```

Il piano separa:

- test offline su prescribed e casi patologici controllati;
- test su checkpoint/rollout gia' esistenti;
- test che richiedono nuovo training PPO;
- metriche PASS/FAIL per ogni fase.

La parte prescritta positiva e' stata completata. La suite negativa resta
aperta.

## Confronto smoke 10 iterazioni

Sono stati confrontati due smoke test:

- `anti_halfcycle_smoke_10iter`, con `episode_start_offset_s = 1.0`;
- `startHS_slipoff_smoke_10iter`, con start corretto su HS sinistro e
  `grf_slip_weight = 0.0`.

Risultati rollout:

| Run rollout | Checkpoint | Step | Return | Reward media | Fine episodio | Cicli validi finali |
|---|---|---:|---:|---:|---|---:|
| `anti_halfcycle_smoke_10iter_best` | best | 9 | `-7.1919` | `-0.7991` | `grf_penetration` | 0 |
| `anti_halfcycle_smoke_10iter_last` | last | 99 | `-19.4658` | `-0.1966` | `grf_penetration` | 0 |
| `startHS_slipoff_smoke_10iter_best` | best | 204 | `-1.4807` | `-0.0073` | `grf_penetration` | 2 |
| `startHS_slipoff_smoke_10iter_last` | last | 188 | `-2.4201` | `-0.0129` | `grf_penetration` | 2 |

Conclusione: lo start corretto migliora nettamente la logica FSM e permette di
osservare la sequenza `HS -> TO -> HS`, ma non risolve ancora la stabilita'
fisica perche' il failure mode rimane `grf_penetration`.

## Training corridor 100 iterazioni

E' stato eseguito il training:

```text
Trajectory Generator/runs/training/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter/
```

Comando logico:

```text
training_exnovo_cfg.yaml + morphology_weight: 0.05
iterations: 100
checkpoint_every: 1
```

Esito:

```text
ok: true
iterations_completed: 100
best_episode_return_mean: -3.1365
best_iteration: 28
last_episode_return_mean: -9.7007
elapsed_wall_time_s: 44339.35
```

Il training non e' monotono: il best e' all'iterazione 28, mentre il checkpoint
finale e' peggiore.

## Rollout corridor

Rollout del checkpoint `best`:

```text
steps: 500
episode_return: 20.0684
reward_mean: 0.04014
terminated: false
truncated: true
end_reason: episode_time_limit
valid_hs_count_final: 4
valid_to_count_final: 4
valid_cycle_count_final: 3
invalid_event_count_final: 7
grf_penetration_loss_max: 0.0
reserve_norm_nm_mean: 419.99
reserve_norm_nm_max: 1041.12
```

Rollout del checkpoint `last`:

```text
steps: 319
episode_return: -57.8582
reward_mean: -0.1814
terminated: true
end_reason: phase_timeout:stance
valid_cycle_count_final: 1
phase_timeout_loss_final: 3.30
morphology_loss_final: 0.8465
```

Conclusione: il checkpoint da usare per analisi successive e' `rl_module_best`,
non `rl_module_last`. Il best passa come smoke esteso per persistenza e reward
positiva, ma non come camminata definitiva.

## Stato finale

Punti chiusi:

- reward prescribed positiva validata;
- slip prescribed non piu' falso negativo con `grf_slip_weight = 0.0`;
- start corretto allineato a HS sinistro reale;
- confronto smoke vecchio vs nuovo completato;
- training 100 iterazioni con corridor completato;
- rollout best e last del training corridor completati;
- report utente del confronto smoke scritto;
- report utente del training corridor scritto.

Punti non ancora chiusi:

- la reward audit suite negativa non e' completata;
- il training con corridor non converge stabilmente fino all'ultima policy;
- il best rollout con corridor ha ancora eventi online invalidi;
- le reserve sono ancora alte e vanno interpretate;
- non e' ancora stata prodotta una decisione finale PROMOTE/REVISE/BLOCKED
  sulla reward.

## File e output principali

- `validation/prescribed_reward_probe.py`
- `validation/prescribed_reward_probe_runs/prescribed_clean_left_hs_13p946870984_17p99/`
- `validation/prescribed_reward_probe_runs/prescribed_full_12p99_17p99/`
- `Trajectory Generator/runs/training/MLP_ExNovo_training_07-02-2026_startHS_slipoff_smoke_10iter/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_slipoff_smoke_10iter_best_rollout_07-03-2026/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_slipoff_smoke_10iter_last_rollout_07-03-2026/`
- `Trajectory Generator/runs/training/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter_best_rollout_07-03-2026/`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter_last_rollout_07-03-2026/`

## TODO propagati

- Completare la reward audit suite negativa:
  static leg, missing TO, missing second HS, swing load, joint OOB, slip
  injection diagnostico e morphology corridor.
- Costruire la separation matrix prescribed vs patologici e verificare che i
  casi patologici ricevano reward significativamente piu' bassa del prescribed
  allineato.
- Usare `rl_module_best` del training corridor per plot, visualizzazione e
  analisi successive.
- Analizzare perche' il training degrada dopo l'iterazione 28.
- Ridurre gli eventi invalidi del best rollout, soprattutto contatti brevi e
  ripetuti.
- Capire se le reserve alte indicano ancora una dinamica protesica non
  sostenibile o un problema di scaling della reward/reserve diagnostic.
- Decidere nel prossimo report se la configurazione corrente va promossa,
  rivista o bloccata.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
