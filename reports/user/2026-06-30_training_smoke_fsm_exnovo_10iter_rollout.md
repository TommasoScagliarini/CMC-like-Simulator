# Training smoke FSM ex-novo 10 iter + rollout

Data: 2026-06-30

## Problema

Verificare che la pipeline reward ex-novo con FSM protesica HS-TO-HS appena implementata sia eseguibile in training e in rollout:

- observation actor deployable minimale con campi FSM;
- reward terms FSM presenti e numericamente sani;
- termination reason coerente;
- rollout diagnostico leggibile dopo un training smoke da 10 iterazioni.

Questo test non aveva l'obiettivo di validare apprendimento robusto. Dieci iterazioni, soprattutto con batch piccolo, servono solo a controllare cablaggio, stabilità runtime e diagnostica.

## Strategia

Il primo tentativo ha usato la configurazione piena:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --iterations 10 \
  --output-dir MLP_ExNovo_FSM_smoke_06-30-2026_10iter
```

La prima iterazione ha richiesto circa 8m14s:

```text
[iter 1/10] return=-6.037 len=71.12 steps=4096 time=0:08:14
```

Il run è stato interrotto dopo 1 iterazione perché 10 iterazioni piene avrebbero superato nettamente la finestra pratica del test. Gli artifact sono stati comunque salvati.

È stato quindi eseguito uno smoke più leggero, mantenendo la stessa config ma riducendo il batch:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --iterations 10 \
  --train-batch-size 256 \
  --minibatch-size 64 \
  --num-env-runners 8 \
  --ray-num-cpus 9 \
  --checkpoint-every 10 \
  --output-dir MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter
```

Poi è stato lanciato il rollout dal best RLModule:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python "Trajectory Generator/baseline_MLP/rollout_eval.py" \
  --checkpoint "Trajectory Generator/runs/training/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter/rl_module_best" \
  --output-dir MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter_rollout \
  --max-steps 1000
```

## Risultati training smoke

Run:

```text
Trajectory Generator/runs/training/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter
```

Esito:

```text
ok: true
stop_reason: completed
iterations_completed: 10
elapsed_wall_time_s: 436.61
best_episode_return_mean: 12.2647
best checkpoint logical iteration: 8
sampled steps lifetime: 2560
```

Andamento iterativo sintetico:

| iter | return mean | len mean | sampled steps | termination principali |
| ---: | ----------: | -------: | ------------: | ---------------------- |
| 1 | n/a | n/a | 256 | nessun episodio chiuso |
| 2 | -5.2766 | 45.0 | 512 | grf_penetration=1 |
| 3 | 0.2148 | 70.5 | 768 | grf_penetration=3 |
| 4 | -3.2349 | 90.1 | 1024 | grf_penetration=9 |
| 5 | n/a | n/a | 1280 | grf_penetration=9 |
| 6 | n/a | n/a | 1536 | grf_penetration=9 |
| 7 | -0.5398 | 98.33 | 1792 | grf_penetration=12 |
| 8 | 12.2647 | 135.5 | 2048 | grf_penetration=13, phase_timeout_swing=1 |
| 9 | n/a | n/a | 2304 | grf_penetration=13, phase_timeout_swing=1 |
| 10 | n/a | n/a | 2560 | grf_penetration=13, phase_timeout_swing=1 |

Interpretazione:

- la pipeline di training con FSM, observation e reward terms è eseguibile;
- il batch ridotto rende alcune iterazioni senza episodio completo, quindi i return `n/a` sono attesi;
- il failure mode dominante resta `grf_penetration`;
- compare anche `phase_timeout:swing`, cioè la termination FSM è cablata e visibile.

## Risultati rollout

Run:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter_rollout
```

Summary:

```text
steps: 143
episode_return: 46.0329
reward_mean: 0.3219
reward_min: -0.4195
reward_max: 0.4248
terminated: true
truncated: false
termination: phase_timeout:swing
action_abs_max: 0.4237
action_clipped_fraction: 0.0
pelvis_ty_min: 0.9529 m
```

Sequenza FSM osservata:

```text
step 1   t=13.000  WAIT_HS             credit=0.00
step 6   t=13.050  STANCE_AFTER_HS     HS valid, credit=0.25
step 13  t=13.120  SWING_AFTER_TO      TO valid, credit=0.50
step 143 t=14.420  TIMEOUT             swing_elapsed=1.302 s
```

Contatori finali:

```text
phase_valid_hs_count: 1
phase_valid_to_count: 1
phase_valid_cycle_count: 0
phase_cycle_progress_credit: 0.5
phase_event_progress_score: 0.5
invalid_event_count: 0
invalid_event_loss: 0
```

Il CSV eventi online conferma la stessa sequenza:

```text
left heel_strike: 12.990 s, confirmed 13.041 s
left toe_off:     13.118 s
nessun secondo left heel_strike prima della termination
```

Landing window:

```text
active steps: 55
first active: step 68, swing_elapsed=0.552 s
last active:  step 122, swing_elapsed=1.092 s
landing_window_contact_score max: 0.0
```

Quindi la finestra temporale viene raggiunta, ma il detector non vede ricomparsa valida della GRF left durante la finestra. Il rollout chiude solo `HS -> TO`, poi resta in swing fino all'hard timeout.

## Analisi

Il risultato è coerente con il vecchio checkpoint patologico atteso, ma ora la diagnosi è più chiara:

- la FSM runtime funziona: transizioni, credito progressivo e timeout sono leggibili;
- la reward dà credito 0.25 dopo HS e 0.5 dopo TO;
- non emerge il nuovo HS, quindi non viene mai assegnato credito 1.0;
- non ci sono invalid event, quindi il fallimento non è ordine eventi sbagliato;
- la landing window è attiva ma non produce contatto valido;
- `phase_timeout:swing` è quindi la termination corretta;
- la policy non è ancora validata come camminata HS-TO-HS.

Il training smoke è da considerare verde per cablaggio e runtime, non verde per performance della policy.

## File modificati/prodotti

File creato:

- `reports/user/2026-06-30_training_smoke_fsm_exnovo_10iter_rollout.md`

Artifact training:

- `Trajectory Generator/runs/training/MLP_ExNovo_FSM_smoke_06-30-2026_10iter/`
- `Trajectory Generator/runs/training/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter/`

Artifact rollout:

- `Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter_rollout/rollout_summary.json`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter_rollout/rollout_policy_trace.json`
- `Trajectory Generator/runs/rollout/MLP_ExNovo_FSM_smoke_fast_06-30-2026_10iter_rollout/sim_outputs/`

Nessuna modifica al codice durante questa fase di training/rollout/report.

## Verifiche eseguite

- Training full-batch avviato e interrotto dopo 1 iterazione per costo temporale eccessivo.
- Training smoke leggero completato: 10/10 iterazioni.
- Best checkpoint esportato: `rl_module_best`, logical iteration 8.
- Rollout completato con output `.sto`, trace JSON, summary JSON e CSV eventi.
- Analisi trace reward/FSM:
  - `HS=1`
  - `TO=1`
  - `cycle=0`
  - `credit=0.5`
  - `invalid_event=0`
  - `termination=phase_timeout:swing`

## Conclusione

La pipeline FSM ex-novo è operativa end-to-end: training, checkpoint, rollout, reward terms e diagnostica FSM sono leggibili. Il comportamento appreso non è ancora sufficiente: la policy produce un mezzo ciclo valido `HS -> TO`, ma non richiude `HS -> TO -> HS`.

Il prossimo passo utile è un training più lungo o una diagnostica mirata sulla fase swing-to-landing, perché il segnale chiave mancante è la ricomparsa di GRF valida nella landing window.
