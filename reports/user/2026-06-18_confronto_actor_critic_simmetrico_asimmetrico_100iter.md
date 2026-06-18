# Confronto actor-critic simmetrico vs asimmetrico a 100 iterazioni

Data: 2026-06-18

## Problema

Serviva confrontare in modo approfondito i due training MLP imitativi a 100
iterazioni:

- actor-critic simmetrico:
  `Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_resume_best_to_100`;
- actor-critic asimmetrico:
  `Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_asym_actor_critic_100`.

L'obiettivo era capire se il critic privilegiato produce un vantaggio reale
rispetto alla baseline simmetrica, senza confondere il return PPO con la qualita
fisica del rollout.

## Caveat metodologico

Il confronto non e' un A/B perfettamente causale.

Il run simmetrico a 100 iterazioni non e' un training da zero: riprende dal best
logico `65` del ramo precedente e poi continua da `66` a `100`.

Il run asimmetrico invece e' un training da zero da `1` a `100`.

Questo significa che:

- la curva completa `1-100` dell'asimmetrico non va confrontata direttamente con
  la finestra `66-100` del simmetrico come misura di velocita di apprendimento;
- il simmetrico parte da un checkpoint gia preselezionato, quindi ha un vantaggio
  iniziale;
- allo stesso tempo il simmetrico puo ereditare local optimum e bias del ramo
  precedente.

La conclusione corretta e': l'asymmetric actor-critic e' il miglior candidato
emerso finora e batte la baseline simmetrica estesa, ma il confronto non isola
al 100% l'effetto architetturale.

## Strategia

Il confronto e' stato diviso in due livelli.

1. Training metrics:
   - `summary.json`;
   - `train_iterations.jsonl`;
   - best/last return;
   - finestre finali;
   - `vf_explained_var`;
   - `vf_loss_unclipped`;
   - entropy;
   - lunghezza media episodio;
   - terminazioni time-limit vs GRF penetration.

2. Rollout deterministici:
   - rollout simmetrico gia presente:
     `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_resume_best_to_100`;
   - rollout asimmetrico generato durante l'analisi:
     `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100`;
   - lettura di `rollout_summary.json`, `rollout_policy_trace.json` e degli
     output `.sto` in `sim_outputs/`.

## Risultati training

| Metrica | Simmetrico 100 | Asimmetrico 100 |
|---|---:|---:|
| Best return | `292.508 @97` | `307.168 @100` |
| Return finale | `282.845` | `307.168` |
| Media return 66-100 | `252.061` | `269.862` |
| Deviazione standard return 66-100 | `33.598` | `17.115` |
| Iterazioni full-length 66-100 | `8/35` | `23/35` |
| Final `vf_explained_var` | `0.024` | `0.240` |
| Last10 `vf_explained_var` medio | `0.065` | `0.157` |
| Entropy finale | `1.177` | `1.477` |

Il risultato piu importante e' sul critic. L'asimmetrico ha un value molto piu
informativo e mantiene una policy piu stabile nella finestra finale. Questo e'
coerente con la funzione prevista del privileged critic: ridurre la varianza
degli advantage senza dare feature privilegiate alla policy in inferenza.

## Terminazioni training

Nella finestra `81-100`:

- simmetrico: circa `87.6%` time-limit e `12.4%` GRF penetration;
- asimmetrico: circa `92.2%` time-limit e `7.8%` GRF penetration.

L'asimmetrico arriva quindi piu spesso a episodio completo e riduce le
terminazioni legate a GRF penetration.

## Rollout deterministici

Entrambi i rollout completano l'episodio da `501` step senza terminazione per
caduta:

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Episode return | `369.827` | `370.374` |
| Reward medio | `0.73818` | `0.73927` |
| Reward minimo | `0.36271` | `0.39843` |
| Pelvis `ty` minimo | `0.95157 m` | `0.95171 m` |
| Action clipped steps | `73` | `47` |
| Action clipped fraction | `7.29%` | `4.69%` |
| Raw action max | `1.425` | `1.219` |

Il return deterministico e' quasi pari, ma l'asimmetrico riduce chiaramente il
clipping e migliora il reward minimo.

## Tracking cinematico

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Knee served-target RMSE | `0.1336 rad` | `0.1010 rad` |
| Ankle served-target RMSE | `0.0263 rad` | `0.0268 rad` |
| Knee actual-served RMSE | `0.00353 rad` | `0.00370 rad` |
| Ankle actual-served RMSE | `0.00788 rad` | `0.00594 rad` |
| Knee actual-target RMSE | `0.1336 rad` | `0.1007 rad` |
| Ankle actual-target RMSE | `0.0263 rad` | `0.0277 rad` |
| Knee served-target corr | `0.974` | `0.986` |
| Ankle served-target corr | `0.976` | `0.972` |

Il SEA continua a non essere il collo di bottiglia: gli errori actual-served
restano bassi. Il miglioramento principale dell'asimmetrico e' sulla reference
servita del ginocchio. La caviglia e' sostanzialmente un pareggio, con RMSE e
correlazione leggermente peggiori ma span servito piu ampio.

## Azioni e command rate

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Knee raw action max | `1.425` | `1.130` |
| Ankle raw action max | `0.942` | `1.219` |
| Knee raw action mean abs delta | `0.693` | `0.616` |
| Ankle raw action mean abs delta | `0.143` | `0.280` |
| Knee raw action clip value fraction | `14.6%` | `9.2%` |
| Ankle raw action clip value fraction | `0.0%` | `0.2%` |

L'asimmetrico riduce aggressivita e clipping sul ginocchio, ma sposta parte
dell'attivita sulla caviglia. Questo e' coerente con il lieve peggioramento
dell'ankle tracking e con il command-rate loss medio piu alto.

## SEA, potenza e reserve

| Metrica | Simmetrico | Asimmetrico |
|---|---:|---:|
| Knee SEA torque error RMSE | `2.373 Nm` | `2.357 Nm` |
| Ankle SEA torque error RMSE | `0.385 Nm` | `0.449 Nm` |
| Knee motor power abs mean | `47.10 W` | `42.47 W` |
| Ankle motor power abs mean | `2.16 W` | `1.71 W` |
| SEA input saturation | `0%` | `0%` |
| `tau_reserve_norm_mean` | `331.91` | `336.90` |
| `reserve_control_norm_mean` | `3.319` | `3.369` |
| `muscle_share_mean` | `0.2606` | `0.2606` |
| equilibrium failures fraction | `2.12%` | `2.22%` |

L'asimmetrico non stressa di piu il SEA: la potenza media motore cala e non ci
sono saturazioni. Tuttavia non risolve il problema strutturale delle reserve,
che restano alte e sono leggermente peggiori nel rollout asimmetrico.

## Conclusione

L'asymmetric actor-critic va promosso a nuova baseline operativa per i training
imitativi MLP.

Motivi:

- best return piu alto;
- critic molto migliore;
- finestra finale piu stabile;
- piu episodi full-length;
- rollout deterministico almeno pari sul return;
- miglior tracking knee served-target;
- meno clipping;
- minore potenza motore media.

Limite:

- non e' ancora una prova causale perfetta perche i due training hanno lineage
  diversi;
- il rollout non migliora reserve/root load;
- ankle e command-rate restano da monitorare.

La formulazione consigliata e':

> L'asymmetric actor-critic e' il miglior candidato emerso finora e batte la
> baseline simmetrica estesa. Il risultato e' forte operativamente, ma per una
> prova scientifica pulita servono training simmetrici e asimmetrici da zero con
> stessa piattaforma, stesso parallelismo e possibilmente piu seed.

## File modificati o creati

Creati durante l'analisi:

- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100/`
  - `rollout_summary.json`;
  - `rollout_policy_trace.json`;
  - `rollout_reset_diagnostics.json`;
  - `sim_outputs/*.sto`;
  - eventi gait online e offline.

Creato da questo report:

- `reports/user/2026-06-18_confronto_actor_critic_simmetrico_asimmetrico_100iter.md`.

Non sono stati modificati codice sorgente, plugin C++ o semantica SEA.

## Verifiche eseguite

- Lettura e confronto di `summary.json` per entrambi i training.
- Lettura e confronto di `train_iterations.jsonl`.
- Calcolo di best/last return, finestre `66-100`, `81-100`, `91-100`.
- Analisi di `vf_explained_var`, `vf_loss_unclipped`, entropy, lunghezze
  episodio e terminazioni.
- Generazione del rollout deterministico asimmetrico con:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint "runs\training\MLP_imitation_training_06-17-2026_asym_actor_critic_100\rl_module_best" --output-dir "runs\rollout\MLP_imitation_rollout_06-17-2026_asym_actor_critic_100" --record-outputs
```

- Confronto di `rollout_summary.json` tra simmetrico e asimmetrico.
- Analisi di `rollout_policy_trace.json` per served-target, actual-served,
  action clipping e command-rate.
- Analisi degli STO:
  - `rollout_episode_sea_diagnostics.sto`;
  - `rollout_episode_power.sto`;
  - `rollout_episode_recruitment.sto`;
  - `rollout_episode_reserve_torques.sto`;
  - `rollout_episode_gait_events.csv`;
  - `rollout_episode_gait_events_online.csv`.

## TODO aperti

- Eseguire un training simmetrico da zero `1 -> 100` con la stessa config del
  run asimmetrico per isolare meglio l'effetto architetturale.
- Se possibile, ripetere simmetrico e asimmetrico con piu seed e stesso OS /
  stesso numero di env runner.
- Generare un confronto plot ufficiale sim100 vs asym100 con figure robuste,
  specialmente torque-angle senza `fill_between` su ascissa non monotona.
- Continuare a monitorare ankle, command-rate, reserve/root load e GRF
  penetration: l'asimmetrico migliora il training, ma non risolve ancora questi
  problemi strutturali.
