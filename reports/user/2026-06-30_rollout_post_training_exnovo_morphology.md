# Rollout post-training ex-novo con diagnostica morphology

## Problema

Dopo la conclusione del training `MLP_ExNovo_training_06-29-2026` serviva
valutare il checkpoint migliore con un rollout deterministico, produrre i plot
diagnostici e verificare se la nuova reward ex-novo, inclusa la diagnostica del
corridoio morphology AB06, fosse pronta per essere analizzata.

Il termine morphology era configurato come diagnostico:

```yaml
morphology_weight: 0.0
```

quindi non poteva influenzare la reward del rollout, ma doveva comunque
produrre segnali diagnostici utili.

## Soluzione

E' stato eseguito il rollout dal checkpoint:

```text
Trajectory Generator/runs/training/MLP_ExNovo_training_06-29-2026/rl_module_best
```

Il rollout e' stato salvato in:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best
```

Sono stati poi prodotti due gruppi di plot:

1. plot specifici di analisi rollout/morphology nella cartella interna del
   rollout;
2. i 7 plot standard della pipeline `plot/plotter.py --mlp` nella cartella
   `plot/`.

## Strategia

La valutazione e' stata fatta in tre passaggi:

1. leggere `rollout_summary.json` e `rollout_policy_trace.json` per estrarre
   metriche reward, azioni, morphology, fase e saturazioni;
2. leggere gli `.sto` prodotti dal rollout per GRF online, recruitment, SEA,
   torque, controlli e cinematica;
3. generare plot aggregati e poi rilanciare il plotter standard del progetto
   sullo stesso rollout.

La diagnostica morphology e' stata confrontata usando il corridoio AB06
`mean +/- 1 std`, come definito dal profilo:

```text
Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_mean_std_corridor.json
```

## Risultati rollout

Metriche principali:

- steps: `501`;
- episode return: `103.777786`;
- reward mean/min/max: `0.207141 / -0.500000 / 0.316893`;
- stop: `terminated=False`, `truncated=True`;
- raw action abs max: `2.046561`;
- applied action abs max: `1.000000`;
- component clip fraction: `0.481038`;
- any-step clip fraction: `0.962076`;
- knee/ankle clip fraction: `0.960080 / 0.001996`.

Il rollout non termina per caduta o terminazione esplicita: arriva al limite di
episodio. Tuttavia la policy satura quasi sempre il canale knee, mentre il
canale ankle resta quasi sempre non clippato.

## Diagnostica morphology

La morphology e' disponibile durante tutto il rollout:

- available fraction: `1.000000`;
- morphology loss mean/max: `13.246425 / 25.000000`;
- knee loss mean/max: `24.540804 / 25.000000`;
- ankle loss mean/max: `1.952047 / 25.000000`.

Il punto critico osservato e':

```text
morphology_phase min/mean/max = 0.000000 / 0.000000 / 0.000000
```

Quindi il corridoio AB06 non viene campionato lungo il gait cycle, ma resta
bloccato alla fase `0.0`. Questo rende il termine morphology non ancora
utilizzabile come guardrail phase-dependent. Con `morphology_weight=0.0` il
problema non altera il rollout, ma deve essere risolto prima di assegnare peso
positivo al termine.

## GRF e recruitment

Metriche principali:

- left/right contact fraction: `0.018196 / 0.759448`;
- left normal force max: `733.780 N`;
- tau reserve norm mean/max: `437.385 / 1032.330 Nm`;
- muscle share mean: `0.226899`;
- equilibrium failures max: `1`.

Gli eventi online riportano solo un ciclo protesico parziale sul lato left:

```text
left: heel_strike=1, toe_off=1
right: heel_strike=4, toe_off=3
```

Questo e' coerente con il problema della fase morphology bloccata: il detector
online non sta fornendo una fase left affidabile e progressiva durante il
rollout.

## Plot generati

Plot specifici del rollout:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/01_rollout_scores_phase.png
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/02_morphology_corridor_served_ref.png
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/03_online_grf_recruitment.png
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/04_actions_sea.png
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/rollout_analysis_summary.json
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/rollout_analysis_report.md
```

Plot standard generati dal plotter:

```text
plot/06_30_2026_1/01_time_sea_control_reserve.png
plot/06_30_2026_1/02_time_joint_motor_states.png
plot/06_30_2026_1/03_gaitcycle_torque_angle_power.png
plot/06_30_2026_1/04_gaitcycle_joint_velocity_power.png
plot/06_30_2026_1/05_time_tau_input_tracking_error.png
plot/06_30_2026_1/06_time_joint_ref_sea_error.png
plot/06_30_2026_1/07_mlp_policy_vs_sound_leg_error.png
plot/06_30_2026_1/missing_channels.txt
```

Il file `missing_channels.txt` riporta:

```text
No missing channels.
```

## File modificati o generati

File generati:

- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/`;
- `Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best/plots/`;
- `plot/06_30_2026_1/`;
- `reports/user/2026-06-30_rollout_post_training_exnovo_morphology.md`.

Non sono stati modificati file di codice o configurazione durante questa fase
di rollout e plotting.

## Test e verifiche

Verifiche eseguite:

- rollout completato con `ok: true`;
- file `.sto` prodotti nella cartella `sim_outputs/`;
- plot specifici rollout creati e non vuoti;
- plot standard `01`-`07` creati e non vuoti;
- `missing_channels.txt` verificato: nessun canale mancante;
- ispezione visiva del plot morphology corridor.

## Conclusione

Il training puo' essere valutato tramite i plot prodotti, ma il risultato non e'
ancora adatto ad attivare il termine morphology con peso positivo.

Le criticita' principali sono:

1. `morphology_phase` resta costante a `0.0`;
2. il lato left/protesico ha contact fraction molto bassa;
3. la policy satura quasi sempre il canale knee;
4. il recruitment usa reserve elevate, con `tau_reserve_norm` massimo sopra
   `1000 Nm`.

## TODO

- Correggere la sorgente della fase usata da `RewardShapingWrapper` per la
  morphology, assicurando che la fase left/protesica avanzi lungo HS-TO-HS.
- Verificare perche' il detector online rileva pochi eventi left nel rollout.
- Ripetere rollout e plot dopo il fix della fase morphology.
- Solo dopo la verifica della fase, valutare un `morphology_weight > 0`.
