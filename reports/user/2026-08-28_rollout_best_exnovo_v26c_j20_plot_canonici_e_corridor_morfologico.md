# Rollout del best checkpoint ex-novo V26C/J20, plot canonici e corridor morfologico

> Aggiornamento della struttura del 2026-10-19: le procedure conservate nella
> precedente `Trajectory Generator/baseline_MLP/validation/` sono ora in
> [`imitation_to_exnovo/`](<../../Trajectory Generator/baseline_MLP/imitation_to_exnovo/>).
> I percorsi e gli hash nel corpo restano quelli del documento originale.
> Mappa della rinomina, compatibilità e dipendenze già mancanti sono nel
> [registro della rinomina](../cleanup/2026-10-19_imitation_to_exnovo_rename/README.md).


**Data:** 2026-08-28  
**Stato:** completato  
**Training analizzato:** `MLP_ExNovo_V26C_J20_pilot_2026-08-28`

## 1. Obiettivo

Al termine del training ex-novo sono state eseguite le seguenti attività:

1. identificazione del best checkpoint del run;
2. rollout deterministico a orizzonte completo del relativo modulo di inferenza;
3. registrazione degli output fisici del simulatore;
4. generazione dei sette plot canonici;
5. predisposizione del comando di visualizzazione Simbody/OpenSim;
6. confronto delle reference cinematiche generate dalla rete con il morphology corridor realmente configurato nel training.

Lo scopo operativo era ottenere artefatti riproducibili per valutare cinematica, coppie, tracking SEA, policy e conformità morfologica del miglior candidato prodotto dal training.

## 2. Lineage e checkpoint utilizzato

Il run di training concluso si trova in:

`Trajectory Generator/runs/training/MLP_ExNovo_V26C_J20_pilot_2026-08-28`

Il training è terminato regolarmente:

- `ok: true`;
- `stop_reason: completed`;
- 50 nuove iterazioni eseguite;
- iterazione logica finale: 51;
- best mean episode return del training: `51.42057620871359`;
- best checkpoint individuato all'iterazione logica/RLlib 24.

Per l'inferenza è stato usato:

`Trajectory Generator/runs/training/MLP_ExNovo_V26C_J20_pilot_2026-08-28/rl_module_best`

### Chiarimento sulla inizializzazione

Il processo delle 50 iterazioni non ha applicato nuovamente un warm-start imitativo grezzo. Il suo `initialization_mode` è `resume_from` e la sorgente di restore è:

`Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j20_runs/j20_critic_warmup_v26c_2026-08-27_r1/checkpoint_last`

Quindi il run riparte dal candidato J20 dopo il warm-up del critic, che rappresenta il prodotto training-ready della pipeline costruita a partire dall'imitativo V26 di agosto. La ricostruzione completa della lineage precedente è documentata in [2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md](2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

L'attestazione training-ready immediatamente precedente è descritta in [2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md](2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md) e nella relativa correzione [2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md](2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md).

## 3. Strategia di rollout

È stato eseguito un rollout:

- deterministico;
- senza exploration noise;
- con registrazione degli output `.sto` e degli eventi di gait;
- con policy trace abilitata;
- sull'intero orizzonte nominale di 500 policy step, corrispondente a 5 secondi simulati.

Comando eseguito:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python \
  'Trajectory Generator/baseline_MLP/rollout_eval.py' \
  --checkpoint 'Trajectory Generator/runs/training/MLP_ExNovo_V26C_J20_pilot_2026-08-28/rl_module_best' \
  --output-dir 'Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout' \
  --action-selection deterministic \
  --record-outputs \
  --startup-timeout-s 600 \
  --step-timeout-s 180 \
  --stall-timeout-s 300 \
  --run-timeout-s 3600
```

Output principale:

`Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout`

## 4. Risultati del rollout

| Metrica | Valore |
|---|---:|
| `ok` | `true` |
| Step | `500/500` |
| Episode return | `42.266472849278166` |
| Reward media per step | `0.08453294569855634` |
| Reward minima | `-0.3038616037332001` |
| Reward massima | `1.0` |
| End reason | `episode_time_limit` |
| `terminated` | `false` |
| `truncated` | `true` |
| Action clipped fraction | `0.0` |
| Minimo `pelvis_ty` | `0.9522629702051837 m` |

`truncated: true` non rappresenta un fallimento: in questo caso è la normale chiusura per raggiungimento del limite temporale dopo avere completato tutti i 500 step. Non si è verificata una terminazione fisica o di sicurezza anticipata.

### Diagnostica GRF/FSM

| Metrica | Valore |
|---|---:|
| Penetrazione GRF massima | `0.024473144876597273 m` (`24.47 mm`) |
| Heel strike validi | `3` |
| Toe-off validi | `3` |
| Cicli validi completati | `2` |
| Eventi invalidi | `0` |

La penetrazione massima:

- supera la soglia soft mantenuta a `20 mm`;
- resta sotto la soglia diagnostica di luglio di `25 mm`;
- resta sotto la soglia hard di terminazione di `28 mm`.

Il rollout ha pertanto completato l'orizzonte, ma il valore di penetrazione va conservato come dato diagnostico.

### Diagnostica morphology runtime

| Metrica | Valore |
|---|---:|
| Segmenti settled | `373` |
| Segmenti discarded | `0` |
| Campioni settled | `377` |
| Campioni discarded | `123` |

## 5. Sette plot canonici

I plot sono stati generati con:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  plot/plotter.py \
  --mlp \
  --results-dir 'Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout/sim_outputs' \
  --prefix rollout_episode \
  --setup 'models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml' \
  --gait-side left \
  --out-root plot
```

> Correzione del 2026-09-03: i grafici gait-cycle `03` e `04` erano stati
> originariamente generati con `--gait-side all`, concatenando impropriamente
> gli eventi left e right per la protesi sinistra. Sono stati sostituiti
> in-place usando `--gait-side left`; il comando sopra riporta il contratto
> corretto. Non sono state conservate copie duplicate dei PNG errati.

Cartella risultante:

`plot/08_28_2026_1`

Figure canoniche prodotte:

1. `01_time_sea_control_reserve.png`;
2. `02_time_joint_motor_states.png`;
3. `03_gaitcycle_torque_angle_power.png`;
4. `04_gaitcycle_joint_velocity_power.png`;
5. `05_time_tau_input_tracking_error.png`;
6. `06_time_joint_ref_sea_error.png`;
7. `07_mlp_policy_vs_sound_leg_error.png`.

Verifiche effettuate:

- esattamente sette PNG canonici presenti;
- dimensioni e formato PNG validi;
- ispezione visiva delle figure 01 e 07;
- nessun pannello mancante o rendering anomalo osservato;
- `missing_channels.txt` riporta `No missing channels.`

## 6. Visualizzazione del modello

Per riprodurre il modello usando esattamente la cinematica dell'ultimo rollout:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python visualize.py \
  --mlp \
  --sto 'Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout/sim_outputs/rollout_episode_kinematics.sto' \
  --speed 1.0 \
  --loop
```

L'opzione `--mlp` carica dal summary del rollout il setup coerente con la simulazione; `--sto` blocca esplicitamente l'input sulla cinematica di questo specifico rollout. `--loop` può essere rimosso per una singola riproduzione.

## 7. Traiettorie della rete nel morphology corridor

È stata utilizzata l'utility esistente:

`validation/plot_morphology_corridor_rollout.py`

Questa utility:

- legge dal `rollout_policy_trace.json` la fase morfologica e le reference cinematiche effettivamente servite dalla rete;
- ricostruisce il corridor dal profilo e dai parametri contenuti nel `rollout_summary.json`;
- verifica che i limiti ricostruiti coincidano con quelli loggati a runtime;
- separa i segmenti continui di fase FSM;
- evidenzia con croci rosse i campioni esterni al corridor.

Configurazione effettiva:

- profilo: `morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json`;
- phase mode: `event_anchored_causal_delayed_experimental`;
- morphology weight: `0.0025`;
- knee: `mean ± 1.6σ ± 7.5°`;
- ankle: `mean ± 0.6σ ± 7.5°`.

Comando eseguito:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  validation/plot_morphology_corridor_rollout.py \
  --rollout-dir 'Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout' \
  --output 'plot/08_28_2026_1/08_morphology_corridor_served_reference.png'
```

Risultati sui 373 step con morfologia disponibile:

| Articolazione | Dentro | Fuori | Frazione fuori | Escursione massima oltre corridor |
|---|---:|---:|---:|---:|
| Knee | `363/373` | `10/373` | `2.68097%` | `0.077891 rad` |
| Ankle | `309/373` | `64/373` | `17.15818%` | `0.199454 rad` |

Il controllo di coerenza tra corridor ricostruito e limiti salvati nel trace ha prodotto errori massimi compresi tra `1.11e-16` e `2.22e-16 rad`, equivalenti a solo roundoff numerico.

Il grafico supplementare è:

`plot/08_28_2026_1/08_morphology_corridor_served_reference.png`

### Interpretazione corretta del grafico

La curva sovrapposta al corridor è la **served reference cinematica generata dalla policy**, cioè il segnale sul quale viene calcolato il termine di reward morfologica. Non è la coordinata fisica misurata del modello dopo il tracking SEA. Le due quantità devono rimanere distinte: la prima misura la morfologia della traiettoria richiesta dalla rete; la seconda include anche l'errore di tracking e la dinamica closed-loop.

L'esito mostra una buona aderenza del knee, mentre l'ankle presenta una quota non trascurabile di campioni fuori corridor, concentrata visivamente attorno alla transizione stance/swing. In questa fase è stata eseguita soltanto la diagnosi richiesta; non sono stati modificati reward, corridor o rete.

## 8. File generati e modifiche al repository

Artefatti generati:

- directory completa del rollout e relativi `.sto`, CSV, policy trace e summary sotto `Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout/`;
- sette figure canoniche sotto `plot/08_28_2026_1/`;
- figura supplementare `plot/08_28_2026_1/08_morphology_corridor_served_reference.png`;
- il presente report.

Non è stato modificato alcun file sorgente Python, YAML, modello OpenSim o profilo morfologico. Sono state riutilizzate le utility già presenti e sono stati aggiunti soltanto artefatti di rollout, grafici e documentazione.

## 9. Esito complessivo

Il best checkpoint è stato valutato correttamente in modalità deterministica e ha completato l'intero orizzonte. La pipeline di logging e plotting ha prodotto tutti gli artefatti attesi senza canali mancanti. Restano come osservazioni diagnostiche, non come errori di esecuzione:

- penetrazione massima di `24.47 mm`, sopra la soglia soft ma sotto entrambe le soglie superiori concordate;
- `64/373` campioni ankle e `10/373` campioni knee della served reference fuori dal morphology corridor.
