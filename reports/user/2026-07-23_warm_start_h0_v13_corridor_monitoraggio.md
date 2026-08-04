# Warm start H0 con detector V13 e Morphology Corridor

## Problema

L'obiettivo della sessione era avviare un training ex-novo comparabile con il
pilot warm-start da 50 aggiornamenti del 15 luglio, aggiungendo:

- il detector V13 in modalità `two_sensor`;
- il Morphology Corridor `event_anchored` con peso `0.05`.

Prima dell'avvio è stato inoltre necessario eliminare l'ambiguità tra il
precedente flag actor-only `--warm-start` e il resume completo dal checkpoint
canonico H0.

## Soluzione implementata per l'inizializzazione

La CLI di `train_ppo_mlp.py` usa ora tre modalità distinte:

- `--warm-start`: ripristina il checkpoint RLlib completo della baseline
  canonica H0, comprendendo actor adattato, critic riscaldato, optimizer, stato
  PPO e contatori;
- `--warm-start-raw`: mantiene il precedente trapianto actor-only dentro un
  Algorithm nuovo, con critic, optimizer e stato PPO inizializzati da zero;
- `--resume-from`: rimane il comando tecnico generale per riprendere un
  checkpoint arbitrario o un training interrotto.

I due warm start sono mutuamente esclusivi. Il supervisor può comunque
continuare automaticamente dai propri `checkpoint_last` senza riapplicare il
trapianto raw. Nei `summary.json` sono stati aggiunti metadati distinti per la
modalità H0 e quella raw.

Il checkpoint canonico selezionato da `--warm-start` è:

```text
validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

## Strategia sperimentale

Per riprodurre il più fedelmente possibile il pilot50 precedente è stato usato
il suo snapshot risolto:

```text
Trajectory Generator/runs/training/validation/warm_start_h1_runs/
2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/
training_cfg.resolved.yaml
```

Questo file è la fotografia dei valori effettivi usati dal run precedente,
dopo l'unione tra YAML e override CLI. Tra i parametri principali conserva:

- 12 EnvRunner, divisi in quattro runner per ciascuno dei tre start;
- 1536 step per start e 4608 step reali per batch;
- interleaving esatto tra gli start;
- minibatch da 512 e una sola epoch PPO;
- learning rate `5e-7`;
- clip PPO `0.05`;
- coefficiente KL iniziale `1.0`;
- `freeze_logstd` e milestone per ogni iterazione attivi.

Rispetto a tale snapshot, il nuovo comando sovrascrive soltanto detector,
modalità FSM e configurazione del corridor:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python \
  "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  --config "Trajectory Generator/runs/training/validation/warm_start_h1_runs/2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50/training_cfg.resolved.yaml" \
  --warm-start \
  --output-dir "validation/warm_start_h1_runs/2026-07-23_h0_exact_v13_corridor005_iter2-51_pilot50" \
  --iterations 51 \
  --online-grf-detector-profile "validation/experimental_detector_profiles/two_sensor_v13_development_toe_down_p0p75mm_heel_x_p3p5mm.json" \
  --phase-fsm-input-mode two_sensor \
  --reward-json '{"morphology_profile":"morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json","morphology_phase_mode":"event_anchored","morphology_weight":0.05}'
```

Il target `--iterations 51` corrisponde a 50 nuovi aggiornamenti, perché il
checkpoint H0 contiene già l'iterazione logica 1.

## Stato del training

Al momento della stesura di questo report:

- il processo supervisor, il child e i 12 EnvRunner risultano attivi;
- sono stati completati 35 aggiornamenti logici, fino all'iterazione 36;
- il training sta proseguendo verso il target 51;
- `checkpoint_last` è all'iterazione 36;
- `checkpoint_best` è all'iterazione 24;
- non sono stati osservati crash, timeout o restart;
- tutti i controlli di bilanciamento esatto e KL hanno superato il gate.

Metriche principali:

| Checkpoint | Return medio | Lunghezza episodio media | KL massimo |
|---|---:|---:|---:|
| Best, iterazione 24 | -2.417059 | 36.53 step | 0.000669 |
| Last, iterazione 36 | -2.437631 | 37.10 step | 0.000408 |

Il limite hard sul KL è `0.01`, quindi gli aggiornamenti sono rimasti
ampiamente entro soglia. Il return, tuttavia, è rimasto sostanzialmente piatto
tra circa `-2.42` e `-2.45`.

## Confronto con il pilot50 senza corridor

Il comparatore è:

```text
Trajectory Generator/runs/training/validation/warm_start_h1_runs/
2026-07-15_h0_exact_interleaved_lr5e-7_iter2-51_pilot50
```

All'iterazione 36:

| Metrica | Pilot precedente | V13 + corridor | Differenza |
|---|---:|---:|---:|
| Return medio | 32.783764 | -2.437631 | -35.221395 |
| Lunghezza episodio media | 410.41 step | 37.10 step | -373.30 step |
| KL massimo | 0.000558 | 0.000408 | -0.000149 |

Il confronto per start mostra che il problema interessa l'intero batch:

| Start | Durata precedente | Durata V13 + corridor |
|---|---:|---:|
| 1.756871 s | 333.2 step | 49.87 step |
| 1.956871 s | 500.0 step | 45.47 step |
| 2.156871 s | 385.4 step | 15.90 step |

La divergenza era già presente nel primo batch, all'iterazione 2:

- pilot precedente: return `4.025223`, durata media `217.0` step;
- V13 + corridor: return `-2.434581`, durata media `37.03` step.

Sulle iterazioni comuni 2-36, il nuovo training ha mantenuto una durata media
di circa 37 step, contro circa 433 step del comparatore. Le terminazioni per
penetrazione GRF sono inoltre molto più frequenti nel nuovo run.

## Interpretazione

Il return totale non è un confronto perfettamente omogeneo perché il corridor
aggiunge un nuovo termine alla reward. La lunghezza degli episodi e le cause di
terminazione sono invece confrontabili e mostrano un problema fisico/FSM
reale, non una semplice traslazione numerica della reward.

Il Morphology Corridor non modifica direttamente la dinamica o le condizioni
di terminazione. Inoltre il fallimento è presente già nel batch raccolto dalla
policy H0 prima che il nuovo PPO abbia avuto il tempo di adattarla. L'ipotesi
principale è quindi un'incompatibilità tra l'actor H0, validato con il detector
precedente, e i nuovi segnali di fase prodotti da V13 in modalità `two_sensor`.

Il run corrente modifica però simultaneamente detector e reward; non è quindi
possibile attribuire causalmente il peggioramento al solo detector o al solo
corridor senza ulteriori ablation.

## File modificati

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `validation/test_training_resume.py`
- `validation/validate_training_config.py`

Non sono state modificate la dinamica SEA, il plugin C++ o la semantica dei
comandi agli attuatori.

## Test e verifiche

- 45 test unitari mirati superati per parser, resume, warm-start actor-only,
  preflight e sampling bilanciato;
- smoke test completo di `validate_training_config.py` superato;
- compilazione Python dei file modificati superata;
- `git diff --check` superato;
- CLI `--help` verificata con i nuovi flag;
- presenza e formato RLlib del checkpoint canonico H0 verificati;
- monitoraggio dei processi Ray, watchdog, milestone, checkpoint e metriche
  fino all'iterazione 36;
- confronto numerico iterazione-per-iterazione con il pilot50 precedente.

## TODO

- Lasciare completare il run corrente fino all'iterazione 51 e verificare
  `summary.json`.
- Eseguire un'A/B `H0 + V13/two_sensor` con corridor disattivato, mantenendo
  identici tutti gli altri parametri.
- Eseguire un'A/B `H0 + detector precedente` con corridor `0.05`, mantenendo
  identici tutti gli altri parametri.
- Confrontare le ablation usando sia return sia durata episodio, cause di
  terminazione, eventi HS/TO e componenti della reward.
- Solo dopo le ablation, decidere se adattare H0 ai nuovi segnali FSM o
  rivedere V13/`two_sensor`.
- Al termine del training, eseguire rollout deterministici su `checkpoint_best`
  e `checkpoint_last`, generare i plot e confrontarli con il pilot50.
