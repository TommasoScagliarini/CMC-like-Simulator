# Historical runs registry per training MLP - 2026-06-17

## Problema

I training della baseline MLP erano distribuiti tra cartelle `runs/training`,
summary JSON, daily report e memoria operativa. Questo rendeva facile perdere
il contesto storico e rischiava di portare a ripetere run gia' fatte o molto
simili, soprattutto quando una run era spezzata in piu' segmenti logici.

Il caso concreto era la catena:

- `baseline_mlp_imit_v4_c2_4hz_obs_target` da 40 iterazioni;
- `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm` come prosecuzione
  con reward normalizzata.

Serviva un documento leggibile da Windows e macOS, con dati minimi sui training:
data, piattaforma, parametri di config, tempo elapsed e collegamenti logici tra
run collegate.

## Soluzione

E' stato introdotto un registro storico sotto:

```text
Trajectory Generator/runs/historical_runs.md
```

Il Markdown e' il file principale da leggere. Viene generato da:

```text
Trajectory Generator/baseline_MLP/update_historical_runs.py
```

e supportato da:

```text
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.manual.yaml
```

Il file manuale contiene annotazioni umane e backfill storico. L'indice JSON e'
generato automaticamente e non va modificato a mano.

## Strategia

La strategia scelta e' ibrida:

- documento finale in Markdown, per lettura quotidiana;
- indice JSON generato, per rigenerazione e parsing futuro;
- YAML manuale per correggere campi non deducibili automaticamente, come
  piattaforma storica, lineage logica e note dai report.

Lo script legge i run presenti in:

```text
Trajectory Generator/runs/training/*
```

e usa:

- `summary.json`;
- `training_cfg.resolved.yaml`;
- `checkpoint_best_meta.json`;
- `checkpoint_last_meta.json`;
- `train_iterations.jsonl`;
- timestamp delle cartelle RLlib come fallback.

Le run interrotte vengono conservate come storico, ma marcate come incomplete.
La catena 40 + 20 con reward normalizzata viene raggruppata come lineage unica.

## Automazione

`train_ppo_mlp.py` ora aggiorna il registry automaticamente in modalita'
best-effort dopo la scrittura di `summary.json`.

Sono stati aggiunti:

```text
--update-history
--no-update-history
```

Il default e' aggiornare il registry. Se l'aggiornamento fallisce, il training
non fallisce: viene stampato solo un warning.

I summary futuri includono anche:

- `platform`;
- `python_version`;
- `started_at`;
- `finished_at`;
- `updated_at`.

## File modificati o aggiunti

```text
.gitignore
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/update_historical_runs.py
Trajectory Generator/runs/historical_runs.md
Trajectory Generator/runs/historical_runs.index.json
Trajectory Generator/runs/historical_runs.manual.yaml
reports/user/2026-06-17_historical_runs_registry_training_mlp.md
```

La `.gitignore` e' stata aggiornata per continuare a ignorare gli artefatti di
training/rollout, ma tenere tracciabili i tre file del registry storico.

## Backfill iniziale

Il registry generato contiene:

- 3 run rilevate da `Trajectory Generator/runs/training`;
- 6 note storiche estratte dai report.

Run rilevate:

- `baseline_mlp_imit_v4_c2_4hz_obs_target`;
- `baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm`;
- `MLP_imitation_training_06-16-2026`.

La run interrotta `MLP_imitation_training_06-16-2026` e' marcata come
`interrupted` e non viene presentata come baseline completata.

## Verifiche eseguite

Compilazione Python:

```powershell
python -m py_compile "Trajectory Generator/baseline_MLP/update_historical_runs.py" "Trajectory Generator/baseline_MLP/train_ppo_mlp.py"
```

Generazione registry:

```powershell
python "Trajectory Generator/baseline_MLP/update_historical_runs.py"
```

Risultato:

```text
Historical runs updated: Trajectory Generator/runs/historical_runs.md (3 run(s), 6 report-only note(s))
```

Sono stati verificati:

- il rendering iniziale del Markdown;
- il raggruppamento logico della catena 40 + 20;
- lo stato interrotto della run non completata;
- assenza di path assoluti Windows/macOS in `historical_runs.index.json`;
- tracking Git dei tre file in `Trajectory Generator/runs`;
- smoke test dell'updater con run temporanee mancanti/parziali;
- `git diff --check`, con soli warning CRLF/LF attesi su Windows.

Non sono stati lanciati nuovi training.

## TODO

Nessun TODO nuovo.
