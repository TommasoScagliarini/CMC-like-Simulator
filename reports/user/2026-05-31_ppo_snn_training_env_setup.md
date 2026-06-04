# PPO SNN e setup training environment - 2026-05-31

## Problema

La parte rete deve vivere nel perimetro `Trajectory Generator/` e deve poter
usare una versione adattata di `src/agent/ppo_snn.py` dalla repository
`TommasoScagliarini/SNN-Colangelo-Mardaru-Scagliarini`.

Serviva chiarire lo stato della pipeline SNN/RL:

- core SNN per generare riferimenti protesici;
- agente `PPO_SNN` derivato dalla logica PPO-RNN di skrl;
- ambiente di traiettoria che collega rete e simulatore CMC-like;
- reward e TODO ancora aperti;
- dipendenze installate nel conda environment corretto.

## Soluzione

Il lavoro relativo alla rete e' stato consolidato dentro `Trajectory Generator/`.
Il root del simulatore resta dedicato alla pipeline CMC-like/OpenSim.

Sono stati aggiunti moduli training opzionali nel package SNN:

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/entropy_scheduler.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
```

`PPO_SNN` mantiene l'idea della sorgente: la memoria/membrana SNN e' trattata
come stato ricorrente, viene portata tra step consecutivi e azzerata a fine
episodio. L'import da progetto sorgente e' stato rimosso: lo scheduler di
entropia e' locale al package.

La decisione architetturale presa oggi e':

```text
actor-critic con backbone SNN condiviso e teste separate policy/value
```

## Strategia

La strategia e' stata conservativa:

- lasciare `ProsthesisReferenceSNN` come core inference leggero;
- mettere `PPO_SNN` e dipendenze skrl nel sotto-package `training`;
- rendere `skrl` una dipendenza opzionale del package;
- evitare di modificare il plugin SEA o la semantica del controllore CMC-like;
- documentare le decisioni aperte in `docs/TODO_integration.md` invece di
  cristallizzarle nel codice.

Durante il setup ambiente e' emerso un conflitto OpenMP:

```text
OMP: Error #15: Initializing libomp.dylib, but found libomp.dylib already initialized.
```

Il problema era causato dalla wheel PyPI di `torch` in `envCMC-like`. La
soluzione operativa e' stata rimuovere `torch` pip da `envCMC-like` e installare
`pytorch` da `conda-forge`, che convive con OpenSim nello stesso processo.

## File modificati

File di istruzioni/documentazione:

```text
AGENTS.md
LLM_SIMULATOR_OVERVIEW.md
Trajectory Generator/Prosthesis_SNN/AGENTS.md
Trajectory Generator/Prosthesis_SNN/README.md
Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md
reports/daily/2026-05-29_daily-report.md
```

Package SNN/training:

```text
Trajectory Generator/Prosthesis_SNN/pyproject.toml
Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/entropy_scheduler.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
```

## Dipendenze installate

Nel conda environment `base` sono state installate le dipendenze training via:

```bash
/opt/anaconda3/bin/python -m pip install -e 'Trajectory Generator/Prosthesis_SNN[training]'
```

Nel conda environment `envCMC-like` e' stato installato il package SNN/training
e poi PyTorch e' stato riallineato a conda-forge:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m pip install -e 'Trajectory Generator/Prosthesis_SNN[training]'
/opt/anaconda3/envs/envCMC-like/bin/python -m pip uninstall -y torch
conda install -n envCMC-like -c conda-forge pytorch -y
```

Versioni verificate in `envCMC-like`:

```text
pytorch 2.10.0
snntorch 0.9.4
skrl 2.1.0
gymnasium 1.3.0
scipy 1.15.2
opensim importabile
```

## Test e verifiche eseguite

Compilazione Python:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/entropy_scheduler.py' \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Esito: passato.

Smoke test SNN:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Esito:

```text
smoke tests passed
```

Import congiunto rete + OpenSim:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -c \
  "import torch, opensim; print('torch_opensim_ok')"
```

Esito: passato senza workaround `KMP_DUPLICATE_LIB_OK`.

Import environment adapter:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -c \
  "import sys; sys.path.insert(0, 'Trajectory Generator'); import osim_trj_cmc_like; print('env_adapter_import_ok')"
```

Esito:

```text
env_adapter_import_ok
```

Controllo dipendenze:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m pip check
/opt/anaconda3/bin/python -m pip check
```

Esito: nessun requisito rotto.

## TODO

- Implementare il wrapper skrl actor-critic con backbone SNN condiviso per
  `ProsthesisReferenceSNN`: policy gaussiana, log-probability, entropy, value
  head e metadata `get_specification()["rnn"]` attesi da `PPO_SNN`.
- Creare un training entrypoint che colleghi environment, skrl memory, wrapper
  actor-critic, `PPO_SNN`, trainer, checkpointing e resume/load.
- Discutere la tecnica di training: RL puro, hybrid supervised learning + RL,
  RL adattiva che prima imita e poi ottimizza da sola, RL from verifiable
  simulator results o strategia staged simile.
- Definire l'observation space: stato completo del simulatore, soli segnali
  realisticamente disponibili sulla protesi, oppure schema ibrido tra stato
  privilegiato e sensori deployable.
- Definire e validare normalizzazione input/output, feature names, unita',
  scaling, output transform, action limits e metadata checkpoint.
- Tarare e validare la reward del trajectory environment: tracking, smoothness,
  effort, coerenza soft `q/qdot/qddot`, safety terms e truncation thresholds.
- Aggiungere smoke test end-to-end training: reset env, rollout breve, una
  update PPO, salvataggio checkpoint, reload con `ReferenceGenerator` e
  verifica di riferimenti protesici finiti.

## Note operative

Il working tree contiene anche modifiche e file non tracciati non appartenenti
necessariamente a questo report, tra cui `validation/rl_env_smoke_ab06_pi.py`,
`validation/rl_env_rollout_ab06_pi.py`, `build/` e un report utente separato
del 31/05. Non sono stati revertiti.
