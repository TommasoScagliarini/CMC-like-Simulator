# Daily report - 2026-05-31

## Sintesi

Giornata dedicata a due linee collegate:

1. validazione dell'ambiente RL di traiettoria AB06 PI su macOS arm64;
2. consolidamento della parte rete dentro `Trajectory Generator/`, con setup
   SNN/PPO e dipendenze training.

Report utente prodotti oggi:

```text
reports/user/2026-05-31_validazione_env_rl_rollout_reset_mac.md
reports/user/2026-05-31_ppo_snn_training_env_setup.md
```

## 1. Validazione env RL AB06 PI su Mac

Il lavoro parte dai TODO lasciati il 2026-05-29: rollout piu' lunghi,
verifica del reset riusabile, costo rebuild vs reuse e monitoraggio della
static optimization.

### Import dopo spostamento env

`osim_trj_cmc_like.py` risulta spostato sotto:

```text
Trajectory Generator/osim_trj_cmc_like.py
```

Lo smoke esistente e' stato adattato per aggiungere anche `Trajectory
Generator/` a `sys.path`, mantenendo la repo root con precedenza per i moduli
canonici del simulatore.

File modificato:

```text
validation/rl_env_smoke_ab06_pi.py
```

### Nuovo validatore rollout

Creato:

```text
validation/rl_env_rollout_ab06_pi.py
```

Copre:

- rollout lungo con osservazioni/reward finiti;
- determinismo e stato residuo del reset riusabile;
- confronto costo reset riusabile vs rebuild del modello.

### Risultati principali

Smoke 2 ms su macOS arm64:

```text
rl_env_smoke_ab06_pi_ok
```

Rollout lunghi:

```text
0.30 s / 30 step: PASS, finite, truncated=False
1.00 s / 100 step: PASS, finite, truncated=False
```

Reset riusabile:

```text
reuse A1 vs A2: d_reward=0, d_obs_norm=0
fresh A2 vs B : d_reward=0, d_obs_norm=0
```

Il reset riusabile non lascia stato residuo osservabile: episodi identici
coincidono bit a bit, anche rispetto a un modello ricostruito.

Costo reset:

```text
reuse reset   ~= 0.00317 s
rebuild reset ~= 5.695 s
ratio         ~= 1800x
```

Conferma `rebuild_model_on_reset=False` come default corretto per training PPO.

### Finding tecnici

- Il knee satura facilmente: 21/30 step nel rollout 0.30 s e 74/100 step nel
  rollout 1.00 s.
- L'ankle resta piu' comodo e non satura nei test riportati.
- Il fenomeno e' probabilmente legato a `segment_duration`, scaling azione,
  derivate spline e gain cascade.
- La static optimization resta deterministica ma mostra alcuni warning:
  feasibility backtracking e un fallback QP iniziale. Non blocca i rollout,
  ma resta da considerare prima di training lunghi.

## 2. Perimetro rete e PPO SNN

E' stata fissata la regola operativa:

```text
tutto il lavoro relativo alla rete vive in Trajectory Generator/
```

Il root del simulatore resta dedicato a CMC-like/OpenSim e integrazione
dinamica.

File aggiornati:

```text
AGENTS.md
LLM_SIMULATOR_OVERVIEW.md
Trajectory Generator/Prosthesis_SNN/AGENTS.md
```

## 3. PPO_SNN adattato

E' stata aggiunta una versione locale adattata di `src/agent/ppo_snn.py` dalla
repo:

```text
TommasoScagliarini/SNN-Colangelo-Mardaru-Scagliarini
```

Nuovi file:

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/entropy_scheduler.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
```

Caratteristiche:

- dipende da `skrl` solo nel package opzionale `training`;
- porta la membrana/stato SNN tra step dello stesso episodio;
- azzera lo stato ricorrente alla terminazione/troncamento;
- supporta modello policy/value condiviso o separato;
- compatibilita' aggiornata per `skrl 2.1.0`, dove lo scheduler e'
  `KLAdaptiveLR`.

Decisione architetturale presa:

```text
actor-critic con backbone SNN condiviso e teste separate policy/value
```

Documentata in:

```text
Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md
```

## 4. Dipendenze training

Aggiornato `pyproject.toml`:

```text
[project.optional-dependencies]
training = ["skrl", "gymnasium", "scipy"]
```

Installato il package editable con extra `training` in:

```text
base
envCMC-like
```

In `envCMC-like` la wheel PyPI di `torch 2.12.0` causava:

```text
OMP: Error #15: Initializing libomp.dylib, but found libomp.dylib already initialized.
```

Risoluzione:

```bash
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

Nota aggiunta al README del package SNN: per uso combinato SNN + OpenSim su
macOS preferire PyTorch da `conda-forge`, non wheel PyPI.

## 5. Verifiche eseguite

Smoke RL AB06 PI:

```bash
conda run --no-capture-output -n envCMC-like \
  python validation/rl_env_smoke_ab06_pi.py
```

Esito: passato.

Rollout RL:

```bash
conda run --no-capture-output -n envCMC-like \
  python validation/rl_env_rollout_ab06_pi.py --reset-reps 2

conda run --no-capture-output -n envCMC-like \
  python validation/rl_env_rollout_ab06_pi.py --rollout-dur 1.0 \
  --residual-dur 0.02 --skip-cost
```

Esito: passati.

Compilazione training package:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/entropy_scheduler.py' \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Esito: passato.

Smoke SNN:

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

Esito: passato senza `KMP_DUPLICATE_LIB_OK`.

Import adapter env:

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

## Stato file

Stato Git osservato durante la chiusura:

```text
 M AGENTS.md
 M LLM_SIMULATOR_OVERVIEW.md
 D osim_trj_cmc_like.py
 M reports/daily/2026-05-29_daily-report.md
 M validation/rl_env_smoke_ab06_pi.py
?? Trajectory Generator/
?? build/
?? reports/user/2026-05-31_ppo_snn_training_env_setup.md
?? reports/user/2026-05-31_validazione_env_rl_rollout_reset_mac.md
?? validation/rl_env_rollout_ab06_pi.py
```

`build/` risulta non tracciata. Non e' stata rimossa.

## TODO chiusi oggi

- Smoke env RL su macOS arm64 con plugin `.dylib`.
- Rollout env RL fino a 1 s con osservazioni e reward finiti.
- Verifica reset riusabile: nessuno stato residuo osservabile.
- Misura costo reset rebuild vs reuse: rebuild circa 1800x piu' lento.
- Import dello smoke riparato dopo spostamento dell'env in `Trajectory
  Generator/`.
- Installazione `gymnasium`, `skrl`, `snntorch` e package SNN/training in
  `envCMC-like`.
- Risolto conflitto `torch`/OpenSim/OpenMP in `envCMC-like` usando PyTorch da
  `conda-forge`.
- Aggiunto `PPO_SNN` training-side e scheduler entropia locale.
- Decisione actor-critic: backbone SNN condiviso con teste policy/value.
- Rimossi dal TODO daily 2026-05-29 i punti su decisione D vs C e plot D vs C,
  come richiesto.

## TODO aperti e propagati

### Trajectory Generator / SNN / PPO

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

### Env RL / simulatore

- Decidere il layout Git definitivo: confermare lo spostamento di
  `osim_trj_cmc_like.py` in `Trajectory Generator/` e aggiornare riferimenti/
  packaging di conseguenza.
- Aggiungere `build/` al `.gitignore` o decidere esplicitamente come gestire la
  cartella.
- Valutare target/bounds biologici della static optimization alla partenza
  AB06: sono presenti warning deterministici e residui relativi alti all'init.
- Rivedere action design per il knee: `segment_duration`, scaling dell'azione,
  policy timing e cap su `qddot_ref`, per ridurre saturazione del comando.
- Decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel
  flusso `start_day` come riferimento stabile.

### Propagati dai daily precedenti

- Sweep `Kp_knee_motor` su valori intermedi tra 3.9 e 18 mantenendo ankle
  morning best.
- Validare diagnosi di coupling knee-ankle isolando la dinamica knee dal
  feedback ankle.
- Notch a 28 Hz sul feedback knee resta opzione aperta se si vuole evitare una
  modifica piu' invasiva del motor driver.
- Cleanup modelli sperimentali, includendo `slow_inner_pd_1405` e
  `pi_asym_knee1405`.
- Windows: build/copia DLL plugin PI completa e documentata.
- Secondo pass knee dello sweep locale.
- Confronto consolidato finale tra configurazioni storiche: PD, PI, cascade,
  retune PI, zeta07, pi-tuned, J_eff, Opzione D, slow inner, asym.
- Pulizia artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`.
- LPF qdot: restano test asimmetrico solo ankle, cutoff 30/35 Hz e run lunga
  30+ s se si riprende quella linea.
