# Core del simulatore CMC-like

Questa cartella contiene il codice OpenSim/Python del simulatore. Riceve
riferimenti cinematici, calcola il tracking biologico e i comandi SEA,
integra la dinamica e registra i risultati STO. Il generatore di traiettorie
rimane nella cartella parallela
[`Trajectory Generator/`](<../Trajectory Generator/baseline_MLP/README.md>).

## Moduli

| File | Ruolo |
| --- | --- |
| [main.py](main.py) | Entry point e coordinamento della simulazione. |
| [config.py](config.py) | Configurazione di modello, controllori, integrazione e output. |
| [setup_io.py](setup_io.py), [create_setup.py](create_setup.py) | Lettura, scrittura e creazione dei setup XML. |
| [path_resolver.py](path_resolver.py) | Risoluzione delle risorse condivise e compatibilità dei percorsi salvati. |
| [model_loader.py](model_loader.py) | Caricamento modello, plugin, contatti, muscoli e reserve. |
| [kinematics_interpolator.py](kinematics_interpolator.py) | Interpolazione della IK per posizioni, velocità e accelerazioni di riferimento. |
| [outer_loop.py](outer_loop.py) | Tracking cinematico dei gradi di libertà biologici. |
| [inverse_dynamics.py](inverse_dynamics.py) | Calcolo delle coppie richieste. |
| [static_optimization.py](static_optimization.py) | Ripartizione delle coppie biologiche tra muscoli e reserve. |
| [prosthesis_controller.py](prosthesis_controller.py) | Controllore high-level dei due SEA. |
| [simulation_runner.py](simulation_runner.py) | Avanzamento della simulazione e coordinamento delle componenti. |
| [output.py](output.py) | Lettura/scrittura STO e registrazione dei risultati. |
| [visualize.py](visualize.py) | Riproduzione della cinematica salvata con il visualizzatore OpenSim. |

## Esecuzione

Servono OpenSim, NumPy, SciPy e i plugin compatibili con il Python e il sistema
operativo utilizzati. Eseguire dalla radice del workspace:

```sh
python "CMC-like Simulator/main.py" --help
python "CMC-like Simulator/main.py"
python "CMC-like Simulator/main.py" --validate
python "CMC-like Simulator/visualize.py" --help
python "CMC-like Simulator/visualize.py" --mlp
```

I modelli e i dati sono in [`../models/`](../models/README.md), i plugin in
[`../plugins/`](../plugins/README.md), il contatto online in
[`../online_GRF/`](../online_GRF/README.md). Le destinazioni condivise
[`../results/`](../results/) e [`../plot/`](../plot/) conservano il loro ruolo
e la loro struttura. I test e il validatore generale restano in
[`../validation/`](../validation/README.md).

## Import e percorsi

Dal 2026-10-20 i moduli sono stati raccolti qui dalla radice. Gli import
canonici restano flat (`config`, `model_loader`, `simulation_runner`, ecc.),
anche per i consumatori e i pickle esistenti. Gli entry point del core
preparano gli import delle risorse condivise; gli altri strumenti usano
[`workspace_bootstrap.py`](../workspace_bootstrap.py), che espone
`REPO_ROOT`, `SIMULATOR_DIR`, `TRAJECTORY_GENERATOR_DIR` e
`ensure_simulator_paths()` con la sola libreria standard.

La radice delle risorse rimane il workspace: `models/`, `plugins/` e
`online_GRF/` non vanno cercati dentro questa cartella. Gli input relativi al
bundle del modello conservano la risoluzione prevista dal setup. I percorsi
e gli hash dei manifest e degli snapshot congelati non vengono riscritti:
gli inventari storici che citano i vecchi sorgenti richiedono il checkout
originale per la verifica.

Per compilazione dei plugin, preparazione dei modelli e limiti dei workflow
storici, leggere [`../tools/README.md`](../tools/README.md). Le invarianti dei
SEA e le istruzioni operative sono in [`../AGENTS.md`](../AGENTS.md).
