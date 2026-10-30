# Simulatore CMC-like e generatore di traiettorie

Il workspace contiene due core Python nella stessa repository:

| Cartella | Responsabilità |
| --- | --- |
| [CMC-like Simulator/](<CMC-like Simulator/README.md>) | Simulazione OpenSim, controllori SEA, tracking biologico, inverse dynamics, static optimization e risultati STO. |
| [Trajectory Generator/](<Trajectory Generator/baseline_MLP/README.md>) | Policy neurali, training PPO, reward, fasi del passo e generazione dei riferimenti protesici. |

Il generatore usa il simulatore nello stesso processo. Questa organizzazione
separa i sorgenti; i due core condividono ancora risorse, configurazioni del
modello e ambiente OpenSim. Non sono due installazioni autonome.

Le cartelle condivise rimangono nella radice:

| Cartella | Contenuto |
| --- | --- |
| [models/](models/README.md) | Modelli, mesh e dati IK/GRF dei soggetti. |
| [online_GRF/](online_GRF/README.md) | Contatto online, detector geometrico, profili e sorgenti del plugin GRF. |
| [plugins/](plugins/README.md) | Librerie SEA e GRF compilate per il sistema operativo locale. |
| [tools/](tools/README.md) | Preparazione e calibrazione dei modelli. |
| [validation/](validation/README.md) | Validatori, test e checkpoint storico del warm-start. |
| [results/](results/) | Output STO del simulatore. |
| [plot/](plot/) | Plotter e grafici; la struttura esistente è conservata. |
| [reports/](reports/) | Report, protocolli e documentazione storica. |

I checkpoint e gli output della policy rimangono in
[`Trajectory Generator/runs/`](<Trajectory Generator/runs/>).
Alcuni dati, run e binari sono esclusi da Git: il clone della repository
richiede anche le risorse locali necessarie al modello scelto.

## Comandi essenziali

Eseguire dalla radice del workspace con un ambiente Python dotato delle
dipendenze indicate nelle guide dei due core. I percorsi quotati funzionano
anche in PowerShell:

```sh
python "CMC-like Simulator/main.py" --help
python "CMC-like Simulator/main.py"
python "CMC-like Simulator/main.py" --validate
python "CMC-like Simulator/visualize.py" --mlp
python "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" --help
python "Trajectory Generator/baseline_MLP/rollout_eval.py" --help
```

Per le opzioni del training e dei rollout, leggere il
[README della baseline MLP](<Trajectory Generator/baseline_MLP/README.md>).

## Import e compatibilità

[`workspace_bootstrap.py`](workspace_bootstrap.py) espone la radice condivisa e
la cartella del simulatore usando solo la libreria standard. Dopo
`ensure_simulator_paths()`, gli import mantengono i nomi originali, per esempio
`from config import SimulatorConfig`. Il generatore richiama questo supporto
attraverso il proprio bootstrap, conservando l'ordine di import necessario
per Torch/OpenSim su Windows e nei worker Ray.

Il resolver è in
[`CMC-like Simulator/path_resolver.py`](<CMC-like Simulator/path_resolver.py>):
i percorsi delle risorse continuano a riferirsi alla radice del workspace.
Manifest, snapshot e protocolli congelati conservano i loro percorsi e hash
originali. Le verifiche degli inventari sorgente storici possono richiedere
il checkout originale completo; la nuova disposizione non riscrive quei record.

Le istruzioni per gli agenti sono in [AGENTS.md](AGENTS.md).
