# TensorBoard non parte: pkg_resources rimosso da setuptools 82 (fix env)

Data: 2026-06-09

## Problema

Lanciando TensorBoard sul run notturno il comando falliva subito:

```text
C:\...\conda.exe run -n envCMC-rllib python -m tensorboard.main --logdir "runs\baseline_mlp_hybrid_win\tensorboard" --port 6006
...
  File ".../tensorboard/default.py", line 30, in <module>
    import pkg_resources
ModuleNotFoundError: No module named 'pkg_resources'
```

Non era TensorBoard mancante (è installato, 2.20.0) ma una **dipendenza interna
rotta**.

## Causa

L'env `envCMC-rllib` (Python 3.12.13) aveva **setuptools 82.0.1**. A partire da
**setuptools >= 81** il modulo `pkg_resources` è deprecato e da **82.0.0 è
rimosso**. TensorBoard 2.20 lo importa ancora in `default.py`, quindi non parte
più con setuptools 82.

## Strategia di diagnosi e fix

1. Riprodotto l'errore con `python -c "import pkg_resources"` → `ModuleNotFound`.
2. Verificato che setuptools **fosse presente ma troppo recente** (82.0.1): è la
   versione, non l'assenza del pacchetto, la causa.
3. Riportato setuptools a una versione che include ancora `pkg_resources`
   (`< 81`), senza toccare altri pacchetti.
4. Verificato gli import e poi un **boot reale** del server (HTTP + dati), infine
   chiuso il server di test per liberare la porta.

## Soluzione

Downgrade mirato di setuptools nell'env (runtime-safe, non tocca torch/ray/opensim):

```text
conda run -n envCMC-rllib python -m pip install "setuptools<81"   ->  setuptools 80.10.2
```

Il fix vale per qualsiasi `--logdir`. Resta solo un `UserWarning` di deprecazione
all'import di `pkg_resources`, innocuo.

## File modificati

```text
Trajectory Generator/baseline_MLP/commands.txt   (sezione 9: prerequisito setuptools<81 Win+macOS + comando single-run)
```

Nessuna modifica al codice del simulatore o della pipeline: è un fix d'ambiente.
La modifica `pip` agisce solo dentro `envCMC-rllib`.

## Test e verifiche eseguite

- `import pkg_resources` nell'env → OK (solo `UserWarning` di deprecazione).
- `from tensorboard import default, program` → OK.
- Avvio reale del server in background su `127.0.0.1:6006`:
  - `GET http://127.0.0.1:6006/` → **HTTP 200**;
  - `GET /data/plugin/scalars/tags` → JSON con i tag reali del run notturno
    (`reward_loss/truncated`, `reward/oob_term`, `env_runners/...`).
- Server di test **terminato** e porta 6006 liberata (verificato con
  `Get-NetTCPConnection -LocalPort 6006` → free).

## TODO chiusi oggi

- TensorBoard non parte per `pkg_resources` mancante: risolto con
  `setuptools<81` su Windows; comando documentato in `commands.txt`.

## TODO aperti e propagati

### Allineamento macOS (NUOVO)
- **Allineare l'env `envCMC-rllib` su macOS arm64**: applicare lo stesso fix
  `conda run -n envCMC-rllib python -m pip install "setuptools<81"` (l'env Mac,
  clonato con la stessa ricetta, avrà anch'esso setuptools 82 e lo stesso errore
  `pkg_resources` su TensorBoard). Verificare lì il boot di TensorBoard.
- Promemoria collegato (già tracciato): prima del training/inference ibrido su
  Mac, **ricompilare/riconfermare il plugin onlineGRF `.dylib` su arm64**.

### Resto dei TODO (propagati)
Vedere l'elenco completo e aggiornato in
[[2026-06-09_anti_stallo_timeout_iterazione_guardia_wall_time_cli_progress]]
(training ibrido da rilanciare, analisi `.sto` del rollout, divergenze
`joint_divergence_pros_knee_angle` / saturazione knee SEA / fallback Static
Optimization, contatto online/COP, housekeeping repo, traiettorie ex-novo,
validazione SNN/PPO, knowledge base letteratura, controllo SEA storici).
