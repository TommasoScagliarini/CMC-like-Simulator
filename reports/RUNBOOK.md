# Runbook operativo

**Ultimo aggiornamento:** 2026-10-29.

Leggere prima di lanciare simulazioni, rollout, training, test o build dei
plugin. Aggiornare nella stessa sessione in cui cambiano un comando,
l'ambiente, un preset o una macchina, senza aspettare `end_day`.

Questo file non ricopia la documentazione: indica dove si trova e raccoglie
solo le ricette in uso, i dettagli delle macchine e gli errori ricorrenti.
Prima di qualsiasi lancio vale il vincolo G01 di
[PROJECT_STATE.md](PROJECT_STATE.md): serve il via esplicito dell'utente.

## Dove sono i comandi

| Cosa | Riferimento |
|---|---|
| Training, rollout, resume, TensorBoard, reward custom, smoke test, diagnostica GRF, setup dell'ambiente | [commands.txt](<../Trajectory Generator/baseline_MLP/commands.txt>), sezioni A–H |
| Opzioni, precedenza delle configurazioni, warm start, output dei run | [README baseline MLP](<../Trajectory Generator/baseline_MLP/README.md>) |
| Simulatore CMC-like (`main.py`, `--validate`, `visualize.py`) | [README del simulatore](<../CMC-like Simulator/README.md>) |
| Test | [validation/README.md](../validation/README.md) |
| Build del plugin SEA e preparazione dei modelli | [tools/README.md](../tools/README.md); sorgenti nella repository esterna `SEA_plugin_core - agent` |
| Build del plugin GRF | [online_GRF/sources/README.md](../online_GRF/sources/README.md) |

## Ambiente

- Ambiente conda `envCMC-rllib`. Versioni registrate il 2026-10-26 su macOS
  arm64: Python 3.10.20, NumPy 2.2.6, SciPy 1.15.2, Torch 2.10.0, Ray 2.55.1,
  OpenSim 4.5.2, solo CPU.
- Lanciare dalla radice della repository, con i percorsi tra virgolette e
  separati da `/`, anche in PowerShell.
- Con `conda run` usare sempre `--no-capture-output`: senza, stdout e log
  restano vuoti fino alla fine del processo.
- Se su macOS compare l'errore OpenMP/libomp, anteporre
  `KMP_DUPLICATE_LIB_OK=TRUE` (vedere commands.txt, sezione H).

## Macchine

| | macOS arm64 (questa) | Windows x86 |
|---|---|---|
| Radice | `/Users/tommy/Documents/CMC-like-Simulator - Claude` | `C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude` |
| conda | `conda` | `C:\Users\tomma\anaconda3\Scripts\conda.exe` |
| CPU | `hw.ncpu` = 15 | da annotare |
| Plugin | `.dylib` in `plugins/SEA/` e `plugins/online_grf_contact/` | `.dll` da compilare sulla macchina Windows |
| Verifiche | prove native delle campagne recenti | verifica live ancora aperta per le campagne recenti |

Training: **12 EnvRunner e 13 CPU Ray**, come nel preset
`training_exnovo_cfg.yaml`, anche al riavvio di un training (decisione
dell'utente del 2026-10-29). Su un'altra macchina chiedere prima di cambiarli.

## Ricette correnti

Rollout deterministico di best47 (passare sempre `--checkpoint`):

```sh
python "Trajectory Generator/baseline_MLP/rollout_eval.py" --checkpoint "Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best"
python "plot/plotter.py" --mlp
```

Campagna transizioni di velocità e fermo del 2026-10-26: casi `control`,
`stop_restart` e `speed_changes`. Il `--suffix` evita di sovrascrivere i
risultati esistenti. Analisi e diagnosi sono descritte nei
[report del 26/10](user/2026-10-26_diagnosi_fermo_ripartenza_best47.md).

```sh
python "Trajectory Generator/validation/speed_transitions_2026_10_26/run_transition.py" --case stop_restart --suffix _repeat
python "Trajectory Generator/validation/speed_transitions_2026_10_26/render_canonical.py" --case <cartella_del_caso>
```

Suite di test:

```sh
python -B -m pytest -q -p no:cacheprovider validation/test/
```

## Errori ricorrenti

- **Rollout senza `--checkpoint`**: sceglie il run valido modificato più di
  recente in `runs/training/`, che non è necessariamente best47.
- **Il preset è la configurazione di best47**: lo snapshot di best47 è stato
  rimosso, quindi modificare `training_exnovo_cfg.yaml` cambia anche i suoi
  rollout futuri.
- **Il training di default non carica i pesi di best47**: senza warm start o
  `--resume-from` costruisce un Algorithm nuovo.
- **I plugin C++ non si ricompilano da soli**: dopo una modifica ai sorgenti
  SEA o GRF la libreria caricata resta quella vecchia. Il sintomo tipico è
  `RuntimeError: Property '<nome>' not present in Object ...`.
- **`summary.json` si scrive solo a fine run**: può mostrare l'esito di un run
  precedente nella stessa cartella. Per l'avanzamento usare
  `watchdog_state.json` o TensorBoard.
- **I plot canonici** vanno in una sottocartella di `plot/` nella radice, mai
  dentro le cartelle dei run.
