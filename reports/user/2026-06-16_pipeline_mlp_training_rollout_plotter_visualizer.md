# Pipeline MLP: training, rollout, plotter e visualizer

Data: 2026-06-16

## Problema

La pipeline baseline MLP richiedeva ancora comandi espliciti e path manuali per
training, rollout, plot e visualizzazione. Inoltre i risultati non erano
organizzati in categorie chiare, il plotter non leggeva automaticamente
l'ultimo rollout MLP e il visualizer aveva un salvataggio video fragile:
`--save-video` usava una cattura macOS-only (`screencapture`) e non esisteva il
flag breve `--save`.

## Soluzione

E' stata applicata la stessa filosofia no-flag a tutta la pipeline:

- training: default config e output automatici;
- rollout: checkpoint e output automatici dall'ultimo training valido;
- plotter: input automatico dall'ultimo rollout valido;
- visualizer: input automatico dall'ultimo rollout valido e video salvati sotto
  `Trajectory Generator/runs/visualize`.

La struttura dei risultati ora separa le categorie principali:

- `Trajectory Generator/runs/training`
- `Trajectory Generator/runs/rollout`
- `Trajectory Generator/runs/visualize`
- `plot/MM_DD_YYYY_N`

## Modifiche training script

File principali: `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`,
`Trajectory Generator/baseline_MLP/training_cfg.yaml`,
`Trajectory Generator/baseline_MLP/commands.txt`,
`Trajectory Generator/baseline_MLP/README.md`.

- Il comando raccomandato e' diventato:

  ```powershell
  python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py"
  ```

- La config default effettiva e' `Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- `training_cfg.yaml` e' stato aggiornato con la configurazione V4 imitation
  obs-target, C2 Butterworth 4 Hz.
- Se `--output-dir` non viene passato, il training salva sotto
  `Trajectory Generator/runs/training`.
- Il nome automatico del run e':

  ```text
  MLP_[strategy]_training_[MM-DD-YYYY]
  ```

  dove `strategy` deriva da `reward_mode`:

  - `imitation` -> `MLP_imitation_training_MM-DD-YYYY`
  - `ex_novo` -> `MLP_ExNovo_training_MM-DD-YYYY`

- Aggiunto `--name` per suffissi custom, con `_` inserito automaticamente se manca.
- Aggiunta gestione collisioni con `_02`, `_03`, ecc.
- `--config` e `--output-dir` restano override compatibili.

## Modifiche rollout script

File principali: `Trajectory Generator/baseline_MLP/rollout_eval.py`,
`Trajectory Generator/baseline_MLP/commands.txt`,
`Trajectory Generator/baseline_MLP/README.md`.

- Il comando raccomandato e' diventato:

  ```powershell
  python "Trajectory Generator\baseline_MLP\rollout_eval.py"
  ```

- Se `--checkpoint` non viene passato, lo script seleziona automaticamente
  l'ultimo training valido sotto `Trajectory Generator/runs/training` con
  `rl_module_best`.
- I run con `summary.json` e `ok: false` vengono ignorati.
- Se non ci sono candidati validi, l'errore suggerisce di passare
  `--checkpoint`.
- L'auto-load di `training_cfg.resolved.yaml` resta attivo anche per il
  checkpoint selezionato automaticamente.
- Il rollout salva sotto `Trajectory Generator/runs/rollout`.
- Il nome automatico deriva dal training sorgente:

  ```text
  MLP_imitation_training_06-16-2026 -> MLP_imitation_rollout_06-16-2026
  ```

- Per nomi legacy senza `training`, il fallback e' `<training_run_name>_rollout`.
- Aggiunti `--name` e collision handling `_02`, `_03`, ecc.
- `--record-outputs` e' diventato default `True`.
- Aggiunto `--no-record-outputs` per rollout leggeri senza `.sto`.
- `--checkpoint` e `--output-dir` restano override manuali.

## Modifiche plotter script

File principale: `plot/plotter.py`.

- Aggiunto:

  ```powershell
  python plot/plotter.py --mlp
  ```

- In modalita' `--mlp`, il plotter prende automaticamente l'ultimo rollout
  valido in `Trajectory Generator/runs/rollout`.
- Usa automaticamente:

  - `results_dir = <latest_rollout>/sim_outputs`
  - `prefix = rollout_episode`
  - eventi `rollout_episode_gait_events.csv`, se presenti

- Gli override manuali restano disponibili: `--results-dir`, `--prefix`,
  `--events`, `--healthy-dir`.
- I PNG continuano a essere salvati in cartelle `plot/MM_DD_YYYY_N`.
- La numerazione delle cartelle plot ora considera il numero subito dopo
  `MM_DD_YYYY_`, anche se il nome cartella ha un suffisso descrittivo.
  Esempio:

  ```text
  06_16_2026_9 - imititation_FullObs -> indice 9
  ```

- Aggiunta Figure 7:

  ```text
  07_mlp_policy_vs_sound_leg_error.png
  ```

- Figure 7 ora mostra:

  - riga 1: sovrapposizione tra policy C2 servita e target sound-leg shiftato;
  - riga 2: sound leg raw/non shiftata da `rollout_episode_states.sto`;
  - riga 3: errore `shifted sound leg target - policy C2`.

- La policy nella Figure 7 usa la reference C2 effettivamente servita dal filtro,
  non i knot raw rumorosi del trace.
- Corretto il segno dell'ankle nella Figure 7: ankle invariato, knee con la
  convenzione grafica gia' usata nel plotter.
- La label del plugin non usa piu' solo il nome configurato in `config.py`.
  Ora risolve il setup/modello effettivo e mostra correttamente il plugin PI
  quando i parametri SEA indicano controllo PI.

## Modifiche visualizer script

File principale: `visualize.py`.

- Aggiunto:

  ```powershell
  python visualize.py --mlp
  ```

- In modalita' `--mlp`, il visualizer prende automaticamente l'ultimo rollout
  valido in `Trajectory Generator/runs/rollout`.
- Se `--sto` non e' passato, usa:

  ```text
  <latest_rollout>/sim_outputs/rollout_episode_kinematics.sto
  ```

- Se `rollout_summary.json` contiene `setup_xml_path`, il visualizer carica
  automaticamente il setup e usa modello/bundle coerenti col rollout.
- `--sto`, `--model-bundle`, `--model`, `--geometry-dir`, `--t-start`,
  `--t-end`, `--speed`, `--loop` restano override manuali.
- Aggiunto `--save` come alias di `--save-video`.
- I video MLP vengono salvati in:

  ```text
  Trajectory Generator/runs/visualize
  ```

- Il nome automatico deriva dal rollout sorgente:

  ```text
  ..._rollout -> ..._visualize
  ```

- Aggiunti `--output-dir` e `--name` per output video manuale/suffisso custom.
- Aggiunta gestione collisioni `_02`, `_03`, ecc.
- Il salvataggio video ora e' platform-aware:

  - macOS: continua a usare `screencapture`;
  - Windows: usa `PIL.ImageGrab`;
  - fallback: cattura schermo intero se non trova la finestra Simbody.

- `ffmpeg` viene verificato prima della registrazione.
- Se vengono catturati 0 frame, lo script non chiama `ffmpeg` e restituisce un
  errore chiaro.
- `opensim` viene importato lazy: `python visualize.py --help` funziona anche
  fuori dall'ambiente OpenSim.
- Corretto un bug collaterale: il file importava `ElementTree` come `ETp` ma
  usava `ET.parse`.

## Strategia

La strategia e' stata evitare nuovi entrypoint separati e rendere intelligenti
gli entrypoint gia' esistenti:

- default semplici per l'uso quotidiano;
- override manuali mantenuti per debug e riproducibilita';
- artefatti categorizzati per fase della pipeline;
- resolver basati sugli artifact gia' scritti (`summary.json`,
  `rollout_summary.json`, `training_cfg.resolved.yaml`);
- test dei resolver senza lanciare training/rollout quando possibile.

## Test e verifiche eseguite

- `python -m py_compile` su:

  - `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
  - `Trajectory Generator/baseline_MLP/training_config.py`
  - `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - `plot/plotter.py`
  - `visualize.py`

- Test resolver training:

  - default output sotto `Trajectory Generator/runs/training`;
  - naming da `reward_mode`;
  - collisioni `_02`;
  - suffisso `--name`;
  - precedenza di `--output-dir`.

- Test resolver rollout:

  - selezione ultimo training valido;
  - esclusione run falliti;
  - naming `training -> rollout`;
  - suffisso `--name`;
  - precedenza `--checkpoint` e `--output-dir`;
  - `--no-record-outputs`.

- Test plotter:

  - `python plot/plotter.py --mlp`;
  - generazione Figure 7;
  - correzione segno ankle;
  - uso reference C2 servita;
  - overlay policy vs target shiftato;
  - riga 2 raw sound leg;
  - fix numerazione cartelle plot con suffissi descrittivi.

- Test visualizer:

  - `python visualize.py --help`;
  - resolver `--mlp` e path automatici;
  - controllo output automatico sotto `Trajectory Generator/runs/visualize`;
  - smoke test reale con GUI:

    ```powershell
    C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-like python visualize.py --mlp --t-start 13 --t-end 13.3 --save --video-fps 5
    ```

  - MP4 creato in:

    ```text
    Trajectory Generator/runs/visualize/baseline_mlp_imit_v4_c2_4hz_obs_target_visualize/161226_16062026.mp4
    ```

- `git diff --check` eseguito sui file modificati; presenti solo warning
  CRLF/LF.

## File modificati principali

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `Trajectory Generator/baseline_MLP/README.md`
- `plot/plotter.py`
- `visualize.py`

## Note

- Il comando visualizer MLP no-flag e' `python visualize.py --mlp`, non
  `python visualize.py`, per preservare il comportamento storico del visualizer
  root.
- `--save-video` resta compatibile, ma il comando raccomandato e' `--save`.
- Il primo smoke test video ha creato la cartella
  `baseline_mlp_imit_v4_c2_4hz_obs_target_visualize`; il prossimo salvataggio
  automatico usera' `_02`.
