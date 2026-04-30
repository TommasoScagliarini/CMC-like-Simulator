# Report - Setup XML GUI e bundle model paths

## Problema

Il progetto era ancora legato a due limiti pratici:

1. il layout dei modelli assumeva path flat o semi-hardcoded, mentre la nuova
   tree organizza ogni modello in un bundle dedicato con `.osim` in root e file
   di supporto dentro `data/`;
2. l'avvio della simulazione dipendeva da `config.py` o da override CLI, senza
   un file di setup esplicito in stile OpenSim e senza un flusso GUI per
   selezionare i file necessari.

Questo rendeva piu fragile il caricamento dei path e meno usabile il simulatore
per run interattivi.

## Soluzione

Sono stati introdotti due livelli complementari.

### 1. Risoluzione bundle-aware dei path

- Aggiunto `path_resolver.py` come resolver centralizzato dei path del
  simulatore.
- `SimulatorConfig` usa ora `model_bundle_dir` come unita primaria e
  `model_file` come filename nel bundle oppure come path esplicito.
- I file `kinematics`, `ExternalLoads` e `CMC_Actuators` vengono risolti
  rispetto al bundle attivo.
- I consumer principali del simulatore, del plotter e dei validator leggono
  i path risolti invece di affidarsi a stringhe flat nel repo.
- `.gitignore` e stato corretto per ignorare solo il contenuto di
  `models/**/data/**`, mantenendo tracciabili i `.osim`.

### 2. Setup XML + ultimo setup persistito

- Aggiunto `setup_io.py` per:
  - scrivere un setup XML custom in stile OpenSim;
  - leggerlo e validarlo;
  - salvare e rileggere l'ultimo setup caricato da
    `.simulator_last_setup.json`.
- Aggiunto `create_setup.py`, un wizard `tkinter` con quattro picker per:
  - modello `.osim`
  - kinematics `.sto/.mot`
  - external loads `.xml`
  - reserve actuators `.xml`
- `main.py` ora supporta:
  - `python main.py --setup`
  - `python main.py --setup path/to/setup.xml`
  - `python main.py` con riuso automatico dell'ultimo setup valido
- Aggiunti anche gli override CLI mancanti:
  - `--external-loads`
  - `--reserve-actuators`

Il setup XML salva solo i quattro file richiesti, con path repo-relative quando
possibile e assoluti se i file sono fuori repo.

## Strategia

La modifica e stata costruita cercando di preservare la pipeline esistente:

- il caricamento high-level continua a passare per `SimulatorConfig`;
- il bundle del modello viene derivato automaticamente dalla cartella del
  modello selezionato nel setup XML;
- gli override CLI espliciti mantengono precedenza sul setup;
- in presenza di override diretti come `--model-bundle` o `--model`, `main.py`
  salta l'auto-load del setup XML per non rompere i flussi batch o gli sweep.

In questo modo la nuova UX interattiva convive con i flussi gia usati per test,
validator e script automatici.

## File modificati

- `path_resolver.py`
- `config.py`
- `main.py`
- `model_loader.py`
- `kinematics_interpolator.py`
- `visualize.py`
- `plot/plotter.py`
- `validation/validate_sim_results.py`
- `validation/hpf_noise_metric.py`
- `validation/cmc_strict_metrics.py`
- `validation/outer_gain_sweep.py`
- `validation/sea_driver_sweep.py`
- `validation/sea_inner_10020_sweep.py`
- `validation/_tmp_sea_parameter_sweep.py`
- `setup_io.py`
- `create_setup.py`
- `.gitignore`

## Test e verifiche eseguite

### Verifiche sul refactor bundle-aware

- `conda run -n envCMC-like python -m py_compile config.py path_resolver.py model_loader.py main.py kinematics_interpolator.py visualize.py plot/plotter.py validation/validate_sim_results.py validation/hpf_noise_metric.py validation/cmc_strict_metrics.py validation/outer_gain_sweep.py validation/sea_driver_sweep.py validation/sea_inner_10020_sweep.py validation/_tmp_sea_parameter_sweep.py`
- `conda run -n envCMC-like python main.py --t-start 4.26 --t-end 4.261 --output-dir results/_bundle_smoke_current_cfg`
  - passato, con caricamento corretto da `models/SEASEA/...`
- `conda run -n envCMC-like python main.py --model-bundle models/SEASEA --model "Adjusted_SEASEA - Copia_tuned.osim" --t-start 4.26 --t-end 4.261 --output-dir results/_bundle_smoke_explicit`
  - passato
- `conda run -n envCMC-like python plot/plotter.py --model-bundle models/SEASEA --model "Adjusted_SEASEA - Copia_tuned.osim" --results-dir results/_bundle_smoke_current_cfg`
  - passato, healthy letto da `models/SEASEA/data/healthy`
- `conda run -n envCMC-like python validation/validate_sim_results.py --results-dir results/_bundle_smoke_current_cfg --model-bundle models/SEASEA --model "Adjusted_SEASEA - Copia_tuned.osim"`
  - il validator parte correttamente col nuovo layout e arriva ai check
    numerici; l'exit code finale riflette `FAIL` fisici sulla smoke run a 1
    step, non errori di path
- `git check-ignore -v models/SEASEA/data/CMC_Actuators.xml`
  - conferma che il contenuto di `data/` e ignorato
- `git check-ignore -v models/SEASEA/Adjusted_SEASEA - Copia_tuned.osim`
  - conferma che il `.osim` non e ignorato

### Verifiche sulla feature setup XML

- `python -m py_compile setup_io.py create_setup.py main.py`
- `conda run -n envCMC-like python -m py_compile setup_io.py create_setup.py main.py`
- round-trip reale con `setup_io.py`:
  - creazione del setup XML
  - rilettura del setup XML
  - scrittura e rilettura di `.simulator_last_setup.json`
- `conda run -n envCMC-like python main.py --setup results/_setup_feature_smoke/setup_sample.xml --t-start 4.26 --t-end 4.261 --output-dir results/_setup_run_explicit`
  - passato
- `conda run -n envCMC-like python main.py --t-start 4.26 --t-end 4.261 --output-dir results/_setup_run_last`
  - passato, con riuso dell'ultimo setup persistito
- `conda run -n envCMC-like python main.py --model-bundle models/SEASEA --model "Adjusted_SEASEA - Copia_tuned.osim" --t-start 4.26 --t-end 4.261 --output-dir results/_setup_run_direct_override`
  - passato, confermando che gli override diretti bypassano l'auto-load del
    setup XML
- `python -c "import create_setup, setup_io; print('setup-gui-import-ok')"`
  - import OK del wizard GUI

## Stato finale

- Il simulatore puo ora essere lanciato con un setup XML selezionato via GUI o
  riusare automaticamente l'ultimo setup valido.
- I path dei modelli e dei file di supporto sono coerenti con la nuova
  organizzazione a bundle.
- I flussi batch esistenti restano utilizzabili tramite override CLI espliciti.
- La GUI `tkinter` del wizard e stata integrata ma non e stata cliccata in modo
  end-to-end automatico; la logica sottostante di validazione, salvataggio e
  rilettura e stata invece verificata con smoke test reali.
