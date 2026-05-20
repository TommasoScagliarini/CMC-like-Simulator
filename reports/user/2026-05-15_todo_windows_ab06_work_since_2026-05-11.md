# TODO Windows alignment per lavoro AB06 dal 2026-05-11

## Scopo del controllo

Questo controllo estende il report Windows precedente sullo sweep PID locale e
copre il lavoro fatto dal `2026-05-11` in poi:

- bundle `AB06_SEASEA_Threadmill`;
- conversione EPIC `.mat -> OpenSim`;
- healthy overlay;
- gait-cycle/GRF filter;
- modelli stiffness `321/500` e `321/500 matched dynamics`;
- outer-loop PID e sweep;
- plot naming `MM_DD_AAAA_iter`;
- pipeline generale `ABx_SEASEA`.

## Stato verificato su macOS

Sono stati ricontrollati i daily/report dal `2026-05-11` al `2026-05-15` e i
file principali coinvolti nella pipeline.

Verifica statica eseguita:

```bash
python -m py_compile \
  config.py \
  main.py \
  model_loader.py \
  output.py \
  prosthesis_controller.py \
  simulation_runner.py \
  path_resolver.py \
  setup_io.py \
  tools/build_abx_seasea_pipeline.py \
  scripts/run_opensim_sea_pipeline.py \
  scripts/run_measured_grf_window_tests.py \
  plot/plotter.py \
  validation/outer_pid_gain_sweep.py \
  validation/outer_pid_local_sweep.py \
  validation/validate_sim_results.py
```

Esito: `OK`.

Verifica sweep locale:

```bash
python validation/outer_pid_local_sweep.py --dry-run
```

Esito: `OK`, con `total_jobs=133`.

Durante il controllo sono state applicate due piccole correzioni di portabilita':

- `tools/build_abx_seasea_pipeline.py` e `scripts/run_opensim_sea_pipeline.py`
  ora scrivono nei log comandi quotati correttamente anche quando il path del
  repo contiene spazi;
- il commento di `config.py` su `T_control=0.001` ora indica correttamente `1 ms`.

## Blocco principale per Windows

La configurazione corrente usa il plugin feedforward:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff
```

Su macOS e' presente:

```text
plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib
```

Nel repo macOS non risulta presente la DLL Windows equivalente:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

E' invece presente la vecchia DLL:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll
```

Questa non e' equivalente per i risultati recenti. La macchina Windows deve
compilare o ricevere la DLL `ff` prima di considerarsi allineata.

## Dati e artefatti da sincronizzare

Obbligatori per riprodurre AB06 e lanciare quick-smoke/sweep:

```text
models/AB06_SEASEA_Threadmill/
results/_stiff321_500_ab06_pd_full/
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

Obbligatori se Windows deve ricostruire o validare anche la pipeline ABx:

```text
models/AB06-raw/
models/AB07-raw/
models/AB08-raw/
models/AB06_SEASEA-raw/
models/SEASEA/data/
```

Utili ma pesanti se si vuole riprodurre esattamente l'analisi gia' fatta:

```text
results/_grf_filtered_ab06_pd_full/
results/_stiff321_500_matched_dynamics_ab06_pd_full/
results/_outer_pid_gain_sweep_20260514_223838/
plot/05_15_2026_1/
```

Nota dimensioni: lo sweep completo `results/_outer_pid_gain_sweep_20260514_223838`
occupa circa `19 GB`; non serve per lanciare il nuovo sweep locale, serve solo
per rianalisi/plot del vecchio sweep.

## TODO Windows

### 1. Ambiente Python/OpenSim

Da PowerShell o Anaconda Prompt:

```powershell
cd "C:\path\to\CMC-like-Simulator - Claude"
conda activate envCMC-like
python -c "import sys, opensim; print(sys.executable); print('opensim OK')"
```

Il `sys.executable` deve puntare a `envCMC-like`, non a `base`.

### 2. Plugin feedforward

Verificare:

```powershell
Test-Path .\plugins\SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

Se restituisce `False`, compilare/copiare la DLL `ff`.

La variante corretta e' quella usata dalle run recenti, con comando motore:

```text
tau_input = tau_ref + Kp * (tau_ref - tau_spring) - Kd * omega_m
```

### 3. Compilazione statica

```powershell
python -m py_compile `
  config.py `
  main.py `
  model_loader.py `
  output.py `
  prosthesis_controller.py `
  simulation_runner.py `
  path_resolver.py `
  setup_io.py `
  tools\build_abx_seasea_pipeline.py `
  scripts\run_opensim_sea_pipeline.py `
  scripts\run_measured_grf_window_tests.py `
  plot\plotter.py `
  validation\outer_pid_gain_sweep.py `
  validation\outer_pid_local_sweep.py `
  validation\validate_sim_results.py
```

Esito atteso: nessun output e return code `0`.

### 4. Validazione discovery ABx

Prima di ricostruire bundle o lanciare MATLAB:

```powershell
python tools\build_abx_seasea_pipeline.py --subject AB06 --task treadmill --trial treadmill_01_01 --validate-only
python tools\build_abx_seasea_pipeline.py --subject AB07 --task treadmill --trial treadmill_01_01 --validate-only
python tools\build_abx_seasea_pipeline.py --subject AB08 --task treadmill --trial treadmill_01_01 --validate-only
```

Se `python_has_opensim` risultasse `false`, rilanciare forzando l'interprete
dell'ambiente:

```powershell
python tools\build_abx_seasea_pipeline.py --subject AB06 --task treadmill --trial treadmill_01_01 --validate-only --python "$env:CONDA_PREFIX\python.exe"
```

Controlli attesi:

- soggetto raw trovato;
- trial statico trovato;
- trial treadmill trovato;
- plugin trovato;
- support data trovato;
- `python_has_opensim=true`.

### 5. Smoke AB06 runtime

Non lanciare subito full run. Prima:

```powershell
python main.py `
  --setup models\AB06_SEASEA_Threadmill\AB06_SEASEA_setup.xml `
  --model models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500.osim `
  --t-start 13.1638 `
  --t-end 13.1938 `
  --output-dir results\_windows_smoke_ab06_stiff321_500 `
  --filter-grf `
  --log
```

Esito atteso:

```text
status=complete
sea_forward_mode=plugin
enable_grf_contact_filter=True
```

nel file:

```text
results/_windows_smoke_ab06_stiff321_500/sim_output_run_status.txt
```

### 6. Smoke plotter

Dopo lo smoke runtime:

```powershell
python plot\plotter.py `
  --results-dir results\_windows_smoke_ab06_stiff321_500 `
  --setup models\AB06_SEASEA_Threadmill\AB06_SEASEA_setup.xml `
  --model models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500.osim `
  --reference models\AB06_SEASEA_Threadmill\data\IK_results_AB06_SEASEA.mot
```

Output atteso:

```text
plot/MM_DD_2026_iter/
```

Il nuovo formato dei plot e' intenzionale. I riferimenti nei vecchi report a
`DD_MM_AAAA - iter` sono storici e possono risultare non piu' corrispondenti al
nome attuale delle cartelle.

### 7. Sweep PID locale

Dry-run:

```powershell
python validation\outer_pid_local_sweep.py --dry-run
```

Atteso:

```text
knee_candidates=27
ankle_candidates=27
full_candidates=25
total_jobs=133
```

Quick-smoke:

```powershell
python validation\outer_pid_local_sweep.py `
  --quick-smoke `
  --workers 2 `
  --sweep-root results\_outer_pid_local_sweep_quick_smoke_windows
```

Atteso:

```text
total_jobs=2
stage1_knee OK
stage1_ankle OK
quick smoke complete; full sweep not launched
```

La progress line deve includere anche:

```text
overall=... total_elapsed=... total_eta=...
```

### 8. Pipeline ABx completa

Solo dopo i punti precedenti, testare una build su un soggetto non AB06 senza
RRA/full sweep:

```powershell
python tools\build_abx_seasea_pipeline.py `
  --subject AB07 `
  --task treadmill `
  --trial treadmill_01_01 `
  --skip-opensim-ik `
  --skip-smoke-test `
  --force
```

Poi, se passa, ripetere con smoke test attivo su finestra corta o con le opzioni
standard desiderate.

## Note non bloccanti

- Esiste `models/AB06_SEASEA_Threadmill/AB06_SEASEA_adjuted.osim` con typo nel
  nome; non risulta referenziato dai comandi recenti. Non blocca Windows.
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml` punta al modello base
  `AB06_SEASEA.osim`; per le run stiffness/PID serve continuare a passare
  esplicitamente `--model AB06_SEASEA_stiff321_500.osim`.
- Le cartelle plot sono state rinominate al formato `MM_DD_AAAA_iter`; i report
  vecchi rimangono storici, non una sorgente di path runtime.

## Criterio finale

La macchina Windows e' allineata quando passano:

1. `import opensim`;
2. presenza e caricamento della DLL `ff`;
3. `py_compile`;
4. discovery `AB06/AB07/AB08 --validate-only`;
5. smoke AB06 runtime;
6. smoke plotter con naming nuovo;
7. dry-run e quick-smoke dello sweep PID locale.

Fino a quel punto: non lanciare full run lunghi o sweep completi.
