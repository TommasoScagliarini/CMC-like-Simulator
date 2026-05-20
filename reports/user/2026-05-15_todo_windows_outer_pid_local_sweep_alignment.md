# TODO Windows alignment per sweep PID locale - 2026-05-15

## Problema

Il lavoro fatto su macOS ha preparato e verificato:

- sweep PID locale attorno al best candidate AB06;
- metriche aggiuntive per giudicare non solo il tracking cinematico, ma anche
  dinamica del motor driver;
- ETA totale dello sweep oltre all'ETA dello stage;
- naming plot nel formato `MM_DD_AAAA_iter`.

La parte Python e' stata resa il piu' possibile cross-platform, ma il runtime
OpenSim/plugin non puo' essere certificato da macOS per Windows. In particolare,
la configurazione corrente usa il plugin:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff
```

Su macOS e' presente:

```text
plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib
```

Nel repo locale macOS non risulta invece presente la DLL Windows:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

Quindi la macchina Windows deve eseguire un passaggio di allineamento nativo
prima di lanciare lo sweep reale.

## Stato macOS gia' verificato

File aggiornati/preparati:

- `validation/outer_pid_gain_sweep.py`
- `validation/outer_pid_local_sweep.py`
- `plot/plotter.py`
- `reports/user/2026-05-15_pid_sweep_best_candidate_plot_naming.md`

Verifiche macOS eseguite:

```bash
python -m py_compile validation/outer_pid_gain_sweep.py validation/outer_pid_local_sweep.py plot/plotter.py
python validation/outer_pid_local_sweep.py --dry-run
python validation/outer_pid_local_sweep.py --quick-smoke --workers 2 --sweep-root results/_outer_pid_local_sweep_quick_smoke_windows_ready
```

Esito:

```text
py_compile: OK
dry-run: OK
quick-smoke: OK
full sweep: non lanciato
```

Lo sweep locale pronto ha questa griglia:

```text
knee Kp: 260, 300, 340
knee Kd: 22, 26, 30
knee Ki: 40, 80, 120

ankle Kp: 650, 750, 850
ankle Kd: 1.5, 2.0, 2.5
ankle Ki: 180, 240, 300
```

Job previsti:

```text
stage1_knee:   27 candidati x 2 finestre = 54 job
stage1_ankle:  27 candidati x 2 finestre = 54 job
full combo:    25 candidati              = 25 job
totale:                                    133 job
```

La progress line ora include sia ETA di stage sia ETA totale:

```text
[stage1_knee] ... eta=... last=OK ... | overall=42/133 (31.58%) total_elapsed=... total_eta=...
```

## Migliorie cross-platform gia' applicate

### Interprete Python

`validation/outer_pid_local_sweep.py` ora:

- usa `CONDA_PREFIX` solo se l'ambiente attivo si chiama `envCMC-like`;
- su macOS usa come fallback `/opt/anaconda3/envs/envCMC-like/bin/python`, se
  presente;
- su Windows, se si lancia dal prompt con `conda activate envCMC-like`, usa il
  `python.exe` dell'ambiente attivo;
- permette comunque override esplicito con `--python`.

### Subprocess

`validation/outer_pid_gain_sweep.py` lancia le simulazioni con lista di argomenti
e `shell=False`, quindi i path con spazi non vengono spezzati dalla shell.

Il comando scritto in `console.txt` ora viene quotato in modo leggibile:

- `shlex.join()` su macOS/Linux;
- `subprocess.list2cmdline()` su Windows.

### Plot naming

`plot/plotter.py` ora crea nuove cartelle nel formato:

```text
MM_DD_AAAA_iter
```

Non sono rimasti riferimenti di codice al vecchio pattern `DD_MM_AAAA - iter`.

## TODO sulla macchina Windows

### 1. Sincronizzare codice e dati

Portare su Windows gli aggiornamenti di codice:

```text
validation/outer_pid_gain_sweep.py
validation/outer_pid_local_sweep.py
plot/plotter.py
reports/user/2026-05-15_pid_sweep_best_candidate_plot_naming.md
reports/user/2026-05-15_todo_windows_outer_pid_local_sweep_alignment.md
```

Verificare che siano presenti anche i dati necessari allo sweep locale:

```text
models/AB06_SEASEA_Threadmill/
results/_stiff321_500_ab06_pd_full/
```

Il secondo path e' necessario per calcolare la baseline:

```text
baseline mean_pros_rmse_deg = 4.270496991108777
```

Se si vuole confrontare anche col best sweep precedente, sincronizzare anche:

```text
results/_outer_pid_gain_sweep_20260514_223838/
plot/05_15_2026_1/
```

### 2. Allineare il plugin `ff` su Windows

Controllare nella root del repo Windows:

```powershell
Test-Path .\plugins\SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

Se restituisce `False`, compilare/copiare la DLL Windows della variante `ff`
del plugin, cioe' quella con legge non-impedance:

```text
tau_input = tau_ref + Kp * (tau_ref - tau_spring) - Kd * omega_m
```

La DLL deve trovarsi qui:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll
```

Nota: usare la vecchia DLL senza suffisso `ff` non e' equivalente. I risultati
non sarebbero allineati con le run macOS `321/500`, PID e local sweep.

### 3. Verificare ambiente Python/OpenSim

Da PowerShell o Anaconda Prompt:

```powershell
cd "C:\path\to\CMC-like-Simulator - Claude"
conda activate envCMC-like
python -c "import sys, opensim; print(sys.executable); print('opensim OK')"
```

Il path stampato deve puntare all'ambiente `envCMC-like`, non a `base`.

Se si vuole forzare l'interprete nello sweep:

```powershell
python validation\outer_pid_local_sweep.py --dry-run --python "%CONDA_PREFIX%\python.exe"
```

### 4. Verificare compilazione Python

```powershell
python -m py_compile `
  validation\outer_pid_gain_sweep.py `
  validation\outer_pid_local_sweep.py `
  plot\plotter.py
```

Esito atteso: nessun output, return code `0`.

### 5. Dry-run dello sweep locale

```powershell
python validation\outer_pid_local_sweep.py --dry-run
```

Controlli attesi:

```text
knee_candidates = 27
ankle_candidates = 27
full_candidates = 25
total_jobs = 133
screen_windows = ((13.1638, 14.7799), (17.9528, 19.5259))
```

Su Windows il default worker count puo' essere `12`; se la macchina e' sotto
carico, usare `--workers 6`.

### 6. Quick-smoke nativo Windows

Non lanciare ancora lo sweep completo. Prima eseguire:

```powershell
python validation\outer_pid_local_sweep.py `
  --quick-smoke `
  --workers 2 `
  --sweep-root results\_outer_pid_local_sweep_quick_smoke_windows
```

Esito atteso:

```text
total_jobs = 2
stage1_knee: OK
stage1_ankle: OK
quick smoke complete; full sweep not launched.
```

Verificare che le righe progress contengano anche l'ETA totale:

```text
overall=... total_elapsed=... total_eta=...
```

Controllare i CSV:

```powershell
Get-Content results\_outer_pid_local_sweep_quick_smoke_windows\sweep_summary.json
Get-Content results\_outer_pid_local_sweep_quick_smoke_windows\stage1_knee_screen.csv -TotalCount 2
Get-Content results\_outer_pid_local_sweep_quick_smoke_windows\stage1_ankle_screen.csv -TotalCount 2
```

Le colonne nuove devono essere presenti:

```text
knee_tau_error_rms
ankle_tau_error_rms
knee_motor_speed_dot_rms
ankle_motor_speed_dot_rms
knee_motor_joint_power_diff_rms
ankle_motor_joint_power_diff_rms
```

### 7. Smoke del plotter con naming nuovo

Dopo una run valida, verificare che il plotter crei cartelle nel nuovo formato:

```powershell
python plot\plotter.py `
  --results-dir results\_outer_pid_local_sweep_quick_smoke_windows\runs\stage1_knee\knee_kkp260_kkd22_kki40_akp420_akd1_aki0_w1 `
  --setup models\AB06_SEASEA_Threadmill\AB06_SEASEA_setup.xml `
  --model models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500.osim `
  --reference models\AB06_SEASEA_Threadmill\data\IK_results_AB06_SEASEA.mot
```

Output atteso:

```text
plot/MM_DD_2026_iter
missing_channels.txt
```

Per una mini-run da 30 ms e' possibile che i gait-cycle plot siano poco
informativi; qui interessa solo validare path, lettura setup/reference e naming.

### 8. Solo dopo: lanciare lo sweep locale reale

Se i punti 1-7 passano:

```powershell
python validation\outer_pid_local_sweep.py --workers 12
```

oppure, piu' conservativo:

```powershell
python validation\outer_pid_local_sweep.py --workers 6
```

Non usare `--quick-smoke` per la run reale.

## Criteri di allineamento

La macchina Windows si considera allineata quando:

1. `import opensim` funziona in `envCMC-like`;
2. `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` esiste e viene caricato;
3. `python -m py_compile ...` passa;
4. `outer_pid_local_sweep.py --dry-run` mostra `total_jobs=133`;
5. `outer_pid_local_sweep.py --quick-smoke` termina con due job `OK`;
6. i CSV quick-smoke includono le nuove metriche motor-driver;
7. il plotter crea cartelle `MM_DD_AAAA_iter`;
8. solo a quel punto si lancia lo sweep reale.

## Rischi residui

- La DLL Windows `ff` potrebbe dover essere ricompilata se non e' gia' presente.
- Le differenze native OpenSim/Python tra macOS e Windows vanno validate con
  quick-smoke, non dedotte dalla sola compilazione Python.
- Se la baseline `results/_stiff321_500_ab06_pd_full` non viene sincronizzata,
  lo sweep non potra' calcolare `delta_mean_rmse_*`.
- Se Windows usa la vecchia DLL non-`ff`, il run puo' completare ma non sara'
  confrontabile con i risultati macOS recenti.
