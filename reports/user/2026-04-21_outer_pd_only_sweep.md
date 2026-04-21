# 2026-04-21 - Sweep outer PD-only per SEA protesici

## Problema

Dopo la rimozione del termine `tau_ff` dal comando dell'outer SEA, la run
`20/2` e' risultata stabile ma con tracking protesico pessimo:

- `pros_knee_angle` RMS circa `24.75 deg`;
- `pros_ankle_angle` RMS circa `162.18 deg`;
- nessuna saturazione del plugin SEA;
- `tau_cmd_raw == outer_pd_cmd`, quindi il controller e' davvero PD-only.

Questo indica che il vecchio tuning `20/2` era valido nel contesto
`tau_ff + PD`, ma non basta piu' quando la protesi viene controllata con solo
outer PD.

## Soluzione implementata

E' stato introdotto uno script di sweep **outer-only** per cercare gain
`sea_kp/sea_kd` PD-only senza modificare:

- plugin C++;
- modello `.osim`;
- inner loop SEA (`K`, `Kp`, `Kd`);
- `config.py` durante i singoli candidati.

Lo script e':

```text
validation/outer_gain_sweep.py
```

La griglia default e molto ampia:

```text
knee Kp  = [10, 20, 40, 60, 80, 120, 160, 220]
knee Kd  = [0.5, 1, 2, 4, 6, 8, 12, 16]
ankle Kp = [20, 40, 80, 120, 180, 250, 350, 500, 700]
ankle Kd = [1, 2, 4, 6, 8, 12, 16, 24]
```

Totale candidati stage 1:

```text
4608
```

## Strategia dello sweep

Lo sweep e pensato per Windows con 12 worker:

```powershell
conda run -n envCMC-like python validation\outer_gain_sweep.py --workers 12
```

Pipeline:

1. **Stage 1**: tutti i candidati su finestra `4.26 -> 5.30`.
2. **Stage 2**: migliori `96` candidati su due finestre:
   - `6.80 -> 7.20`
   - `8.70 -> 9.10`
3. **Full**: migliori `12` candidati su `4.26 -> 11.06`.

Output default:

```text
results/_outer_gain_sweep_YYYYMMDD_HHMMSS/
  screen_stage1.csv
  screen_stage2.csv
  full_results.csv
  best_candidates.json
```

Per contenere lo spazio disco, i run di screening eliminano di default i file
`.sto` dopo aver estratto le metriche, mantenendo:

- `console.txt`;
- `sim_output_run_status.txt`;
- `sim_output_gait_events.csv`;
- righe CSV con metriche complete.

I full run vengono invece conservati integralmente.

## Metriche e criteri

Per ogni candidato vengono raccolti:

- tracking RMS/max/mean per:
  - `pros_knee_angle`;
  - `pros_ankle_angle`;
  - `ankle_angle_r`;
  - `mtp_angle_r`;
- `max|u|` e frazione `|u| > 0.95`;
- numero saturazioni `tau_input`;
- `max|tau_input_raw|`;
- HPF noise su `tau_input`;
- `motor_speed` e `motor_speed_dot`;
- diagnostica recruitment:
  - `muscle_share`;
  - `reserve_control_norm`;
  - `residual_norm`;
  - `muscle_capable_share`;
- controllo anti-regressione PD-only:
  - `max_tau_cmd_minus_pd_abs`, che deve restare circa zero.

Hard reject:

- run incompleta;
- output non finiti;
- saturazione `tau_input`;
- `max|u| > 0.99`;
- `max|tau_input_raw| > 450 Nm`;
- `tau_cmd_raw` diverso da `outer_pd_cmd`.

Il punteggio privilegia tracking protesico, poi penalizza errore massimo, noise,
uso elevato di `u` e side-effect sul lato biologico distale.

## File modificati

- `main.py`
  - aggiunte CLI:
    - `--sea-kp-knee`
    - `--sea-kd-knee`
    - `--sea-kp-ankle`
    - `--sea-kd-ankle`
  - i gain `sea_kp/sea_kd` vengono stampati nella configurazione attiva.

- `config.py`
  - aggiornato il commento del controller SEA:
    - da `tau_ff + PD`;
    - a `PD-only`.
  - `tau_ff` resta descritto come diagnostica/oracle, non come comando.

- `validation/outer_gain_sweep.py`
  - nuovo script di sweep outer PD-only.

## Verifiche eseguite

Compilazione:

```bash
python -m py_compile main.py config.py validation/outer_gain_sweep.py
```

Dry run:

```bash
python validation/outer_gain_sweep.py --dry-run
```

Risultato:

```text
candidates=4608
stage1_windows=((4.26, 5.3),)
stage2_windows=((6.8, 7.2), (8.7, 9.1))
top_n_stage2=96
top_n_full=12
```

Quick smoke con ambiente OpenSim su macOS:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  validation/outer_gain_sweep.py \
  --quick-smoke \
  --workers 1 \
  --top-n-full 0 \
  --sweep-root /tmp/cmc_outer_gain_smoke3 \
  --timeout-minimum 60
```

Risultato:

- 3 candidati eseguiti;
- CLI gain override confermati nel `console.txt`;
- CSV prodotto correttamente;
- `.sto` di screening rimossi come previsto.

Nota: nella quick smoke il `score_total` puo essere `inf` per i candidati
accettabili, perche la finestra `4.26 -> 4.29` e piu corta dello skip HPF
`0.1 s`. Questo non vale per lo sweep reale, dove stage 1 dura piu di un
secondo.

## Prossimi passi

1. Copiare/portare il progetto aggiornato su Windows.
2. Eseguire:

```powershell
conda run -n envCMC-like python validation\outer_gain_sweep.py --workers 12
```

3. Se 12 worker saturano CPU/RAM o producono timeout anomali, rilanciare con:

```powershell
conda run -n envCMC-like python validation\outer_gain_sweep.py --workers 8
```

4. Validare i migliori full run con:

```powershell
python validation\validate_sim_results.py --results-dir <full_run_dir>
python validation\cmc_strict_metrics.py --results-dir <full_run_dir>
```

