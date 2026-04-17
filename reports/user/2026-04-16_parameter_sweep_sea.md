# Parameter Sweep SEA K/Kp/Kd

Data: 2026-04-16

## Problema

Dopo il tuning manuale dei gain SEA, l'obiettivo era esplorare in modo sistematico le stiffness `K` dei due attuatori SEA e ricavare automaticamente i gain low-level `Kp` e `Kd` mantenendo costanti:

```text
omega_n = 500 rad/s
zeta    = 0.7
```

Il vincolo principale era non cambiare la dinamica SEA e non introdurre scorciatoie:

- nessuna modifica al plugin C++;
- nessuna modifica alla legge dinamica del SEA;
- nessuna modifica alla logica del simulatore;
- tuning limitato a `stiffness`, `Kp`, `Kd` nel modello `.osim`.

## Formula Usata

Il plugin lavora in modalita non-impedance:

```text
tau_spring = K * (theta_m - theta_j)
tau_input  = Kp * (tau_ref - tau_spring) - Kd * omega_m
Jm * omega_dot = tau_input - tau_spring - Bm * omega_m
```

Linearizzando intorno a riferimento e giunto quasi fermi:

```text
Jm * theta_m_ddot + (Kd + Bm) * theta_m_dot + K * (Kp + 1) * theta_m = forcing
```

Da cui:

```text
omega_n = sqrt(K * (Kp + 1) / Jm)
zeta    = (Kd + Bm) / (2 * Jm * omega_n)
```

e quindi:

```text
Kp = (Jm * omega_n^2 / K) - 1
Kd = (2 * zeta * Jm * omega_n) - Bm
```

Con `Jm=0.01`, `Bm=0.1`, `omega_n=500`, `zeta=0.7`:

```text
Kd = 6.9
Kp = 2500 / K - 1
```

Lo script rifiuta candidati non fisicamente validi:

- `K <= 0`;
- `Kp <= 0`;
- `Kd < 0`;
- `K >= 2500`.

## Script Implementato

E stato creato lo script temporaneo:

```text
validation/_tmp_sea_parameter_sweep.py
```

Lo script:

- usa come template `models/Adjusted_SEASEA - Copia_tuned.osim`;
- crea modelli candidati temporanei dentro la cartella sweep;
- modifica solo `stiffness`, `Kp`, `Kd` per `SEA_Knee` e `SEA_Ankle`;
- lancia `main.py` in subprocess;
- usa `--validate` per ogni candidato;
- legge direttamente gli `.sto` prodotti;
- calcola metriche e score;
- salva `sweep_results.csv`;
- produce `best_candidate.json`;
- produce un report Markdown automatico.

Per evitare oversubscription durante i run paralleli, ogni subprocess imposta:

```text
OMP_NUM_THREADS=1
MKL_NUM_THREADS=1
OPENBLAS_NUM_THREADS=1
NUMEXPR_NUM_THREADS=1
```

Il default parallelo e stato impostato a `--workers 12`.

## Strategia Di Sweep

Griglia stiffness usata per knee e ankle:

```text
K = [75, 100, 125, 150, 175, 200, 250, 300, 350, 500, 750, 1000]
```

Strategia staged:

1. sweep knee variando `K_knee`, con `K_ankle=500`;
2. sweep ankle variando `K_ankle`, con `K_knee=250`;
3. combinazione dei migliori valori knee/ankle;
4. full run progressivi sui candidati ordinati per score.

Screening:

```text
t_start = 4.26
t_end   = 6.55
```

Full run:

```text
t_start = 4.26
t_end   = 11.06
```

## Criteri Di Accettazione

Un candidato e accettabile solo se:

- simulazione completa;
- validator senza FAIL;
- zero campioni con `tau_input_saturated`;
- `max_abs(tau_input_raw) <= 480 Nm`;
- `max |u| < 0.95`;
- nessun NaN/Inf in stati, derivate, potenze o diagnostica SEA.

Lo score usato per ordinare i candidati era:

```text
score =
    0.55 * worst(pros_knee_rms_deg, pros_ankle_rms_deg)
  + 0.25 * mean(pros_knee_rms_deg, pros_ankle_rms_deg)
  + 0.10 * (max_motor_speed_dot_abs / 10000)
  + 0.05 * (max_tau_input_raw_abs / 100)
  + 0.05 * (max_motor_power_rms_abs / 1000)
```

## Test Eseguiti

### Py Compile

```powershell
python -m py_compile validation\_tmp_sea_parameter_sweep.py
```

Esito: OK.

### Dry Run

```powershell
python validation\_tmp_sea_parameter_sweep.py --dry-run
```

Esito: OK. Ha stampato la tabella `K`, `Kp`, `Kd`.

Esempi:

| K | Kp | Kd |
|---:|---:|---:|
| 75 | 32.3333 | 6.9 |
| 100 | 24 | 6.9 |
| 125 | 19 | 6.9 |
| 250 | 9 | 6.9 |
| 500 | 4 | 6.9 |
| 1000 | 1.5 | 6.9 |

### Quick Smoke

```powershell
conda run -n envCMC-like python validation\_tmp_sea_parameter_sweep.py --workers 2 --quick-smoke
```

Esito: OK.

Lo smoke ha verificato:

- creazione dei modelli candidati;
- lancio subprocess OpenSim;
- generazione outputs;
- parsing metriche;
- scrittura CSV.

### Sweep Reale

```powershell
conda run -n envCMC-like python validation\_tmp_sea_parameter_sweep.py --workers 12
```

Esito: lo sweep e arrivato a conclusione, ma non ha trovato una soluzione accettabile.

Il comando ha terminato con exit code non-zero perche lo script restituisce `1` quando non trova una soluzione valida. Questo comportamento e voluto.

## Risultati

Cartella sweep:

```text
results/_sea_parameter_sweep_20260416_205113
```

File principali:

```text
results/_sea_parameter_sweep_20260416_205113/sweep_results.csv
results/_sea_parameter_sweep_20260416_205113/best_candidate.json
reports/user/2026-04-16_sea_parameter_sweep.md
```

Il report automatico riporta:

```text
Nessuna soluzione accettabile trovata.
```

Top candidates:

| Rank | Run | Knee K | Ankle K | Score | Problema |
|---:|---|---:|---:|---:|---|
| 1 | `full_k75_a75` | 75 | 75 | 7.8417 | saturazione tau_input, raw 867.935 Nm |
| 2 | `screen_combo_k75_a75` | 75 | 75 | 7.9692 | saturazione tau_input, raw 867.935 Nm |
| 3 | `full_k100_a75` | 100 | 75 | 8.1601 | saturazione tau_input, raw 629.782 Nm |
| 4 | `screen_combo_k100_a75` | 100 | 75 | 8.4059 | validator FAIL + saturazione |
| 5 | `full_k125_a75` | 125 | 75 | 8.5202 | raw 489.011 Nm sopra soglia + validator FAIL |
| 6 | `full_k150_a75` | 150 | 75 | 8.8190 | validator FAIL |

## Conclusione Tecnica

Con il vincolo rigido:

```text
omega_n = 500 rad/s
zeta    = 0.7
```

e con la formula completa derivata dalla dinamica del plugin, la griglia testata non contiene una soluzione che soddisfi simultaneamente:

- stabilita numerica;
- assenza di saturazioni;
- margine su `tau_input_raw`;
- tracking protesico entro soglia;
- validator senza FAIL.

I candidati con stiffness bassa migliorano lo score e il tracking relativo, ma producono comandi motore troppo elevati e saturazioni. I candidati con stiffness piu alta riducono alcuni problemi dinamici, ma peggiorano il tracking o falliscono il validator.

## Stato Finale

Poiche non e stata trovata una soluzione accettabile, lo script ha rispettato la policy prevista:

- non ha modificato `models/Adjusted_SEASEA - Copia_tuned.osim`;
- non ha modificato `config.py`;
- non si e cancellato;
- ha lasciato disponibili script e cartella sweep per analisi successiva.

Il modello tuned resta:

```text
SEA_Knee:
  K  = 250
  Kp = 18.5
  Kd = 10

SEA_Ankle:
  K  = 500
  Kp = 35
  Kd = 20
```

e `config.py` resta puntato a:

```text
models/Adjusted_SEASEA - Copia_tuned.osim
```

## Prossimi Passi Consigliati

Per proseguire l'ottimizzazione servira rilassare almeno uno dei vincoli. Le opzioni piu sensate sono:

1. Sweep anche su `omega_n`, per esempio `200-500 rad/s`, mantenendo `zeta=0.7`.
2. Mantenere `omega_n=500`, ma rilassare il margine `tau_input_raw <= 480 Nm`.
3. Includere anche il tuning del controller high-level `sea_kp/sea_kd`.
4. Separare gli obiettivi knee e ankle, evitando di imporre la stessa griglia stiffness a entrambi.

La direzione piu pulita sembra la prima: fare uno sweep su `omega_n` e `K`, mantenendo la formula fisica completa.
