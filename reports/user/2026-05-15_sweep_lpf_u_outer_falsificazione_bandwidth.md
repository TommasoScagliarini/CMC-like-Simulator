# Sweep LPF su u outer e falsificazione ipotesi bandwidth - 2026-05-15

## Problema

Dopo l'esperimento `relative-D`
(`reports/user/2026-05-15_esperimento_relative_d_motor_driver.md`) sapevamo
che:

1. il chattering del motor driver (alta `motor_speed_dot RMS`) non era stato
   ridotto dal PID outer quasi-best, anzi era peggiorato rispetto al PD;
2. sostituire abs-D con relative-D puro destabilizzava il sistema globale
   per perdita di damping implicito sul giunto.

L'opzione (3) discussa era: **se il chattering del motor driver dipende dalla
banda di `tau_ref`, allora filtrare `u` (e quindi `tau_ref = u*F_opt`) prima
del plugin deve ridurlo senza degradare troppo il tracking cinematico**.
Esperimento chirurgico: zero modifiche al driver, abs-D ripristinato, LPF di
primo ordine su `u` per ciascun SEA, sweep su `fc`.

## Soluzione e strategia

Pre-processing del comando outer nel controller Python:

```text
alpha = dt / (dt + 1/(2*pi*fc))
u_filt[k] = u_filt[k-1] + alpha * (u_raw[k] - u_filt[k-1])
u_filt = clip(u_filt, -1, +1)
```

Stato separato per `pros_knee_angle` e `pros_ankle_angle`. `fc <= 0` disattiva
il filtro su quel SEA.

Setup esperimento:

- driver SEA: **abs-D** (`-Kd * omega_m`), ripristinato da backup
  `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.abs_omega_backup_20260515`;
- modello: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim`;
- outer: PID quasi-best `knee 340/30/120` e `ankle 850/2/300`;
- finestra: `t in [12.0, 14.5] s` (~1 ciclo gait, 2500 step, screening);
- sweep `fc [Hz] = [off, 200, 100, 50, 25, 15, 10]`;
- GRF filter attivo; tutto il resto invariato.

## File modificati

Codice Python:

- `output.py:473` - ripristinata la legge non-impedance `-Kd * omega_m` nella
  diagnostica SEA.
- `prosthesis_controller.py:354` - ripristinata la stessa legge nel
  feasibility predictor non-impedance.
- `prosthesis_controller.py` - aggiunti stato del filtro
  (`_u_filtered`, `_u_filtered_initialised`), helper `_apply_u_lpf`,
  reset esteso e log di `u_raw` / `u_lpf_alpha` nel `u_dict`.
- `config.py` - aggiunto campo `sea_u_lpf_cutoff_hz` (dict per coord, default
  `0.0` = filtro disattivo).
- `main.py` - aggiunte CLI `--sea-u-lpf-knee` e `--sea-u-lpf-ankle` e binding
  su `cfg.sea_u_lpf_cutoff_hz`.

Plugin: nessuna modifica. Ripristinato `.dylib` abs-D dal backup.

Nuovo script:

- `validation/lpf_sweep_metrics.py` - collector read-only dei metrics
  (RMSE protesico, `tau_error`, `motor_speed_dot`, saturazioni, reserves)
  per ogni run dello sweep, con interpolazione del file IK come reference.

Script di lancio temporaneo (non versionato): `/tmp/lpf_sweep_launcher.sh`.

## Run e risultati

Comando tipo (variando `--sea-u-lpf-*`):

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --t-start 12.0 --t-end 14.5 \
  --output-dir results/_lpf_sweep_25s_fc<FC>_20260515 \
  --sea-outer-controller pid \
  --sea-kp-knee 340 --sea-kd-knee 30 --sea-ki-knee 120 \
  --sea-kp-ankle 850 --sea-kd-ankle 2 --sea-ki-ankle 300 \
  [--sea-u-lpf-knee <FC> --sea-u-lpf-ankle <FC>] \
  --filter-grf --log
```

Esito per ciascun `fc`:

```text
fc    | mean RMSE [deg] | knee mdot RMS | ankle mdot RMS | ankle_sat | res RMS | status
------|-----------------|---------------|----------------|-----------|---------|--------
off   |    2.02         |    494        |    821         |   0       |  114    | OK (baseline)
200   |   16.33  (8x)   |  14 215       | 31 890         | 347       |  334    | OK ma rotto
100   |     -           |     -         |     -          |   -       |   -     | CRASH t=13.08
 50   |  296.66 (147x)  |   6 467       | 21 488         | 247       | 2 616   | OK ma divergente
 25   |     -           |     -         |     -          |   -       |   -     | CRASH t=12.79
 15   |     -           |     -         |     -          |   -       |   -     | KILLED (SO stuck)
 10   |  446.18 (221x)  |   1 819       | 12 267         |  66       | 3 171   | OK ma divergente
```

Per le run "OK ma divergente" la simulazione completa i 2500 step ma il
tracking protesico va su centinaia di gradi di errore e le reserve crescono
fino a `2-3 kNm`. Per le run "CRASH" l'integratore RK4 produce accelerazioni
non finite sui DoF del pelvis. La run `fc=15` e' stata terminata
manualmente dopo ~14 minuti perche' lo static optimizer era entrato in un
loop di backtracking infinito con `|res|~10^3`.

Smoke separati prima dello sweep:

- `results/_lpf_smoke_absD_off_20260515` - sanity check su 60 step,
  baseline abs-D senza filtro;
- `results/_lpf_smoke_absD_fc25_20260515` - verifica che `u` venga
  effettivamente filtrato: con `fc=25 Hz` e `dt=1 ms` si ha `alpha ≈ 0.136`,
  e tra `u_raw = 0.0236` e `u_filt = 0.0032` il rapporto e' coerente con la
  teoria.

## Interpretazione

L'ipotesi "il chattering del driver e' bandwidth-driven da `tau_ref`" e'
**falsificata** da questi dati. Tre punti netti:

1. Il `motor_speed_dot RMS` della baseline `fc=off` e' gia' alto
   (`494/821 rad/s2`). Esiste senza alcun filtro, quindi non e' iniettato da
   spike di `tau_ref`.
2. Filtrare leggermente (`fc=200 Hz`, oltre `omega_n_ankle ≈ 111 Hz`) non
   riduce il chattering: lo fa esplodere (`14 215 / 31 890`). Cioe' anche
   una distorsione di fase modesta peggiora tutto.
3. A `fc` piu' bassi la simulazione o crasha (`100, 25`) o sopravvive con
   tracking inutile (`50, 10`). Non c'e' una zona "sweet spot" dove il
   filtro aiuta.

Meccanismo:

- LPF su `u` introduce ~45° di lag a `fc` e piu' sotto;
- l'outer PID quasi-best ha `Ki = 120` (knee) e `Ki = 300` (ankle) -
  autorita' integrale alta;
- lag + alta `Ki` = de-stabilizzazione lenta;
- la fase del loop si rompe, l'integrale rincorre, `tau_ref` sale, l'inner
  ankle satura (347 campioni a `fc=200`), il giunto scivola, le reserve
  esplodono. In alcune frequenze la dinamica patologica innesca direttamente
  `non-finite RK4`.

Conseguenza diagnostica: il chattering del motor driver e' una proprieta'
**del loop chiuso** (driver + plant + outer PID) e non del contenuto in banda
di `tau_ref`. Filtrare `tau_ref` non e' una strada utile per ridurlo.

## Verifiche eseguite

Compilazione statica:

```bash
python -m py_compile config.py main.py prosthesis_controller.py output.py validation/lpf_sweep_metrics.py
```

Esito: `OK`.

Smoke filter funzionale (60 step, `fc=off` vs `fc=25`):

```text
t=11.991  u_raw=0.023648  u_filt(fc=25)=0.003210  (alpha~0.136)
t=11.992  u_raw=0.045459  u_filt(fc=25)=0.009011
...
```

Differenze coerenti con la formula `alpha = dt/(dt + 1/(2*pi*fc))`.

Lancio sweep:

```bash
nohup /tmp/lpf_sweep_launcher.sh > /tmp/lpf_sweep_launcher.out 2>&1 &
```

Tempi reali per run:

```text
fc=off  : 3:27
fc=200  : 4:22
fc=100  : 2:20 (crash)
fc=50   : 4:32
fc=25   : 2:51 (crash)
fc=15   : ~35 min, killed (SO stuck)
fc=10   : 4:32
```

Metriche calcolate via:

```bash
python validation/lpf_sweep_metrics.py
```

JSON di riepilogo: `results/_lpf_sweep_log/metrics_20260515.json`.
Log dello sweep: `results/_lpf_sweep_log/sweep_20260515_120800.log`.

## File creati o aggiornati nei risultati

```text
results/_lpf_sweep_25s_fcoff_20260515/   (complete)
results/_lpf_sweep_25s_fc200_20260515/   (complete - tracking rotto)
results/_lpf_sweep_25s_fc100_20260515/   (failed)
results/_lpf_sweep_25s_fc50_20260515/    (complete - divergente)
results/_lpf_sweep_25s_fc25_20260515/    (failed)
results/_lpf_sweep_25s_fc15_20260515/    (parziale, killed)
results/_lpf_sweep_25s_fc10_20260515/    (complete - divergente)
results/_lpf_smoke_absD_off_20260515/    (smoke)
results/_lpf_smoke_absD_fc25_20260515/   (smoke)
results/_lpf_sweep_log/sweep_20260515_120800.log
results/_lpf_sweep_log/metrics_20260515.json
```

## Prossimi passi possibili

Tre direzioni separate, ortogonali tra loro, in ordine di costo crescente:

1. **Diagnostica shadow su run abs-D esistente.** Calcolare offline nei log
   il `tau_input` che si sarebbe ottenuto con damping parziale
   `-Kd*(omega_m - alpha*omega_j)` per `alpha in {0, 0.25, 0.5, 0.75, 1}`,
   senza modificare l'attuazione. Quantifica la divergenza tra le varianti
   senza far girare nuove simulazioni rischiose. Zero rischio, massima
   informazione su priori.
2. **Sweep damping parziale sul plugin.** Implementare nel driver C++
   `-Kd*(omega_m - alpha*omega_j)` con `alpha` proprieta' del SEA, e fare
   sweep su `alpha` con l'outer PID quasi-best. Risponde direttamente alla
   domanda "quanto del damping abs-D era genuino vs spurio sul giunto?".
3. **Sweep su `Ki` dell'outer mantenendo abs-D.** Vede se il chattering era
   alimentato dall'autorita' integrale dell'outer piu' che dalla struttura
   del driver. Sweep su `Ki_knee/Ki_ankle` ridotti (es. `30/60`, `60/120`,
   `120/300` baseline), stessa finestra.

Raccomandato per primo: (1), perche' usa dati gia' presenti e disambigua le
priori prima di toccare il plugin o l'outer.
