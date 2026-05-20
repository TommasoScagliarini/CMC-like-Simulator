# PID sweep best candidate e naming plot - 2026-05-15

## Problema

Dopo la preparazione dello script `validation/outer_pid_gain_sweep.py` era stato
lanciato lo sweep reale dei gain PID dell'outer loop protesico sul caso
`AB06_SEASEA_Threadmill` con stiffness paper-equivalent:

```text
SEA_Knee  K = 321 Nm/rad
SEA_Ankle K = 500 Nm/rad
```

Serviva leggere i risultati prodotti dallo sweep, commentare il best candidate,
generare i soliti sei plot diagnostici e allineare il naming delle cartelle plot
al formato richiesto:

```text
MM_DD_AAAA_iter
```

## Soluzione

La cartella sweep analizzata e':

```text
results/_outer_pid_gain_sweep_20260514_223838
```

Lo sweep ha eseguito:

```text
total_jobs = 425
stage1_knee = 100 candidati x 2 finestre
stage1_ankle = 100 candidati x 2 finestre
full = 25 candidati full AB06
```

Il best full candidate e':

```text
combo_kkp300_kkd26_kki80_akp750_akd2_aki240

pros_knee_angle:
  Kp = 300
  Kd = 26
  Ki = 80

pros_ankle_angle:
  Kp = 750
  Kd = 2
  Ki = 240
```

Rispetto alla baseline PD filtrata con stiffness `321/500`:

```text
metric                 baseline PD       best PID
knee RMS error          3.1246 deg        1.5284 deg
ankle RMS error         5.4164 deg        2.7522 deg
mean pros RMS           4.2705 deg        2.1403 deg
delta mean RMS              -            -49.88 %
max knee error          9.6662 deg        5.0087 deg
max ankle error        13.1123 deg        6.4805 deg
```

Il candidato e' stabile nel full run:

```text
status = complete
t_start = 11.99 s
t_end = 21.00 s
steps = 9010
max |u| knee = 0.3452703
max |u| ankle = 0.42981127
tau_input saturations = 0
max tau_input_raw = 110.0409 Nm
```

Il validator sul best candidate restituisce:

```text
PASS = 38
WARN = 7
FAIL = 1
```

Il `FAIL` rimane il noto `mtp_angle_r output vs IK`, gia' presente nelle
baseline e non introdotto dal PID.

## Interpretazione

Il best PID migliora molto il tracking cinematico protesico, ma il risultato va
letto come best candidate per la metrica primaria dello sweep, cioe'
`mean_pros_rmse_deg`, non come soluzione finale ottimizzata sul comportamento
del motor driver.

Confronto dinamica SEA rispetto alla baseline PD `321/500`:

```text
metric                         baseline PD      best PID
knee tau_error RMS              3.1095 Nm        3.3136 Nm
ankle tau_error RMS             0.9761 Nm        1.0547 Nm
knee speed_dot RMS             92.25 rad/s2    271.18 rad/s2
ankle speed_dot RMS           128.03 rad/s2    345.31 rad/s2
knee speed_dot max            970.35 rad/s2   2505.66 rad/s2
ankle speed_dot max          2570.59 rad/s2   5747.31 rad/s2
```

Quindi il PID riduce fortemente l'errore cinematico, ma non migliora il tracking
interno `tau_ref - tau_spring`; anzi lo peggiora leggermente e aumenta la
dinamica rapida del motore. Questo e' coerente con l'interpretazione: il PID
outer aumenta l'autorita' del tracking cinematico, ma non risolve da solo il
tema motor-driver/chattering.

## Plot generati

Sono stati generati i sei plot standard del best candidate con:

```bash
python plot/plotter.py \
  --results-dir results/_outer_pid_gain_sweep_20260514_223838/full_runs/combo_kkp300_kkd26_kki80_akp750_akd2_aki240 \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --reference models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot \
  --sea-outer-controller pid \
  --sea-kp-knee 300 \
  --sea-kd-knee 26 \
  --sea-ki-knee 80 \
  --sea-kp-ankle 750 \
  --sea-kd-ankle 2 \
  --sea-ki-ankle 240
```

Output finale dopo il rename:

```text
plot/05_15_2026_1
```

File presenti:

```text
01_time_sea_control_reserve.png
02_time_joint_motor_states.png
03_gaitcycle_torque_angle_power.png
04_gaitcycle_joint_velocity_power.png
05_time_tau_input_tracking_error.png
06_time_joint_ref_sea_error.png
missing_channels.txt
```

Il plotter ha caricato:

```text
healthy overlay: OK
reference IK: models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
gait events: left=4, right=4
missing channels: No missing channels.
```

## Naming cartelle plot

Sono state rinominate le cartelle esistenti sotto `plot/` dal vecchio formato:

```text
DD_MM_AAAA - iter
```

al nuovo formato:

```text
MM_DD_AAAA_iter
```

Esempi:

```text
plot/15_05_2026 - 1 -> plot/05_15_2026_1
plot/14_05_2026 - 5 -> plot/05_14_2026_5
plot/22_04_2026 - 7 -> plot/04_22_2026_7
```

Inoltre `plot/plotter.py` e' stato aggiornato per creare direttamente nuove
cartelle nel formato `MM_DD_AAAA_iter`.

## File modificati

Modificato:

- `plot/plotter.py`

Creato:

- `reports/user/2026-05-15_pid_sweep_best_candidate_plot_naming.md`

Output generato/rinominato:

- `plot/05_15_2026_1`
- tutte le cartelle plot storiche nel vecchio formato `DD_MM_AAAA - iter`
  rinominate nel nuovo formato `MM_DD_AAAA_iter`

## Verifiche eseguite

Analisi CSV sweep:

- `sweep_summary.json`
- `ranking.csv`
- `full_results.csv`
- `stage1_knee_screen.csv`
- `stage1_ankle_screen.csv`
- `failures.csv`

Controlli principali:

```text
stage1_knee: 100/100 acceptable
stage1_ankle: 65/100 acceptable, 35 unstable/incomplete
full_results: 25/25 acceptable
```

Validator sul best candidate:

```bash
python validation/validate_sim_results.py \
  --results-dir results/_outer_pid_gain_sweep_20260514_223838/full_runs/combo_kkp300_kkd26_kki80_akp750_akd2_aki240 \
  --model-bundle models/AB06_SEASEA_Threadmill \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --reference models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
```

Esito:

```text
PASS=38, WARN=7, FAIL=1
```

Verifica plot:

```text
plot/05_15_2026_1/missing_channels.txt -> No missing channels.
```

Verifica naming:

```text
nessuna cartella residua nel formato DD_MM_AAAA - iter
```

Verifica sintassi:

```bash
python -m py_compile plot/plotter.py
```

Esito: OK.

## Prossimi passi

Il best candidate puo essere usato come riferimento per il miglior tracking
cinematico ottenuto finora su AB06 con stiffness `321/500`, ma prima di
considerarlo tuning finale conviene fare una mini-sweep locale multi-obiettivo
attorno al best, includendo nel punteggio:

- `mean_pros_rmse_deg`;
- `tau_ref - tau_spring` RMS;
- `motor_speed_dot` RMS/max;
- `motor_power/joint_power`;
- saturazioni `u` e `tau_input`.

Una griglia locale ragionevole:

```text
knee Kp: 260, 300, 340
knee Kd: 22, 26, 30
knee Ki: 40, 80, 120

ankle Kp: 650, 750, 850
ankle Kd: 1.5, 2.0, 2.5
ankle Ki: 180, 240, 300
```

Obiettivo: mantenere il grosso miglioramento cinematico senza far crescere
troppo chattering, accelerazione motore e disaccoppiamento power motor/joint.
