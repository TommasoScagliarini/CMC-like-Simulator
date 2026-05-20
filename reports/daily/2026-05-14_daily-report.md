# Daily report - 2026-05-14

## Sintesi

Giornata centrata su AB06-SEASEA con tre filoni principali:

1. filtro GRF opt-in e validazione full run con baseline `1000/700`;
2. analisi stiffness SEA paper-equivalent `Knee=321 Nm/rad`, `Ankle=500 Nm/rad`;
3. preparazione dello sweep PID dell'outer loop per ottimizzare il tracking
   cinematico protesico.

Il risultato operativo piu importante e' che la stiffness `321/500` mantiene o
migliora leggermente il tracking cinematico rispetto alla baseline filtrata,
fa lavorare molto di piu la molla e rende piu visibile la separazione
`motor_angle` / `joint_angle`. Il retuning aggressivo dei gain interni per
matchare `omega_n` e `zeta` migliora molto il tracking di coppia, ma aumenta le
accelerazioni del motore e quindi il rischio di chattering.

## 1. Filtro GRF opt-in

E' stato aggiunto un filtro di contatto per le GRF con comportamento non
invasivo:

- disattivo di default (`enable_grf_contact_filter = False`);
- attivabile via CLI con `--filter-grf`;
- crea una copia run-local del `.mot` filtrato sotto `<output_dir>/_grf_filter`;
- non modifica i file sorgente e non impatta gli altri dataset se non viene
  esplicitamente attivato;
- registra nel run status sorgente GRF, file filtrato e report filtro.

Parametri principali:

```text
grf_contact_threshold_n = 20
grf_min_contact_duration_s = 0.05
grf_min_cycle_duration_s = 0.30
```

File modificati:

- `config.py`
- `main.py`
- `model_loader.py`
- `simulation_runner.py`

Run full AB06 con filtro:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --output-dir results/_grf_filtered_ab06_pd_full \
  --filter-grf \
  --plot \
  --log
```

Risultato:

```text
results: results/_grf_filtered_ab06_pd_full
plot:    plot/14_05_2026 - 3
t:       11.99 -> 21.00 s
steps:   9010
status:  complete
```

Validazione:

```text
PASS = 40
WARN = 5
FAIL = 1
```

Il FAIL e' il solito `mtp_angle_r output vs IK`, gia presente nelle baseline e
non specifico del filtro GRF.

Metriche protesiche baseline filtrata `1000/700`:

```text
pros_knee_angle RMS  = 3.134225 deg
pros_ankle_angle RMS = 5.438842 deg
SEA_Knee tau_error RMS  = 3.188139 Nm
SEA_Ankle tau_error RMS = 1.042513 Nm
```

Report utente:

- `reports/user/2026-05-14_validazione_grf_filtered_pd.md`

## 2. Stiffness paper-equivalent 321/500

Sono stati chiariti i valori di stiffness dai paper:

- MyFlex-zeta riporta una stiffness torsionale sagittale equivalente alla
  caviglia in `Nm/deg`;
- il plugin SEA usa angoli in radianti, quindi il valore deve essere convertito
  in `Nm/rad`;
- conversione:

```text
K_Nm_per_rad = K_Nm_per_deg * 180 / pi
```

Interpretazione:

```text
SEA_Ankle = 500 Nm/rad ~= 8.73 Nm/deg
SEA_Knee  = 321 Nm/rad ~= 5.60 Nm/deg
```

Nota importante emersa durante la verifica: cambiare `config.py` non basta,
perche' `simulation_runner._sea_stiffness()` usa prima le proprieta lette dal
plugin nel modello `.osim`. Quindi la stiffness reale va cambiata nel modello.

Modelli creati:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_matched_dynamics.osim`

Nel modello `321/500` sono stati modificati solo:

```text
SEA_Knee  stiffness = 321
SEA_Ankle stiffness = 500
```

I gain interni sono rimasti quelli originali:

```text
SEA_Knee  Kp=3.9, Kd=9.7
SEA_Ankle Kp=8.8, Kd=9.7
```

Run full:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim \
  --output-dir results/_stiff321_500_ab06_pd_full \
  --filter-grf \
  --plot \
  --log
```

Risultato:

```text
results: results/_stiff321_500_ab06_pd_full
plot:    plot/14_05_2026 - 4
t:       11.99 -> 21.00 s
steps:   9010
status:  complete
```

Validazione:

```text
PASS = 40
WARN = 5
FAIL = 1
```

Metriche principali rispetto alla baseline filtrata:

```text
metric                         1000/700       321/500
pros_knee RMS [deg]             3.134          3.125
pros_ankle RMS [deg]            5.439          5.416
knee tau_error RMS [Nm]         3.188          3.109
ankle tau_error RMS [Nm]        1.043          0.976
knee deflection RMS [deg]       0.557          1.740
ankle deflection RMS [deg]      3.265          4.552
knee speed_dot RMS [rad/s2]    59.51          92.25
ankle speed_dot RMS [rad/s2]  112.45         128.03
tau_input saturations           0              0
```

Interpretazione:

- tracking cinematico leggermente migliore o sostanzialmente invariato;
- molla molto piu caricata, soprattutto al ginocchio;
- separazione `motor_angle` / `joint_angle` piu evidente;
- nessuna saturazione `tau_input`;
- accelerazioni motore piu alte, quindi maggiore contenuto rapido.

Report utente:

- `reports/user/2026-05-14_validazione_stiff321_500.md`

## 3. Retuning interno con omega_n/zeta matched

E' stata fatta una run mantenendo le stiffness `321/500`, ma ritunando i gain
interni per ottenere la stessa pulsazione naturale e lo stesso damping del caso
`1000/700`.

Formula usata:

```text
omega_n = sqrt(Kp * K / Jm)
zeta = (Bm + Kd) / (2 * sqrt(Jm * Kp * K))
```

Per mantenere `omega_n`:

```text
Kp_new = Kp_old * K_old / K_new
```

Poiche' `Kp*K` resta costante, `Kd` puo restare invariato per mantenere `zeta`.

Gain risultanti:

```text
SEA_Knee:
  K = 321
  Kp = 12.149532710280374
  Kd = 9.7
  omega_n = 624.50 rad/s
  zeta = 0.785

SEA_Ankle:
  K = 500
  Kp = 12.32
  Kd = 9.7
  omega_n = 784.86 rad/s
  zeta = 0.624
```

Run full:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --model models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_matched_dynamics.osim \
  --output-dir results/_stiff321_500_matched_dynamics_ab06_pd_full \
  --filter-grf \
  --plot \
  --log
```

Risultato:

```text
results: results/_stiff321_500_matched_dynamics_ab06_pd_full
plot:    plot/14_05_2026 - 5
t:       11.99 -> 21.00 s
steps:   9010
status:  complete
```

Confronto principale:

```text
metric                         1000/700   321/500 no retune   321/500 matched
knee tracking RMS [deg]          3.134        3.125              3.089
ankle tracking RMS [deg]         5.439        5.416              5.433
knee tau_error RMS [Nm]          3.188        3.109              1.202
ankle tau_error RMS [Nm]         1.043        0.976              0.704
reserve norm RMS               114.736      114.313            114.654
knee speed_dot RMS [rad/s2]     59.51        92.25             175.67
ankle speed_dot RMS [rad/s2]   112.45       128.03             171.05
tau_input saturations            0            0                  0
```

Interpretazione:

- il matching migliora nettamente `tau_ref - tau_spring`;
- il tracking cinematico migliora poco al knee e peggiora leggermente alla
  caviglia rispetto al `321/500` no-retune;
- il costo e' un aumento forte di `motor_speed_dot`, quindi maggiore chattering;
- le riserve alte intorno a circa `13.4 s` e `17.7 s` restano quasi invariate.

Report utente:

- `reports/user/2026-05-14_validazione_stiff321_500_matched_dynamics.md`

## 4. Analisi motor driver e banda

E' stata discussa la relazione tra outer loop e motor driver. Chiarimento
importante:

- il motor driver non ha un callback digitale separato piu rapido dell'outer
  loop;
- l'outer aggiorna il comando a `T_control = 0.001 s` (1 kHz);
- la dinamica del motore e' integrata nello stesso step con RK4;
- quindi la velocita del driver va valutata come banda dinamica equivalente,
  non come frequenza di callback.

Frequenze naturali equivalenti:

```text
case                       fn knee [Hz]   fn ankle [Hz]
baseline 1000/700             99.39          124.91
321/500 no-retune             56.31          105.57
321/500 matched               99.39          124.91
```

Il contenuto di `tau_ref` nei dati AB06 e' molto piu lento:

```text
tau_ref knee:
  f95 ~= 2-3 Hz
  f99 ~= 13-15 Hz

tau_ref ankle:
  f95 ~= 2 Hz
  f99 ~= 3 Hz
```

Conclusione:

- rispetto al contenuto effettivo di `tau_ref`, il driver e' piu veloce;
- rispetto alla frequenza di aggiornamento digitale dell'outer loop, no: il
  comando e il driver condividono il passo da 1 ms;
- il TODO corretto e' quindi quantificare il margine di banda tra `tau_ref` e
  dinamica del driver, non pretendere un callback piu rapido.

Report/TODO aggiornato:

- `reports/user/2026-05-14_todo_motor_driver_stiffness_plot.md`

TODO tecnico lasciato:

1. abbassare l'errore di tracking di `tau_ref` senza rendere il motore troppo
   aggressivo;
2. abbassare la potenza erogata dal motore rispetto alla potenza al joint,
   favorendo il contributo elastico della molla quando possibile;
3. diminuire o eliminare il chattering in `tau_input`, `motor_speed_dot` e
   `tau_ref - tau_input`;
4. analizzare `f95/f99` di `tau_ref`, `tau_error` e `motor_speed_dot` rispetto
   a `fn = omega_n / (2*pi)` e `T_control = 1 ms`.

## 5. Outer PID e sweep dei gain

E' stato discusso il ruolo dell'outer PID:

- il sistema ha gia una struttura a cascata:

```text
outer kinematic controller -> tau_ref/u -> inner SEA torque driver -> spring torque -> joint
```

- il PID puo aiutare se l'errore residuo e' quasi-statico o a bassa frequenza;
- se il problema e' fase, compliance, ritardo del driver o contatto GRF, il
  termine integrale puo aumentare `tau_ref`, potenza motore e chattering.

Run PID full gia disponibile:

```text
results/_outer_pid_ab06_full
plot/14_05_2026 - 2
```

Metriche PID vs PD storiche:

```text
knee:
  q RMS PD  = 3.135085 deg
  q RMS PID = 3.010021 deg
  improvement ~= 3.99 %

ankle:
  q RMS PD  = 5.434958 deg
  q RMS PID = 4.847111 deg
  improvement ~= 10.82 %
```

La validazione resta:

```text
PASS = 40
WARN = 5
FAIL = 1
```

con FAIL residuo su `mtp_angle_r`, non introdotto dal PID.

Per preparare una verifica sistematica, e' stato creato:

- `validation/outer_pid_gain_sweep.py`

Caratteristiche dello script:

- baseline obbligatoria: `results/_stiff321_500_ab06_pd_full`;
- modello: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim`;
- setup: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml`;
- GRF filter sempre attivo;
- metrica primaria:

```text
mean_pros_rmse_deg = mean(RMSE(pros_knee_angle), RMSE(pros_ankle_angle))
```

Baseline letta dallo script:

```text
knee RMSE  = 3.1245995 deg
ankle RMSE = 5.4163945 deg
mean RMSE  = 4.2704970 deg
```

Strategia sweep:

```text
stage1 knee:   100 candidati x 2 finestre = 200 job
stage1 ankle:  100 candidati x 2 finestre = 200 job
full combo:     25 candidati x full AB06  = 25 job
totale:        425 job
```

Griglie:

```text
knee Kp = [80, 120, 160, 220, 300]
knee Kd = [6, 12, 18, 26]
knee Ki = [0, 10, 20, 40, 80]

ankle Kp = [250, 350, 420, 560, 750]
ankle Kd = [0.5, 1, 2, 4]
ankle Ki = [0, 30, 60, 120, 240]
```

Progress reporting implementato:

- percentuale di completamento;
- job completati/totali;
- elapsed time;
- ETA;
- ultimo candidato concluso con status `OK`, `FAIL` o `TIMEOUT`.

Timeout:

```text
screen windows: 300 s per job
full AB06:      1621.8 s per job
```

Verifiche eseguite, senza lanciare lo sweep reale:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile validation/outer_pid_gain_sweep.py

/opt/anaconda3/envs/envCMC-like/bin/python validation/outer_pid_gain_sweep.py --dry-run

/opt/anaconda3/envs/envCMC-like/bin/python validation/outer_pid_gain_sweep.py \
  --quick-smoke \
  --workers 2 \
  --sweep-root results/_outer_pid_gain_sweep_quick_smoke

/opt/anaconda3/envs/envCMC-like/bin/python validation/outer_pid_gain_sweep.py \
  --quick-smoke \
  --workers 2 \
  --timeout-minimum 0.001 \
  --sweep-root results/_outer_pid_gain_sweep_quick_smoke_timeout
```

Esito:

- `py_compile`: OK;
- `dry-run`: OK;
- quick smoke: 2 mini-run completate e registrate;
- timeout smoke: 2 timeout registrati correttamente, sweep continuato.

Output creati:

- `results/_outer_pid_gain_sweep_quick_smoke`
- `results/_outer_pid_gain_sweep_quick_smoke_timeout`

Comando consigliato per lo sweep reale:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python validation/outer_pid_gain_sweep.py --workers 6
```

Lo sweep reale non e' stato lanciato.

## File principali creati o modificati

Codice:

- `config.py`
- `main.py`
- `model_loader.py`
- `simulation_runner.py`
- `validation/outer_pid_gain_sweep.py`

Modelli:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_matched_dynamics.osim`

Risultati:

- `results/_grf_filtered_ab06_pd_full`
- `results/_stiff321_500_ab06_pd_full`
- `results/_stiff321_500_matched_dynamics_ab06_pd_full`
- `results/_outer_pid_gain_sweep_quick_smoke`
- `results/_outer_pid_gain_sweep_quick_smoke_timeout`

Plot:

- `plot/14_05_2026 - 2`
- `plot/14_05_2026 - 3`
- `plot/14_05_2026 - 4`
- `plot/14_05_2026 - 5`

Report utente:

- `reports/user/2026-05-14_validazione_simulatore.md`
- `reports/user/2026-05-14_validazione_grf_filtered_pd.md`
- `reports/user/2026-05-14_validazione_stiff321_500.md`
- `reports/user/2026-05-14_validazione_stiff321_500_matched_dynamics.md`
- `reports/user/2026-05-14_todo_motor_driver_stiffness_plot.md`

## Stato finale e prossimi passi

Stato finale:

- filtro GRF pronto, opt-in e validato su AB06;
- modello `321/500` pronto e full-run completo;
- modello `321/500 matched dynamics` pronto e full-run completo;
- analisi motor driver aggiornata con metrica di banda corretta;
- script sweep PID outer pronto e verificato in dry-run/quick-smoke;
- nessuno sweep reale lanciato.

Prossimi passi consigliati:

1. lanciare lo sweep PID reale:

   ```bash
   /opt/anaconda3/envs/envCMC-like/bin/python validation/outer_pid_gain_sweep.py --workers 6
   ```

2. confrontare il best candidate con baseline `mean_pros_rmse_deg = 4.270497`;
3. se nessuna tripla PID migliora il tracking, passare a controllo outer con
   feedforward dinamico filtrato o cascata piu strutturata;
4. continuare il lavoro sul motor driver valutando contemporaneamente
   `tau_error`, `motor_power/joint_power`, `motor_speed_dot`, saturazioni e
   contenuto frequenziale.
