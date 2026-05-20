# Daily report - 2026-05-16

## Sintesi

Giornata di **chiusura della linea diagnostica** aperta il 2026-05-15
sul disaccoppiamento BW inner/outer, seguita da una **certificazione
formale del driver in isolamento** e da un **tentativo fallito di
implementare il PI inner nel plugin C++** che ha portato alla luce un
blocco architetturale nel sub-stepper Python.

Quattro fili di lavoro, eseguiti in ordine:

1. **Bump dei guadagni inner SEA in-place sul `.osim` corrente** per
   portare entrambi i SEA al target `BW_torque ≈ 125 Hz / zeta ≈ 0.71`
   pianificato il 2026-05-15. Run full di verifica con outer PID
   quasi-best: tracking pros RMSE `-12.46 %`, `knee_tau_err_rms`
   `-62.9 %`, ma `motor_speed_dot_max` `+258 %` (atteso).

2. **Diagnosi del chattering 88 Hz** osservato visivamente nei plot del
   bump inner. Tre check FFT mirati hanno escluso il SO come sorgente
   primaria e identificato il **loop chiuso outer-D ↔ driver-inner**
   come responsabile. Test di conferma con `Kd_knee_outer 30 → 15` ha
   ridotto il chattering del `60 %` su tutti i sintomi.

3. **Driver isolation test suite (5 test, 10 worker)** in Python
   standalone. Tutti i test PASS. Verifica analitica importante:
   `zeta = 0.711 > 1/sqrt(2)` → la FdT chiusa del driver in isolamento
   NON ha picco di risonanza. **Il driver e' matematicamente corretto**;
   il chattering nel sistema completo NON e' attribuibile al driver.

4. **Tentativo plugin SEA con PI inner**: modifica sorgenti C++,
   ricompilazione, aggiornamento `.osim`, run full. PI **non operativo**
   perche' il sub-stepper Python `rk4_bypass` integra hardcoded solo
   `motor_angle/motor_speed` per ogni SEA e ignora la nuova state
   variable `torque_error_integral`. Discussione architetturale aperta
   sulla collocazione di `xi`. Rollback completo del plugin alla
   versione PD post-bump.

Report utente consolidati:

- `reports/user/2026-05-16_bump_inner_gains_run_full.md`
- `reports/user/2026-05-16_diagnosi_chattering_88Hz_loop_outer_inner.md`
- `reports/user/2026-05-16_driver_isolation_test_suite.md`
- `reports/user/2026-05-16_plugin_pi_inner_blocco_strutturale.md`

Risultato operativo principale: lo **stato di fine giornata coincide
con lo stato di inizio giornata sul piano dei file attivi** (plugin PD
post-bump, `.osim` con `Kp=18/11.3, Kd=11/11`, nessun `<Ki>`). Ma il
lavoro **diagnostico e' avanzato sostanzialmente**: il driver e'
certificato, il chattering e' compreso, il blocco architetturale per il
PI inner e' identificato.

## 1. Bump inner gains in-place sul .osim

Modifica chirurgica (4 valori cambiati su un `.osim` di 50 MB) per
portare entrambi i SEA al target di banda pianificato il 2026-05-15:

```text
SEA_Knee   linee 7561, 7563:    Kp=3.9 -> 18      Kd=9.7 -> 11
SEA_Ankle  linee 7589, 7591:    Kp=8.8 -> 11.3    Kd=9.7 -> 11
```

Target post-modifica: `omega_n ≈ 785 rad/s, zeta ≈ 0.71,
f_d ≈ 88 Hz` per entrambi (per design dell'utente, knee e ankle ora
sono dinamicamente equivalenti).

Backup salvato come `*.absD_inner_3p9_8p8_backup_20260516`.

Run full `[11.99, 21.00] s` con outer PID quasi-best
(`knee 340/30/120 + ankle 850/2/300`), output in
`results/_fast_inner_pid_20260516/`. Status `complete`,
`wall=742.6 s`, 0 saturazioni, 0 reserves protesiche.

Confronto vs baseline best PID (sweep notturno `_outer_pid_gain_sweep_20260514_223838/.../combo_kkp300_kkd26_kki80_akp750_akd2_aki240`):

```text
metric                       baseline    fast_inner       delta
mean_pros_rmse_deg              2.1403      1.8735       -12.46 %
knee_rmse_deg                   1.5284      1.2958       -15.22 %
ankle_rmse_deg                  2.7522      2.4512       -10.93 %
knee_tau_err_rms  [Nm]            3.31        1.23       -62.90 %
ankle_tau_err_rms [Nm]            1.05        1.00        -4.81 %
knee_mdot_max  [rad/s2]         2505.7      8962.4      +257.69 %
```

Ipotesi del 2026-05-15 sul knee ("BW insufficiente del driver inner")
**confermata** dai numeri: tracking migliora, errore di coppia crolla,
nessuna instabilita'. Il knee passa da `BW_inner ~32 Hz` a `~125 Hz`,
ankle da `~111 Hz` a `~125 Hz`.

Plot in `plot/05_16_2026_1/` (6 PNG: sea_control_reserve,
joint_motor_states, gait_cycle_torque/angle/power, tau_input_tracking_error,
joint_ref_sea_error).

## 2. Diagnosi del chattering 88 Hz

Ispezione visiva dei plot del bump inner ha mostrato chattering
notevole su 4 segnali: `tau_input knee`, `motor_speed knee`,
`motor_speed ankle`, `joint_speed ankle`. Tre check FFT mirati hanno
permesso di assegnare le responsabilita'.

### Check (a) - SO come sorgente?

Spettro di `sim_output_tau_bio.sto` su 7 DoF biologici (hip, knee,
ankle, pelvis, lumbar):

```text
tau_bio[*]   FAST mag a 86 Hz   KDOUT mag a 86 Hz   delta
identico ovunque (1.48e3 hip_l, 1.24e3 pelvis_tilt, 676 hip_r, ...)
```

Identico tra FAST e KDOUT (run con `Kd_knee_outer/2`). Se il SO fosse
sorgente, dimezzare `Kd_outer` non avrebbe effetto. Esclude SO.

### Check (b) - Riferimento IK come sorgente?

```text
                              peak       mag       E_88/E_lf
q_REF pros_knee   (file IK)   83.0 Hz   2.49      5.4e-06
q_SIM pros_knee   (FAST)      86.2 Hz   3.80e-02  1.2e-08

q_REF pros_ankle  (file IK)   96.7 Hz   6.23      7.8e-05
q_SIM pros_ankle  (FAST)      86.2 Hz   1.93e-01  1.6e-06
```

Frequenze diverse (83/96.7 Hz nel ref vs 86.2 Hz nel simulato),
magnitudini 3-6 ordini di grandezza piu' piccole nel simulato (filtraggio
naturale del loop chiuso). Esclude riferimento.

### Check (c) - Decomposizione outer P/D/I

```text
joint   knob              P-cmd mag   D-cmd mag   I-cmd mag
Knee    FAST  (Kd=30)         12.9        623       8.5e-3
Knee    KDOUT (Kd=15)         10.7        259       7.1e-3
Ankle   FAST  (Kd=2)         164          210       1.1e-1
Ankle   KDOUT (Kd=2 inv.)    158          202       1.0e-1
```

Sul knee il D-cmd a 86 Hz **dimezza esattamente** quando dimezzo
`Kd_knee_outer`. P quasi invariato, I trascurabile.

### Diagnosi

Non e' "rumore numerico che eccita una risonanza". E' un **modo
oscillatorio del loop chiuso outer-D ↔ driver-inner** auto-sostenuto:

```text
1. driver inner ha autovalore complesso a f_d = 88 Hz, Q = 0.71
2. piccola perturbazione broadband eccita il modo
3. motor_speed oscilla microscopicamente a 88 Hz
4. molla trasmette al giunto pros (mag spettrale 3.8e-2 rad)
5. outer D deriva la vibrazione, amplifica per Kd_outer (623 vs 259)
6. D-cmd entra in tau_ref outer -> chiude il loop alla risonanza
```

Test di conferma `Kd_knee_outer 30 → 15` lanciato come run full
`_fast_inner_kdouter15_20260516`:

```text
                              FAST     KDOUT     delta
knee_rmse_deg                1.296    1.433     +10.6 %
mean_rmse                    1.874    1.917      +2.3 %
knee_tau_err_rms  [Nm]       1.229    0.970    -21.1 %
knee_mdot_max  [rad/s2]      8962     4015     -55.2 %

CHATTERING 86 Hz                                  
tau_input KNEE  mag a 86 Hz   8.0e3    3.3e3    -59 %
motor_speed KNEE  mag a 86 Hz  901      373    -59 %
outer_D_cmd KNEE  mag a 86 Hz  623      259    -58 %
ANKLE (Kd_outer invariato)                     ~0 % invariato
```

Tracking peggiora del 10.6 % sul knee (ancora meglio della baseline
best PID), ma chattering crolla del 60 %. Conferma il loop come
canale dominante. Plot in `plot/05_16_2026_2/`.

## 3. Driver isolation test suite (5 test)

Domanda di fondo: come capire se il motor driver e' "corretto" in
isolamento (senza outer PID, senza SO, senza dinamica del corpo)?

Suite di 5 test in Python standalone (`tools/driver_isolation_suite.py`,
~580 LOC) che riproducono la formula del driver non-impedance attiva in
`output.py:471-476`. Esecuzione `wall = 43.8 s` con `10 worker` (coda
condivisa longest-job-first, 128 job paralleli su Test 2+3+4 + 4
seriali su Test 1, 5).

### Test 1 - Step response (omega_j=0)

```text
SEA          tau_spring_ss   e_ss [Nm]     overshoot [%]   settling 2% [ms]
SEA_Knee       50.000          5e-13          4.11           7.81
SEA_Ankle      50.000          1e-12          4.30           7.63
```

Errore a regime sotto 1 pNm, overshoot coerente con teoria
(`zeta=0.71 → 4.5 %`), settling sotto la previsione 9 ms.

### Test 2 - Bode magnitude/phase (50 freq)

Sovrapposizione perfetta simulato vs analitico, `|H(0)| = 0 dB`,
BW -3 dB a ~100 Hz, rolloff -40 dB/decade, fase smooth da 0 a -160°.

### Test 3 - Sensitivita' al disturbo omega_j (10 freq)

Plateau DC a `0.584 Nm/(rad/s)` knee e `0.902` ankle, esattamente
sulla teoria. Niente peak intorno a 88 Hz.

### Test 4 - Bias a omega_j costante (4 velocita')

RMS residuo `0.0000 Nm` su entrambi i SEA. Punti osservati cadono
esattamente sulle rette teoriche `-0.584*omega_j` (knee) e
`-0.902*omega_j` (ankle).

### Test 5 - Risposta a noise broadband (PSD)

Niente picco di risonanza significativo (peaking `0.7-0.8 dB` =
transizione plateau→rolloff, non amplificazione). Coerente con
`zeta = 0.711 > 1/sqrt(2) = 0.7071` → FdT chiusa monotonicamente
decrescente.

### Verdetto

**Il driver e' matematicamente corretto** per il tuning post-bump.
Chattering, errore cinematico e picco residuo del sistema completo
**non sono attribuibili al driver in isolamento**. Conferma indipendente
della diagnosi del check (a)+(b)+(c) del mattino.

Plot in `plot/05_16_2026_3_driver_isolation/` (10 PNG: step, bode,
disturbance, bias, noise per knee e ankle).

PI inner: l'analisi mostra che il PI risolverebbe solo il bias DC
`-(Kd+Bm)*omega_j/(1+Kp) ≈ 1.7 Nm` a `omega_j ≈ 3 rad/s` (cammino).
Non risolve il chattering (che e' del loop chiuso, non del driver).

## 4. Tentativo plugin SEA con PI inner (failed)

Decisione operativa di fine pomeriggio: implementare il PI inner nel
plugin C++ per cancellare il bias DC residuo identificato dal Test 4
della isolation suite.

### Sequenza completata (poi ripristinata)

```text
Phase A  modifica sorgenti C++ (Ki property, state torque_error_integral,
         getMotorTorque con +Ki*xi, computeStateVariableDerivatives con xi_dot)
Phase B  validazione analitica Ki: omega_int=30 rad/s -> Ki_knee=570, Ki_ankle=369
         max |H_PI - H_PD| = 0.49 dB su 0-200 Hz (entro vincolo 0.5 dB)
Phase C  ricompilazione (cmake regenerate per fix target name) + install .dylib
Phase D  aggiunta di <Ki>570</Ki> / <Ki>369</Ki> ai due blocchi SEA del .osim
Phase E  run full [11.99, 21.00] s outer PID quasi-best
         status=complete, wall=770 s, 0 saturazioni
```

### Risultato: PI non operativo

Metriche identiche alla run FAST (no PI):

```text
                       FAST (no PI)   PI v1     delta
knee_rmse_deg            1.296        1.296     invariato
ankle_rmse_deg           2.451        2.451     invariato
knee_tau_err_rms [Nm]    1.229        1.229     invariato
ankle_tau_err_rms [Nm]   1.004        1.004     invariato
```

Diagnostica plugin vs Python predictor:

```text
tau_input_plugin - tau_input_python    knee  RMS = 0.000000 Nm
                                       ankle RMS = 0.000001 Nm
```

Il Python predictor (`output.py:471-476`) ha formula PD; se il plugin
applicasse `Ki*xi`, ci sarebbe differenza ≠ 0. Invece sono identici.

### Diagnosi del blocco

Plugin OK: `Ki=570/369` letto correttamente, state variable
`/forceset/SEA_*/torque_error_integral` registrata. Verificato via
`opensim.LoadOpenSimLibrary + Model.initSystem`.

**Blocco strutturale nel simulator Python**: `rk4_bypass` in
`simulation_runner.py:_advance_rk4_bypass_state:832` integra hardcoded
`n_coords*2 + n_sea*2` stati (2 per SEA). Le state variables ulteriori
del plugin vengono ignorate. `xi` resta a `0.0` per tutta la
simulazione → `Ki*xi = 0` sempre.

### Discussione architetturale

L'utente ha sollevato un'obiezione di principio: `xi` non e' uno stato
fisico del motore (gli stati del motore sono `theta_m, omega_m`),
quindi non deve essere registrato come state variable OpenSim. La
logica del PI deve restare dentro il plugin.

Quattro alternative discusse:

```text
1. xi come state variable + estensione del rk4_bypass    [rifiutato dall'utente]
2. xi come mutable double nel plugin + Euler interno     [problemi tecnici con RK4
                                                          multi-stage + sub-step]
3. Disabilitare rk4_bypass, tornare a OpenSim Manager    [rischio regressione
                                                          performance del 2026-04-20]
4. Plugin separato SEA_PI_Controller                     [da progettare]
```

Discusso a fondo il problema 1 del punto 2 (RK4 chiama il plugin 4
stage × 5 sub-step = 20 chiamate per step di 1 ms; nessun modo pulito
di integrare xi internamente senza perdere precisione, rollback e
introspezione standard). Conclusione: la decisione architetturale e'
**deferita**.

### Rollback completo

Tutte le modifiche ripristinate dai backup:

```text
SEA_plugin_core - agent/SeriesElasticActuator.cpp           <- pre_pi_backup_20260517
SEA_plugin_core - agent/SeriesElasticActuator.h             <- pre_pi_backup_20260517
plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib      <- no_pi_backup_20260517
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim <- no_ki_backup_20260517
```

Stato corrente: plugin PD pulito (no Ki, no xi), `.osim` con
`Kp=18/11.3, Kd=11/11` (post-bump), dimensione `.dylib` 217664 byte
(coincide con backup).

## File principali creati / modificati

Codice Python:

- `tools/driver_isolation_suite.py` (nuovo, ~580 LOC, regression test
  riusabile per future modifiche del driver)

Modello:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim`
  (modifica permanente: bump inner gains; PI rollback)
- backup di sicurezza: `*.absD_inner_3p9_8p8_backup_20260516`,
  `*.no_ki_backup_20260517`

Plugin (stato finale = stato iniziale dopo rollback):

- `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib` (invariato
  net del rollback)
- backup PI: `*.no_pi_backup_20260517` (conservato per riferimento)

Sorgenti C++ (stato finale = stato iniziale dopo rollback):

- `SEA_plugin_core - agent/SeriesElasticActuator.{cpp,h}` (invariati
  net del rollback)
- backup PI: `*.pre_pi_backup_20260517` (conservato)

Risultati:

- `results/_fast_inner_pid_20260516/` (bump inner, run full, 18 file)
- `results/_fast_inner_kdouter15_20260516/` (test Kd_outer/2)
- `results/_driver_isolation_20260516/` (suite 5 test, JSON+NPZ)
- `results/_pi_inner_20260517/` (run PI fallita, conservata per
  riferimento)

Plot:

- `plot/05_16_2026_1/` (bump inner, 6 PNG)
- `plot/05_16_2026_2/` (Kd_outer=15, 6 PNG)
- `plot/05_16_2026_3_driver_isolation/` (5 test, 10 PNG)
- `plot/05_17_2026_1/00_reserve_torques_compare.png` (confronto reserve
  4 run)

Report utente:

- `reports/user/2026-05-16_bump_inner_gains_run_full.md`
- `reports/user/2026-05-16_diagnosi_chattering_88Hz_loop_outer_inner.md`
- `reports/user/2026-05-16_driver_isolation_test_suite.md`
- `reports/user/2026-05-16_plugin_pi_inner_blocco_strutturale.md`

## Stato finale e prossimi passi

### Stato finale

- driver SEA: abs-D, PD puro, plugin invariato (rollback completo PI);
- `.osim` con bump inner permanente (`Kp=18/11.3, Kd=11/11`);
- outer PID quasi-best `340/30/120 + 850/2/300` come configurazione
  raccomandata (CLI);
- driver certificato in isolamento (5 test PASS);
- chattering 88 Hz diagnosticato come loop chiuso outer-D ↔ driver-inner,
  NON come problema del driver;
- PI inner non implementato (decisione architetturale deferita).

### Prossimi passi consigliati (in ordine)

```text
1. DECIDERE l'architettura del PI inner:
   - opzione 1 (state variable + estensione rk4_bypass)
   - opzione 2 (mutable nel plugin + Euler interno)
   - opzione 3 (disabilitare rk4_bypass, OpenSim Manager)
   - opzione 4 (Controller separato SEA_PI_Controller)

2. INTERVENTO sul chattering 88 Hz, separato dalla questione PI:
   - sweep di Kd_knee_outer in {10, 15, 20, 25, 30} per il sweet spot
     tracking vs chattering;
   - oppure LPF sul D-term outer (fc 25-40 Hz) come intervento
     chirurgico che non sacrifica autorita' del D in banda;
   - oppure bump Kd_inner per portare zeta a 1.0 (costo -30 % BW).

3. PIPELINE Windows da allineare (vedi TODO sotto). Operazione separata
   da 1 e 2.

4. Possibile: usare la driver isolation suite come **regression test**
   nel CI dopo ogni modifica al plugin. Esecuzione < 1 min.
```

## TODO aperti (propagati e nuovi)

Propagati da 2026-05-15:

- **TODO Windows pendente** (dal 2026-05-15): compilare/copiare la
  DLL `_ff` sulla macchina di Tommy. Replica del bump inner gains nel
  `.osim` (gia' presente nel `.osim` cross-platform). Esecuzione dei
  passi smoke + dry-run + quick-smoke per validare l'ambiente.

Nuovi:

- **Decisione architetturale PI inner**: 4 opzioni discusse, nessuna
  scelta. Bloccante per ogni implementazione del PI.
- **Intervento per il chattering 88 Hz**: 3 opzioni discusse (sweep
  Kd_outer, LPF su D-term, bump zeta inner). Indipendente dalla
  questione PI.
- **Aggiornamento del Python predictor in `output.py:471-476`** per
  includere `Ki*xi` (quando il PI sara' operativo, per mantenere
  `tau_input_plugin ≡ tau_input_python` nella diagnostica).
- **Driver isolation suite estesa con formula PI** per regression
  test (utile dopo l'implementazione del PI).
- **TODO Windows arricchito**: includere anche la decisione PI inner
  e l'intervento chattering 88 Hz quando saranno finalizzati.
- **`results/_pi_inner_20260517/`** conservato come riferimento per la
  diagnostica del blocco; eventualmente da cancellare quando il PI
  sara' operativo.
