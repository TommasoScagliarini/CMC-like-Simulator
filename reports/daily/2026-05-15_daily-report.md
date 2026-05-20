# Daily report - 2026-05-15

## Sintesi

Giornata centrata su due fili paralleli, intrecciati a meta' giornata:

1. **chiusura amministrativa del lavoro accumulato** dal 2026-05-11: lettura
   del best candidate dello sweep PID notturno, rinaming delle cartelle
   plot in formato `MM_DD_AAAA_iter`, preparazione TODO Windows per
   allineare la macchina di Tommy con lo stato macOS;
2. **investigazione strutturale del sistema di controllo SEA**: tre
   esperimenti diagnostici (relative-D, LPF su `u` outer, analisi
   bandwidth cascata) che, presi insieme, hanno spostato il focus dal
   tuning dei guadagni al **mancato disaccoppiamento frequenziale tra
   driver inner e outer PID**.

Risultato operativo principale: il problema del chattering del motor
driver e della sensibilita' di tracking del cammino non e' di tuning, e'
**strutturale**. Sul ginocchio il rapporto `BW_inner / BW_outer ≈ 0.8`
(invertito rispetto alla rule-of-thumb cascata `5-10x`); sull'ankle e'
`~4x`, borderline. La prossima mossa concordata e' alzare la BW del
motor driver tramite bump di `Kp_inner` e `Kd_inner` nell'`.osim`, senza
ancora toccare la struttura outer o ricompilare il plugin.

Report utente consolidati:

- `reports/user/2026-05-15_pid_sweep_best_candidate_plot_naming.md`
- `reports/user/2026-05-15_todo_windows_outer_pid_local_sweep_alignment.md`
- `reports/user/2026-05-15_todo_windows_ab06_work_since_2026-05-11.md`
- `reports/user/2026-05-15_esperimento_relative_d_motor_driver.md`
- `reports/user/2026-05-15_sweep_lpf_u_outer_falsificazione_bandwidth.md`
- `reports/user/2026-05-15_analisi_strutturale_bandwidth_cascata_e_proposta_impedance_ff.md`
- `reports/user/2026-05-15_pi_inner_e_target_bandwidth_motor_driver.md`

## 1. PID sweep best candidate e rinominazione plot

Lo sweep notturno (`results/_outer_pid_gain_sweep_20260514_223838`) ha
prodotto 425 job: 100x2 windows knee, 100x2 windows ankle, 25 full AB06.

Best candidate full:

```text
combo_kkp300_kkd26_kki80_akp750_akd2_aki240

pros_knee_angle:   Kp=300  Kd=26  Ki=80
pros_ankle_angle:  Kp=750  Kd=2   Ki=240
```

Rispetto alla baseline PD filtrata `321/500`:

```text
metric                 baseline PD    best PID
knee RMS error         3.1246 deg     1.5284 deg
ankle RMS error        5.4164 deg     2.7522 deg
mean pros RMS          4.2705 deg     2.1403 deg     (-49.88 %)
```

Il candidato e' stabile sull'intera finestra `11.99-21.00 s`. Validator:
`PASS=38, WARN=7, FAIL=1` (FAIL sempre su `mtp_angle_r output vs IK`, non
introdotto dal PID).

Lato dinamico SEA il PID **peggiora** rispetto al PD baseline:

```text
metric                         baseline PD    best PID
knee tau_error RMS              3.11 Nm        3.31 Nm
ankle tau_error RMS             0.98 Nm        1.05 Nm
knee speed_dot RMS             92.25 rad/s2  271.18 rad/s2
ankle speed_dot RMS           128.03 rad/s2  345.31 rad/s2
knee speed_dot max            970.35        2505.66 rad/s2
ankle speed_dot max          2570.59        5747.31 rad/s2
```

Cioe' tracking cinematico nettamente migliore, ma motor driver piu' nervoso.
Da qui la domanda del giorno: il chattering del motor driver e' bandwidth
del comando o struttura del driver?

Plot del best candidate generati in `plot/05_15_2026_1/`. Tutte le cartelle
plot storiche rinominate dal vecchio `DD_MM_AAAA - iter` al nuovo
`MM_DD_AAAA_iter`. `plot/plotter.py` aggiornato a creare direttamente
cartelle nel nuovo formato.

## 2. Allineamento Windows

Documentati in due TODO i passaggi necessari per portare lo stato di lavoro
di oggi e degli ultimi giorni su Windows:

```text
reports/user/2026-05-15_todo_windows_outer_pid_local_sweep_alignment.md
reports/user/2026-05-15_todo_windows_ab06_work_since_2026-05-11.md
```

Blocco principale: la macchina Windows ha solo la vecchia DLL
`plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll`, non la variante `ff` usata
dalle run recenti su macOS
(`plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`). Va
compilata/copiata la DLL `ff` su Windows prima di considerare la macchina
allineata. Definiti i passaggi smoke + dry-run + quick-smoke per validare
l'ambiente.

Migliorie cross-platform applicate:

- `tools/build_abx_seasea_pipeline.py` e
  `scripts/run_opensim_sea_pipeline.py` ora quotano correttamente i comandi
  con path che contengono spazi;
- commento `T_control = 0.001` in `config.py` corretto a `1 ms`;
- `validation/outer_pid_local_sweep.py` rispetta `CONDA_PREFIX` solo se
  l'ambiente attivo e' `envCMC-like`;
- `validation/outer_pid_gain_sweep.py` usa lista di argomenti `shell=False`
  per evitare problemi di parsing dei path.

Lo sweep locale `outer_pid_local_sweep.py` con griglia `3x3x3` attorno al
best (133 job totali) e' stato preparato e validato in `--dry-run` e
`--quick-smoke` su macOS, **non lanciato in full**.

## 3. Esperimento relative-D nel motor driver

Esperimento chirurgico: sostituire `-Kd*omega_m` con `-Kd*(omega_m - omega_j)`
nel driver non-impedance, mantenendo tutto il resto invariato (stiffness
`321/500`, outer PID quasi-best `knee 340/30/120` e `ankle 850/2/300`,
filtro GRF, plugin abs-D ripristinato dal backup).

Razionale: scomporre il damping abs-D
`-Kd*omega_m = -Kd*(omega_m - omega_j) - Kd*omega_j` rivela che il termine
contiene sia damping della deflessione molla sia un trascinamento
proporzionale a `omega_j` (~2 Nm/(rad/s) knee, ~1 ankle), che e' damping
"spurio" rispetto alla molla.

Modifiche:

- `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp:139` legge
  `-Kd*(omega_m - omega_joint)`;
- `output.py:473` e `prosthesis_controller.py:354` aggiornati per
  rispecchiare la stessa legge;
- nuovo `.dylib` compilato e installato come
  `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`;
- backup abs-D salvato come
  `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.abs_omega_backup_20260515`.

Smoke breve: match plugin/Python entro `1e-6 Nm`.

Full run con outer PID quasi-best:

```text
results/_relative_d_pid_tradeoff_full_20260515
status=failed
t=12.666 s
step=676
error_type=FloatingPointError
error=Non-finite RK4 accelerations on pelvis DoFs
```

Le reserve sono esplose progressivamente:

```text
t=12.090   tau_res=105 Nm
t=12.240   tau_res=2284 Nm
t=12.540   tau_res=10970 Nm
t=12.666   stop
```

Plot parziali: `plot/05_15_2026_3/`.

Interpretazione: la legge abs-D non era solo damping della molla, era
**anche stabilizzatore implicito del giunto via SEA**. Rimuovendola in
drop-in il giunto perde ~100x del suo damping a 9-21 Hz, l'outer PID
tarato sul plant abs-D perde fase, oscilla, satura, le reserve esplodono.

Da qui sono partite due indagini parallele.

## 4. Sweep LPF su `u` outer (falsificazione ipotesi bandwidth)

Ipotesi: il chattering del driver dipende dalla banda di `tau_ref` (cioe'
dello scaling di `u`). Test diagnostico: ripristinare abs-D, applicare un
LPF di primo ordine su `u` per ciascun SEA, sweep su `fc`.

Modifiche:

- `output.py:473` e `prosthesis_controller.py:354` riportati ad abs-D
  (`-Kd * omega_m`);
- aggiunto stato `_u_filtered`/`_u_filtered_initialised` e helper
  `_apply_u_lpf` in `prosthesis_controller.py`;
- aggiunto campo `sea_u_lpf_cutoff_hz` (dict per coord) in `config.py`;
- aggiunte CLI `--sea-u-lpf-knee` e `--sea-u-lpf-ankle` in `main.py`;
- nuovo `validation/lpf_sweep_metrics.py` per raccogliere RMSE protesico,
  `tau_error`, `motor_speed_dot`, saturazioni, reserves dalle run.

Sweep `fc = [off, 200, 100, 50, 25, 15, 10]` su finestra `[12.0, 14.5] s`
con outer PID quasi-best e `.dylib` abs-D ripristinato.

Risultati:

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

Conclusione netta: **ipotesi falsificata**. Filtrare `u` non riduce il
chattering, lo aggrava e devasta il tracking gia' a `fc = 200 Hz`. Il
chattering della baseline `fc=off` esiste senza filtro, quindi non e'
iniettato da `tau_ref`. Il LPF inietta phase lag che si scontra con
l'autorita' integrale dell'outer PID e destabilizza il loop chiuso.

Diagnostica spostata: il chattering e' proprieta' **del loop chiuso**
(driver + plant + outer), non del contenuto in banda di `tau_ref`.

## 5. Analisi strutturale del disaccoppiamento bandwidth

Verifica della rule-of-thumb cascata `BW_inner >> BW_outer` (5-10x):

**Inner loop SEA abs-D**, FdT `tau_spring / tau_ref`:

```text
                 omega_n         zeta          BW efficace
SEA_Knee   K=321 Kp=3.9  -> 397   1.24    ~32 Hz (polo dominante overdamped)
SEA_Ankle  K=500 Kp=8.8  -> 700   0.70    ~111 Hz
```

**Outer loop PID**, crossover `|L(j*omega_c)| = 1`:

```text
Knee   (Kp=340 Kd=30 Ki=120): omega_c ≈ 250 rad/s ≈ 40 Hz
Ankle  (Kp=850 Kd=2  Ki=300): omega_c ≈ 170 rad/s ≈ 27 Hz
```

Confronto:

```text
              BW_inner   BW_outer   ratio   giudizio
Knee          ~32 Hz     ~40 Hz     0.8x    INVERTITO
Ankle         ~111 Hz    ~27 Hz     4x      borderline
```

Sul knee non c'e' disaccoppiamento: outer piu' veloce dell'inner.
Strutturalmente e' un singolo loop nervoso, non una cascata. Spiega tutti
e tre i fenomeni di oggi:

- chattering motore intrinseco (loop al limite della sua banda);
- best PID con piu' chattering del PD (banda outer alzata, ratio
  peggiorato);
- LPF instabile (qualsiasi distorsione di fase consuma il margine
  residuo);
- rel-D in drop-in esploso (rimuove margine senza compensare).

## 6. Discussione: alternative outer e errore a regime

Sono state discusse tre direzioni strutturali:

1. **Cascade impedance + feedforward dinamico filtrato**: outer impedance
   pura (`K_imp` e `B_imp` come parametri di design, no `Ki`), `tau_ff`
   da inverse dynamics filtrato a `fc_ff < BW_inner / 5`. Architettura
   ortodossa per SEA in cammino. Tommy ha pero' osservato che `tau_ff` da
   ID non e' portabile su una protesi reale (no force plate, no modello
   biomeccanico, no muscoli). Le sostituzioni real-world sono profili
   `tau` per fase del gait, learned policy, CPG, o detector di eventi.
2. **Cascade P-pos + PI-vel** (proposta di Tommy): outer `qdot_ref =
   Kp_pos*(q_ref - q)`, mid `tau_ref = Kp_v*(qdot_ref - qdot) +
   Ki_v*integral(...)`, inner SEA invariato. Architettura classica
   servo motion control, naturalmente bandwidth-limited a livello di
   posizione, senza windup integrale.
3. **PI sull'errore di coppia nel driver** (al posto del PD attuale): per
   eliminare l'errore di tracking di coppia residuo quando il giunto si
   muove.

Tommy ha poi messo in luce un punto fondamentale: nell'equazione di
closed-loop del driver abs-D, lato destro non e' zero a regime "del
controllore" ma resta `-(Kd+Bm)*omega_j - Jm*omega_j_dot`, quindi il P
sull'errore di coppia + damping **non** rimuove il bias quando il giunto
si muove (caso del cammino).

Caso (a), giunto fermo:

```text
tau_spring_ss = tau_ref      <- errore zero (merito del feedforward (1+Kp)*tau_ref)
```

Caso (b), giunto a velocita' costante:

```text
tau_spring - tau_ref = -(Kd+Bm)*omega_j / (1+Kp)
                     = -2.00 * omega_j  Nm  (knee)
                     = -1.00 * omega_j  Nm  (ankle)
```

A `omega_j ≈ 5 rad/s` sono ~10 Nm di errore di coppia non eliminato. Da
qui la motivazione del PI sull'errore di coppia.

Dimostrazione algebrica di PI + damping:

```text
tau_input = tau_ref + Kp*(tau_ref - tau_spring) + Ki*xi - Kd*omega_m
xi_dot    = tau_ref - tau_spring

Closed-loop in e = theta_m - theta_j:
Jm*e_ddot + (Kd+Bm)*e_dot + (1+Kp)*K*e - Ki*xi = (1+Kp)*tau_ref - (Kd+Bm)*omega_j - Jm*omega_j_dot
xi_dot = tau_ref - K*e

A regime con omega_j costante:
xi_dot = 0  =>  tau_spring_ss = tau_ref          <-- errore zero
xi_ss = (Kd+Bm)*omega_j / Ki                     <-- finito, scala col disturbo
```

L'integratore si "carica" a `xi_ss = (Kd+Bm)*omega_j/Ki` per compensare
esattamente il trascinamento, e l'errore di coppia va a zero.

## 7. Decisione: target di bandwidth e priorita' inner

Per il cammino il target ragionevole:

```text
BW_position ≈ 5 Hz       (cattura f95 = 2-3 Hz con margine)
BW_velocity ≈ 25 Hz      (5x rispetto a BW_position)
BW_torque   ≈ 125 Hz     (5x rispetto a BW_velocity)
```

Gap rispetto allo stato attuale:

```text
              BW_attuale     BW_target     gap
Knee          ~32 Hz         125 Hz        ~4x da recuperare
Ankle         ~111 Hz        125 Hz        ~1.1x (gia' vicino)
```

Stima dei nuovi guadagni inner (`omega_n_target ≈ 785 rad/s`,
`zeta = 0.7`):

```text
SEA_Knee:  Kp = 18    Kd = 11     (era 3.9, 9.7)
SEA_Ankle: Kp = 11.3  Kd = 11     (era 8.8, 9.7)
```

Esposti come `<Kp>` e `<Kd>` nell'`.osim`: **non serve ricompilare il
plugin**. Modifica single-variable rispetto agli esperimenti precedenti.

Sequenza concordata:

```text
1. creare AB06_SEASEA_stiff321_500_fast_inner.osim con i nuovi Kp/Kd inner
2. run full con outer PID quasi-best, stessa finestra di confronto
3. raccogliere metriche e confrontare con baseline
4. se l'inner e' "veloce abbastanza", decidere se basta o se serve PI
   inner (richiede modifica plugin C++ + ricompilazione)
5. solo dopo, valutare cambio struttura outer (cascade P-pos + PI-vel)
```

Vantaggio operativo: la decisione su bandwidth (su cui c'e' consenso) e
quella sulla struttura outer (su cui aspettiamo i dati) sono trattate
separatamente, in ordine.

## File principali creati o modificati

Codice Python:

- `config.py` (campo `sea_u_lpf_cutoff_hz`)
- `main.py` (CLI `--sea-u-lpf-knee/ankle`)
- `prosthesis_controller.py` (LPF su `u`, helper `_apply_u_lpf`,
  ripristino abs-D nel feasibility predictor)
- `output.py` (ripristino abs-D nella diagnostica SEA)

Codice C++ (sorgente di lavoro):

- `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp` (legge rel-D, non
  piu' attiva nel plugin installato)

Plugin:

- `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib`
  (ripristinato abs-D dal backup, stato attuale)
- `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib.abs_omega_backup_20260515`
  (backup conservato)

Plotter:

- `plot/plotter.py` (creazione cartelle in formato `MM_DD_AAAA_iter`,
  rinaming storico fatto)

Validation:

- `validation/outer_pid_local_sweep.py` (creato, sweep locale `133` job
  intorno al best, validato in dry-run + quick-smoke)
- `validation/lpf_sweep_metrics.py` (creato, collector metriche LPF)

Risultati:

- `results/_outer_pid_gain_sweep_20260514_223838/` (sweep PID notturno
  consolidato e analizzato)
- `results/_relative_d_pid_tradeoff_smoke_20260515/`
- `results/_relative_d_pid_tradeoff_full_20260515/` (failed t=12.666 s)
- `results/_lpf_smoke_absD_off_20260515/`
- `results/_lpf_smoke_absD_fc25_20260515/`
- `results/_lpf_sweep_25s_fcoff_20260515/`
- `results/_lpf_sweep_25s_fc200_20260515/` (rotto)
- `results/_lpf_sweep_25s_fc100_20260515/` (crash)
- `results/_lpf_sweep_25s_fc50_20260515/` (divergente)
- `results/_lpf_sweep_25s_fc25_20260515/` (crash)
- `results/_lpf_sweep_25s_fc15_20260515/` (killed)
- `results/_lpf_sweep_25s_fc10_20260515/` (divergente)
- `results/_lpf_sweep_log/sweep_20260515_120800.log`
- `results/_lpf_sweep_log/metrics_20260515.json`

Plot:

- `plot/05_15_2026_1/` (best PID candidate)
- `plot/05_15_2026_3/` (rel-D failed run, parziale)
- rinominate tutte le cartelle storiche da `DD_MM_AAAA - iter` a
  `MM_DD_AAAA_iter`

Modelli:

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim` (invariato)
- `AB06_SEASEA_stiff321_500_fast_inner.osim` (**da creare** come prossimo
  passo, con `SEA_Knee Kp=18, Kd=11` e `SEA_Ankle Kp=11.3, Kd=11`)

Report utente:

- `reports/user/2026-05-15_pid_sweep_best_candidate_plot_naming.md`
- `reports/user/2026-05-15_todo_windows_outer_pid_local_sweep_alignment.md`
- `reports/user/2026-05-15_todo_windows_ab06_work_since_2026-05-11.md`
- `reports/user/2026-05-15_esperimento_relative_d_motor_driver.md`
- `reports/user/2026-05-15_sweep_lpf_u_outer_falsificazione_bandwidth.md`
- `reports/user/2026-05-15_analisi_strutturale_bandwidth_cascata_e_proposta_impedance_ff.md`
- `reports/user/2026-05-15_pi_inner_e_target_bandwidth_motor_driver.md`

## Stato finale e prossimi passi

Stato finale:

- driver SEA: abs-D, ripristinato dal backup, attivo;
- diagnostica Python e feasibility predictor: abs-D, allineati;
- LPF su `u` outer implementato e disponibile via CLI, ma non utile
  (ipotesi falsificata);
- sweep PID locale `133` job pronto, non lanciato;
- modello `fast_inner` con guadagni inner alzati: **da creare**;
- nessun run con i nuovi guadagni inner ancora eseguito;
- TODO Windows aggiornato e completo.

Prossimi passi consigliati, in ordine:

1. **Creare** `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_fast_inner.osim`
   con `SEA_Knee Kp=18 Kd=11` e `SEA_Ankle Kp=11.3 Kd=11`.
2. **Lanciare** una run full con outer PID quasi-best
   (`knee 340/30/120` e `ankle 850/2/300`) sul nuovo modello.
3. **Confrontare** con baseline `_outer_pid_gain_sweep_20260514_223838/full_runs/combo_kkp300_kkd26_kki80_akp750_akd2_aki240`
   sulle metriche `mean_pros_rmse_deg`, `tau_error RMS`,
   `motor_speed_dot RMS/max`, saturazioni `tau_input`, `reserve_norm RMS`.
4. **Decidere** in base ai numeri:
   - se il chattering cala e il tracking migliora, sweep su `Kp_inner`
     per ottimizzare;
   - se l'errore residuo a regime e' ancora problematico, passare a PI
     inner (riscrittura plugin C++ + ricompilazione);
   - solo dopo che l'inner e' "veloce abbastanza", valutare se serve
     cambiare struttura outer (cascade P-pos + PI-vel, o impedance + FF
     reale-world).
5. **Sweep locale PID** (`outer_pid_local_sweep.py`) attorno al best
   eventualmente rimandato: prima allarghiamo la BW inner, poi
   ottimizziamo l'outer su un plant migliore.
