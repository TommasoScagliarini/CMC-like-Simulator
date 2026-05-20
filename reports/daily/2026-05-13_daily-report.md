# Daily report - 2026-05-13

## Sintesi

Questo daily consolida tutti i report utente datati `2026-05-13`.

La giornata ha chiuso quattro filoni principali sul caso `AB06_SEASEA_Threadmill`
e sulla generalizzazione della pipeline:

- pulizia della segmentazione gait-cycle da GRF, senza cambiare la soglia fisica
  di contatto;
- aggiunta del dataset healthy overlay AB06 per confronti cinematici e dinamici;
- creazione di una pipeline unica per costruire bundle `ABx_SEASEA` da soggetti
  EPIC locali;
- analisi dei plot `plot/13_05_2026 - 3` e implementazione di un outer-loop PID
  selezionabile con anti-windup, mantenendo il PD come default.

Il risultato operativo piu importante e' che il confronto protesi/healthy ora e'
tracciabile: i plot hanno gait cycle coerenti, healthy overlay caricato e
diagnostiche piu esplicite sul controllo SEA. Il PID outer-loop e' stato
implementato e validato su AB06 nell'intervallo del setup `11.99 - 21.00 s`;
risulta stabile e migliora moderatamente il tracking cinematico, soprattutto alla
caviglia, ma e' volutamente conservativo.

Report utente considerati:

- `reports/user/2026-05-13_ab06_grf_gait_cycle_plotter.md`
- `reports/user/2026-05-13_ab06_healthy_overlay_threadmill.md`
- `reports/user/2026-05-13_pipeline_unica_abx_seasea.md`
- `reports/user/2026-05-13_analisi_tracking_pid_sea.md`
- `reports/user/2026-05-13_outer_pid_anti_windup_ab06.md`

Nota: il report `2026-05-13_ab06_grf_gait_cycle_plotter.md` era gia stato
richiamato anche nel daily `2026-05-12` per richiesta esplicita, ma viene
riconsolidato qui perche fa parte dell'insieme completo dei report datati
`2026-05-13`.

## 1. GRF, gait cycle e plotter AB06

E' stata diagnosticata l'origine delle anomalie nei plot gait-cycle del bundle
`AB06_SEASEA_Threadmill`.

Il problema non era:

- mapping piede-force plate;
- conversione `.mat -> .mot`;
- associazione `Treadmill_L/Treadmill_R`;
- errore del plotter nella lettura dei canali.

La causa era nei dati raw EPIC: il file force-plate contiene micro-contatti GRF
sopra la vecchia soglia storica di `20 N`. Il detector precedente interpretava
ogni attraversamento di soglia come evento valido, generando cicli non fisiologici
di pochi millisecondi.

Sono stati mantenuti i `20 N` come soglia di contatto, aggiungendo filtri di
robustezza:

```text
grf_contact_threshold_n     = 20.0
grf_min_contact_duration_s  = 0.05
grf_min_cycle_duration_s    = 0.30
```

Sono stati aggiornati:

- `config.py`;
- `output.py`.

Il CSV eventi ora include anche:

- `cycle_duration_s`;
- `contact_duration_s`;
- `min_contact_duration_s`;
- `min_cycle_duration_s`.

La rigenerazione degli eventi da:

```text
models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot
```

ha prodotto cicli coerenti:

```text
left cycles:  4
right cycles: 4
```

Durate dei cicli rilevati:

```text
left:  1.669, 1.548, 1.553, 1.571 s
right: 1.616, 1.603, 1.570, 1.573 s
```

Gli heel strike EPIC puliti nella finestra risultano coerenti:

```text
left:  13.965 15.635 17.190 18.745 20.305
right: 13.180 14.800 16.400 17.970 19.545
```

E' stato generato:

```text
plot/13_05_2026 - 1
```

In questa fase i plot gait-cycle sono stati corretti nella segmentazione, ma il
plotter segnalava ancora l'assenza di healthy overlay. Questo e' stato risolto
nel secondo filone della giornata.

Limite rimasto aperto: il detector ignora i micro-contatti per segmentare i gait
cycle, ma le micro-GRF restano comunque nel file di carico usato dalla dinamica.
Per rimuovere anche il rumore dinamico serve filtrare/azzerare il `_GRF_FullSpan`
prima della simulazione e rilanciare.

## 2. Healthy overlay AB06 per confronto protesi/sano

E' stata aggiunta una cartella healthy nel bundle operativo:

```text
models/AB06_SEASEA_Threadmill/data/healthy/
```

E' stato creato:

```text
tools/export_ab06_healthy_overlay.m
```

Lo script esporta dati healthy AB06 dal dataset EPIC verso file `.sto` leggibili
dal plotter:

```text
models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Kinematics_q.sto
models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Actuation_force.sto
```

Sorgenti EPIC usate:

```text
models/AB06-raw/10_09_18/treadmill/ik/treadmill_01_01.mat
models/AB06-raw/10_09_18/treadmill/id/treadmill_01_01.mat
```

Mapping cinematico principale:

```text
knee_angle_l  -> pros_knee_angle
ankle_angle_l -> pros_ankle_angle
```

Coordinate escluse dal confronto protesico:

```text
subtalar_angle_l
mtp_angle_l
```

Mapping dinamico principale:

```text
reserve_pros_knee_angle  <- knee_angle_l_moment
reserve_pros_ankle_angle <- ankle_angle_l_moment
```

Risultati MATLAB:

```text
rows:              28612
time span:         11.990 - 155.045 s
kinematics cols:   21
actuation cols:    21
```

Validazione Python con `read_sto()`:

```text
Kinematics:
  rows:       28612
  cols:       21
  time range: 11.990 - 155.045 s
  inDegrees:  yes
  contains:   pros_knee_angle, pros_ankle_angle

Actuation:
  rows:       28612
  cols:       21
  time range: 11.990 - 155.045 s
  inDegrees:  no
  contains:   reserve_pros_knee_angle, reserve_pros_ankle_angle
```

Compilazione statica:

```text
python -m py_compile plot/plotter.py output.py config.py
```

eseguita con esito positivo.

Il plotter e' stato poi rilanciato generando:

```text
plot/13_05_2026 - 3
```

Risultato:

- healthy overlay caricato;
- kinematic reference caricato;
- `No missing channels`;
- nessuna modifica a plugin C++, `.osim`, setup XML o simulazione.

## 3. Pipeline unica ABx_SEASEA

E' stato creato un entrypoint Python generale:

```text
tools/build_abx_seasea_pipeline.py
```

Obiettivo: generalizzare la pipeline costruita per AB06 ai soggetti EPIC locali
`ABx`, senza download automatici, senza dipendenze remote e senza modificare il
plugin SEA.

Funzioni principali dello script:

- discovery di `models/ABxx-raw`;
- selezione del modello healthy da `osimxml/*.osim`;
- selezione trial statico (`static_01`, poi fallback `static01`);
- selezione trial operativo;
- graft del modello healthy verso catena sinistra `SEASEA`;
- preservazione della massa;
- conversione EPIC MATLAB `.mat -> .trc/.mot/.xml`;
- calibrazione marker;
- costruzione bundle operativo `models/ABxx_SEASEA_Treadmill/`;
- export healthy overlay;
- diagnostica RRA opzionale;
- smoke test opzionale.

Comando operativo tipico:

```bash
python tools/build_abx_seasea_pipeline.py --subject AB07 --task treadmill --trial treadmill_01_01
```

Il graft sostituisce la catena sana sinistra:

```text
femur_l
tibia_l
talus_l
calcn_l
toes_l
```

con:

```text
transfemur
osseo_pylon
tibia_pylon
foot_l
```

e sostituisce le coordinate:

```text
knee_angle_l
ankle_angle_l
subtalar_angle_l
mtp_angle_l
```

con:

```text
pros_knee_angle
pros_ankle_angle
```

Vengono aggiunti:

```text
SEA_Knee
SEA_Ankle
```

La massa viene preservata: `transfemur` riceve la massa di `femur_l`, mentre la
massa residua della gamba sinistra viene distribuita su `osseo_pylon`,
`tibia_pylon` e `foot_l` secondo le proporzioni del donor; le inerzie vengono
scalate coerentemente.

File aggiornati o creati:

- `tools/build_abx_seasea_pipeline.py`;
- `tools/convert_epic_ab06_to_opensim.m`;
- `tools/export_ab06_healthy_overlay.m`.

Aggiornamenti MATLAB:

- `convert_epic_ab06_to_opensim.m` ora mappa le coordinate protesiche quando il
  `TargetModel` contiene `SEASEA`;
- `export_ab06_healthy_overlay.m` accetta il parametro `Subject`, cosi puo
  emettere prefissi `AB07`, `AB08`, ecc.

Opzioni principali della pipeline:

- `--subject`;
- `--all-subjects`;
- `--task`;
- `--trial`;
- `--static-trial`;
- `--run-rra`;
- `--skip-opensim-ik`;
- `--skip-smoke-test`;
- `--force`;
- `--validate-only`;
- `--python`.

Verifiche eseguite:

```text
python -m py_compile tools/build_abx_seasea_pipeline.py scripts/run_opensim_sea_pipeline.py tools/calibrate_ab06_seasea_markers.py
```

eseguita con esito positivo.

Discovery locale validata per:

```text
AB06
AB07
AB08
```

Static trial rilevati:

```text
AB06 -> static_01
AB07 -> static01
AB08 -> static01
```

Per tutti i soggetti sono stati rilevati:

- `treadmill/treadmill_01_01`;
- marker;
- force plate;
- IK;
- ID;
- plugin;
- support data.

Graft temporanei in `/private/tmp` validati per AB07 e AB08:

```text
AB07: 58.80765053 kg -> 58.80765053 kg
AB08: 72.41236865 kg -> 72.41236865 kg
```

I graft temporanei hanno confermato:

- rimozione dei vecchi riferimenti body/coordinate della catena sana sinistra;
- presenza dei body protesici;
- presenza delle coordinate protesiche;
- presenza degli attuatori SEA.

Sono state inoltre provate conversioni MATLAB AB07 per static e treadmill in
`/private/tmp`: il trial statico ha correttamente saltato le GRF, mentre il trial
treadmill ha generato gli output attesi.

## 4. Analisi dei plot 3, 4 e 6: tracking, coppie e confronto healthy

E' stata fatta un'analisi puntuale di:

```text
plot/13_05_2026 - 3
```

insieme a codice, setup e report precedenti.

Setup operativo analizzato:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
t_start:            11.99 s
t_end:              21.00 s
dt:                 0.001 s
T_control:          0.001 s
sea_forward_mode:   plugin
integration_scheme: rk4_bypass
```

Gains outer-loop PD correnti:

```text
pros_knee_angle:
  Kp = 160 N*m/rad
  Kd = 12  N*m*s/rad

pros_ankle_angle:
  Kp = 420 N*m/rad
  Kd = 1   N*m*s/rad
```

Parametri SEA dal modello:

```text
SEA_Knee:
  F_opt      = 100 Nm
  K          = 1000 Nm/rad
  Kp_inner   = 3.9
  Kd_inner   = 9.7
  Bm         = 0.1
  Jm         = 0.01
  Impedence  = false

SEA_Ankle:
  F_opt      = 250 Nm
  K          = 700 Nm/rad
  Kp_inner   = 8.8
  Kd_inner   = 9.7
  Bm         = 0.1
  Jm         = 0.01
  Impedence  = false
```

Conclusione sul plot 6: l'errore cinematico non nullo e' reale e coerente con
l'architettura. Il controller protesico precedente era un PD puro su posizione e
velocita, senza integratore e senza feedforward da inverse dynamics. In presenza
di gravita, GRF, mismatch inerziale, banda SEA-molla limitata e riferimento IK
healthy adattato alla protesi, un errore di regime e' atteso.

Metriche cinematiche PD baseline:

```text
knee:
  q RMS error:       0.054718 rad
  q mean error:      0.023963 rad
  q max abs error:   0.167141 rad
  qdot RMS error:    0.340822 rad/s

ankle:
  q RMS error:       0.094858 rad
  q mean error:      0.063706 rad
  q max abs error:   0.231940 rad
  qdot RMS error:    0.507272 rad/s
```

Metriche coppia PD baseline:

```text
knee:
  max |u|:                    0.306404
  max |tau_spring|:           29.484864 Nm
  RMS(tau_ref - tau_spring):  3.197243 Nm
  max |tau_ref - tau_spring|: 8.633281 Nm
  saturation count:           0

ankle:
  max |u|:                    0.389860
  max |tau_spring|:           97.755248 Nm
  RMS(tau_ref - tau_spring):  1.050540 Nm
  max |tau_ref - tau_spring|: 4.165224 Nm
  saturation count:           0
```

Interpretazione dei plot torque-angle:

- la curva protesica e' coppia `SEA_*_tau_spring` contro angolo protesico
  simulato;
- la curva healthy e' momento netto biologico da inverse dynamics AB06 contro
  angolo healthy;
- sono entrambe grandezze di coppia articolare, ma generate da fisiche diverse;
- non e' corretto aspettarsi automaticamente forme "canoniche" da letteratura,
  perche cambiano segni, normalizzazione, coordinate OpenSim, massa, dinamica
  muscolo-tendinea e convenzioni di plot.

La compressione delle curve protesiche non e' stata attribuita al gait-cycle
detector, che ora produce cicli coerenti. Le cause principali sono:

- ampiezza di coppia SEA piu bassa del momento healthy biologico;
- outer PD che non comanda momenti ID healthy, ma solo errore cinematico;
- tracking cinematico non perfetto;
- confronto tra un sistema muscolo-tendineo biologico e una SEA lineare;
- possibili micro-GRF ancora presenti nel carico dinamico.

Conclusione sul possibile PID: un integrale outer-loop e' giustificato per
ridurre offset e errore di regime, ma deve essere introdotto con:

- anti-windup;
- limite sull'integrale;
- leakage;
- reset temporale;
- diagnostica esplicita;
- sweep conservativo dei gain.

Il PID nel motor driver interno e' stato considerato una seconda fase, non la
prima modifica, perche la metrica corretta `tau_ref - tau_spring` era gia
ragionevolmente buona.

## 5. Outer PID selezionabile con anti-windup

E' stato implementato un nuovo outer-loop PID selezionabile, mantenendo il PD
esistente come default.

File modificati:

- `config.py`;
- `main.py`;
- `prosthesis_controller.py`;
- `output.py`;
- `plot/plotter.py`.

Configurazione aggiunta:

```python
sea_outer_controller_mode = "pd"
sea_ki = {
    "pros_knee_angle": 20.0,
    "pros_ankle_angle": 60.0,
}
sea_integral_limit = {
    "pros_knee_angle": 0.25,
    "pros_ankle_angle": 0.25,
}
sea_integral_leak_s_inv = 0.1
```

CLI aggiunta:

```text
--sea-outer-controller {pd,pid}
--sea-ki-knee
--sea-ki-ankle
--sea-integral-limit-knee
--sea-integral-limit-ankle
--sea-integral-leak
```

Logica controller:

```text
pd:
  tau_cmd = Kp * e_q + Kd * e_qdot

pid:
  tau_cmd = Kp * e_q + Kd * e_qdot + Ki * integral(e_q)
```

Dettagli implementativi:

- stato interno `integral_error[coord]`;
- `last_time` per calcolare `dt_control`;
- `dt_control` da `state.getTime()`, con fallback a `cfg.T_control`;
- reset se il tempo torna indietro;
- leakage prima dell'integrazione;
- clamp dell'integrale a `sea_integral_limit`;
- anti-windup condizionale quando il comando saturerebbe e l'errore spingerebbe
  ulteriormente nella stessa direzione;
- `sea_feasibility_scale` applicato al comando outer completo;
- semantica `u = tau_ref / F_opt` invariata;
- plugin C++ SEA non modificato.

Diagnostiche aggiunte in output:

- `outer_p_cmd`;
- `outer_d_cmd`;
- `outer_i_cmd`;
- `outer_integral_error`;
- `outer_integral_clamped`;
- `outer_anti_windup_active`;
- `outer_controller_mode_id`.

Sono state mantenute le colonne esistenti per compatibilita.

E' stata anche corretta la diagnostica/mirror di `tau_input` per il plugin in
modalita `ff`, usando:

```text
tau_input = tau_ref + Kp_inner * (tau_ref - tau_spring) - Kd_inner * omega_m
```

Questa modifica e' diagnostica: non cambia la semantica del comando passato al
plugin.

Verifiche statiche:

```text
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py prosthesis_controller.py output.py main.py plot/plotter.py validation/validate_sim_results.py
```

eseguita con esito positivo.

Smoke test eseguiti:

```text
results/_outer_pid_smoke_pd
results/_outer_pid_smoke_pid
results/_outer_pid_smoke_pid_ffdiag
```

Run completo PID richiesto:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml \
  --sea-outer-controller pid \
  --output-dir results/_outer_pid_ab06_full \
  --validate \
  --plot
```

Il run ha usato l'intervallo del setup:

```text
11.99 - 21.00 s
```

Stato run:

```text
status:       complete
t final:      21.00 s
step:         9010
wall_time_s:  744.7091625
```

Plot generati:

```text
plot/14_05_2026 - 2
```

con:

```text
No missing channels.
```

Metriche PD vs PID:

```text
knee:
  q RMS PD:                  0.054718 rad  (3.135085 deg)
  q RMS PID:                 0.052535 rad  (3.010021 deg)
  improvement:              -3.99 %
  max |u| PID:               0.307064
  RMS(tau_ref - tau_spring): 3.205758 Nm
  saturation count:          0
  max |I|:                   2.128318 Nm
  anti-windup samples:       0

ankle:
  q RMS PD:                  0.094858 rad  (5.434958 deg)
  q RMS PID:                 0.084598 rad  (4.847111 deg)
  improvement:              -10.82 %
  max |u| PID:               0.399096
  RMS(tau_ref - tau_spring): 1.051801 Nm
  saturation count:          0
  max |I|:                   14.902613 Nm
  anti-windup samples:       0
```

Differenza diretta PID - PD sugli angoli:

```text
knee:
  RMS difference: 0.465 deg
  max difference: 0.734 deg

ankle:
  RMS difference: 1.285 deg
  max difference: 2.018 deg
```

Validazione full PID:

```text
PASS: 40
WARN: 5
FAIL: 1
```

Il FAIL residuo riguarda:

```text
mtp_angle_r output vs IK
RMS: 14.11482 deg
max: 29.79542 deg
```

Lo stesso problema era gia presente nel baseline PD:

```text
RMS: 14.11410 deg
max: 29.79734 deg
```

Quindi non e' stato introdotto dal PID.

I controlli SEA sono passati:

- accordo diagnostico plugin/Python su `tau_input`;
- saturazione `tau_input` assente;
- saturazione del comando `u` assente.

Conclusione: il PID e' implementato, selezionabile e stabile su AB06. L'effetto
visivo sui plot e' contenuto perche i gain integrali sono conservativi, l'errore
iniziale non e' enorme e il termine integrale resta limitato. La caviglia migliora
piu del ginocchio.

## File principali creati o modificati

Creati:

- `tools/build_abx_seasea_pipeline.py`;
- `tools/export_ab06_healthy_overlay.m`;
- `models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Kinematics_q.sto`;
- `models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Actuation_force.sto`;
- `reports/user/2026-05-13_ab06_grf_gait_cycle_plotter.md`;
- `reports/user/2026-05-13_ab06_healthy_overlay_threadmill.md`;
- `reports/user/2026-05-13_pipeline_unica_abx_seasea.md`;
- `reports/user/2026-05-13_analisi_tracking_pid_sea.md`;
- `reports/user/2026-05-13_outer_pid_anti_windup_ab06.md`;
- `reports/daily/2026-05-13_daily-report.md`.

Modificati:

- `config.py`;
- `main.py`;
- `prosthesis_controller.py`;
- `output.py`;
- `plot/plotter.py`;
- `tools/convert_epic_ab06_to_opensim.m`;
- `tools/export_ab06_healthy_overlay.m`.

Output generati o rigenerati:

- `results/sim_output_gait_events.csv`;
- `plot/13_05_2026 - 1`;
- `plot/13_05_2026 - 3`;
- `results/_outer_pid_smoke_pd`;
- `results/_outer_pid_smoke_pid`;
- `results/_outer_pid_smoke_pid_ffdiag`;
- `results/_outer_pid_ab06_full`;
- `plot/14_05_2026 - 2`.

## Verifiche principali

Compilazioni statiche:

```text
python -m py_compile plot/plotter.py output.py config.py
python -m py_compile tools/build_abx_seasea_pipeline.py scripts/run_opensim_sea_pipeline.py tools/calibrate_ab06_seasea_markers.py
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py prosthesis_controller.py output.py main.py plot/plotter.py validation/validate_sim_results.py
```

Validazioni dati:

- gait-cycle detector: 4 cicli sinistri e 4 destri fisiologici;
- healthy overlay: `.sto` con 28612 righe, range `11.990 - 155.045 s`;
- plotter: healthy overlay caricato e `No missing channels`;
- pipeline ABx: discovery locale AB06/AB07/AB08;
- graft AB07/AB08: massa preservata esattamente;
- conversioni MATLAB AB07 static/treadmill provate in `/private/tmp`;
- PID AB06 full run: `status=complete`, nessuna saturazione `u`, nessuna
  saturazione `tau_input`.

## Stato finale

Alla chiusura del 13/5:

- `AB06_SEASEA_Threadmill` ha plot gait-cycle affidabili;
- il confronto healthy e' disponibile e caricato dal plotter;
- le differenze prosthetic vs healthy sono state ricondotte a cause note:
  fisica SEA, tuning, riferimento healthy IK, mismatch dinamico e confronto
  biologico/protesico;
- il vecchio PD resta default;
- il nuovo PID outer-loop e' attivabile via config o CLI;
- il PID e' stato testato su AB06 e migliora il tracking RMS di ginocchio e
  caviglia senza saturare;
- la pipeline e' stata resa scalabile verso altri soggetti EPIC locali `ABx`.

## Limiti aperti e prossimi passi

1. Filtrare o azzerare le micro-GRF sub-threshold nel file di carico dinamico,
   non solo nel detector gait-cycle, e rilanciare la simulazione.

2. Eseguire uno sweep controllato dei gain PID:

```text
Ki_knee:  20 -> 30 -> 40
Ki_ankle: 60 -> 90 -> 120
```

   mantenendo monitorati saturazioni, `tau_ref - tau_spring`, integral clamp e
   RMS cinematico.

3. Valutare in una fase successiva un feedforward da inverse dynamics per
   l'outer-loop, perche un PID puro puo ridurre offset ma non risolve del tutto
   il mismatch dinamico con GRF e massa.

4. Non modificare il motor driver interno SEA prima di avere esaurito tuning
   outer-loop e feedforward: il tracking `tau_ref - tau_spring` attuale e' gia
   molto migliore del tracking cinematico.

5. Chiarire nei plot/report che:

- `prosthetic` = coppia SEA spring sul giunto protesico simulato;
- `healthy` = momento netto biologico da inverse dynamics AB06;
- il confronto e' utile, ma non e' un confronto tra due attuatori equivalenti.

6. Per la pipeline `ABx_SEASEA`, completare un run end-to-end reale su AB07 o
   AB08 fuori da `/private/tmp`, includendo IK/OpenSim, healthy overlay, smoke
   test e, se richiesto, RRA diagnostics.
