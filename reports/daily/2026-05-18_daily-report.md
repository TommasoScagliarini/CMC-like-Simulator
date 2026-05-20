# 2026-05-18 Daily Report

## Sintesi

La giornata del 18/05/2026 e' stata centrata sul tuning cascade dei due SEA
protesici AB06, con tre blocchi principali:

1. recupero e completamento dello sweep ankle del mattino;
2. analisi e tentativo di redesign del motor driver/cascade per ridurre il
   chattering;
3. retune sul modello `pi-tuned` e successivo ripristino finale della
   configurazione best del mattino.

Stato finale a fine giornata:

- outer loop cascade ripristinato al best candidate del mattino;
- motor driver PI ripristinato ai gain del modello PI originale;
- `AB06_SEASEA_stiff321_500_pi-tuned.osim` e'
  byte-identico a `AB06_SEASEA_stiff321_500_pi.osim`;
- `.simulator_last_setup.json` punta ancora a
  `AB06_SEASEA_stiff321_500_pi-tuned_setup.xml`, ma quel setup ora carica un
  modello con motor driver ripristinato alla configurazione PI del mattino.

Configurazione finale attiva:

```text
outer cascade
knee:  Kp_outer=18.85,  Kp_inner=29.2,    Ki_inner=1377.0, I_limit=50
ankle: Kp_outer=47.125, Kp_inner=2.8275, Ki_inner=213.0,  I_limit=200

motor driver PI
SEA_Knee:  Kp=18,   Kd=11, Ki=190, integral_torque_limit=100
SEA_Ankle: Kp=11.3, Kd=11, Ki=123, integral_torque_limit=100
```

## Report utente creati oggi

- `reports/user/2026-05-18_sweep_cascade_ankle_best_config.md`
- `reports/user/2026-05-18_motor_driver_cascade_redesign_zeta07.md`
- `reports/user/2026-05-18_run_zeta07_analisi_fallimento_tracking_ankle.md`
- `reports/user/2026-05-18_retune_cascade_pi_tuned.md`

## 1. Recupero sweep e full run dei 5 candidati ankle

Richiesta iniziale:

- analizzare lo sweep terminato;
- capire se lo stage 1 era stato salvato;
- proseguire senza rifare tutto da capo, se possibile.

Sweep analizzato:

```text
results/_cascade_local_gain_sweep_20260517_234151
```

Esito stage 1:

```text
stage1_knee_screen.csv  = 125 candidati, 0 accettabili
stage1_ankle_screen.csv = 125 candidati, 5 accettabili
```

Il knee non aveva candidati accettabili per i gate:

```text
tau_input_saturated
tau_input_raw_gt_500
```

L'ankle aveva invece 5 candidati accettabili, quindi sono state lanciate 5 full
run mantenendo il knee alla configurazione corrente e variando solo ankle.

Output:

```text
results/_cascade_full_ankle5_20260518_105345
```

Best candidate:

```text
full_ankle5_kpo18p85_kpi29p2_kii1377_kil50_apo47p125_api2p8275_aii213_ail200
```

Metriche:

```text
score_kinematic_deg = 0.907516
knee RMS            = 0.177624 deg
ankle RMS           = 1.220328 deg
score_chattering    = 1.425529
score_power         = 0.758111
sat_count           = 0
max tau_input_raw   = 124.823 Nm
knee max |u|        = 0.331625
ankle max |u|       = 0.481839
```

Plot:

```text
plot/05_18_2026_1
```

Verifica plot:

```text
No missing channels.
```

## 2. Iterazione originale PI per ridurre chattering

Dopo il best ankle del mattino e' stata fatta una campagna aggiuntiva sul
modello PI originale per cercare di ridurre chattering e power senza perdere
troppo tracking.

Output:

```text
results/_cascade_iter_original_pi_20260518_184556
plot/05_18_2026_3
```

Screening:

- 12 configurazioni;
- setup PI originale:
  `AB06_SEASEA_stiff321_500_pi_setup.xml`;
- modello:
  `AB06_SEASEA_stiff321_500_pi.osim`;
- 4 candidati mandati in full run.

Miglior compromesso trovato in quella fase:

```text
full_k15_i700_base_ankle

knee:
  Kp_outer = 15.0
  Kp_inner = 18.0
  Ki_inner = 700.0
  I_limit  = 50.0

ankle:
  Kp_outer = 47.125
  Kp_inner = 2.8275
  Ki_inner = 213.0
  I_limit  = 200.0
```

Metriche full:

```text
score_kinematic_deg = 0.942591
knee RMS            = 0.322486 deg
ankle RMS           = 1.208350 deg
score_chattering    = 0.808341
score_power         = 0.784452
sat_count           = 0
max tau_input_raw   = 123.492 Nm
```

Interpretazione:

- ankle tracking leggermente migliore del best mattutino;
- knee tracking peggiore ma ancora basso;
- chattering ridotto di circa 19%;
- power ridotta di circa 22%.

Questo candidato e' stato poi superato dalla richiesta successiva di testare
specificamente il modello `pi-tuned`, e infine dal ripristino finale del best
mattutino.

## 3. Redesign motor driver e cascade con zeta=0.7

Report sorgente:

```text
reports/user/2026-05-18_motor_driver_cascade_redesign_zeta07.md
```

Obiettivo:

- ridurre il chattering strutturale;
- caratterizzare i poli del motor driver PI corrente;
- progettare un nuovo motor driver con `zeta=0.7`;
- definire una gerarchia motor/velocity/position a separazione 5x.

Analisi plant:

```text
SEA_Knee:  Jm=0.01, Bm=0.10, Ks=321, F_opt=100
SEA_Ankle: Jm=0.01, Bm=0.10, Ks=500, F_opt=250
```

Driver PI corrente:

```text
knee:  poli complessi ~774 rad/s, zeta=0.71, polo reale -10.19
ankle: poli complessi ~777 rad/s, zeta=0.71, polo reale -10.19
```

Nuovo design zeta07:

```text
SEA_Knee:  Kp=4.17, Kd=5.66, Ki=290.7
SEA_Ankle: Kp=1.01, Kd=4.38, Ki=87.81
```

Cascade proposta:

```text
knee:
  Kp_outer = 14.4
  Kp_inner = 31.85
  Ki_inner = 1638.0
  I_limit  = 50.0

ankle:
  Kp_outer = 11.2
  Kp_inner = 1.51
  Ki_inner = 60.5
  I_limit  = 80.0
```

File creati:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07_setup.xml
tools/motor_driver_pole_map.py
tools/motor_driver_pole_locus.py
plot/05_18_2026_motor_driver_poles/motor_driver_poles.png
plot/05_18_2026_motor_driver_locus/motor_driver_pole_locus.png
```

Verifiche analitiche:

- coefficienti del polinomio caratteristico target verificati per knee e ankle;
- `python -m py_compile config.py`: PASS;
- grep dei tag `<Kp>`, `<Kd>`, `<Ki>` nel nuovo `.osim`: coerente.

## 4. Analisi full run zeta07 e diagnosi fallimento ankle

Report sorgente:

```text
reports/user/2026-05-18_run_zeta07_analisi_fallimento_tracking_ankle.md
```

Run zeta07:

```text
setup: AB06_SEASEA_stiff321_500_pi_zeta07_setup.xml
model: AB06_SEASEA_stiff321_500_pi_zeta07.osim
```

Esito:

```text
status     = complete
wall time  = 794 s
steps      = 9010
sat tau    = 0
sat u      = 0
```

Problema:

```text
ankle RMS zeta07    = 27.49 deg
ankle RMS baseline  = 1.220 deg
```

Il tracking ankle e' risultato catastrofico, nonostante l'assenza di
saturazioni hard.

Diagnosi:

- position-loop ankle troppo lenta (`Kp_outer=11.2 1/s`);
- velocity-loop ankle troppo lenta (`omega_v=56 rad/s`);
- integratore cascade ankle al clamp per il 14.7% del tempo;
- assumption sul `J_joint` ankle probabilmente errata o non valida nella forma
  usata per la regola 5x.

Aspetto positivo:

- HPF50 `tau_input` ankle ridotto del 63%;
- HPF50 `motor_speed_dot` ankle ridotto del 65%.

Verdetto:

- zeta07 e' un fallimento operativo sul tracking ankle;
- la teoria 5x/zeta=0.7 applicata uniformemente e' troppo conservativa per il
  push-off ankle AB06;
- eventuale design futuro deve dimensionare ankle sulla banda spettrale reale
  della cinematica, non solo su regole di separazione astratte.

Plot creati:

```text
plot/05_18_2026_zeta07_run_analysis/tracking_comparison.png
plot/05_18_2026_zeta07_run_analysis/ankle_internal_signals.png
```

## 5. Retune sul modello pi-tuned

Report sorgente:

```text
reports/user/2026-05-18_retune_cascade_pi_tuned.md
```

Richiesta:

- rifare i test usando:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned_setup.xml
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim
```

Differenza del modello `pi-tuned` prima del ripristino finale:

```text
SEA_Knee tuned:  Kp=2.13, Kd=4.38, Ki=136
SEA_Ankle tuned: Kp=1.01, Kd=4.38, Ki=87.8
```

Output principale:

```text
results/_cascade_iter_pi_tuned_20260518_192606
plot/05_18_2026_4
```

Campagna:

- 3 iterazioni di screening;
- 2 batch di full run;
- 7 full run totali;
- 7/7 complete, finite, senza saturazioni.

Candidato scelto durante quella fase:

```text
full_s3_mid_a80_v20_i140

knee:
  Kp_outer = 14.0
  Kp_inner = 15.0
  Ki_inner = 500.0
  I_limit  = 50.0

ankle:
  Kp_outer = 80.0
  Kp_inner = 2.0
  Ki_inner = 140.0
  I_limit  = 200.0
```

Metriche:

```text
score cinematico          = 0.9169 deg
knee RMS                  = 0.4596 deg
ankle RMS                 = 1.1128 deg
motor/tau HPF chatter     = 0.516x rispetto baseline mattina
score power               = 0.721x
sat_count                 = 0
max tau_input_raw         = 118.1 Nm
```

Alternativa piu' quieta:

```text
full_s3_kfast_a75_v18_i130
motor/tau HPF chatter = 0.301x
score cinematico      = 0.963 deg
ankle RMS             = 1.278 deg
```

Interpretazione:

- sul modello `pi-tuned` la cascade deve essere piu' position-driven e meno
  velocity/torque-stiff;
- `mid_a80` mantiene tracking vicino alla baseline mattutina e dimezza circa
  il chatter richiesto;
- il risultato e' stato poi superseduto dalla richiesta finale di ripristinare
  la configurazione best del mattino sia per outer loop sia per motor driver.

## 6. Ripristino finale richiesto a fine giornata

Richiesta:

```text
ripristina la configurazione best di stamattina sia per outer loop sia per motordriver
```

Azioni eseguite:

1. `config.py` riportato ai gain cascade del best mattutino.
2. `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim`
   riportato ai gain PI del modello originale.

Stato finale `config.py`:

```text
knee:
  Kp_outer = 18.85
  Kp_inner = 29.2
  Ki_inner = 1377.0
  I_limit  = 50.0

ankle:
  Kp_outer = 47.125
  Kp_inner = 2.8275
  Ki_inner = 213.0
  I_limit  = 200.0
```

Stato finale `pi-tuned.osim`:

```text
SEA_Knee:  Kp=18,   Kd=11, Ki=190
SEA_Ankle: Kp=11.3, Kd=11, Ki=123
```

Verifiche:

```text
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py
cmp -s AB06_SEASEA_stiff321_500_pi-tuned.osim AB06_SEASEA_stiff321_500_pi.osim
```

Esito:

- `config.py`: sintassi OK;
- `pi-tuned.osim` e `pi.osim`: byte-identici dopo il ripristino;
- `.simulator_last_setup.json` punta ancora al setup `pi-tuned`, ma il modello
  caricato da quel setup ha ora il motor driver del best mattutino.

## Artefatti principali della giornata

Risultati:

```text
results/_cascade_full_ankle5_20260518_105345
results/_cascade_iter_original_pi_20260518_184556
results/_cascade_iter_retune_20260518_183502
results/_cascade_iter_pi_tuned_20260518_192606
```

Plot:

```text
plot/05_18_2026_1
plot/05_18_2026_2
plot/05_18_2026_3
plot/05_18_2026_4
plot/05_18_2026_motor_driver_locus
plot/05_18_2026_zeta07_run_analysis
```

Modelli/setup creati o modificati:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07_setup.xml
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim
config.py
```

Tool creati:

```text
tools/motor_driver_pole_map.py
tools/motor_driver_pole_locus.py
```

## Stato finale operativo

Configurazione attiva:

- controller outer: `cascade`;
- outer loop: best candidate del mattino;
- motor driver: PI originale `18/11/190` knee e `11.3/11/123` ankle;
- setup ricordato dal simulatore: `AB06_SEASEA_stiff321_500_pi-tuned_setup.xml`;
- modello caricato dal setup ricordato: `AB06_SEASEA_stiff321_500_pi-tuned.osim`,
  ora riallineato al PI originale.

Da ricordare:

- i risultati `pi-tuned` con chatter ridotto restano disponibili, ma non sono
  piu' il default attivo dopo il ripristino;
- il redesign zeta07 resta un esperimento diagnostico, non una configurazione
  operativa consigliata.

## TODO aperti e propagati

Propagati dai report user di oggi e dal daily 2026-05-17.

- **Windows pendente**: compilare/copiare la DLL plugin PI sulla macchina
  Windows, verificare il caricamento del plugin, eseguire smoke load/run e
  controllare che il modello PI legga `Ki`, `integral_torque_limit` e
  `torque_error_integral`.

- **Secondo pass knee dello sweep locale**: lo stage 1 knee del 17/18 maggio
  aveva 125 run complete/finite ma 0 accettabili per gate rigidi su
  `tau_input_raw` e saturazioni. Valutare se:
  - rilassare i gate solo per analisi;
  - cambiare griglia knee;
  - usare una baseline omogenea con `--filter-grf`.

- **Confronto consolidato finale**: costruire un confronto unico tra PD, PI,
  cascade aggressive, best mattutino, retune PI originale, zeta07 e retune
  `pi-tuned`, usando tracking, HPF chatter, `tau_input`, motor power e reserve.

- **Zeta07 non operativo**: se si vuole riprendere quella linea, ritunare ankle
  con banda reale del push-off:
  - `Kp_outer_ankle` circa 30-40 1/s;
  - `omega_v_ankle` circa 90-100 rad/s;
  - `Kp_inner_ankle` circa 2.5;
  - `Ki_inner_ankle` circa 175;
  - separazione motor/velocity almeno 3x.

- **Valutare omega_motor ankle zeta07**: possibile rialzo da 280 a circa
  320 rad/s, vicino al cap di saturazione teorico, se si vuole recuperare
  separazione 5x senza distruggere tracking.

- **Validare J_joint usato nel back-calc cascade**: eseguire run dedicati o
  smoke con step in `qdot_ref` per misurare l'inerzia effettiva vista dal loop
  `tau -> qdot`, soprattutto per ankle.

- **Knee nel design zeta07**: il tracking era OK ma `motor_speed` HPF e
  `tau_error` peggioravano. Valutare `Kd_plugin` piu' alto o ritorno a
  `omega_motor_knee` piu' vicino a 500 rad/s.

- **Regola 5x da validare**: testare una gerarchia 3x come compromesso, perche'
  la 5x uniforme e' risultata troppo conservativa sull'ankle.

- **Zero/polo PI ankle**: il miss del 22% nel design zeta07 non ha dominato il
  fallimento perche' l'integratore cascade era gia' al clamp, ma resta da
  valutare in un design ankle non saturato.

- **Alternativa piu' quieta su pi-tuned**: se in futuro si torna a preferire
  riduzione massima di chattering rispetto al tracking, riesaminare
  `full_s3_kfast_a75_v18_i130`.

- **Pulizia artefatti**: valutare cancellazione o archiviazione delle cartelle
  di sweep parziali/supersedute:
  - `results/_cascade_local_gain_sweep_20260517_233607`
  - `results/_cascade_local_gain_sweep_20260517_234151`
  - eventuali plot con header sbagliato gia' spostati in cartelle
    `*_wrong_header_backup`.

## TODO chiusi o superseduti oggi

- **Verificare se lo stage 1 dello sweep era recuperabile**: chiuso. I file
  stage 1 esistevano e sono stati usati per proseguire con i 5 candidati ankle.
- **Lanciare full run dei 5 candidati ankle**: chiuso. Tutte complete e
  accettabili.
- **Smoke/full zeta07 iniziale**: chiuso con esito negativo operativo. Il run
  era numericamente completo ma il tracking ankle e' fallito.
- **Aggiornare default verso zeta07**: superseduto. Dopo la diagnosi del
  fallimento zeta07 e il ripristino finale, non va promosso a default.
- **Retune `pi-tuned` come default**: superseduto dal ripristino finale del
  best mattutino.
