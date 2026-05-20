# Daily report - 2026-05-12

## Sintesi

Questo report chiude il lavoro svolto su `AB06_SEASEA` nei giorni `2026-05-11`
e `2026-05-12`.

L'obiettivo era portare il soggetto sano AB06 del dataset EPIC Lab Lower Limb
Biomechanics Dataset dentro una pipeline CMC-like con protesi SEA sinistra a
ginocchio e caviglia, dati sperimentali convertiti, modello utilizzabile dal
simulatore e plot diagnostici coerenti.

Sono stati consolidati:

- modello AB06 con protesi SEA sinistra;
- conversione dati EPIC `.mat -> .trc/.mot/.xml`;
- calibrazione marker protesici;
- correzione GRF/free torque;
- redistribuzione mass-preserving della gamba sinistra;
- pipeline OpenSim IK/ID/RRA/CMC-like;
- bundle standard `AB06_SEASEA` e bundle operativo `AB06_SEASEA_Threadmill`;
- plotter collegato al setup realmente usato dalla simulazione;
- detector gait-cycle piu robusto contro micro-contatti GRF.

Report utente correlati:

- `reports/user/2026-05-11_ab06_seasea_epic_pipeline.md`
- `reports/user/2026-05-13_ab06_grf_gait_cycle_plotter.md`

Nota: il secondo report e stato incluso qui per richiesta esplicita, anche se il
file porta data `2026-05-13`; il contenuto viene consolidato in questo end-day
datato `2026-05-12`.

## 2026-05-11 - Modello AB06_SEASEA e pipeline EPIC

### Modello protesico

E' stato creato un modello AB06 con protesi SEA sul lato sinistro, partendo dal
modello sano AB06 e dal template SEASEA tuned.

La catena sana sinistra:

```text
femur_l
tibia_l
talus_l
calcn_l
toes_l
```

e' stata sostituita con:

```text
transfemur
osseo_pylon
tibia_pylon
foot_l
```

Le coordinate sane distali:

```text
knee_angle_l
ankle_angle_l
subtalar_angle_l
mtp_angle_l
```

sono state sostituite con:

```text
pros_knee_angle
pros_ankle_angle
```

Sono stati aggiunti gli attuatori:

```text
SEA_Knee  -> pros_knee_angle
SEA_Ankle -> pros_ankle_angle
```

I marker AB06 sinistri sono stati riagganciati ai body protesici:

- marker coscia sinistra su `transfemur`;
- marker ginocchio/segmento prossimale su `osseo_pylon`;
- marker shank su `tibia_pylon`;
- marker ankle/heel/toe su `foot_l`.

### Conversione dati EPIC

E' stato creato lo script MATLAB:

```text
tools/convert_epic_ab06_to_opensim.m
```

Motivo: i `.mat` AB06 sono table/MCOS MATLAB e non risultavano leggibili in modo
robusto con SciPy.

Lo script converte i trial EPIC in:

- `.trc` per marker;
- `_grf.mot` per GRF;
- `_ExternalLoads.xml`;
- `_ik_dataset_ab06_seasea.mot`;
- setup IK/ID specifici per trial.

Trial convertiti:

```text
treadmill/treadmill_01_01
static/static_01
```

Risultati principali del trial treadmill:

```text
TRC:     28612 frame, 28 marker, 200 Hz
GRF MOT: 143056 righe, 19 colonne, 1000 Hz
range:   11.990 - 155.045 s
```

Il trial static ha generato marker/IK dataset ma ha correttamente saltato le GRF
con warning, perche non dispone di `fp`.

### Calibrazione marker

E' stato creato:

```text
tools/calibrate_ab06_seasea_markers.py
```

La calibrazione usa `static_01.trc` e il dataset IK per riallineare i marker
sinistri protesici.

Risultati:

```text
errore marker locale medio prima: ~0.100755 m
errore marker locale medio dopo:  ~0.000616 m
```

Sul treadmill:

```text
IK full window prima calibrazione: RMS 0.089647 m, max 0.397880 m
IK full window dopo calibrazione:  RMS 0.009161 m, max 0.107720 m
IK RRA window dopo calibrazione:   RMS 0.007392 m, max 0.027178 m
```

### Force plate, COP, segni e frame

Mapping confermato:

```text
Treadmill_L -> ground_force1 -> foot_l
Treadmill_R -> ground_force2 -> calcn_r
```

Con soglia `>100 N`, il COP e' coerente col piede corretto:

```text
plate1-COP vicino al piede sinistro: 0.0876 m
plate1-COP vicino al piede destro:   0.4291 m
plate2-COP vicino al piede destro:   0.0917 m
plate2-COP vicino al piede sinistro: 0.4212 m
```

Le GRF verticali sono coerenti col peso:

```text
massa AB06 sano: 78.09885033 kg
BW:              ~765.89 N
GRF totale media ~757.86 N
```

La sincronizzazione marker/GRF e' risultata corretta:

```text
TRC start/end: 11.990 - 155.045 s
GRF start/end: 11.990 - 155.045 s
delta start:   0.000000000 s
delta end:     0.000000000 s
```

### Correzione momenti GRF

I canali treadmill `moment_*` si comportano come momenti rispetto all'origine
laboratorio. Poiche OpenSim applica gia il contributo `r x F` quando riceve COP
e forza, il converter e' stato corretto per scrivere solo il free torque
verticale al COP.

Effetto della correzione:

```text
moment_norm RRA prima: ~695.97 Nm
moment_norm dopo:      ~85.31 Nm
```

### Massa preservata

Il modello protesico iniziale pesava circa `69.76 kg`, mentre AB06 sano pesa
`78.10 kg`. Questo generava un residuo verticale RRA di circa `-76 N`.

La massa della gamba sinistra sana rimossa era:

```text
femur_l   9.6645049193 kg
tibia_l   3.8522321359 kg
talus_l   0.1039037663 kg
calcn_l   1.2987970788 kg
toes_l    0.2250555578 kg
totale   15.1444934582 kg
```

Redistribuzione sui body protesici:

```text
transfemur    9.6645049193 kg
osseo_pylon   1.9866857556 kg
tibia_pylon   1.8655463803 kg
foot_l        1.6277564030 kg
totale       15.1444934582 kg
```

Massa totale preservata:

```text
AB06 sano    78.09885033 kg
AB06_SEASEA  78.09885033 kg
```

### Pipeline mass-preserved

Pipeline finale del giorno:

```text
results/ab06_seasea_mass_preserved_pipeline_treadmill_01_01/
```

Report:

```text
results/ab06_seasea_mass_preserved_pipeline_treadmill_01_01/pipeline_report.md
```

RRA finale:

```text
FX = -2.39608 N
FY =  6.60408 N
FZ = -0.573838 N
MX = -61.7 Nm
MY = -10.7815 Nm
MZ =  28.1085 Nm

Force norm  = 7.0487 N
Moment norm = 68.6529 Nm
```

Smoke test CMC-like:

```text
return code: 0
status: complete
intervallo: 18.965 - 18.995 s
SEA forward mode: plugin
```

## 2026-05-12 - Bundle, full-span e simulazione AB06

### Riorganizzazione bundle

Le cartelle sorgenti/importate sono state separate dai bundle simulabili:

```text
models/AB06           -> models/AB06-raw
models/AB07           -> models/AB07-raw
models/AB08           -> models/AB08-raw
models/AB06_SEASEA    -> models/AB06_SEASEA-raw
```

E' stato creato un bundle pulito nello stile `models/SEASEA`, con modello alla
radice, setup alla radice e dati in `data/`.

E' stato poi consolidato il bundle operativo treadmill:

```text
models/AB06_SEASEA_Threadmill/
```

File principali:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot
models/AB06_SEASEA_Threadmill/data/ExternalForces.xml
models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml
models/AB06_SEASEA_Threadmill/data/CMC_Tasks - modified Kp_Kv.xml
```

Il modello `AB06_SEASEA.osim` nel bundle Threadmill corrisponde al modello
marker-calibrato e mass-preserving.

### Full-span EPIC

E' stato confermato che il vero intervallo del trial `treadmill_01_01` e':

```text
11.990 - 155.045 s
```

Sono stati rigenerati i dati full-span:

```text
treadmill_01_01.trc
treadmill_01_01_grf.mot
treadmill_01_01_ExternalLoads.xml
treadmill_01_01_ik_dataset_ab06_seasea.mot
```

La pipeline full-span ha prodotto:

```text
results/ab06_seasea_fullspan_pipeline_treadmill_01_01/
```

Report:

```text
results/ab06_seasea_fullspan_pipeline_treadmill_01_01/pipeline_report.md
```

RRA full-span:

```text
Selected window: 11.990000 - 155.045000 s
Selected candidate: repo_cmc_tasks_torso
Residuals:
FX = -11.3278 N
FY =   8.01496 N
FZ =  -0.334532 N
MX = -54.539 Nm
MY =  -1.64518 Nm
MZ = 394.064 Nm

Force norm  = 13.8806 N
Moment norm = 397.8236 Nm
```

Il forte aumento di `MZ` ha portato alla decisione di non usare direttamente il
modello RRA full-span come modello operativo per il CMC-like, ma di usare il
modello AB06_SEASEA marker-calibrato/mass-preserving e la cinematica IK
OpenSim come riferimento.

### Scelta operativa senza RRA per la simulazione corrente

E' stato chiarito che, dato che i dati cinematici sperimentali sono puliti, si
puo saltare RRA per la run CMC-like corrente e usare:

- modello `AB06_SEASEA.osim` marker-calibrato e mass-preserving;
- cinematica IK OpenSim prodotta dalla pipeline;
- GRF full-span convertite;
- setup CMC-like dedicato.

Bundle operativo:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
```

Contenuto del setup:

```text
model_file:          models/AB06_SEASEA_Threadmill/AB06_SEASEA.osim
kinematics_file:     models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
external_loads_xml:  models/AB06_SEASEA_Threadmill/data/ExternalForces.xml
reserve_actuators:   models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml
t_start:             11.99
t_end:               21
```

Il file `.simulator_last_setup.json` e' stato aggiornato per puntare al setup
Threadmill.

### Plotter e riferimento cinematico reale

Il plotter inizialmente usava ancora la cinematica default di `models/SEASEA`:

```text
models/SEASEA/data/3DGaitModel2392_Kinematics_q.sto
```

Questo causava errore perche quel file finisce a `11.069 s`, mentre la
simulazione AB06 corrente usa `11.990 - 20.999 s`.

`plot/plotter.py` e' stato aggiornato per:

- leggere automaticamente `.simulator_last_setup.json`;
- caricare il setup usato dal simulatore;
- usare come riferimento cinematico il file realmente usato nella run:
  `models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`;
- aggiungere opzioni CLI `--setup` e `--no-last-setup`;
- stampare il range temporale dei risultati;
- stampare il numero di eventi gait caricati;
- risolvere meglio path bundle-local;
- usare cache Matplotlib scrivibile su macOS.

Test:

```text
python -m py_compile plot/plotter.py
python plot/plotter.py
```

Output corretto:

```text
Simulator setup loaded from: models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
Result time range: 11.990 - 20.999 s
Reference kinematics: models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
Kinematics ready: 21 coordinates, t in [11.990, 155.045] s
```

## GRF, gait cycle e plot

Questa sezione integra il contenuto del report:

```text
reports/user/2026-05-13_ab06_grf_gait_cycle_plotter.md
```

### Problema nei plot gait-cycle

Nei plot:

```text
03_gaitcycle_torque_angle_power.png
04_gaitcycle_joint_velocity_power.png
```

si osservavano:

- torque-angle plot schiacciati;
- deviazione standard molto alta;
- chattering tra `13 s` e `14 s`.

La causa era il file:

```text
results/sim_output_gait_events.csv
```

che conteneva per il lato sinistro 14 cicli, molti dei quali non fisiologici:

```text
0.0298 s
0.0140 s
0.0140 s
0.0141 s
0.0290 s
0.0069 s
```

Questi micro-segmenti venivano stirati a `0-100%` gait cycle, distorcendo media
e deviazione standard.

### Verifica sul sorgente GRF EPIC

E' stato ispezionato:

```text
models/AB06-raw/10_09_18/treadmill/fp/treadmill_01_01.mat
```

Il file contiene una table `data` da:

```text
143056 x 19
```

Range:

```text
11.990 - 155.045 s
```

Nel `.mat` sorgente sono gia presenti gli stessi micro-crossing a `20 N` visti
nel `.mot` convertito.

Per `Treadmill_L_vy`:

```text
13.430-13.560 s:
min  = -3.214 N
max  = 44.837 N
mean = 6.424 N
```

Crossing a bassa forza:

```text
13.446967
13.476794
13.490773
13.504777
13.518852
13.547862
13.939984
13.946871
```

Conclusione: non e' un errore del converter `.mat -> .mot`; sono artefatti a
bassa forza presenti nel dato raw.

### Confronto con gcLeft/gcRight

I file EPIC `gcLeft/gcRight` confermano eventi puliti:

```text
gcLeft  HS: 13.965  15.635  17.190  18.745  20.305
gcRight HS: 13.180  14.800  16.400  17.970  19.545
```

Questi fronti sono coerenti con soglie piu robuste o con un detector basato su
durata minima del contatto.

### Detector robusto implementato

Per non rompere i dataset gia funzionanti, la soglia storica e' rimasta:

```python
grf_contact_threshold_n = 20.0
```

Sono stati aggiunti:

```python
grf_min_contact_duration_s = 0.05
grf_min_cycle_duration_s = 0.30
```

`output.py` ora:

1. rileva contatti sopra soglia;
2. calcola la durata del contatto;
3. scarta contatti sotto `0.05 s`;
4. costruisce cicli heel-strike -> heel-strike;
5. scarta cicli sotto `0.30 s`.

Il CSV ora include:

```text
cycle_duration_s
contact_duration_s
min_contact_duration_s
min_cycle_duration_s
```

### Eventi e plot rigenerati

E' stato rigenerato:

```text
results/sim_output_gait_events.csv
```

usando:

```text
models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot
```

Nuovi eventi:

```text
left:  4 cicli, durate 1.669, 1.548, 1.553, 1.571 s
right: 4 cicli, durate 1.616, 1.603, 1.570, 1.573 s
```

Plot rigenerati:

```text
plot/13_05_2026 - 1
```

Il plotter ora trova:

```text
left: 4
right: 4
```

I plot gait-cycle risultano molto piu leggibili:

- i torque-angle non sono piu schiacciati dai micro-cicli;
- la deviazione standard di angle/velocity/power e' rientrata;
- resta solo il warning atteso sull'assenza di overlay healthy AB06.

## File principali modificati o creati

Modello, dati e bundle:

- `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim`
- `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml`
- `models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`
- `models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot`
- `models/AB06_SEASEA_Threadmill/data/ExternalForces.xml`
- `models/AB06_SEASEA_Threadmill/data/CMC_Actuators.xml`
- `models/AB06_SEASEA_Threadmill/data/CMC_Tasks - modified Kp_Kv.xml`

Script:

- `tools/convert_epic_ab06_to_opensim.m`
- `tools/calibrate_ab06_seasea_markers.py`
- `scripts/run_opensim_sea_pipeline.py`
- `plot/plotter.py`
- `output.py`
- `config.py`

Risultati:

- `results/ab06_seasea_mass_preserved_pipeline_treadmill_01_01/`
- `results/ab06_seasea_fullspan_pipeline_treadmill_01_01/`
- `results/sim_output_gait_events.csv`
- `plot/13_05_2026 - 1/`

Report:

- `reports/user/2026-05-11_ab06_seasea_epic_pipeline.md`
- `reports/user/2026-05-13_ab06_grf_gait_cycle_plotter.md`
- `reports/daily/2026-05-12_daily-report.md`

## Verifiche eseguite

Sono stati eseguiti:

```bash
python -m py_compile config.py output.py plot/plotter.py
python plot/plotter.py
```

Sono state eseguite verifiche MATLAB R2024a sui file sorgenti EPIC:

- `fp/treadmill_01_01.mat`;
- `gcLeft/treadmill_01_01.mat`;
- `gcRight/treadmill_01_01.mat`.

Sono stati verificati:

- time span sorgente `11.990 - 155.045 s`;
- presenza di micro-crossing GRF a `20 N` nel raw;
- eventi `HeelStrike` puliti in `gcLeft/gcRight`;
- CSV gait-cycle rigenerato con 4 cicli left e 4 right;
- plotter agganciato alla cinematica di riferimento reale.

## Stato finale

Il bundle operativo per la simulazione corrente e':

```text
models/AB06_SEASEA_Threadmill/
```

Setup da usare:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
```

La run corrente usa:

```text
modello:     models/AB06_SEASEA_Threadmill/AB06_SEASEA.osim
cinematica:  models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
GRF:         models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot
```

Il modello e' marker-calibrato e mass-preserving rispetto ad AB06 sano.

Il plotter ora usa il setup corretto e i gait cycle puliti.

## Limiti aperti

Il chattering temporale tra `13 s` e `14 s` resta nei risultati gia simulati,
perche le micro-GRF erano comunque presenti nelle `ExternalLoads` applicate alla
dinamica. La correzione del detector pulisce i plot gait-cycle, ma non modifica
retroattivamente la dinamica gia calcolata.

Per rimuovere il chattering dalla simulazione serve un ulteriore passaggio:

- filtrare o azzerare le GRF sotto soglia quando il contatto non supera la
  durata minima;
- rigenerare il file GRF usato da `ExternalForces.xml`;
- rilanciare la simulazione CMC-like;
- rigenerare i plot.

Il full-span RRA ha ancora un momento residuo elevato, soprattutto `MZ`, quindi
non e' stato scelto come modello finale operativo per la run CMC-like corrente.
