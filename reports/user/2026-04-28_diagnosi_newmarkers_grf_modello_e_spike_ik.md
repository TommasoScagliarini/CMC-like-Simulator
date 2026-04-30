# Report - Diagnosi newmarkers, GRF, modello adjusted e spike IK

## Problema

Il setup `Adjusted_newmarkers_setup.xml` doveva usare il modello
`Adjusted_newmarkers.osim`, scalato su un nuovo set di marker sano, con lo
scopo di far inseguire alla protesi un riferimento sano.

La simulazione inizialmente falliva o produceva risultati fuori scala. Le cause
da verificare erano:

- correttezza del file `ExternalForces.xml`;
- compatibilita del modello `Adjusted_newmarkers.osim` con il plugin SEA;
- coerenza tra IK, GRF e finestra temporale;
- presenza di spike nel riferimento cinematico sano.

## Diagnosi

### 1. ExternalForces XML

La prima versione del file GRF conteneva identificatori scambiati:

- `force_identifier = FP1_p`
- `point_identifier = FP1_v`

Nel file `.mot`, invece:

- `FP*_v` sono le forze;
- `FP*_p` sono i punti di applicazione;
- `FP*_moment_` sono i momenti.

Questo faceva applicare quasi una posizione come forza e una forza come punto.
Il risultato era un carico dinamico irrealistico.

Il file e stato poi corretto. La versione verificata usa:

- destra: `FP1_v`, `FP1_p`, `FP1_moment_` su `calcn_r`;
- sinistra: `FP2_v`, `FP2_p`, `FP2_moment_` su `foot_l`;
- datafile: `C:\Users\tomma\Downloads\Archive\ground_walking_cw_fp.mot`.

Tutte le colonne richieste sono presenti nel `.mot`.

### 2. Modello Adjusted_newmarkers

Durante una verifica intermedia, il modello non conteneva piu gli attuatori SEA
nel `ForceSet`, e il loader falliva con:

```text
[ModelLoader] Attuatore 'SEA_Knee' non trovato nell'ActuatorSet.
```

Il modello e stato poi corretto reinserendo:

- `SeriesElasticActuator name="SEA_Knee"` su `pros_knee_angle`;
- `SeriesElasticActuator name="SEA_Ankle"` su `pros_ankle_angle`.

I parametri SEA verificati sono coerenti con il modello tuned precedente:

- knee: `F_opt=100`, `K=1000`, `Kp=3.9`, `Kd=9.7`;
- ankle: `F_opt=250`, `K=700`, `Kp=8.8`, `Kd=9.7`.

La smoke run finale conferma che il modello viene caricato, i SEA vengono
trovati, e le state variables del plugin vengono lette.

### 3. Problema dinamico residuo

Dopo la correzione di GRF e modello, il fallimento di caricamento e risolto,
ma resta un problema dinamico nel tratto critico intorno a `t = 19.6 s`.

Test `19.600 -> 19.750 s`:

```text
status=complete
step=150
SEA max u:
  knee  = 1.0
  ankle = 1.0
SEA_Ankle_tau_input_raw max ~= 819.9 Nm
SEA_Ankle_tau_input_saturated = 1.0
pelvis_ty_reserve_torque max ~= 1080 Nm
pelvis_tx_reserve_torque max ~= 582 Nm
```

Quindi il setup ora gira, ma il riferimento porta ancora il sistema fuori scala.

La causa piu probabile e uno spike nel file:

```text
models/SEASEA - whealthy data/data/3DGaitModel2392-scaled_Kinematics_q.sto
```

In particolare:

- `pros_ankle_angle` cambia molto rapidamente attorno a `19.655 s`;
- la velocita numerica stimata arriva a circa `9300 deg/s`;
- anche `ankle_angle_r` mostra picchi molto alti attorno a `17.56 s`.

Con cutoff IK piu severo a `2 Hz`, sul tratto critico erano stati osservati
miglioramenti forti:

- `max reserve_tau`: circa `1106 -> 261 Nm`;
- `SEA_Ankle max |u|`: `1.0 -> 0.79`;
- `SEA_Ankle_tau_input_raw`: circa `820 -> 362 Nm`.

Questo indica che il riferimento sano/newmarkers e troppo brusco per la
pipeline attuale con filtro a `6 Hz`.

### 4. Copertura temporale GRF

Nel `.mot`, dentro la finestra del setup `16.9 -> 22.999`, i contatti utili
sopra soglia sono limitati:

```text
FP1_vy > 20 N: 16.9 -> 19.172 s
FP2_vy > 20 N: 18.965 -> 19.839 s
```

Dopo circa `19.839 s`, il riferimento cinematico continua ma le GRF utili sono
praticamente assenti. Questo puo contribuire ai grandi reserve su pelvis e anca.

## Soluzione / Stato attuale

Sono stati verificati e corretti:

- `ExternalForces.xml`, ora coerente con le colonne del `.mot`;
- `Adjusted_newmarkers.osim`, ora compatibile con il plugin SEA;
- setup XML, che punta ai file corretti.

La simulazione non fallisce piu in fase di setup/caricamento.

Restano da affrontare:

- spike e filtraggio del riferimento IK newmarkers;
- compatibilita dinamica tra riferimento sano e protesi SEA;
- scelta della finestra temporale in base alla copertura GRF reale;
- possibile allineamento temporale e assegnazione trial IK/GRF.

## File coinvolti

- `models/SEASEA - whealthy data/Adjusted_newmarkers_setup.xml`
- `models/SEASEA - whealthy data/Adjusted_newmarkers.osim`
- `models/SEASEA - whealthy data/data/ExternalForces.xml`
- `models/SEASEA - whealthy data/data/3DGaitModel2392-scaled_Kinematics_q.sto`
- `C:\Users\tomma\Downloads\Archive\ground_walking_cw_fp.mot`

## Test e verifiche eseguite

- Parsing statico di `ExternalForces.xml`.
- Verifica delle colonne `FP1_*`, `FP2_*`, `FP5_*`, `FP6_*` nel `.mot`.
- Verifica degli intervalli di contatto GRF:
  - `FP1_vy > 20 N`;
  - `FP2_vy > 20 N`.
- Parsing del modello `Adjusted_newmarkers.osim` per coordinate, body, muscoli e SEA.
- Confronto con `models/SEASEA/Adjusted_SEASEA - Copia_tuned.osim`.
- Smoke run:

```text
conda run -n envCMC-like python main.py --setup "models\SEASEA - whealthy data\Adjusted_newmarkers_setup.xml" --t-start 16.9 --t-end 16.901 --output-dir results\_diag_newmarkers_adjusted_check
```

Esito: completata.

- Test breve:

```text
16.900 -> 17.100 s
status=complete
step=200
```

- Test sul tratto critico:

```text
19.600 -> 19.750 s
status=complete
step=150
```

Esito: completato, ma con saturazione SEA e reserve pelvis molto elevati.

## Raccomandazione

La prossima iterazione dovrebbe concentrarsi su:

1. filtrare o pulire il riferimento IK newmarkers, partendo dal tratto
   `19.6 -> 19.75 s`;
2. provare cutoff IK piu basso, ad esempio `2 Hz`, come baseline diagnostica;
3. limitare temporaneamente `t_end` entro la copertura GRF utile;
4. verificare se IK e GRF appartengono allo stesso trial e sono temporalmente
   allineati.
