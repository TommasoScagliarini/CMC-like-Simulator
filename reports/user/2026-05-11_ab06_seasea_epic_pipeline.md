# AB06_SEASEA EPIC Pipeline

Data: 2026-05-11

Aggiornamento struttura: 2026-05-12

Il bundle finale e' stato riorganizzato nello standard `models/SEASEA`: i sorgenti/import raw sono stati spostati in `models/*-raw`, mentre `models/AB06_SEASEA` contiene solo il modello finale, il setup CMC-like e i dati minimi necessari alla simulazione.

## Problema

Il modello `AB06` sano del dataset EPIC Lab Lower Limb Biomechanics Dataset doveva essere adattato a una protesi SEA su ginocchio e caviglia sinistri e poi usato con dati sperimentali AB06 per IK, ID, RRA e simulatore CMC-like.

I problemi emersi erano quattro:

- il modello protesico iniziale era caricabile, ma i marker sinistri distali erano agganciati in modo troppo grezzo ai nuovi body protesici;
- i `.mat` EPIC erano MATLAB table/MCOS e non leggibili in modo robusto con SciPy;
- le GRF convertite avevano forze e COP corretti, ma i momenti treadmill erano scritti come `ground_torque` senza rimuovere il contributo `r x F`;
- il modello `AB06_SEASEA` pesava circa `69.76 kg`, mentre AB06 sano pesa `78.10 kg`, generando un residuo verticale RRA di circa `-76 N`.

## Soluzione

E' stata prodotta una pipeline AB06_SEASEA completa basata su:

- modello raw `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim`;
- modello raw marker-calibrato `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim`;
- conversione MATLAB dei trial EPIC in TRC/MOT/XML OpenSim;
- GRF corrette con free torque verticale al COP;
- massa totale preservata rispetto ad AB06 sano;
- pipeline OpenSim eseguibile su macOS senza `opensim-cmd`, tramite OpenSim Python API quando necessario.

Il modello finale consigliato per la run CMC-like corrente e ora nel bundle standard:

`models/AB06_SEASEA/Adjusted_newmarkers_pipeline_ready.osim`

Setup associato:

`models/AB06_SEASEA/Adjusted_newmarkers_pipeline_ready_setup.xml`

Report pipeline:

`results/ab06_seasea_mass_preserved_pipeline_treadmill_01_01/pipeline_report.md`

## Strategia

1. Copiato `models/AB06` in `models/AB06_SEASEA` e creato `AB06_SEASEA.osim`; successivamente questo materiale sorgente e' stato archiviato in `models/AB06_SEASEA-raw`.

2. Sostituita la catena sinistra sana:

   `femur_l`, `tibia_l`, `talus_l`, `calcn_l`, `toes_l`

   con:

   `transfemur`, `osseo_pylon`, `tibia_pylon`, `foot_l`.

3. Sostituite le coordinate sinistre distali sane:

   `knee_angle_l`, `ankle_angle_l`, `subtalar_angle_l`, `mtp_angle_l`

   con:

   `pros_knee_angle`, `pros_ankle_angle`.

4. Aggiunti gli attuatori SEA:

   - `SEA_Knee -> pros_knee_angle`
   - `SEA_Ankle -> pros_ankle_angle`

5. Creato `tools/convert_epic_ab06_to_opensim.m` per convertire i `.mat` EPIC in:

   - `.trc` marker;
   - `_grf.mot`;
   - `_ExternalLoads.xml`;
   - `_ik_dataset_ab06_seasea.mot`;
   - setup IK/ID specifici per trial.

6. Convertiti i trial:

   - `treadmill/treadmill_01_01`;
   - `static/static_01`.

7. Creato `tools/calibrate_ab06_seasea_markers.py` per calibrare i marker sinistri protesici usando `static_01.trc` e IK dataset.

8. Verificata la corrispondenza force plate/piede usando:

   - pagina ufficiale EPIC Lab sul dataset Camargo et al.;
   - nomi raw del `.mat`: `Treadmill_L_*`, `Treadmill_R_*`;
   - confronto COP-marker nel trial treadmill.

9. Corretto il trattamento dei momenti GRF:

   - i canali treadmill `moment_*` si comportano come momenti rispetto all'origine laboratorio;
   - OpenSim applica gia `r x F` quando riceve COP e forza;
   - il converter ora usa `TorqueMode='free_vertical'`, cioe scrive solo il free torque verticale al COP.

10. Redistribuita la massa della gamba sinistra sana sulla catena protesica, preservando la massa totale AB06.

## File Modificati o Creati

Modelli e setup:

- `models/AB06_SEASEA/Adjusted_newmarkers_pipeline_ready.osim`
- `models/AB06_SEASEA/Adjusted_newmarkers_pipeline_ready_setup.xml`
- `models/AB06_SEASEA/data/IK_rra_window.mot`
- `models/AB06_SEASEA/data/ExternalForces.xml`
- `models/AB06_SEASEA/data/treadmill_01_01_grf.mot`
- `models/AB06_SEASEA/data/CMC_Actuators.xml`
- `models/AB06_SEASEA/data/CMC_Tasks - modified Kp_Kv.xml`
- `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA.osim`
- `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.osim`
- `models/AB06_SEASEA-raw/osimxml/AB06_SEASEA_marker_calibrated.marker_calibration.csv`
- `models/AB06_SEASEA-raw/osimxml/iksetup.xml`
- `models/AB06_SEASEA-raw/osimxml/idsetup.xml`

Conversione e pipeline:

- `tools/convert_epic_ab06_to_opensim.m`
- `tools/calibrate_ab06_seasea_markers.py`
- `scripts/run_opensim_sea_pipeline.py`
- `scripts/run_measured_grf_window_tests.py`

Dati convertiti principali:

- `models/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01.trc`
- `models/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_grf.mot`
- `models/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_ExternalLoads.xml`
- `models/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_ik_dataset_ab06_seasea.mot`
- `models/AB06_SEASEA-raw/data/converted/static/static_01/static_01.trc`
- `models/AB06_SEASEA-raw/data/converted/static/static_01/static_01_ik_dataset_ab06_seasea.mot`

Risultati pipeline:

- `results/ab06_seasea_pipeline_treadmill_01_01/`
- `results/ab06_seasea_marker_calibrated_pipeline_treadmill_01_01/`
- `results/ab06_seasea_corrected_grf_pipeline_treadmill_01_01/`
- `results/ab06_seasea_mass_preserved_pipeline_treadmill_01_01/`

Non e' stato modificato il plugin C++ SEA.

## Verifiche Eseguite

### Caricamento Modello

Il modello marker-calibrato si inizializza correttamente con plugin SEA:

- coordinate: `21`
- body: `11`
- `SEA_Knee` presente;
- `SEA_Ankle` presente.

### Conversione MATLAB

Conversione `treadmill_01_01`:

- TRC: `28612` frame, `28` marker, `200 Hz`;
- GRF MOT: `143056` righe, `19` colonne, `1000 Hz`;
- intervallo comune marker/GRF: `11.990 - 155.045 s`.

Conversione `static_01`:

- TRC: `400` frame, `28` marker, `100 Hz`;
- IK dataset disponibile;
- GRF assenti, correttamente saltate con warning.

### Calibrazione Marker

Sul trial static, la calibrazione marker ha ridotto l'errore locale medio dei marker sinistri:

- prima: circa `0.100755 m`;
- dopo: circa `0.000616 m`.

Sul treadmill:

- IK full window prima calibrazione: RMS `0.089647 m`, max `0.397880 m`;
- IK full window dopo calibrazione: RMS `0.009161 m`, max `0.107720 m`;
- IK RRA window dopo calibrazione: RMS `0.007392 m`, max `0.027178 m`.

### Force Plate e COP

Mapping confermato:

- `Treadmill_L -> ground_force1 -> foot_l`;
- `Treadmill_R -> ground_force2 -> calcn_r`.

Con soglia `>100 N`:

- plate1-COP vicino al piede sinistro: `0.0876 m`;
- plate1-COP vicino al piede destro: `0.4291 m`;
- plate2-COP vicino al piede destro: `0.0917 m`;
- plate2-COP vicino al piede sinistro: `0.4212 m`.

COP:

- `py = 0` per entrambi i plate;
- zero-COP durante carico `>100 N`: `0.00%`.

### Segni e Frame

Le forze verticali sono coerenti con il peso AB06 sano:

- massa AB06 sano: `78.09885033 kg`;
- BW: circa `765.89 N`;
- GRF totale media: circa `757.86 N`.

Il problema sui momenti e' stato corretto trasformando i momenti da origine laboratorio a free torque verticale al COP.

Effetto della correzione:

- `moment_norm` RRA prima: circa `695.97 Nm`;
- `moment_norm` dopo correzione torque: circa `85.31 Nm`.

### Sincronizzazione

Marker e GRF sono allineati:

- TRC start/end: `11.990 - 155.045 s`;
- GRF start/end: `11.990 - 155.045 s`;
- delta start: `0.000000000 s`;
- delta end: `0.000000000 s`.

Cross-correlation contatto piede/forza:

- plate1-left: lag circa `-0.020 s`;
- plate2-right: lag circa `-0.060 s`;
- lato sbagliato molto peggiore.

Non e' emersa evidenza di desincronizzazione globale.

### Massa Preservata

Massa gamba sinistra sana rimossa:

```text
femur_l   9.6645049193 kg
tibia_l   3.8522321359 kg
talus_l   0.1039037663 kg
calcn_l   1.2987970788 kg
toes_l    0.2250555578 kg
totale   15.1444934582 kg
```

Nuova massa protesica:

```text
transfemur    9.6645049193 kg
osseo_pylon   1.9866857556 kg
tibia_pylon   1.8655463803 kg
foot_l        1.6277564030 kg
totale       15.1444934582 kg
```

Massa totale:

```text
AB06 sano       78.09885033 kg
AB06_SEASEA     78.09885033 kg
```

Le inerzie dei body protesici sono state scalate proporzionalmente alle nuove masse.

### Pipeline Finale

Pipeline finale:

`results/ab06_seasea_mass_preserved_pipeline_treadmill_01_01/`

Risultati RRA finali:

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

- return code: `0`;
- status: `complete`;
- intervallo smoke: `18.965 - 18.995 s`;
- SEA forward mode: `plugin`.

## Stato Finale

Il modello AB06_SEASEA e' ora utilizzabile con i dati sperimentali AB06 convertiti per la pipeline IK/ID/RRA/CMC-like.

Il problema verticale principale era la massa mancante della gamba sinistra protesica; dopo redistribuzione mass-preserving, il residuo verticale e' sceso da circa `-76.6 N` a circa `+6.6 N`.

Il problema principale residuo non e' piu' la massa totale o la sincronizzazione GRF-marker, ma il momento residuo ancora non nullo (`~68.7 Nm`), probabilmente legato a distribuzione COM/inerzie della protesi e tuning RRA piu' fine.
