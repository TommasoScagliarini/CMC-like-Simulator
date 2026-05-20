# Pipeline unica ABx_SEASEA

Data: 2026-05-13

## Problema

La pipeline sviluppata per `AB06_SEASEA` era ricostruibile dai report e dagli
script presenti, ma non esisteva ancora un comando unico capace di generalizzare
il processo ai soggetti EPIC `ABx`.

In particolare, i passaggi erano distribuiti tra:

- modello AB06 gia materializzato in `.osim`;
- conversione MATLAB specifica per AB06;
- calibrazione marker protesici;
- pipeline OpenSim IK/ID/RRA/CMC-like;
- export overlay healthy AB06;
- bundle operativo `AB06_SEASEA_Threadmill`.

L'obiettivo era creare un entrypoint generale per soggetti `ABxx-raw` presenti
localmente, senza download automatici e senza modificare il plugin SEA.

## Soluzione

E' stato creato un nuovo entrypoint:

```text
tools/build_abx_seasea_pipeline.py
```

Lo script orchestra la pipeline ABx -> ABx_SEASEA e supporta:

- discovery automatica di `models/ABxx-raw`;
- selezione modello sano da `osimxml/*.osim`;
- selezione trial statico, con preferenza `static_01`, poi `static01`;
- selezione trial operativo, con preferenza al trial richiesto o al primo trial
  valido del task;
- graft modello sano -> modello SEASEA sinistro;
- preservazione massa totale;
- conversione MATLAB EPIC `.mat -> .trc/.mot/.xml`;
- calibrazione marker protesici;
- preparazione bundle operativo `models/ABxx_SEASEA_Treadmill/`;
- export overlay healthy soggetto-specifico;
- RRA diagnostico opzionale;
- smoke test opzionale.

Comando principale:

```bash
python tools/build_abx_seasea_pipeline.py --subject AB07 --task treadmill --trial treadmill_01_01
```

## Strategia

### Discovery soggetto

Lo script cerca soggetti locali nella forma:

```text
models/ABxx-raw/
```

Per ogni soggetto identifica:

- cartella sessione EPIC, ad esempio `10_14_18`;
- modello sano in `osimxml/`;
- trial statico;
- trial operativo;
- disponibilita di `markers`, `fp`, `ik`, `id`;
- plugin SEA;
- cartella supporto `models/SEASEA/data`.

Esempio validato:

```bash
python tools/build_abx_seasea_pipeline.py --subject AB07 --validate-only
```

### Graft modello ABx -> ABx_SEASEA

Il graft usa come base il modello sano del soggetto e come donor un modello
SEASEA esistente.

La catena sana sinistra:

```text
femur_l
tibia_l
talus_l
calcn_l
toes_l
```

viene sostituita con:

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

vengono sostituite da:

```text
pros_knee_angle
pros_ankle_angle
```

Gli attuatori SEA aggiunti sono:

```text
SEA_Knee  -> pros_knee_angle
SEA_Ankle -> pros_ankle_angle
```

I marker sinistri vengono riagganciati ai body protesici:

- marker coscia su `transfemur`;
- marker ginocchio su `osseo_pylon`;
- marker shank su `tibia_pylon`;
- marker ankle/heel/toe su `foot_l`.

La massa totale viene preservata:

- `transfemur` riceve la massa di `femur_l`;
- la massa restante della catena sinistra sana viene distribuita su
  `osseo_pylon`, `tibia_pylon`, `foot_l` secondo le proporzioni del donor;
- le inerzie dei body protesici sono scalate proporzionalmente alla nuova massa.

Durante il graft vengono rimossi i muscoli/forze sane sinistre che referenziano
i body o le coordinate eliminate, e vengono aggiunti i componenti protesici dal
donor.

## File modificati o creati

File nuovo:

```text
tools/build_abx_seasea_pipeline.py
```

File aggiornati:

```text
tools/convert_epic_ab06_to_opensim.m
tools/export_ab06_healthy_overlay.m
```

Modifiche principali:

- `convert_epic_ab06_to_opensim.m` ora applica il mapping
  `knee_angle_l -> pros_knee_angle` e `ankle_angle_l -> pros_ankle_angle`
  per qualunque `TargetModel` che contenga `SEASEA`, non solo per
  `AB06_SEASEA`.
- `export_ab06_healthy_overlay.m` accetta il parametro `Subject`, cosi puo
  generare file healthy con prefisso `AB07`, `AB08`, ecc.
- `build_abx_seasea_pipeline.py` espone opzioni:
  - `--subject ABxx`;
  - `--all-subjects`;
  - `--task treadmill|levelground|ramp|stair`;
  - `--trial`;
  - `--static-trial`;
  - `--run-rra`;
  - `--skip-opensim-ik`;
  - `--skip-smoke-test`;
  - `--force`;
  - `--validate-only`;
  - `--python` per indicare un interprete Python con OpenSim disponibile.

## Verifiche eseguite

### Compilazione Python

```bash
python -m py_compile tools/build_abx_seasea_pipeline.py scripts/run_opensim_sea_pipeline.py tools/calibrate_ab06_seasea_markers.py
```

Risultato: nessun errore.

### Discovery soggetti

Eseguiti:

```bash
python tools/build_abx_seasea_pipeline.py --subject AB06 --validate-only
python tools/build_abx_seasea_pipeline.py --subject AB07 --validate-only
python tools/build_abx_seasea_pipeline.py --subject AB08 --validate-only
python tools/build_abx_seasea_pipeline.py --all-subjects --validate-only
```

Risultato: discovery corretta per AB06, AB07, AB08.

Dettagli confermati:

- AB06 usa static trial `static_01`;
- AB07 e AB08 usano static trial `static01`;
- tutti e tre hanno trial operativo `treadmill/treadmill_01_01`;
- `markers`, `fp`, `ik`, `id` sono presenti per il trial treadmill;
- il plugin SEA esiste;
- `models/SEASEA/data` esiste.

### Graft modello

Sono stati generati graft temporanei in `/private/tmp` per:

```text
AB07
AB08
```

Verifiche:

- massa totale preservata;
- riferimenti a `femur_l`, `tibia_l`, `talus_l`, `calcn_l`, `toes_l` rimossi;
- riferimenti a `knee_angle_l`, `ankle_angle_l`, `subtalar_angle_l`,
  `mtp_angle_l` rimossi;
- presenti `transfemur`, `osseo_pylon`, `tibia_pylon`, `foot_l`;
- presenti `pros_knee_angle`, `pros_ankle_angle`;
- presenti `SEA_Knee`, `SEA_Ankle`.

Risultati massa:

```text
AB07: 58.80765053 kg -> 58.80765053 kg
AB08: 72.41236865 kg -> 72.41236865 kg
```

### Conversione MATLAB generalizzata

Eseguita conversione temporanea AB07 in `/private/tmp`:

```matlab
convert_epic_ab06_to_opensim( ...
  'SubjectDir','models/AB07-raw/10_14_18', ...
  'Task','static', ...
  'Trial','static01', ...
  'OutputDir','/private/tmp/abx_convert_test', ...
  'TargetModel','AB07_SEASEA', ...
  'ModelFile','/private/tmp/AB07_SEASEA_graft_test.osim')
```

Risultato:

- `static01.trc` creato;
- `static01_ik_dataset_ab06_seasea.mot` creato;
- GRF correttamente saltate per il trial statico.

Eseguita anche conversione AB07 treadmill:

```matlab
convert_epic_ab06_to_opensim( ...
  'SubjectDir','models/AB07-raw/10_14_18', ...
  'Task','treadmill', ...
  'Trial','treadmill_01_01', ...
  'OutputDir','/private/tmp/abx_convert_test', ...
  'TargetModel','AB07_SEASEA', ...
  'ModelFile','/private/tmp/AB07_SEASEA_graft_test.osim')
```

Risultato:

- TRC creato;
- GRF MOT creato;
- ExternalLoads XML creato;
- IK dataset creato;
- setup IK/ID creati.

### Overlay healthy

Eseguito export temporaneo AB07 in `/private/tmp`:

```matlab
export_ab06_healthy_overlay( ...
  'Subject','AB07', ...
  'SourceRoot','models/AB07-raw/10_14_18', ...
  'Task','treadmill', ...
  'Trial','treadmill_01_01', ...
  'OutputDir','/private/tmp/abx_healthy_test')
```

Risultato:

```text
Rows: 28612
Time span: 10.360 - 153.415 s
Kinematics columns: 21
Actuation columns: 21
```

File generati:

```text
AB07_treadmill_01_01_Kinematics_q.sto
AB07_treadmill_01_01_Actuation_force.sto
```

## Limiti aperti

Il Python disponibile nella shell corrente non importa `opensim`.

Per questo motivo:

- lo script segnala `python_has_opensim: false` in `--validate-only`;
- una build completa richiede `--python` puntato a un ambiente Python con
  OpenSim disponibile;
- il load-test OpenSim del modello graftato e lo smoke test CMC-like completo
  non sono stati eseguiti in questa sessione.

Esempio:

```bash
python tools/build_abx_seasea_pipeline.py \
  --subject AB07 \
  --task treadmill \
  --trial treadmill_01_01 \
  --python /path/to/opensim-python
```

## Stato finale

La pipeline unica ABx -> ABx_SEASEA e' stata implementata come entrypoint
generale.

Lo script e' pronto per soggetti EPIC locali `ABxx-raw`, inclusi futuri
AB09-AB30 se organizzati con la stessa struttura vista in AB06, AB07 e AB08.

Il bundle operativo nuovo usa spelling corretto:

```text
models/ABxx_SEASEA_Treadmill/
```

Il bundle storico:

```text
models/AB06_SEASEA_Threadmill/
```

rimane compatibile ma non viene usato come nuovo standard.
