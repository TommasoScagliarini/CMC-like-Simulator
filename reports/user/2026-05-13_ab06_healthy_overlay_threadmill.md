# AB06_SEASEA_Threadmill - Dati healthy AB06 per overlay plotter

Data: 2026-05-13

## Sintesi

E' stata aggiunta al bundle `models/AB06_SEASEA_Threadmill` una cartella
`data/healthy` con i dati sani AB06 convertiti in formato OpenSim `.sto`,
compatibili con `plot/plotter.py`.

Lo scopo era sostituire l'overlay healthy generico del bundle `SEASEA` con un
overlay coerente col soggetto e col trial usati nella simulazione corrente:

```text
models/AB06-raw/10_09_18/treadmill/ik/treadmill_01_01.mat
models/AB06-raw/10_09_18/treadmill/id/treadmill_01_01.mat
```

## Problema

Il plotter cercava dati healthy da usare nei plot gait-cycle, in particolare per
angle, torque e power. Per il bundle AB06_SEASEA_Threadmill serviva una coppia di
file healthy locali, con:

- cinematica sana AB06 in formato `Kinematics_q.sto`;
- momenti/forze sane AB06 in formato `Actuation_force.sto`;
- coordinate compatibili con il modello protesico `AB06_SEASEA`;
- time span completo del trial `treadmill_01_01`.

I dati sorgente EPIC sono `.mat` MATLAB table, quindi non sono stati convertiti
con SciPy ma con MATLAB, in modo coerente con la pipeline gia usata per AB06.

## Soluzione

E' stato creato lo script riproducibile:

```text
tools/export_ab06_healthy_overlay.m
```

Lo script carica i `.mat` IK e ID AB06 raw, valida il vettore temporale comune e
scrive due file nel bundle:

```text
models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Kinematics_q.sto
models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Actuation_force.sto
```

Default dello script:

```text
Task = treadmill
Trial = treadmill_01_01
SourceRoot = models/AB06-raw/10_09_18
OutputDir = models/AB06_SEASEA_Threadmill/data/healthy
```

## Mapping

La cinematica esportata contiene 21 coordinate compatibili con
`AB06_SEASEA`.

Le coordinate sinistre distali sane sono mappate sulla protesi:

```text
knee_angle_l  -> pros_knee_angle
ankle_angle_l -> pros_ankle_angle
```

Sono escluse le coordinate non presenti nel modello protesico:

```text
subtalar_angle_l
mtp_angle_l
```

Nel file `Actuation_force.sto` le colonne sono scritte come reserve compatibili
col plotter. I mapping principali sono:

```text
reserve_pros_knee_angle  <- knee_angle_l_moment
reserve_pros_ankle_angle <- ankle_angle_l_moment
```

Sono incluse anche le altre coordinate disponibili come:

```text
reserve_<coord>
```

## File modificati e generati

File nuovo:

```text
tools/export_ab06_healthy_overlay.m
```

File generati:

```text
models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Kinematics_q.sto
models/AB06_SEASEA_Threadmill/data/healthy/AB06_treadmill_01_01_Actuation_force.sto
```

Nuovo set di plot prodotto per verifica:

```text
plot/13_05_2026 - 3
```

Non sono stati modificati:

- plugin SEA;
- modello `.osim`;
- setup XML;
- simulazione.

## Verifiche

Lo script MATLAB e' stato eseguito con:

```text
/Applications/MATLAB_R2024a.app/bin/matlab -batch "cd('/Users/tommy/Documents/CMC-like-Simulator - Claude'); addpath('tools'); summary = export_ab06_healthy_overlay(); disp(summary)"
```

Risultato esportazione:

```text
Rows: 28612
Time span: 11.990 - 155.045 s
Kinematics columns: 21
Actuation columns: 21
```

Validazione Python con `read_sto()`:

```text
AB06_treadmill_01_01_Kinematics_q.sto:
  rows = 28612
  cols = 21
  range = 11.990 - 155.045 s
  inDegrees = yes
  contiene pros_knee_angle e pros_ankle_angle

AB06_treadmill_01_01_Actuation_force.sto:
  rows = 28612
  cols = 21
  range = 11.990 - 155.045 s
  inDegrees = no
  contiene reserve_pros_knee_angle e reserve_pros_ankle_angle
```

Compilazione Python:

```text
python -m py_compile plot/plotter.py output.py config.py
```

eseguita senza errori.

Il plotter e' stato rilanciato con:

```text
python plot/plotter.py
```

Output rilevante:

```text
Healthy overlay loaded from:
models/AB06_SEASEA_Threadmill/data/healthy

Reference kinematics:
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot

No missing channels.
Plots saved to:
plot/13_05_2026 - 3
```

## Stato finale

Il bundle `models/AB06_SEASEA_Threadmill` ora contiene dati healthy AB06 locali,
derivati dal trial raw EPIC coerente con la simulazione.

Il plotter carica correttamente:

- la cinematica di riferimento effettivamente usata nella simulazione;
- i gait cycle rigenerati;
- l'overlay healthy AB06 da `data/healthy`.

I plot 3 e 4 possono quindi usare angle, torque e power healthy nel range del
trial AB06 completo.
