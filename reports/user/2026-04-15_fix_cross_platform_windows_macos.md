# Fix cross-platform Windows/macOS - 2026-04-15

## Problema

Le ultime modifiche del progetto erano state sviluppate e verificate su macOS
M1. Su Windows, durante la costruzione del `SimulationRunner`, la simulazione si
fermava con:

```text
Property 'Kp' not present in Object SEA_Knee
```

La causa non era un errore nella logica SEA, ma una differenza di binding
OpenSim/Python: su questa build Windows le proprieta custom del plugin
`SeriesElasticActuator` non risultano accessibili tramite
`Object.getPropertyByName()`, anche se il plugin viene caricato e usa
correttamente quei valori dal modello `.osim`.

In piu, `visualize.py` su Windows non trovava i file `.vtp` standard della
geometria OpenSim. La cartella locale `Geometry/` contiene principalmente le
mesh protesiche, mentre il modello richiede anche mesh standard come
`pelvis.vtp`, `femur_r.vtp`, `tibia_r.vtp`, ecc. Su macOS queste venivano
trovate tramite l'app OpenSim installata.

## Strategia

La correzione e stata fatta senza cambiare la semantica del simulatore:

- nessuna modifica al plugin C++;
- nessuna modifica alla legge di controllo SEA high-level;
- nessuna modifica a inverse dynamics, static optimization, recruitment
  muscolare o integrazione;
- nessuna scorciatoia con valori duplicati o hardcoded nella logica.

Per le proprieta SEA, la sorgente autorevole e ora il file `.osim`, cioe la
stessa sorgente dati da cui il plugin carica i valori. Questo evita la
reflection fragile via `getPropertyByName()` e mantiene coerente il
comportamento tra macOS e Windows.

Per le mesh, il loader e il visualizzatore registrano piu directory Geometry in
modo cross-platform: cartella locale del progetto, directory del modello,
variabili ambiente OpenSim e percorsi standard macOS/Windows.

## Soluzione

### Lettura proprieta SEA da `.osim`

In `model_loader.py` e stato aggiunto un parser XML leggero che cerca i nodi:

```xml
<SeriesElasticActuator name="SEA_Knee">
...
</SeriesElasticActuator>
```

e legge:

- `stiffness` -> `K`
- `Kp`
- `Kd`
- `motor_damping` -> `Bm`
- `motor_inertia` -> `Jm`
- `optimal_force` -> `F_opt`
- `Impedence` -> `impedance`

I valori confermati dal modello corrente sono:

| SEA | K | Kp | Kd | Bm | Jm | F_opt | Impedence |
|---|---:|---:|---:|---:|---:|---:|---|
| `SEA_Knee` | 250 | 64 | 1.28 | 0.1 | 0.01 | 100 | false |
| `SEA_Ankle` | 500 | 100 | 1.6 | 0.1 | 0.01 | 250 | false |

Questi valori vengono salvati in `SimulationContext.sea_props` e poi usati da
`SimulationRunner` e `OutputRecorder` per ricostruire `tau_spring` e
`tau_input`.

### Compatibilita Geometry

In `model_loader.py` viene registrata la ricerca Geometry prima di caricare il
modello, cosi OpenSim non emette piu warning sui mesh mancanti durante
`opensim.Model(...)`.

In `visualize.py` la stessa logica viene usata prima di aprire il visualizer.
Sono inclusi:

- `Geometry/` e `geometry/` nella root del progetto;
- directory del modello e possibili sottocartelle `Geometry/`;
- `OPENSIM_HOME`, `OPENSIM_ROOT`, `OPENSIM_INSTALL_DIR`;
- `/Applications/OpenSim*/.../Geometry` per macOS;
- `C:\OpenSim*\Geometry` e `C:\Program Files\OpenSim*\Geometry` per Windows.

Il codice Windows-specific e protetto da `os.name == "nt"`, quindi non altera
il percorso macOS.

### Path relativi in `visualize.py`

`visualize.py` ora risolve i path relativi rispetto alla directory dello script
quando serve. Questo permette di lanciarlo anche da una working directory
diversa dalla root del progetto, senza rompere i default di `config.py`.

### Output console Windows

Alcune stampe runtime contenevano caratteri Unicode non rappresentabili dalla
console Windows cp1252, ad esempio frecce e puntini di sospensione Unicode.
Sono state convertite in ASCII (`->`, `...`, `-`) solo nelle stringhe stampate a
runtime. Non cambia nessun calcolo.

## File modificati

- `model_loader.py`
  - aggiunto parser XML per `SeriesElasticActuator`;
  - aggiunto `SimulationContext.sea_props`;
  - registrazione cross-platform delle directory Geometry prima del model load;
  - `sea_f_opt` derivato dai valori letti dal `.osim`.

- `simulation_runner.py`
  - rimossa la dipendenza da `getPropertyByName()` per le proprieta custom SEA;
  - cache SEA popolata da `ctx.sea_props`;
  - `_sea_stiffness()` ora usa prima i valori del `.osim`, con fallback legacy
    alla config.

- `visualize.py`
  - ricerca Geometry cross-platform estesa;
  - diagnostica mesh mancanti;
  - risoluzione robusta di path relativi e varianti plugin `.dll`, `.dylib`,
    `.so`, `lib*.dylib`, `lib*.so`;
  - stampe runtime rese compatibili con console Windows.

- `main.py`
  - stampe principali rese ASCII per evitare errori di encoding su Windows.

- `kinematics_interpolator.py`
  - stampe di caricamento rese ASCII per evitare errori di encoding su Windows.

## Verifiche eseguite

### Syntax/compile

```powershell
python -m py_compile config.py model_loader.py simulation_runner.py visualize.py output.py prosthesis_controller.py main.py kinematics_interpolator.py
```

Risultato: OK.

### Diff check

```powershell
git diff --check -- model_loader.py simulation_runner.py visualize.py main.py kinematics_interpolator.py
```

Risultato: OK. Git segnala solo che alcuni file LF verranno convertiti in CRLF
quando Git li tocchera su Windows; non sono stati rilevati errori di whitespace.

### Smoke run Windows

Ambiente usato:

```powershell
$env:PYTHONPATH = 'C:\OpenSim-mCMC\sdk\Python'
$env:PATH = 'C:\OpenSim-mCMC\bin;C:\OpenSim-mCMC\sdk\Simbody\bin;' + $env:PATH
& 'C:\Users\tomma\AppData\Local\Programs\Python\Python312\python.exe' main.py --t-start 4.26 --t-end 4.28 --output-dir results\_codex_smoke
```

Risultato:

- plugin caricato correttamente;
- modello caricato senza warning sui `.vtp`;
- proprieta SEA lette da `.osim`;
- `SimulationRunner` costruito senza errore `Property 'Kp' not present`;
- 2 step completati;
- output salvati in `results\_codex_smoke`.

### Compatibilita macOS

Non e stata eseguita una run macOS da questa macchina Windows, ma la diff e
stata controllata per evitare conflitti:

- i path Windows sono condizionati da `os.name == "nt"`;
- i path macOS OpenSim.app sono ancora presenti;
- il nome plugin resta senza estensione, come richiesto da
  `opensim.LoadOpenSimLibrary()`;
- la lettura da `.osim` e indipendente dal sistema operativo e usa la stessa
  sorgente dati del plugin.

## Stato

Il fix e cross-platform e non modifica la logica numerica del simulatore. La
differenza principale e che le proprieta custom del plugin non vengono piu
lette via reflection OpenSim/Python, perche quella strada non e portabile tra
le build macOS e Windows.
