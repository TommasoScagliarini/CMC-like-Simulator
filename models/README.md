# Modelli OpenSim

I modelli sono organizzati in cinque famiglie:

```text
models/
├── AB06/
│   ├── AB06-raw/
│   ├── AB06_SEASEA-raw/
│   └── AB06_SEASEA_Threadmill/
├── AB07/
│   ├── AB07-raw/
│   ├── AB07_SEASEA-raw/
│   └── AB07_SEASEA_Threadmill/
├── AB08/
│   ├── AB08-raw/
│   ├── AB08_SEASEA-raw/
│   └── AB08_SEASEA_Threadmill/
├── SEASEA/
├── SEASEA_healthy/
└── Geometry/
```

`SEASEA_healthy` e il nome corretto della precedente cartella
`SEASEA - whealthy data`. Le varianti dei soggetti mantengono i propri nomi,
file e sottocartelle `data/`. Le otto mesh STL condivise sono in `Geometry/`,
separata dalle cinque famiglie di modelli. Simulatore e visualizzatore
registrano questa directory prima delle mesh standard dell'installazione OpenSim.
Non ci sono collegamenti simbolici tra le vecchie e le nuove posizioni.

Il setup predefinito del simulatore e
`AB06/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`.
I nuovi percorsi salvati nelle configurazioni devono includere la famiglia,
per esempio `models/AB07/AB07_SEASEA_Threadmill/...`.

Il modulo centrale [path_resolver.py](<../CMC-like Simulator/path_resolver.py>)
nel core del simulatore riconosce anche i
vecchi indirizzi dei bundle nei setup, nella CLI e nelle configurazioni dei
checkpoint. Gestisce i separatori Windows e macOS e i vecchi percorsi assoluti
di checkout con nome riconosciuto, inclusi i precedenti indirizzi `Geometry/`.
File esterni esistenti e copie archiviate
in `validation/` o `runs/` conservano il proprio significato. Per nuovi punti
di ingresso usare `resolve_repo_path()` o `remap_model_path()`.
La radice delle risorse resta il workspace: `models/` non è stato spostato
dentro `CMC-like Simulator/`.

I dataset locali in `data/` e nelle directory `*-raw/` restano esclusi da Git:
per trasferirli su un altro computer serve anche la copia dei dati.

La riorganizzazione del 2026-10-15 conserva tutti i file `.osim` e i dati
numerici; gli XML operativi hanno soltanto correzioni di percorso. Restano
alcuni riferimenti gia mancanti prima dello spostamento: in particolare i tre
setup healthy non hanno i rispettivi input IK, GRF e reserve nella propria
cartella `data/`. I loro modelli si caricano, ma questi setup richiedono i dati
originali prima di poter essere eseguiti. Non sostituirli con dati di altri
bundle.

Report, manifest, snapshot e script di validazione storici conservano i
percorsi originali come evidenza. Gli script congelati che concatenano quei
percorsi direttamente possono richiedere il checkout precedente per il replay.

Dettagli e verifiche: [report della riorganizzazione](../reports/user/2026-10-15_riorganizzazione_modelli_per_famiglia.md).
