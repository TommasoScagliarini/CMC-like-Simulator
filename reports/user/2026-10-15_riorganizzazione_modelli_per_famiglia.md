# 2026-10-15 — Riorganizzazione dei modelli per famiglia

La cartella `models/` contiene ora cinque famiglie: `AB06`, `AB07`, `AB08`,
`SEASEA` e `SEASEA_healthy`. Le tre varianti di ciascun soggetto sono state
raggruppate nella relativa famiglia. Il nome precedente
`SEASEA - whealthy data` e stato corretto in `SEASEA_healthy`.

## Problema e soluzione

I bundle raw, SEA raw e treadmill erano tutti nello stesso livello di
`models/`. Sono state eseguite dieci rinomine di directory sullo stesso disco,
senza duplicare dati o creare symlink. Esempio:

```text
models/AB06/
    AB06-raw/
    AB06_SEASEA-raw/
    AB06_SEASEA_Threadmill/
```

La stessa struttura vale per AB07 e AB08. `models/SEASEA/` conserva il proprio
contenuto. La struttura completa e descritta in [models/README.md](../../models/README.md).

## Strategia e file modificati

Prima dello spostamento sono stati acquisiti hash, riferimenti XML, caricamenti
nativi e simulazioni brevi di confronto. Tre subagenti hanno lavorato su asset
e XML, runtime e checkpoint, strumenti e provenienza; le verifiche finali sono
state rieseguite anche indipendentemente.

- Spostati 7.045 dei 7.060 file presenti nei modelli, per un inventario totale
  di 5.829.808.852 byte. Nessun file originario e stato perso.
- Conservati byte per byte tutti i 22 `.osim` e i 6.977 asset numerici
  MAT/MOT/STO/TRC/CSV. Solo 20 XML hanno cambiamenti di contenuto: 70
  sostituzioni di percorso, senza modifiche ai parametri.
- Aggiornati `config.py`, `setup_io.py`, `visualize.py` e il setup ricordato
  localmente in `.simulator_last_setup.json` al nuovo layout.
- Esteso `path_resolver.py` per accettare i vecchi percorsi dei bundle, anche
  con separatori Windows e vecchi checkout riconosciuti. Non vengono
  reindirizzati file esterni esistenti o copie archiviate in directory annidate.
- Aggiornati 33 file di `Trajectory Generator/`: default e factory delle tre
  famiglie MLP, controlli di compatibilita, utility, 19 preset operativi e test.
  Le configurazioni salvate nei checkpoint non richiedono riscrittura.
- Aggiornati nove file degli strumenti: builder ABx, pipeline OpenSim,
  calibrazione marker, diagnostica driver e cascade, due script MATLAB e un
  esempio nel README online GRF. I nuovi output del builder vengono generati
  dentro la famiglia del soggetto.
- Adattata `.gitignore` alle directory raw annidate; aggiunta documentazione
  in `AGENTS.md` e `models/README.md`, oltre al test
  `validation/test_model_layout_paths.py`.

L'inventario completo dei cambiamenti e i relativi hash si trovano nelle
[evidenze della migrazione](../cleanup/2026-10-15_models_reorganization/README.md).

## Verifiche eseguite

| Verifica | Esito |
| --- | --- |
| Hash dei 7.060 file originari dei modelli | Tutti presenti: 7.040 identici, 20 XML corrispondenti agli hash previsti |
| Controllo esteso di 72.190 file preesistenti | 72.121 identici; 69 modifiche previste; nessuna perdita o modifica inattesa |
| Stato di esclusione Git dei 7.060 file dopo la rilocazione | Invariato |
| Parsing di 63 XML/OSIM e controllo di 116 riferimenti | Nessun nuovo riferimento rotto |
| Caricamento OpenSim e `initSystem()` prima/dopo | 22/22 modelli; stessa struttura di coordinate, stati e forze |
| Cascade+PI AB06, AB07, AB08, 30 ms per soggetto | 48 file STO e 52.920 valori: identici anche byte per byte |
| Test resolver e setup del simulatore | 7/7 PASS |
| Test contratti runtime MLP | 37/37 PASS |
| Test esistenti builder ABx | 14/14 PASS |
| Discovery reale ABx in `--validate-only` | Trovati AB06, AB07, AB08 e riferimenti condivisi |
| Factory delle tre famiglie MLP e 19 preset | Percorsi risolti correttamente |
| Checkpoint reale best47 con vecchio percorso setup, avvio da `/tmp` | PASS: caricamento e due passi; checkpoint e configurazione salvata invariati |
| `git diff --check` | PASS |

Nel test best47 l'attore conserva 35 azioni e 84 osservazioni. Il comando non
viene clippato; il residuo dinamico massimo e 2,64e-13 N m. Il test e uno smoke
di caricamento e integrazione breve, non la validazione di un episodio completo.
Le prove native sono state eseguite su macOS arm64. La portabilita dei percorsi
Windows e coperta dai test; non e stata eseguita una sessione nativa Windows
ne il motore MATLAB. Non sono stati modificati plugin o parametri fisici.

## Limiti e riferimenti gia mancanti

Il confronto prima/dopo distingue i collegamenti funzionanti dalle anomalie
gia presenti. Restano 21 riferimenti input mancanti: nove nei tre setup
healthy, quattro `ExtLoads.xml` nei template raw, tre percorsi Dropbox Windows
nei vecchi setup IK raw, due riferimenti statici AB06 al vecchio nome
inesistente `AB06_SEASEA`, tre risultati IK non generati usati come input ID.
Sono distinti dalle dieci destinazioni output gia assenti prima delle prove.
Elenco completo nelle [note sui riferimenti mancanti](../cleanup/2026-10-15_models_reorganization/models_missing_reference_qualification.md).

I tre modelli healthy si caricano correttamente, ma i rispettivi setup
richiedono ancora cinematica, GRF e reserve originali: la loro cartella `data/`
era gia vuota prima della rinomina. Non sono stati sostituiti con dati di altri
bundle. Anche il vecchio modello PD richiesto dalla diagnostica driver
`AB06_SEASEA_stiff321_500.osim` era gia assente; questa diagnostica completa non
e stata eseguita ne sostituita con un altro modello.

## Provenienza e validazioni storiche

Manifest, snapshot, checkpoint, report precedenti e script congelati
conservano i propri contenuti. Le DLL e dylib presenti in `plugins/` sono
identiche alla baseline. Il controllo esteso non rileva modifiche inattese.

L'archivio V26C supera `--verify`: 692 oggetti, 257.138.708 byte. La sua suite
storica resta parzialmente non superata: **17 PASS, 1 ERROR, 1 SKIP su 19**.
L'errore e il disallineamento preesistente del file J1, identico alla baseline
di questa operazione. Il test della source closure ora salta perche sei file
non si trovano piu nei vecchi percorsi dei modelli. Questo e una riduzione di
copertura rispetto al precedente errore della closure, non una correzione
dell'anomalia. Il contenuto congelato dell'archivio rimane verificabile.

Gli script storici che concatenano direttamente i vecchi indirizzi possono
richiedere il checkout precedente per il replay. Non vengono aggiornati
silenziosamente gli hash delle evidenze per adeguarli al codice corrente.

HEAD rimane `dba42388c32a441a5543f20881612a52781d7a99`. Non sono stati creati
commit o push. Le modifiche e le cancellazioni gia presenti prima di questa
riorganizzazione sono state preservate.
