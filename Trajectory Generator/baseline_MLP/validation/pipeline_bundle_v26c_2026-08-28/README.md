# V26C validation pipeline bundle — 2026-08-28

Questo bundle rende fisicamente disponibile sotto `baseline_MLP/validation`
la chiusura della pipeline V26C J0–J21 senza invalidarne la lineage storica.

## Contenuto

- `objects/sha256/`: object store byte-identico a path corto. Il manifest
  conserva la topologia logica repo-relative senza ripeterla sul filesystem,
  evitando i path oltre MAX_PATH prodotti dai checkpoint RL su Windows.
  Contiene runtime production, tool di validazione, configurazioni, asset
  AB06, plugin macOS, parent imitativo V26, run B0820, precedenti July/V26B e
  report usati per ricostruzione e analisi.
- `forensics/j5_run.log`: copia recuperata dal percorso temporaneo della
  sessione J5; SHA-256 atteso
  `83ec482c2261c757aee9450d97bdf65be952d1bc5cc5e71a8c4a0aed5622f887`.
- `bundle_manifest.json`: indice deterministico old-path → mirror-path con
  SHA-256, dimensione, ruolo e stage consumer.
- `in_place_manifest.json`: inventario content-addressed dei runner, test e
  artefatti già presenti nelle workspace F0/F1/F2R/V26B/V26C.
- `materialize_pipeline_bundle.py`: materializzatore/verificatore standard
  library, no-clobber e cross-platform.
- `test_pipeline_bundle.py`: regressione strutturale del bundle.

Gli oggetti sono un archivio, non un secondo runtime. Le
copie production originali restano canoniche perché J20/J21, gli import flat e
i pickle dipendono dai percorsi e dai byte storici.

Le directory complete B0820, parent August e July sono snapshot di contesto
conservativi. Solo i file realmente aperti dalla pipeline ricevono nel
manifest ruoli come `operational_input`, `runtime_configuration_anchor` o
`methodological_evidence_opened_by_pipeline`; gli altri sono marcati
`context_snapshot_extra` e non implicano un accesso file-per-file.

## Verifica

Dalla root del repository:

    python "Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/materialize_pipeline_bundle.py" --verify

La verifica completa ricalcola anche gli hash dei 2,6 GB già in place:

    python "Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/materialize_pipeline_bundle.py" --verify --full

Il confronto opzionale con tutti gli originali canonici live è separato, così
il bundle resta verificabile anche quando i run originali ignorati da Git non
sono disponibili:

    python "Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/materialize_pipeline_bundle.py" --verify --full --check-live-sources

La suite del bundle si esegue con:

    python "Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/test_pipeline_bundle.py"

## Confini e gap dichiarati

- La run attestata è macOS e usa le due dylib incluse. Gli equivalenti Windows
  `SEA ... _ff.dll` e `OnlineGRFContact.dll` non erano presenti, quindi il
  bundle documenta il gap invece di fabbricare binari.
- Per le analisi manuali J13/J17 e parte della ricalibrazione J18 non furono
  creati script riusabili. Sono preservati i report e i JSON di misura: il gap
  di provenance è esplicito nel manifest.
- Alcune mesh OpenSim di sola visualizzazione risultano assenti; non furono
  necessarie per la run computazionale attestata. Le quattro mesh disponibili
  e referenziate sono incluse; il manifest elenca le quattordici VTP mancanti.
- Non esiste un lock completo dell'ambiente `baseline_MLP`: i leaf registrano
  alcune versioni chiave, ma non chiudono tutte le dipendenze Python/native.
- I path assoluti e gli interpreti macOS presenti nei leaf storici sono dati di
  provenance e non vengono riscritti.

Il bundle non autorizza training, PPO, rollout, deployment o promozioni oltre
lo scope già attestato `TRAINING_INPUT_ONLY`.

## Windows

Senza supporto long-path, la workspace storica in-place richiede una root di
checkout molto corta (il limite calcolato è nel manifest; ad esempio `C:\cmc`).
In alternativa occorre abilitare `LongPathsEnabled` nel sistema e
`git config --global core.longpaths true` prima del checkout, quindi passare
`--windows-long-paths-enabled` al verificatore. Gli oggetti del bundle hanno
nomi corti; i path lunghi residui appartengono esclusivamente ai leaf storici
che non possono essere rinominati senza invalidare i pin.
