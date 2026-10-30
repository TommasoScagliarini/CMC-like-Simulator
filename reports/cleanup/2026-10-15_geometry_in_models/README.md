# Evidenze — Geometry dentro models, 2026-10-15

Resoconto: [report utente](../../user/2026-10-15_geometry_in_models_e_profili_grf.md).

- `before_manifest.json.gz`: hash di 72.047 file prima dell'operazione;
  `compressed_evidence.json` contiene hash e dimensione del JSON originale.
- `final_repository_integrity.json`: otto mesh rilocate, 13 modifiche previste,
  nessuna perdita o modifica inattesa; 7.060 file dei modelli e 24 JSON dei
  profili immutati.
- `geometry_assets_before.json`, `geometry_assets_after.json`: le otto STL.
- `native_models_geometry_before.json`, `native_models_geometry_after.json`:
  caricamenti nativi e 402 riferimenti mesh, con hash delle destinazioni.
- `native_geometry_migration_comparison.json`: confronto prima/dopo, inclusi
  i 48 STO delle prove AB06/07/08, 30 ms ciascuno.
- `warning_before_after_comparison.json`: warning invariati, zero mesh mancanti.
- `final_runtime_review.json`: 24 controlli di risoluzione dei percorsi,
  directory esterne, priorita e deduplicazione; GUI simulata con un mock.
- `core_paths_tests.log`: 9 test del resolver; `mlp_runtime_contract_tests.log`:
  37 test dei contratti runtime.
- `validation_paths_receipt.json`, log e hash associati: 31 test di
  registrazione/default ABx e 44 test di builder/audit superati; tre test V4
  rimangono non superati per attestazioni storiche. Leggere nel receipt la
  distinzione tra la baseline degli strumenti e quella delle fixture dopo il
  movimento delle directory.
- `mesh_sole_deterministic_migration_comparison.json`: dieci candidati prodotti
  in memoria identici come dizionario e JSON serializzato al vecchio builder
  eseguito in una copia temporanea con i percorsi originali.
- `online_grf_profiles_audit.json`: inventario e caricamento dei 24 profili,
  default dei preset e del checkpoint best47, riferimenti storici.
- `simulation_outputs.tar.gz`: output delle sei brevi simulazioni prima/dopo;
  tutti i file archiviati sono stati riletti e confrontati con gli originali.

Le copie `.py.txt` e `.py.before.txt` sono documentazione degli script
temporanei e dei sorgenti precedenti, non nuovi entry point operativi. I
receipt conservano i percorsi temporanei e assoluti usati durante le prove.

Le prove native sono macOS arm64; non attestano una sessione nativa Windows,
l'apertura della GUI o una nuova qualificazione scientifica completa.

`artifact_manifest.json` contiene dimensioni e SHA delle evidenze, escludendo
se stesso e `delivery_checks.json`. Il controllo finale verifica anche i file
modificati, il report, i collegamenti della documentazione e gli archivi.
