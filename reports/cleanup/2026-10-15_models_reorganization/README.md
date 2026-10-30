# Evidenze — riorganizzazione modelli, 2026-10-15

Resoconto: [report utente](../../user/2026-10-15_riorganizzazione_modelli_per_famiglia.md).

## Inventari e conservazione

- `before_manifest.json.gz`: baseline SHA-256 di 72.190 file presenti prima
  dell'operazione, incluso lo stato Git precedente e le cancellazioni gia presenti.
- `model_files_before.json.gz`: inventario dei 7.060 file originari dei modelli.
- `directory_mapping.json`, `file_mapping.json.gz`, `move_journal.json` e
  `move_result.json`: dieci spostamenti e corrispondenza vecchio/nuovo percorso.
- `models_xml_changes.patch`, piano e receipt XML: 20 file, 70 sostituzioni di
  percorso; nessuna modifica ai parametri o ai file `.osim`.
- `final_models_integrity.json.gz`: verifica indipendente di tutti i modelli.
- `final_repository_integrity.json`: confronto esteso, 72.121 file identici e
  69 cambiamenti previsti, zero perdite o cambiamenti inattesi.
- `change_inventory.json`: elenco dei cambiamenti e hash dei nuovi file sorgente
  e della documentazione. Le ulteriori evidenze sono elencate nel manifest finale.
- `compressed_evidence.json`: nomi, dimensioni e SHA dei JSON non compressi;
  ogni archivio gzip e stato decompresso e confrontato con l'originale.

I file `.py.txt` sono copie documentarie degli script temporanei di acquisizione
e verifica. I percorsi assoluti nei receipt descrivono il workspace usato nella
prova. Non sono nuove dipendenze del simulatore.

## Collegamenti e funzionamento

- `models_asset_reference_baseline.json`, `models_reference_after_check.json`:
  116 riferimenti con destinazioni equivalenti; zero nuovi riferimenti rotti.
- `models_missing_reference_qualification.md`, `healthy_qualification.json`:
  elenco e distinzione tra 21 riferimenti input gia mancanti e dieci
  destinazioni output gia assenti; nove input mancanti riguardano healthy.
- `native_models_before.json`, `native_models_after.json`, confronto nativo:
  caricamento e inizializzazione dei 22 modelli.
- `cascade_before_after_comparison.json`: AB06/AB07/AB08, 30 ms ciascuno,
  48 STO e 52.920 valori identici byte per byte.
- `core_model_layout_tests.log`: 7 test resolver/setup superati.
- Receipt e log `trajectory_*`: factory, 19 preset e 37 test dei contratti.
- Receipt e log `tools_*`: 14 test builder, discovery reale in validate-only,
  percorsi MATLAB controllati senza eseguire il motore MATLAB.
- `best47_legacy_smoke_receipt.json` e log associati: caricamento reale del
  checkpoint, vecchio percorso setup da `/tmp`, due passi e checksum invariati.
- `final_path_resolver_review.json`: revisione indipendente del resolver.
- `simulation_outputs.tar.gz`: output e log delle sei brevi simulazioni di
  confronto, smoke best47 e variante temporanea dell'oracle. Tutti i file
  archiviati sono stati riletti e confrontati con gli originali.

Le prove native sono macOS arm64. I test coprono anche la sintassi Windows dei
percorsi, senza dichiarare una prova nativa Windows o un rollout scientifico
completo.

## Evidenze storiche

`provenance_review.json` e `models_v26c_final_receipt.json` descrivono la
conservazione di manifest, controlli e snapshot. Archivio V26C `--verify`: PASS.
Suite storica: 17 PASS, 1 ERROR preesistente J1, 1 SKIP della closure per i sei
file spostati dai vecchi percorsi. Lo SKIP riduce la copertura: non risolve il
precedente errore di closure. I log completi sono conservati.

`artifact_manifest.json` registra hash e dimensioni delle evidenze, esclusi se
stesso e `delivery_checks.json`. Quest'ultimo controlla manifest, archivi,
collegamenti della documentazione e hash finali dei file modificati.
