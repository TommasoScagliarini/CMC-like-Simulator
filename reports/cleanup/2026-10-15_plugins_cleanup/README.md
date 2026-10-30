# Evidenze — pulizia e riorganizzazione plugin, 2026-10-15

Report: [pulizia plugin](../../user/2026-10-15_pulizia_e_riorganizzazione_plugin.md).

- `before_manifest_summary.json`: dimensioni e SHA dell'inventario completo
  acquisito prima delle modifiche, conteggio di 72.320 file e record SHA dei
  file modificati, spostati o rimossi. L'inventario completo resta nel workspace
  temporaneo di verifica; qui e conservato un estratto per non duplicare altri
  grandi inventari nel repository.
- `binary_migration_receipt.json`: sette librerie eliminate e due spostate,
  con byte e SHA. Le copie temporanee di rollback sono state eliminate dopo
  i controlli; nessun archivio dei vecchi binari e stato aggiunto alla repo.
- `runtime_agent_changes.json`, `runtime_path_tests.log`,
  `runtime_native_legacy_load.json`: resolver, caricatore e visualizzatore,
  16 test superati e caricamento nativo dai due vecchi percorsi piatti.
- `tools_changes_and_checks.json` e log associati: tre strumenti aggiornati,
  38 test superati prima e dopo, preflight reale di tre soggetti e vecchia CLI
  Windows accettata. Il converter V14 congelato e invariato.
- `trajectory_changes.json`, `trajectory_tests.log`: inventario ricorsivo dei
  binari nel pilot e guardia hash, 52 test superati inclusi i 37 runtime MLP.
- `final_independent_review.json`: revisione di 60 combinazioni di percorsi,
  inclusi file esterni, namespace storici e nomi delle librerie.
- `cascade_before_after_comparison.json`: 48 STO e 52.920 valori identici
  byte per byte, tre simulazioni cascade+PI di 30 ms ciascuna prima e dopo.
- `best47_after_migration_receipt.json` e log: caricamento reale best47 e due
  passi dopo la migrazione, checksum invariati di checkpoint e configurazione.
- `final_repository_integrity.json`: confronto con la baseline, zero modifiche
  inattese, nessuna perdita oltre alle sette rimozioni previste, HEAD invariato.
- `simulation_outputs.tar.gz`: output delle sei simulazioni brevi e dello
  smoke best47, riletti e confrontati con i file temporanei originali.

Gli script `.py.txt` sono copie documentarie dei verificatori temporanei. I
receipt registrano i percorsi effettivi delle prove. Le qualifiche restano
quelle del report: native macOS arm64, smoke brevi, nessuna sessione Windows
nativa o nuova validazione scientifica completa.

I manifest e gli script congelati preservano percorsi e hash originali. Per i
replay che richiedono vecchie varianti o accessi diretti ai percorsi precedenti
serve la copia precedente. Gli alias operativi riguardano solo i due plugin
correnti e non sostituiscono una dinamica SEA diversa.

`artifact_manifest.json` elenca SHA e dimensioni delle evidenze, esclusi se
stesso e `delivery_checks.json`; quest'ultimo controlla anche sorgenti,
documentazione, layout e binari conservati.
