# Evidenze — rimozione profili GRF, 2026-10-15

Resoconto: [report utente](../../user/2026-10-15_pulizia_profili_grf_online.md).

- `removal_plan.json`, `removal_receipt.json`: 19 nomi rimossi, byte, SHA e
  commit verificato per il recupero esatto. Nessuna copia dei 19 JSON rimossi
  e stata aggiunta alle evidenze.
- `independent_integrity_before.json`, `independent_integrity_after.json`:
  confronto dei cinque profili conservati e di codice/config/plugin, validita
  dei profili e risoluzione dei riferimenti YAML.
- `mlp_runtime_contract_tests.log`: 37 test superati.
- `best47_legacy_smoke_receipt.json`, stdout/stderr e
  `best47_smoke_outputs.tar.gz`: smoke reale dopo la rimozione, due passi
  OpenSim e checksum invariati del checkpoint e della configurazione salvata.
- `removed_profile_code_references.txt`: i 19 nomi non hanno consumatori
  operativi specifici trovati; sono ancora citati negli inventari storici.
- `retained_validation_profiles_audit.json`: uso, limiti e requisiti di
  riproduzione dei due profili di validazione conservati.
- `retained_profiles_git_recovery.json`: i due profili conservati sono
  byte-identici alle rispettive versioni del commit corrente.
- `v4_regeneration_receipt.json`, `v4_regenerated_current_checkout.json`, log
  del builder: rigenerazione V4 eseguita soltanto su un file temporaneo;
  geometria e parametri runtime esatti, quattro differenze nei metadati.

Gli script `.py.txt` sono copie documentarie dei verificatori temporanei. I
receipt conservano i percorsi effettivi usati durante le prove. Nessuna nuova
calibrazione sensor-basis e stata eseguita e nessun manifest congelato e stato
riscritto per adeguarlo alla rimozione dei profili.

`artifact_manifest.json` registra le evidenze, esclusi se stesso e
`delivery_checks.json`; quest'ultimo verifica anche la working tree e i
collegamenti della documentazione aggiornata.
