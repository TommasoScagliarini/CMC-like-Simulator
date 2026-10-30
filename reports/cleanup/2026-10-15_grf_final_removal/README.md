# Evidenze — ultima rimozione profili GRF, 2026-10-15

Report: [rimozione finale](../../user/2026-10-15_rimozione_finale_profili_grf_validazione.md).

- `removal_receipt.json`: i due JSON eliminati, hash, byte e commit verificato
  prima della cancellazione per il ripristino esatto. Nessuna copia dei JSON
  eliminati e stata aggiunta a questa directory.
- `independent_final_before.json`, `independent_final_after.json`: tre profili
  validi, dipendenze YAML risolte, confronto di 172 file e dei 54 superstiti
  della baseline precedente. La cattura corrente e successiva all'unlink;
  il confronto precedente fornisce la continuita con lo stato anteriore.
- `independent_final_check.log`, `mlp_runtime_contract_tests.log`: esiti reali,
  inclusi i 37 test superati.
- `verify_final_profiles.py.txt`: copia documentaria del verificatore temporaneo.

Le verifiche storiche che richiedono i file eliminati conservano i loro
contratti originali e necessitano del ripristino da Git. Le prove qui raccolte
riguardano i profili e gli ingressi operativi correnti. Non e stato eseguito
un nuovo rollout, ne una sessione nativa Windows.

Il manifest delle evidenze e `artifact_manifest.json`; `delivery_checks.json`
verifica anche i file rimasti e i collegamenti della documentazione.
