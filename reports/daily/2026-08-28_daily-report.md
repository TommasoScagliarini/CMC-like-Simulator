# Daily Report - 2026-08-28

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [J20 restore audit R2 — esecuzione](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md)
- [J20 restore audit R3 — preparazione](../user/2026-08-28_v26c_j20_restore_audit_r3_preparazione.md)
- [J20 restore audit R3 — correzione rev7 del gate learning rate](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md)
- [J20 restore audit R3 — GO operativo](../user/2026-08-28_v26c_j20_restore_audit_r3_go_operativo_approvato.md)
- [J20 restore audit R3 — esecuzione](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md)
- [J21 — preparazione attestazione training-ready](../user/2026-08-28_v26c_j21_preparazione_attestazione_training_ready.md)
- [J21 — correzione rev1](../user/2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md)
- [J21 — attestazione finale training-ready](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md)
- [Guida di ricostruzione della pipeline ex-novo da V26 agosto](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md)

## Sintesi

La pipeline AB06 ha raggiunto lo stato **`TRAINING_READY_ATTESTED`** senza lanciare alcun training actor-updating. Il restore audit R2 ha raggiunto il percorso reale di restore, ma è fallito closed a **7/13** per otto mismatch di rappresentazione nel `param_group` dell'optimizer — booleani/tensori scalari e float Python/float32 — mentre i momenti Adam coincidevano. R3 ha corretto il criterio confrontando la semantica post-conversione e ha reso vincolante anche il learning rate ripristinato.

L'esecuzione R3 è passata **13/13**: modulo, actor, critic, sigma, momenti, step e learning rate sono stati ripristinati esattamente; G9 è stato chiuso. J21 ha poi aggregato le evidenze già prodotte senza costruire environment né eseguire update: dopo le correzioni su numero dei gate, scope dei contatori e doppia autorizzazione, l'attestazione finale è passata **18/18**, con `training_ready: true` e promozione limitata a `TRAINING_INPUT_ONLY`.

Il checkpoint finale è quello J20 critic-only; l'attore coincide con J19A, mentre il critic è stato inizializzato con una sola iterazione/4096 step. Il training PPO/ex-novo successivo resta **non lanciato e non autorizzato**. È stata inoltre scritta una guida dettagliata che ricostruisce l'intera lineage dal parent imitativo V26 agosto, inclusi fallimenti, correzioni, artefatti canonici e riferimenti storici.

## Lavoro svolto

- Eseguito R2 una sola volta e diagnosticato il falso mismatch dell'optimizer state.
- Preparato, corretto e autorizzato R3 con confronto canonico fail-closed e gate sul learning rate.
- Eseguito R3 una sola volta: `RESTORE_AUDIT_PASS`, G9 chiuso, nessun training/sampling/rollout.
- Preparato e revisionato J21; corretti i gate T15, i contatori e il requisito di doppia autorizzazione.
- Eseguito J21: `TRAINING_READY_ATTESTED`, promozione `TRAINING_INPUT_ONLY`.
- Creata la guida di ricostruzione completa in `reports/user/`, senza modificare checkpoint o artefatti storici.

## Test e verifiche

- R2: preflight **76/76**, esecuzione FAIL_CLOSED **7/13** con restore effettivamente raggiunto.
- R3 rev7: suite **344/344**, preflight **92/92**; esecuzione **13/13 PASS**.
- J21: suite finale **381/381**, preflight **18/18 gate e 139/139 pin**, GO **141/141 pin**, esecuzione **18/18 PASS**.
- Post-commit J21: set esatto di quattro file, hash coerenti, marker corretto e nessun processo trainer/J21 residuo.
- Guida: riferimenti storici e artefatti canonici verificati; nessun comando di training autorizzato dalla documentazione.

## TODO completati

- [x] Correggere e superare il restore audit reale del checkpoint J20.
- [x] Chiudere G9 senza mutare retroattivamente la foglia sorgente.
- [x] Attestare separatamente il checkpoint come training-ready.
- [x] Arrestarsi prima di qualsiasi training PPO/ex-novo.
- [x] Documentare passo per passo la ricostruzione della pipeline dal V26 imitativo di agosto.

## TODO aperti e propagati

- [ ] Prima di qualsiasi training, preregistrare e revisionare un pilot PPO conservativo con config, comando, stop rule, monitor, rollback, nuovo GO e nuova autorizzazione dell'utente.
- [ ] Rivalutare sigma nel protocollo del pilot; 0,005 è verificata per la qualifica corrente ma non va ereditata automaticamente.
- [ ] Monitorare la penetrazione: tutte le celle A–I superano 20 mm; F e H entrano nella banda diagnostica ≥25 mm; nessuna supera la soglia hard di 28 mm.
- [ ] Generalizzazione multimodello con dataset EPIC e validazione su soggetto completamente held-out.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri e non fanno parte della baseline operativa; ogni integrazione richiede approvazione esplicita dell'utente.
- [ ] Conservare come debito storico il sidecar J8 stantio; non modificarlo retroattivamente.
- [ ] Valutare separatamente le osservazioni non gateate di R3 rev7, il marker sorgente `RESTORE_AUDIT_PENDING`, i due `gcs_server` orfani del 18/08 e i check storici R1/R2 legati all'assenza della destinazione post-esecuzione.
- [ ] Debiti tecnici/amministrativi ereditati non esplicitamente chiusi: semantica `cycle_valid` dopo HS cancellato, osservabilità RLlib, swing timeout 1,3 s, probe schema `train_ppo_mlp.py:1408`, pin V20, `best == last`, `fit_p2` V12R3 e deflessione SEA iniziale.

