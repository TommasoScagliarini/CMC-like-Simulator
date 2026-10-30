# Daily Report - 2026-08-28

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

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
- [Rollout del best ex-novo V26C/J20, sette plot canonici e morphology corridor](../user/2026-08-28_rollout_best_exnovo_v26c_j20_plot_canonici_e_corridor_morfologico.md)

## Sintesi

La pipeline AB06 ha dapprima raggiunto lo stato **`TRAINING_READY_ATTESTED`**
senza lanciare alcun training actor-updating. Il restore audit R2 ha raggiunto
il percorso reale di restore, ma è fallito closed a **7/13** per otto mismatch
di rappresentazione nel `param_group` dell'optimizer — booleani/tensori scalari
e float Python/float32 — mentre i momenti Adam coincidevano. R3 ha corretto il
criterio confrontando la semantica post-conversione e ha reso vincolante anche
il learning rate ripristinato.

L'esecuzione R3 è passata **13/13**: modulo, actor, critic, sigma, momenti, step e learning rate sono stati ripristinati esattamente; G9 è stato chiuso. J21 ha poi aggregato le evidenze già prodotte senza costruire environment né eseguire update: dopo le correzioni su numero dei gate, scope dei contatori e doppia autorizzazione, l'attestazione finale è passata **18/18**, con `training_ready: true` e promozione limitata a `TRAINING_INPUT_ONLY`.

Il checkpoint training-ready è quello J20 critic-only; l'attore coincide con
J19A, mentre il critic è stato inizializzato con una sola iterazione/4096 step.
È stata inoltre scritta una guida dettagliata che ricostruisce l'intera lineage
dal parent imitativo V26 agosto, inclusi fallimenti, correzioni, artefatti
canonici e riferimenti storici.

In un momento successivo, dopo la separata autorizzazione dell'utente, da J20 è
stato eseguito il training `MLP_ExNovo_V26C_J20_pilot_2026-08-28` per 50 nuove
iterazioni. Il best checkpoint, selezionato all'iterazione logica 24, è stato
valutato con un rollout deterministico da `500/500` step. Il rollout ha
completato due cicli validi senza terminazione fisica, ma ha confermato due
osservazioni diagnostiche: penetrazione massima `24,47 mm` e served reference
ankle fuori corridor in `64/373` campioni.

## Lavoro svolto

- Eseguito R2 una sola volta e diagnosticato il falso mismatch dell'optimizer state.
- Preparato, corretto e autorizzato R3 con confronto canonico fail-closed e gate sul learning rate.
- Eseguito R3 una sola volta: `RESTORE_AUDIT_PASS`, G9 chiuso, nessun training/sampling/rollout.
- Preparato e revisionato J21; corretti i gate T15, i contatori e il requisito di doppia autorizzazione.
- Eseguito J21: `TRAINING_READY_ATTESTED`, promozione `TRAINING_INPUT_ONLY`.
- Creata la guida di ricostruzione completa in `reports/user/`, senza modificare checkpoint o artefatti storici.
- Dopo una nuova autorizzazione, eseguito il training ex-novo da J20 per 50
  iterazioni e selezionato `rl_module_best` all'iterazione logica 24.
- Eseguito il rollout deterministico completo del best checkpoint con output
  fisici, policy trace ed eventi gait.
- Generati i sette plot canonici e il plot 8 della served reference nel
  morphology corridor.

## Test e verifiche

- R2: preflight **76/76**, esecuzione FAIL_CLOSED **7/13** con restore effettivamente raggiunto.
- R3 rev7: suite **344/344**, preflight **92/92**; esecuzione **13/13 PASS**.
- J21: suite finale **381/381**, preflight **18/18 gate e 139/139 pin**, GO **141/141 pin**, esecuzione **18/18 PASS**.
- Post-commit J21: set esatto di quattro file, hash coerenti, marker corretto e nessun processo trainer/J21 residuo.
- Guida: riferimenti storici e artefatti canonici verificati; nessun comando di training autorizzato dalla documentazione.
- Training ex-novo: `50/50` nuove iterazioni, stop regolare, best mean episode
  return `51,4205762`.
- Rollout best: `500/500`, return `42,2664728`, `3` HS, `3` TO, `2` cicli,
  zero eventi invalidi e zero action clipping.
- Penetrazione: `24,47 mm`, sopra la soglia soft `20 mm` ma sotto `25/28 mm`.
- Plot canonici: sette PNG validi e nessun canale mancante.
- Plot 8: bound ricostruiti entro `2,22e-16 rad`; knee fuori `10/373`
  (`2,68%`), ankle fuori `64/373` (`17,16%`).

## TODO completati

- [x] Correggere e superare il restore audit reale del checkpoint J20.
- [x] Chiudere G9 senza mutare retroattivamente la foglia sorgente.
- [x] Attestare separatamente il checkpoint come training-ready.
- [x] Arrestarsi prima di qualsiasi training PPO/ex-novo.
- [x] Documentare passo per passo la ricostruzione della pipeline dal V26 imitativo di agosto.
- [x] Dopo la successiva autorizzazione, eseguire il primo training ex-novo da
  J20, il rollout del best checkpoint e produrre gli otto plot diagnostici.

## TODO aperti e propagati

- [ ] Prima di ogni ulteriore training, preregistrare e revisionare il pilot con
  config, comando, stop rule, monitor, rollback, nuovo GO e nuova autorizzazione
  dell'utente.
- [ ] Rivalutare sigma nel protocollo del pilot; 0,005 è verificata per la qualifica corrente ma non va ereditata automaticamente.
- [ ] Monitorare la penetrazione con le bande `20/25/28 mm`; il rollout best
  ex-novo ha raggiunto `24,47 mm`.
- [ ] Diagnosticare e correggere il toe-off ankle: `64/373` campioni della
  served reference sono fuori corridor, contro `10/373` per il knee.
- [ ] Separare sempre served reference e cinematica fisica nelle valutazioni
  del corridor.
- [ ] Generalizzazione multimodello con dataset EPIC e validazione su soggetto completamente held-out.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri e non fanno parte della baseline operativa; ogni integrazione richiede approvazione esplicita dell'utente.
- [ ] Conservare come debito storico il sidecar J8 stantio; non modificarlo retroattivamente.
- [ ] Valutare separatamente le osservazioni non gateate di R3 rev7, il marker sorgente `RESTORE_AUDIT_PENDING`, i due `gcs_server` orfani del 18/08 e i check storici R1/R2 legati all'assenza della destinazione post-esecuzione.
- [ ] Debiti tecnici/amministrativi ereditati non esplicitamente chiusi: semantica `cycle_valid` dopo HS cancellato, osservabilità RLlib, swing timeout 1,3 s, probe schema `train_ppo_mlp.py:1408`, pin V20, `best == last`, `fit_p2` V12R3 e deflessione SEA iniziale.


## Addendum retrospettivo del 5 ottobre 2026 — completamento dei report di fine agosto

Questa integrazione ricostruisce le informazioni mancanti dai report originali.
Il testo precedente e la data del daily restano invariati. Esiti, test, GO,
training e rollout citati appartengono alle esecuzioni storiche: il 5 ottobre
sono stati controllati soltanto documenti e collegamenti. Le vecchie leaf,
le autorizzazioni consumate e i risultati negativi conservano il loro stato.
I TODO ancora aperti sono riportati separatamente dai passi risolti o superati.

<a id="completion-20261005-2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto"></a>

### Guida di ricostruzione della pipeline ex-novo training-ready dal training imitativo V26 di agosto

Fonte: [2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

La guida ricostruisce la lineage operativa August V26→J19A→J20→R3→J21 e i contratti 35D/clock zero, nuove leaf senza clobber e GO monouso. AB06 soltanto; sigma 0.005 di qualifica non autorizza un pilot. La replica deve usare un nuovo namespace/leaf senza clobber, input hashati della sola lineage operativa agosto e GO nuovi, mai i token consumati dei comandi storici. J11→J17 può essere omesso dalla ricostruzione ottimizzata perché non è antenato dei byte finali, mantenendolo come diagnosi; il parent operativo rimane J8→J19A. La guida vincola mapping per nome, bias compensation, actor35/critic84, sigma misurata, DAgger student-visited e gate6/6+3/3; restore reale e attestazione read-only sono separati. I riferimenti/artefatti/digest furono verificati nel daily28. AB06, margini20/25/28 e future EPIC/pilot restano limiti; nessun comando storico autorizza una nuova esecuzione. Artefatti pertinenti del report: `module_state.pkl`, `train_ppo_mlp.py`.

<a id="completion-20261005-2026-08-28_rollout_best_exnovo_v26c_j20_plot_canonici_e_corridor_morfologico"></a>

[Rollout del best checkpoint ex-novo V26C/J20, plot canonici e corridor morfologico](../user/2026-08-28_rollout_best_exnovo_v26c_j20_plot_canonici_e_corridor_morfologico.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-28_v26c_j20_restore_audit_r2_esecuzione"></a>

### V26C — J20 R2: esecuzione del secondo tentativo correttivo del restore audit

Fonte: [2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).

R2 raggiunge il vero restore ma fallisce 7/13 per otto mismatch di rappresentazione param_group. I momenti Adam coincidono; conversione Torch/RLlib, non corruzione del checkpoint. G9 e decisione orfani restano aperti. Gli otto mismatch riguardano rappresentazioni bool/scalari/float nei param_group, mentre i dodici tensori dei momenti Adam coincidono byte per byte. La conversione `convert_to_torch_tensor` di RLlib2.55.1 modifica tali rappresentazioni: non prova checkpoint corrotto. Preflight76 e restore reale raggiunto costituiscono il risultato, ancora FAIL 7/13. R3 corregge il comparatore con una nuova semantica dichiarata; restano assertion storiche di assenza-destinazione, marker e decisione sugli orfani, senza affermare processi attivi oggi. Artefatti pertinenti del report: `v26c_j20_restore_audit_r2_architect_go.json`, `v26c_j20_restore_audit_receipt.json`, `v26c_j20_restore_audit_result.json`.

<a id="completion-20261005-2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate"></a>

### V26C — J20 R3: correzione rev7, il learning rate ripristinato diventa un gate

Fonte: [2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).

R3 rev7 rende vincolante il learning rate ripristinato, prima osservato senza effetto sul gate. Suite 344 e pin 92; tre osservazioni restano non vincolanti e la decisione sugli orfani rimane aperta. Il learning rate osservato prima non faceva fallire il gate in caso di scostamento: rev7 introduce l’enforcement fail-closed, con 344 check e 92 pin. Le tre osservazioni non gateate restano distinte: clausola conversione downcast/None, riproduzione raw dei mismatch R2 e tipi optimizer registrati. Il resto del contratto continua a verificare stato/momenti/step/LR esatti; non sono nuove tolleranze scientifiche. GO e R3 successivi risolvono la readiness; marker, assertion legacy e decisione orfani restano debiti. Artefatti pertinenti del report: `v26c_j20_prereg_restore_audit_rev7.json`, `v26c_j20_restore_audit_r3_go_DRAFT_rev7.json`, `v26c_j20_restore_audit_r3_child.py`.

<a id="completion-20261005-2026-08-28_v26c_j20_restore_audit_r3_esecuzione"></a>

### V26C — J20 R3: esecuzione del terzo tentativo correttivo del restore audit

Fonte: [2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).

R3 passa 13/13 a zero update: state, Adam e LR ripristinati esattamente. G9 chiuso ma training_ready ancora falso fino a un’attestazione separata; marker della fonte immutabile, osservazioni e orfani restano debiti. Il PASS 13/13 chiudeG9 a zero update e non rende automaticamente training_ready vero: occorre J21, poi eseguito separatamente. I byte del warm-up e il suo marker `RESTORE_AUDIT_PENDING` restano immutabili come stato storico della fonte, anche dopo la prova esterna positiva. Le tre osservazioni rev7 non vincolanti, i check R1/R2 divenuti falsi post-esecuzione e la decisione sugli orfani non sono chiusi dal restore. Generalizzazione oltre AB06 rimane aperta. Artefatti pertinenti del report: `v26c_j20_restore_audit_result.json`, `v26c_j20_restore_audit_receipt.json`, `module_state.pkl`.

<a id="completion-20261005-2026-08-28_v26c_j20_restore_audit_r3_go_operativo_approvato"></a>

[V26C — J20 R3: materializzazione del GO operativo dell'architetto](../user/2026-08-28_v26c_j20_restore_audit_r3_go_operativo_approvato.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.

<a id="completion-20261005-2026-08-28_v26c_j20_restore_audit_r3_preparazione"></a>

### V26C — J20 R3: preparazione del terzo tentativo correttivo del restore audit

Fonte: [2026-08-28_v26c_j20_restore_audit_r3_preparazione.md](../user/2026-08-28_v26c_j20_restore_audit_r3_preparazione.md).

R3 prepara un confronto canonico sotto la stessa conversione RLlib, esatto su dtype/shape/byte senza tolleranze o cast manuali. Suite 297 e pin 91; R2 non prova checkpoint corrotto, GO ed esecuzione separati. Il confronto canonico applica ai due lati la stessa funzione reale di conversione RLlib e poi pretende dtype/shape/byte identici: nessuna tolleranza, cast manuale o sostituzione dei momenti. Suite 297/pin 91 passati; rev7 aggiunge il gateLR e porta92 pin. R3 eseguito poi chiudeG9. Il falso mismatch R2 è una rappresentazione serializzata, distinta dalla corruzione; assertion legacy, marker storico e decisione orfani restano amministrativamente aperti. Artefatti pertinenti del report: `v26c_j20_prereg_restore_audit_rev6.json`, `v26c_j20_restore_audit_r3_child.py`, `v26c_j20_restore_audit_r3.py`.

<a id="completion-20261005-2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready"></a>

### V26C — J21: correzione rev1 dell'attestazione di aggregazione (training-ready)

Fonte: [2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md](../user/2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md).

J21 rev1 corregge T15: J19A aveva legittimamente promosso actor, mentre la restrizione riguarda J20 training input. Verifica contatori tipizzati, worker sync reale, lineage e integrità; 18 gate e pin 139. T15 originariamente negava qualunque promozione a monte, ma J19A registra legittimamente actor_promoted:true. Rev1 limita l’attestazione alla promozione del checkpoint J20 come training input, verifica contatori tipizzati e worker-sync realmente osservato, oltre a lineage/hash;354 check,18 gate e 139 pin. L’esecuzione finale corregge ulteriormente testo/doppio GO. Restano le osservazioni R3, marker, assertion legacy e decisione orfani, senza autorizzare un pilot dal solo PASS preparatorio. Artefatti pertinenti del report: `v26c_j21_prereg_training_ready_attestation.json`, `v26c_j21_prereg_training_ready_attestation_rev1.json`, `v26c_j21_training_ready_attestation.py`.

<a id="completion-20261005-2026-08-28_v26c_j21_preparazione_attestazione_training_ready"></a>

### V26C — J21: preparazione dell'attestazione di aggregazione delle evidenze (training-ready)

Fonte: [2026-08-28_v26c_j21_preparazione_attestazione_training_ready.md](../user/2026-08-28_v26c_j21_preparazione_attestazione_training_ready.md).

J21 prepara un aggregatore read-only delle evidenze e distingue attestazione del checkpoint da autorizzazione del pilot. Prima proposta 15 gate/pin 129, poi corretta; sigma e limiti AB06 restano da trattare nel protocollo. L’aggregatore usa libreria standard, rihasha prove collegate e attesta un artefatto senza costruire ambiente o aggiornare pesi. La prima proposta15 gate/pin 129 è superata da18 gate/pin 139 e dalla verifica finale381; la distinzione checkpoint-ready/permesso di training rimane. J21 conclude poi TRAINING_INPUT_ONLY e il training50 del28 richiede autorizzazione successiva separata. Restano marker, osservazioni R3, assertion legacy e decisione orfani; il nuovo pilot non può riusare un GO storico. Artefatti pertinenti del report: `module_state.pkl`, `v26c_j21_prereg_training_ready_attestation.json`, `v26c_j21_training_ready_attestation.py`.

<a id="completion-20261005-2026-08-28_v26c_j21_training_ready_attestation_esecuzione"></a>

[V26C — J21: attestazione finale `training_ready`](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md): copertura sostanziale già presente nelle sezioni precedenti, verificata contro la fonte; esiti e verifiche pregresse non vengono duplicati.


### TODO riconciliati dalle fonti di questa data

Il registro distingue passi storici eseguiti, piani superati, debiti aperti e
condizioni/invarianti per eventuali lavori futuri. Le chiusure qui citate sono
retrospettive: nessuna nuova autorizzazione o esecuzione deriva dall’addendum.

- **Aperto — Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint.** [fonte, riga 250](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- **Risolto — Revisionare l’aggregatore J21 e ottenere GO separato/doppia autorizzazione per una sola attestazione.** [fonte, riga 258](../user/2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md). Chiusura/supersessione 2026-08-28: [J21 attesta training_ready e TRAINING_INPUT_ONLY con 18/18, senza nuove esecuzioni.](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md).
- **Condizionato — Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati.** [fonte, riga 276](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- **Condizionato — Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget.** [fonte, riga 1059](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- **Aperto — Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati.** [fonte, riga 299](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).
- **Condizionato — Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training.** [fonte, riga 1029](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- **Risolto — Revisione di rev 7 e GO approvato con autorizzazione utente per R3.** [fonte, riga 297](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md). Chiusura/supersessione 2026-08-28: [R3 restore PASS 13/13 e G9 chiuso a zero update.](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- **Condizionato — Rivalutare sigma nel protocollo del pilot e dopo actor predittivo/cambiamento causale morphology; non ereditare automaticamente 0.005 o 0.010.** [fonte, riga 1055](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- **Risolto — Progettare/autorizzare/eseguire l’attestazione separata che porta training_ready a vero dopo R3, senza addestrare.** [fonte, riga 300](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md). Chiusura/supersessione 2026-08-28: [J21 attesta training_ready e TRAINING_INPUT_ONLY con 18/18, senza nuove esecuzioni.](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md).

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).
- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).
- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).
- [ ] **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).
- [ ] Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate. — [fonte 2026-08-27, riga 233](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici. — [fonte 2026-08-27, riga 353](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata. — [fonte 2026-08-27, riga 219](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).
- [ ] Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti. — [fonte 2026-08-27, riga 231](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11. — [fonte 2026-08-27, riga 232](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf. — [fonte 2026-08-27, riga 341](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).
- [ ] Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1. — [fonte 2026-08-27, riga 337](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).
- [ ] Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint. — [fonte 2026-08-28, riga 250](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- [ ] Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati. — [fonte 2026-08-28, riga 299](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](../user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).
- [ ] **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](../user/2026-08-24_v26b_rev4e_r2_replay.md).
- [ ] **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).
- [ ] **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).
- [ ] **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](../user/2026-08-25_v26b_b1r2_diagnostica.md).
- [ ] **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).
- [ ] **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).
- [ ] **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).
- [ ] **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. — [fonte 2026-08-26, riga 184](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Verificare retrospettivamente l’equivalenza semantica del teacher prescritto sotto v3 nell’audit della collection storica. — [fonte 2026-08-26, riga 178](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Attestare il gate della collection teacher storica sotto v3 e corridoio corrente con la relativa evidenza. — [fonte 2026-08-26, riga 179](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop. — [fonte 2026-08-27, riga 256](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).
- [ ] **Condizionato:** Eventuale integrazione/riapertura dei rami LOTO, LOCO, B1R1/B1R2 con protocollo e autorizzazione; conservarne i verdetti storici, senza considerarli già operativi. — [fonte 2026-08-27, riga 343](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18. — [fonte 2026-08-27, riga 265](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato. — [fonte 2026-08-27, riga 376](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).
- [ ] **Condizionato:** Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione. — [fonte 2026-08-27, riga 267](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale. — [fonte 2026-08-27, riga 227](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati. — [fonte 2026-08-27, riga 192](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- [ ] **Condizionato:** Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding. — [fonte 2026-08-27, riga 332](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] **Condizionato:** Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori. — [fonte 2026-08-27, riga 247](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- [ ] **Condizionato:** Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy. — [fonte 2026-08-27, riga 431](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente. — [fonte 2026-08-27, riga 212](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati. — [fonte 2026-08-27, riga 205](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).
- [ ] **Vincolo:** Conservare immutati sidecar J8 e artefatti storici. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).
- [ ] **Condizionato:** La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento. — [fonte 2026-08-27, riga 435](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente. — [fonte 2026-08-27, riga 241](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] **Condizionato:** Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati. — [fonte 2026-08-28, riga 276](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- [ ] **Condizionato:** Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget. — [fonte 2026-08-28, riga 1059](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training. — [fonte 2026-08-28, riga 1029](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Rivalutare sigma dopo actor predittivo o modifica causale del morphology corridor. — [fonte 2026-08-28, riga 1055](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Completare il milestone di recupero AB06 qualificato e attestato come training-input-only. La ricostruzione J19–J21 sostituisce la vecchia proposta F0/F1–F7, senza dichiarare eseguite tutte quelle fasi né il benchmark generale di parità. — [fonte 2026-08-23, riga 200](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md). **Chiuso:** J21 attesta training_ready e TRAINING_INPUT_ONLY con 18/18, senza nuove esecuzioni. ([evidenza](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md)).
- [x] Rimanere senza actor-updating PPO finché non sono chiusi restore e attestazione; il training successivo richiede autorizzazione distinta. — [fonte 2026-08-27, riga 326](../user/2026-08-27_v26c_j12_closed_loop_qualification_execution.md). **Superato:** Dopo separata autorizzazione utente, training da J20 per 50 nuove iterazioni; il vincolo preparatorio non è un divieto permanente. ([evidenza](2026-08-28_daily-report.md)).
- [x] Decisione sulla causa del restore fallito e nuovo stadio/GO, preservando il leaf precedente senza retry in-place. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j20_restore_audit_esecuzione.md). **Chiuso:** R3 restore PASS 13/13 e G9 chiuso a zero update. ([evidenza](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md)).
- [x] Eseguire restore audit reale a zero iterazioni e chiudere G9; R1/R2 restano fallimenti storici, R3 è la prova positiva. — [fonte 2026-08-27, riga 257](../user/2026-08-27_v26c_j20_critic_warmup_execution.md). **Chiuso:** R3 restore PASS 13/13 e G9 chiuso a zero update. ([evidenza](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md)).
- [x] Valutare eventuali correzioni preparatorie senza riscrivere rev 1/rev 2. — [fonte 2026-08-27, riga 225](../user/2026-08-27_v26c_j20_restore_audit_preparazione.md). **Superato:** R2 eseguito col rimedio PYTHONPATH: vero restore raggiunto, FAIL 7/13 per rappresentazione. ([evidenza](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md)).
- [x] Revisione/autorizzazione e GO separato per R2. — [fonte 2026-08-27, riga 271](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md). **Chiuso:** R2 eseguito col rimedio PYTHONPATH: vero restore raggiunto, FAIL 7/13 per rappresentazione. ([evidenza](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md)).
- [x] Revisionare l’aggregatore J21 e ottenere GO separato/doppia autorizzazione per una sola attestazione. — [fonte 2026-08-28, riga 258](../user/2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md). **Chiuso:** J21 attesta training_ready e TRAINING_INPUT_ONLY con 18/18, senza nuove esecuzioni. ([evidenza](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md)).
- [x] Revisione di rev 7 e GO approvato con autorizzazione utente per R3. — [fonte 2026-08-28, riga 297](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md). **Chiuso:** R3 restore PASS 13/13 e G9 chiuso a zero update. ([evidenza](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md)).
- [x] Progettare/autorizzare/eseguire l’attestazione separata che porta training_ready a vero dopo R3, senza addestrare. — [fonte 2026-08-28, riga 300](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md). **Chiuso:** J21 attesta training_ready e TRAINING_INPUT_ONLY con 18/18, senza nuove esecuzioni. ([evidenza](../user/2026-08-28_v26c_j21_training_ready_attestation_esecuzione.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
