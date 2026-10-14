# Daily Report - 2026-08-16

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md`
- `2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense.md`
- `2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron.md`
- `2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass.md`
- `2026-08-16_h0_v12r16_p3_verdetto_plus020_pass_seed127_fail.md`
- `2026-08-16_training_esplorativo_exnovo_v26_corridor_h0ws_abort_iter31.md`

## Sintesi

Giornata della saga one-shot V12: applicati e verificati i fix pre-freeze di
V12R12, poi eseguite in sequenza le lineage R12→R16. Prime storiche della
filiera: P0 PASS (R13, dopo la ricalibrazione dei gate offline mai
dry-fit-validati), P1 PASS (R15, 8.404 righe, dopo il bug producer→verifier
di `safety_stop_count` scoperto in R14), catena offline completa
deterministica (R16) e **`+0.20` puro 500/500 PASS** (R16); seed 127 resta
terminale (FAIL a step 202). In parallelo, il training esplorativo ex-novo
con V26 + corridor e warm-start H0 (50 iter, nessun nuovo yaml) è
**abortito a 31/50**: H0 legacy collassa in 0,4–1,7 s sotto V26 (eventi
invalidi worker-fatali + guard di interleaving) — evidenza che l'adattamento
della baseline legacy a V26 è un problema di recovery fuori supporto.

## Lavoro svolto

- Ricalibrazione gate P0 su metriche riprodotte deterministicamente e
  persistenza di `fit_metrics.json` prima dei gate (R13).
- Gate P1 a due verdetti tollerante alla troncatura (R14) e check
  `safety_stop_semantics` a sorgente produttore con test byte-real (R15).
- Gate P2 calibrati sul dry-fit persistito (R16); matrice P3 eseguita.
- Workaround Errno 63 per `--reward-json` inline (file JSON), poi fix in
  `_load_reward_json_for_run_name`.
- Commit `218492d3` (lineage R13–R16) e `cb62b56` (.gitattributes v12 +
  OSError guard).

## Test e verifiche

- Suite v12r12 82/82 pre-freeze; receipt/ledger letti dagli artefatti reali;
  riproduzione bit-exact della catena offline tra lineage.

## TODO completati

- [x] Fix pre-freeze V12R12 applicati, testati, verificati avversarialmente.
- [x] Freeze + lock + sequenza one-shot eseguita (R12→R16).
- [x] Correzione `--reward-json` inline (Errno 63) in `train_ppo_mlp.py`.
- [x] Commit checkpoint R13–R16 e fix `.gitattributes`.

## TODO aperti e propagati

- [ ] Decisione utente su R17/R18 (matrice P3 a 5 casi con seed 127
  known-open; fresh-init W256) — vedi daily 17/08.
- [ ] Risultati della run esplorativa da trattare come diagnostici (nessuna
  promozione).
- [ ] Amministrativo (dal 15/08): riconciliare il pin V20 del modulo
  residuale in `train_ppo_mlp.py` con la lineage V26; chiarire l'anomalia
  best==last==warm_start nel tree del teacher H0 retry; chiarire la metrica
  offline che bocciò `fit_p2` V12R3; TODO storico sulla deflessione SEA
  iniziale coerente con la coppia richiesta.

## Integrazione documentale retrospettiva — 2026-10-05

Queste integrazioni consolidano risultati e TODO dei report utente già esistenti. La data di registrazione è il 5 ottobre 2026: non sono nuove simulazioni, test, training o modifiche del codice. Gli esiti terminali storici restano distinti dai risultati dei successori.

<a id="integrazione-2026-10-05-2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense"></a>

### 2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense

Fonte: [2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense](../user/2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense.md).

La prima execution R12 supera preflight/freeze ma termina al fitP0 con **7/7check metrici FAIL**, prima di qualsiasi reset/step ambiente. La forense riproduce deterministicamente RMSE0.004531, max0.05541 e reset0.000222, valori non persistiti nel canonico prima del gate. Le nuove soglie, fino a18–23volte più strette, non erano state validate con dry-fit; il costo della mask18 è moderato e quei valori avrebbero passato il contratto storico. Il risultato non dimostra impossibilità delle feature invarianti.

R13 resta una nuova lineage: stesso design/data, gate calibrati indipendentemente sui valori riprodotti e `fit_metrics.json` persistito prima della valutazione. Il successivo P0PASS R13 documenta il rimedio, non trasforma R12 in PASS. Gli stage downstream non vengono eseguiti su R12 e il pivot17/08 supera la vecchia catena.

<a id="integrazione-2026-10-05-2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron"></a>

### 2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron

Fonte: [2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron](../user/2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron.md).

R13 ottiene il primo P0PASS dopo ricalibrazione/persistenza, con82test e valori riprodotti byte-exact. P1 però è **FAIL al rollout11/18**, alpha0.50/seed127:202step, penetrazione25.057668mm, appena57µm oltre il limite. Dieci collection precedenti sono complete; non è un errore di detector o una nuova qualification riuscita.

Su5202righe il trigger p99 dello shield non si attiva: la distanza nelle18feature cinematiche è cieca al rischio di penetrazione. Il tape/regime ha circa1mm di margine fisico; il completamento protetto non basta a dimostrare autonomia. Il successoreR14 deve tollerare prefissi fisicamente validi e valutare l’eventuale dimensione penetrazione nell’envelope. Il ramo viene poi consumato/superato da R14–R17 e dal pivot nativo; non si rilancia R13.

<a id="integrazione-2026-10-05-2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass"></a>

### 2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass

Fonte: [2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass](../user/2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass.md).

La progressione contiene tre esiti distinti: R13P1FAIL per57µm; R14P1FAIL spurio perché `safety_stop_count=1` descriveva correttamente lo stop fisico del produttore ma il verifier lo pretendeva zero; R15 corregge questo contratto con fixture byte-real e ottiene il primo P1PASS,8.404righe da16collection complete e due prefissi troncati.

R15 non passa però il fitP2: **due gate di coda FAIL**, valori0.005596/0.006841 contro soglia0.0055. Le soglie derivavano dal corpusP0 e non dal P2 arricchito; R16 le ricalibra sul dry-fit persistito prima del giudizio fisico, senza ritentare R15. Test producer→verifier, bit-identità e commit218492d3 sono verifiche di questa progressione. R16P3 e decisioni17/08 chiudono il follow-up; il suo successo non cancella le failure intermedie.

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Riferimento storico al TODO giugno sulla scelta warm-start: luglio ha usato il 23 giugno grfsoft, non il 24 giugno. — [fonte 2026-08-25, riga 40](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md).
- [ ] Solo dopo tale freeze, eseguire prima le sei coppie A/B e, se tutte passano, i sei casi C. — [fonte 2026-08-05, riga 144](../user/2026-08-05_protocollo_h0_v25_abc_congelato.md).
- [ ] Mantenere H0_sep, trial protetti, corridor positivo e PPO chiusi fino ai gate rispettivi. — [fonte 2026-08-05, riga 232](../user/2026-08-05_binary_active_v25_v20_adapter_h0.md).
- [ ] Mantenere corridor, reward positivo e PPO chiusi fino ai rispettivi gate. — [fonte 2026-08-05, riga 147](../user/2026-08-05_protocollo_h0_v25_abc_congelato.md).
- [ ] Eseguire la prova A/B bit-exact prevista dal protocollo congelato, prima con V20 disabilitata e poi in shadow. — [fonte 2026-08-05, riga 226](../user/2026-08-05_binary_active_v25_v20_adapter_h0.md).
- [ ] Solo dopo il gate H0 decidere l'apertura one-shot dei trial protetti 05/06. — [fonte 2026-08-05, riga 146](../user/2026-08-05_protocollo_h0_v25_abc_congelato.md).
- [ ] Eseguire il caso C `binary_active` soltanto dopo il PASS A/B e con le autorità previste dal protocollo. — [fonte 2026-08-05, riga 228](../user/2026-08-05_binary_active_v25_v20_adapter_h0.md).
- [ ] Definire un nuovo gate baseline A per il candidato adattato e un nuovo percorso A/B/C soltanto dopo avere ottenuto una baseline valida. — [fonte 2026-08-05, riga 229](../user/2026-08-05_preflight_h0_v25_abc_error_h0_reference.md).
- [ ] Mantenere B, C, H0_sep, trial 05/06, corridor positivo e PPO chiusi fino ai rispettivi gate. — [fonte 2026-08-05, riga 231](../user/2026-08-05_preflight_h0_v25_abc_error_h0_reference.md).
- [ ] Mantenere morphology reward positivo, corridor training e PPO chiusi fino a `H0_TRAINING_READY` reale. — [fonte 2026-08-07, riga 290](../user/2026-08-07_v26_v8r1p1_teacher_pass_h0_p0_fail.md).
- [ ] Dopo tutti i gate: eseguire zero-update save/reload e creare il vero `checkpoint_zero`; solo allora pubblicare il comando di warm-start. — [fonte 2026-08-07, riga 288](../user/2026-08-07_v26_v8r1p1_teacher_pass_h0_p0_fail.md).
- [ ] Congelare ed eseguire la qualification indipendente del candidato. — [fonte 2026-08-07, riga 285](../user/2026-08-07_v26_v8r1p1_teacher_pass_h0_p0_fail.md).
- [ ] Richiedere autorizzazione separata prima di aprire one-shot trial 05 e, solo dopo PASS, trial 06. — [fonte 2026-08-07, riga 286](../user/2026-08-07_v26_v8r1p1_teacher_pass_h0_p0_fail.md).
- [ ] Solo dopo un nuovo P0 PASS: eseguire due DAgger completi da 500 step, fit P1 e sei rollout development. — [fonte 2026-08-07, riga 283](../user/2026-08-07_v26_v8r1p1_teacher_pass_h0_p0_fail.md).
- [ ] Mantenere morphology reward positivo, corridor training e PPO chiusi finché non esiste `H0_TRAINING_READY` scientifico. — [fonte 2026-08-09, riga 332](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Eseguire zero-update save/reload, produrre un vero `checkpoint_zero` e pubblicare il comando warm-start soltanto dopo la qualification. — [fonte 2026-08-09, riga 330](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Mantenere V26, GRF primaria, SEA, morphology reward positivo, corridor training e PPO intoccati fino a un `H0_TRAINING_READY` reale. — [fonte 2026-08-09, riga 383](../user/2026-08-09_h0_v11_weighted_full_mean_fail_closed_loop.md).
- [ ] Ottenere tutti i fit, pure probe, label, round di raccolta e sei rollout autonomi development PASS con almeno due cicli, zero safety stop e non-regressione SEA/reserve. — [fonte 2026-08-09, riga 324](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Solo dopo tutti i gate: eseguire zero-update save/reload, produrre il vero `checkpoint_zero` e pubblicare il comando di warm-start. — [fonte 2026-08-09, riga 381](../user/2026-08-09_h0_v11_weighted_full_mean_fail_closed_loop.md).
- [ ] Solo dopo sei rollout autonomi PASS: eseguire V10Q/qualification, poi richiedere autorizzazione separata per i trial protetti 05 e 06. — [fonte 2026-08-09, riga 379](../user/2026-08-09_h0_v11_weighted_full_mean_fail_closed_loop.md).
- [ ] Eseguire V12R2 una sola volta con il lock corrente; non ritentare in caso di failure terminale. — [fonte 2026-08-09, riga 322](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Solo dopo un PASS development, eseguire V10Q/qualification indipendente. — [fonte 2026-08-09, riga 327](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Aprire trial 05 e, soltanto dopo il relativo PASS, trial 06; mantenere 03/07 chiusi come reserve. — [fonte 2026-08-09, riga 328](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] portare Q3 su R9 ed eseguirlo sullo stesso candidato; — [fonte 2026-08-14, riga 176](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md).
- [ ] solo allora pubblicare freeze/lock ed eseguire una sola volta le 12 rollout fisiche; — [fonte 2026-08-14, riga 180](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md).
- [ ] Eseguire e superare la qualification positiva separata `v12r8morph`. — [fonte 2026-08-14, riga 119](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).
- [ ] Eseguire una sola volta il runtime V12R8zero e ottenere il receipt PASS del full `checkpoint_zero` a progresso zero. — [fonte 2026-08-14, riga 117](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).
- [ ] Solo dopo il PASS morfologico, pubblicare ed eseguire il training da 50 update tramite `--resume-from checkpoint_zero`. — [fonte 2026-08-14, riga 120](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).
- [ ] abilitare morphology positiva esclusivamente nel protocollo causale dedicato, dopo la validazione a peso zero. — [fonte 2026-08-14, riga 143](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md).
- [ ] Eseguire l'A/B preregistrata del Morphology Corridor con il primo peso positivo autorizzato e pubblicare la config training solo dopo PASS. — [fonte 2026-08-14, riga 111](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md).
- [ ] Attendere e verificare il terminal PASS V12R8-Q3 sullo stesso candidato. — [fonte 2026-08-14, riga 121](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md).
- [ ] Creare `v12r6q3` soltanto dopo candidato e sei development PASS R6. — [fonte 2026-08-14, riga 84](../user/2026-08-14_q3_runtime_source_only_fail_closed.md).
- [ ] creare il checkpoint-zero actor-only con critic 512 fresco; — [fonte 2026-08-14, riga 162](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md).
- [ ] dopo R8 `PASS`, eseguire una sola volta freeze, tape materialization e qualifica V12R8-Q3 con la sequenza preregistrata; — [fonte 2026-08-14, riga 141](../user/2026-08-14_h0_v12r8_q3_source_closed.md).
- [ ] creare e validare il checkpoint-zero imitativo; — [fonte 2026-08-14, riga 109](../user/2026-08-14_h0_v12r7_prefix_projection_terminal_fail.md).
- [ ] solo dopo Q3 terminale `PASS`, definire ed eseguire un protocollo separato per checkpoint-zero/warm-start; — [fonte 2026-08-14, riga 143](../user/2026-08-14_h0_v12r8_q3_source_closed.md).
- [ ] Eseguire Q3 indipendente, checkpoint-zero e Morphology Corridor A/B soltanto dopo un terminal PASS imitativo del successore. — [fonte 2026-08-14, riga 128](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md).
- [ ] pubblicare il comando finale da `--resume-from` soltanto dopo tutti i PASS. — [fonte 2026-08-14, riga 111](../user/2026-08-14_h0_v12r7_prefix_projection_terminal_fail.md).
- [ ] conservare il path esatto dei log Ray della sessione di training; — [fonte 2026-08-14, riga 213](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] dopo R9 PASS, eseguire Q3 separato e poi il protocollo checkpoint-zero; — [fonte 2026-08-14, riga 142](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md).
- [ ] mantenere Q3 e checkpoint-zero chiusi in caso di terminal `FAIL`; — [fonte 2026-08-14, riga 141](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md).
- [ ] eseguire una sola volta il runner R9 e richiedere ledger terminale `PASS`; — [fonte 2026-08-14, riga 140](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md).
- [ ] completare l'additive successor R9 e ottenerne il terminal PASS senza modificare o rieseguire R8; — [fonte 2026-08-14, riga 174](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md).
- [ ] Abilitare una reward morphology positiva soltanto con un protocollo A/B successivo, separato e autorizzato. — [fonte 2026-08-14, riga 104](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md).
- [ ] Dopo Q3 PASS, creare un protocollo checkpoint-zero separato; non incorporare checkpoint o update nella qualification Q3. — [fonte 2026-08-14, riga 102](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md).
- [ ] Non ritentare o promuovere V12R9. — [fonte 2026-08-14, riga 350](../user/2026-08-14_h0_v12r10_fit_forensics.md).
- [ ] eseguire una sola volta il preflight 12-EnvRunner e ottenere il receipt PASS; — [fonte 2026-08-14, riga 211](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] Attendere e verificare V12R8-zero e il full checkpoint `checkpoint_zero` sullo stesso candidate/tree/actor binding. — [fonte 2026-08-14, riga 122](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md).
- [ ] mantenere Q3 chiuso se V12R9 non termina con ledger `PASS` e candidato completo a cinque file; — [fonte 2026-08-14, riga 124](../user/2026-08-14_h0_v12r9_q3_source_ready_deferred.md).
- [ ] Dopo un PASS imitativo: congelare il candidato, eseguire Q3, creare e validare checkpoint-zero con detector V26 attivo, quindi svolgere l'A/B positivo del Morphology Corridor. — [fonte 2026-08-14, riga 106](../user/2026-08-14_h0_v12r5_case_balanced_fit_terminal_fail.md).
- [ ] ottenere i terminal PASS della pipeline V12R9, della Q3, del checkpoint-zero e della qualification morphology; — [fonte 2026-08-14, riga 210](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] Dopo terminal PASS morphology, ottenere il preflight V12R9 training su 12 EnvRunner, avviare il resume da 50 update e produrre l'audit post-run obbligatorio. — [fonte 2026-08-14, riga 163](../user/2026-08-14_h0_v12r9_morphology_source_ready_deferred.md).
- [ ] Mantenere chiuso `v12r6q3`: detector V26 live, checkpoint-zero e Morphology Corridor restano subordinati a un futuro terminal PASS imitativo. — [fonte 2026-08-14, riga 117](../user/2026-08-14_h0_v12r6_functional_composite_terminal_fail.md).
- [ ] Solo dopo i prerequisiti, congelare ed eseguire una singola qualification positiva; pubblicare il comando da 50 update esclusivamente su PASS. — [fonte 2026-08-14, riga 128](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md).
- [ ] Solo allora pubblicare freeze e lock ed eseguire una volta le 12 rollout fisiche morphology. — [fonte 2026-08-14, riga 161](../user/2026-08-14_h0_v12r9_morphology_source_ready_deferred.md).
- [ ] dopo PASS R8, eseguire Q3 weight=0 con detector V26 attivo; — [fonte 2026-08-14, riga 108](../user/2026-08-14_h0_v12r7_prefix_projection_terminal_fail.md).
- [ ] soltanto dopo Q3 terminale `PASS`, passare al protocollo separato checkpoint-zero/warm-start; — [fonte 2026-08-14, riga 126](../user/2026-08-14_h0_v12r9_q3_source_ready_deferred.md).
- [ ] Non aprire gli output canonici Q3 V12R5. — [fonte 2026-08-14, riga 87](../user/2026-08-14_q3_runtime_source_only_fail_closed.md).
- [ ] In una nuova revisione immutabile, collegare path, status e validator terminali definitivi dei tre upstream senza indovinarli. — [fonte 2026-08-14, riga 124](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md).
- [ ] Solo dopo entrambi i PASS, congelare una volta il lock V12R8zero. — [fonte 2026-08-14, riga 116](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).
- [ ] Attendere e verificare il terminal PASS V12R8. — [fonte 2026-08-14, riga 120](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md).
- [ ] se R7 passa, eseguire Q3 V26/corridor shadow; — [fonte 2026-08-14, riga 161](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md).
- [ ] mantenere la morphology positiva fuori da Q3 e validarla nel protocollo causale dedicato. — [fonte 2026-08-14, riga 128](../user/2026-08-14_h0_v12r9_q3_source_ready_deferred.md).
- [ ] validare la morphology reward con peso positivo esclusivamente in una fase successiva separata: questa milestone valida integrazione e causalità a peso zero, non autorizza ancora effetti positivi sulla reward. — [fonte 2026-08-14, riga 145](../user/2026-08-14_h0_v12r8_q3_source_closed.md).
- [ ] In caso di terminal PASS R7, completare, congelare ed eseguire Q3. — [fonte 2026-08-14, riga 108](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md).
- [ ] se e solo se il terminale morphology R9 è PASS, usare l'handoff pubblicato per il resume di 50 update dal checkpoint-zero. — [fonte 2026-08-14, riga 182](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md).
- [ ] Eseguire freeze, tape preparation e qualification fisica soltanto nel nuovo namespace V12R8-Q3. — [fonte 2026-08-14, riga 100](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md).
- [ ] Valutare la capability reale delle trace Q3: riuso di sei control se sufficiente, altrimenti fallback esplicito a dodici rollout locali. — [fonte 2026-08-14, riga 126](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md).
- [ ] Dichiarare training-ready soltanto dopo il preflight reale 0 -> 12 EnvRunner e avviare il training da 50 update attraverso il launcher morphology. — [fonte 2026-08-14, riga 130](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md).
- [ ] Non ritentare, modificare o promuovere V12R9. — [fonte 2026-08-14, riga 125](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md).
- [ ] Ottenere e verificare il terminal PASS V12R8-Q3 sullo stesso candidate ID e sullo stesso tree SHA-256. — [fonte 2026-08-14, riga 114](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).
- [ ] eseguire l'audit post-run e richiedere il relativo PASS prima di qualsiasi qualification downstream; — [fonte 2026-08-14, riga 214](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] Conservare la matrice baseline-first, detector V26 obbligatorio e morphology a peso zero nella qualification R6. — [fonte 2026-08-14, riga 85](../user/2026-08-14_q3_runtime_source_only_fail_closed.md).
- [ ] avviare il comando canonico dei 50 update solo dopo quel receipt; — [fonte 2026-08-14, riga 212](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] Attendere terminal PASS V12R9 recovery, V12R9-Q3 e V12R9 checkpoint-zero sulla stessa identità candidato/checkpoint. — [fonte 2026-08-14, riga 157](../user/2026-08-14_h0_v12r9_morphology_source_ready_deferred.md).
- [ ] Ottenere e verificare il terminal PASS V12R8. — [fonte 2026-08-14, riga 113](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).
- [ ] portare checkpoint-zero su R9/Q3 e verificarne il terminal PASS; — [fonte 2026-08-14, riga 177](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md).
- [ ] eseguire A/B del Morphology Corridor con peso positivo `0.0025`; — [fonte 2026-08-14, riga 110](../user/2026-08-14_h0_v12r7_prefix_projection_terminal_fail.md).
- [ ] Mantenere Q3, checkpoint-zero e Morphology Corridor chiusi fino al terminal PASS fisico di V12R10. — [fonte 2026-08-14, riga 351](../user/2026-08-14_h0_v12r10_fit_forensics.md).
- [ ] validare identity A/B del corridor a peso zero e pubblicare il comando training definitivo. — [fonte 2026-08-14, riga 163](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md).
- [ ] Non ritentare e non promuovere V12R6. — [fonte 2026-08-14, riga 111](../user/2026-08-14_h0_v12r6_functional_composite_terminal_fail.md).
- [ ] Rieseguire l'audit ABI/hash del namespace morphology contro gli artefatti terminali definitivi. — [fonte 2026-08-14, riga 159](../user/2026-08-14_h0_v12r9_morphology_source_ready_deferred.md).
- [ ] Attendere gli endpoint immutabili V12R8 e creare `v12r8q3` come nuova lineage; non clonare o congelare parzialmente prima del terminal PASS R8. — [fonte 2026-08-14, riga 98](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md).
- [ ] Creare il checkpoint-zero full RLlib 2x512 con actor imitativo esatto e critic/optimizer/counter freschi. — [fonte 2026-08-14, riga 109](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md).
- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).
- [ ] Mantenere Q3, checkpoint-zero e Morphology Corridor chiusi finche il nuovo successore V26-puro non ottiene un terminal PASS fisico completo. — [fonte 2026-08-15, riga 250](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Conservare V12R10 chiuso al suo terminal FAIL, senza retry, resume, promozione o completamento dei cinque development successivi. — [fonte 2026-08-15, riga 238](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Q3, checkpoint-zero e Morphology Corridor restano chiusi fino al terminal PASS fisico P3. — [fonte 2026-08-15, riga 181](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md).
- [ ] Materializzare le label V26-pure per l'audit offline, pubblicare soltanto se eleggibile la copia core byte-exact dell'H0 W256 e rieseguire da zero le qualification fisiche fail-closed, senza usare un nuovo fit o riutilizzare il candidato V12R10 come predecessore promosso. — [fonte 2026-08-15, riga 245](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Definire e congelare in un namespace successore il relabel stateless e byte-deterministico dell'esatta osservazione V26-35; non introdurre uno shadow legacy online. — [fonte 2026-08-15, riga 240](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Conservare l'esito dell'audit di osservabilita come rischio e gate esplicito della qualification; l'audit e concluso, ma l'alias non e dichiarato risolto dal solo fit offline. — [fonte 2026-08-15, riga 235](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Soltanto dopo tale PASS, eseguire in ordine Q3, checkpoint-zero e la qualification del Morphology Corridor sul candidato successore congelato, mantenendo tutte le superfici fail-closed. — [fonte 2026-08-15, riga 252](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Solo dopo tali gate, congelare il protocollo training con warm-start imitativo, detector binario e corridor attivi; completare il restore/preflight su 12 EnvRunner prima di avviare e monitorare le 50 update previste. — [fonte 2026-08-15, riga 255](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Non dichiarare il sistema training-ready finche development, Q3, checkpoint-zero, Morphology Corridor e preflight training non sono tutti terminal PASS sullo stesso candidato successore congelato. — [fonte 2026-08-15, riga 258](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Autorizzare la lineage successore (V12R14) con l'opzione scelta (raccomandata: collection tolerante ai prefissi, importando le 10 collection PASS e il prefisso forense di R13). — [fonte 2026-08-16, riga 111](../user/2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron.md).
- [ ] In caso di (C): nuova lineage con matrice di development dichiarata, poi port filiera Q3 → checkpoint-zero → morphology → preflight → training. — [fonte 2026-08-16, riga 98](../user/2026-08-16_h0_v12r16_p3_verdetto_plus020_pass_seed127_fail.md).
- [ ] Se P3 PASS: port Q3 → checkpoint-zero → morphology → preflight → training (filiera già costruita). — [fonte 2026-08-16, riga 110](../user/2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass.md).
- [ ] Dopo un P1 completo: P2 → P3 (il discriminatore fisico resta il punto aperto) → Q3 → checkpoint-zero → morphology → training. — [fonte 2026-08-16, riga 117](../user/2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron.md).
- [ ] Dopo un eventuale PASS P3: port Q3 → checkpoint-zero → morphology → preflight 12 EnvRunner → training 50 update (percorso già costruito). — [fonte 2026-08-16, riga 120](../user/2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense.md).
- [ ] Decisione dell'utente tra le opzioni (A)/(C) — o loro combinazione (dry-fit A per informare C). — [fonte 2026-08-16, riga 96](../user/2026-08-16_h0_v12r16_p3_verdetto_plus020_pass_seed127_fail.md).
- [ ] fitter: `--write-protocol-freeze` / `--write-execution-lock` con `--acknowledge-governance V12R12_FREEZE_REVIEWED_SOURCES`, poi `--execute --stage p0 --acknowledge-one-shot V12R12_ONE_SHOT_FIXED_FIT`; — [fonte 2026-08-16, riga 172](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Se P3 FAIL: forense (journal completi garantiti) e decisione sul successore con dati recovery aggiuntivi — o de-scope dichiarata. — [fonte 2026-08-16, riga 112](../user/2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass.md).
- [ ] In V12R14 valutare anche l'estensione dell'envelope di supporto con la dimensione di penetrazione (lo shield cinematico è cieco al rischio reale — dato nuovo di questa run). — [fonte 2026-08-16, riga 114](../user/2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron.md).
- [ ] Valutare, nel design del successore, il dato nuovo di questa run: la fragilità dipende dalla fase di partenza (nominale peggiore del `+0,20` in condizioni di training) — i casi di development/Q3 coprono già start diversi, ma il P1 candidate-exposed potrebbe pesare anche il nominale. — [fonte 2026-08-16, riga 124](../user/2026-08-16_training_esplorativo_exnovo_v26_corridor_h0ws_abort_iter31.md).
- [ ] P3: `--prepare-lock` (ack `V12R12_P3_FREEZE_AFTER_P2_PASS`) solo dopo il PASS P2, poi `--execute` (ack `V12R12_P3_ONE_SHOT_NO_RETRY`) col caso `+0.20` per primo. — [fonte 2026-08-16, riga 179](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] In caso di FAIL fisico P3, chiudere la lineage senza retry e valutare il successore con dati recovery aggiuntivi, come da protocollo. — [fonte 2026-08-16, riga 190](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Riprendere il percorso qualificato: autorizzare V12R13 (design V12R12, gate offline ricalibrati sui valori dimostrati, metriche persistite pre-gate) e portarla fino al P3 fisico. — [fonte 2026-08-16, riga 121](../user/2026-08-16_training_esplorativo_exnovo_v26_corridor_h0ws_abort_iter31.md).
- [ ] fitter `--execute --stage p2`; — [fonte 2026-08-16, riga 178](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] P1: freeze/lock con `--acknowledge-governance V12R12_P1_FREEZE_REVIEWED_SOURCES`, poi `--execute --acknowledge-one-shot V12R12_P1_ONE_SHOT_18_ROLLOUTS_9000_STEPS`; — [fonte 2026-08-16, riga 175](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Eseguirne la sequenza one-shot fino a P3: la domanda aperta — stabilità fisica a `+0.20` con feature invarianti + recovery candidate-exposed — si decide lì. — [fonte 2026-08-16, riga 117](../user/2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense.md).
- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Rieseguire la suite completa e un re-audit dei punti toccati — vedi addendum. — [fonte 2026-08-15, riga 176](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md)).
- [x] Aggiungere i test d'integrazione producer→verifier — vedi addendum. — [fonte 2026-08-15, riga 175](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md)).
- [x] Freeze + lock P0/P2 (ack `V12R12_FREEZE_REVIEWED_SOURCES`) ed esecuzione one-shot `--execute --stage p0` secondo la sequenza preregistrata nei runner. — [fonte 2026-08-15, riga 178](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md). **Chiuso:** Freeze ed execution R12→R16 eseguiti, con stage terminali distinti. ([evidenza](2026-08-16_daily-report.md)).
- [x] Applicare i fix pre-freeze (3 major + minor elencati) — vedi addendum. — [fonte 2026-08-15, riga 174](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md)).
- [x] Trattare i risultati della run come diagnostici: checkpoint e modulo `rl_module_best/last` non sono candidati e non vanno promossi né riusati come warm-start. — [fonte 2026-08-16, riga 128](../user/2026-08-16_training_esplorativo_exnovo_v26_corridor_h0ws_abort_iter31.md). **Chiuso:** Run esplorativa trattata come diagnostica, senza promozione. ([evidenza](2026-08-16_daily-report.md)).
- [x] Completare R16: P1 → P2 (deterministici) → **P3** (verdetto fisico). — [fonte 2026-08-16, riga 109](../user/2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass.md). **Chiuso:** R16 P0→P2 PASS e P3 eseguito: +0.20 PASS, seed127 FAIL. ([evidenza](2026-08-16_daily-report.md)).
- [x] A valle, commit checkpoint del lavoro R13–R16. — [fonte 2026-08-16, riga 114](../user/2026-08-16_h0_v12r13_r14_r15_progressione_p0_p1_pass.md). **Chiuso:** Commit218492d3 e cb62b56, inclusa .gitattributes, documentati. ([evidenza](2026-08-16_daily-report.md)).
- [x] Commit checkpoint del lavoro09–15/08 con correzione .gitattributes. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md). **Chiuso:** Commit218492d3 e cb62b56, inclusa .gitattributes, documentati. ([evidenza](2026-08-16_daily-report.md)).
- [x] Commit checkpoint R13–R16. — [fonte 2026-08-16, riga 101](../user/2026-08-16_h0_v12r16_p3_verdetto_plus020_pass_seed127_fail.md). **Chiuso:** Commit218492d3 e cb62b56, inclusa .gitattributes, documentati. ([evidenza](2026-08-16_daily-report.md)).
- [x] Correggere in `train_ppo_mlp.py` la gestione di `--reward-json` inline lungo (catch di `OSError` in `_load_reward_json_for_run_name`). — [fonte 2026-08-16, riga 119](../user/2026-08-16_training_esplorativo_exnovo_v26_corridor_h0ws_abort_iter31.md). **Chiuso:** Workaround su file e fix OSError del loader reward-json documentati. ([evidenza](2026-08-16_daily-report.md)).
- [x] Autorizzare ed eseguire il freeze e la sequenza one-shot P0 → P1 → P2 → P3. — [fonte 2026-08-16, riga 188](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md). **Chiuso:** Freeze ed execution R12→R16 eseguiti, con stage terminali distinti. ([evidenza](2026-08-16_daily-report.md)).
- [x] Autorizzare la lineage V12R13 (design invariato, gate ricalibrati e dry-fit-validati, persistenza metriche pre-gate). — [fonte 2026-08-16, riga 115](../user/2026-08-16_h0_v12r12_p0_fit_terminal_fail_forense.md). **Chiuso:** R13 autorizzata/eseguita: P0 PASS, successiva P1 FAIL storica. ([evidenza](2026-08-16_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
