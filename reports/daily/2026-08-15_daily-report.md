# Daily Report - 2026-08-15

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [V12R10 — protocollo canonico, fit W1024 e terminal FAIL](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md)
- [V12R11 — probe H0 diretto su V26, terminal FAIL](../user/2026-08-15_h0_v12r11_direct_v26_terminal_fail.md)
- [Retrospettiva forense — sessione notturna 09–10/08: V12R2 → V12R4](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md)
- [V12R12 — stato source-only, test PASS e review pre-freeze](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md)

## Sintesi

Giornata in due parti: la mattina/primo pomeriggio ha chiuso la diagnosi del
blocco imitativo con due esecuzioni discriminanti (V12R10 e V12R11, entrambe
terminal FAIL ma con esito scientifico netto); il pomeriggio ha aperto la
lineage successore V12R12 (source-only); la sessione successiva ha condotto
l'audit dello stato reale del repo, colmato il debito documentale
(daily 10/08, 14/08, retrospettiva 09–10/08) ed eseguito test + review
avversariale pre-freeze di V12R12.

### V12R10 — il fit non era il blocker sufficiente

Lineage import-only W1024 gate-aligned: fit production PASS, candidato
congelato (`AB06_H0_V12R10_RECOVERY_W1024:dcae7d71…`), ma **FAIL al primo
development** `+0.20`: 212 step, rampa di penetrazione 12,8→26,7 mm in
0,10 s, più 4 clipping dell'azione 0 (gate indipendente fallito). Estende di
33 step l'orizzonte di R6 senza chiudere il rischio nella stance post-HS.
Ledger `FAIL_H0_V12R10_RECOVERY_PIPELINE_TERMINAL`, 3 stage completati,
`STOP_TERMINAL_NO_RETRY`.

### V12R11 — il discriminatore: H0 diretto su V26 fallisce da solo

Probe zero-fit del source H0 W256 (tree `f7f6c898…`) alimentato direttamente
con `observation[:35]` su route V26: **FAIL a 209/500 step**, penetrazione
25,489 mm, 0 cicli validi, **zero clipping**, integrità diagnostica PASS.
Conclusione: il blocker è l'incompatibilità closed-loop tra la policy storica
e la semantica V26, non l'errore di approssimazione dei fit. La comparativa
col tape sicuro V8R1P1 mostra divergenza dall'azione allo step 1, uscita dal
99° percentile del supporto 18D allo step 14, e un margine fisico del tape
stesso di soli 0,676 mm — il corridoio del caso `+0.20` è strettissimo.
Decisione vincolante: successore V26-only W256 dai sei tape sicuri, 18
feature invarianti (2:10, 25:35), dati candidate-exposed prima del primo test
puro.

### V12R12 — successore scritto, testato e revisionato; non congelato

La sessione precedente ha scritto i 10 sorgenti (15:12–15:48) senza report né
esecuzione: protocollo `AB06_H0_V12R12_V26_INVARIANT_SAFE_TEACHER`, pipeline
P0 (fit tape) → P1 (tape-dagger candidate-exposed, 18 rollout, shield p99 +
latch 15/10 mm) → P2 (refit 12.000 righe) → P3 (development puro risk-first),
gate offline ~10× più severi. La sessione di audit ha poi registrato:

- **pytest 68/68 PASS** con l'interprete canonico;
- **review avversariale (40 agenti): 0 critical, 3 major, ~15 minor, ~14
  info confermati; nessun mismatch produttore/verificatore attivo**; le 4
  classi di bug storiche (R2/R7/R8/R10) risultano chiuse nei path di
  produzione; masking 10:25 e hash del corpus verificati indipendentemente;
- **fix pre-freeze applicati e verificati** (stessa giornata, dettagli
  nell'addendum del report v12r12): ri-canonicalizzazione mask reale,
  `BaseException` in P0/P2, `__init__.py` in closure, guardie anti-subdir,
  `_tree_record` unificato nei tre stadi, occupancy fuori dal lock P1, tre
  check tautologici trasformati in ricalcoli indipendenti, binding
  freeze/lock in receipt/ledger P1 (produttore e verificatore in lockstep),
  CLI P1 con ack a stringa esatta, tolleranza std P3 allineata al runtime,
  `risk_first` data-driven;
- **nuovi test 68 → 82**: roundtrip reale produttore→verificatore P1→P2 con
  tre negative sul binding governance (lezione R8 chiusa con guardia
  automatica), lockstep naming contract↔fitter sui valori reali, lock
  post-occupazione (R10), prefisso troncato (R7), contatori V26 annidati,
  drift identità NPZ;
- **verifica avversariale dei fix** (4 verificatori): fitter CLEAN, P3
  CLEAN; 1 major e 5 minor su P1/test, tutte risolte o accettate e
  documentate;
- **stato finale: 82/82 PASS, ruff pulito, one-shot intatto — namespace
  pronto per il freeze** (ack `V12R12_FREEZE_REVIEWED_SOURCES`).

### Audit dello stato reale e debito documentale

Censimento parallelo (6 agenti) di namespace, ledger, timeline, git, run e
lineage non documentate: tutti i 9 hash dichiarati nei report combaciano su
disco; nessun training non autorizzato; ricostruita la sessione notturna
09–10/08 mai documentata (V12R2 **eseguita** e fallita per bug forense,
V12R3 col **primo pure-probe PASS del progetto**, salvage P1, V12R4,
satelliti). Scritti retroattivamente: retrospettiva forense, daily
2026-08-10, daily 2026-08-14, report V12R12, questo daily.

Rischi amministrativi emersi dall'audit:

- **nessun commit dal 04/08** (HEAD `4be9f2a`, fase "detector V17"): tutto il
  ciclo V19→V26 + H0 V11→V12R12 è untracked/non committato;
- `.gitattributes` pinna la directory inesistente `h0_v12_runs/**` invece
  delle run reali `validation/v12r*/`;
- `train_ppo_mlp.py` pinna il modulo residuale V25 al contratto evento V20
  mentre la lineage corrente è V26 (da riconciliare prima del training);
- DLL Windows assenti (`SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`,
  `OnlineGRFContact.dll`): Windows resta NO-GO fail-closed;
- anomalia da chiarire: nel tree del teacher H0 retry, `best`, `last` e
  `initial_warm_start` hanno `module_state.pkl` byte-identici.

## TODO completati o superseduti

- [x] Implementare ed eseguire V12R10 import-only W1024 (terminal FAIL,
  chiusa).
- [x] Eseguire il probe discriminante V12R11 H0-diretto-su-V26 (terminal
  FAIL, chiusa; diagnosi causale acquisita).
- [x] Colmare il debito documentale: daily 10/08, 14/08, 15/08,
  retrospettiva 09–10/08, report V12R12.
- [x] Registrare l'esito dei test v12r12 (68/68 PASS) e completare la
  revisione sorgenti pre-freeze (review avversariale, GO condizionato).

## TODO aperti e propagati

### V12R12 (percorso critico)

- [x] Applicare i fix pre-freeze e i test producer→verifier, rieseguire la
  suite (82/82) e verificare avversarialmente il diff — completato, vedi
  [report V12R12](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md)
  e relativo addendum.
- [ ] Freeze + lock (ack `V12R12_FREEZE_REVIEWED_SOURCES` per il fitter,
  `V12R12_P1_FREEZE_REVIEWED_SOURCES` per P1) ed esecuzione one-shot
  P0 → P1 → P2 → P3 secondo la sequenza aggiornata nell'addendum, col caso
  `+0.20` come primo gate fisico.
- [ ] Non ritentare, modificare o promuovere V12R5–V12R11; nessun nuovo fit
  verso il target legacy; nessuno shadow legacy online nel runtime.
- [ ] Q3, checkpoint-zero e Morphology Corridor restano chiusi fino al
  terminal PASS fisico P3; poi, in ordine e sullo stesso candidato: Q3
  held-out → checkpoint-zero (resume-only) → morphology A/B 0→0,0025 →
  preflight 12 EnvRunner → 50 update con audit d'integrità.

### Amministrativo e igiene del repo

- [ ] Commit checkpoint del lavoro 09–15/08, correggendo prima i pattern
  `.gitattributes` verso le directory reali `v12r*/` (protezione binari e
  stabilità hash su checkout Windows).
- [ ] Riconciliare il pin V20 del modulo residuale in `train_ppo_mlp.py` con
  la lineage V26.
- [ ] Chiarire l'anomalia best==last==warm_start nel tree del teacher H0
  retry.
- [ ] Chiarire quale metrica offline bocciò il `fit_p2` V12R3 (eval offline
  read-only sul corpus + modulo salvati).

### Multipiattaforma e deployment (di lungo corso)

- [ ] Compilare e testare su Windows x86_64 i DLL esatti
  `SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` e `OnlineGRFContact.dll`;
  claim numerico separato per quella piattaforma.
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery,
  soggetti/profili esterni, rumore e delay; prove su trial/velocità/soggetti
  diversi prima di dichiarare generalizzazione.
- [ ] Equivalenza host-target, latenza worst-case, HIL e contratto hardware
  prima di qualsiasi claim di deployment; detector su segnali hardware con
  ground truth localizzata; GRF generation production-ready.
- [ ] Conservare come TODO storico la deflessione SEA iniziale coerente con
  la coppia richiesta.

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
- [ ] Rieseguire la suite completa e un re-audit dei punti toccati — vedi addendum. — [fonte 2026-08-15, riga 176](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md).
- [ ] Q3, checkpoint-zero e Morphology Corridor restano chiusi fino al terminal PASS fisico P3. — [fonte 2026-08-15, riga 181](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md).
- [ ] Materializzare le label V26-pure per l'audit offline, pubblicare soltanto se eleggibile la copia core byte-exact dell'H0 W256 e rieseguire da zero le qualification fisiche fail-closed, senza usare un nuovo fit o riutilizzare il candidato V12R10 come predecessore promosso. — [fonte 2026-08-15, riga 245](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Definire e congelare in un namespace successore il relabel stateless e byte-deterministico dell'esatta osservazione V26-35; non introdurre uno shadow legacy online. — [fonte 2026-08-15, riga 240](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Conservare l'esito dell'audit di osservabilita come rischio e gate esplicito della qualification; l'audit e concluso, ma l'alias non e dichiarato risolto dal solo fit offline. — [fonte 2026-08-15, riga 235](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Soltanto dopo tale PASS, eseguire in ordine Q3, checkpoint-zero e la qualification del Morphology Corridor sul candidato successore congelato, mantenendo tutte le superfici fail-closed. — [fonte 2026-08-15, riga 252](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Aggiungere i test d'integrazione producer→verifier — vedi addendum. — [fonte 2026-08-15, riga 175](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md).
- [ ] Solo dopo tali gate, congelare il protocollo training con warm-start imitativo, detector binario e corridor attivi; completare il restore/preflight su 12 EnvRunner prima di avviare e monitorare le 50 update previste. — [fonte 2026-08-15, riga 255](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Freeze + lock P0/P2 (ack `V12R12_FREEZE_REVIEWED_SOURCES`) ed esecuzione one-shot `--execute --stage p0` secondo la sequenza preregistrata nei runner. — [fonte 2026-08-15, riga 178](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md).
- [ ] Non dichiarare il sistema training-ready finche development, Q3, checkpoint-zero, Morphology Corridor e preflight training non sono tutti terminal PASS sullo stesso candidato successore congelato. — [fonte 2026-08-15, riga 258](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md).
- [ ] Applicare i fix pre-freeze (3 major + minor elencati) — vedi addendum. — [fonte 2026-08-15, riga 174](../user/2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Raccogliere e pesare esplicitamente gli stati delle regioni di recovery, evitando che il teacher nasconda la divergenza dello student senza che il protocollo la contabilizzi. — [fonte 2026-08-09, riga 374](../user/2026-08-09_h0_v11_weighted_full_mean_fail_closed_loop.md). **Chiuso:** R9 raccoglie quattro nuove collection student-exposed valide, corpus11875; la forense corregge poi la pesatura nel R10. ([evidenza](2026-08-14_daily-report.md)).
- [x] Concludere l'audit di osservabilita e conservarne il rischio come gate esplicito della qualification fisica. — [fonte 2026-08-14, riga 346](../user/2026-08-14_h0_v12r10_fit_forensics.md). **Chiuso:** Audit osservabilità LegacyGaitShadow/TIMEOUT concluso, rischio preservato. ([evidenza](2026-08-14_daily-report.md)).
- [x] Implementare V12R10 import-only W1024 in namespace nuovo, congelarlo ed eseguire una sola volta i sei development pure-policy. — [fonte 2026-08-14, riga 348](../user/2026-08-14_h0_v12r10_fit_forensics.md). **Chiuso:** R10 import-only production fit PASS, primo puro FAIL a212step. ([evidenza](2026-08-15_daily-report.md)).
- [x] Debito documentale della sessione 09–10/08: colmato da questo report e dal daily retroattivo `2026-08-10_daily-report.md`. — [fonte 2026-08-15, riga 184](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md). **Chiuso:** Il report marca esplicitamente il TODO come completato. ([evidenza](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md)).
- [x] Eseguire come primo discriminatore V12R11 il source H0 W256 direttamente sulla observation V26-35, senza fit, widening o distillazione. — [fonte 2026-08-15, riga 243](../user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md). **Chiuso:** Discriminatore H0 diretto su V26, R11, eseguito: FAIL a209step. ([evidenza](2026-08-15_daily-report.md)).
- [x] TODO successivo: costruire una nuova lineage V26-only W256 dai sei action tape sicuri riprodotti 6/6 su V26, usando inizialmente soltanto le 18 feature meccaniche/reference invarianti (`2:10` e `25:35`). Il primo gate fisico resta lo stesso caso deterministico `+0.20`; solo dopo il suo PASS si potranno aprire development completo, Q3, checkpoint-zero, Morphology Corridor e training. — [fonte 2026-08-15, riga 134](../user/2026-08-15_h0_v12r11_direct_v26_terminal_fail.md). **Chiuso:** Nuova lineage R12 V26-only W256, mask18/tape-dagger, costruita e verificata. ([evidenza](2026-08-15_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
