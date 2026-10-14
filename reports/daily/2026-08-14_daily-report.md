# Daily Report - 2026-08-14

Instruction check token: CMC_AGENT_OK_2026

> **Nota**: daily scritto retroattivamente il 2026-08-15. Il report
> `fit_forensics` è datato 14/08 ma è stato finalizzato il 15/08 alle 13:12:
> le sue ultime sezioni (dry-fit gate-aligned, audit di osservabilità) coprono
> lavori conclusi la mattina del 15/08.

## Report utente consolidati

Lineage esecutive:

- [V12R5 case-balanced — fit terminal FAIL](../user/2026-08-14_h0_v12r5_case_balanced_fit_terminal_fail.md)
- [V12R6 functional composite — sintesi PASS, sviluppo fisico terminal FAIL](../user/2026-08-14_h0_v12r6_functional_composite_terminal_fail.md)
- [V12R7 — design recovery same-state](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md)
- [V12R7 — chiusura sorgente recovery e scaffold Q3](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md)
- [V12R7 — terminal FAIL per proiezione errata del prefisso V26](../user/2026-08-14_h0_v12r7_prefix_projection_terminal_fail.md)
- [V12R8 — prefisso R7 adjudicated e protocollo source-ready](../user/2026-08-14_h0_v12r8_adjudicated_prefix_source_ready.md)
- [V12R8 — errore terminale di proiezione dell'albero candidato](../user/2026-08-14_h0_v12r8_candidate_tree_projection_terminal_fail.md)
- [V12R9 — recovery additiva da R8 source-ready](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md)
- [V12R9 — raccolta valida, fit terminal FAIL](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md)
- [V12R10 — forense del fit R9 e selezione del recupero](../user/2026-08-14_h0_v12r10_fit_forensics.md)

Infrastruttura satellite (tutta deferred/fail-closed):

- [Q3 runtime source-only e blocco sulla lineage R5 terminale](../user/2026-08-14_q3_runtime_source_only_fail_closed.md)
- [V12R7-Q3 — runtime completo, storico e fail-closed](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md)
- [V12R8-Q3 — protocollo canonico source-closed](../user/2026-08-14_h0_v12r8_q3_source_closed.md)
- [V12R8 — runtime latente full checkpoint-zero](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md)
- [V12R8 — scaffold qualification positiva Morphology Corridor](../user/2026-08-14_h0_v12r8_morphology_positive_qualification_scaffold.md)
- [V12R8 morphology — runtime latent-live e training gate](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md)
- [V12R9-Q3 — qualifica indipendente source-ready e deferred](../user/2026-08-14_h0_v12r9_q3_source_ready_deferred.md)
- [V12R9 checkpoint-zero — runtime latente e fail-closed](../user/2026-08-14_h0_v12r9_checkpoint_zero_latent_runtime.md)
- [V12R9 morphology causale — source-ready, esecuzione differita](../user/2026-08-14_h0_v12r9_morphology_source_ready_deferred.md)
- [V12R9 — preflight 12 EnvRunner e monitor training 50 update](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md)

## Sintesi

Giornata di iterazione serrata (ripresa 16:33, chiusura ~23:00, con coda
diagnostica nella notte): cinque lineage one-shot chiuse in terminal FAIL, di
cui due per bug del verificatore e tre per ragioni scientifiche reali. La
forense finale ha prodotto la prima diagnosi causale solida del blocco
imitativo.

| Lineage | Esito | Causa | Classe |
|---|---|---|---|
| V12R5 | FAIL al fit | RMSE 0,00785 / max 0,0689 oltre gate; il bilanciamento uniforme migliora `+0.20` ma degrada globale e `-0.20` | scientifica |
| V12R6 | FAIL fisico, step 179 | composito esatto `0.70·P2+0.30·R5` (W512) passa tutti i gate offline ma penetra a 25,79 mm: covariate shift closed-loop | scientifica |
| V12R7 | FAIL spurio, 1º stage | gate legge contatori V26 dalla proiezione top-level del summary dove non vengono pubblicati (vivono in `binary_phase_event_gate`); il rollout aveva riprodotto byte-exact il terminale R6 | verificatore |
| V12R8 | FAIL spurio, 2º stage | confronto stretto fra record candidato full-tree (con `files`) e proiezione ridotta nel lock; il prefisso `-0.20` (252 step) è evidenza valida | verificatore |
| V12R9 | FAIL al fit | 4 raccolte nuove tutte 500/500 PASS, corpus 11.875 righe valido, ma fit W512: RMSE 0,011 / max 0,140 / reset 0,0135 | scientifica |

### Diagnosi forense (report `fit_forensics`)

- nessuna collisione contraddittoria nel corpus: l'impossibilità matematica su
  35 feature non è dimostrata;
- il fitter R9 aveva perso i pesi intra-strato (le 26 righe reset allo 0,265%
  della massa);
- esclusi via dry-fit: fresh-R6, continuation R9, residual ancorato 0,70·P2,
  W512 esteso, W512 IRLS tail-aware, W1024 residuale (0,006105 — vicino ma
  FAIL);
- **unico PASS offline: dry-fit W1024 gate-aligned** (0,00497 / 0,049, reset
  0,00054), replicato byte-exact — autorizza il design della lineage canonica
  V12R10, non la promozione;
- **diagnosi causale chiave**: i picchi observer (step 385–388) coincidono con
  il cambio di stato per TIMEOUT del teacher legacy mentre V26 resta
  correttamente in STANCE — alias semantico history-hidden del
  `LegacyGaitShadow` (232 righe teacher in TIMEOUT vs 0 in V26; 107 cliff di
  label senza transizione V26 adiacente). La certificazione puramente offline
  Markov35 è fragile; l'opzione più pura è un futuro relabel stateless
  V26-puro.

### Infrastruttura satellite

L'intera filiera post-imitazione è stata costruita e portata due volte
(R8 → R9), sempre source-only e fail-closed: Q3 held-out (matrice ±0,30 s +
seed 130–133), checkpoint-zero (resume-only, cwd=repo root), morphology
causale A/B 0,0→0,0025 con buffer strict-delay 40 ms (correzione additiva del
flush terminale del corridor congelato, iniettata via `sitecustomize`),
validator training (preflight 0→12 EnvRunner, audit 50 update, comando
canonico congelato). Nessun namespace satellite ha artefatti canonici: tutto
resta subordinato a un terminal PASS imitativo che il 14/08 non esiste.

### Contatori globali della giornata

Reset/step ambiente solo nelle raccolte e nei development autorizzati; zero
update critic, zero update PPO, zero training; nessuna modifica a plugin C++,
GRF primaria, detector/FSM V26, semantica SEA o checkpoint storici.

## TODO completati o superseduti

- [x] Nuova lineage post-V12R4 con validazione offline preventiva (V12R5,
  eseguita e chiusa).
- [x] Recupero del candidato senza ripetere V12R5: composito funzionale V12R6
  (eseguito e chiuso).
- [x] Diagnosi del fail fisico R6 ed esclusione del rescue via alpha-blend.
- [x] Protocollo observer-DAgger same-state (R7→ R8 → R9) con adjudication dei
  prefissi storici senza rerun.
- [x] Correzione dei due bug di proiezione (R7 summary, R8 candidate tree)
  nei rispettivi successori additivi.
- [x] Forense completa del fit R9: pesi intra-strato, dry-fit esclusi,
  gate-aligned W1024 PASS offline, audit di osservabilità.

## TODO aperti e propagati

### Lineage imitativa

- [ ] Implementare V12R10 import-only W1024 gate-aligned in namespace nuovo,
  congelarla ed eseguire una sola volta i sei development pure-policy
  (→ eseguito il 15/08).
- [ ] Non ritentare, modificare o promuovere V12R5–V12R9.
- [ ] Conservare il rischio di alias teacher/legacy come gate esplicito della
  qualification fisica; se il fisico fallisce, il successore è il relabel
  stateless V26-puro.
- [ ] Chiarire quale metrica offline bocciò il `fit_p2` V12R3 (dalla sessione
  09–10/08; valori non persistiti).

### Qualification e training

- [ ] Q3, checkpoint-zero e Morphology Corridor restano chiusi fino a un
  terminal PASS fisico imitativo.
- [ ] Dopo un PASS: Q3 held-out → checkpoint-zero → morphology A/B →
  preflight 12 EnvRunner → 50 update, in quest'ordine, sullo stesso candidato.
- [ ] Zero-update save/reload e `checkpoint_zero` prima di pubblicare il
  comando warm-start (resume-only).

### Multipiattaforma e deployment (di lungo corso)

- [ ] Compilare e testare su Windows x86_64 i DLL esatti
  `SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` e `OnlineGRFContact.dll`
  (assenti: Windows resta NO-GO fail-closed).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery,
  soggetti/profili esterni, rumore e delay.
- [ ] Equivalenza host-target, latenza worst-case, HIL e contratto hardware
  prima di qualsiasi claim di deployment; detector su segnali hardware con
  ground truth localizzata.
- [ ] Conservare come TODO storico la deflessione SEA iniziale coerente con la
  coppia richiesta.

## Integrazione documentale retrospettiva — 2026-10-05

Queste integrazioni consolidano risultati e TODO dei report utente già esistenti. La data di registrazione è il 5 ottobre 2026: non sono nuove simulazioni, test, training o modifiche del codice. Gli esiti terminali storici restano distinti dai risultati dei successori.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r7_q3_runtime_scaffold_storico"></a>

### 2026-08-14_h0_v12r7_q3_runtime_scaffold_storico

Fonte: [2026-08-14_h0_v12r7_q3_runtime_scaffold_storico](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md).

Il runtime `validation/v12r7q3/runtime/` era completo a sorgente ma reso **storico e non pubblicabile** dopo il terminal FAIL R7. Le entrypoint canoniche di freeze, preparazione noise tape ed execution rifiutano prima di scrivere: non basta clonare lo scaffold o cambiare il candidato nello stesso namespace. Il protocollo prevede12rollout baseline-first, cinque verifier semantici, noise tape float32(500,2), V26 e morphology causale a peso0 con identità sulle superfici e gate di noninferiorità SEA/reserve.

Il report attesta43test del runtime,75combinati e clean-import senza dipendere da un lock canonico; nessuna qualification fisica Q3 è stata eseguita. Il seguito previsto era una lineage R8-Q3 separata. Questo è un risultato di chiusura sorgente e blocco I/O del R7-Q3, distinto dalla sintesi dei port R8→R9 già presente. I suoi stage di execution vengono poi superati dal pivot nativo del17/08.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r7_source_closure_q3_scaffold"></a>

### 2026-08-14_h0_v12r7_source_closure_q3_scaffold

Fonte: [2026-08-14_h0_v12r7_source_closure_q3_scaffold](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md).

La chiusura sorgente R7 comprendeva collector/labeler observer, fitterW512 a13strati, freezer/runner e scaffold Q3 indipendente. Il recovery usa azioni R6 pure e label H0 interrogate offline sugli stessi stati visitati; non un rescue mediante blend statico. I contratti verificano il candidato esatto, gli output e i verifier, mentre il Q3 conserva baseline-first, detectorV26 e identità della morphology a peso0.

La suite congiunta59/59 e le attestazioni sorgente chiudevano la milestone prima del freeze, senza dichiarare training-ready o eseguire la qualification. Freeze, collection, fit e development restavano stage distinti: il successivo errore R7 sui contatori nested non annulla questa chiusura sorgente, ma ne consuma la lineage. Q3/checkpoint-zero/morphology della vecchia V12 sono poi superati il17/08.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r8_adjudicated_prefix_source_ready"></a>

### 2026-08-14_h0_v12r8_adjudicated_prefix_source_ready

Fonte: [2026-08-14_h0_v12r8_adjudicated_prefix_source_ready](../user/2026-08-14_h0_v12r8_adjudicated_prefix_source_ready.md).

R8 recuperava in sola lettura il prefisso R7 `+0.20` di179step/1790campioni: trace, replay e riproduzione R6 byte-exact restano evidenza utile, mentre il terminal FAIL R7 rimane immutabile. La chiusura source-ready non si limitava a correggere una chiave: verificava indipendentemente la query H0 sulle label, la ricostruzione del corpus e le metriche del candidato, oltre al binding full-tree del teacher e all’accounting degli stage/prefissi. I manifest autoprodotti non sostituiscono queste prove semantiche.

Il report conserva71test,120combinati e audit avversariali con GO pre-freeze, senza nuovo rollout per adjudicare il prefisso storico. La successiva execution R8 fallisce per un’altra proiezione, quella del record candidato: è un esito distinto dalla milestone sorgente e già riportato nella tabella della giornata. Il ramo qualificativo V12 è poi superato il17/08.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime"></a>

### 2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime

Fonte: [2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime](../user/2026-08-14_h0_v12r8_checkpoint_zero_latent_runtime.md).

Il runtime `v12r8zero` prepara un **full checkpoint RLlib a progresso zero**, non soltanto un export actor. Dopo PASS R8 e Q3 sullo stesso candidato deve attestare critic fresco invariato, optimizer vuoto con registrazione parametrica esatta e counter0 su transplant, save e restore. La save avviene una sola volta; il restore positivo con morphology0.0025 è una verifica senza `train()` o `sample()`. L’handoff resta esclusivamente `--resume-from checkpoint_zero`.

Test del runtime/AST e verifiche delle superfici chiudono la sorgente latente; nella milestone non esistono lock né checkpoint canonico. Il risultato attestato è quindi la disponibilità verificata del runtime, non un PASS live di checkpoint-zero. L’upstream R8 terminale ne impedisce l’esecuzione, e il port della vecchia filiera viene poi superato dal pivot del17/08.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r8_morphology_latent_live_training_gate"></a>

### 2026-08-14_h0_v12r8_morphology_latent_live_training_gate

Fonte: [2026-08-14_h0_v12r8_morphology_latent_live_training_gate](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md).

La revisione `v12r8morph` rendeva lo scaffold latent-live: dopo i PASS R8/Q3/zero dello stesso candidato sono obbligatori sei paired rerun `control→positive`, quindi12rollout. Il gate richiede un effetto di reward nonzero al peso0.0025, con identità sulle altre superfici osservazione/azione/dinamica/eventi; un controllo con perdita morphology sempre zero non dimostra una qualification positiva. Il riuso del controllo Q3 inizialmente proposto non basta per questa revisione.

Il strict-delay40ms corregge il flush terminale: un campione che non raggiunge il ritardo prima della fine viene scartato, mentre un persist nonterminale anticipato fallisce. L’iniezione `sitecustomize` è stata verificata anche su un vero worker Ray, oltre a supervisore e child, con test dedicati. Sono prove di implementazione/propagazione, non12rollout scientifici canonici: nessun freeze o qualification morphology è avvenuto. R8 resta terminale; il port additivo R9 e infine il pivot17/08 supersedono i suoi TODO di execution.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r8_q3_source_closed"></a>

### 2026-08-14_h0_v12r8_q3_source_closed

Fonte: [2026-08-14_h0_v12r8_q3_source_closed](../user/2026-08-14_h0_v12r8_q3_source_closed.md).

R8-Q3 chiudeva un rischio di falso PASS nei verifier: non fabbricava contatori zero quando mancavano e vincolava i payload agli effettivi JSON dei cinque endpoint semantici nativi. La qualification prevista è indipendente,12rollout baseline-first su±0.30s/seed130–133, con lo stesso candidato, morphology reale0, identità e noninferiorità SEA/reserve; non un gate ricostruito da soli titoli o manifest.

Il report attesta self-check, suite di chiusura sorgente e uno smoke reale macOS: reset84float32, passo10ms con10campioni raw e identità a peso0. Questo smoke verifica ABI/inserimento causale, non esegue il Q3 scientifico: freeze, tape e rollout canonici sono assenti perché manca il terminal PASS R8. La qualification della vecchia filiera è poi superata il17/08.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r9_checkpoint_zero_latent_runtime"></a>

### 2026-08-14_h0_v12r9_checkpoint_zero_latent_runtime

Fonte: [2026-08-14_h0_v12r9_checkpoint_zero_latent_runtime](../user/2026-08-14_h0_v12r9_checkpoint_zero_latent_runtime.md).

Il port `v12r9zero` usa i terminali nativi R9/Q3 e lo stesso record candidato completo a cinque file. La prova richiesta dal full checkpoint verifica actor esatto, critic fresco, optimizer senza stato e contatori zero sulle superfici transplant/save/restore; una save e smoke di restore positivo con V26 e morphology0.0025, senza `train()` o `sample()`. Il cwd deve essere la root del repository su freezer, runner e handoff; il resume-only non autorizza un nuovo warm-start.

Il report conserva24test, test negativi di cwd e self-check ABI/AST. È una chiusura verificata di sorgente latente: nessun `checkpoint_zero` canonico viene creato dopo il FAIL R9. Questa prova è distinta dal generico vincolo cwd/resume già consolidato; i relativi stage live vengono poi superati il17/08.

<a id="integrazione-2026-10-05-2026-08-14_h0_v12r9_q3_source_ready_deferred"></a>

### 2026-08-14_h0_v12r9_q3_source_ready_deferred

Fonte: [2026-08-14_h0_v12r9_q3_source_ready_deferred](../user/2026-08-14_h0_v12r9_q3_source_ready_deferred.md).

R9-Q3 è una qualification indipendente nativa della nuova lineage: i cinque verifier richiedono il record completo con `files`, rifiutano la proiezione ridotta che aveva causato il bug R8 e non accettano un endpoint di altra lineage. Gli held-out±0.30s/seed130–133 escludono i seed126–128 del fit;12rollout baseline-first conservano pairing, morphology0 reale, identità e gate di noninferiorità SEA/reserve.

La suite57test, i12self-check e la source closure attestano il port/verifier, con regressioni negative sul full-tree e sul cross-lineage. Non esistono tape o rollout Q3 canonici: il terminal PASS R9 non viene ottenuto e nessun successivo successo può essere attribuito a questo Q3. Il pivot17/08 supera i suoi stage di execution.

<a id="integrazione-2026-10-05-2026-08-14_q3_runtime_source_only_fail_closed"></a>

### 2026-08-14_q3_runtime_source_only_fail_closed

Fonte: [2026-08-14_q3_runtime_source_only_fail_closed](../user/2026-08-14_q3_runtime_source_only_fail_closed.md).

Il Q3 del R5 aveva runtime one-shot source-only, detectorV26 obbligatorio e morphology a peso0. Poiché R5 termina al fit, il preflight reale restituisce `R5_TERMINAL_NOT_PASS` **prima** di aprire noise tape, protocollo, lock o runroot;13test del runtime e la closure sorgente attestano questo blocco senza output canonici. Il namespace R5 non può essere retargettato in-place per ottenere un PASS.

Il seguito previsto era un nuovo Q3R6 soltanto dopo candidato e sei development PASS; checkpoint/update e morphology positiva restano protocolli separati. Questa milestone verifica il runtime e il rifiuto causale, non una qualification fisica. Gli stage downstream della vecchia V12 diventano superati col pivot17/08, mentre il R5 terminale rimane evidenza immutabile.

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
- [ ] Raccogliere e pesare esplicitamente gli stati delle regioni di recovery, evitando che il teacher nasconda la divergenza dello student senza che il protocollo la contabilizzi. — [fonte 2026-08-09, riga 374](../user/2026-08-09_h0_v11_weighted_full_mean_fail_closed_loop.md).
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
- [ ] Concludere l'audit di osservabilita e conservarne il rischio come gate esplicito della qualification fisica. — [fonte 2026-08-14, riga 346](../user/2026-08-14_h0_v12r10_fit_forensics.md).
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
- [ ] Implementare V12R10 import-only W1024 in namespace nuovo, congelarlo ed eseguire una sola volta i sei development pure-policy. — [fonte 2026-08-14, riga 348](../user/2026-08-14_h0_v12r10_fit_forensics.md).
- [ ] Attendere gli endpoint immutabili V12R8 e creare `v12r8q3` come nuova lineage; non clonare o congelare parzialmente prima del terminal PASS R8. — [fonte 2026-08-14, riga 98](../user/2026-08-14_h0_v12r7_q3_runtime_scaffold_storico.md).
- [ ] Creare il checkpoint-zero full RLlib 2x512 con actor imitativo esatto e critic/optimizer/counter freschi. — [fonte 2026-08-14, riga 109](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Aprire un nuovo lineage soltanto con un meccanismo che riduca l'errore closed-loop sulla distribuzione student-exposed; le opzioni in valutazione sono un fit 512 con recovery labels o una raccolta teacher-from-start preregistrata e sicura. — [fonte 2026-08-14, riga 113](../user/2026-08-14_h0_v12r6_functional_composite_terminal_fail.md). **Chiuso:** Forense R6 e nuovo recovery observer same-state R7→R9 documentati. ([evidenza](2026-08-14_daily-report.md)).
- [x] Eseguire una sola pipeline R7: sei collect/label, unico fit e sei development rollout. — [fonte 2026-08-14, riga 106](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md). **Chiuso:** Freeze/runner R7 consumati; terminal FAIL al primo stage. ([evidenza](2026-08-14_daily-report.md)).
- [x] completare l'audit e l'esecuzione one-shot R8; Q3 resta bloccato se il ledger R8 non è terminale `PASS`; — [fonte 2026-08-14, riga 139](../user/2026-08-14_h0_v12r8_q3_source_closed.md). **Chiuso:** Audit, freeze ed execution R8 consumati; terminal FAIL per candidate-tree projection. ([evidenza](2026-08-14_daily-report.md)).
- [x] eseguire una sola volta collection, labeling, fit e development R7; — [fonte 2026-08-14, riga 160](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md). **Chiuso:** Freeze/runner R7 consumati; terminal FAIL al primo stage. ([evidenza](2026-08-14_daily-report.md)).
- [x] completare e testare fitter 512 a 13 strati; — [fonte 2026-08-14, riga 158](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md). **Chiuso:** Collector/labeler/fitter e Q3 source-complete/verificati. ([evidenza](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md)).
- [x] completare l'audit indipendente finale su hash e ABI dello snapshot formattato; — [fonte 2026-08-14, riga 138](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md). **Chiuso:** Test R9 e contratto pre-execution PASS; runner eseguito. ([evidenza](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md)).
- [x] Pubblicare e verificare protocol freeze ed execution lock R7. — [fonte 2026-08-14, riga 105](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md). **Chiuso:** Freeze/runner R7 consumati; terminal FAIL al primo stage. ([evidenza](2026-08-14_daily-report.md)).
- [x] Completare la diagnosi forense R6/V12R4/teacher replay. — [fonte 2026-08-14, riga 112](../user/2026-08-14_h0_v12r6_functional_composite_terminal_fail.md). **Chiuso:** Forense R6 e nuovo recovery observer same-state R7→R9 documentati. ([evidenza](2026-08-14_daily-report.md)).
- [x] congelare protocollo ed execution lock R7; — [fonte 2026-08-14, riga 159](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md). **Chiuso:** Freeze/runner R7 consumati; terminal FAIL al primo stage. ([evidenza](2026-08-14_daily-report.md)).
- [x] portare questa implementazione in un namespace morphology R9, collegando i tre nuovi endpoint semantici; — [fonte 2026-08-14, riga 178](../user/2026-08-14_h0_v12r8_morphology_latent_live_training_gate.md). **Chiuso:** Port morphology R9 source-ready, deferred. ([evidenza](../user/2026-08-14_h0_v12r9_morphology_source_ready_deferred.md)).
- [x] Chiudere la forense del fit e costruire un successore in namespace nuovo, senza nuove raccolte salvo evidenza che le renda indispensabili. — [fonte 2026-08-14, riga 126](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md). **Chiuso:** Forense fit R9 chiusa, R10 import-only definito e poi eseguito il15/08. ([evidenza](2026-08-14_daily-report.md)).
- [x] solo dopo GO dell'audit, pubblicare freeze e execution lock R9; — [fonte 2026-08-14, riga 139](../user/2026-08-14_h0_v12r9_additive_recovery_source_ready.md). **Chiuso:** Test R9 e contratto pre-execution PASS; runner eseguito. ([evidenza](../user/2026-08-14_h0_v12r9_additive_recovery_fit_terminal_fail.md)).
- [x] completare freeze ed esecuzione one-shot V12R8; — [fonte 2026-08-14, riga 107](../user/2026-08-14_h0_v12r7_prefix_projection_terminal_fail.md). **Chiuso:** Audit, freeze ed execution R8 consumati; terminal FAIL per candidate-tree projection. ([evidenza](2026-08-14_daily-report.md)).
- [x] Progettare una nuova lineage V12R6, senza riaprire o ritentare V12R5, che corregga il trade-off introdotto dal bilanciamento uniforme dei sei casi. — [fonte 2026-08-14, riga 102](../user/2026-08-14_h0_v12r5_case_balanced_fit_terminal_fail.md). **Chiuso:** R6 composito W512 offline PASS, poi FAIL fisico a179step. ([evidenza](2026-08-14_daily-report.md)).
- [x] completare e testare collector/labeler observer R7; — [fonte 2026-08-14, riga 157](../user/2026-08-14_h0_v12r7_same_state_recovery_design.md). **Chiuso:** Collector/labeler/fitter e Q3 source-complete/verificati. ([evidenza](../user/2026-08-14_h0_v12r7_source_closure_q3_scaffold.md)).
- [x] Validare offline la strategia V12R6 prima del freeze one-shot, includendo esplicitamente il caso `-0.20`, seed 127 e i limiti globali/P2. — [fonte 2026-08-14, riga 104](../user/2026-08-14_h0_v12r5_case_balanced_fit_terminal_fail.md). **Chiuso:** R6 composito W512 offline PASS, poi FAIL fisico a179step. ([evidenza](2026-08-14_daily-report.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
