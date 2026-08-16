# H0 V12R8 — prefisso R7 adjudicated e protocollo source-ready

Data: 2026-08-14

## Problema

Il one-shot V12R7 si è chiuso correttamente come terminal FAIL al primo stage,
`collect_label__deterministic_offset_plus_0p20`, nonostante il rollout fisico
avesse prodotto un prefisso utile di 179 step e 1790 campioni detector.

La diagnosi ha isolato un difetto di proiezione del summary, non un difetto del
detector V26 o della riproduzione della policy:

- trace e journal per-step sono chiusi e byte-exact;
- replay e trace hanno 179 step, 180 boundary e contratto evento V26 esatto;
- la riproduzione del prefisso R6 è 179/179 byte-exact sui campi condivisi;
- `binary_event_prefix_integrity.passed` è `true` per 1790 campioni;
- i cinque contatori nested V26 sono interi e uguali a zero;
- `binary_phase_event_gate.passed` è `false` perché quel gate è definito per
  l'orizzonte completo da 5000 campioni e non è applicabile al prefisso;
- il summary R7 non proietta a top-level
  `duplicate_event_count`, `out_of_order_event_count` e
  `left_non_v26_source_count`, e non materializza `target_contract_id`;
- per questo il gate R7 persisted fallisce solamente
  `contract_integrity`, `detector_active` e `zero_detector_anomalies`.

Gli artefatti R7 e il ledger terminale devono restare immutabili: il caso plus
non può essere corretto, ripreso o rieseguito nello stesso namespace.

## Soluzione

È stato creato il successor additivo e fail-closed `validation/v12r8/`.

Il suo adjudicator divide esplicitamente lettura e calcolo:

1. l'adapter read-only verifica hash, size e semantica di protocol freeze,
   execution lock, ledger terminale, trace, replay, summary, gate e receipt R7;
2. verifica inoltre che trace e journal per-step coincidano e ricalcola la
   riproduzione R6 179/179;
3. solo dopo questa catena deriva `target_contract_id` dalla source closure del
   contratto R7 attestata sia dal freeze sia dal lock;
4. il normalizzatore puro copia il summary e aggiunge esclusivamente i tre
   contatori nested mancanti e il target derivato, senza cambiare valori,
   soglie o il `false` del gate nested non applicabile;
5. il gate R7 controfattuale, ricalcolato sul summary normalizzato, passa e
   autorizza esclusivamente il label offline, mai una claim di autonomia.

Sul corpus storico il risultato reale è
`PASS_H0_V12R8_R7_PLUS_PREFIX_ADJUDICATION`. La proiezione esatta è:

- `duplicate_event_count = 0`;
- `out_of_order_event_count = 0`;
- `left_non_v26_source_count = 0`;
- `target_contract_id = primary_grf_split_v1+binary_point_v25+heel_qualified_fsm_v2`.

## Strategia del protocollo R8

L'ordine one-shot è ora:

1. adjudication read-only del prefisso plus R7;
2. label H0 offline del replay storico nel solo namespace R8;
3. cinque nuove collection R8, escludendo per costruzione il rerun del plus;
4. label H0 offline di ciascuna nuova collection;
5. unico fit full-mean W512 con gli stessi 13 strata a massa uguale di R7;
6. freeze semantico del candidato ordinario `35 -> 512 -> 512 -> 2`;
7. sei development pure-policy, teacher-free, senza blend e senza latch;
8. receipt finale e ledger terminale one-shot.

Ogni nuovo probe normalizza il summary dai dati nested V26 prima della
finalizzazione forense e prima del gate. Anche i development mantengono il
summary fisico raw immutato e pubblicano un `normalized_summary.json`
separatamente hash-bound prima del gate R8.

L'accounting distingue esplicitamente:

- adjudication e label storico: 0 reset, 0 step ambiente;
- nuove collection: esattamente 5 reset;
- development: esattamente 6 reset;
- totale atteso al terminal PASS: 11 reset, con gli step separati per scope;
- un solo actor fit/update, zero critic update e zero PPO update.

La superficie di scrittura del runner rifiuta qualunque destinazione fuori da
`validation/v12r8/`; i path R7 sono accettati soltanto dalle letture attestative.

Il boundary CLI è inoltre fail-closed anche quando il ledger terminale esiste
già: `--execute` e `--verify` verificano e stampano il ledger, ma restituiscono
exit code 1 se `passed` è `false`. L'API Python `execute()` resta idempotente e
non muta, riprende o ritenta un lineage terminale.

## Chiusura semantica pre-freeze

Un audit avversariale successivo ha identificato tre closure inizialmente
insufficienti. La verifica del fit si limitava a fidarsi delle metriche già
persistite; le label NPZ venivano controllate per schema e finitezza ma non
ricalcolate dal replay; inoltre il checkpoint H0 interrogato e il corpus usato
per la coverage non erano entrambi input locked del protocollo.

La closure è stata quindi resa eseguibile e indipendente dal fit:

1. per ciascuno dei sei casi, il verifier richiude receipt e replay sorgente,
   ricarica H0 solo dopo tale boundary, ricostruisce le observation teacher,
   esegue esattamente una query H0 per riga e confronta dtype, shape e bytes di
   tutte le array con `labels.npz`;
2. summary, gate, receipt e stage receipt delle label vengono ricostruiti e
   confrontati semanticamente, con zero reset/step ambiente;
3. il fitter ricompone gli otto NPZ sorgente reali — base R5, R4 e sei label
   observer — e confronta byte-exact tutte le array di `corpus.npz`;
4. il mapping dei 13 strata viene ora persistito esplicitamente come
   `stratum_ids` e verificato insieme a ordine, row count, masse uniformi e
   digest;
5. il candidato ricaricato viene rieseguito offline sull'intero corpus; vengono
   ricalcolati global RMSE/max error, reset max error, metriche per caso base,
   R4, sei casi observer, coda plus da step 140 e worst row;
6. vengono ricalcolate anche preservation full-mean/logstd/clock, normalizzazione,
   invarianti statiche dell'optimizer, candidate manifest, corpus manifest e
   fit gate; summary, report, gate e receipt devono concordare con questi dati
   ricalcolati.

Il candidato non è verificato soltanto tramite algebra sui tensori: il verifier
ricarica il vero `RLModule`, esegue `forward_inference` sull'intero corpus e
richiede parità byte-exact fra logits runtime, logits ricostruite dallo state e
predizioni da cui derivano le metriche.

Il teacher H0 è ora locked come tree completo a tre file, con ID
`H0_MARKOV35_PHASE_ALIGNED_SIGMA0005_ITER1_RETRY` e tree SHA-256
`f7f6c898975af109412af8c3f1a338b5076f9fefcec1e2723673fd821f1f13ee`.
Il reference corpus coverage da 6000 righe è locked con SHA-256
`232e0776f67e7a1425288c4f3979409df998ef34a7e60c618ec6c5d7cd9c4933`.
Entrambi vengono attestati nel protocol freeze e nell'execution lock,
ricontrollati immediatamente prima e dopo ogni labeling e legati in label
summary, receipt e stage receipt. Qualunque drift fa fallire anche le verifiche
successive del freeze/lock.

Il terminal ledger è stato irrobustito nello stesso passaggio: ogni verifica
invoca protocol e lock verifier in modalità non-pristine, riesegue il verifier
semantico di ogni stage completato e, su terminal FAIL, richiede un proper
prefix, `attempted_stage` uguale al successivo stage preregistrato, errore con
schema e stringhe stretti e contatori compatibili col prefisso e con l'eventuale
stage parzialmente tentato.

Il re-audit indipendente ha poi chiuso anche le superfici di control-plane:

- pipeline claim e worker claim hanno payload semanticamente esatto, non solo
  hash aggiornabile;
- la directory claim deve contenere esattamente il prefisso completato e lo
  stage tentato, senza claim mancanti, futuri o extra;
- ledger e stage receipt hanno schema esatto e ricostruiscono ordine, candidate
  selection, candidate tree/ID, observation, freeze e receipt finale;
- i cinque probe chiudono identità di summary/receipt, candidate R6 e conteggi
  replay/label/query contro la ricostruzione semantica;
- i sei development rileggono il trace, ricalcolano
  `pure_policy_trace_audit` e derivano dal summary V26 attivo, morphology a zero
  e contatori update strettamente nulli;
- gli step finali vengono derivati dai cinque replay collection e dai sei
  summary development, quindi confrontati con receipt e ledger.

## Interfaccia Q3 preservata

I cinque endpoint pubblici mantengono firma e semantica della ABI Q3, con
identità esclusivamente R8:

- `verify_protocol_freeze()`;
- `verify_execution_lock(*, require_pristine=False, runtime_attestor=None)`;
- `verify_candidate_freeze_receipt(*, fit_verifier=None)`;
- `verify_final_development_receipt(*, fit_verifier=None)`;
- `verify_terminal_ledger(*, fit_verifier=None)`.

Candidate selection, path, ID prefix, receipt finali e ledger usano schema 1280,
protocol ID `AB06_H0_V12R8_ADJUDICATED_RECOVERY_W512_V26` e pipeline ID
`H0_V12R8_R7_PREFIX_ADJUDICATION_SINGLE_FIT`, senza alias identitari R7.

## File creati

Produzione:

- `Trajectory Generator/baseline_MLP/validation/v12r8/__init__.py`;
- `h0_v12r8_recovery_contract.py`;
- `h0_v12r8_prefix_adjudicator.py`;
- `h0_v12r8_recovery_probe.py`;
- `h0_v12r8_recovery_fitter.py`;
- `freeze_h0_v12r8_recovery.py`;
- `run_h0_v12r8_recovery.py`.

Test nella stessa cartella di validazione:

- `test_h0_v12r8_recovery_contract.py`;
- `test_h0_v12r8_prefix_adjudicator.py`;
- `test_h0_v12r8_recovery_probe.py`;
- `test_h0_v12r8_recovery_fitter.py`;
- `test_h0_v12r8_semantic_closure_mutations.py`;
- `test_freeze_h0_v12r8_recovery.py`;
- `test_run_h0_v12r8_recovery.py`.

Nessun file o artefatto V12R7 è stato modificato.

## Test e verifiche

- adjudication sui veri artefatti canonici R7: PASS;
- mutation test fail-closed su hash, riproduzione, event contract e prefix
  integrity: PASS;
- test del divieto di rerun storico e del blocco H0 prima
  dell'adjudication: PASS;
- test che la normalizzazione preceda finalize e gate: PASS;
- test label same-state e zero accessi ambiente: PASS;
- test fitter W512, 13 strata, AdamW 2000 + LBFGS 300/600, logstd e clock
  invarianti: PASS;
- test freezer, path canonicali, symlink/reparse rejection e source closure:
  PASS;
- test runner one-shot, no retry, accounting scoped e write guard R8-only:
  PASS;
- regressione CLI su ledger FAIL preesistente per `--execute` e `--verify`:
  PASS, exit code nonzero e zero retry;
- mutation test su label NPZ, corpus ricomposto, summary/metrica, actor mean,
  teacher H0 tree e coverage reference: PASS, tutti fail-closed;
- mutation test terminal ledger su `attempted_stage` e completed-stage
  semanticamente invalido: PASS;
- `ruff check`: PASS;
- `ruff format --check`: PASS;
- `compileall`: PASS;
- suite R8 lanciata dalla root del repository senza `PYTHONPATH`: 71 PASS;
- integrazione con lo scaffold Q3-R8: 120 PASS complessivi tra core R8 e Q3-R8;
- smoke avversariale indipendente su claim pipeline, worker tentato, claim
  extra, schema terminale, development trace/runtime e identità probe: 5/5
  PASS, tutte le falsificazioni respinte.

La source closure attesa comprende 82 file e include le dipendenze R7 usate
soltanto per l'adjudication immutabile.

## Stato della milestone

Il core V12R8 ha ricevuto **GO pre-freeze** dall'audit indipendente. Non sono
stati ancora creati il protocol freeze, l'execution lock o il run root
canonico R8; il tentativo one-shot è quindi ancora intatto e può passare alla
pubblicazione controllata di freeze e lock.
