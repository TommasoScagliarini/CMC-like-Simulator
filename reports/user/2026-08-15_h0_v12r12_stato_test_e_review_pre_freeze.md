# H0 V12R12 — stato source-only, test PASS e review avversariale pre-freeze

Data: 2026-08-15

## Esito

Il namespace `Trajectory Generator/baseline_MLP/validation/v12r12/` è
**source-only**: 10 file Python scritti il 15/08 tra le 15:12 e le 15:48,
nessun protocol freeze, execution lock, run root o artefatto canonico.
Nessuno stage è stato eseguito; non esiste alcun PASS/FAIL V12R12.

Verifiche di questa milestone:

- **suite pytest: 68/68 PASS in 6,16 s** con l'interprete canonico
  `/opt/anaconda3/envs/envCMC-rllib/bin/python` (3.10), dalla root del repo
  (`pytest -q -p no:cacheprovider`), esito che le sessioni precedenti non
  avevano registrato;
- **review avversariale multi-agente dei ~9.300 righe di sorgente**
  (6 dimensioni di review + uno scettico per ogni finding, 40 agenti):
  **32 finding confermati (0 critical, 3 major, ~15 minor, ~14 info),
  0 incerti, 2 refutati**;
- **nessun blocker live**: tre audit indipendenti campo-per-campo dei confini
  P0→P1, P1→P2 e P2→P3 non hanno trovato alcun mismatch
  produttore/verificatore attivo.

**Raccomandazione: GO condizionato al freeze** — prima di pubblicare freeze e
lock conviene applicare la lista di fix sotto, perché il namespace non è
ancora congelato (i fix ora costano zero) mentre ogni esecuzione è one-shot e
quattro lineage precedenti sono state bruciate esattamente da difetti di
questa classe.

## Cosa è V12R12

Protocollo `AB06_H0_V12R12_V26_INVARIANT_SAFE_TEACHER` (schema 1212), la
lineage V26-only progettata come successore vincolante dopo i terminal FAIL
V12R10 (fit W1024 legacy-target) e V12R11 (H0 diretto su V26):

- actor **W256 mascherato** `35→256→256→(2 mean + 2 logstd)` tanh: colonne
  clock 0–1 e colonne 10:25 azzerate bit-exact, il mean dipende solo dalle
  **18 colonne invarianti 2:10 e 25:35**;
- teacher = **tape sicuri congelati** `V8R1P1_V26_SAFE_TAPE_6X500_FROZEN_MEAN`
  (6 casi × 500 step del 07/08, `frozen_teacher_mean`), mai H0 live nel path
  di rollout;
- pipeline **P0** (fit su 3.000 righe tape) → **P1 tape-dagger
  candidate-exposed** (18 rollout = 6 casi × alpha 0,25/0,50/0,75, shield di
  supporto p99 18D con soglia riderivata a ogni preflight, latch penetrazione
  15/10 mm) → **P2** refit su 12.000 righe → **P3 development puro** 6 casi
  risk-first con `deterministic_offset_plus_0p20` per primo;
- gate offline ~10× più severi dei predecessori (RMSE globale ≤2,5e-4, max
  ≤3e-3, reset ≤1e-5, fold ≤2e-6);
- gate fisico P3 conforme ai `successor_requirements` di V12R11: 500 step
  esatti con `episode_time_limit`, ≥2 cicli, penetrazione `<0,025 m` stretta,
  zero clipping, 12 campi ZERO_INVALID, solo `binary_active_v26`.

## Esiti positivi della review (aree verificate pulite)

- **le 4 classi di bug storiche sono chiuse nei path di produzione**:
  R2 (writer condiviso: `artifact_records()` restituisce esattamente le 6
  chiavi attese), R7 (i contatori V26 sono letti dal mapping annidato
  `binary_phase_event_gate` dove il produttore li pubblica davvero), R8
  (tutti i confronti d'albero trasportano il record completo a 4 chiavi con
  `files`), R10 (il lock P0/P2 e il lock P3 non contengono campi dipendenti
  dall'occupazione del namespace);
- **masking 10:25 corretto**: `MASKED_COLUMNS=(0,1,*range(10,25))` = 17
  colonne mascherate, 18 attive, identico in fitter e contratto P1, con
  enforcement bit-exact (positive-zero) attraverso fit, save/reload e
  transplant;
- **gli 8 `EXPECTED_ARRAY_HASHES` del corpus P0 riprodotti bit-exact da uno
  script indipendente** (estrazione dai 6 trace.json del tape), il che
  conferma anche la semantica dello slice e delle transizioni;
- **tutti gli input esterni pinnati verificati su disco** (tape ledger
  `f8a1807d…`, lock v8r1p1 `2046f33c…`, lock V12R10 `e6601a67…`, tree H0
  `f7f6c898…`, training cfg, preflight/compatibility);
- soglia p99 `0,1937808123139821` **non hardcoded ciecamente**: l'envelope
  LOO viene riderivato a ogni preflight/lock/esecuzione e confrontato con
  atol 2e-15;
- semantica alpha/blend/latch conforme al design, con ricalcolo indipendente
  del blend atteso per ogni step;
- matematica del fold di normalizzazione corretta (andata/inversa con
  ri-azzeramento colonne mascherate); determinismo (seed 20260815,
  `use_deterministic_algorithms`, full-batch, nessuna best-state selection);
- letterali cross-modulo coerenti byte-per-byte (PROTOCOL_ID, run root
  `h0_v12r12_run_20260815`, status P2→P3, offset dei casi);
- governance P0/P2 e P3: no-clobber `O_EXCL`+fsync ovunque, rifiuto
  symlink/reparse, scritture confinate al namespace, ack esatti
  (`V12R12_FREEZE_REVIEWED_SOURCES`, `V12R12_ONE_SHOT_FIXED_FIT`,
  `V12R12_P3_FREEZE_AFTER_P2_PASS`, `V12R12_P3_ONE_SHOT_NO_RETRY`).

## Finding confermati da correggere prima del freeze

### Major (3)

1. **Gate P3 sulla std più stretto del runtime** —
   `h0_v12r12_physical_development.py:936` esige `|std−0,005| ≤ 1e-9` mentre
   il runtime che la produce accetta `1e-8`: finestra `(1e-9, 1e-8]` di FAIL
   terminale spurio per pura aritmetica float32. Su questa macchina il valore
   misurato dista `1,1e-10` (nessun falso negativo oggi), ma il rischio è
   latente/cross-platform. Fix: allineare la tolleranza al runtime.
2. **Nessun test producer-vs-verifier P1→P2** — le fixture riproducono la
   proiezione che il verificatore si aspetta e il test P2 monkeypatcha
   `_load_p1_candidate_exposed`: l'anti-pattern esatto che non intercettò il
   bug R8. Fix: test d'integrazione che generi gli artefatti P1 col vero
   producer e li dia in pasto al vero verificatore del fitter.
3. **Fixture verifier-shaped anche per P2→P3 e per il produttore fisico P3**
   (`_fake_p2`, `_passing_summary`/`_trace_row`): stesso punto cieco. Fix
   analogo, più un test del lock verificato **dopo** l'occupazione del
   namespace (lezione R10).

### Minor rilevanti (selezione)

- `_canonicalize_model_mask` è un **no-op** (advanced indexing su copia:
  `weight[:, list(MASKED_COLUMNS)].zero_()` non modifica il tensore) e
  l'audit del summary pubblica il campo **fattualmente falso**
  `input_mask_recanonicalized_after_each_update: true` — inaccettabile in un
  artefatto forense congelato (fitter:1936 e :2423);
- P0/P2: `run_production_fit` cattura solo `Exception` — un
  KeyboardInterrupt/SystemExit durante il fit lascia il namespace claimed
  **senza ledger terminale** (P1 e P3 catturano `BaseException`);
- il payload del lock P1 include `p1_destination_unoccupied` ricomputato
  dall'occupazione corrente (firma della classe R10; oggi inerte perché ogni
  call-site verifica a namespace vuoto, ma una verifica post-run fallirebbe
  spuriamente);
- **tre implementazioni divergenti di `_tree_record`** (fitter: sort su
  stringa POSIX; P1/P3: sort su oggetti `Path`): identiche solo su alberi
  piatti; una subdir futura in `save_to_path` produrrebbe `tree_sha256`
  diversi per lo stesso albero, cross-confrontati in modo stretto tra stadi;
- **4 check tautologici**: `support_exact`, `latch_exact`, `target_exact`
  (P1, runner:751) e `risk_first` (P3 aggregate:1137) confrontano un valore
  con se stesso — i relativi contatori di violazione non possono scattare;
- governance P1: freeze/lock **senza ack** (a differenza di P0/P2/P3), ack di
  execute booleani anziché stringhe, `main()` senza gestione eccezioni (exit
  code incoerenti), ledger/receipt P1 che non bindano freeze/lock;
- `__init__.py` non compare in nessuna delle tre source closure;
- gate fold 2e-6 valutato dopo il claim del namespace (un fail numerico
  occuperebbe la one-shot);
- vari info: lock P1 blinda hash dichiarati non cross-platform; finestre
  pre-claim senza evidenza terminale; `end_reason` null nel summary
  early-close; schema_version P1 `12121` vs `1212` non documentato; gap di
  copertura minori (prefisso troncato P1, negativi sui contatori V26
  annidati P3).

I 2 finding refutati (predicato occupancy P1 come rischio di produzione;
costo dei 19 ricalcoli di source_snapshot) sono documentati nell'output del
workflow con le motivazioni degli scettici.

## Metodo

Review eseguita con workflow multi-agente: 6 reviewer paralleli per
dimensione (bug di proiezione produttore/verificatore, matematica del
masking/fit, governance one-shot, logica P1 tape-dagger, gate P3, adeguatezza
test + coerenza trasversale), ogni finding sottoposto a uno scettico
avversariale indipendente istruito a refutarlo leggendo il codice reale;
severità ricalibrate dagli scettici. Tutte le letture read-only; nessun file
del namespace modificato.

## File coinvolti

Nessun file modificato in questa milestone. Documentati:

- `Trajectory Generator/baseline_MLP/validation/v12r12/` (10 sorgenti, stato
  source-only);
- questo report in `reports/user/`.

## Test e verifiche

- pytest v12r12 (3 suite): 68/68 PASS, interprete canonico, cache pytest
  disabilitata (nessuna scrittura persistente);
- review: 40 agenti, 32 finding confermati / 2 refutati / 0 incerti;
- hash input esterni ricalcolati e combacianti;
- nessuna esecuzione di freeze, lock, fit, rollout o training.

## TODO

- [x] Applicare i fix pre-freeze (3 major + minor elencati) — vedi addendum.
- [x] Aggiungere i test d'integrazione producer→verifier — vedi addendum.
- [x] Rieseguire la suite completa e un re-audit dei punti toccati — vedi
  addendum.
- [ ] Freeze + lock P0/P2 (ack `V12R12_FREEZE_REVIEWED_SOURCES`) ed
  esecuzione one-shot `--execute --stage p0` secondo la sequenza
  preregistrata nei runner.
- [ ] Q3, checkpoint-zero e Morphology Corridor restano chiusi fino al
  terminal PASS fisico P3.

---

# Addendum (stessa giornata) — fix pre-freeze applicati e verificati

## Fix applicati

`h0_v12r12_masked_teacher_fitter.py`:

- `_canonicalize_model_mask` ora azzera davvero in-place
  (`weight.index_fill_(1, …, 0.0)`): la ri-canonicalizzazione per-update è
  reale e il campo d'audit `input_mask_recanonicalized_after_each_update`
  diventa veritiero senza modifiche;
- `run_production_fit` cattura `BaseException`: `failure.json` viene
  pubblicato anche su KeyboardInterrupt/SystemExit, che poi propagano
  inalterati;
- `__init__.py` aggiunto alla source closure (`_governance_source_records`);
- doppia guardia anti-sottodirectory sugli alberi candidato (core a 3 file e
  schema a 5 file): qualunque entry non-file fa fallire con messaggio
  esplicito invece di essere silenziosamente ignorata;
- i letterali `500/3/18/3000` del loader P1→P2 sono derivati dalle costanti
  (`ROWS_PER_CASE`, `len(P1_ALPHAS)`, ecc.) — identici ai valori di
  produzione, ma ora il loader è testabile in miniatura;
- il loader verifica il nuovo binding governance: receipt e ledger P1 devono
  contenere i record byte-esatti di freeze ed execution lock P1.

`h0_v12r12_tape_dagger.py` + `run_h0_v12r12_tape_dagger.py` (P1):

- `_tree_record` unificato all'algoritmo del fitter (sort per stringa POSIX)
  e check symlink allineato su tutte le entry;
- il payload dell'execution lock non dipende più dall'occupazione corrente
  del namespace (campo statico `p1_destination_unoccupied_at_lock_time`,
  attestato da `write_execution_lock` prima della pubblicazione): il lock
  resta verificabile dopo l'esecuzione (lezione R10 chiusa anche qui);
- i tre check tautologici sono ora ricalcoli indipendenti: `support_exact`
  riesegue `support.query(actor)`; `latch_exact` riapplica
  `advance_safety_latch` sugli input causali catturati prima della select
  (`SafetyLatchState` è frozen: doppia chiamata deterministica);
  `target_exact` attesta identità caso/step del reference — i contatori
  `support_decision_mismatch_count`/`latch_rule_violation_count`/
  `target_provenance_mismatch_count` ora possono davvero scattare;
- receipt e ledger P1 bindano freeze ed execution lock; i payload sono
  estratti nei builder puri `build_p1_ledger_payload`/`build_p1_receipt_payload`
  (usati da `execute_p1` e testabili);
- CLI riallineata al pattern P0/P2: ack a stringa esatta
  (`V12R12_P1_FREEZE_REVIEWED_SOURCES` per freeze/lock,
  `V12R12_P1_ONE_SHOT_18_ROLLOUTS_9000_STEPS` per `--execute`), `main()` con
  JSON su stderr ed exit 2 su errore. **I vecchi flag booleani
  `--ack-one-shot-no-retry`/`--ack-18-rollouts-9000-steps` non esistono più.**

`h0_v12r12_physical_development.py` + runner (P3):

- tolleranza std del trace audit allineata al runtime: `1e-9 → 1e-8`
  (chiude la finestra di FAIL spurio; il valore reale dista ~1,1e-10);
- `risk_first` è data-driven: attesta che `case_bindings[0]` osservato sia il
  discriminatore `+0.20`;
- `tree_record` unificato; `end_reason` null nell'early-close produce il
  segnaposto `physical_runtime_exception`.

## Nuovi test (68 → 82)

- **`test_p1_real_builders_roundtrip_through_p2_loader`** (lezione R8): i
  produttori reali (`build_p1_corpus`, i due builder payload, record
  forensi) generano un namespace P1 in miniatura (6 casi × 3 alpha × 2 step)
  che il vero `_load_p1_candidate_exposed` carica e valida end-to-end; tre
  negative indipendenti (lock manomesso, freeze manomesso, record freeze
  corrotto nel solo ledger con receipt riallineato) provano che ogni ramo
  del binding governance morde;
- **`test_p1_naming_and_layout_lockstep_between_contract_and_fitter`**: lega
  sui valori REALI (zero monkeypatch) naming, alpha-tag, trajectory id,
  layout `collections/…`, path canonici e row count fra contratto P1 e
  fitter — una rinomina unilaterale non può più sopravvivere alla suite;
- `test_execution_lock_payload_is_occupancy_independent` (lezione R10);
- `test_collection_gate_rejects_physically_truncated_prefix` (lezione R7);
- `test_development_gate_rejects_each_nested_v26_counter_drift` (5 contatori);
- `test_aggregate_gate_risk_first_is_data_driven`;
- `test_p1_npz_rejects_identity_journal_field_drift`
  (case_ids/trajectory_ids/step_indices);
- test CLI P1 aggiornati agli ack a stringa esatta (execute + governance).

## Verifica avversariale dei fix

Workflow con 4 verificatori indipendenti istruiti a refutare i fix:
**fitter CLEAN, P3 CLEAN**; sul perimetro P1 e sui test sono emerse 1 major
e 5 minor, tutte risolte o accettate:

- [risolta] major: lockstep naming contract↔fitter non vincolato dai test →
  aggiunto il test statico di congruenza;
- [risolta] minor: negative del binding governance limitato al primo ramo →
  estesi a freeze-side e ledger-side;
- [risolta] minor: check symlink del `_tree_record` di contratto più lasco →
  allineato;
- [accettata] minor: la clausola array di `target_exact` confronta lo stesso
  oggetto del lookup (potere discriminante residuo coperto da identità
  caso/step + pin hash del tape);
- [accettata] minor, pre-esistente: il lock P1 blinda anche gli hash
  intermedi dichiarati non cross-platform (stabile sul flusso single-machine
  macOS; eventuale port Windows richiederà lock separato);
- [accettata] minor: nel roundtrip gli artefatti per-caso (journal/trace/
  summary/gate) restano fixture manuali — il layer per-caso è coperto dal
  test di ricostruzione e dall'audit per ispezione; la chiusura totale
  richiederebbe `run_collection_case` con env fittizio (futuro).

## Stato finale

- suite: **82/82 PASS** (interprete canonico `envCMC-rllib`);
- `ruff check` e `ruff format --check`: PASS su tutti i file del namespace;
- nessun freeze/lock/artefatto canonico creato: il one-shot è intatto;
- **il namespace è pronto per il freeze**. Sequenza aggiornata:
  1. fitter `--write-protocol-freeze` / `--write-execution-lock` con
     `--acknowledge-governance V12R12_FREEZE_REVIEWED_SOURCES`;
  2. fitter `--execute --stage p0 --acknowledge-one-shot
     V12R12_ONE_SHOT_FIXED_FIT`;
  3. P1 `--write-protocol-freeze` / `--write-execution-lock` con
     `--acknowledge-governance V12R12_P1_FREEZE_REVIEWED_SOURCES`, poi
     `--execute --acknowledge-one-shot
     V12R12_P1_ONE_SHOT_18_ROLLOUTS_9000_STEPS`;
  4. fitter `--execute --stage p2`; poi P3 secondo i propri ack.
