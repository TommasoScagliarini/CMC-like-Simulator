# V12R12 — fix pre-freeze applicati, testati e verificati avversarialmente

Data: 2026-08-16 (lavoro svolto nella sessione a cavallo del 15/08)

## Problema

La review avversariale pre-freeze del namespace
`Trajectory Generator/baseline_MLP/validation/v12r12/`
([report del 15/08](2026-08-15_h0_v12r12_stato_test_e_review_pre_freeze.md))
aveva confermato 32 finding (0 critical, 3 major) su un protocollo one-shot
fail-closed in cui un solo difetto del verificatore brucia l'intera lineage —
la classe di bug che ha già ucciso V12R2, V12R7, V12R8 e (in forma latente)
V12R10. I tre major: finestra di FAIL terminale spurio sulla tolleranza std
del gate P3 (1e-9 contro 1e-8 del runtime) e due punti ciechi di test
producer-vs-verifier (le fixture riproducevano la proiezione attesa dal
verificatore invece della struttura prodotta dal produttore — l'anti-pattern
esatto che non intercettò il bug R8). Tra i minor: una ri-canonicalizzazione
del mask che era un no-op con campo d'audit fattualmente falso, l'assenza di
ledger terminale su interruzione in P0/P2, un campo del lock P1 dipendente
dall'occupazione del namespace (firma R10), tre implementazioni divergenti di
`_tree_record`, quattro check tautologici e una CLI P1 senza ack a stringa
esatta.

## Soluzione

Tutti i finding sono stati chiusi (o formalmente accettati e documentati)
prima del freeze, con produttore e verificatore modificati sempre in
lockstep e con nuove guardie automatiche di regressione.

### `h0_v12r12_masked_teacher_fitter.py`

- `_canonicalize_model_mask`: `weight[:, list].zero_()` (no-op su copia) →
  `weight.index_fill_(1, tensor(MASKED_COLUMNS), 0.0)` in-place; il campo
  d'audit `input_mask_recanonicalized_after_each_update: true` diventa
  veritiero senza modifiche;
- `run_production_fit`: `except Exception` → `except BaseException`;
  `failure.json` è pubblicato anche su KeyboardInterrupt/SystemExit, che poi
  propagano inalterati;
- `__init__.py` incluso nella source closure di freeze/lock;
- guardie anti-sottodirectory sugli alberi candidato (set core a 3 file e
  schema a 5 file): entry non-file → errore esplicito;
- letterali `500/3/18/3000` del loader P1→P2 derivati dalle costanti
  monkeypatchabili (identici ai valori di produzione, loader ora testabile
  in miniatura);
- nuovo blocco di verifica: receipt e ledger P1 devono bindare byte-exact i
  record di freeze ed execution lock P1 (`_validate_referenced_artifact` +
  path canonico).

### `h0_v12r12_tape_dagger.py` + `run_h0_v12r12_tape_dagger.py` (P1)

- `_tree_record` unificato all'algoritmo del fitter (sort per stringa POSIX
  relativa) con check symlink su tutte le entry — i tre stadi producono ora
  digest identici per qualunque forma d'albero;
- lock P1: il campo dinamico `p1_destination_unoccupied` è sostituito dalla
  attestazione storica statica `p1_destination_unoccupied_at_lock_time`
  (l'inoccupazione resta provata da `write_execution_lock` prima della
  pubblicazione): il lock è verificabile anche dopo l'esecuzione;
- i tre check tautologici sono ricalcoli indipendenti: `support_exact`
  riesegue `prepared.support.query(actor)`; `latch_exact` riapplica
  `v10s_blend.advance_safety_latch` sugli input causali catturati prima
  della select (`SafetyLatchState` è frozen: doppia chiamata deterministica);
  `target_exact` attesta l'identità caso/step del reference — i tre
  contatori di violazione corrispondenti ora possono scattare davvero;
- ledger e receipt P1 bindano freeze+lock e sono costruiti dai builder puri
  estratti `build_p1_ledger_payload`/`build_p1_receipt_payload` (usati da
  `execute_p1` e direttamente testabili);
- CLI riallineata al pattern P0/P2: ack a stringa esatta
  (`V12R12_P1_FREEZE_REVIEWED_SOURCES` per i governance write,
  `V12R12_P1_ONE_SHOT_18_ROLLOUTS_9000_STEPS` per `--execute`), `main()`
  con JSON d'errore su stderr ed exit 2 (inclusa `ForensicRolloutError`).
  I vecchi flag booleani non esistono più.

### `h0_v12r12_physical_development.py` + runner (P3)

- tolleranza std del trace audit `1e-9 → 1e-8`, allineata al runtime che
  produce la std (valore reale misurato a ~1,1e-10 dal target: nessun falso
  negativo possibile su questa macchina, finestra latente chiusa);
- `aggregate_gate.risk_first` è data-driven: attesta che
  `case_bindings[0]` osservato sia il discriminatore `+0.20`;
- `tree_record` unificato; `end_reason` null nell'early-close produce il
  segnaposto `physical_runtime_exception`.

## Strategia

1. fix applicati file per file leggendo prima ogni sito indicato dalla
   review, con produttore e verificatore aggiornati nello stesso passaggio
   quando un formato cambiava (binding governance P1);
2. refactor minimo e semanticamente identico dei letterali del loader per
   rendere miniaturizzabile il confine P1→P2;
3. nuovi test di regressione ancorati alle quattro lezioni storiche;
4. verifica avversariale finale del diff con 4 verificatori indipendenti
   istruiti a refutare i fix;
5. le issue emerse dalla verifica risolte subito (1 major + 2 minor) o
   accettate con motivazione scritta (3 minor);
6. nessun freeze/lock/artefatto canonico creato: la one-shot resta intatta
   e il freeze è demandato ad autorizzazione esplicita.

## Nuovi test (suite 68 → 82, tutti PASS)

- `test_p1_real_builders_roundtrip_through_p2_loader` — chiusura della
  lezione R8: i produttori reali (`build_p1_corpus`, builder payload,
  record forensi) generano un namespace P1 in miniatura (6 casi × 3 alpha ×
  2 step) che il vero `_load_p1_candidate_exposed` carica e valida; tre
  negative indipendenti (lock manomesso, freeze manomesso, record freeze
  corrotto nel solo ledger con receipt riallineato) provano che ogni ramo
  del binding governance morde;
- `test_p1_naming_and_layout_lockstep_between_contract_and_fitter` — lega
  sui valori reali (zero monkeypatch) naming, alpha-tag, trajectory id,
  layout, path canonici e row count fra contratto P1 e fitter;
- `test_execution_lock_payload_is_occupancy_independent` (lezione R10);
- `test_collection_gate_rejects_physically_truncated_prefix` (lezione R7);
- `test_development_gate_rejects_each_nested_v26_counter_drift`
  (5 contatori annidati, lezione R7 lato P3);
- `test_aggregate_gate_risk_first_is_data_driven`;
- `test_p1_npz_rejects_identity_journal_field_drift`
  (case_ids / trajectory_ids / step_indices);
- test CLI P1 riscritti sugli ack a stringa esatta (execute + governance).

## Verifica avversariale dei fix

Workflow con 4 verificatori read-only (uno per perimetro) istruiti a
refutare: **fitter CLEAN, P3 CLEAN**; su P1 e test 1 major e 5 minor:

| Esito | Finding |
|---|---|
| risolta | major: lockstep naming contract↔fitter non vincolato → test statico di congruenza |
| risolta | negative governance limitato al primo ramo → estesi a freeze-side e ledger-side |
| risolta | check symlink `_tree_record` contratto più lasco → allineato |
| accettata | clausola array di `target_exact` senza potere discriminante aggiuntivo (coperta da identità caso/step + pin hash del tape) |
| accettata | pre-esistente: il lock P1 blinda hash intermedi non cross-platform (stabile single-machine macOS; un port Windows richiederà lock proprio) |
| accettata | artefatti per-caso del roundtrip ancora fixture manuali (layer coperto dal test di ricostruzione e da audit per ispezione; chiusura totale richiederebbe `run_collection_case` con env fittizio) |

## File modificati

Produzione (`Trajectory Generator/baseline_MLP/validation/v12r12/`):

- `h0_v12r12_masked_teacher_fitter.py`;
- `h0_v12r12_tape_dagger.py`;
- `run_h0_v12r12_tape_dagger.py`;
- `h0_v12r12_physical_development.py`;
- `run_h0_v12r12_physical_development.py`.

Test (stessa cartella):

- `test_h0_v12r12_masked_teacher_fitter.py`;
- `test_h0_v12r12_tape_dagger.py`;
- `test_h0_v12r12_physical_development.py`.

Non sono stati toccati: plugin C++, semantica SEA, GRF primaria, detector e
FSM V26, tape sicuri V8R1P1, checkpoint H0, namespace v12r2–v12r11 e tutti
gli artefatti storici. Nessun freeze, lock, run root o artefatto canonico
V12R12 è stato creato.

## Test e verifiche

- suite v12r12: **82/82 PASS** con l'interprete canonico
  `/opt/anaconda3/envs/envCMC-rllib/bin/python` (3.10), dalla root del repo,
  cache pytest disabilitata;
- `ruff check`: PASS; `ruff format --check`: PASS su tutti i file del
  namespace;
- verifica avversariale del diff: 4/4 verificatori completati, esiti sopra;
- immutabilità di SafetyLatchState (`@dataclass(frozen=True)`) verificata
  prima di introdurre la doppia chiamata del latch;
- hash pinnati degli input esterni non toccati dai fix (tree H0, tape
  ledger, lock V12R10): invarianti.

## Stato e prossimo passo

Il namespace V12R12 è **pronto per il freeze**; la one-shot è intatta.
Sequenza autorizzabile (ack aggiornati):

1. fitter: `--write-protocol-freeze` / `--write-execution-lock` con
   `--acknowledge-governance V12R12_FREEZE_REVIEWED_SOURCES`, poi
   `--execute --stage p0 --acknowledge-one-shot V12R12_ONE_SHOT_FIXED_FIT`;
2. P1: freeze/lock con `--acknowledge-governance
   V12R12_P1_FREEZE_REVIEWED_SOURCES`, poi `--execute
   --acknowledge-one-shot V12R12_P1_ONE_SHOT_18_ROLLOUTS_9000_STEPS`;
3. fitter `--execute --stage p2`;
4. P3: `--prepare-lock` (ack `V12R12_P3_FREEZE_AFTER_P2_PASS`) solo dopo il
   PASS P2, poi `--execute` (ack `V12R12_P3_ONE_SHOT_NO_RETRY`) col caso
   `+0.20` per primo.

Q3, checkpoint-zero e Morphology Corridor restano chiusi fino al terminal
PASS fisico P3.

## TODO

- [ ] Autorizzare ed eseguire il freeze e la sequenza one-shot
  P0 → P1 → P2 → P3.
- [ ] In caso di FAIL fisico P3, chiudere la lineage senza retry e valutare
  il successore con dati recovery aggiuntivi, come da protocollo.
- [ ] TODO amministrativi invariati dal daily 15/08: commit checkpoint del
  lavoro 09–15/08 con correzione `.gitattributes`, riconciliazione pin
  V20/V26 in `train_ppo_mlp.py`, chiarimento anomalia teacher retry
  (best==last==warm_start), DLL Windows.
