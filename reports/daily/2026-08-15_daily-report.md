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
