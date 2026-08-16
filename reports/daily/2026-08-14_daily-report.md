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
