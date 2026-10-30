# Daily Report - 2026-08-10

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

> **Nota**: daily scritto retroattivamente il 2026-08-15 sulla base dell'audit
> forense degli artefatti; la sessione notturna 09→10/08 non aveva prodotto né
> report utente né daily. Ricostruzione completa in
> [Retrospettiva forense 09–10/08](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).

## Report utente consolidato

- [Retrospettiva forense — sessione notturna 09–10/08: V12R2 → V12R4](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md)
  (scritto il 15/08, copre le attività di questa giornata)

## Sintesi

La sessione è la coda notturna del 09/08 (20:06 → 05:30). Contrariamente a
quanto lasciato agli atti dal report V12R2 («pronta ma non eseguita»), la
one-shot V12R2 è stata eseguita la sera stessa. In cascata sono fallite
terminalmente quattro lineage esecutive; tre design freeze satellite sono
rimasti deferred; la bozza V12R5 è stata scritta all'alba e sarà eseguita solo
il 14/08.

| Lineage | Esito | Causa |
|---|---|---|
| V12R2 (run) | FAIL terminale a `probe_p0` | bug integrità forense (`run_start` drifted) — fail spurio, classe verificatore |
| V12R3 | FAIL terminale a `fit_p2` | gate `offline_metrics` (valori non persistiti); prima però **primo pure-probe P1 PASS del progetto** (500/500, 2 cicli, pen. 23,56 mm) |
| v12p1q | design freeze PASS, mai eseguita | bloccata dal fail del salvage |
| v12p1s (salvage P1) | FAIL terminale caso 2/6 a step 179 | FSM rifiuta `to_too_early_after_hs` in `STANCE_AFTER_HS` |
| V12R4 | FAIL terminale a `collect +0.20` | penetrazione GRF a step 212 (25,064 mm), 212 label recuperate |
| v12r4q2 / v12r4zero / v12r5q3 | design freeze / scaffold | mai avanzati (upstream FAIL) |

Eredità della sessione, riusata dalle lineage successive: corpus P2
(materializzato dal `fit_p2` bocciato di V12R3, SHA-256 `42a40869…`), 500
label nominali PASS di V12R4 e il prefisso `+0.20` da 212 label di V12R4.

Nessun training PPO, nessun update critic, nessuna modifica a plugin C++,
GRF primaria, detector/FSM V26 o semantica SEA.

## TODO aperti al termine della sessione (stato dell'epoca)

- [ ] Progettare ed eseguire una lineage che corregga il trade-off dei fit
  full-mean (→ diventerà V12R5 il 14/08).
- [ ] Mantenere Q2/Q3, checkpoint-zero e morphology chiusi fino a un candidato
  imitativo PASS.
- [ ] Chiarire quale metrica offline ha bocciato il `fit_p2` V12R3 (valori non
  persistiti — resta aperto anche al 15/08).

> Questi TODO sono stati poi superseduti dagli sviluppi del 14–15/08
> (V12R5→V12R11): vedere i daily `2026-08-14` e `2026-08-15`.

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

<!-- reconciled-report-todos-2026-10-05:end -->
