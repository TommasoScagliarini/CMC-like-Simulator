# Daily Report - 2026-08-10

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
