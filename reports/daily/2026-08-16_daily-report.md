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
- Commit `22dae86b` (lineage R13–R16) e `cb62b56` (.gitattributes v12 +
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
