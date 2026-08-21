# Daily Report - 2026-08-20

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md`
- `2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md`
- `2026-08-20_runbook_catena_warm_start_imitativo_exnovo.md`
- `2026-08-20_risultato_training_ready_exnovo_B0820.md`

## Sintesi

**Audit di equivalenza** giugno ↔ fresh run: reward effettiva identica,
PPO identico, convenzione osservazioni identica (il bug del probe esisteva
già a giugno: modulo 31 su 35); due derive scovate e decise dall'utente —
soglie di penetrazione **20/28 mm** come giugno e profilo GRF applicato di
giugno (ripristinato da git; poi scoperto byte-identico a
`grf_correct_magnitude` dell'area ex-novo). **Fresh run june-equivalent**:
100/100 in 14,4 h, 0 off-grid, ultime 20 iter 257,4 vs 268,6 (96 %), len
494 vs 476, metà delle penetrazioni di giugno; rollout del best (iter 87)
return 363,8, orizzonte pieno, 7 figure piene, smorzamento del ginocchio
sparito; coppia SEA positiva alla caviglia in loading response spiegata
dalla heel-qualification e accettata come baseline. **Catena B0820**:
guardie di processo → terminazioni pulite (9/9 test), manifest actor,
critic warmup (5 iter, `vf_loss` 8,86→0,07), qualifica 3/3 fisica sicura
(stance-lock a ~3 s come limite), checkpoint-zero, corridor readiness 8/8,
preflight smoke pulito → **training-ready**. Runbook riproducibile della
catena; canonico `training_exnovo_cfg.yaml` promosso al contenuto B0820
(scelta utente b). Lanciato il training ex-novo B0820 da 50 iterazioni.

## Lavoro svolto

- `simulation_runner.py` (`OnlineGRFPenetrationLimitExceeded`),
  `osim_trj_cmc_like.py` (`grf_penetration_hard`, catch terminale
  BinaryPhaseTransferError), `binary_phase_adapter_v26.py` (3 raise
  tipizzati), `rollout_eval.py` (passthrough invalid-policy),
  `validation/test_process_guards_termination.py`.
- Config: `experimental_configs/exnovo_v26_B0820_chain.yaml`; canonico
  promosso; manifest actor nei moduli della baseline.
- Figure: `plot/08_20_2026_2_june_equiv_fresh_iter87_best`,
  `plot/08_20_2026_3_morphology_corridor_binary_v26` (+ script versionato
  `validation/plot_morphology_corridor_binary_v26.py`).

## Test e verifiche

- Suite guardie+policy 9/9; readiness corridoio 8/8; ruff; audit
  riproducibile da yaml/commit/pickle; SHA dei profili GRF.

## TODO completati

- [x] Smoke di equivalenza, fresh run 100 iter, confronto formale.
- [x] Rollout + 7 plot del best; catena di luglio sulla baseline nativa.
- [x] Passthrough invalid-policy in `rollout_eval`.
- [x] Decisione guardie di processo (approvata e implementata).

## TODO aperti e propagati

- [ ] Esito del training ex-novo B0820 da 50 iter (vedi 21/08).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato.
- [ ] Fix probe (condizionato alla migrazione di schema).
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).
