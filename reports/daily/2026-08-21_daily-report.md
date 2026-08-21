# Daily Report - 2026-08-21

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md`
- `2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md`
- Piani: `reports/plans/2026-08-21_piano_test_fsm_v3_resync.md`,
  `reports/plans/2026-08-21_design_fsm_v3_resync.md`

## Sintesi

Il training ex-novo B0820 da 50 iter è pulito ma **piatto** (return −18 →
−35, muri costanti); il best allo start nominale è identico al
pre-training. Riformulazione: è l'esito tecnico del pilot del 15/07, il cui
H0 però già camminava agli start esatti grazie all'adattamento
supervisionato. L'utente rifiuta il tampone (fine-tune della policy) e
chiede robustezza architetturale: **FSM attore v3** (resync su
contraddizione col latch funzionale del detector, cancellazione HS
rimbalzato, bootstrap verificato), design approvato e validata su 4 livelli
— L0 23/23 con golden v2 byte-identico; L1 replay fedele 7/7 con desync
da ~2 s a ≤ 80 ms; L2 23 start × 2 versioni senza errori, v3 ≡ v2; L3 che
smaschera sotto la FSM il ledger causale del corridoio: ri-armo sui repair
e **bug preesistente di tolleranza temporale (1e-12 vs drift 1,1e-12 tra
due orologi float)**, verosimile causa del 43 % di contract failure anche
in v2. Esito: L3a 3/3 start a orizzonte pieno; L3b v3 contract failure +0,
stance timeout +0, +20 episodi a orizzonte (controllo v2 sullo stesso
corridoio: +5/+12/+7). Decisioni utente applicate: **v3 default**,
**candidato corridoio V26 promosso post-Q2** (test di governance riscritto,
guard runtime preservato), **nessun training lanciato**. Repo a zero test
rossi (57/57 + 101/102→102/102).

## Lavoro svolto

- `prosthetic_phase_fsm.py` (v3), `binary_phase_adapter.py` (latch,
  validatore tipizzato), `experimental_morphology_corridor.py` (repair
  whitelist, ri-armo causale, `_TIME_EPS` 1e-9, contatore repair),
  `osim_trj_cmc_like.py` / `training_config.py` / `train_ppo_mlp.py` /
  `rollout_eval.py` (chiave `binary_phase_actor_fsm_version`, default v3,
  telemetria `morphology_causal_diagnostics` nei trace).
- Test: `validation/test_prosthetic_phase_fsm_v3_resync.py`,
  `validation/test_causal_ledger_fsm_repair.py`, golden e scenari
  condivisi; readiness corridoio aggiornata al contratto promosso.
- Strumenti: `validation/replay_actor_fsm_v3_on_traces.py`,
  `validation/sweep_fsm_v3_prescribed_starts.py` (+ receipt).
- Config: canonico con v3; snapshot B0820 esplicitamente v2; candidato V26
  promosso. Figure corridoio: `plot/08_21_2026_1_*`, `plot/08_21_2026_2_*`.
- Commit presente in git: `e95a56c9` "new warm start with v26" (utente).

## Test e verifiche

- Unit: FSM/adapter/corridoio 102/102 dopo la promozione; live env 36/36;
  receipt L1/L2; L3 su env reale (rollout e smoke) con log Ray a 0 morti.

## TODO completati

- [x] Esito training ex-novo 50 iter (piatto) e diagnosi.
- [x] Design, test ladder e validazione L0–L3 della FSM v3.
- [x] Decisioni: v3 default; promozione candidato V26 (opzione ii).

## TODO aperti e propagati

- [ ] Training ex-novo B0820 con v3 + corridoio corretto (prima misura
  onesta della ricetta) — su decisione utente; valutare con quei numeri un
  regime lr/epoche più deciso sotto KL-guard.
- [ ] Follow-up v3: riportare lo swing timeout da 2,6 a 1,3 s.
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato.
- [ ] Fix probe `train_ppo_mlp.py:1408` solo con migrazione di schema
  dichiarata.
- [ ] Commit checkpoint del lavoro FSM v3 / corridoio / governance (verificare
  cosa resta fuori da `e95a56c9`).
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).
