# Daily Report - 2026-08-19

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md`
- `2026-08-19_verdetto_lineage_nativa_v26_a_200_confronto_giugno.md`

## Sintesi

Su richiesta dell'utente (iterazioni a 36 min contro gli 8–15 di giugno)
l'indagine esclude costo per step, stati costosi, learner e config, e trova
nei log Ray una **tempesta di crash silenziosi** (1.044 errori off-grid,
639 file worker, ~30 morti/iterazione) causata da un **mio bug** nel
percorso `reject_continue` (cursore sensori avanzato senza commit delle FSM
→ `previous_time_s off-grid` allo step dopo). Corretto con il drop-mode
nell'adapter (commit dei candidati window-consistent) e un unico percorso
di commit nell'env; la regressione reale ha scoperto un **secondo bug
latente** (microstep degenere all'orizzonte, tolleranza 1e-12 vs accumulo
float), corretto col test dell'orizzonte a mezza finestra. Certificazione:
7,7 min/iter, 0 off-grid, primi episodi V26 a 500 step pieni. Revisione
onesta del verdetto smoke. Continuazioni 132→150→200 pulite: best 196,4,
poi plateau; **equivalenza con giugno non raggiunta** (65 % return, 84 %
len) per storia sporca della lineage; rollout del best a orizzonte pieno
(return 210,7) con ginocchio smorzato e caviglia scarica. Raccomandato il
fresh run con tutti i fix.

## Lavoro svolto

- `binary_phase_adapter.py` (`invalid_event_mode="drop"`),
  `binary_phase_adapter_v26.py` (passthrough), `osim_trj_cmc_like.py`
  (commit unico; orizzonte robusto); test di regressione "assorbe e
  sopravvive"; suite 6/6.
- Strumenti: probe per-step, watcher di fase, analisi timer tensorboard.
- Corse: verify3 (7,7 min/iter), cont150, cont200_final; rollout + 7 plot
  del best 200 (`plot/08_20_2026_1_imitation_native_v26_iter162_best`).

## Test e verifiche

- Suite invalid-event 6/6; regressione probe 2×500 step; 0 off-grid su
  tutte le sessioni Ray post-fix; durate iterazione da tensorboard.

## TODO completati

- [x] Causa delle iterazioni lente trovata e corretta; requisito 8–15 min
  certificato.
- [x] Continuazione a 200 e confronto formale coi criteri di giugno.
- [x] Rollout + 7 plot del best 200.

## TODO aperti e propagati

- [ ] Decisione fresh run vs reward shaping (poi: fresh, 20/08).
- [ ] Fix probe (condizionato alla migrazione di schema, vedi 18/08).
- [ ] Passthrough invalid-policy in `rollout_eval`.
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).
