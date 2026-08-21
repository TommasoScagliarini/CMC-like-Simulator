# Daily Report - 2026-08-18

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-18_training_imitativo_nativo_v26_100iter_verdetto.md`
- `2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md`

## Sintesi

Il training imitativo nativo V26 completa 100/100 senza crash: tracking per
step ≈ 88 % di giugno, ma episodi dimezzati (~250 step) da un muro unico,
`phase_timeout_swing` (1,3 s senza HS heel-qualified), con tetto secondario
di eventi invalidi. Il rollout del best viene rifiutato dal validator di
schema (39 vs 43): **bug del probe** di `train_ppo_mlp` (dimensiona l'actor
saltando `make_cmc_env`), effetto benigno (persi solo `u_abs/u_saturated`,
ridondanti); rollout eseguito con bypass fedele e 7 plot prodotti
(`plot/08_18_2026_1_...`, fig. 3/4 vuote: nessun ciclo qualificato completo).
Avviata l'opzione (A): swing 2,6 s (via reward JSON, già plumbato) +
`reject_continue`; smoke 10 iter letto come PASS (len fino a 383, best 177)
e lanciata la continuazione a 200. *Nota a posteriori (19/08): il PASS
dello smoke era parzialmente un artefatto — gli episodi con evento invalido
uccidevano il worker invece di terminare.*

## Lavoro svolto

- Diagnosi empirica del mismatch di schema (diff dei nomi feature tra env
  di training e di rollout; larghezze dal pickle del modulo).
- Driver di rollout con bypass dichiarato del validator; plot canonici.
- `experimental_configs/reward_swing_timeout_2p6.json`; smoke + continuazione.

## Test e verifiche

- Rollout `ok: true` (return 136,9, fine per `phase_timeout:swing`,
  penetrazione max 14,8 mm); contatori del training letti dai jsonl reali.

## TODO completati

- [x] Verdetto training nativo 100 iter e confronto coi criteri di giugno.
- [x] Rollout del best + 7 plot.

## TODO aperti e propagati

- [ ] Verdetto onesto della leva (A) (poi rivisto il 19/08).
- [ ] Fix del probe `train_ppo_mlp.py:1408`: solo per training *fresh*
  con migrazione di schema dichiarata (decisione 20/08: non per i training
  comparabili con giugno, che condividono la stessa convenzione di slicing).
- [ ] Passthrough di `binary_phase_invalid_event_policy` in `rollout_eval`.
- [ ] A baseline raggiunta: catena di luglio.
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).
