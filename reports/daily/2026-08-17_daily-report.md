# Daily Report - 2026-08-17

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md`
- `2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md`

## Sintesi

V12R17 (matrice P3 a 5 casi) riconferma `+0.20` PASS ma il caso `−0.20`
puro **FAIL terminale a step 71** (penetrazione 25,557 mm): essendo uno
dei tre start di training, chiude la strada della de-scope. Il probe di
capacità falsifica W512 (≈ W256) e indica l'inizializzazione come leva. La
diagnosi richiesta dall'utente ("perché luglio riuscì e V26 no") conclude
che non è la stessa trasformazione: luglio attraversava un cambio di
dominio GRF a semantica eventi identica con teacher vivo e margini ampi;
oggi un cambio di semantica delle osservazioni senza teacher e con margini
sub-millimetrici. **Pivot**: su intuizione dell'utente, rifare il training
imitativo nativamente sotto V26 (reward, non label: immune al tetto delle
label). Autorizzata la modifica runtime «evento invalido → terminazione
episodio» (policy `terminate`) con test live, e lo smoke da 5 iterazioni
prima del run da 100. Commit `6ab6612f` (R17 + probe) e `68a62cd8`
(invalid-event policy + test + smoke + diagnosi).

## Lavoro svolto

- `binary_phase_invalid_event_policy` {raise, terminate, reject_continue}
  in env/config/trainer + `BinaryPhaseTransferError` tipizzato;
  `validation/test_binary_invalid_event_policy.py` (5 test, live sull'env
  reale).
- Lancio del training imitativo nativo V26 (100 iter, 13 worker, ricetta
  di giugno + V26, `terminate`).

## Test e verifiche

- Suite v12r17 84/84; catena P0→P2 riprodotta bit-exact; smoke 5 iter
  pulito; suite invalid-event 5/5.

## TODO completati o superseduti

- [x] Decisione su R18/fresh-init: **superseduta** dal pivot al retraining
  nativo (alternative R18-fresh e causal-teacher restano documentate, non
  eseguite).
- [x] Port Q3 → checkpoint-zero → morphology via lineage V12: superseduto
  (la catena si rifarà sulla baseline nativa).

## TODO aperti e propagati

- [ ] Verdetto del training imitativo nativo V26 a 100 iter e confronto
  con giugno sui criteri fissati (return, len, completion).
- [ ] A baseline raggiunta: catena di luglio sulla baseline nativa.
- [ ] Amministrativo (dal 15/08): pin V20, anomalia best==last, metrica
  `fit_p2` V12R3, deflessione SEA iniziale (storico).
