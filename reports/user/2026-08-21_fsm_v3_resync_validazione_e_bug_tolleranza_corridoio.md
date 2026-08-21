# FSM attore v3 (resync + cancellazione HS): validazione L0–L3 e scoperta del bug di tolleranza del corridoio causale

Data: 2026-08-21 — stato: L0–L3 PASS; decisioni utente applicate (v3 default, candidato V26 promosso post-Q2, nessun training lanciato)

## Problema

Ai tre start esatti dell'ex-novo la baseline imitativa andava in
stance-lock: un evento invalido (TO a 147 ms dall'HS, oppure stance di
0,33 s con carico medio 0,06 BW sotto il gate `min_stance_load 0,04 BW·s`)
veniva rigettato e la FSM attore v2, priva di vie di recupero, restava in
STANCE fino al timeout di 2,2 s → episodio morto. L'utente ha chiesto di
rendere robusta l'architettura detector binario + FSM invece di aggirare il
difetto con un fine-tune della policy.

## Soluzione

Design approvato (`reports/plans/2026-08-21_design_fsm_v3_resync.md`):

- **A — Resync**: dopo ≥ 80 ms di contraddizione tra stato FSM e **latch
  funzionale di stance del detector** (verità corretta dopo L1: il contatto
  di sola punta è swing legittimo nel contratto V26), transizione degradata
  (`toe_off_resync`/`heel_strike_resync`): zero crediti, segmento invalido,
  ciclo senza bonus, nessuna doppia penalità.
- **B — Cancellazione HS rimbalzato**: TO entro `min_stance_duration_s` da
  un HS "giovane" revoca l'HS (`heel_strike_cancelled`), penalità
  invariata, FSM coerente.
- **C — Bootstrap**: verificato già robusto in v2 (nessuna modifica).
- Osservazioni byte-identiche; a flag spenti v3 ≡ v2 (golden pre-modifica);
  selezione via `binary_phase_actor_fsm_version` (env, yaml, trainer,
  rollout); detector e contratto eventi intoccati.
- Consumatori adeguati: adapter (passa il latch), validatore v26 e ledger
  retrospettivo (scarto ordinario sui repair), **ledger causale con ri-armo**
  (scarta pendenti, azzera ancore, allineamento sospeso fino alla prima
  ancora reale; eventi sconosciuti ancora fail-closed).

## Bug preesistente scoperto (indipendente dalla v3)

`experimental_morphology_corridor.py` usava `_TIME_EPS = 1e-12` per
confrontare il tempo env con gli onset dei candidati pendenti del detector:
due accumulatori float indipendenti che derivano di ~1e-12 nell'episodio
(misurato: 17.076870983804135 vs 17.07687098380528). Ogni candidato
pendente coincidente con un confine diventava `invalid_pending_transition_time`
→ `morphology_causal_contract_failure` → episodio terminato. Spiega il
~43 % di terminazioni per contract failure nello smoke v2 e verosimilmente
gran parte delle 298 del training B0820 da 50 iterazioni. Fix: 1e-9 (come
il resto dello stack V26) + test di regressione.

## Validazione (piano `2026-08-21_piano_test_fsm_v3_resync.md`)

| Livello | Esito |
|---|---|
| L0 unit | 23/23; golden v2 byte-identico; suite preesistenti 65/65; live 36/36 |
| L1 replay 7 trace | replay v2 fedele 7/7 (match 1,0); v3 ≡ v2 sui puliti; desync 1,9–2,1 s → ≤ 0,08 s sui lock |
| L2 sweep prescritto | 23 start × 2 versioni: 0 errori, 0 timeout, 0 invalidi; v3 ≡ v2; 100 % cicli (orizzonte 4,5 s) |
| L3a baseline ai 3 start | **3/3 a orizzonte pieno** (`episode_time_limit`): eventi assorbiti 2/5/5, penetrazione max 13,6/17,1/22,1 mm < 28, nessun lock, nessun contract failure |
| L3b smoke ex-novo v3 | contract failure +0, stance timeout +0, episodi a orizzonte +20 in 2 iter, 0 worker morti |
| L3b controllo v2 (stesso corridoio corretto) | contract failure +5, stance timeout +12, episodi a orizzonte +7 — la sola tolleranza non basta: serve la combinazione v3 + tolleranza per azzerare entrambi i muri |

## File modificati/creati

- FSM: `Trajectory Generator/prosthetic_phase_fsm.py` (config v3, resync,
  cancellazione, payload); adapter `binary_phase_adapter.py` (latch
  passthrough, validatore config tipizzato); env `osim_trj_cmc_like.py`
  (chiave versione + flag FSM); `training_config.py`, `train_ppo_mlp.py`,
  `rollout_eval.py` (flag/propagazione; trace con `morphology_causal_diagnostics`);
  corridoio `experimental_morphology_corridor.py` (repair whitelist,
  ri-armo causale, `_TIME_EPS` 1e-9).
- Test: `validation/test_prosthetic_phase_fsm_v3_resync.py`,
  `validation/test_causal_ledger_fsm_repair.py`,
  `validation/fsm_v3_scenarios.py`, golden in `validation/fixtures/`.
- Strumenti: `validation/replay_actor_fsm_v3_on_traces.py` (+ receipt),
  `validation/sweep_fsm_v3_prescribed_starts.py` (+ receipt).

## Decisioni da prendere

1. **Promozione v3** come default dei training ex-novo (config canonico
   `binary_phase_actor_fsm_version: v3`) — raccomandata dai dati.
2. **Governance del corridoio**: il test di readiness pretende il canonico
   con corridoio dormiente (`morphology_weight 0`) finché il candidato V26 è
   `q2_blocked`; la promozione B0820 nel canonico (scelta b del 20/08) l'ha
   messo a 0,0025 → test rosso. Opzioni: (i) canonico dormiente + peso
   solo nello snapshot di lineage; (ii) promozione formale del candidato
   V26 (status, autorizzazioni, test) — ora sostenuta da L3.
3. Rilancio del training ex-novo B0820 con v3 + corridoio corretto: i 50
   update precedenti sono stati fatti con episodi troncati dal bug di
   tolleranza; la ripetizione è la misura onesta della ricetta.

## TODO

- [x] Esito +0,20 (L3a): orizzonte pieno.
- [x] Controllo v2 su corridoio corretto (L3b): +5 contract failure, +12 stance timeout — attribuzione chiusa.
- [x] Decisione 1: v3 default (EnvCfg, CLI trainer/rollout, canonico `binary_phase_actor_fsm_version: v3`; snapshot B0820 esplicitamente v2).
- [x] Decisione 2 (opzione ii): candidato V26 promosso post-Q2 (`status`, autorizzazioni, `morphology_causal_allow_effects 1.0`, peso 0,0025) e test di governance riscritto sul contratto promosso (guard Q2 runtime preservato).
- [x] Decisione 3: nessun training lanciato.
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s.
