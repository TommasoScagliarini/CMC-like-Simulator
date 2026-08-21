# Piano di test per il redesign FSM attore v3 (resync, cancellazione HS, bootstrap phase-aware)

Data: 2026-08-21 — stato: PROPOSTA (nessun codice toccato)

## Oggetto

Rendere l'architettura detector binario + FSM attore robusta a start in fase
casuale del ciclo e a sequenze di eventi non nominali (rimbalzo HS, TO
precoce, mid-cycle start), senza toccare il detector V26 (geometria, eventi,
conferma 5 ms). Tre meccaniche concordate:

1. **Resync su rigetto**: dopo un evento droppato, dwell di osservazione del
   contatto fisico (50–100 ms); se contraddice lo stato FSM, transizione
   accettata in ritardo come *degradata* (credito ridotto) invece del
   timeout a 2,2 s.
2. **Cancellazione HS rimbalzato**: HS seguito da stacco entro il minimo di
   stance → "HS annullato" (ritorno a swing), non "TO rigettato".
3. **Bootstrap phase-aware**: a t0 le regole HS-relative si ancorano a una
   fase stimata (contatto + gait clock), non a t0 come heel strike.

Governance: nuovo contract id `binary_point_v25+heel_qualified_fsm_v3`;
la v2 resta selezionabile (riproducibilità dei risultati B0820).

## Principio: separare la FSM dalla policy

Il difetto da misurare è della FSM, non della policy. Ogni livello usa
quindi almeno una sorgente di gait "perfetta o registrata" in cui la FSM è
l'unica variabile: cinematica prescritta (nastro AB06) e trace registrati.
La policy entra solo nell'ultimo livello.

## Ambiente di test a 4 livelli (dal più veloce al più realistico)

### L0 — Unit test della FSM isolata (ms, deterministici)

Riuso: `validation/test_prosthetic_phase_fsm_two_sensor.py`,
`test_prosthetic_phase_fsm_high_rate.py`, `test_binary_phase_fsm_v26.py`,
`test_binary_phase_active_adapter_v26.py` (non-regressione v2).
Nuovi (`validation/test_prosthetic_phase_fsm_v3_resync.py`), sequenze
sintetiche di campioni/eventi:

- ciclo nominale HS→TO→HS: v3 identico a v2 (stessi stati, crediti, payload);
- **rimbalzo HS** (HS, stacco a 20/40/49 ms): v2 → TO rigettato + lock;
  v3 → HS annullato, ritorno in swing, nessun lock, credito HS revocato;
- **TO precoce reale** (HS valido, TO a 30 ms, poi aria sostenuta): v3
  accetta TO degradato entro il dwell; stato coerente col contatto;
- **aria transitoria breve** (sotto il dwell): nessun resync spurio
  (il dwell deve filtrare i glitch);
- **start mid-ciclo** in 8 fasi (stance 10/40/70/95 %, swing 10/40/70/95 %):
  il primo evento fisico successivo viene accettato, nessun falso
  `to_too_early_after_hs`;
- timeout stance/swing: scattano ancora quando NESSUN contatto contraddice
  la FSM (la v3 non deve rendere i timeout irraggiungibili);
- invarianti latch/fase (oggi BinaryPhaseTransferError) mai violate lungo
  tutte le sequenze sopra (fuzz: 10k sequenze casuali di campioni binari);
- **monotonia dei crediti**: una transizione degradata non vale mai più
  della nominale; cicli con resync non completano bonus pieno;
- **v3 con meccaniche disattivate ≡ v2** byte-per-byte sui payload.

### L1 — Replay offline su trace registrati (minuti)

Riuso del pattern `binary_phase_fsm_v26_v7_replay_receipt.json`. Input:
`rollout_policy_trace.json` (campioni `detector_sensors` per step, payload
`phase_fsm`, `online_events`) di: qualifica B0820 ×3 start, baseline
full-horizon, ex-novo best, pilot 15/07. Procedura: ri-eseguire
adapter + FSM v2 e v3 sugli stessi campioni e confrontare.

Gate:
- sui segmenti in cui v2 non ha rigetti: **v3 identica** (evento per evento,
  stato per stato);
- sui casi di lock noti (qualifica ×3, ex-novo best): v3 risincronizza
  entro il dwell e produce i cicli che la fisica mostra (contatto
  registrato); metrica primaria **tempo-in-desync** (FSM vs contatto)
  da ~2,2 s a ≤ dwell;
- journal completo e JSON stretto (stesso receipt della v26).

### L2 — Env reale a cinematica prescritta, sweep di start (ore)

Riuso: `validation/test_phase_fsm_prescribed_env.py` e gli strumenti
`*_prescribed_*`. Il nastro AB06 è il "gait perfetto": qualunque fallimento
è imputabile alla FSM.

- sweep di **20 offset di start** distribuiti su un ciclo intero + i 3
  start esatti ex-novo, FSM v2 vs v3;
- gate: con v3, cicli qualificati al **100 %** degli start, zero timeout,
  zero transfer error; con v2 si documenta il tasso di fallimento attuale
  (baseline del confronto);
- perturbazioni controllate del nastro (rimbalzo HS iniettato sui campioni,
  contatto intermittente breve): v3 recupera entro il dwell.

### L3 — Env reale con la policy (la realtà)

- rollout deterministici della baseline B0820 ai 3 start esatti + 10
  offset casuali, v2 vs v3: gate = nessuno stance-lock; fine episodio per
  orizzonte o ragioni genuine; conteggi di resync/cancellazioni loggati;
- **preflight smoke** 3–5 iterazioni ex-novo con v3 (stessa ricetta dello
  smoke B0820 v2 come controllo): gate = `phase_timeout_stance` e
  `morphology_causal_contract_failure` giù di almeno il 50 % rispetto al
  controllo v2 (24/26 alla prima iterazione), 0 crash, interleaving
  exact-start intatto, KL quieto, telemetria corridoio con segmenti
  completati > 0.

## Criteri di accettazione complessivi (dichiarati prima del codice)

| Livello | Criterio |
|---|---|
| L0 | 100 % pass; identità v2 a meccaniche spente; fuzz 10k senza violazioni |
| L1 | zero diff sui segmenti senza rigetti; desync ≤ dwell sui lock noti |
| L2 | 100 % start con cicli qualificati, zero timeout (nastro perfetto) |
| L3 | niente stance-lock con la policy; muri −50 % nello smoke; 0 crash |

Un FAIL a qualunque livello blocca la promozione di v3 (nessun training).

## Cosa si riusa / cosa è nuovo

Riuso: 4 suite FSM/adapter esistenti, pattern replay+receipt v26, env
prescritto, rollout con `--episode-start-offset-s`, trace già registrati.
Nuovo: suite L0 v3, replayer v2/v3 con diff (L1), sweep di start (L2/L3).

## Ordine e costo stimato

1. Design delle transizioni di stato v3 (documento + diagrammi) — 0,5 g
2. L0 (scritti PRIMA dell'implementazione, stile TDD) + implementazione — 1 g
3. L1 replay + receipt — 0,5 g
4. L2 sweep prescritto — 0,5 g
5. L3 rollout + smoke — 0,5 g (macchina)

Totale ~3 giorni, di cui ~1 di macchina.
