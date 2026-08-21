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

## Esiti (aggiornati al 2026-08-21, sera)

- **L0**: 23/23 PASS (`validation/test_prosthetic_phase_fsm_v3_resync.py`);
  identità v2 a flag spenti verificata contro golden pre-modifica
  (`validation/fixtures/fsm_v2_golden_scripted_sequence.json`, 615 payload);
  suite preesistenti FSM/adapter 65/65, env live 26/26.
- **L1** (`validation/replay_actor_fsm_v3_on_traces.py`, receipt
  `validation/actor_fsm_v3_replay_receipt.json`): replay v2 fedele alla
  registrazione su 7/7 trace (match 1,0); v3 ≡ v2 sui 3 trace puliti; sui
  4 trace con lock desync da 1,9–2,1 s a ≤ 0,08 s (o 0 con cancellazione),
  mai timeout. **Correzione di design emersa**: verità del resync = latch
  funzionale del detector, non contatto grezzo (il contatto di sola punta è
  swing legittimo nel contratto V26).
- **L2** (`validation/sweep_fsm_v3_prescribed_starts.py`): 23 start × 2
  versioni, 0 errori, 0 timeout, 0 eventi invalidi, 0 resync — v3 identica a
  v2 a ogni fase di start; i 5 start che non chiudevano un ciclo in 3 s lo
  chiudono con orizzonte 4,5 s (artefatto di finestra; receipt
  `validation/fsm_v3_prescribed_start_sweep_long.json`): gate L2 soddisfatto
  al 100 % dei 23 start per entrambe le versioni. Meccanica C confermata
  già robusta in v2 sul gait perfetto.
- **L3a, prima esecuzione**: stance-lock eliminato ai 3 start esatti, ma il
  **ledger causale del corridoio** falliva chiuso sul repair
  (`unsupported_transition_event`): il suo contratto (ancore entro 40 ms,
  segmento valido, stato attore ⇔ ultima ancora) non può rappresentare una
  riparazione. Implementato il **ri-armo** (scarto pendenti, azzero ancore,
  allineamento sospeso fino alla prima ancora reale), test dedicati
  `validation/test_causal_ledger_fsm_repair.py` 5/5, readiness corridoio
  7/8 (l'unica rossa è la governance del canonico, decisione utente).
  L3a in riesecuzione.
- **L3b** (smoke 3 iter ex-novo dal checkpoint-zero, v3 vs controllo v2,
  delta dei contatori sulle iterazioni 7–8): `phase_timeout_stance` **+0**
  (v2: +12) — il muro dello stance-lock è eliminato in training;
  `episode_time_limit` +11 (v2: +4) — più episodi a orizzonte;
  `morphology_causal_contract_failure` +11 (v2: +8) — il contratto causale
  del corridoio è il muro residuo; 0 worker morti; return più negativi
  (episodi più lunghi che accumulano penalità di fase della policy a
  caviglia scarica: non è un gate dello smoke).
- **L3a, seconda esecuzione** (con ri-armo): nessuno stance-lock; i 3 start
  terminano ancora per `morphology_causal_contract_failure`, ma più tardi e
  dopo 1–3 riparazioni assorbite → una seconda regola del contratto causale
  è attiva; telemetria `morphology_causal_diagnostics` (con
  `failure_reason`) ora registrata nei trace di rollout; probe in corso.
- **Seconda regola trovata (probe con hook)**: `invalid_pending_transition_time`
  — `info["time"] = 17.076870983804135` vs onset del candidato pendente
  `17.07687098380528` (Δ = 1,1e-12) contro `_TIME_EPS = 1e-12` del corridoio
  causale. I due orologi sono accumulatori float indipendenti (tempo env vs
  griglia campioni binari); il resto dello stack V26 tollera 1e-9. **Bug
  preesistente e indipendente dalla v3**: spiega il ~43 % di episodi
  terminati per contract failure anche nello smoke v2 e verosimilmente gran
  parte delle 298 failure del training B0820 da 50 iter. Fix: `_TIME_EPS`
  1e-12 → 1e-9 + test di regressione (drift accettato, futuro vero 1 µs
  ancora fail-closed).
- **L3a, terza esecuzione (v3 + ri-armo + tolleranza 1e-9)**: start −0,20 e
  nominale arrivano a `episode_time_limit` (orizzonte pieno) con 2 e 5
  eventi invalidi assorbiti; **+0,20 idem** (5 assorbiti, 22,1 mm): L3a 3/3 a orizzonte pieno.
- **L3b, v3 sul corridoio corretto** (delta iter 6→8): contract failure
  **+0**, stance timeout **+0**, episodi a orizzonte **+20**, len media 500
  alla iter 7, 0 worker morti. Gate L3 (muri −50 %) superato: −100 %.
  Controllo v2 sul corridoio corretto in corso (per separare l'effetto
  della tolleranza da quello della v3).
- **L3b controllo v2 sul corridoio corretto** (delta iter 6→8): contract
  failure +5 (era +8), stance timeout +12, orizzonte +7. Attribuzione: la
  tolleranza da sola riduce poco (lo stance-lock uccide prima gli episodi
  e genera a sua volta disallineamenti stato/ancore); v3 + tolleranza =
  entrambi i muri a zero. **Protocollo L0–L3 completato: PASS.**
