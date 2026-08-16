# Daily Report - 2026-08-03

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Detector V18 - confronto semantiche del segnale e stop fail-closed](../user/2026-08-03_detector_v18_semantica_segnale_fail_closed.md)

## Sintesi

Il ciclo V18 development-only ha confrontato tre semantiche preregistrate del
segnale detector e si e' chiuso correttamente in fail-closed:

```text
TERMINAL_NO_V18_SIGNAL_SEMANTIC_PASSES_SELECTION_PREREQUISITES
```

Nessuna semantica ha superato i prerequisiti di selezione sul trial 04. Il
trial 08 non e' stato nuovamente campionato o valutato e i trial protetti
03/05/06/07 sono rimasti chiusi. Non sono stati eseguiti H0, PPO, training o
promozione runtime.

Il dato principale e' che l'errore HS non e' un semplice ritardo costante: nel
subset storico, 25 dei 26 HS fuori `+/-50 ms` erano anticipati e uno era
tardivo. Le semantiche monotone provate non ritardano causalmente l'ON e non
possono risolvere gli anticipi dominanti. La GRF primaria, il contatto C++, il
plugin SEA, la geometria V17, le soglie `0.5/0.25 N` e il dwell `30 ms` sono
rimasti invariati.

## 1. Problema affrontato

V17 aveva mostrato un evento HS tardivo di `+122 ms`, ma considerarlo da solo
avrebbe suggerito erroneamente una correzione costante del dwell. La
distribuzione completa mostra invece una netta prevalenza di anticipi; aumentare
il dwell per correggere il singolo ritardo avrebbe peggiorato molti altri
eventi.

E' stato inoltre isolato l'errore metodologico storico V15: un router
signal-only era stato confrontato come se fosse equivalente al full FSM, ma
non riproduceva rejection e timeout dipendenti dallo stato. L'errore non
riguardava GRF primaria o raw trace.

## 2. Protocollo e strategia V18

Il protocollo separato ha congelato:

- trial 02 come diagnosi/challenge;
- trial 04 come selezione;
- trial 08 come holdout interno soltanto condizionale a un PASS;
- tre semantiche senza sweep: `heel_only`, `first_stable_regional` e
  `combined_load`;
- oracle derivato esclusivamente dalle ledger canoniche V17;
- replay sequenziale a `1 ms` e batch sugli stessi gruppi da `10 ms`;
- stop prima del full FSM se tutti i candidati falliscono gia' un gate
  necessario.

Sono state acquisite e congelate le raw trace dei soli trial development:

| Trial | Campioni | Integrita' |
| --- | ---: | --- |
| 02 | 143.206 | timestamp assoluti, unici e monotoni; valori finiti |
| 04 | 143.541 | timestamp assoluti, unici e monotoni; valori finiti |

Manifest e raw trace sono legati da SHA-256. Non e' stata acquisita una nuova
trace 08.

## 3. Risultati

Sul trial 04:

- `heel_only`: P1 produce 23 HS anziche' 24; P2/P3/P4 hanno errore HS massimo
  rispettivamente `69/54/57 ms`, oltre il gate `50 ms`;
- `first_stable_regional`: riduce il caso 02/P3 da `+122` a `+60 ms`, ancora
  fuori gate, e introduce eventi/cicli extra; su 04/P2 produce 45 HS contro 33
  e su P3 41 contro 38;
- `combined_load`: ha lo stesso esito necessario del primo sensore stabile,
  non ritarda i 14 HS anticipati del trial 04 e introduce conteggi extra.

Gli output evento/edge sono identici tra processamento sequenziale e batch.
Nessun candidato e' eleggibile; il protocollo ha quindi arrestato il ciclo
prima del full FSM e prima dell'apertura condizionale del trial 08.

## 4. Limiti emersi dalla review

La review indipendente ha ritirato il claim toe-clear: V18 non scansiona una
possibile riattivazione intermedia del latch toe con la stessa semantica del
gate V17 autorevole. Questo non cambia lo stop, perche' ogni candidato fallisce
comunque conteggi o timing HS.

La parita' dimostrata riguarda journal evento/edge e lo stato incluso nel
digest; non e' una parita' del full FSM runtime. Anche la lettura, in preflight,
dell'identita' della ledger 08 preesistente non equivale a generare raw trace o
scoring V18 sul trial 08.

## 5. Decisione e scope chiuso

```text
selezione V18                         = FAIL TERMINALE
finalist V18                          = NESSUNO
nuova acquisizione/scoring trial 08   = NON ESEGUITI
trial protetti 03/05/06/07            = NON LETTI
H0 / PPO / training                   = NON ESEGUITI
promozione runtime                    = NON ESEGUITA
contratto statico GRF primaria/V17    = PASS
claim toe-clear V18                   = RITIRATO
parita' full FSM                      = NON DIMOSTRATA
```

La chiusura e' un esito valido del protocollo: impedisce di usare il failure
come nuovo ciclo di tuning e conserva separati detector, GRF primaria e
training.

## File principali aggiunti

- `reports/plans/2026-08-03_piano_detector_v18_semantica_segnale.md`
- `validation/two_sensor_v18_signal_semantics_protocol.json`
- `validation/acquire_two_sensor_v18_raw_traces.py`
- `validation/two_sensor_v18_selection_analysis_contract.json`
- `validation/evaluate_two_sensor_v18_signal_semantics.py`
- `validation/test_two_sensor_v18_raw_trace_acquisition.py`
- `validation/test_two_sensor_v18_signal_semantics.py`
- `validation/two_sensor_v18_signal_semantics_runs/`
- `.gitattributes`

## Test e verifiche

- `81` test mirati PASS, inclusi V18, V17, oracle canonico, FSM high-rate e
  data path detector;
- entrambi i preflight V18 PASS;
- audit indipendente di conteggi, hash, dimensioni e finitezza PASS;
- `3.641` record JSON/JSONL strict e senza NaN/Inf;
- validazione statica del contratto GRF primaria/V17 PASS;
- nessun file temporaneo residuo;
- `.gitattributes` forza LF sui nuovi artefatti per preservare gli hash su
  checkout Windows.

Il claim numerico resta macOS arm64.

## TODO chiusi o rispettati il 2026-08-03

- [x] Eseguire un nuovo ciclo separato e preregistrato, senza sweep o
  micro-tuning post-hoc, preservando geometria e parametri V17.
- [x] Arrestare il protocollo prima del full FSM e del trial 08 quando nessuna
  semantica supera i prerequisiti development.
- [x] Mantenere chiusi i trial protetti e non usare il failure V18 per H0, PPO,
  training o promozione.
- [x] Verificare integrita', finitezza, hash e parita' evento/edge tra consumo
  sequenziale e batch entro il claim esplicitamente limitato di V18.
- [x] Conservare GRF primaria, contatto C++, plugin SEA, geometria V17, soglie
  e dwell invariati durante il ciclo.

## TODO aperti e propagati

### Prossima revisione detector

- [ ] Preregistrare un nuovo ciclo con una semantica capace di ritardare
  causalmente l'ON; le alternative monotone V18 non sono eleggibili.
- [ ] Valutare il candidato con il full FSM, includendo rejection, timeout e
  stato runtime, senza riutilizzare un router signal-only come suo sostituto.
- [ ] Ripristinare nel validatore il gate toe-clear autorevole, scansionando
  eventuali riattivazioni intermedie del latch toe.
- [ ] Dimostrare conteggi, ordine e timing HS `<=50 ms`, timing TO `<=80 ms`,
  continuita', clear causale, zero invalidi e coerenza `1/10 ms` prima di
  qualsiasi holdout.
- [ ] Non acquisire o valutare il trial 08 finche' un unico candidato congelato
  non supera integralmente il development; mantenere 03/05/06/07 chiusi.
- [ ] Creare ogni revisione come ramo distinto senza sovrascrivere V9, V13,
  V17 o V18 e senza reinterpretarne i FAIL.
- [ ] Ottenere una ground truth localizzata o dichiarare esplicitamente la
  semantica `initial_contact`; la sola GRF totale non certifica un heel strike
  anatomico.

### Routing, H0 e ablation ancora aperti dal 2026-07-23

- [ ] Eseguire da processo nuovo il frozen-policy H0 a zero update con GRF
  primaria fissa e detector assente, shadow e autoritativo.
- [ ] Verificare in shadow l'invarianza di observation load/contact, azioni,
  reward fisico e terminazioni; quantificare separatamente gli eventi soltanto
  dopo la promozione di un detector.
- [ ] Rivalidare H0 prima di un nuovo warm start; mantenere `legacy_events` e
  morphology weight zero come default finche' i gate non sono completi.
- [ ] Documentare l'esito finale del pilot V13/corridor avviato il 23 luglio e
  mantenerlo diagnostico, senza promuoverne checkpoint.
- [ ] Eseguire le ablation condition-matched `H0 + V13/two_sensor` senza
  corridor e `H0 + detector precedente` con corridor, se ancora scientificamente
  utili; confrontare durata, terminazioni, HS/TO e componenti reward oltre al
  return.
- [ ] Non avviare H0/PPO/training su una nuova revisione finche' detector,
  routing e protocollo non superano i gate prescribed e protected previsti.

### Morphology Corridor

- [ ] Completare un rollout live shadow da 500 step complete-segment in una
  sessione OpenSim pulita e il relativo A/B.
- [ ] Definire bordo phase-dependent, sicurezza immediata o retrospettiva,
  `WAIT_HS`, disponibilita' morfologica e gate ankle lento.
- [ ] Implementare la riscrittura complete-segment/complete-episode prima del
  GAE, inclusi segmenti incompleti, timeout e bootstrap.
- [ ] Solo dopo i gate, eseguire un A/B corto a weight
  `0 / 0.0025 / 0.005`, conservando checkpoint, start, seed, batch e learning
  rate e monitorando loss, corridor, cicli, penetrazione, SEA, reserve e actor.
- [ ] Validare start/seed held-out e almeno un profilo o modello esterno ad
  AB06 prima di training lungo o promozione.

### Training, robustezza e audit

- [ ] Conservare H0, detector storici e rollback come artefatti immutabili;
  ripartire soltanto da una sorgente preregistrata.
- [ ] Valutare ogni checkpoint con multistart, seed held-out, worst-case
  recovery, cicli, penetrazione, reserve, SEA e clipping, non solo return.
- [ ] Non modificare reward, hard limit `25 mm` o feature actor per forzare un
  PASS; separare causalmente reward, GRF online, reserve e timing FSM.
- [ ] Mantenere exact-start, compaction, interleaving, singola epoca e gate
  reserve condition-matched; preregistrare la non-regressione reserve.
- [ ] Mantenere i seed `126-128` sigillati fino al PASS development completo e
  raccogliere recovery data event-aligned indipendenti, con attenzione agli
  step `210-230`.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
  multi-ciclo.
- [ ] Confrontare H0 e logical 24 con protocollo identico e overlay di azioni,
  served, cinematica, SEA, GRF, eventi e reward.
- [ ] Eseguire ablation progressive del plant prescribed e training A/B da H0
  variando un solo gruppo reward per volta.

### Deployment e hardware

- [ ] Congelare ordine/scaling feature, frequenza, unita', normalizzazione BW,
  filtri, FSM, reset, governor, limiti, fallback e watchdog.
- [ ] Validare detector con rumore/delay realistici, export actor-only,
  equivalenza host-target, latenza worst-case, HIL e poi human-in-the-loop.
- [ ] Portare la GRF online a production-ready mantenendola separata dai
  sensori detector; conservare le reserve come gate fisico finche' il residuo
  non e' attribuito.
- [ ] Eseguire prove piu' lunghe su trial, velocita' e soggetti differenti
  prima di dichiarare generalizzazione o deployment.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta.
