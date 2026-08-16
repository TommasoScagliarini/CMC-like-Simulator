# Integrazione `binary_active` V25/V20 verso la FSM di H0

Data: 2026-08-05

## Esito

È stato implementato l'approccio 1 per rendere utilizzabile il detector
binario V25 senza modificare o riaddestrare H0:

```text
V25: bit grezzi heel/toe
    -> BinaryPhaseFSM V20
    -> adapter transazionale
    -> ProstheticPhaseFSM esistente
    -> osservazioni e reward già attesi da H0
```

La nuova modalità runtime è `binary_active`. Il suo significato non è
"tradurre V25 in V20": V20 interpreta già i due bit e produce gli eventi
funzionali. `binary_active` seleziona invece V20 come unica autorità per gli
eventi HS/TO sinistri e collega tali eventi alla FSM già consumata da H0.

L'implementazione è completa e non presenta blocker noti dopo l'audit
indipendente. Non sono stati eseguiti H0, H0_sep, PPO, morphology reward
positivo o trial protetti.

## Problema

V25 e la FSM binaria V20 erano già disponibili, ma il runtime poteva usarli
soltanto in `binary_shadow`: V20 veniva elaborata per diagnostica mentre gli
eventi legacy restavano autorevoli.

Il caso attivo richiedeva di risolvere contemporaneamente questi vincoli:

- H0 deve continuare a ricevere esattamente la struttura di stato della
  `ProstheticPhaseFSM` esistente;
- gli eventi sinistri legacy non devono poter rientrare come fallback;
- gli eventi destri devono restare sul percorso analogico legacy;
- carico continuo e `in_contact` devono provenire soltanto dalla GRF primaria;
- un reset con il piede già in appoggio non deve inventare un HS;
- V20 e la FSM actor-facing non devono potersi disallineare temporalmente;
- un batch o evento malformato non deve produrre un commit parziale;
- il percorso `disabled`/`binary_shadow` deve restare invariato.

Riutilizzare direttamente la vecchia modalità force-based `two_sensor` non
sarebbe stato corretto: quella modalità applica una seconda logica di soglia e
debounce, mentre V25/V20 ha già una semantica binaria funzionale congelata.

## Soluzione

È stato introdotto `BinaryPhaseActiveAdapter`, un bridge sottile dal punto di
vista funzionale ma deliberatamente rigoroso nei controlli. L'adapter:

1. riceve il batch V25 di dieci campioni Booleani a 1 ms;
2. avanza una copia privata della FSM V20;
3. verifica gli eventi accettati da V20;
4. li inoltra a una copia privata della `ProstheticPhaseFSM`;
5. verifica il trasferimento uno-a-uno di tipo, ordine e timestamp;
6. restituisce una coppia completa di FSM candidate;
7. lascia al runtime il commit soltanto dopo la validazione di eventi destri,
   gait state e diagnostica strict JSON.

Il percorso attivo viene eseguito prima della costruzione dell'osservazione e
del reward dello stesso policy step. Non viene quindi introdotto un policy step
aggiuntivo di ritardo.

### Routing definitivo

- Sinistra HS/TO: esclusivamente V25 -> V20 -> adapter.
- Destra HS/TO: esclusivamente stream analogico legacy.
- GRF continua sinistra e `in_contact`: esclusivamente GRF primaria fisica.
- Segnali V25: detector force-free, mai applicati alla dinamica.
- Prescribed GRF: nessun fallback per gli eventi sinistri attivi.

### Reset e partial stance

- Baseline V25 `AIR`: la FSM actor-facing parte da `WAIT_HS`.
- Baseline `HEEL`, `TOE` o `BOTH`: parte in stance parziale non accreditato.
- Non viene creato alcun HS sintetico.
- Il primo TO reale può chiudere la stance parziale senza bonus o ciclo
  artificiale.
- Dopo almeno 200 ms di swing, il primo HS reale apre il primo ciclo completo,
  mantenendo `valid_cycle_count=0` e `last_period_s=0`.

### Contratto temporale e fail-closed

Il percorso active accetta soltanto:

- profilo V25 congelato;
- contratto `binary_point_v25+functional_contact_fsm_v1`;
- `detector_sample_dt_s=0.001`;
- debounce V20 esatto di 5 ms;
- policy step esatto di 10 ms con dieci campioni;
- timestamp espliciti `event_time_s`, `confirmed_time_s` e
  `delivered_time_s`;
- conferma esattamente 5 ms dopo l'onset;
- conferma nell'intervallo policy aperto a sinistra e chiuso a destra;
- consegna al boundary corrente con ritardo massimo di 10 ms;
- stati runtime actor-facing `WAIT_HS`, `STANCE_AFTER_HS`, `SWING_AFTER_TO`
  o `TIMEOUT`.

Campioni mancanti, duplicati, non monotoni, non Booleani, non finiti, cursori
disallineati, stati proibiti/sconosciuti, eventi legacy sinistri, payload con
NaN/Inf e trasferimenti incompleti chiudono il percorso in errore senza
avanzare le FSM originali.

## Compatibilità con H0

L'adapter non serve a evitare un adattamento già necessario e non modifica la
rete. Serve a preservare l'interfaccia che H0 conosce:

- `ProstheticPhaseFSM.observation()` conserva le stesse otto chiavi;
- dopo un HS equivalente, modalità legacy e `binary_active` producono gli
  stessi otto valori;
- nessun campo V25/V20 viene aggiunto all'osservazione actor;
- le diagnostiche binarie restano soltanto in `info`;
- la forma e l'ordine dell'osservazione ambiente non sono stati modificati dal
  nuovo routing.

V25 non è stato usato per warm-start o adattamento di H0. Nessun checkpoint è
stato caricato, modificato o salvato in questo lavoro.

## Strategia di implementazione

- Aggiunta della sorgente `binary_active` alla FSM actor-facing, con API
  dedicate di reset e update.
- Blocco dell'API generica `update()` in modalità active, impedendo
  l'iniezione accidentale di eventi legacy.
- Cursor temporale privato della FSM actor-facing, distinto dalla semantica
  storica dell'accumulo di evidenza continua.
- Elaborazione di V20 e FSM actor-facing su deep copy e commit congiunto.
- Separazione esplicita degli eventi destri prima del commit.
- Controlli strict JSON e finitezza su configurazione, payload, eventi e gait
  state candidati.
- Mantenimento esatto dell'ordine storico del percorso `disabled` e
  `binary_shadow`.

## File modificati o creati

Runtime:

- `Trajectory Generator/binary_phase_adapter.py`, nuovo adapter
  transazionale;
- `Trajectory Generator/prosthetic_phase_fsm.py`, API active, partial stance,
  cursor e blocco del percorso generico;
- `Trajectory Generator/osim_trj_cmc_like.py`, validazione configurazione,
  inizializzazione, routing same-step, commit atomico e diagnostica
  `binary_active`.

Test:

- `validation/test_binary_phase_active_adapter.py`, test pure-Python di reset,
  cicli, timestamp, cadenza, rollback, stati e finitezza;
- `validation/test_binary_phase_active_env.py`, test environment-level di
  configurazione, routing sinistro/destro, assenza di fallback, same-step e
  atomicità.

## Invarianti preservate

Non sono stati modificati in questo intervento:

- `Trajectory Generator/binary_phase_fsm.py` V20;
- profilo geometrico V25;
- `online_grf.py` e la GRF primaria;
- profilo, materiale, geometria o legge di contatto della GRF primaria;
- plugin C++ e semantica SEA;
- checkpoint o pesi H0;
- configurazione attiva, che resta `legacy_events` con morphology reward a
  peso zero;
- lock e receipt storici;
- trial protetti 05/06 e reserve 03/07.

Hash congelati verificati dopo l'implementazione:

- FSM V20:
  `0f7669b60a72c1b27ee3c4f1a43161eeb9f2d091dff5558cc4fa43f1fce8d9c1`;
- profilo V25:
  `db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2`;
- profilo analogico legacy V25:
  `61ea948a3c0613e5c0e684a3197de118c7116e36188fca6993da79ce713fd99e`.

Hash dell'implementazione al momento del report:

- adapter:
  `34c9044b22edc260265164fe58f9adbe42624187c8fc3c8231af91eaaf2dc48b`;
- `prosthetic_phase_fsm.py`:
  `cd6578920f01a707695597f487541c7fce4bbf920394bf0328bf24beaf09b282`;
- `osim_trj_cmc_like.py`:
  `7c820c9e491c38fb59b272306c9e7b376c21cc0df4f181d371afd99a18c5c4a1`;
- test adapter:
  `9a3e668a42dabac93240a072ada811b0d02e193ba3b4ec2d4939b26c4e871404`;
- test ambiente:
  `c998e4b150c7be3ad4c2a6fb619fcd0a6963ccd9588cb14aac99873cd190ee4e`.

## Test e verifiche eseguite

- Suite unittest combinata detector/adapter/env/GRF: 75/75 PASS.
- Regressioni storiche `ProstheticPhaseFSM` two-sensor: 17/17 PASS.
- Regressioni trasporto detector high-rate: 5/5 PASS.
- Regressioni reward e morphology a peso zero: 32/32 PASS.
- Totale: 129/129 PASS.
- Test specifici `binary_active`: 20/20 PASS.
- `py_compile`: PASS.
- `ruff check`: PASS.
- `git diff --check`: PASS.
- Audit indipendente conclusivo: nessun blocker ad alta o media severità.
- Verifica hash V20, V25 e profilo analogico: PASS.

Non sono stati eseguiti rollout OpenSim con H0, PPO, training o accessi a dati
protetti.

## Stato del gate

La precedente voce "modalità active C e adapter non implementati" del report
`2026-08-05_protocollo_h0_v25_abc_congelato.md` è ora superata.

Questo lavoro dimostra la readiness del codice dell'approccio 1, ma non
autorizza ancora l'esecuzione H0. Non è stato creato un execution-unlock receipt
e non è ancora possibile dichiarare la compatibilità numerica di H0 con V25.

## TODO

- Eseguire un preflight full-environment senza update per verificare
  esplicitamente i layout actor/full 35/84, dtype `float32` e assenza di campi
  V25/V20 nell'osservazione.
- Eseguire la prova A/B bit-exact prevista dal protocollo congelato, prima con
  V20 disabilitata e poi in shadow.
- Eseguire il caso C `binary_active` soltanto dopo il PASS A/B e con le autorità
  previste dal protocollo.
- Produrre un nuovo receipt no-clobber con hash runtime aggiornati, destinazioni
  output e comparator esatti prima di qualsiasi rollout H0.
- Mantenere H0_sep, trial protetti, corridor positivo e PPO chiusi fino ai gate
  rispettivi.
