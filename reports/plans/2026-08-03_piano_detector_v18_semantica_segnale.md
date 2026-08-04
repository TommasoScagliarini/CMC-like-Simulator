# Piano V18 — diagnosi e selezione della semantica del detector a due sensori

Data di congelamento: `2026-08-03`

## Obiettivo e autorizzazione

V17 ha chiuso `TWO_SENSOR_HIGH_RATE_DEVELOPMENT_READY` in FAIL: il caso
canonico peggiore ha un HS confermato 122 ms dopo l'onset oracle, contro il
limite di 50 ms, e precision/recall non sono unitarie. Il receipt V17 resta
immutabile e autorevole. L'autorizzazione esplicita dell'utente del 2026-08-03
apre un nuovo ciclo V18, esclusivamente development, per capire se il problema
si risolve cambiando la semantica causale dei due segnali già esistenti.

V18 non riapre la GRF primaria, non modifica geometria, materiale, soglie,
dwell, SEA, policy, H0 o reward. Non accede ai trial 03/05/06/07 e non avvia
PPO.

## Provenienza e correzione metodologica

- V17 FAIL receipt SHA-256:
  `f2412d28721083fe2a5e654e937ad2dba4da00e0e1185118f56c4dd5cd6077fc`.
- Profilo detector V17 SHA-256:
  `2225823282743b55f2cbd3bdcb8c345f3d2e3b878bd0646283a4cc0b739df0bc`.
- Oracle development V17 manifest SHA-256:
  `c743fe0e177e3f5e2c5d12cb4e4def79f18fcfa9dd0fb2b3c018f92364c58ccd`.
- V15 è solo storia diagnostica. Il suo errore era il confronto tra eventi
  accettati dal full FSM e un router signal-only incapace di riprodurre
  rejection e timeout dipendenti dallo stato. V18 non usa quel router e non
  interpreta l'errore come difetto della GRF o delle tracce raw.
- I riferimenti evento sono letti esclusivamente dalle ledger canoniche V17.
  È vietato ricostruire l'oracle dalla GRF o riusare gli oracle per-cadenza
  V14/V15.

## Split development

- Trial 02: diagnosi e challenge nota; non determina il ranking.
- Trial 04: selezione fra le tre semantiche preregistrate.
- Trial 08: holdout interno V18 one-shot del solo finalist congelato.
- Trial 01: escluso.
- Trial 03/07: reserve chiusi.
- Trial 05: validation protetto chiuso.
- Trial 06: sealed protetto chiuso.

Il trial 08 è un holdout procedurale interno V18, non un dato scientificamente
vergine: 02/04/08 sono già stati usati nelle iterazioni precedenti.

## Gate 1 — `V18_RAW_TRACE_LOCKED`

Campionare una sola volta il profilo V17 sul setup prescritto e preprocessing
V14.2 hash-pinned, al solo reticolo 1 ms, senza riconversione e senza IK rerun.
Per ogni trial persistere:

- `time_s`;
- `left_heel_normal_n`;
- `left_toe_normal_n`;
- penetrazione ground-normal di tallone e punta, solo diagnostica.

Vincoli fail-closed: campioni finiti e non negativi, timestamp assoluti,
monotoni e unici, passo esatto 1 ms, nessun endpoint oltre intervallo, ruoli
esattamente heel/toe, conteggi 143206/143541/144201 per 02/04/08. File e
manifest sono no-clobber, strict JSON dove applicabile e legati da SHA-256.

L'apertura avviene in ordine: prima 02 e 04; 08 non viene campionato finché non
esiste un finalist lock che ha superato sia la selezione 04 sia il challenge
02.

## Gate 2 — `V18_SIGNAL_SEMANTIC_SELECTED`

Con geometria V17 e parametri invariati (`on=0.5 N`, `off=0.25 N`,
`dwell=0.03 s`) confrontare esattamente:

1. `heel_only`: HS sul rising debounced del tallone; TO quando tallone e punta
   sono entrambi stable-off.
2. `first_stable_regional`: HS sul primo rising debounced fra tallone e punta;
   tie-break deterministico su onset e poi tallone; stesso TO di `heel_only`.
3. `combined_load`: un latch causale su `heel + toe`; rising=HS e falling=TO.

Non sono ammessi sweep, soglie alternative, dwell alternativi, geometrie o
regole trial-specifiche. Il campione `t0` inizializza soltanto lo stato e non
genera eventi. Un nuovo HS viene armato solo dopo clear debounced. Ogni evento
espone onset, conferma e consegna alla prima boundary policy da 10 ms non
precedente alla conferma.

Ogni semantica viene processata sia sequenzialmente a 1 ms sia raggruppando gli
stessi campioni in batch da 10 ms. La parità deve essere esatta per latch,
onset, conferme, consegne, ordine, cicli, transizioni accettate e stati FSM.

Sul trial 04 un candidato è eleggibile solo se supera tutti i gate V17 su tutti
i quattro plateau e sull'aggregato: conteggi/ordine/cicli esatti,
precision=recall=1, F1 almeno 0.95, IoU almeno 0.90, HS confermato entro 50 ms,
TO entro 80 ms, latenza policy entro 60/90 ms, consegna entro 10 ms dalla
conferma, toe-clear almeno 30 ms, zero invalidità/timeout/stati proibiti o
metriche non finite.

Se più candidati passano, il ranking lessicografico congelato minimizza:

1. peggior rapporto di latenza rispetto al limite;
2. p95 dell'errore assoluto HS;
3. p95 dell'errore assoluto TO;
4. `-min(F1)`;
5. `-min(IoU)`;
6. ordine semantico `heel_only`, `first_stable_regional`, `combined_load`.

Non esiste un “migliore dei FAIL”. Il finalist selezionato deve poi superare,
senza riselezione, tutti gli stessi gate sul trial 02. Altrimenti V18 termina.

## Gate 3 — `V18_INTERNAL_HOLDOUT_READY`

Prima di aprire 08 congelare atomicamente semantica finalist, codice, test,
profilo, parametri, trace 02/04, oracle e receipt. Campionare 08 one-shot e
valutare soltanto il finalist nelle due modalità di consumo. Un FAIL termina
V18 senza rescue, retuning o nuova semantica.

Un PASS di 02/04/08 consente soltanto di proporre l'integrazione del finalist
nel detector e ripetere il gate completo V18 dopo save/reload a zero update.
Non apre automaticamente H0 né i trial protetti: questi richiedono un receipt
separato e la sequenza del piano principale.

## Stop rules e invarianti

- V17 non viene sovrascritto né reinterpretato.
- Il default resta `legacy_events` e `morphology_weight=0`.
- Nessun file della GRF primaria, contatto fisico o plugin C++ viene modificato.
- Nessun fallback trasforma un FAIL in tuning aggiuntivo.
- Sample mancante, duplicato, non monotono, non finito, hash drift, oracle non
  canonico o accesso a trial non autorizzato chiudono il gate fail-closed.
- Claim numerico macOS-only; codice e formati restano portabili su Windows.
