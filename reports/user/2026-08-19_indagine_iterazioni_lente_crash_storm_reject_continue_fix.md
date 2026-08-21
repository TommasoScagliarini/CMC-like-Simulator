# Indagine iterazioni lente (36 vs 8–15 min): tempesta di crash silenziosi da `reject_continue` — bug trovato e corretto

Data: 2026-08-19

## Problema (requisito utente)

Le iterazioni della continuazione (`_cont200`) duravano mediana 36,5 min
contro gli 8–15 min di giugno e della baseline nativa. Requisito esplicito:
capire la causa prima di ripartire e tornare a 8–15 min/iterazione.

## Catena di evidenze (in ordine di falsificazione)

1. **Non è il costo per step**: `env_step_timer` 1,18 s (cont) vs 1,35 s
   (giugno) vs 1,32 s (nativa) — invariato; reset 5–11 ms; overhead
   learner/GAE/connettori in millisecondi; step campionati/iter invariati
   (~6.0k vs ~6.2k). Utilizzo aggregato: 98% a giugno, ~25% nella cont.
2. **Non sono gli stati "costosi"**: il rollout deterministico attraversa
   gli stati a 12–15 mm di penetrazione a 0,93 s/step; un probe per-step in
   config identica al training è rimasto piatto a ~1,0–1,2 s per 340 step.
3. **Watch dal vivo**: fase di campionamento con 13/13 runner al 100%
   (come giugno), poi coda con 1–2 processi inchiodati e il resto fermo.
4. **La pistola fumante nei log Ray** (`/tmp/ray/session_*`):
   - smoke (10 iter): **336 errori** `previous_time_s off-grid`,
     300 file worker;
   - cont200 (18 iter): **1.044 errori**, **639 file worker** (~30 morti
     e ricostruzioni di worker per iterazione, silenziose per
     `ignore_env_runner_failures`);
   - baseline nativa e prima esecuzione: 0 errori.

**Meccanica dei 36 minuti**: ogni worker morto viene ricostruito da Ray
(caricamento modello OpenSim, ~minuti su un core — i "processi busy"
osservati erano rebuild, non simulazione) mentre la barriera sincrona PPO
aspetta. ~30 morti/iterazione → 3× il tempo.

## Il bug (mio, nella modifica `reject_continue` del 17/08)

`adapter.advance()` è transazionale: su evento invalido la rejection scatta
in `_validate_transfer` DOPO che le FSM candidate hanno già consumato la
finestra di 10 ms, e i candidati vengono scartati. Il mio percorso di
assorbimento faceva `cursore_sensori := t` **senza committare nulla** → al
passo successivo la FSM binaria confronta il cursore col proprio orologio
interno (rimasto indietro di 10 ms) → `ValueError: previous_time_s
off-grid` → **morte garantita del worker un passo dopo ogni evento
assorbito**. In modalità `terminate` il bug era invisibile: l'episodio
finiva nello stesso step e il reset riallineava gli orologi (per questo il
run da 100 iter è stato pulito).

Il probe per-step l'ha riprodotto deterministicamente (crash allo step 342,
un passo dopo l'assorbimento avvenuto allo step 341 — il contatore stampato
ogni 20 step l'aveva mascherato).

## Revisione del verdetto smoke (correzione di un mio errore di lettura)

Il "PASS" dello smoke si basava sul contatore `phase_timeout_swing`
congelato e su `invalid_binary_event` fermo. In realtà: gli episodi con
evento invalido **crashavano il worker invece di terminare** (nessun
contatore incrementato) → campionamento distorto (le traiettorie con eventi
invalidi sparivano dal batch senza penalità) oltre al costo di tempo. I
return e le lunghezze degli episodi sopravvissuti restano reali, ma
l'efficacia della leva (A) va ri-misurata con contatori onesti dopo il fix.

## Fix implementato (3 file + test)

- `Trajectory Generator/binary_phase_adapter.py`: `advance()` accetta
  `invalid_event_mode="raise"|"drop"` (default `raise`, byte-identico).
  In `drop`: l'evento rifiutato non viene pubblicato (`left_events=[]`),
  i candidati window-consistent vengono restituiti e il payload flagga
  `invalid_event_dropped` + `invalid_event_type`. La phase FSM aveva già
  gestito l'evento internamente (transizione rifiutata, stato preservato):
  si committa il suo comportamento nativo.
- `Trajectory Generator/binary_phase_adapter_v26.py`: passthrough del
  kwarg + propagazione dei flag nel payload V26.
- `Trajectory Generator/osim_trj_cmc_like.py`: eliminato il percorso di
  assorbimento manuale; un unico commit sempre coerente. `terminate` e
  `reject_continue` usano `drop` (poi la policy decide se flaggare la
  terminazione MDP); `raise` invariato fail-closed.
- `validation/test_binary_invalid_event_policy.py`: iniezione spostata su
  `_validate_transfer` (esercita il percorso drop reale) + nuovo test di
  regressione "assorbe e sopravvive agli step successivi".
  **Suite: 6/6 PASS. Ruff: pulito** (rimossi anche 2 import morti
  preesistenti in adapter_v26).

## Secondo bug latente scoperto dalla regressione: microstep degenere all'orizzonte

Il probe post-fix ha superato il vecchio punto di crash (2 eventi assorbiti
a ~16,4–16,6 s, contatore onesto, ~0,9 s/step piatti) ed è arrivato dove
nessun episodio V26 era mai arrivato: la fine del segmento. Lì è emerso un
secondo bug, indipendente dal primo: `_episode_end = start + 5.0 =
17.990000000000002` mentre `self.t`, dopo 500 accumulazioni float, resta
~1e-12 s sotto — la tolleranza 1e-12 del test `reached_horizon` non scatta
e parte un 501° step degenere da ~1,5e-12 secondi, che il trasporto
sensori binario rifiuta correttamente come timestamp duplicato
(worker-fatal). A giugno lo stesso microstep esisteva (i `len 501` storici
sono 500 step reali + 1 microstep degenere) ma il legacy non aveva il
contratto di trasporto che lo rifiuta. Nessun training V26 l'aveva mai
visto perché nessun episodio V26 era mai sopravvissuto fino all'orizzonte.

Fix: test dell'orizzonte robusto a mezza finestra di policy
(`self.t >= _episode_end − segment_duration/2`), che elimina il microstep
senza cambiare la durata reale degli episodi (500 step pieni).

## Verifiche

- suite dedicata 6/6 PASS (terminate, reject_continue+sopravvivenza,
  raise fail-closed, typed error, SECTION_MAP);
- regressione reale: riesecuzione del probe per-step che crashava — in
  corso al momento della scrittura (esito nel prossimo report/daily);
- evidenze quantitative da tensorboard/log Ray citate sopra, tutte
  riproducibili dai run dir e dalle sessioni Ray.

## Verifica finale certificata (run `postfix_timing_verify3`, iter 129–131)

- **durata iterazione: 7,7 min** (misurata da tensorboard) — requisito
  8–15 min soddisfatto, tornati al livello di giugno (mediana 8,2);
- **log Ray: 0 errori off-grid, 81 file worker** (= solo il boot normale;
  prima: 1.044 errori e 639 file);
- regressione probe: 2 episodi da **500 step pieni** con chiusura
  `episode_time_limit` — primi episodi V26 a completare l'orizzonte;
- apprendimento sano e onesto: len_mean fino a **424,7** (massimo storico
  della lineage), best return 158,4, penetrazione ~8 terminazioni pulite
  per iterazione, nessun worker morto.

## Aspettativa post-fix e piano di verifica per la ripresa

Con i crash eliminati l'iterazione deve tornare a ~sample 6,5–7 min +
overhead = **~8–15 min** (il requisito). Alla ripresa (resume da
`cont200/checkpoint_last`, iter 129): smoke di verifica breve monitorando
(i) durata iterazione, (ii) **zero** errori off-grid nei log Ray della
nuova sessione, (iii) contatori onesti: `invalid_binary_event` che si
muove di nuovo (ora come drop assorbiti conteggiati dall'env) e la verità
su `phase_timeout_swing` con la 2.6.

## Continuazione 132→150 post-fix (run `_cont150`): ri-misura onesta della leva (A)

19/19 iterazioni a ~8,2 min, **0 errori off-grid, 81 file worker** (zero
churn). Contatori ora veritieri, delta sulle 19 iterazioni:

- `phase_timeout_swing` **+0** — il muro dello swing è genuinamente
  dissolto dalla 2.6 s (stavolta senza crash a mascherare);
- `phase_timeout_stance` +1, `joint_divergence` +0, `invalid_binary_event`
  +0 terminazioni (gli eventi invalidi sono drop assorbiti, by design);
- `grf_penetration` +109 (≈5,7/iter, ~40% degli episodi): unico margine
  attivo rimasto; il resto degli episodi arriva a orizzonte pieno.

Apprendimento: best return **189,8** (iter 137, nuovo massimo lineage),
media ultime 5 = 177,8; len media 403, ultime 5 = **431**, max 471 (giugno:
~270 return, ~478 len). Traiettoria in salita netta; il gap residuo è
concentrato nella penetrazione e nel tracking del ginocchio.

## TODO

- [x] Esito regressione probe per-step: PASS (2×500 step pieni).
- [x] Verifica 3 iter: 7,7 min/iter, 0 off-grid — requisito certificato.
- [x] Ri-misura onesta della leva (A): vedi sezione sopra.
- [ ] Prosecuzione 151→200 (~6,5–7 h) per il confronto formale coi criteri
  di giugno — richiede conferma utente.
- [ ] Fix probe `train_ppo_mlp.py:1408` (make_cmc_env) prima del prossimo
  training fresh.
- [ ] Daily 2026-08-18/19 da consolidare al prossimo `end_day`.
