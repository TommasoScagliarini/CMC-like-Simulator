# Detector V21 — sweep geometrico binario e finalista development

## Stato

`V21_GEOMETRY_SWEEP_ELIGIBLE_FINALIST_DEV02_04_TRIAL08_CLOSED`

Lo sweep geometrico V21 è terminato con esito positivo sui trial development
02 e 04. Il candidato selezionato è:

`v21_678b0b5162b706dd`

Il candidato supera tutti i gate development, ma non è ancora il detector
promosso né una validazione protetta definitiva. Il profilo attivo, il training
e il morphology reward non sono stati modificati o avviati.

## Problema

V19 aveva eliminato la gradualità delle due sfere introducendo due segnali
puramente booleani punto-piano. V20 aveva poi dimostrato che la FSM era
causalmente corretta: debounce esatto di 5 ms, timestamp conservato all'onset e
parità fra consumo scalare a 1 ms e batch da 10 ms.

La combinazione V19 + V20 falliva però il confronto biomeccanico: gli eventi
erano anticipati e, soprattutto alle velocità più alte, comparivano conteggi e
cicli aggiuntivi. Poiché la FSM introduceva soltanto i 5 ms congelati, il
problema residuo era il significato fisico della geometria grezza rispetto
all'oracle GRF a 20 N.

## Soluzione e invarianti

È stato implementato uno sweep V21 che modifica esclusivamente la geometria
mesh-anchored dei due punti:

- posizione longitudinale del tallone;
- reach plantare del tallone rispetto alla mesh;
- posizione longitudinale della punta;
- reach plantare della punta rispetto alla mesh.

Sono rimasti congelati:

- piano e regola binaria `signed_clearance <= 0`;
- campionamento detector a 1 ms;
- FSM V20 e debounce a 5 ms;
- consegna alla policy a 10 ms;
- oracle canonici 02/04;
- GRF primaria online e relativo contatto fisico;
- semantica SEA;
- configurazione attiva e training.

Sono stati aperti soltanto DEV02/04. Trial 08, trial protetti e trial reserve
sono rimasti chiusi. La GRF prescritta non è stata risogliata: sono state
riutilizzate le ledger canoniche congelate.

## Strategia dello sweep

Lo script `validation/sweep_binary_phase_detector_v21_geometry.py` esegue:

1. preflight fail-closed di sorgenti, input e hash;
2. acquisizione una sola volta della trasformazione affine del piede a 1 ms;
3. ricostruzione di controllo delle tracce V19 tramite hash completi;
4. griglia coarse a quattro parametri;
5. selezione di quattro seed geometricamente distinti;
6. raffinamento fine attorno ai seed;
7. screening veloce equivalente alla semantica aggregata della FSM V20;
8. verifica dei tre migliori candidati con la FSM reale, sia scalare sia batch.

La progressione a terminale mostra barra ASCII, percentuale, conteggio,
`time elapsed` ed ETA. Gli artifact sono strict JSON/JSONL, atomici e
no-clobber.

Conteggi effettivi:

| Stadio | Candidati | PASS screening completo |
|---|---:|---:|
| Coarse | 900 | 0 |
| Fine | 11.898 | 170 |
| Totale | 12.798 | 170 |
| Verifica FSM reale top-k | 3 | 3 |

Il fatto che nessun candidato coarse passi, mentre 170 geometrie fine passano,
mostra che la risoluzione geometrica era determinante. Le geometrie valide
provengono da tutti e quattro i bacini di refinement; non si tratta quindi di
un singolo punto isolato.

## Geometria selezionata

| Punto | x [m] | Reach mesh [m] | Coordinate locali [m] |
|---|---:|---:|---|
| Tallone | -0,059315516055 | 0,025 | `[-0.059315516055, -0.06316904531668477, 0.01716156589300303]` |
| Punta | 0,135837908089 | 0,027 | `[0.135837908089, -0.06545974250405881, 0.005834590883117434]` |

Rispetto a V19:

- tallone avanzato di 35,345 mm, con reach invariato a 25 mm;
- punta avanzata di 20,089 mm;
- reach della punta aumentato da 25 a 27 mm.

La correzione dominante è quindi la posizione longitudinale sul rocker del
piede, accompagnata da una correzione plantare di 2 mm per la punta. Non è una
ricostruzione della vecchia geometria basata sul fondo delle sfere V17.

Profilo finalista:

`validation/binary_phase_detector_v21_runs/2026-08-04_run01/eligible_finalist_profile.json`

SHA-256:

`be8e063304a4798e5fc9947beb69c7b2ad813b4cab65e3bfdb0f2cd7284439bc`

## Risultati scientifici finali

Sono state valutate 16 unità finali:

- 2 trial development;
- 4 plateau per trial;
- 2 modalità di consumo degli stessi campioni.

Risultati complessivi:

- unità PASS: 16/16;
- eventi di riferimento per modalità: 244 HS e 236 TO;
- conteggi, ordine globale e cicli: esatti;
- precision e recall: 1,0;
- errore confirmed massimo HS: 24 ms, gate 50 ms;
- errore confirmed massimo TO: 55 ms, gate 80 ms;
- F1 minimo: 0,97194, gate 0,95;
- IoU minimo: 0,94542, gate 0,90;
- flight accettati: PASS in ogni unità;
- gate di consegna alla policy: PASS;
- parità scalare/batch: esatta per eventi, transizioni, cancellazioni,
  digest di stato e payload finale;
- firma eventi dello screening veloce uguale alla FSM reale;
- tallone e punta entrambi stabili, non degeneri e distinti in ogni plateau.

Il segno degli errori mediani seguenti è `confirmed - oracle`: negativo indica
anticipo, positivo ritardo.

| Trial | Plateau | Mediana HS [ms] | Max abs HS [ms] | Mediana TO [ms] | Max abs TO [ms] | F1 | IoU |
|---|---|---:|---:|---:|---:|---:|---:|
| 02 | 01 | -12,5 | 24 | -5,0 | 55 | 0,9854 | 0,9713 |
| 02 | 02 | +6,0 | 11 | -0,5 | 12 | 0,9913 | 0,9827 |
| 02 | 03 | +14,5 | 18 | -14,0 | 22 | 0,9722 | 0,9459 |
| 02 | 04 | -1,0 | 5 | -6,0 | 21 | 0,9929 | 0,9859 |
| 04 | 01 | -13,5 | 22 | 0,0 | 40 | 0,9869 | 0,9741 |
| 04 | 02 | +9,0 | 16 | -6,0 | 14 | 0,9873 | 0,9750 |
| 04 | 03 | +16,0 | 21 | -12,0 | 21 | 0,9719 | 0,9454 |
| 04 | 04 | +2,0 | 13 | -9,5 | 24 | 0,9909 | 0,9820 |

## Confronto con V19 e V17

| Geometria | Unità PASS fast | Failure strutturali | Eventi non abbinati | F1 minimo | IoU minimo |
|---|---:|---:|---:|---:|---:|
| V19, reach comune 25 mm | 0/8 | 8 | 513 | 0,8469 | 0,7344 |
| Equivalente fondo sfere V17 | 0/8 | 8 | 329 | 0,9187 | 0,8496 |
| V21 selezionata | 8/8 | 0 | 0 | 0,9719 | 0,9454 |

La FSM è rimasta bit-identica durante questo confronto. Il pass V21 costituisce
quindi evidenza diretta che il disallineamento development era principalmente
geometrico, non un errore della logica HS/TO e non un errore della GRF primaria.

## Ranking e finalisti vicini

Tutti e tre i candidati sottoposti alla FSM reale hanno superato 16/16 unità.
Il secondo candidato presenta F1/IoU minimi leggermente superiori, ma il
candidato selezionato rispetta il ranking congelato:

- stesso errore temporale normalizzato peggiore, pari a 0,6875;
- nessun deficit F1/IoU rispetto ai gate;
- tie-break finale verso la geometria più vicina a V19.

Non è stata effettuata una riselezione post-hoc sulla base delle metriche
osservate dopo lo sweep.

## Integrità e provenienza

Il run è durato 805,874 s, circa 13 min 26 s. Sono presenti sei artifact
terminali, senza `failure.json` o file temporanei residui. Tutte le 12.798 righe
JSONL e i JSON sono stati riletti con parsing strict; non sono presenti NaN o
Inf. Conteggi, dimensioni e hash dichiarati coincidono con i file.

Hash principali:

- script sweep: `32aeb6dadba42000607e7d1e7a2480d16574b032fb0d59ad2cd5fd6558147a47`;
- manifest: `ecc89b780a22a0762a121572234cfde2a1062762e03981935f8484ac0b21055e`;
- run start: `05647ecea5fe30ddaf9fbb77239b3e7c2cbd4b99b407d4649a289331bda28dcf`;
- profilo finalista: `be8e063304a4798e5fc9947beb69c7b2ad813b4cab65e3bfdb0f2cd7284439bc`.

Il sorgente sweep presente nel repository coincide con l'hash registrato dal
run. Trial 08, trial protetti e reserve non sono stati aperti.

## Limiti dell'attestazione

- Il claim numerico resta development-only e macOS arm64.
- I record compatti contengono 16 unità ma non conservano il nome della modalità
  di consumo; la sezione parity attesta comunque esplicitamente l'identità
  scalare/batch.
- I massimi esatti del ritardo di consegna e dell'errore delivered non sono
  riportati nei record compatti; il PASS dello scorer V20 attesta i relativi
  gate congelati.
- La barra di avanzamento non è persistita in un log; completezza e durata sono
  verificabili da artifact, conteggi e timestamp filesystem.
- Il profilo V21 non è ancora stato promosso nel registry e non è stato usato
  per H0, corridor o training.

Questi sono limiti di tracciabilità o di scope, non failure del risultato
development.

## File aggiunti o modificati

- `validation/sweep_binary_phase_detector_v21_geometry.py`;
- `validation/test_sweep_binary_phase_detector_v21_geometry.py`;
- `validation/experimental_detector_profiles/README.md`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/run_start.json`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/coarse_results.jsonl`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/fine_results.jsonl`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/final_verification.json`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/eligible_finalist_profile.json`;
- `validation/binary_phase_detector_v21_runs/2026-08-04_run01/manifest.json`;
- `reports/user/2026-08-04_sweep_geometria_detector_binario_v21.md`.

I moduli della GRF primaria, `online_grf.py`, il plugin C++, la geometria di
contatto primaria e la semantica SEA non sono stati modificati.

## Test e verifiche eseguite

- 17/17 test specifici V21 PASS;
- 45/45 regressioni V19/V20/V21 PASS nell'ambiente OpenSim;
- 500 tracce sintetiche casuali nella suite permanente: screening veloce e FSM
  V20 con firma eventi esattamente uguale;
- ricostruzione V19 sulle tracce complete 02/04 tramite hash: PASS;
- ricostruzione mesh della geometria V19: errore massimo inferiore a 5e-13 m;
- griglia coarse: 900 candidati e 900 ID unici, comparator V19/V17 presenti una
  sola volta;
- controllo strict JSON, atomicità e no-clobber: PASS;
- controllo hardcoded allowlist DEV02/04: PASS;
- `py_compile`: PASS;
- `ruff check`: PASS;
- `git diff --check`: PASS;
- audit post-run di tutti gli artifact e hash: PASS.

## TODO e prossimo gate

- [ ] Risolvere formalmente l'allocazione del prossimo trial: il contratto V21
  indica trial 08 come holdout one-shot, mentre il piano storico indicava 05/06
  come trial protetti. Nessun nuovo trial deve essere aperto prima di questa
  decisione.
- [ ] Congelare in un receipt no-clobber il candidato
  `v21_678b0b5162b706dd`, il profilo SHA `be8e0633...`, lo script SHA
  `32aeb6da...`, FSM V20, detector, oracle e input.
- [ ] Preparare e hashare il validatore one-shot senza leggere il trial scelto.
- [ ] Eseguire il trial protetto una sola volta, senza modificare geometria,
  FSM, ranking o gate.
- [ ] In caso di PASS, promuovere atomicamente geometria V21 + FSM V20 e
  proseguire con H0/corridor a morphology reward zero.
- [ ] In caso di FAIL, chiudere lo stage senza utilizzare il trial protetto per
  retuning o rescue.

Fino alla promozione restano vietati PPO e morphology reward positivo.
