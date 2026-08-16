# Detector binario V24/V25 — diagnostica clearance e sweep della reach

Data: 2026-08-04

## Esito

Il gap OFF/OFF di 12 ms osservato in V23 è stato diagnosticato e corretto in
development senza modificare la FSM.

- V24: `PASS_V24_TRIAL08_GEOMETRIC_CLEARANCE_GAP_CONFIRMED`;
- V25: `PASS_V25_LOCAL_GEOMETRY_DEVELOPMENT`;
- candidato selezionato: `v25_4b351f67b5b86ab0`;
- tallone invariato: reach `25,0 mm`;
- punta: reach aumentata da `27,0 mm` a `27,5 mm`;
- posizione x, posizione laterale, mesh anchor, piano, regola binaria e FSM
  V20 invariati;
- stage 2 bidimensionale non aperto, perché lo stage 1 ha prodotto un unico
  candidato esattamente eleggibile.
- candidato development congelato globalmente con stato
  `V25_DEVELOPMENT_CANDIDATE_FROZEN_H0_PROTOCOL_REQUIRED` e lock SHA
  `04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346`.

Il risultato è ancora development, ma non è più riselezionabile o ritunabile.
Non costituisce promozione runtime, claim su H0 o apertura dei trial protetti.

## Problema

Con la geometria V21, nel plateau a 2,05 m/s del trial 08 il trasferimento
tallone-punta conteneva un intervallo grezzo:

- `99.882–99.893 s`: tallone OFF e punta OFF, 12 campioni a 1 ms;
- `99.894 s`: ricontatto della punta.

L'intervallo cadeva dentro lo stesso stance oracle, fra HS `99.662 s` e TO
`100.168 s`. La FSM V20 non generava il difetto: applicava correttamente il
debounce di 5 ms al segnale ricevuto e produceva quindi un TO e un HS spurii.

## Diagnostica V24

V24 ha ricampionato a 1 ms le clearance continue della geometria V21 nella
finestra `99.750–100.200 s`, usando le stesse coordinate IK su:

1. modello marker/source, SHA
   `98cfcbc4f7155ea4576f583654fbd50a6e8bd2f2f33ff0894c9f3f24dce5fa8d`;
2. modello runtime, SHA
   `33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1`.

Le due tracce coincidono esattamente:

- massima differenza clearance tallone: `0 m`;
- massima differenza clearance punta: `0 m`;
- massima differenza della sensibilità alla reach: `0`;
- bit marker, bit runtime e bit V23: identici.

È quindi esclusa una incompatibilità cinematica marker/runtime. Il problema è
la copertura della geometria V21 sul segnale cinematico prescritto.

Nel gap critico:

- clearance tallone: da `+0,120749 mm` a `+2,540058 mm`;
- clearance punta: da `+0,346367 mm` a `+0,009447 mm`;
- shift normale comune minimo per chiudere il gap: `0,316483 mm`;
- incremento minimo della sola reach della punta: `0,346773 mm`;
- incremento minimo della sola reach del tallone: `2,542265 mm`.

Questa evidenza ha motivato uno sweep locale sulla sola reach, senza aprire
un nuovo tuning della FSM o delle coordinate x.

## Strategia V25

Lo stage 1 ha valutato nove geometrie uniche:

- braccio punta: tallone fisso a 25,0 mm e punta a
  `27,00/27,25/27,50/27,75/28,00 mm`;
- braccio tallone: punta fissa a 27,0 mm e tallone a
  `25,00/25,25/25,50/25,75/26,00 mm`;
- la baseline comune è stata contata una sola volta.

Il gate aggiuntivo richiede zero run OFF/OFF interni delimitati da contatto
prima e dopo nello stesso stance oracle. Le normali fasi di swing sono escluse
da questo conteggio.

Ogni candidato screen-eligible deve poi superare la verifica esatta su:

- trial development 02/04/08;
- quattro plateau per trial;
- processamento sequenziale 1 ms e batch 10 ms sugli stessi campioni;
- 24 unità complessive;
- conteggi, ordine, cicli, precision/recall, F1, IoU, latenze e flight;
- parità esatta scalar/batch;
- prossimità mesh e struttura a due canali.

## Risultati dello sweep

| Tallone | Punta | Unit PASS screen | Gap interni | Elegibile |
|---:|---:|---:|---:|---:|
| 25,00 mm | 27,00 mm | 11/12 | 1 | no |
| 25,00 mm | 27,25 mm | 12/12 | 1 breve | no |
| 25,00 mm | 27,50 mm | 12/12 | 0 | sì |
| 25,00 mm | 27,75 mm | 10/12 | 0 | no |
| 25,00 mm | 28,00 mm | 10/12 | 0 | no |
| 25,25–26,00 mm | 27,00 mm | 11/12 | 1 | no |

Il candidato a 27,50 mm è quindi l'unico punto che chiude completamente il
gap senza introdurre regressioni temporali.

Verifica esatta del candidato selezionato:

- 24/24 unità PASS;
- precision e recall HS/TO: `1,0`;
- conteggi, ordine e cicli: esatti;
- parità sequenziale/batch: PASS su 02, 04 e 08;
- gap interni: zero su 127 + 132 + 139 = 398 cicli oracle;
- F1 minimo: `0,958970`;
- IoU minimo: `0,921174`;
- errore confermato massimo HS: `27 ms`;
- errore confermato massimo TO: `53 ms`;
- errore consegnato massimo HS: `31 ms`;
- errore consegnato massimo TO: `50 ms`;
- flight accettato minimo: `364 ms`;
- prossimità mesh e reach verticale: PASS.

## Artifact e binding

V24:

- script diagnostico SHA
  `5a037622848fbd47ee302b158fe7ec17f0fbdde4f5cd51f3681ea5653282a0c0`;
- diagnostic SHA
  `72112d308810685263d5f7af0c4371c6bcf0336e92192da9550feed6253a3546`;
- receipt SHA
  `3c4c0caa78745bafeaae812f332158f44152c4148a99d6aa4fb3fd0ec36c59c2`.

V25:

- script sweep SHA
  `0b66c1f74e8c9bb658af14099058fba292b70968cfe8039a8d35589c73216299`;
- manifest SHA
  `4feeee1a32db9d1b8efc8cef5cdb08c8b35c3726b91cabd7013b52ac11748094`;
- terminal decision SHA
  `f34b805051883b85f26a9c42b6b4601d9c68bf5bf58dda14da82cf69c42db9f0`;
- profilo selezionato SHA
  `db704e502b99e49bea6d89493812bafdac748f8ce8d3ce28214ff624078539a2`.

Freeze globale V25:

- script freeze SHA
  `9b9a956be5a80fb07717b86ca4a9c9aa657e346144f1e0eaa6147539c06746ed`;
- lock SHA
  `04ecfe68937bc0d4baa3be9ab9b62060b20eb92c2f218f8540db1cebe423d346`;
- run V25 marcato consumato e non ripetibile;
- candidato, geometria, profilo, FSM V20, sorgenti e artifact V24/V25 legati
  esplicitamente dagli hash;
- prossimo stage vincolato a
  `FREEZE_H0_A_B_C_PROTOCOL_BEFORE_ANY_H0_EXECUTION`.

Gli artifact sono strict JSON/JSONL, senza NaN/Inf, no-clobber e verificati
contro gli hash dichiarati nel manifest. Non esiste `failure.json`.

## File introdotti

- `validation/diagnose_binary_phase_detector_v24_clearance_gap.py`;
- `validation/test_binary_phase_detector_v24_clearance_gap.py`;
- `validation/binary_phase_detector_v24_diagnostic_runs/`;
- `validation/sweep_binary_phase_detector_v25_geometry.py`;
- `validation/test_sweep_binary_phase_detector_v25_geometry.py`;
- `validation/binary_phase_detector_v25_geometry_runs/`;
- `validation/freeze_binary_phase_detector_v25_development_candidate.py`;
- `validation/test_freeze_binary_phase_detector_v25_development_candidate.py`;
- `validation/binary_phase_detector_v25_development_candidate_freeze_lock.json`;
- questo report.

Nessun file V21–V23 è stato modificato dal run. Non sono stati modificati
`online_grf.py`, il plugin C++, la semantica SEA o la FSM V20.

## Test e verifiche

- V24: 15/15 test PASS, zero skip;
- V25: 14/14 test PASS, zero skip;
- `py_compile`: PASS;
- preflight V24 e V25: PASS;
- progress bar, ETA e time elapsed: verificati e mostrati durante il run;
- strict JSON/JSONL e finitezza ricorsiva: PASS;
- hash e dimensioni dei sette file legati dal manifest V25: PASS;
- freeze V25: 20/20 assertion preflight PASS;
- suite indipendente del freeze: 5/5 test PASS, zero skip;
- audit indipendente finale: PASS, nessun blocker;
- `git diff --check`: PASS.

## Scope rimasto chiuso

- trial protetti 05/06: non aperti;
- trial reserve 03/07: non aperti;
- trial storico 01: non riaperto;
- H0: non eseguito;
- profilo runtime: non modificato;
- training, corridor e PPO: non avviati;
- claim numerico: macOS arm64; Windows resta privo di DLL/parità attestata.

## TODO

- [x] Integrare il profilo V25 in modalità dormant/shadow, mantenendo come
  default `legacy_events` e morphology reward a peso zero. Completato il
  2026-08-04; l'integrazione non costituisce promozione runtime.
- [x] Congelare il protocollo H0 A/B/C prima di qualsiasi esecuzione H0.
  Completato il 2026-08-05 con il lock corretto SHA
  `04ae8e209ccae05075b625f89ac827b145d5149e4237fe2128b1c822d105fe8b`.
  Il primo lock SHA `4f24deb4...`, mai autorizzato né eseguito, è conservato
  come evidenza rigettata perché A/B avevano uno schema non perfettamente
  simmetrico.
- [ ] Implementare e testare i prerequisiti ancora chiusi: journal V25 di A,
  replay B, adapter V20 -> `ProstheticPhaseFSM` e modalità active C, bootstrap
  partial-stance, driver/comparatore bit-exact, tape stocastico, mapping delle
  tracce SEA e destinazioni no-clobber.
- [ ] Congelare un receipt separato di execution-unlock; solo dopo quel gate
  eseguire i casi H0 A/B/C, senza aprire 05/06.
- Solo dopo il gate H0, decidere l'apertura one-shot dei trial protetti 05/06.
- Mantenere corridor, morphology reward positivo e PPO chiusi fino ai gate
  successivi.
