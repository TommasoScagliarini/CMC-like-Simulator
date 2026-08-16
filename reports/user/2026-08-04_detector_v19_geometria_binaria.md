# Detector V19 — geometria binaria force-free

## Stato

`BINARY_GEOMETRY_READY_HS_TO_UNDEFINED`

È stata implementata e verificata una nuova sorgente tallone/punta puramente
binaria. Il lavoro si ferma qui: i bit non sono collegati alla FSM, non vengono
generati HS/TO e non è stato eseguito training.

## Problema

Il detector V17/V18 usava due sfere Hunt-Crossley sensor-only. Il segnale
disponibile alla logica eventi era quindi una forza continua dipendente da
penetrazione, materiale e dinamica di contatto. Questo rendeva difficile
distinguere un errore della geometria simulata da un errore della logica HS/TO.

La richiesta V19 era ridurre il sensore al minimo: due segnali indipendenti
`0/1`, senza gradualità di forza.

## Soluzione

Il nuovo modulo usa esattamente:

- due punti solidali a `/bodyset/foot_l`, uno al tallone e uno alla punta;
- un piano detector congelato;
- la distanza firmata punto-piano;
- `contact = signed_clearance <= 0`.

Non esistono sfere, raggi, materiali, legge di contatto, forza, soglia in N,
dwell o debounce. Il detector non aggiunge `Force` o altri componenti OpenSim.

Il payload pubblico è separato dal vecchio stream analogico:

```text
binary_phase_sensor_samples = [
  {time_s, left_heel_contact, left_toe_contact}
]
```

I due contatti sono `bool` Python. La clearance continua resta disponibile
soltanto in una diagnostica separata e non entra nel payload binario.

## Scelta geometrica

Gli x/z V17 sono stati conservati. Per ogni ruolo è stata trovata la proiezione
sulla faccia plantare di `Geometry/AM_foot_l.STL` e applicata la stessa
estensione locale di suola virtuale di 25 mm.

Il primo tentativo, con i punti esattamente sul mesh, è stato chiuso in FAIL
prima di qualsiasi lavoro sugli eventi: la punta restava `0` su tutti i
campioni DEV02 e DEV04. Ciò conferma che il mesh di visualizzazione non coincide
con la superficie di collisione effettiva del simulatore.

Sono state controllate quattro estensioni comuni usando soltanto continuità e
alternanza dei bit grezzi, senza oracle HS/TO: 15, 18, 20 e 25 mm. A 15/18 mm
la punta restava sostanzialmente sempre OFF; a 20 mm era ON soltanto per
0,18–0,80% del tempo. La variante 25 mm è la prima controllata in cui entrambi
i canali producono blocchi ON/OFF sostenuti su entrambi i trial development.

Coordinate finali:

- tallone: `[-0.0946600475, -0.06990531781384512, 0.01399567]`;
- punta: `[0.11574858501553537, -0.06403460020354058, 0.0030479021621026177]`.

## Trasporto causale

Il runner campiona la geometria a 1 ms. Al reset legge `t0` come baseline
diagnostica ma non lo emette. Ogni step normale da 10 ms restituisce esattamente
dieci campioni unici in `(previous_sample_time, t_stop]`, incluso `t_stop`.

Campioni mancanti, duplicati, non monotoni, off-grid, non finiti o contatti non
booleani falliscono in modo esplicito. Il profilo analogico a sfere e quello
binario sono mutuamente esclusivi.

## Replay raw development

Il validatore V19 ha riprodotto gli interi intervalli congelati DEV02/04 a
1 ms usando soltanto IK marker-based, modello e geometria. Non ha letto GRF,
oracle eventi o FSM.

| Metrica raw | DEV02 | DEV04 |
|---|---:|---:|
| Campioni | 143.206 | 143.541 |
| Tallone ON | 50,87% | 52,19% |
| Punta ON | 33,41% | 35,25% |
| Almeno uno ON | 68,02% | 68,98% |
| Entrambi OFF | 31,98% | 31,02% |
| Transizioni raw tallone | 258 | 266 |
| Transizioni raw punta | 338 | 360 |
| Non finiti / non booleani | 0 / 0 | 0 / 0 |

Questi numeri attestano soltanto che la geometria produce due sequenze binarie
finite e non degeneri. Non attestano accuratezza HS/TO.

## File modificati o aggiunti

- `binary_phase_detector.py`;
- `config.py`;
- `path_resolver.py`;
- `model_loader.py`;
- `simulation_runner.py`;
- `validation/experimental_detector_profiles/two_point_binary_v19_outsole_25mm.json`;
- `validation/experimental_detector_profiles/README.md`;
- `validation/validate_binary_phase_detector_v19_raw_geometry.py`;
- `validation/test_binary_phase_detector_v19.py`;
- `validation/binary_phase_detector_v19_geometry_receipt.json`.

## Verifiche

- 11/11 test V19 nuovi PASS;
- 24/24 test routing detector esistenti PASS;
- 5/5 test trasporto high-rate/FSM legacy esistenti PASS;
- 7/7 test core online-GRF PASS;
- 6/6 test contratto V17 PASS;
- 12/12 test gate development V17 PASS;
- 61/61 test primary-GRF training readiness PASS, 1 skip atteso;
- smoke OpenSim reale: reset + 10 ms, 10/10 campioni booleani PASS;
- replay raw completo DEV02/04 PASS;
- `git diff --check` PASS.

La GRF primaria, `online_grf.py`, la FSM e la semantica SEA sono rimaste
intatte. Trial protetti e reserve non sono stati aperti. Il claim numerico del
replay resta macOS arm64; l'implementazione Python non contiene path o API
specifici di macOS/Windows.

## TODO da discutere

Definire separatamente la logica che trasforma i due bit in HS/TO. Finché tale
decisione non viene congelata:

- `legacy_events` resta autorevole;
- il nuovo stream non entra nelle osservazioni né nella FSM;
- non viene valutata latenza eventi;
- non viene avviato PPO o `H0_sep`.
