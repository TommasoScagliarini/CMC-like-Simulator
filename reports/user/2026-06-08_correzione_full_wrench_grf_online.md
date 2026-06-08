# Correzione full-wrench e residuale state-based GRF online

Data: 2026-06-08

## Stato finale

La modalita `online` e stata migliorata in modo sostanziale, ma **non e ancora
promossa a validated**.

Il problema tangenziale/momento e stato corretto con un residuale puramente
state-based. Il gate dinamico verticale sul run da 500 ms resta pero fallito:

- miglior profilo candidato: `AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`;
- `pelvis_ty` reserve p95: `45.41 N`;
- rapporto active/sensor: `1.818x`, soglia `1.5x`;
- acceptance complessiva: `FAIL`, 31 criteri passati e 12 falliti.

Il profilo resta quindi `requires_forward_validation`.

## Problema

La precedente calibrazione considerava soprattutto forza verticale e impulso.
Questo nascondeva differenze dinamicamente importanti:

- forza tangenziale destra online praticamente nulla;
- centro di pressione e momenti non inclusi nel fit/gate;
- compensazione quasi diretta dell'errore GRF tramite reserve della pelvi;
- calibrazioni memoryless valide in replay ma non robuste in dinamica attiva.

La correlazione tra mismatch di forza prescribed-online e incremento delle
reserve della pelvi resta molto alta nel run attivo da 500 ms:

- `pelvis_tx`: `0.9995`;
- `pelvis_ty`: `0.9989`;
- `pelvis_tz`: `0.9216`.

## Soluzione implementata

### Calibrazione e validazione full-wrench

La calibrazione sparse-basis ora:

- usa una patch plantare 2D, non una sola linea;
- fitta forza e momento insieme;
- supporta stati salvati da rollout attivi e piu traiettorie;
- usa stiffness bounded;
- riporta forza, momento, COP ed eventi su calibration e holdout.

Il validator forward e l'acceptance gate controllano ora full-wrench,
penetrazione, impulso per lato, sink e reserve.

### Residuale state-based

Il plugin `OnlineGRFContact`, separato dal plugin SEA, supporta ora un residuale
opzionale che:

- e proporzionale alla forza normale istantanea;
- puo dipendere da penetrazione e penetration-rate;
- puo aggiungere forza tangenziale e momento libero;
- non usa tempo, fase o prescribed GRF a runtime;
- e zero di default, preservando la legge di contatto precedente.

Il calibratore dedicato e:

`validation/calibrate_online_grf_residual.py`

### Watchdog

Il simulatore pubblica heartbeat atomici quando riceve
`CMC_SIM_HEARTBEAT_FILE`. Il watchdog supervisiona startup, stallo e timeout
totale e registra correttamente l'heartbeat finale `complete`.

## Risultati principali

| Profilo/run | Durata | Impulso totale | pelvis_ty p95 | active/sensor |
|---|---:|---:|---:|---:|
| tuned senza residuale | 100 ms | 0.9994x | 9.71 N | 1.221x |
| tuned senza residuale | 500 ms | 1.0202x | 46.34 N | 1.855x |
| residuale tangenziale v2 | 100 ms | 1.0050x | 11.49 N | 1.444x |
| residuale tangenziale v2 | 500 ms | 1.0203x | 45.41 N | 1.818x |

Nel probe v2 da 100 ms, l'errore medio della forza destra
`[Fx, Fy, Fz]` diventa circa:

`[0.004, -0.350, 0.198] N`

La forza tangenziale destra, prima strutturalmente assente, viene quindi
ricostruita quasi completamente.

Nel run v2 da 500 ms:

- impulso sinistro: `1.0773x`;
- impulso destro: `0.9533x`;
- penetrazione massima sinistra/destra: `10.78 / 10.12 mm`;
- max sink pelvi: `0.038 mm`;
- run completato senza guard trip o stallo.

## Candidati scartati

- patch densa calibrata sugli stati attivi: probe 100 ms `6.32x`;
- residuale full-wrench costante v1: probe 100 ms `3.58x`;
- residuale verticale state-based conservativo v3: probe `1.556x`;
- residuale verticale v3 scalato al 25%: probe `1.565x`.

Nessuno di questi candidati e stato esteso automaticamente dopo il fallimento
del probe breve.

## File modificati

- `online_grf.py`
- `simulation_runner.py`
- `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.h`
- `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp`
- `tools/online_grf_contact/README.md`
- `online_grf_profiles/README.md`
- `validation/calibrate_online_grf_basis.py`
- `validation/calibrate_online_grf_residual.py`
- `validation/validate_online_grf.py`
- `validation/verify_online_grf_plugin.py`
- `validation/validate_online_grf_forward_drift.py`
- `validation/online_grf_acceptance.py`
- `validation/online_grf_acceptance_thresholds.json`
- `validation/test_online_grf_core.py`
- `Trajectory Generator/baseline_MLP/process_watchdog.py`

## Test e verifiche

- build Windows Release `OnlineGRFContact.dll`: PASS;
- parita Python/plugin su forza e momento: errore massimo `0`;
- retrocompatibilita profili senza residuale: PASS;
- `python -m py_compile`: PASS;
- `validation.test_online_grf_core`: 7/7 PASS;
- `git diff --check`: PASS, sole warning LF/CRLF;
- watchdog self-test con stallo intenzionale: PASS, terminazione in circa 1.76 s;
- tutti i probe e run da 500 ms eseguiti sotto watchdog;
- acceptance miglior candidato v2: FAIL, 31 PASS / 12 FAIL.

## Conclusione tecnica

Il problema non e piu la sola assenza di forza tangenziale o di momento:
queste componenti sono ora rappresentabili e numericamente coerenti tra
Python e plugin.

Il limite residuo e la generalizzazione verticale della legge di contatto
sulla dinamica forward. Un residuale verticale lineare basato soltanto su
penetrazione e penetration-rate non migliora il probe breve e tende a saturare
i bound sul lato destro. Aggiungere altre sfere o continuare a regolare
guadagni costanti non e giustificato dai risultati.

## TODO

- Raccogliere stati attivi che coprano almeno un intero ciclo di gait, con
  eventi di contatto reali, e usare split per fase/stato invece del solo tratto
  iniziale da 500 ms.
- Valutare una legge tangenziale stateful con stiction/deflessione tangenziale,
  invece di ulteriori patch memoryless.
- Ricalibrare il residuale su piu rollout attivi e validarlo su rollout non
  usati dal fit.
- Eseguire `online_sensor` e `online` sullo stesso profilo per una finestra
  gait-scale, poi rieseguire l'acceptance gate.
- Ricompilare e riconfermare il plugin modificato su macOS arm64.
- Non promuovere alcun profilo a `validated` finche
  `active_vs_sensor.pelvis_ty_reserve_p95_ratio <= 1.5`.
