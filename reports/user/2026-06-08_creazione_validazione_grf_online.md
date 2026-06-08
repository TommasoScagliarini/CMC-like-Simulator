# Creazione e validazione GRF online

Data: 2026-06-08

## Obiettivo

Creare un profilo GRF online applicabile alla dinamica e validarlo con
particolare attenzione alla plausibilita fisica. Il piano
`2026-06-08_piano_calcolo_puro_grf_online.md` e stato usato come traccia, ma
i rollout lunghi sono stati subordinati a gate progressivi.

## Problema iniziale

Il profilo `online_sensor_basis` riproduce bene la GRF prescritta su IK replay,
ma non e fisicamente utilizzabile come contatto attivo:

- penetrazione holdout: `102 mm` sinistra, `43.5 mm` destra;
- rollout active precedente: `95.7 mm` sinistra, `32.1 mm` destra;
- reserve `pelvis_ty` p95: `5.01x` il prescribed;
- il simulatore completava comunque il run, nascondendo il problema tramite
  le reserve root.

La validazione precedente dimostrava parita numerica Python/C++ e accuratezza
sensor, non la validita fisica del contatto applicato.

## Soluzione implementata

### Validazione fisica separata dalla sensor accuracy

- `validation/validate_online_grf.py` ora riporta anche penetrazione, frazione
  di contatto e slip per lato/sfera.
- Nuovo `validation/validate_online_grf_forward_drift.py`: confronta run
  active, sensor e prescribed misurando deriva `pelvis_ty`, reserve root,
  impulso verticale, penetrazione e slip.
- Nuovo gate `validation/online_grf_acceptance.py` con soglie esplicite in
  `validation/online_grf_acceptance_thresholds.json`.
- Il confronto reserve principale usa active vs `online_sensor` sulla stessa
  finestra temporale.

### Creazione profilo active fisico

`validation/calibrate_online_grf_basis.py` supporta ora
`--maximum-penetration`:

- abbassa automaticamente il piano;
- scarta i contatti candidati che superano il limite;
- usa esponenti fisici `>= 1`;
- limita la stiffness;
- salva metriche fisiche e stato `requires_forward_validation`.

Il candidato migliore mantenuto e:

`online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_physical_basis_10mm_balanced.json`

### Protezione runtime

In active mode il simulatore interrompe il run se una penetrazione supera
`online_grf_max_penetration_m`, default `0.03 m`. Il vecchio profilo sensor
viene ora fermato immediatamente a `94.8 mm`.

`in_contact` ora rappresenta contatto geometrico (`penetration > 0`) e non la
piccola forza residua introdotta dallo smoothing.

## Risultato candidato balanced, rollout active 0.5 s

- plugin Python/C++: errore massimo `0 N`;
- penetrazione active massima: `9.75 mm` sinistra, `3.78 mm` destra;
- impulso active totale: `0.953x` prescribed;
- impulso per lato: `0.869x` sinistra, `1.053x` destra;
- max sink `pelvis_ty`: `0.073 mm`;
- reserve `pelvis_ty` p95: `148.4 N`;
- reserve p95 active/sensor stessa finestra: `5.94x`.

Gate automatico: **FAIL, 20/21 criteri passati**.

L'unico failure del candidato balanced e l'uso eccessivo della reserve
verticale. Non e stato quindi promosso a profilo active validato e non sono
stati eseguiti rollout piu lunghi.

## File modificati o aggiunti

- `config.py`
- `main.py`
- `simulation_runner.py`
- `online_grf.py`
- `online_grf_profiles/README.md`
- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_physical_basis_10mm_balanced.json`
- `validation/calibrate_online_grf_basis.py`
- `validation/tune_online_grf_support.py`
- `validation/validate_online_grf.py`
- `validation/validate_online_grf_forward_drift.py`
- `validation/online_grf_acceptance.py`
- `validation/online_grf_acceptance_thresholds.json`
- `validation/test_online_grf_core.py`

## Test e verifiche

- `validation/test_online_grf_core.py`: `6/6` PASS.
- `validation/verify_online_grf_plugin.py`: errore massimo `0 N`.
- validazione IK replay con metriche fisiche per profili sensor, marker,
  preliminare e candidati active.
- rollout progressivi active/sensor: `0.1 s`, poi `0.5 s`.
- probe del guard runtime: PASS, il profilo da `94.8 mm` viene abortito.
- `py_compile`: PASS.
- `git diff --check`: PASS.
- acceptance balanced: atteso `FAIL`, `20/21` criteri PASS.

## TODO

- Ridurre la reserve `pelvis_ty` active/sensor p95 da `5.94x` a `<= 1.5x`
  lavorando sulla forma temporale della forza normale, in particolare
  dissipazione/materiali e distribuzione dei contatti, non su un semplice
  scaling dell'impulso.
- Ripetere il gate a `0.5 s`; estendere a `1 s`, `2 s` e passo completo solo
  dopo PASS.
- Validare successivamente con `sea_forward_mode=plugin`; i rollout
  diagnostici attuali isolano il contatto usando `ideal_torque`.
