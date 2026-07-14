# Daily Report - 2026-04-30

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-04-30_finestra_fp12_modello_seasea_simulazione.md`

## Problema

Serviva scegliere una finestra con GRF realmente misurate per una simulazione
CMC-like SEASEA, confrontando force plate `FP1/FP2` e treadmill, e produrre un
bundle locale stabile e riproducibile.

## Soluzione e strategia

E' stato creato `scripts/run_measured_grf_window_tests.py` per eseguire IK,
ExternalForces, ID, sweep RRA, promozione del modello e test CMC-like per ogni
finestra. Sono stati aggiunti timeout, pulizia delle cartelle run e mapping
anatomico corretto delle forze treadmill.

La finestra scelta e':

```text
FP1/FP2 misurata: 15.889-19.839 s
finestra operativa: 17.500-19.000 s
```

La finestra treadmill `30.065-34.275 s` e' stata scartata dopo timeout e
divergenza non-finita anche con mapping corretto.

## Risultati

- simulazione locale: `1500` step, status `complete`;
- nessun valore non finito e nessuna saturazione SEA;
- reserve norm mean/max: `233.684/385.845`;
- pelvis reserve max: `374.208 Nm`;
- tredici fallback SO feasibility;
- plot completi in `plot/30_04_2026 - 1`.

Il modello operativo era:

```text
models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_ready.osim
```

## File e artefatti

- `scripts/run_measured_grf_window_tests.py`
- `models/SEASEA - whealthy data/Adjusted_newmarkers_fp12_15889_19839_setup.xml`
- `models/SEASEA - whealthy data/data/IK_fp12_15889_19839.mot`
- `models/SEASEA - whealthy data/data/ExternalForces_fp12_15889_19839.xml`
- `results/fp12_17500_19000_local_model/`
- `plot/30_04_2026 - 1/`

## Test e verifiche

- `py_compile` del runner: PASS;
- test timeout: PASS;
- run FP e treadmill confrontate;
- run locale `17.5-19.0 s`: PASS;
- generazione plot e valutazione candidati: PASS.

## TODO e stato successivo

- [x] Selezionare una finestra measured-GRF stabile per debug locale.
- [x] Il tentativo di estendere questo specifico trial oltre `19.839 s` e'
      stato superseduto dalla successiva pipeline AB06 full-span del maggio
      2026.
- [x] Le reserve pelvis elevate restano un limite scientifico documentato del
      candidato FP12 e impediscono di trattarlo come baseline biomeccanica.

