# Validazione TODO cascade Jeff con J_eff

Data: 2026-05-19

## Problema

Il file `2026-05-19_cascade_redesign_jeff_analysis.md` proponeva un redesign
cascade basato su `J_eff` validata, ma lasciava alcuni TODO prima di promuovere
il design: scelta operativa di `J_knee`, saturazione del motor ankle V3',
chattering HPF50, variante PI con `p/omega_n=0.1` e necessita' di ripetere
`J_eff` dopo aggiornamenti modello.

## Strategia

Ho lasciato invariata la configurazione attiva e ho creato validazioni dedicate:

- step analitico su `qdot_ref` usando la matrice di massa OpenSim campionata
  lungo l'IK;
- modelli `.osim` derivati solo per cambiare i gain del motor driver;
- screen di simulazione sulla finestra critica `13.1638 -> 14.7799 s`;
- confronto con il morning-best sulla stessa finestra;
- variante `p/omega_n=0.1` per verificare se il miss zero/polo PI dominava.

## File modificati

- `2026-05-19_cascade_redesign_jeff_analysis.md`
- `validation/cascade_qdot_step_inertia.py`
- `validation/cascade_jeff_todo_runner.py`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap_p01.osim`

## Risultati principali

Step `delta_qdot_ref=1 rad/s` sul knee:

| design | qddot free | qddot pair | qddot locked |
|---|---:|---:|---:|
| morning_best | 182.8 | 65.62 | 59.13 |
| V3a J_free | 168.3 | 60.41 | 54.44 |
| V3m J_mid | 284.0 | 101.9 | 91.86 |

`J_free_all` predice ~2.79x piu' accelerazione di `J_free_pros_pair`.
`J_free_pros_pair` e `J_locked` sono invece vicini, quindi il knee non e'
ben descritto da un singolo `J` scalare senza validazione in simulazione.

Screen push-off `13.1638 -> 14.7799 s`:

| run | knee RMSE deg | ankle RMSE deg | knee max u | ankle max u | sat knee/ankle |
|---|---:|---:|---:|---:|---:|
| morning_best | 0.207 | 2.410 | 0.654 | 0.636 | 0 / 0 |
| V3a J_free | 0.180 | 15.614 | 1.000 | 0.549 | 0 / 0 |
| V3m J_mid | 0.463 | 15.118 | 1.000 | 0.549 | 0 / 0 |
| V3' motor cap | 0.404 | 11.318 | 1.000 | 0.536 | 0 / 0 |
| V3' p/on=0.1 | 0.291 | 11.246 | 1.000 | 0.521 | 0 / 0 |

HPF50:

| run | knee tau HPF50 | ankle tau HPF50 | knee motor_speed_dot HPF50 | ankle motor_speed_dot HPF50 |
|---|---:|---:|---:|---:|
| morning_best | 15.69 | 19.07 | 1688 | 2102 |
| V3a J_free | 8.22 | 1.28 | 968 | 338 |
| V3m J_mid | 81.17 | 1.37 | 10671 | 479 |
| V3' motor cap | 74.08 | 2.30 | 9784 | 566 |
| V3' p/on=0.1 | 37.80 | 1.82 | 5149 | 422 |

## Decisione

Il redesign teorico non va promosso. Riduce il chattering ankle, ma perde
troppo tracking cinematico: l'RMSE ankle nella finestra critica passa da
2.41 deg del morning-best a 11.25-15.61 deg. La variante `p/omega_n=0.1`
riduce il chattering knee rispetto a V3'/V3m, ma non recupera il tracking
ankle; quindi il miss zero/polo PI non e' la causa dominante.

La configurazione morning-best resta la baseline consigliata.

## Verifiche eseguite

- `/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile validation/cascade_qdot_step_inertia.py validation/cascade_jeff_todo_runner.py`
- `validation/cascade_qdot_step_inertia.py --out-dir results/_cascade_qdot_step_20260519`
- `validation/cascade_jeff_todo_runner.py --variants v3a,v3m,v3cap --t-start 13.1638 --t-end 14.7799`
- `validation/cascade_jeff_todo_runner.py --variants morning_best --t-start 13.1638 --t-end 14.7799`
- `validation/cascade_jeff_todo_runner.py --variants v3cap_p01 --t-start 13.1638 --t-end 14.7799`

## TODO

- Ripetere `J_eff` solo dopo aggiornamenti a geometria, masse, inerzie o
  struttura fisica del modello OpenSim.
