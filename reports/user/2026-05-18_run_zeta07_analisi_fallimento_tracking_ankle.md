# Analisi run zeta07: fallimento tracking ankle - 2026-05-18

## Problema

Lanciata una full simulation con il nuovo design motor driver + cascade
zeta07 progettato nella sessione del pomeriggio (report
`2026-05-18_motor_driver_cascade_redesign_zeta07.md`):

```text
setup: AB06_SEASEA_stiff321_500_pi_zeta07_setup.xml
model: AB06_SEASEA_stiff321_500_pi_zeta07.osim
cascade: knee Kp_p=14.4, Kp_v=31.85, Ki_v=1638, I_lim=50
         ankle Kp_p=11.2, Kp_v=1.51, Ki_v=60.5, I_lim=80
```

Il run e' terminato `complete` in 794 s wall-time senza saturazioni
`tau_input` ne saturazioni `u`. Il tracking cinematico ankle pero' e'
**catastrofico**: RMSE 27.5 deg contro 1.22 deg della baseline cascade
aggressive ankle5.

Necessario capire la causa del fallimento e valutare se il design del
motor driver e' valido a parte il problema cascade.

## Strategia

Analisi quantitativa multi-livello dei file di output (sea_diagnostics,
states, sea_controls), confronto numerico contro due baseline run:

```text
results/_cascade_aggressive_full_20260517
results/_cascade_full_ankle5_20260518_105345/full_runs/full_ankle5_*
```

Metriche calcolate:

1. **Verifica caricamento modello**: lettura di `inner_integral_gain` dal
   diagnostics per confermare che il plugin abbia i nuovi Ki.
2. **Tracking RMSE** kinematic vs IK reference per knee e ankle.
3. **Saturazioni**: `tau_input_saturated`, `|u|>0.95`, cascade
   integral clamp, plugin xi.
4. **Chattering HPF >50 Hz** su `tau_input_plugin`, `motor_speed`,
   `motor_speed_dot_plugin`.
5. **Driver tracking interno**: `tau_error` RMS.
6. **Plot di confronto** tracking ankle/knee e segnali interni ankle.

## Soluzione (diagnosi e direzione)

### Modello caricato correttamente

```text
Ki_plugin knee  = 290.700 (atteso 290.7)  OK
Ki_plugin ankle =  87.810 (atteso 87.81)  OK
```

Il run e' effettivamente quello del design zeta07.

### Knee — tracking OK, motor speed chattering peggiore

| Metrica | zeta07 | baseline ankle5 | Delta |
|---|---:|---:|---:|
| RMSE knee | 0.187 deg | 0.178 deg | +5 % |
| tau_input no sat | 0 | 0 | OK |
| u max | 0.37 | 0.33 | +12 % |
| HPF50 tau_input | 4.14 Nm | 4.33 Nm | -4 % |
| HPF50 motor_speed | 1.47 rad/s | 0.63 rad/s | **+133 %** |
| HPF50 motor_speed_dot | 566 rad/s2 | 475 rad/s2 | +19 % |
| tau_error RMS | 3.23 Nm | 0.96 Nm | **+236 %** |

Il knee mantiene il tracking ma:

- chattering motor_speed in HPF aumenta di 2.4x,
- tau_error del driver triplica perche' la banda motor driver e'
  scesa da 124 Hz a 57 Hz e l'integratore plugin (al limite di
  banda 11.5 Hz) non riesce a inseguire le richieste rapide della
  cascade.

### Ankle — tracking distrutto

| Metrica | zeta07 | baseline ankle5 | Delta |
|---|---:|---:|---:|
| **RMSE ankle** | **27.49 deg** | 1.220 deg | **+2153 %** |
| tau_input no sat | 0 | 0 | OK |
| u max | 0.49 | 0.48 | +2 % |
| cascade I clamp | **14.7 %** del tempo | 0 % | saturato |
| cascade xi_v max | 1.32 | 0.56 | +136 % |
| plugin xi max | 1.04 | 0.16 | **+550 %** |
| motor_speed RMS | 5.07 rad/s | 0.81 rad/s | **+526 %** |
| motor_speed max | 22.2 rad/s | 6.9 rad/s | +222 % |
| HPF50 tau_input | 0.83 Nm | 2.24 Nm | **-63 %** |
| HPF50 motor_speed_dot | 86 rad/s2 | 248 rad/s2 | **-65 %** |
| tau_error RMS | 3.93 Nm | 0.67 Nm | **+486 %** |

Dal plot `tracking_comparison.png` la posizione ankle ha picchi fino a
**+130 deg** contro riferimento ~20 deg, con grosse escursioni
positive ad ogni push-off.

### Causa del fallimento ankle

Identificate tre cause concorrenti:

1. **Banda position-loop ankle troppo bassa**: `Kp_p_ankle = 11.2 1/s`
   corrisponde a costante di tempo 89 ms e bandwidth 1.8 Hz. Il
   push-off ankle ha eventi a 5-10 Hz, la position loop non riesce
   ad inseguirli.

2. **Cascade integrator saturato per il 14.7 % del tempo** all'I_limit
   nuovo di 80 Nm. Combinato con `Ki_v=60.5` (ridotto da 213), l'errore
   integrato cresce velocemente al clamp e non puo' produrre tau_cmd
   sufficiente.

3. **Banda velocity-loop ankle troppo bassa**: `omega_v=56 rad/s
   (8.9 Hz)`, sotto la banda spettrale degli eventi push-off.

L'assunzione che J_joint ankle = 0.0193 kg*m^2 dal back-calc del
cascade attuale a `zeta=0.7` potrebbe essere errata. Se il cascade
precedente operava effettivamente a `zeta<0.7`, il J reale sarebbe
piu' alto e l'omega_v stimato e' sbagliato.

### Chattering — risultati misti

L'unico aspetto positivo del nuovo design:

- ankle HPF50 tau_input scende da 2.24 a 0.83 Nm (**-63 %**),
- ankle HPF50 motor_speed_dot scende da 248 a 86 rad/s2 (**-65 %**),
- knee HPF50 tau_input invariato.

Questo conferma che il motor driver ha effettivamente ridotto la sua
banda HF e l'amplificazione del rumore dal cascade.

Pero':

- knee HPF50 motor_speed aumenta del 133 %,
- knee tau_error triplica.

Il motor driver knee con il nuovo design e' diventato troppo "morbido"
in coppia (Kp_plugin 18 -> 4.17) ed espone piu' alle perturbazioni
veloci dalla cascade.

### Verdetto

Il design zeta07 e' un **fallimento operativo** sul tracking ankle.
Il guadagno di chattering ankle e' annullato dall'errore di posizione
di 23x peggio della baseline.

La regola "separazione 5x + zeta=0.7" applicata uniformemente e'
troppo conservativa per gli eventi gait push-off ankle. La banda
position e velocity ankle deve essere dimensionata sulla **dinamica
spettrale reale del riferimento**, non solo sulla teoria del cascade.

## File modificati

Nessun file di codice o config modificato in questa sessione:
l'analisi e' stata eseguita su run gia' prodotti dalla simulazione
lanciata dall'utente. File creati:

```text
plot/05_18_2026_zeta07_run_analysis/tracking_comparison.png
plot/05_18_2026_zeta07_run_analysis/ankle_internal_signals.png
```

Script di analisi non persistente in `/tmp/analyze_zeta07_run.py`
(usato una sola volta).

## Verifiche eseguite

- letta `sim_output_run_status.txt`: `status=complete`, t=21 s, 9010
  step, dt=1ms, wall=794 s.
- letti `sim_output_sea_diagnostics.sto`, `sim_output_states.sto`,
  `sim_output_sea_controls.sto`.
- confrontato `inner_integral_gain` plugin con i valori attesi del
  design zeta07: PASS.
- calcolato kinematic RMSE rispetto a `IK_results_AB06_SEASEA.mot`
  interpolato sui timestamp del run.
- calcolato saturazioni `tau_input` (entrambi 0) e `|u|>0.95`
  (entrambi 0).
- calcolato cascade I clamp ankle: 1328/9010 = 14.7 %.
- calcolato HPF50 RMS via filtro IIR del primo ordine.
- confrontato tutti i numeri con due baseline:
  `_cascade_aggressive_full_20260517` e
  `_cascade_full_ankle5_20260518_105345/full_runs/...`.

## TODO

Propagati da `2026-05-18_motor_driver_cascade_redesign_zeta07.md`,
aggiornati o nuovi:

- **Re-tuning cascade ankle** mantenendo motor driver zeta07:
  riportare `Kp_p_ankle` a 30-40 1/s (posizione 5-6 Hz), `omega_v_ankle`
  a ~90-100 rad/s (15 Hz) => `Kp_v_ankle ~ 2.5`, `Ki_v_ankle ~ 175`.
  Verificare separazione motor/velocity >= 3x.

- **Valutare se alzare omega_motor_ankle** da 280 a ~320 rad/s
  (vicino al saturation cap 316) per ripristinare la separazione 5x.

- **Validare J_joint con run dedicato**: smoke con step in `qdot_ref`
  e misurare l'inerzia effettiva del joint dalla risposta `tau→qdot`.
  Aggiornare il design cascade con J reale.

- **Knee**: chattering motor_speed peggiorato di 2.4x e tau_error
  triplicato. Considerare un Kd_plugin knee piu' alto per recuperare
  smorzamento HF, oppure riportare omega_motor_knee a 500 rad/s.

- **Decidere se la regola 5x e' troppo rigida**: validare con
  3x come compromesso (knee 360→120 rad/s vel, ankle 280→93 rad/s vel).

- **Valutare impatto della mancata cancellazione zero/polo ankle**
  (TODO propagato): in questo run il 22% di miss e' irrilevante,
  l'integratore e' saturato e domina ogni effetto sottile della
  cancellazione.

- **Windows pendente**: build/copia DLL plugin PI sulla macchina
  Windows + smoke load/run e verifica del modello PI con
  `Ki`, `integral_torque_limit`, `torque_error_integral`.

- **Pulizia artefatti**: valutare cancellazione cartelle di sweep
  parziali (`_cascade_local_gain_sweep_20260517_233607`,
  `_cascade_local_gain_sweep_20260517_234151`).
