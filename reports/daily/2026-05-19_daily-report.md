# 2026-05-19 Daily Report

## Sintesi

Giornata centrata sul problema del chattering nel cascade SEA. Quattro
linee di lavoro, una soluzione operativa trovata:

1. **Validazione J_eff** (matrice di massa OpenSim) - chiude il TODO 18/05
   sull'inerzia effettiva di knee/ankle.
2. **Redesign cascade teorico con J_eff corretta** (documento root) - 4
   varianti (V1, V2, V3, V3') con calcoli completi.
3. **Validazione redesign cascade** - 5 run di test su push-off window,
   tutte falliscono il tracking ankle. Morning best resta baseline.
4. **Diagnosi origine chattering + verifica 4 ipotesi + Opzione D**:
   chattering nasce dal feedback `qdot` del joint amplificato dal velocity
   loop. Implementato LPF su `qdot_cur`. **Riduce chattering del 53-86%
   preservando l'ankle tracking**.

Configurazione attiva a fine giornata: invariata (morning best 18/05).
Il LPF qdot e' implementato e validato ma disabilitato di default
(`sea_qdot_feedback_lpf_cutoff_hz = 0` su entrambi i SEA).

## Report utente creati oggi

```text
reports/user/2026-05-19_validazione_inerzia_effettiva_knee_ankle.md
reports/user/2026-05-19_validazione_todo_cascade_jeff.md
reports/user/2026-05-19_diagnosi_chattering_motor_vs_cascade.md
reports/user/2026-05-19_optD_lpf_qdot_feedback.md
```

Documento di analisi in root:

```text
2026-05-19_cascade_redesign_jeff_analysis.md
```

## 1. Validazione J_eff knee/ankle

Report sorgente:

```text
reports/user/2026-05-19_validazione_inerzia_effettiva_knee_ankle.md
```

Nuovo script `validation/effective_joint_inertia.py` che campiona la
matrice di massa OpenSim lungo l'IK del setup AB06 PI (181 sample,
11.99 -> 21.0 s) e calcola tre stime di inerzia effettiva:

- `J_free_all = 1 / (M^-1)_ii`: tutti i DOF liberi (risposta impulsiva)
- `J_locked = M_ii`: altri DOF bloccati ad accelerazione zero
- `J_free_pros_pair`: liberi solo i due DOF protesici

Risultati mediani:

| coord | J_back zeta07 | J_free_all | J_locked | J_free_pros_pair |
|---|---:|---:|---:|---:|
| pros_knee_angle | 0.316 | 0.159738 | 0.493791 | 0.444954 |
| pros_ankle_angle | 0.0193 | 0.009892 | 0.011098 | 0.010000 |

Verdetto: il back-calc del 2026-05-18 sovrastimava ~2x l'inerzia libera.
Ankle e' robusto (3 stime convergono a `~0.010`). Knee dipende fortemente
dall'accoppiamento (0.16-0.49).

Artefatti:

```text
validation/effective_joint_inertia.py
results/_effective_joint_inertia_20260519/
```

## 2. Redesign cascade con J_eff corretta (teorico)

Documento root:

```text
2026-05-19_cascade_redesign_jeff_analysis.md
```

Computate quattro varianti del cascade outer mantenendo motor driver
zeta07 (knee 4.17/5.66/290.7, ankle 1.01/4.38/87.81):

| variante | gerarchia | knee cascade | ankle cascade |
|---|---|---|---|
| V1 | 5x/5x | Kp_p=14.4, Kp_v=16.13, Ki_v=829 | Kp_p=11.2, Kp_v=0.78, Ki_v=31.4 |
| V2 | 5x/3x | Kp_p=24, Kp_v=16.13, Ki_v=829 | Kp_p=18.7, Kp_v=0.78, Ki_v=31.4 |
| V3 | 3x/3x | Kp_p=40, Kp_v=26.88, Ki_v=2304 | Kp_p=31.1, Kp_v=1.31, Ki_v=87 |
| V3' | 3x/3x + motor cap | come V3 | Kp_p=35.1, Kp_v=1.47, Ki_v=110.9 |

Re-interpretazione del morning best con J_eff:

- Ankle: omega_v_actual = 146 rad/s, zeta_actual = 0.97 (sovrasmorzato)
- Knee con J_free: zeta_actual = 0.98 (sovrasmorzato)
- Knee con J_pair: zeta_actual = 0.59 (sotto-smorzato)

Configurazione teorica raccomandata (mai promossa):

```text
Motor driver:
  knee:  Kp=4.17, Kd=5.66, Ki=290.7
  ankle: Kp=1.56, Kd=4.96, Ki=126.2  (omega_n=316 rad/s al cap)

Cascade outer:
  knee  (J=0.27 medio): Kp_p=40, Kp_v=45.36, Ki_v=3888, I_limit=50
  ankle (J=0.010):      Kp_p=35.1, Kp_v=1.474, Ki_v=110.88, I_limit=80
```

## 3. Validazione del redesign cascade

Report sorgente:

```text
reports/user/2026-05-19_validazione_todo_cascade_jeff.md
```

Lanciate 5 simulazioni sulla finestra critica `13.1638 -> 14.7799 s` con
`validation/cascade_jeff_todo_runner.py`. Esiti:

| run | knee RMSE | ankle RMSE | max u | esito |
|---|---:|---:|---:|---|
| morning_best | 0.207 | 2.410 | 0.654 | OK baseline |
| V3a J_free | 0.180 | 15.614 | 1.000 | FAIL u=1 |
| V3m J_mid | 0.463 | 15.118 | 1.000 | FAIL u=1 |
| V3' motor cap | 0.404 | 11.318 | 1.000 | FAIL u=1 |
| V3' p/on=0.1 | 0.291 | 11.246 | 1.000 | FAIL u=1 |

Step `delta_qdot_ref=1 rad/s` su knee (validation/cascade_qdot_step_inertia.py):

```text
J_free_all ~ 2.79x J_free_pros_pair sull'accelerazione predetta
=> knee non e' descritto da un singolo J scalare senza validazione
   in simulazione
```

Verdetto: redesign teorico non promosso. Il chattering si riduce ma il
tracking ankle peggiora 4.7x-6.5x. Il miss zero/polo PI non era la causa
dominante (V3' p/on=0.1 non recupera).

Artefatti:

```text
validation/cascade_qdot_step_inertia.py
validation/cascade_jeff_todo_runner.py
results/_cascade_qdot_step_20260519/
results/_cascade_jeff_todo_20260519_screen_morning_pushoff1/
results/_cascade_jeff_todo_20260519_screen_pushoff1/
results/_cascade_jeff_todo_20260519_screen_p01_pushoff1/
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap_p01.osim
```

## 4. Tentativo Opzione B (motor wn=500) e Opzione C (Kd=15)

Estensione di `cascade_jeff_todo_runner.py` con due varianti conservative
sul motor driver, cascade morning best invariata:

- **Opzione B** (ankle motor omega_n=500 rad/s, zeta=0.7): Kp=5.4, Kd=7.9,
  Ki=500
- **Opzione C** (ankle Kd=15, banda invariata): zeta passa da 0.71 a ~0.97

Risultati push-off `13.1638 -> 14.7799 s`:

| run | knee RMSE | ankle RMSE | tau HPF50 | mdot HPF50 |
|---|---:|---:|---:|---:|
| morning_best | 0.207 | 2.410 | 19.07 | 2102 |
| opt_b wn=500 | 0.208 | 2.404 | 17.35 (-9%) | 2225 (+6%) |
| opt_c Kd=15 | 0.207 | 2.419 | 18.61 (-2.4%) | 2069 (-1.6%) |

Tracking preservato in entrambi i casi, ma riduzione chattering
**marginale**. Il motor driver tuning non e' la leva giusta.

Artefatti:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optB_ankle_wn500.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optC_ankle_kd15.osim
results/_cascade_optB_20260519_pushoff/
results/_cascade_optC_20260519_pushoff/
```

## 5. Diagnosi origine chattering

Report sorgente:

```text
reports/user/2026-05-19_diagnosi_chattering_motor_vs_cascade.md
```

Decomposizione HPF50 lungo la catena di controllo
(`/tmp/chattering_decomposition.py`) sul push-off window:

```text
Segnale                 morning   opt_b    opt_c
cascade qdot_ref         0.127    0.203    0.148   <- input pulito
joint_qdot (feedback)    1.011    1.693    1.146   <- FEEDBACK rumoroso
cascade velocity_err     1.013    1.700    1.151   <- eredita rumore qdot
cascade inner_p_cmd      2.865    4.808    3.254   <- Kp_v amplifica
tau_input_plugin        15.96    16.42    15.20    <- output PI motor
```

Loop di chattering identificato:

```text
joint_qdot rumoroso -> velocity_err -> inner_p_cmd -> tau_ref -> tau_input -> tau_motor -> sostiene joint
```

Verifica delle 4 cause candidate
(`/tmp/chattering_causes_investigation.py`):

- **C1 GRF/contatto**: parzialmente vera. Chattering ankle massimo durante
  **heel strike** (0.56 rad/s), NON durante push-off (0.055).
- **C2 J_ankle bassa**: smentita. Rapporto qddot ankle/knee misurato 4.3x
  vs atteso 16-49x.
- **C3 Risonanza spring-rotor**: confermata per il **knee** (picco netto a
  28 Hz = omega_mech_knee), smentita per l'ankle (no picco a 36 Hz).
- **C4 Rumore numerico**: confermata per ankle (HPF50 in swing 0.075,
  push-off 0.055 = rumore di fondo costante), smentita per knee.

Verdetto sulle cause:

- Ankle: heel-strike shock + rumore numerico costante
- Knee: risonanza meccanica spring-rotor a 28 Hz

In entrambi i casi, il problema e' il **velocity loop del cascade che
amplifica il rumore HF del joint senza filtrarlo**.

## 6. Opzione D - LPF sul feedback qdot (soluzione operativa)

Report sorgente:

```text
reports/user/2026-05-19_optD_lpf_qdot_feedback.md
```

Filtro del primo ordine sul feedback `qdot_cur` letto dallo stato OpenSim,
prima del calcolo del velocity_error. Cutoff 25 Hz su entrambi i SEA.

### Implementazione

File modificati:

```text
config.py
  + sea_qdot_feedback_lpf_cutoff_hz: Dict[str, float] (default 0.0)
prosthesis_controller.py
  + _apply_qdot_lpf(coord_name, qdot_raw, dt_control) method
  + stato persistente _qdot_filtered, _qdot_filtered_initialised
  + qdot_cur filtrato in compute() prima di e_qdot / cascade_velocity_error
  + esposizione qdot_feedback_raw/filtered/lpf_alpha
  + reset() include reset filtro qdot
main.py
  + --sea-qdot-lpf-knee  CLI flag
  + --sea-qdot-lpf-ankle CLI flag
output.py
  + SEA_DIAGNOSTIC_WIDTH: 42 -> 45
  + qdot_feedback_raw/filtered/lpf_alpha in sea_diagnostics.sto
```

### Smoke test

- Test algoritmico isolato: 80 Hz attenuato 0.283x (atteso teorico 0.298),
  5 Hz preservato a 98%.
- Test end-to-end sim 0.236 s: alpha = 0.1358 (atteso 0.1357 per fc=25Hz,
  dt=1ms). Colonne diagnostiche presenti. Status complete.

### Full simulation

```bash
main.py --t-start 11.99 --t-end 21.0 \
  --sea-qdot-lpf-knee 25.0 --sea-qdot-lpf-ankle 25.0 \
  --output-dir results/_optD_full_20260519
```

```text
status=complete, sat=0/0, 9010 step, wall 772 s
```

### Risultati full window 11.99-21.0 s

| metric | morning | optD | delta% |
|---|---:|---:|---:|
| ankle_rmse_deg | 1.220 | 1.236 | **+1.3%** |
| knee_rmse_deg | 0.178 | 0.296 | +66% |
| max_u_knee | 0.332 | 0.425 | +28% |
| max_u_ankle | 0.482 | 0.489 | +1.4% |
| sat_knee / sat_ankle | 0/0 | 0/0 | invariato |
| ankle velocity_err HPF50 | 0.160 | 0.059 | **-63%** |
| knee velocity_err HPF50 | 0.016 | 0.004 | **-75%** |
| ankle tau_input HPF50 | 2.68 | 1.26 | **-53%** |
| knee tau_input HPF50 | 4.99 | 1.00 | **-80%** |
| ankle mdot HPF50 | 318 | 46 | **-86%** |
| knee mdot HPF50 | 535 | 105 | **-80%** |

Confermato su entrambi i cicli del piede destro (13.16-14.78 e
17.95-19.53): chattering ridotto 53-88%, ankle tracking invariato,
knee tracking peggiora di 0.1-0.15 deg in assoluto.

### Plot

```text
plot/05_19_2026_2/
  01_time_sea_control_reserve.png
  02_time_joint_motor_states.png
  03_gaitcycle_torque_angle_power.png
  04_gaitcycle_joint_velocity_power.png
  05_time_tau_input_tracking_error.png
  06_time_joint_ref_sea_error.png
```

`No missing channels`.

## Artefatti principali della giornata

Risultati:

```text
results/_effective_joint_inertia_20260519/
results/_cascade_qdot_step_20260519/
results/_cascade_jeff_todo_20260519_screen_morning_pushoff1/
results/_cascade_jeff_todo_20260519_screen_pushoff1/
results/_cascade_jeff_todo_20260519_screen_p01_pushoff1/
results/_cascade_optB_20260519_pushoff/
results/_cascade_optC_20260519_pushoff/
results/_optD_smoke_20260519/
results/_optD_full_20260519/
```

Plot:

```text
plot/05_19_2026_2/  (Opzione D full run)
```

Modelli creati:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_jeff_v3cap_p01.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optB_ankle_wn500.osim
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optC_ankle_kd15.osim
```

Tool e validatori:

```text
validation/effective_joint_inertia.py
validation/cascade_qdot_step_inertia.py
validation/cascade_jeff_todo_runner.py (esteso con opt_b, opt_c, morning_best_run)
/tmp/chattering_decomposition.py        (throwaway, riproducibile)
/tmp/chattering_causes_investigation.py (throwaway, riproducibile)
/tmp/optD_analysis.py                   (throwaway, riproducibile)
```

Codice principale modificato:

```text
config.py
prosthesis_controller.py
main.py
output.py
```

## Stato finale operativo

- Configurazione attiva: **morning best 18/05** invariata.
- LPF qdot: implementato, smoke + full validati, **disabilitato di default**.
  Per attivarlo: CLI flag `--sea-qdot-lpf-knee 25 --sea-qdot-lpf-ankle 25`
  oppure modificare default in `config.py`.
- Setup ricordato dal simulatore: `AB06_SEASEA_stiff321_500_pi_setup.xml`
  (cambiato durante la giornata per i test diagnostici).
- Modello attivo: `AB06_SEASEA_stiff321_500_pi.osim` (motor driver morning
  best 18/11/190 knee, 11.3/11/123 ankle, invariato).

## TODO aperti e propagati

### Propagati dal daily 2026-05-18

- **Windows pendente**: build/copia DLL plugin PI su Windows, verifica
  caricamento, smoke load/run, controllo lettura di `Ki`,
  `integral_torque_limit`, `torque_error_integral`.
- **Secondo pass knee dello sweep locale** (125 run, 0 accettabili):
  rilassare gate, cambiare griglia knee, o filtro GRF omogeneo.
- **Confronto consolidato finale**: PD, PI, cascade aggressive, best
  mattutino, retune PI originale, zeta07, retune pi-tuned, varianti
  J_eff, Opzione D. Tracking, HPF chatter, tau_input, motor power,
  reserve.
- **Zeta07 non operativo se ripreso**: ankle Kp_outer 30-40, omega_v
  90-100, separazione motor/velocity >= 3x.
- **Knee nel design zeta07**: Kd_plugin piu' alto o ritorno
  omega_motor_knee vicino a 500.
- **Regola 5x**: testare gerarchia 3x come compromesso (oggi confermato
  che 5x e' troppo conservativo per ankle).
- **Zero/polo PI ankle**: miss 22% verificato non dominante oggi
  (V3' p/on=0.1 vs V3' standard non recupera tracking ankle).
- **Alternativa quieta pi-tuned**: `full_s3_kfast_a75_v18_i130` per
  riduzione massima chattering.
- **Pulizia artefatti**: archiviazione cartelle sweep
  `_cascade_local_gain_sweep_20260517_233607` e `_234151`.

### Nuovi TODO da oggi

- **Decidere se promuovere LPF qdot a default** in `config.py`. Pro:
  chattering -53/-86%. Contro: knee RMSE +0.12 deg (+66% relativo).
- **Testare LPF asimmetrico** (cutoff su ankle, knee disabilitato) per
  evitare il costo knee tracking, mantenendo i benefici ankle.
- **Esplorare cutoff alternativi** (30, 35 Hz) per trovare il miglior
  trade-off tracking/chattering.
- **Notch a 28 Hz sul feedback knee** come alternativa piu' chirurgica
  al LPF (preserva la banda < 25 Hz utile, taglia solo la risonanza
  spring-rotor).
- **Validare LPF su run lunga** (30+ s) per stabilita' in regime
  continuo e assenza di derive.
- **Capire perche' knee RMSE peggiora con LPF**: probabile fase-lag
  ridotto smorzamento effettivo se la dinamica knee reale e' vicina a
  J_pair (zeta_actual = 0.59 sotto-smorzato). Test con J_eff scelto
  esplicitamente in cascade.
- **Cleanup modelli sperimentali** opt_b, opt_c, jeff_v3cap*: tenere
  per riferimento o rimuovere dopo conferma del fallimento del tuning
  motor driver.
- **Validazione J_eff dopo aggiornamenti modello**: ripetere solo se
  cambiano geometria, masse, inerzie o struttura fisica OpenSim.

## TODO chiusi oggi

- **Validare J_joint usato nel back-calc cascade**: chiuso da
  `validation/effective_joint_inertia.py`. Risultato: il back-calc
  sovrastimava ~2x l'inerzia libera; ankle robusto, knee dipendente
  dal vincolo.
- **TODO redesign cascade con J_eff corretta**: chiuso teoricamente
  (documento root) e validato sperimentalmente. Esito negativo
  operativo: il design teorico V3/V3' fallisce il tracking ankle
  nonostante riduca il chattering.
- **Verificare 4 cause candidate del chattering ankle**: chiuso da
  `/tmp/chattering_causes_investigation.py`. Diagnosi: heel-strike
  shock + rumore numerico per ankle, risonanza spring-rotor 28 Hz
  per knee.
- **Trovare soluzione operativa al chattering senza perdere
  tracking**: chiuso da Opzione D. Riduzione 53-86% del chattering
  con ankle tracking invariato; knee tracking peggiora ma resta
  basso in assoluto.
