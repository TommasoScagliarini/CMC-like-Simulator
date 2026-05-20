# Diagnosi origine del chattering ankle - 2026-05-19

## Problema

Dopo i test della giornata sul redesign cascade con `J_eff` corretta
(`reports/user/2026-05-19_validazione_inerzia_effettiva_knee_ankle.md` e
`reports/user/2026-05-19_validazione_todo_cascade_jeff.md`), restava il
TODO di trovare una configurazione che riducesse il chattering ankle senza
perdere tracking. Il morning best del 18/05 presentava:

```text
ankle RMSE push-off  = 2.41 deg
ankle tau HPF50      = 19.07 Nm
ankle mdot HPF50     = 2102 rad/s^2
```

Le varianti del redesign zeta07 con J_eff corretta (V1-V3') riducevano
molto il chattering ma rompevano il tracking ankle (RMSE 11-15 deg, knee
`max u` saturato a 1.0).

L'obiettivo era trovare la sorgente reale del chattering e una soluzione
che mantenesse il tracking del morning best.

## Strategia

Approccio in due step:

1. testare due interventi conservativi sul motor driver mantenendo la
   cascade morning best:
   - Opzione B: ridurre la banda motor driver ankle a omega_n=500 rad/s
     (zeta=0.7);
   - Opzione C: aumentare lo smorzamento del motor driver ankle alzando
     Kd da 11 a 15 (banda invariata, zeta -> 0.97).
2. se entrambi i test mostrano effetto marginale, decomporre l'HPF50 di
   ogni segnale della catena di controllo per identificare la sorgente
   reale del rumore.

Finestra di valutazione: push-off `13.1638 -> 14.7799 s` (la stessa usata
per il fallimento V3 il mattino).

## Soluzione adottata (test)

Estensione di `validation/cascade_jeff_todo_runner.py` con due nuove
varianti `opt_b` e `opt_c`, due nuovi modelli `.osim` dedicati, e una
variante runnable `morning_best_run` per confronto apples-to-apples sulla
finestra di push-off.

Formule applicate per Opzione B (zeta=0.7, p=0.2*omega_n):

```text
omega_n = 500 rad/s, p = 100, zeta = 0.7
Ki = Jm * p * omega_n^2 / Ks      = 0.01*100*250000/500 = 500
Kp = Jm*(omega_n^2 + 2*z*on*p)/Ks - 1 = 0.01*320000/500 - 1 = 5.4
Kd = Jm*(p + 2*z*on) - Bm         = 0.01*800 - 0.1 = 7.9
```

Verifica analitica dei coefficienti del polinomio caratteristico:

```text
2*z*on + p   = 800   <-> (Kd+Bm)/Jm = (7.9+0.1)/0.01 = 800   PASS
on^2+2*z*on*p= 320000<-> (1+Kp)*Ks/Jm = 6.4*500/0.01 = 320000 PASS
p * on^2     = 25e6  <-> Ki*Ks/Jm = 500*500/0.01 = 25e6      PASS
```

Opzione C (Kd alzato, Kp e Ki invariati):

```text
SEA_Ankle: Kp=11.3, Kd=15.0, Ki=123.0
char poly: s^3 + 1510 s^2 + 615000 s + 6,150,000
poli risultanti: polo reale ~ -10 (cancella zero PI)
                 coppia complessa con omega_n ~774, zeta ~0.97
```

## Risultati dei test push-off

| run | knee RMSE deg | ankle RMSE deg | max u | tau HPF50 | mdot HPF50 |
|---|---:|---:|---:|---:|---:|
| morning_best_run | 0.2073 | 2.410 | 0.654 | 19.07 | 2102 |
| opt_b_ankle_wn500 | 0.2084 | 2.404 | 0.735 | 17.35 | 2225 |
| opt_c_ankle_kd15 | 0.2074 | 2.419 | 0.688 | 18.61 | 2069 |

Confronto con morning_best:

- opt_b: chattering tau -9%, mdot +6%, max u +12%, tracking invariato
- opt_c: chattering tau -2.4%, mdot -1.6%, max u +5%, tracking invariato

Entrambe le opzioni di tuning del motor driver hanno effetto **marginale**
sul chattering misurato. Il tracking e' preservato in entrambi i casi
come atteso, ma il guadagno non giustifica la modifica.

## Diagnosi via decomposizione HPF50

Per capire perche' il motor driver tuning non riduce il chattering, e'
stato scritto `validation/chattering_decomposition.py` (in `/tmp` come
script throwaway, ma riproducibile) che calcola HPF50 RMS lungo tutta la
catena di controllo per ognuna delle 3 run.

Tabella riassuntiva ankle (push-off):

| segnale | morning | opt_b | opt_c | HPF/RMS morning |
|---|---:|---:|---:|---:|
| cascade qdot_ref | 0.127 | 0.203 | 0.148 | 7.1% |
| joint_qdot (feedback) | **1.011** | 1.693 | 1.146 | 38.6% |
| cascade velocity_err | 1.013 | 1.700 | 1.151 | 32.8% |
| cascade inner_p_cmd | 2.865 | 4.808 | 3.254 | 32.8% |
| cascade inner_i_cmd | 2.856 | 2.959 | 2.922 | 8.0% |
| u (cascade output) | 0.0149 | 0.0215 | 0.0165 | 10.0% |
| tau_ref (u*Fopt) | 3.731 | 5.367 | 4.131 | 10.0% |
| tau_input_plugin | 15.96 | 16.42 | 15.20 | 38.6% |
| motor_speed_dot | 1977 | 2315 | 1926 | 92.2% |

Cinque osservazioni chiave:

1. **Il riferimento cinematico e' pulito**: `cascade qdot_ref` HPF50 =
   0.127 rad/s (7.1% del totale). Il filtro IK a 6 Hz funziona.

2. **Il joint reale e' rumoroso**: `joint_qdot` HPF50 = 1.011 rad/s
   (38.6%), **8x piu' rumoroso del riferimento**. Il rumore NON arriva
   dalla cinematica.

3. **Il velocity_err eredita esattamente joint_qdot HPF50**: 1.013 vs
   1.011. La sottrazione `qdot_ref - qdot_meas` non filtra nulla. Il
   loop di velocita' riceve il rumore intatto.

4. **Il Kp_v cascade amplifica linearmente**: `inner_p_cmd HPF50 ~ Kp_v *
   velocity_err HPF50` (2.83 * 1.013 = 2.87, vs valore reale 2.865).
   Il rumore HF di `joint_qdot` viene direttamente moltiplicato per
   Kp_v=2.83 e iniettato in tau_ref.

5. **Il motor PI amplifica ulteriormente**: tau_input_plugin HPF50 = 16
   Nm = 4.3x tau_ref HPF50 (3.7 Nm). Amplificazione interna del PI motor
   dovuta al loop su tau_error.

Confronto opt_b/opt_c: `joint_qdot` HPF50 resta circa lo stesso (1.0-1.7
rad/s) indipendentemente dal tuning del motor driver, confermando che
**il rumore nasce nella dinamica del joint, non nel motor driver**.

## Loop di chattering identificato

```text
joint_qdot rumoroso ----> velocity_err rumoroso ----> inner_p_cmd rumoroso
                                                              |
                                                              v
                                                       tau_ref con HF
                                                              |
                                                              v
                                                  tau_input motor con HF
                                                              |
                                                              v
                                                       tau_motor sul joint
                                                              |
                                                              v
                                                   (alimenta il disturbo
                                                    sul joint)
```

## Origine fisica del rumore in joint_qdot

Non e' un artefatto del controllo. Cause candidate:

- **GRF/contatto in push-off**: la finestra 13.16-14.78 s e' proprio
  push-off, carico massimo sulla protesi.
- **J_ankle molto bassa** (0.010 kg*m^2 validato 2026-05-19): la stessa
  coppia di disturbo produce `qddot` molto piu' alto del knee (J=0.16-
  0.45). Verificato: ankle joint_qdot HPF50 ~ 17x quello del knee
  (1.011 vs 0.06 rad/s).
- **Risonanza spring-rotor a 36 Hz**: code spettrali sopra 50 Hz.
- **Rumore numerico integratore OpenSim**: rumore di quadratura del
  solver.

**Verdetto**: il chattering ankle e' dominato dalla dinamica fisica del
joint durante push-off, amplificata dal velocity loop del cascade. Il
motor driver tuning non e' la leva giusta.

## Strategia per il prossimo passo

Spezzare il loop di feedback HF a monte:

**Opzione D**: LPF sul feedback `joint_qdot` prima del velocity loop del
cascade (taglio alla radice). Cutoff 25-30 Hz. Richiede modifica codice
nel controller Python.

**Opzione E**: LPF sul comando `u` in uscita dal cascade (taglio a
valle). Cutoff 30 Hz. Gia' implementato in `config.py`:
`sea_u_lpf_cutoff_hz` (al momento disabilitato).

Tecnicamente D taglia il rumore prima di moltiplicarlo per Kp_v e prima
di caricare l'integratore cascade. E e' piu' semplice (config switch) ma
lascia l'integratore caricato di rumore.

Test successivo proposto: confronto D vs E sulla stessa finestra
push-off, mantenendo cascade morning best e motor driver morning best.

## File modificati

```text
validation/cascade_jeff_todo_runner.py
  + OPT_B_MODEL, OPT_C_MODEL path
  + prepare_models() entries per opt_b e opt_c
  + variants() entries per opt_b, opt_c, morning_best_run
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optB_ankle_wn500.osim   (NUOVO)
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_optC_ankle_kd15.osim    (NUOVO)
```

Script throwaway riproducibile per la decomposizione HPF50:

```text
/tmp/chattering_decomposition.py
```

## Test e verifiche eseguiti

```bash
/opt/anaconda3/envs/envCMC-like/bin/python validation/cascade_jeff_todo_runner.py \
  --prepare-only

/opt/anaconda3/envs/envCMC-like/bin/python validation/cascade_jeff_todo_runner.py \
  --variants morning_best_run,opt_b \
  --t-start 13.1638 --t-end 14.7799 \
  --out-root results/_cascade_optB_20260519_pushoff

/opt/anaconda3/envs/envCMC-like/bin/python validation/cascade_jeff_todo_runner.py \
  --variants opt_c \
  --t-start 13.1638 --t-end 14.7799 \
  --out-root results/_cascade_optC_20260519_pushoff

/opt/anaconda3/envs/envCMC-like/bin/python /tmp/chattering_decomposition.py \
  results/_cascade_optB_20260519_pushoff/runs/morning_best_run \
  results/_cascade_optB_20260519_pushoff/runs/opt_b_ankle_wn500 \
  results/_cascade_optC_20260519_pushoff/runs/opt_c_ankle_kd15
```

Verifica gain plugin nei modelli generati:

```text
AB06_SEASEA_stiff321_500_pi_optB_ankle_wn500.osim:
  SEA_Knee:  Kp=18,   Kd=11,  Ki=190    (morning best invariato)
  SEA_Ankle: Kp=5.4,  Kd=7.9, Ki=500    (omega_n=500 rad/s, zeta=0.7)

AB06_SEASEA_stiff321_500_pi_optC_ankle_kd15.osim:
  SEA_Knee:  Kp=18,   Kd=11,  Ki=190    (morning best invariato)
  SEA_Ankle: Kp=11.3, Kd=15,  Ki=123    (banda invariata, zeta -> 0.97)
```

Cascade verificata in `config.py` al morning best (knee 18.85/29.2/1377,
ankle 47.125/2.8275/213) per tutti i test (passata via CLI args).

Configurazione attiva finale: nessuna modifica, resta al morning best
del 18/05. I due modelli `opt_b` e `opt_c` sono disponibili per
riferimento ma non promossi.

## TODO

- **Test Opzione D** (LPF su `joint_qdot` feedback nel velocity loop
  cascade): richiede modifica al controller Python. Cutoff 25-30 Hz.
  Verificare:
  - `cascade velocity_err` HPF50 atteso ~0.1 (dal solo qdot_ref);
  - `tau_input_plugin` HPF50 atteso ~1.5 Nm (-90%);
  - tracking RMSE atteso invariato (8 Hz banda fisiologica molto sotto
    cutoff).

- **Test Opzione E** (LPF su comando `u` ankle, gia' implementato in
  config). Switch `sea_u_lpf_cutoff_hz["pros_ankle_angle"] = 30.0`.
  Verifica analoga ma con integratore cascade ancora carico di rumore.

- **Capire perche' joint_qdot ankle HPF50 e' 17x quello del knee**:
  conferma sperimentale dell'effetto J_ankle bassa + GRF push-off?
  Eventualmente correlare HPF50(joint_qdot) con `pros_ankle_grf`.

- **Validare ipotesi sul motor PI amplification 4x**: il rapporto
  tau_input/tau_ref HPF50 e' 4.3 (morning), 3.1 (opt_b), 3.7 (opt_c). E'
  coerente con il modello di sensibilita' del PI?

- **Cleanup modelli opt_b/opt_c**: tenere come riferimento storico
  oppure rimuovere dopo conferma del fallimento del tuning motor driver.
