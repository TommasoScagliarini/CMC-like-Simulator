# 2026-05-20 — Diagnosi HPF50: motor driver vs cascade outer e coupling meccanico knee→ankle

## Problema

Dalla giornata del 19/05 era rimasta aperta la domanda di Tommy:

> Perché il chattering sulla joint velocity (e su `tau_input`) e' aumentato
> da quando e' stato integrato il cascade outer? Nei grafici del 14/05 con
> baseline PD non c'era chattering, solo problema di tracking.

L'Opzione D del 19/05 (LPF su `qdot_cur` a 25 Hz) riduceva il chattering
del 53–86% ma peggiorava il knee RMSE del 66% (+0.12 deg). Tommy ha
guardato i plot e ha rigettato il trade-off: **baseline operativa resta
morning best 18/05** (`AB06_SEASEA_stiff321_500_pi.osim`), LPF qdot
implementato ma disabilitato di default.

L'ipotesi di lavoro era che **il cascade outer P-pos → PI-vel** stesse
amplificando il rumore HF su `qdot` letto dallo stato OpenSim. La
diagnosi del 19/05 aveva infatti decomposto la catena:

```text
joint_qdot rumoroso -> velocity_err -> inner_p_cmd -> tau_ref -> tau_input
```

Tommy ha chiesto la diagnosi HPF50 confrontando architetture
strutturalmente diverse senza tornare a simulare il modello 14/05
("lo stesso discorso puo' essere fatto col morning best").

## Soluzione

Eseguita una diagnosi HPF50 (cutoff 50 Hz, Butterworth ord. 4, filtfilt,
fs=1 kHz) su tre run gia' disponibili e due nuove run create oggi.

**Falsificata l'ipotesi che il cascade amplifichi il rumore HF rispetto
al PID single-loop.**

Il colpevole del chattering knee e' **il motor driver veloce**
(Kp=18/Kd=11/Ki=190), che eccita la risonanza spring-rotor a 28 Hz gia'
identificata il 19/05. Sostituendo il motor driver col PD lento del 14/05
(Kp=3.9/Kd=9.7/Ki=0), il chattering knee crolla del 70% **senza degradare
il tracking**.

Sull'ankle pero' la stessa sostituzione **peggiora** il chattering (+53%).
Inoltre, tentando un setup asimmetrico (knee 14/05 + ankle morning best),
l'ankle peggiora ancora di piu' (+71% vs morning best) nonostante il suo
controllore sia identico: c'e' un **coupling meccanico knee→ankle** che
inietta rumore HF nell'ankle quando il motor driver knee e' meno
reattivo.

## Strategia

### Decomposizione in cinque configurazioni

```text
A  outer PID single-loop + motor driver PD veloce  (Kp=18, Kd=11)
B  outer PID single-loop + motor driver PI         (Kp=18, Kd=11, Ki=190)
C  outer cascade P-pos+PI-vel + motor driver PI    (morning best 18/05)
D  outer cascade (morning best) + motor driver PD lento entrambi
   (knee Kp=3.9/Kd=9.7, ankle Kp=8.8/Kd=9.7, Ki=0)
E  outer cascade (morning best) + motor driver ASIMMETRICO
   (knee Kp=3.9/Kd=9.7/Ki=0, ankle Kp=11.3/Kd=11/Ki=123)
```

A, B, C erano gia' presenti in `results/`. D ed E sono state lanciate oggi.

### Falsificazione del cascade come sorgente del chattering

Confronto A vs C (knee):

| metrica | A: PID single | B: PID single + PI inner | C: cascade morning best |
|---|---:|---:|---:|
| joint_qdot HPF50 [rad/s] | 0.0206 | 0.0213 | **0.0157** |
| tau_ref HPF50 [Nm] | 0.619 | 0.638 | **0.471** |
| tau_input HPF50 [Nm] | 5.30 | 5.48 | **4.82** |
| motor_speed HPF50 | 0.71 | 0.74 | **0.59** |
| outer_d_cmd HPF50 [Nm] | 0.619 | 0.638 | n/a |

Confronto A vs C (ankle):

| metrica | A: PID single | B: PID single + PI inner | C: cascade morning best |
|---|---:|---:|---:|
| joint_qdot HPF50 [rad/s] | 0.273 | 0.282 | **0.150** |
| tau_ref HPF50 [Nm] | 0.775 | 0.801 | **0.435** |
| tau_input HPF50 [Nm] | 2.99 | 3.11 | **2.71** |
| motor_speed HPF50 | 0.38 | 0.40 | **0.34** |
| outer_d_cmd HPF50 [Nm] | 0.546 | 0.564 | n/a |

Verifiche di consistenza algebrica (gain di trasmissione `qdot_HF → tau_ref`):

```text
PID single-loop:  outer_d_cmd_HPF = Kd_outer * qdot_HPF
  knee  : 26 * 0.0206 = 0.54  ~ misurato 0.619  OK
  ankle :  2 * 0.273  = 0.55  ~ misurato 0.546  OK

Cascade:          cascade_inner_p_cmd_HPF = Kp_inner * vel_err_HPF
  knee  : 29.2 * 0.0157 = 0.458  ~ misurato 0.458  OK
  ankle : 2.83 * 0.150  = 0.424  ~ misurato 0.422  OK
```

**Conclusione 1:** il cascade NON amplifica piu' rumore HF del PID
single-loop. Anzi, su `tau_input` e' del 9–10% piu' pulito sia su knee
sia su ankle.

### Isolamento del motor driver come sorgente del chattering knee

Run D — cascade morning best invariato + motor driver Kp=3.9/Kd=9.7/Ki=0
su entrambe le coordinate:

| metrica | C: morning best | D: slow inner | Δ% |
|---|---:|---:|---:|
| **Knee** | | | |
| tau_input HPF50 [Nm] | 4.82 | **1.45** | **−70%** |
| motor_speed HPF50 | 0.59 | 0.24 | −60% |
| joint_qdot HPF50 [rad/s] | 0.0157 | 0.0166 | +6% |
| knee RMSE [deg] | 0.178 | 0.181 | +2% |
| **Ankle** | | | |
| tau_input HPF50 [Nm] | 2.71 | **4.14** | **+53%** |
| motor_speed HPF50 | 0.34 | 0.56 | +66% |
| joint_qdot HPF50 [rad/s] | 0.150 | 0.257 | +71% |
| ankle RMSE [deg] | 1.220 | 1.216 | −0.4% |

**Conclusione 2 (knee):** col motor driver "14/05", knee chattering −70%
e tracking essenzialmente identico. Coerente con la diagnosi del 19/05:
la risonanza spring-rotor knee a 28 Hz era eccitata dal motor driver
veloce (omega_n_motor knee 397 → 785 rad/s, ovvero 63 Hz → 125 Hz).
Riportando omega_n_motor a 63 Hz la risonanza non e' piu' eccitata
direttamente.

**Conclusione 3 (ankle):** col motor driver "14/05", ankle chattering
+53%. Sull'ankle il problema NON era la banda inner (era gia' a 111 Hz
del 14/05 vs 125 Hz attuale, variazione marginale). Il problema era e
resta heel-strike shock + rumore numerico (diagnosi 19/05). Senza Ki
inner e con Kd 11 → 9.7, il cascade ankle vede un plant inner meno
reattivo, accumula piu' velocity error, e si autoalimenta.

### Tentativo asimmetrico

Run E — knee con motor driver 14/05, ankle con motor driver morning best:

```text
SEA_Knee:  Kp=3.9,  Kd=9.7, Ki=0
SEA_Ankle: Kp=11.3, Kd=11,  Ki=123
```

Atteso: prendere il meglio di entrambi (knee da D, ankle da C).
Osservato: knee migliora come D, ma **l'ankle peggiora ancora di piu' di
D** nonostante il suo controllore sia identico a C.

| metrica | C | D | **E asym** |
|---|---:|---:|---:|
| knee tau_input HPF50 | 4.82 | 1.45 | **1.38** |
| ankle tau_input HPF50 | 2.71 | 4.14 | **4.64** |
| ankle joint_qdot HPF50 | 0.150 | 0.257 | **0.233** |
| somma tau_input HPF50 | 7.53 | 5.59 | 6.02 |
| knee RMSE | 0.178 | 0.181 | 0.182 |
| ankle RMSE | 1.220 | 1.216 | 1.222 |

L'ankle ha **lo stesso** controllore in C ed E. L'unica differenza tra C
ed E e' il motor driver knee. L'aumento di `joint_qdot HPF50` ankle da
0.150 a 0.233 deve quindi venire da un coupling meccanico knee→ankle:
quando il motor driver knee insegue meno bene `tau_ref`, la cinematica
knee oscilla diversamente, il piede si muove diversamente, i contatti
GRF cambiano, e l'ankle ne risente.

**Conclusione 4:** il free-lunch asimmetrico NON esiste. Il
disaccoppiamento knee/ankle al livello del motor driver non basta:
serve un'azione anche piu' a valle (sul cascade outer ankle, sul
filtraggio del feedback ankle, o sulla dinamica del contatto).

## File modificati o creati

### Modelli

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_slow_inner_pd_1405.osim
  SEA_Knee  Kp=3.9, Kd=9.7, Ki=0
  SEA_Ankle Kp=8.8, Kd=9.7, Ki=0
  (resto invariato vs morning best)

models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_asym_knee1405.osim
  SEA_Knee  Kp=3.9,  Kd=9.7, Ki=0       (motor driver 14/05)
  SEA_Ankle Kp=11.3, Kd=11,  Ki=123     (morning best 18/05)
```

### Codice

Nessuna modifica al codice Python o al plugin C++. Solo modifiche `.osim`
single-property tramite script Python regex (mantengono tabs e
indentazione originale).

### Script di analisi

```text
/tmp/hpf_diagnosis_pd_vs_cascade.py
  HPF50 RMS (Butterworth ord.4, fs=1000 Hz, filtfilt) per:
  joint_qdot, tau_ref, tau_input, motor_speed,
  outer_d_cmd, cascade_velocity_error, cascade_inner_p_cmd.

/tmp/tracking_metrics_runs.py
  RMSE knee/ankle vs IK_results_AB06_SEASEA.mot.
```

Entrambi throwaway, riproducibili, con elenco esplicito delle 5 run.

### Risultati

```text
results/_slow_inner_pd1405_cascade_morning_20260520/        (run D)
results/_asym_knee1405_ankle_morning_20260520/              (run E)
```

Ognuna 9010 step, 12.6 min wall, status complete.

## Test e verifiche

- Plugin C++ con Ki=0: ispezione di
  `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp` ai punti
  rilevanti (linee 123, 132, 224, 227). Con `|Ki| < 1e-12` viene saltato
  il computeStateVariableDerivatives dell'integrale e `tau_i = Ki*xi`
  e' identicamente zero. Equivalente a PD puro motor driver.

- Consistenza algebrica del cascade: `cascade_inner_p_cmd_HPF =
  Kp_inner_cascade * cascade_velocity_error_HPF` verificato a meno di
  1% di errore sulle 4 stime knee/ankle in C/D/E.

- Consistenza algebrica del PID single-loop: `outer_d_cmd_HPF ≈
  tau_ref_HPF` verificato per A e B (la derivata e' la sorgente
  dominante del rumore HF su `tau_ref`).

- Tracking ankle invariato tra C, D, E entro 0.006 deg RMSE
  (1.220/1.216/1.222), quindi l'aumento del chattering ankle in D/E NON
  e' un artefatto di un cambio di traiettoria del joint.

- Tracking knee invariato tra C, D, E entro 0.004 deg RMSE
  (0.178/0.181/0.182), quindi la riduzione del chattering knee in D/E
  NON viene a costo del tracking.

- Wall time confrontabile per le 5 run: 758–772 s.

## Verdetto e stato finale

Configurazione attiva a fine giornata: **morning best 18/05 invariata**
(`AB06_SEASEA_stiff321_500_pi.osim`, modello attivo nel setup).

Mappa causa→soluzione del chattering:

```text
KNEE
  causa     : risonanza spring-rotor a 28 Hz eccitata da omega_n_motor
              alto (Kp=18, omega_n=125 Hz)
  leva      : ridurre Kp_motor_knee
  costo     : nessun costo sul tracking knee
  costo     : aumento chattering ankle via coupling meccanico (+71% in E)
              -> il free-lunch asimmetrico non esiste

ANKLE
  causa     : heel-strike shock + rumore numerico
  leva      : NON e' il motor driver. Banda inner 14/05 vs morning best
              quasi identica (111 vs 125 Hz). Ridurre Kp_motor_ankle
              peggiora perche' toglie autorita' al velocity loop.
  candidati : LPF chirurgico sul feedback ankle (Notch 28 Hz no, perche'
              la risonanza era del knee), abbassare Kp_outer_ankle
              (47.125 e' aggressivo), filtraggio GRF piu' aggressivo.
```

## TODO aperti e propagati

### Nuovi da oggi

- **Decidere se promuovere D** (slow inner su entrambi) o **rimanere a C**.
  D ha somma chattering −26%, tracking essenzialmente identico, ma il
  chattering ankle in valore assoluto sale a 4.14 Nm RMS HPF50 (puo'
  essere visibile sui plot, da verificare).

- **Sweep Kp_knee_motor su 2–3 valori intermedi** (es. 8, 12) tra
  3.9 (lentissimo) e 18 (morning best), mantenendo motor driver ankle a
  morning best, per cercare il punto di flesso del coupling knee→ankle.
  Costo: 30–40 min. Obiettivo: capire se esiste un Kp_knee che riduce
  la risonanza 28 Hz senza svegliare l'ankle.

- **Generare plot D vs C** per validare visivamente le metriche HPF50
  prima di un'eventuale promozione.

- **Validare la diagnosi del coupling knee→ankle** isolando la dinamica
  knee dal feedback ankle (es. fissare temporaneamente la cinematica
  knee e rilanciare solo l'ankle, oppure aumentare la massa del piede
  per smorzare il coupling). Diagnostico, non operativo.

### Propagati dal 19/05 (chiusi o aggiornati)

- **Decidere se promuovere LPF qdot a default**: **chiuso negativamente
  oggi**. Tommy ha rigettato dopo aver visto i plot.
- **Testare LPF asimmetrico**: superato da D ed E (motor driver
  asimmetrico, stessa finalita').
- **Esplorare cutoff alternativi LPF**: superato.
- **Notch a 28 Hz sul feedback knee**: ancora aperto. La causa knee e'
  ora chiara (risonanza spring-rotor) e D la risolve abbassando
  omega_n_motor, ma il Notch resta un'opzione meno invasiva sul motor
  driver.
- **Validare LPF su run lunga (30+ s)**: rimandato (LPF non promosso).
- **Capire perche' knee RMSE peggiora con LPF**: non piu' rilevante
  oggi.
- **Cleanup modelli sperimentali**: aggiungere `slow_inner_pd_1405` e
  `pi_asym_knee1405` ai modelli da decidere se tenere o archiviare.

### Propagati dal 18/05 (ancora aperti)

- Windows: build/copia DLL plugin PI, smoke load/run, verifica `Ki`,
  `integral_torque_limit`, `torque_error_integral`.
- Secondo pass knee dello sweep locale (125 run, 0 accettabili).
- Confronto consolidato finale di tutte le configurazioni storiche
  (PD, PI, cascade, retune PI, zeta07, pi-tuned, J_eff, opt D, slow
  inner, asym).
- Pulizia artefatti sweep `_cascade_local_gain_sweep_20260517_233607`
  e `_234151`.
