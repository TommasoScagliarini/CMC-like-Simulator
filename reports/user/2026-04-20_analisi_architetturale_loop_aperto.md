# Analisi architetturale: loop aperto e divergenza con gain alti

**Data**: 2026-04-20

## Problema

Il simulatore CMC-like diverge con gain dell'outer loop `Kp=100`, `Kd=20`,
identici a quelli usati con successo nel CMC nativo di OpenSim sulla stessa
cinematica e con lo stesso modello. L'analisi del segnale `tau_input` del SEA
rivela spike isolati di un singolo timestep che il PD inner amplifica fino a
valori fisicamente irrealistici.

## Evidenza sperimentale

### Spike nel segnale di controllo `u`

Su una run con i parametri attualmente nel modello (`Knee K=1000, Kp=3.9,
Kd=9.7`; `Ankle K=500, Kp=8.8, Kd=8.7`; outer `Kp=20, Kd=2`), l'analisi del
segnale `u_knee` ha identificato il caso peggiore a `t = 5.467 s`:

```
t [s]      u_knee    tau_input [Nm]   tau_spring [Nm]
5.4660    +0.1543       +13.70           +12.97     (regime)
5.4670    -0.1193       -93.56           +12.98     (spike: u salta da +0.15 a -0.12)
5.4680    +0.1581       +97.68            +8.09     (ritorno: PD overshoot)
5.4690    +0.1593       +42.71            +5.69     (ringing)
5.4700    +0.1570       +13.23            +7.25     (assestamento)
```

L'intero range del knee `tau_input` ([-93.6, +97.7] Nm) e determinato da
**un singolo glitch di 1 ms** nel segnale `u`. Senza di esso, `tau_input`
oscillerebbe in una fascia comparabile al `tau_spring` ([-25, +29] Nm).

### Distribuzione degli spike

| Soglia |du/dt|   | Timestep coinvolti | Percentuale |
|----------------------|--------------------:|------------:|
| > 50 /s              | 13 / 6558           | 0.20%       |
| > 100 /s             | 6 / 6558            | 0.09%       |
| > 200 /s             | 2 / 6558            | 0.03%       |

Il 99% del segnale ha `|du/dt| < 5.5 /s`. Il problema non e diffuso ma
concentrato in rari eventi isolati.

### Confronto con CMC nativo OpenSim

Dati dalla stessa cinematica IK, stesso modello, stessi gain `Kp=100, Kd=20`:

| Metrica              | CMC OpenSim          | Nostro simulatore    |
|----------------------|---------------------:|---------------------:|
| Knee tau_input range | [-22, +5] Nm         | [-93.6, +97.7] Nm   |
| Ankle tau_input range| [-45, +10] Nm        | [-90.8, +21.2] Nm   |
| Tracking             | ottimo               | accettabile (5-9 deg)|
| Stabilita Kp=100     | stabile              | **diverge**          |

### Analisi HPF-energy-ratio

Metrica HPF (Butterworth 4th order, zero-phase, cutoff 50 Hz) sui dati
attuali del simulatore:

| Segnale               | HPF noise (50 Hz) |
|----------------------|-------------------:|
| Knee tau_input        | **6.01%**          |
| Knee tau_spring       | 0.04%              |
| Knee tau_ref          | 0.22%              |
| Ankle tau_input       | 0.07%              |
| Ankle tau_spring      | 0.00%              |
| Ankle tau_ref         | 0.00%              |

Il rumore e confinato al `tau_input` del knee. `tau_spring` e `tau_ref` sono
puliti: il problema non e nell'inner PD ne nel controller high-level, ma
nella catena che produce il comando `u`.

## Diagnosi architetturale

### Architettura attuale del simulatore

```
Per ogni timestep dt:
  1. realizeVelocity(state)
  2. q_ref, qdot_ref, qddot_ref = kin.get(t)
  3. qddot_des = qddot_ref + Kp*(q_ref - q) + Kd*(qdot_ref - qdot)
  4. [ID]  tau = M * (qddot_des - qddot_0)          ← zero-actuator baseline
  5. [SO]  minimizza sum(a_i^2) + w*sum(u_res^2)
          s.t.  A_muscle*a + R_res*u_res = tau_bio
  6. Applica controlli, compute udot via bypass
  7. Semi-explicit Euler: qdot += udot*dt; q += qdot*dt
```

### Architettura CMC OpenSim

```
Per ogni finestra di controllo [t, t+T]:
  1. qddot_des = qddot_ref + Kp*(q_ref - q) + Kd*(qdot_ref - qdot)
  2. [ID]  tau_des = stima della coppia necessaria
  3. [Ottimizzazione] trova eccitazioni muscolari che producono tau_des
     valutando la RISPOSTA FORWARD del modello muscolo-tendineo
  4. Integrazione forward dynamics (RK45 con error control)
     L'integratore propaga lo stato con le eccitazioni trovate
  5. Il controller vede il RISULTATO dell'integrazione al passo successivo
```

### Differenze critiche

#### 1. Loop aperto vs loop chiuso sulla dinamica muscolare

Nel nostro simulatore, la catena e:

```
qddot_des  →  ID(tau)  →  SO(a, u_res)  →  apply_controls  →  Euler(q, qdot)
     ↑                                                              |
     └──────────── PD su (q_ref - q) ──────────────────────────────┘
```

Il feedback avviene solo attraverso il PD sull'errore di posizione. Ma tra
il `tau_bio` richiesto dall'ID e il `tau_bio` effettivamente erogato dai
muscoli, **non c'e alcuna verifica**. Se la SO non riesce a produrre
esattamente la coppia richiesta (per limiti muscolari, geometria sfavorevole,
infeasibility del QP), il residuo diventa un errore non compensato che si
accumula.

Nel CMC, l'ottimizzatore vede la risposta forward dynamics e aggiusta le
eccitazioni di conseguenza. Non puo chiedere piu di quanto i muscoli possano
dare perche il vincolo e implicito nella simulazione forward.

#### 2. Rigidita numerica del single-step

L'outer loop ricalcola `qddot_des` a ogni `dt = 0.001 s` senza garanzia di
continuita. Se tra il passo `k` e il passo `k+1` cambia leggermente la
geometria muscolare (perche lo stato e avanzato con Euler), il `qddot_0`
(baseline zero-actuator) puo saltare in modo discontinuo. Con un termine
`M * (qddot_des - qddot_0)`, questo salto in `qddot_0` si traduce
direttamente in un salto in `tau_bio`, che la SO distribuisce ai muscoli,
producendo un'accelerazione effettiva diversa da quella desiderata.

Il CMC usa un integratore a passo variabile (Runge-Kutta-Merson o RK45) con
error control che seleziona automaticamente il timestep per mantenere
l'errore sotto soglia. Il nostro Euler a passo fisso non ha questa proprieta.

#### 3. Accoppiamento biologico-protesico

Nel nostro simulatore, l'ID calcola `tau_full = M * delta_qddot` per TUTTI i
DOF simultaneamente. La matrice di massa `M` accoppia tutti i gradi di
liberta: un errore nell'accelerazione di un DOF biologico si propaga
attraverso `M` a tutti gli altri DOF, inclusi quelli protesici. Se un DOF
biologico accumula errore (perche la SO non ha prodotto la coppia esatta),
l'ID al passo successivo lo compensa con un `tau_full` che contiene
contributi spuri anche sui DOF protesici → il feed-forward `tau_pros_ff`
salta → `u` salta → `tau_input` esplode.

Questo spiega perche gli spike nel `u_knee` sono rari ma violenti: occorrono
quando il residuo SO su qualche DOF biologico raggiunge una soglia critica
che, propagata attraverso M, produce un delta improvviso nel feed-forward
protesico.

## Soluzioni

### Soluzione A: Fix palliativo — rate limiter e filtraggio

Limitare la velocita di variazione di `u` e/o filtrare `qddot_des` con un
LP. Questo sopprime i sintomi senza risolvere la causa.

**Pro**: implementazione immediata, non modifica l'architettura.

**Contro**: mascherare gli spike non corregge l'errore cumulativo sulla
coppia. I gain restano limitati, il tracking non puo migliorare oltre un
certo punto, e il simulatore non puo replicare le prestazioni del CMC.

### Soluzione B: Riscrittura del loop in stile CMC (consigliata)

Riscrivere il loop di simulazione per chiudere il feedback tra la richiesta
di coppia e la risposta muscolare effettiva. L'obiettivo e riprodurre la
catena di controllo del CMC nativo di OpenSim.

#### B.1 — Forward dynamics con integrazione controllata

Sostituire il bypass ID + Euler con un'integrazione forward dynamics vera.
Il passo diventa:

```
Per ogni finestra [t, t+T] (T = 0.01-0.035 s come nel CMC):
  1. qddot_des = qddot_ref + Kp*(q_ref - q) + Kd*(qdot_ref - qdot)
  2. ID: tau_des stima della coppia target
  3. Distribuzione: trova le eccitazioni muscolari che producono tau_des
     attraverso il modello muscolo-tendineo (lookup table o iterazione)
  4. Integrazione forward: applica le eccitazioni, integra con RK45
     o con l'integratore nativo OpenSim su [t, t+T]
  5. Al passo successivo, il PD vede lo stato reale prodotto dalla
     forward dynamics
```

Il punto chiave e che l'integrazione forward dynamics propaga lo stato
**attraverso le equazioni del moto complete**, incluse le forze muscolari
non lineari, la compliance tendinea e le dinamiche SEA. Il risultato e che:

- i muscoli producono automaticamente solo la forza che possono generare
- l'integratore RK45 adatta il passo per mantenere la stabilita
- il PD outer vede lo stato effettivo, non uno stato Euler approssimato

#### B.2 — Prerequisiti e vincoli tecnici

Il principale ostacolo tecnico e che nel nostro setup
`realizeAcceleration()` provoca un crash nativo del plugin SEA. Il bypass
attuale (`compute_udot_bypass`) esiste per questo motivo. Per implementare
la soluzione B1 bisogna risolvere questo crash, oppure integrare la forward
dynamics usando lo stesso bypass ma con un integratore multi-stage
(ad esempio RK4 esplicito con 4 valutazioni per step):

```python
# RK4 esplicito con bypass (sketch)
def rk4_step(state, controls, dt):
    k1 = compute_udot_bypass(state)       # udot a t
    state_mid = advance(state, k1, dt/2)
    k2 = compute_udot_bypass(state_mid)   # udot a t+dt/2
    state_mid2 = advance(state, k2, dt/2)
    k3 = compute_udot_bypass(state_mid2)  # udot a t+dt/2
    state_end = advance(state, k3, dt)
    k4 = compute_udot_bypass(state_end)   # udot a t+dt
    udot = (k1 + 2*k2 + 2*k3 + k4) / 6
    return advance(state, udot, dt)
```

Nota: ogni valutazione di `compute_udot_bypass` richiede
`realizeDynamics` + `InverseDynamicsSolver` + `multiplyByM`, quindi il
costo computazionale e circa 4x per step. Compensato dal fatto che un
passo RK4 con `dt = 0.001` ha errore O(dt^5) vs O(dt) dell'Euler, e
potenzialmente puo usare un `dt` piu grande.

#### B.3 — Feedback sulla coppia effettiva (closed-loop SO)

Un'alternativa meno invasiva della full forward dynamics: dopo la SO,
calcolare la coppia effettivamente prodotta dai muscoli e confrontarla con
`tau_bio` richiesto. Se il residuo supera una soglia, ridurre `qddot_des`
proporzionalmente prima del passo Euler.

```python
tau_delivered = A_muscle @ a_opt + R_res @ (u_res * f_opt_res)
tau_error = tau_bio - tau_delivered
# Se |tau_error| e grande, l'accelerazione effettiva sara diversa
# da qddot_des. Compensare riducendo qddot_des:
qddot_actual = qddot_des - M_inv @ tau_error_full
# Usare qddot_actual per l'Euler step
```

Questo chiude il feedback senza richiedere forward dynamics vera, ma
introduce una dipendenza dalla linearizzazione della risposta muscolare
che potrebbe non essere accurata in configurazioni estreme.

#### B.4 — Piano di implementazione (soluzione B.1)

| Fase | Descrizione | File coinvolti |
|------|-------------|----------------|
| 1 | Implementare `rk4_step_bypass()` in `simulation_runner.py` usando `compute_udot_bypass` come valutatore di accelerazioni | `simulation_runner.py` |
| 2 | Aggiungere una finestra di controllo `T_control` in `config.py`. Il PD outer e la SO vengono ricalcolati solo ogni `T_control`, non ogni `dt` di integrazione | `config.py`, `simulation_runner.py` |
| 3 | Separare il substep di integrazione (che avanza lo stato con RK4) dal control step (che ricalcola eccitazioni) | `simulation_runner.py` |
| 4 | Integrare lo stato SEA motor all'interno del RK4 step, non come substep separato | `simulation_runner.py` |
| 5 | Validare con `Kp=100, Kd=20` e confrontare tracking/tau_input con i risultati del CMC | validazione |

La finestra di controllo `T_control` e un elemento chiave del CMC originale:
il controller produce eccitazioni muscolari che vengono mantenute costanti
per tutta la finestra mentre l'integratore propaga lo stato. Questo e
fondamentalmente diverso dal ricalcolare il controllo a ogni `dt`.

## Conclusione

Il problema non e nel tuning dei gain SEA inner ne nell'outer loop PD. Il
problema e architetturale: il loop di simulazione e un sistema in open loop
tra richiesta di coppia (ID) e erogazione effettiva (SO + muscoli). Con gain
bassi il residuo resta contenuto e il simulatore converge; con gain alti del
CMC nativo (`Kp=100, Kd=20`) il residuo diverge perche il PD amplifica un
errore che il sistema non riesce a correggere.

La soluzione consigliata e la riscrittura del loop con un integratore
multi-stage (RK4) e una finestra di controllo separata dall'integrazione,
replicando l'architettura del CMC originale. Questo e il passo necessario
per poter usare gli stessi gain del CMC e ottenere prestazioni confrontabili.
