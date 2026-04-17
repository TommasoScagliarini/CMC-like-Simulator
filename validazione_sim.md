# Validazione del Simulatore CMC-like: Analisi Critica

## Context

L'utente sospetta che i risultati della simulazione siano "troppo belli per essere veri". Dopo un'analisi approfondita del codice sorgente, dei dati numerici e dei grafici, **il sospetto e' fondato**: il simulatore contiene semplificazioni strutturali che rendono i risultati tautologici in alcuni aspetti chiave, in particolare per la protesi SEA.

---

## Problemi Identificati (per gravita')

### CRITICO 1: La dinamica del motore SEA e' completamente bypassata

**Cosa succede:** In [simulation_runner.py:479-520](simulation_runner.py#L479-L520), il metodo `_update_sea_motor_state` imposta l'angolo motore **algebricamente**:

```python
theta_m_eq = theta_j + tau_req / K
```

Questo garantisce che `tau_spring = K * (theta_m - theta_j) = tau_req` **esattamente**, ogni timestep, con zero ritardo.

**Cosa dovrebbe succedere:** Il plugin C++ SEA implementa una vera dinamica del motore:
```
d(theta_m)/dt = omega_m
d(omega_m)/dt = (tau_input - tau_spring - Bm * omega_m) / Jm
```
Ma `compute_udot_bypass` non chiama mai `realizeAcceleration()` (per evitare un crash del plugin), quindi `computeStateVariableDerivatives()` non viene mai invocato. I parametri `Jm` (inerzia), `Bm` (smorzamento), `Kp`, `Kd` del plugin **non hanno alcun effetto**.

**Evidenza nei grafici:** Plot 05 mostra tracking error `tau_ref - tau_spring` dell'ordine di **1e-6 N.m (caviglia)** e **1e-7 N.m (ginocchio)** — precisione macchina, non risultato fisico.

### CRITICO 2: Lo stato motore e' impostato PRIMA della valutazione dinamica

`_update_sea_motor_state` e' chiamato a step G ([simulation_runner.py:273](simulation_runner.py#L273)), **prima** di `compute_udot_bypass` a step H ([simulation_runner.py:286](simulation_runner.py#L286)). Quando il sistema dinamico viene valutato, la molla SEA **gia' produce la coppia esatta comandata**. Non esiste transitorio, overshoot, ne' ritardo.

### CRITICO 3: La velocita' del motore e' approssimata = velocita' giunto

In [simulation_runner.py:517](simulation_runner.py#L517):
```python
sv.set(ms_idx, omega_j)  # omega_motor = omega_joint
```
In realta' `omega_m = omega_j + d(deflection_molla)/dt`. Questa approssimazione elimina la dinamica di accumulo/rilascio energia della molla. **Questo spiega perche' nel Plot 04 la potenza giunto e la potenza motore sono quasi identiche.**

### CRITICO 4: La saturazione del controllo e' inefficace

Il controller SEA clippa `u` a [-1, +1] in [prosthesis_controller.py:120](prosthesis_controller.py#L120), ma `_update_sea_motor_state` imposta l'angolo motore per produrre `tau_sea_cmd` **indipendentemente** da qualsiasi limite fisico del motore (corrente, coppia massima, velocita' massima). Non c'e' saturazione reale dell'attuatore.

### MODERATO 5: Reserve actuators a zero per costruzione

Plot 01 mostra reserve actuators **piatte a zero** per caviglia e ginocchio protesici. Non e' perche' il controller e' buono — e' perche' il modello e' **circolare**: l'ID calcola la coppia necessaria, e il SEA algebrico la produce esattamente. Le reserve non servono mai.

### MODERATO 6: La cinematica e' quasi-prescritta, non emergente

L'outer loop con Kp=100, Kd=20 ([config.py:106-107](config.py#L106-L107)) + feedforward garantisce che la cinematica simulata segua fedelmente il riferimento IK. Per i giunti protesici, la coppia SEA algebricamente esatta rende il tracking ancora piu' stretto. I risultati cinematici **non sono predizioni** ma essenzialmente una riproduzione dell'input.

---

## Cosa e' Genuinamente Sbagliato vs. Semplificazione Accettabile

### Da correggere o dichiarare esplicitamente:
1. **Non si puo' affermare** che la simulazione valida le prestazioni del controller SEA — il controller e' bypassato
2. **L'errore di tracking di 1e-6 N.m** e' una tautologia, non un risultato
3. **Reserve a zero** non prova che la protesi sia autosufficiente
4. **La potenza motore** non e' significativa con `omega_m = omega_j` hardcoded

### Semplificazioni accettabili (se dichiarate):
1. Semi-implicit Euler con dt=0.01s — standard per simulazioni del cammino
2. PD tracking per i DOF biologici — pratica CMC standard
3. Spline cubiche per il riferimento IK — standard
4. Static optimization con min(sum a_i^2) — formulazione standard

---

## Piano di Verifica Proposto

### Test 1: Dinamica motore di primo ordine (sensitivity analysis)
Sostituire il modello algebrico con un filtro del primo ordine:
```python
theta_m_current = sv.get(ma_idx)
theta_m_eq = theta_j + tau_req / K
tau_motor = Jm / Bm  # costante di tempo motore
alpha = dt / (tau_motor + dt)
theta_m_new = theta_m_current + alpha * (theta_m_eq - theta_m_current)
```
Variare `tau_motor` da 0 (attuale) a valori realistici (5-50 ms). Misurare: tracking error, reserve, deviazione cinematica.

### Test 2: Stress test saturazione coppia
Ridurre `F_opt` dei SEA del 50% e rieseguire. Risultato atteso: `u` satura durante stance, reserve si attivano, cinematica devia. Se la cinematica resta perfetta, il path di saturazione e' rotto.

### Test 3: Verifica indipendenza cinematica
Calcolare RMS(output_kinematics - IK_reference) per i giunti protesici. Se < 0.1 gradi, l'output e' una riproduzione dell'input, non una predizione.

### Test 4: Confronto potenza molla
Calcolare `E_spring = 0.5 * K * (theta_m - theta_j)^2` ad ogni timestep. Con il modello algebrico, l'energia segue esattamente il profilo di coppia. Con dinamica motore reale, l'energia dovrebbe mostrare accumulo/rilascio indipendente durante le transizioni stance/swing.

---

## File Critici da Modificare

| File | Modifica |
|------|----------|
| [simulation_runner.py](simulation_runner.py) | `_update_sea_motor_state`: implementare dinamica motore reale |
| [prosthesis_controller.py](prosthesis_controller.py) | Adattare control law per il ritardo introdotto dalla dinamica |
| [config.py](config.py) | Aggiungere flag `use_algebraic_sea` e parametri motore |
| [output.py](output.py) | Aggiornare calcolo potenza per `omega_m != omega_j` |

---

## Raccomandazione

Il simulatore e' **matematicamente corretto per cio' che fa** (inversione dinamica + ottimizzazione statica per i muscoli biologici), ma il modello SEA e' un **attuatore di coppia ideale**, non una simulazione di Series Elastic Actuator. Per validare il controller protesico servono le dinamiche reali del motore. Per validare solo il reclutamento muscolare biologico, il simulatore attuale puo' essere sufficiente con le dovute dichiarazioni.
