# Verifica inner loop SEA e analisi tau_input - 2026-04-17

## Contesto

Dopo aver ripristinato i parametri inner loop dalla baseline stabile (report
[[2026-04-17_diagnosi_divergenza_sea_risultati_pessimi]]), il tracking dei DOF
protesici e accettabile ma tau_input presenta oscillazioni significative,
in particolare per il knee. Questa analisi verifica la correttezza
dell'implementazione e identifica la causa del rumore.

Parametri attivi nel modello `Adjusted_SEASEA - Copia_tuned.osim`:

```text
SEA_Knee:   K=250  Kp=18.5  Kd=10   F_opt=100  Jm=0.01  Bm=0.1  Impedence=false
SEA_Ankle:  K=500  Kp=35    Kd=20   F_opt=250  Jm=0.01  Bm=0.1  Impedence=false
Outer loop: sea_kp=20  sea_kd=2  (entrambi i giunti)
```

## 1. Verifica correttezza implementazione

### Dinamica SEA (plugin C++)

Le equazioni implementate in `SeriesElasticActuator.cpp`:

```
dtheta_m/dt = omega_m
domega_m/dt = (tau_input - K*(theta_m - theta_joint) - Bm*omega_m) / Jm
```

Forza applicata al giunto (modo non-impedance):

```
F_joint = K * (theta_m - theta_joint)
```

**Verdetto: corretta.** Le derivate sono esposte via `OpenSim_DECLARE_OUTPUT` e
lette dal runner Python ad ogni substep.

### Low-level controller PD (plugin C++)

Con `Impedence=false`:

```
tau_ref   = u * F_opt
tau_input = Kp * (tau_ref - tau_spring) - Kd * omega_m
```

Dove `tau_spring = K*(theta_m - theta_joint)`, clamp a +/-500 Nm.

A regime (domega_m/dt = 0, omega_m = 0):

```
tau_spring_ss = Kp/(Kp + 1) * tau_ref
```

Errore a regime strutturale (PD senza termine integrale):

| Giunto | Kp   | tau_spring_ss / tau_ref | Errore a regime |
|--------|------|------------------------|-----------------|
| Knee   | 18.5 | 94.9%                  | 5.1%            |
| Ankle  | 35   | 97.2%                  | 2.8%            |

Pulsazione naturale e smorzamento del sistema linearizzato:

| Giunto | omega_n (rad/s) | zeta  |
|--------|-----------------|-------|
| Knee   | 698             | 0.723 |
| Ankle  | 1342            | 0.749 |

**Verdetto: corretta.** Nessun bug nell'implementazione. Il PD senza termine
integrale ha un errore a regime strutturale che e trascurabile per Kp >> 1.

### Integrazione Python (simulation_runner.py)

Semi-explicit Euler con substep (5 default, max 80):

```python
omega_next = omega_m + motor_speed_dot * h
theta_next = theta_m + omega_next * h          # usa omega aggiornato
```

Le derivate vengono ricalcolate dal plugin C++ ad ogni substep dopo
`realizeDynamics`. Le accelerazioni del giunto (`udot`) sono tenute costanti
nei substep (scelta progettuale documentata: i controlli biologici sono
ottimizzati per lo stato all'inizio dello step).

**Verdetto: corretta.**

## 2. Analisi risultati con parametri baseline

### Tracking dei DOF protesici

| Giunto | Tracking RMS | Max errore | u range         | u saturo |
|--------|-------------|------------|-----------------|----------|
| Knee   | 2.69 deg    | 5.59 deg   | [-0.25, +0.31]  | 0%       |
| Ankle  | 1.93 deg    | 4.28 deg   | [-0.29, +0.11]  | 0%       |

Il tracking e accettabile. Il controllo u non satura mai: l'outer loop produce
comandi ragionevoli.

### Decomposizione outer loop

| Componente          | Knee RMS (Nm) | Knee range (Nm)   | Ankle RMS (Nm) | Ankle range (Nm)   |
|---------------------|---------------|--------------------|-----------------|---------------------|
| tau_ff (da ID)      | 9.45          | [-23.3, +29.1]     | 29.59           | [-71.0, +25.8]      |
| outer_pd            | 1.33          | [-2.9, +2.8]       | 0.81            | [-3.1, +6.7]        |
| tau_ref (totale)    | 9.62          | [-24.6, +30.8]     | 30.35           | [-72.8, +26.5]      |

Il tau_ref e dominato dal feed-forward (ID). Il termine PD dell'outer loop
contribuisce poco (RMS 1.33 Nm su 9.62 per il knee). **L'outer loop e pulito
e ben dimensionato.**

### Analisi tau_input (inner PD)

Dati post-transiente (t > 4.36 s, 6700 campioni):

| Metrica                            | Knee         | Ankle        |
|------------------------------------|--------------|--------------|
| tau_ref RMS                        | 9.69 Nm      | 30.54 Nm     |
| tau_spring RMS                     | 9.60 Nm      | 29.78 Nm     |
| **tau_input RMS**                  | **15.49 Nm** | 30.55 Nm     |
| Correlazione tau_input vs tau_ref  | **0.628**    | 0.969        |
| **Frazione di rumore (1 - R^2)**  | **60.6%**    | 6.2%         |
| d(tau_input)/dt RMS                | 19023 Nm/s   | 4780 Nm/s    |
| tau_input range                    | [-500, +463] | [-76, +158]  |

Il tau_input del knee contiene **60.6% di rumore** non correlato al comando.
L'ankle e pulito (93.8% segnale utile).

### Decomposizione inner PD

| Termine inner PD        | Knee RMS (Nm) | Ankle RMS (Nm) |
|-------------------------|---------------|-----------------|
| Kp*(tau_ref - tau_spring) | 25.5        | 29.0            |
| -Kd*omega_m               | 25.2        | 9.9             |

Per il knee i due termini hanno **la stessa magnitudine** (25.5 vs 25.2 Nm RMS)
e si cancellano parzialmente. Il residuo oscilla ad alta frequenza. Per l'ankle
il termine derivativo e 3x piu piccolo del proporzionale, quindi il segnale
domina.

### Velocita motore

| Giunto | omega_m RMS (rad/s) | omega_m range (rad/s)  |
|--------|---------------------|------------------------|
| Knee   | 2.52                | [-34.67, +10.63]       |
| Ankle  | 0.49                | [-5.73, +9.94]         |

Il motore del knee sviluppa velocita 5x maggiori dell'ankle. Questo amplifica
il termine Kd*omega_m nell'inner PD.

## 3. Diagnosi: perche il knee oscilla e l'ankle no

### Meccanismo dell'oscillazione

1. Con K basso (250 vs 500), una coppia comandata richiede una **deflessione
   angolare piu grande** della molla: Delta_theta = tau/K. Il motore deve
   muoversi di piu per produrre la stessa coppia.
2. Con Jm = 0.01, il motore risponde quasi istantaneamente alle correzioni PD:
   accelerazione = tau_input/Jm = ordine 10^4 rad/s^2.
3. Il motore sviluppa velocita elevate, il termine Kd*omega_m cresce fino
   a eguagliare il termine proporzionale, e i due si combattono.
4. Il clamp a +/-500 Nm introduce nonlinearita: al primo step tau_input per il
   knee raggiunge -118.9 Nm, in rari casi tocca il clamp (-500 Nm a t=5.467).

### Perche il tracking del giunto resta accettabile

La molla agisce da **filtro passa-basso** sul rumore del motore. Il giunto
vede tau_spring (RMS 9.60, range [-23.6, +29.4]), che e ben correlato al
comando tau_ref. L'oscillazione del tau_input e interna alla dinamica del
motore e viene attenuata dalla molla.

### Analisi del transitorio iniziale

Al primo step (t=4.26, tau_spring=0, omega_m=0):

| Giunto | tau_ref  | tau_input iniziale | Accel motore  |
|--------|----------|--------------------|---------------|
| Knee   | -6.43 Nm | -118.9 Nm          | 11890 rad/s^2 |
| Ankle  | +9.47 Nm | +331.4 Nm          | 33145 rad/s^2 |

Entrambi hanno spike iniziali inevitabili. L'ankle si stabilizza in ~30 step
(0.03 s). Il knee mantiene chattering residuo piu a lungo, visibile come
15 cambi di segno in 50 ms intorno a t=5.0.

## 4. Conclusione

| Componente        | Causa dei tau_input rumorosi? | Evidenza                        |
|-------------------|-----------------------------|---------------------------------|
| **Outer loop**    | No                          | tau_ref pulito, u mai saturo    |
| **Inner loop knee** | **Si**                    | 60% rumore, chattering motore   |
| **Inner loop ankle** | No (post-transiente)     | 94% segnale, spike solo iniziale |

L'implementazione e **corretta** in tutte le sue parti. Il problema e
strutturale: con Jm=0.01 e K=250, il PD inner del knee opera in un regime
dove il guadagno proporzionale e il damping hanno la stessa magnitudine,
generando oscillazione.

## 5. Opzioni di miglioramento

Per ridurre il rumore tau_input del knee **senza modificare la logica del
plugin**:

| Intervento            | Effetto atteso                                  | Trade-off                     |
|-----------------------|-------------------------------------------------|-------------------------------|
| Aumentare K knee (400-500) | Meno deflessione necessaria, meno omega_m   | Modifica proprieta fisiche    |
| Aumentare Kd knee (15-20) | Piu smorzamento, decay piu rapido             | Risposta piu lenta            |
| Diminuire Kp knee (12-15) | Meno amplificazione iniziale                  | Errore regime: 1/(Kp+1) sale |
| Combinazione K=400, Kp=15, Kd=15 | Bilancio tra tracking e stabilita     | Da validare su full run       |

La combinazione suggerita porta il rapporto di smorzamento del knee da
zeta=0.723 a zeta=0.827, riducendo significativamente l'oscillazione.

## Vedi anche

- [[2026-04-17_diagnosi_divergenza_sea_risultati_pessimi]] - diagnosi precedente e matrice parametrica
- [[AGENT]] - equazioni dinamica SEA e architettura plugin
- [[DIVERGENZA_BUGFIXES]] - bug storici su scaling PD e baseline molla
