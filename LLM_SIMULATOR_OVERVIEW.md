# CMC-like Simulator Overview for LLMs

Questo documento e' pensato per essere letto da un LLM. Riassume lo scopo,
il flusso computazionale e la struttura di controllo del simulatore
OpenSim/Python CMC-like con due attuatori SEA protesici.

## Scopo del simulatore

Il simulatore riproduce un cammino muscoloscheletrico OpenSim in stile
CMC-like. L'obiettivo e' far seguire al modello una cinematica di riferimento
`kin_ref`, mantenendo:

- le coordinate biologiche/non protesiche guidate da muscoli e reserve
  actuator residuali;
- le due coordinate protesiche controllate da attuatori Series Elastic
  Actuator (SEA): `pros_knee_angle` e `pros_ankle_angle`;
- il lato protesico comandato da un controller Python high-level che produce
  un comando normalizzato `u in [-1, +1]`;
- la dinamica low-level del motore SEA integrata dal plugin C++ OpenSim.

Il simulatore non e' un CMC OpenSim nativo. E' una pipeline Python che usa
OpenSim come modello dinamico, ma calcola controlli, inverse dynamics,
static optimization e integrazione nel codice del repository.

## Flusso generale

I file principali sono:

- `main.py`: entry point.
- `config.py`: parametri, path, tempi, gain, output.
- `model_loader.py`: carica modello, plugin SEA, GRF, reserve e indicizza
  coordinate, controlli e state variable.
- `kinematics_interpolator.py`: legge `kin_ref` e costruisce spline per
  `q_ref`, `qdot_ref`, `qddot_ref`.
- `simulation_runner.py`: coordina il loop CMC-like e integra lo stato.
- `outer_loop.py`: tracking biologico.
- `inverse_dynamics.py`: calcolo delle coppie richieste.
- `static_optimization.py`: recruitment muscoli/reserve.
- `prosthesis_controller.py`: controller high-level dei due SEA.
- `output.py`: salvataggio di stati, controlli, coppie e diagnostica.

Ad ogni control window, normalmente a `T_control = 0.001 s`, il runner fa:

1. Realizza lo stato a `Velocity`.
2. Legge `q_ref`, `qdot_ref`, `qddot_ref` dal riferimento cinematico.
3. Calcola le accelerazioni desiderate per le coordinate biologiche.
4. Usa inverse dynamics per stimare le coppie biologiche richieste.
5. Risolve una static optimization per distribuire le coppie biologiche tra
   muscoli e reserve.
6. Calcola il comando protesico `u` per knee/ankle SEA.
7. Applica controlli muscolari, reserve e SEA.
8. Integra coordinate e stati interni del SEA con il path validato
   `integration_scheme = "rk4_bypass"` e `sea_forward_mode = "plugin"`.

## Parte biologica / sana

Per "parte sana" in questo repository si intende, in pratica, tutto cio' che
non e' controllato direttamente dai due SEA protesici. Include:

- arto controlaterale sano;
- bacino e tronco;
- coordinate biologiche residue;
- anche biologiche, inclusa l'anca sopra la protesi se presente nel modello.

Le coordinate biologiche seguono il file `kin_ref`. Il tracking non avviene
forzando cinematicamente la traiettoria, ma tramite un loop dinamico:

```text
qddot_des = qddot_ref + Kp*(q_ref - q) + Kd*(qdot_ref - qdot)
```

Queste accelerazioni desiderate entrano nella inverse dynamics. Il risultato
viene passato alla static optimization, che cerca attivazioni muscolari e
reserve controls tali da produrre le coppie biologiche richieste.

La static optimization e' muscle-first:

- i muscoli sono il canale preferito;
- le reserve sono supporto residuale;
- le reserve di coordinate non ben attuabili possono chiudere infeasibilita';
- i DOF protesici sono esclusi dal recruitment biologico principale.

Il simulatore evita `realizeAcceleration()` nel loop validato e usa una
proiezione tramite massa/inverse dynamics per ridurre crash nativi OpenSim e
mantenere il controllo del processo numerico.

## Parte protesica

Le coordinate protesiche sono:

```text
pros_knee_angle
pros_ankle_angle
```

Gli attuatori plugin associati sono normalmente:

```text
SEA_Knee
SEA_Ankle
```

Per queste coordinate il simulatore non usa la static optimization
biologica. La pipeline e':

```text
q_ref/qdot_ref protesici
    -> ProsthesisController Python
    -> comando normalizzato u in [-1, +1]
    -> control vector OpenSim
    -> plugin SeriesElasticActuator C++
    -> tau_spring applicata alla coordinata protesica
```

Il valore `tau_pros_ff` calcolato dalla inverse dynamics puo' essere salvato
come diagnostica/oracolo, ma non e' il comando operativo del SEA nel path
corrente. Il comando operativo viene dal tracking high-level.

## Dinamica low-level SEA

Ogni SEA ha almeno due state variable interne:

```text
theta_m = motor_angle
omega_m = motor_speed
```

La coppia elastica applicata al giunto e':

```text
tau_spring = K * (theta_m - theta_j)
```

dove `theta_j` e' l'angolo della coordinata OpenSim.

Nel path non-impedance corrente con plugin feed-forward/PI, la diagnostica
Python ricostruisce la legge low-level come:

```text
tau_ref = u * F_opt
tau_error = tau_ref - tau_spring
tau_i = clamp(Ki * xi, -integral_torque_limit, +integral_torque_limit)
tau_input_raw = tau_ref + Kp*tau_error + tau_i - Kd*omega_m
tau_input = clamp(tau_input_raw, -500, +500)
```

La dinamica del rotore e':

```text
d(theta_m)/dt = omega_m
d(omega_m)/dt = (tau_input - tau_spring - Bm*omega_m) / Jm
```

Il plugin C++ e' la sorgente della dinamica low-level validata. Non va
modificato senza richiesta esplicita.

## Controller high-level protesico

Il controller protesico e' in `prosthesis_controller.py`. Produce `u` per
ciascuno dei due SEA. Le modalita' supportate sono:

- `pd`: tracking posizione/velocita' con PD.
- `pid`: PD piu' integrale sull'errore di posizione.
- `cascade`: modalita' raccomandata e default corrente.

### Modalita' cascade

La modalita' cascade separa position loop e velocity loop:

```text
e_q = q_ref - q
qdot_cas = qdot_ref + Kp_outer * e_q
e_v = qdot_cas - qdot

tau_p = Kp_inner * e_v
tau_i = Ki_inner * integral(e_v)
tau_cmd = tau_p + tau_i
u_raw = tau_cmd / F_opt
u = clip(u_raw, -1, +1)
```

Il termine integrale interno del cascade ha anti-windup e clamp in coppia
tramite `sea_cascade_inner_i_torque_limit`.

Il comando `u` puo' passare attraverso `sea_u_lpf_cutoff_hz`, un filtro
low-pass opzionale sul comando normalizzato. Nel codice corrente questo filtro
e' disabilitato di default. Non assumere che un filtro sul feedback `qdot`
sia disponibile nel codice attuale: alcuni report storici lo citano, ma la
versione corrente del controller espone il filtro su `u`.

### Relazione tra controller Python e plugin C++

Il controller Python non comanda direttamente una coppia al giunto. Comanda:

```text
u = tau_cmd / F_opt
```

Il plugin interpreta `u` come riferimento normalizzato e costruisce la dinamica
motore+molla. La coppia effettiva sul giunto deriva dalla molla:

```text
tau_out = tau_spring
```

Quindi il tracking protesico dipende sia dal controller high-level Python sia
dalla banda e stabilita' del motor driver SEA nel plugin.

## Interfaccia RL / generazione traiettorie

Il file `osim_trj_cmc_like.py` fornisce un adapter Gymnasium per RL a livello
di traiettoria. In quell'env la policy non comanda muscoli, reserve o coppie.
La policy genera un segmento di traiettoria cinematica per:

```text
pros_knee_angle
pros_ankle_angle
```

Il segmento viene interpolato e sostituisce solo il riferimento protesico.
Tutte le coordinate non protesiche continuano a seguire il `kin_ref` originale.
Il CMC-like simulator poi avanza normalmente tramite `SimulationRunner`.

Questa distinzione e' importante:

- RL decide il riferimento cinematico protesico;
- `ProsthesisController` decide il comando SEA `u`;
- il plugin SEA decide la risposta fisica motore+molla;
- il lato biologico resta governato dal tracking CMC-like di `kin_ref`.

## Invarianti importanti per un LLM

Non assumere che:

- il simulatore forzi cinematicamente l'output a seguire `kin_ref`;
- le reserve siano il canale principale di controllo biologico;
- le coordinate protesiche siano risolte dalla static optimization biologica;
- `tau_pros_ff` sia il comando operativo del SEA;
- il plugin C++ possa essere modificato liberamente;
- il comando `u` sia una coppia fisica diretta al giunto.

Assumere invece che:

- `kin_ref` e' un riferimento, non una traiettoria imposta;
- le coordinate biologiche cercano di seguire `kin_ref` tramite muscoli e
  reserve residuali;
- le coordinate protesiche cercano di seguire il proprio `q_ref` tramite il
  controller SEA high-level;
- il plugin SEA applica la dinamica fisica low-level;
- la compatibilita' Windows x86 e macOS arm64 e' un requisito del progetto.

