# AB06_SEASEA - Analisi tracking, torque-angle e proposta PID

Data: 2026-05-13

## Sintesi

E' stata analizzata la cartella:

```text
plot/13_05_2026 - 3
```

insieme al codice del controllore protesico, del plotter, dell'output recorder
e ai report gia presenti sul bundle `AB06_SEASEA_Threadmill`.

La conclusione principale e' che l'errore cinematico in `plot 6` e' reale e
coerente con l'architettura attuale: l'outer loop protesico e' un PD puro su
posizione e velocita, senza integratore e senza feedforward di inverse dynamics.
Un termine integrale sull'outer loop puo' quindi essere utile, ma va introdotto
con anti-windup e sweep controllato.

Il motor driver interno, invece, non risulta la prima causa del problema nella
run corrente: il tracking corretto da guardare e' `tau_ref - tau_spring`, non
`tau_ref - tau_motor`. In questa simulazione l'errore `tau_ref - tau_spring` e'
piccolo rispetto alle coppie richieste, non ci sono saturazioni del motore SEA e
il comando normalizzato resta lontano da `|u| = 1`.

## Problema

Le criticita discusse erano:

- errore di tracking cinematico non prossimo a zero per ginocchio e caviglia;
- dubbio se l'errore fosse dovuto al controllore o al dataset;
- forme torque-angle protesiche non canoniche in `plot 3`;
- loop torque-angle protesici ancora compressi;
- differenza tra dati protesici e overlay sano in `plot 3` e `plot 4`;
- ipotesi di introdurre un PID sia nell'outer loop sia nel motor driver SEA.

## Contesto tecnico

Setup operativo:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
```

Run analizzata:

```text
t_start = 11.99 s
t_end   = 21.00 s
dt      = 0.001 s
T_control = 0.001 s
sea_forward_mode = plugin
integration_scheme = rk4_bypass
```

Outer loop protesico da `config.py`:

```text
pros_knee_angle:  Kp = 160 N*m/rad, Kd = 12 N*m*s/rad
pros_ankle_angle: Kp = 420 N*m/rad, Kd = 1  N*m*s/rad
```

Il codice in `prosthesis_controller.py` implementa:

```text
e_q    = q_ref - q
e_qdot = qdot_ref - qdot
tau_cmd = Kp * e_q + Kd * e_qdot
u = clip(tau_cmd / F_opt, -1, 1)
```

Non viene usato un termine `tau_ff` da inverse dynamics nel comando SEA.

Parametri SEA letti dal modello attivo `AB06_SEASEA.osim`:

```text
SEA_Knee:
  F_opt = 100 Nm
  K     = 1000 N*m/rad
  Kp    = 3.9
  Kd    = 9.7
  Bm    = 0.1
  Jm    = 0.01
  Impedence = false

SEA_Ankle:
  F_opt = 250 Nm
  K     = 700 N*m/rad
  Kp    = 8.8
  Kd    = 9.7
  Bm    = 0.1
  Jm    = 0.01
  Impedence = false
```

Overlay healthy:

```text
models/AB06_SEASEA_Threadmill/data/healthy/
```

con mapping:

```text
knee_angle_l_moment  -> reserve_pros_knee_angle
ankle_angle_l_moment -> reserve_pros_ankle_angle
```

Quindi l'overlay sano e' un momento netto da inverse dynamics del soggetto AB06
sano, non una coppia prodotta da una SEA.

## Verifiche numeriche

Le metriche sono state ricalcolate sui file `results/` caricando il setup AB06
operativo.

Errore cinematico rispetto alla reference IK filtrata usata dal simulatore:

```text
pros_knee_angle:
  q RMS       = 0.054718 rad
  q mean      = 0.023963 rad
  q max abs   = 0.167141 rad
  qdot RMS    = 0.340822 rad/s
  qdot max abs= 1.023772 rad/s

pros_ankle_angle:
  q RMS       = 0.094858 rad
  q mean      = 0.063706 rad
  q max abs   = 0.231940 rad
  qdot RMS    = 0.507272 rad/s
  qdot max abs= 6.992509 rad/s
```

Tracking inner SEA, usando la metrica corretta `tau_ref - tau_spring`:

```text
pros_knee_angle:
  max |u|                 = 0.306404
  max |tau_spring|         = 29.484864 Nm
  RMS(tau_ref-tau_spring) = 3.197243 Nm
  max |tau_ref-tau_spring|= 8.633281 Nm
  saturazioni tau_motor    = 0

pros_ankle_angle:
  max |u|                 = 0.389860
  max |tau_spring|         = 97.755248 Nm
  RMS(tau_ref-tau_spring) = 1.050540 Nm
  max |tau_ref-tau_spring|= 4.165224 Nm
  saturazioni tau_motor    = 0
```

Questo separa due problemi:

- l'outer loop non insegue perfettamente la cinematica;
- l'inner loop SEA, nella run corrente, segue abbastanza bene la `tau_ref` che
  gli viene chiesta.

## Diagnosi

### Errore cinematico in plot 6

L'errore non e' un semplice bug del plotter. E' coerente con un controllo PD
senza integratore e senza feedforward.

Un PD puro lascia errore di regime quando deve compensare disturbi quasi statici
o lentamente variabili, per esempio:

- gravita;
- GRF;
- mismatch dinamico tra modello protesico e cinematica sana;
- inerzie effettive dopo redistribuzione mass-preserving;
- deformazione e banda finita della SEA.

La caviglia e' il caso piu debole perche' ha `Kd = 1`, quindi e' poco smorzata.
Il ginocchio e' meglio bilanciato, ma mantiene comunque errore perche' non ha
ne' integratore ne' feedforward dinamico.

Il dataset contribuisce, ma non come "errore banale" dei dati. La reference e'
IK di AB06 sano adattata al modello protesico, e il report daily del 2026-05-12
documenta che l'RRA full-span e' stato saltato per residui non accettabili. Di
conseguenza la reference non e' perfettamente dinamicamente consistente con la
protesi e con le GRF applicate.

### Torque-angle non canonici in plot 3

Il confronto e' utile, ma non e' omogeneo al 100%.

La curva protesica e':

```text
SEA tau_spring vs angolo protesico simulato
```

La curva healthy e':

```text
momento netto ID biologico AB06 vs angolo sano AB06
```

Sono entrambe grandezze di coppia al giunto, ma nascono da fisiche diverse:

- SEA lineare con molla in serie e motore controllato;
- sistema muscolo-tendineo biologico non lineare;
- diversa distribuzione di massa;
- diversa generazione del push-off;
- diversa convenzione e normalizzazione rispetto a molte figure in letteratura.

In piu, il plotter applica una convenzione di segno al ginocchio per leggibilita.
Quindi non ci si deve aspettare che le curve protesiche riproducano
automaticamente le forme "canoniche" dei diagrammi biologici.

### Loop torque-angle compressi

La compressione non sembra piu causata dal gait-cycle detector. Il detector ora
produce 4 cicli sinistri e 4 destri fisiologici, e:

```text
plot/13_05_2026 - 3/missing_channels.txt
No missing channels.
```

La compressione deriva soprattutto da:

- ampiezza di coppia SEA piu bassa rispetto al momento biologico sano;
- comando outer PD che non chiede necessariamente il momento ID sano;
- tracking cinematico non perfetto;
- confronto tra `tau_spring` protesico e momento ID healthy;
- possibile rumore residuo da micro-GRF ancora presenti nei carichi della
  simulazione, anche se il detector le ignora nei gait cycle.

## Valutazione dell'ipotesi PID

### Outer loop PID

Un PID sull'outer loop e' giustificato.

Il termine integrale puo' ridurre l'offset di posizione, specialmente alla
caviglia, dove il PD deve compensare disturbi persistenti senza feedforward.

La forma consigliata e':

```text
tau_cmd = Kp * e_q + Kd * e_qdot + Ki * integral(e_q)
u = clip(tau_cmd / F_opt, -1, 1)
```

ma solo con protezioni:

- anti-windup;
- limite esplicito sull'integrale;
- eventuale integratore "leaky";
- reset dell'integrale a inizio simulazione;
- logging di contributo P, D, I e saturazione;
- sweep di `Ki`, non valore scelto a mano.

### Motor driver PID

Un PID anche nel motor driver e' plausibile, ma non e' la prima modifica da fare.

Nella run corrente:

- `tau_ref - tau_spring` e' piccolo;
- `|u|` resta lontano dalla saturazione;
- `tau_motor` non satura;
- il problema piu visibile resta il tracking cinematico dell'outer loop.

Due integratori accoppiati, uno sull'outer loop e uno sull'inner SEA, possono
creare oscillazioni o mascherare il vero collo di bottiglia. Il motor driver PID
andrebbe valutato solo se, dopo la correzione outer loop, resta un errore
sistematico in `tau_ref - tau_spring`.

## Strategia consigliata

Ordine operativo consigliato:

1. Pulire o azzerare nel file GRF i micro-contatti sotto durata minima e
   rilanciare la simulazione. Il detector e' gia robusto, ma le micro-GRF sono
   ancora carichi applicati alla dinamica.
2. Fare uno sweep conservativo del damping outer ankle, per esempio:

```text
Kd_ankle = 1, 3, 6, 10
```

3. Aggiungere un outer PID con `Ki` piccolo, anti-windup e logging diagnostico.
4. Ripetere il confronto su:

```text
q RMS
q max abs
qdot RMS
max |u|
tau_ref - tau_spring
tau_motor saturation count
joint power
torque-angle loop area
```

5. Solo se il residuo `tau_ref - tau_spring` resta sistematico, valutare un
   integratore anche nel motor driver.
6. Rendere esplicite nei plot le etichette:

```text
prosthetic = SEA spring torque
healthy    = AB06 biological ID joint moment
```

## Nota aperta su stiffness

Durante il controllo e' emerso un mismatch da tenere a mente:

```text
config.py:
  sea_stiffness["SEA_Knee"] = 250

AB06_SEASEA.osim:
  SEA_Knee stiffness = 1000
```

La run analizzata usa `sea_forward_mode = plugin` e il codice legge le proprieta
SEA dal modello tramite `ctx.sea_props`; quindi questo mismatch non appare la
causa principale del tracking corrente. Tuttavia `config.py` dichiara che la
stiffness dovrebbe combaciare col plugin, e il valore di fallback puo' contare
in modalita diagnostiche o future. Va allineato in una modifica separata.

## File modificati

Nessun codice modificato.

File aggiunto:

```text
reports/user/2026-05-13_analisi_tracking_pid_sea.md
```

## Test e verifiche eseguite

Sono stati ispezionati:

```text
AGENT.md
config.py
prosthesis_controller.py
plot/plotter.py
output.py
model_loader.py
simulation_runner.py
models/AB06_SEASEA_Threadmill/AB06_SEASEA.osim
reports/daily/2026-05-12_daily-report.md
reports/user/2026-05-13_ab06_grf_gait_cycle_plotter.md
reports/user/2026-05-13_ab06_healthy_overlay_threadmill.md
```

Sono stati letti/verificati:

```text
results/sim_output_run_status.txt
results/sim_output_states.sto
results/sim_output_sea_controls.sto
results/sim_output_sea_torques.sto
plot/13_05_2026 - 3/missing_channels.txt
```

Non e' stata rilanciata la simulazione e non sono stati rigenerati i plot.

