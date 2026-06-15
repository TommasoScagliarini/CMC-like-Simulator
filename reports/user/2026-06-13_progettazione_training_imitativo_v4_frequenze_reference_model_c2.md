# Progettazione training imitativo V4: frequenze, reference model C2 e curriculum

**Data:** 2026-06-13

## Obiettivo

Definire una strategia tecnicamente solida per il prossimo training imitativo,
partendo dai risultati e dalle anomalie osservate nel rollout V3:

- imitazione del ginocchio ancora insufficiente;
- grande estensione del ginocchio all'inizio dell'episodio;
- buon tracking del riferimento realmente servito;
- chattering marcato nei segnali SEA, soprattutto al knee;
- fallimento del rollout esteso a `5 s` per penetrazione GRF;
- dipendenza indesiderata della policy dalla phase normalizzata dell'episodio.

Il principio guida resta:

> migliorare la traiettoria generata e la sua compatibilita fisica senza
> modificare il plugin C++ o mascherare il problema attraverso un abbassamento
> arbitrario dei gain SEA gia validati con dati prescribed.

Questo report consolida la discussione progettuale e corregge esplicitamente
alcune interpretazioni iniziali errate sulle frequenze dei diversi livelli di
controllo.

## Stato verificato del rollout V3

La diagnostica corretta del riferimento realmente servito ha dimostrato che il
controllore high-level segue bene la traiettoria generata:

| Errore `simulated - served reference` | Knee | Ankle |
|---|---:|---:|
| RMSE globale | `2.50 deg` | `0.55 deg` |
| Errore massimo | `5.33 deg` | `1.97 deg` |

Il problema principale non e quindi un generico mancato tracking del
controllore. La policy genera un riferimento:

- non sufficientemente simile al target imitativo, soprattutto al knee;
- aggressivo all'inizio dell'episodio;
- continuo in posizione e velocita, ma non in accelerazione;
- capace di eccitare la dinamica SEA ad alta frequenza.

Il confronto V3/prescribed in modalita plugin ha mostrato:

| Metrica knee | Prescribed plugin | Policy V3 |
|---|---:|---:|
| `tau_ref` RMS | `11.68 Nm` | `55.62 Nm` |
| Torque error RMS | `1.30 Nm` | `11.83 Nm` |
| `tau_input_raw` RMS | `14.32 Nm` | `158.66 Nm` |
| Componente `tau_ref` attorno a `100 Hz` | `0.03 Nm` | `11.45 Nm` |

Questi dati non contraddicono la validazione dei controllori con riferimento
prescribed: il V3 sottopone la cascata e il SEA a un riferimento con contenuto
spettrale e richiesta di coppia molto diversi.

## Correzione delle frequenze effettive

### Errore corretto

Durante la prima discussione era stata riportata una tabella che indicava:

```text
integrazione SEA = 1000 Hz
outer loop       = 1000 Hz
```

Questa formulazione era imprecisa perche mescolava:

1. frequenza di callback digitale;
2. passo numerico dell'integratore;
3. banda dinamica equivalente di un sistema continuo.

### Timing effettivo del V3

Il run V3 usa:

```text
segment_duration   = 0.010 s
policy_knots       = 3
use_control_window = true
T_control          = 0.001 s
integration_dt     = 0.001 s
integration_scheme = rk4_bypass
sea_forward_mode   = plugin
```

La gerarchia temporale corretta e:

| Componente | Funzionamento effettivo |
|---|---|
| Policy / trajectory generator | una nuova azione ogni `10 ms`, quindi `100 Hz` |
| Knot policy | tre knot futuri dentro ogni segmento, distanziati circa `3.33 ms`, ma generati simultaneamente dalla stessa azione a `100 Hz` |
| Prosthetic outer cascade | ricalcolata ogni `T_control = 1 ms`, quindi callback a `1 kHz` |
| Biological outer loop + ID + SO | ricalcolati ogni `T_control = 1 ms`, quindi callback a `1 kHz` |
| Integrazione numerica | un passo RK4 ogni `1 ms` |
| Motor driver SEA nel plugin | ODE continua, senza callback digitale separato |

Il motor driver SEA non deve quindi essere descritto come un controllore
digitale a `1 kHz`. Il plugin definisce una dinamica continua che viene
integrata numericamente tramite RK4 con passo `1 ms`.

Le quattro valutazioni interne di RK4 per passo non corrispondono a un driver
digitale a `4 kHz`: sono valutazioni numeriche della stessa ODE con controlli
tenuti costanti durante la finestra.

I timestamp degli output del rollout confermano un campionamento registrato a
intervalli di `1 ms`.

### Bande dinamiche rilevanti

Per il knee SEA PI attuale del modello
`AB06_SEASEA_stiff321_500_pi.osim`:

```text
K  = 321 Nm/rad
Jm = 0.01 kg m^2
Bm = 0.1 Nm s/rad
Kp = 18
Kd = 11
Ki = 190
```

Le analisi precedenti hanno stimato:

```text
poli complessi motor driver  ~= -550 +/- 544j rad/s
frequenza naturale           ~= 123-124 Hz
frequenza oscillatoria       ~= 86-87 Hz
polo lento integrale         ~= 1.6 Hz
risonanza meccanica SEA      ~= 28-36 Hz
```

Per il cascade protesico attuale, le bande utili sono invece nell'ordine di
pochi hertz fino a circa una decina di hertz:

```text
position loop knee  ~= 3.0 Hz
position loop ankle ~= 7.5 Hz
velocity loop       ~= 7.5-12 Hz
riferimento cinematico utile principalmente entro circa 6 Hz
```

La gerarchia da considerare e quindi:

```text
riferimento cinematico utile   circa 0-6 Hz
outer cascade                  circa 3-12 Hz
risonanza meccanica SEA        circa 28-36 Hz
modo smorzato motor driver     circa 86-87 Hz
modo naturale motor driver     circa 123-124 Hz
callback outer / passo RK4     1000 Hz
```

### Interpretazione del problema a 100 Hz

Il problema del trajectory generator a `100 Hz` non e che sia piu veloce
dell'outer loop: l'outer viene aggiornato a `1 kHz`.

Il problema e che ogni nuovo segmento puo cambiare la legge di accelerazione
del riferimento. Questa discontinuita viene ripetuta ogni `10 ms`, introducendo
una componente attorno a `100 Hz`, vicina alla dinamica rapida del motor driver.

I tre knot interni possono inoltre produrre variazioni molto rapide entro il
segmento, ma non costituiscono tre callback distinti della rete.

## Frequenza proposta per il trajectory generator

Non e ancora corretto fissare direttamente una nuova frequenza unica.

La prima proposta di portare il generatore a `25 Hz` era insufficientemente
giustificata: `25 Hz` e vicino alla risonanza meccanica SEA identificata nella
fascia `28-36 Hz`. Rallentare il generatore senza correggere la regolarita del
riferimento potrebbe soltanto spostare l'eccitazione da un modo dinamico a un
altro.

La frequenza del generatore deve essere scelta rispetto a:

- banda utile della cinematica;
- banda dell'outer cascade;
- risonanze meccaniche SEA;
- dinamica del motor driver;
- armoniche prodotte dal meccanismo di interpolazione;
- qualita imitativa ottenibile.

### Strategia raccomandata

Prima rendere il riferimento servito almeno C2 e jerk-limited. Solo dopo,
confrontare tramite oracle e rollout controllati:

```text
20 Hz  -> segment_duration = 0.050 s
50 Hz  -> segment_duration = 0.020 s
100 Hz -> segment_duration = 0.010 s
```

Queste frequenze non sono ancora candidate approvate. Sono punti sperimentali
per misurare il compromesso tra:

- accuratezza della traiettoria;
- latenza di adattamento;
- contenuto spettrale;
- stress SEA;
- stabilita GRF.

Il confronto deve includere almeno:

- FFT e densita spettrale di `q_ref`, `qdot_ref`, `qddot_ref`;
- FFT di `tau_ref`, `tau_input`, `motor_speed` e `motor_speed_dot`;
- energia nelle bande `20-40 Hz`, `50-100 Hz` e sopra `100 Hz`;
- tracking `actual - served`;
- imitazione `served - target` e `actual - target`;
- saturazione e torque error SEA;
- penetrazione e continuita del contatto.

## Generazione del solo `q_ref` e requisito C2

### Perche il solo `q_ref` non basta

Se la policy produce solamente campioni discreti di posizione e l'ambiente
calcola:

```text
qdot_ref  = derivata numerica di q_ref
qddot_ref = derivata numerica di qdot_ref
```

allora variazioni finite tra campioni possono diventare picchi molto grandi
nelle derivate. Una interpolazione C1 evita salti in posizione e velocita, ma
non garantisce continuita dell'accelerazione.

Quindi la scelta:

> policy emette solo `q_ref`

e valida solamente se tra policy e controllore esiste un reference model
dinamico che produce coerentemente posizione, velocita e accelerazione.

### Limite del governor attuale

Il governor corrente:

1. riceve il segmento raw prodotto dalla policy;
2. costruisce una spline raw tramite `CubicHermiteSpline`;
3. usa un reference model del secondo ordine a `6 Hz`;
4. limita accelerazione e velocita;
5. integra posizione e velocita;
6. ricostruisce il segmento servito con una nuova `CubicHermiteSpline`.

Questo garantisce una buona continuita di:

```text
q_ref
qdot_ref
```

ma non garantisce globalmente la continuita di:

```text
qddot_ref
```

La spline Hermite cubica e C1 ai confini. L'accelerazione puo cambiare
bruscamente quando arriva una nuova azione policy, anche se rimane limitata in
ampiezza.

### Reference model raccomandato

La soluzione proposta e un reference model del terzo ordine con stato interno:

```text
x_ref = [q_ref, qdot_ref, qddot_ref]
```

La policy produce un comando di posizione lento:

```text
q_cmd
```

Il reference model calcola un jerk limitato:

```text
j_cmd = f(q_cmd - q_ref, qdot_ref, qddot_ref)
```

e integra:

```text
qddot_ref_next = qddot_ref + j_cmd * dt
qdot_ref_next  = qdot_ref  + qddot_ref * dt
q_ref_next     = q_ref     + qdot_ref * dt
```

con limiti espliciti:

```text
|j_cmd|     <= jerk_limit
|qddot_ref| <= acceleration_limit
|qdot_ref|  <= velocity_limit
q_min <= q_ref <= q_max
```

Il riferimento risultante e:

- continuo in posizione;
- continuo in velocita;
- continuo in accelerazione;
- con jerk limitato e potenzialmente discontinuo solo ai cambi comando;
- indipendente dal numero di knot interni emessi dalla policy.

### Azione raccomandata per il V4

Dopo l'introduzione del reference model C2, la prima architettura da provare e:

```text
una posizione desiderata per knee
una posizione desiderata per ankle
```

per un totale di due azioni per policy step.

Il passaggio da sei azioni a due riduce:

- dimensionalita del problema PPO;
- liberta di generare segmenti internamente incoerenti;
- clipping dei knot;
- derivate artificiali create dai tre punti futuri.

Il reference model, non la rete, diventa responsabile della regolarita fisica
di `q_ref`, `qdot_ref` e `qddot_ref`.

## `random_init=true`

### Funzionamento corrente

Con `random_init=true`, l'ambiente seleziona:

```text
episode_start ~ Uniform(t_start, t_end - episode_duration)
episode_end   = episode_start + episode_duration
```

In modalita imitation, `imitation_initialize_to_target=true` viene abilitato
automaticamente. L'ambiente:

1. calcola il target phase-based al tempo iniziale casuale;
2. inizializza il riferimento protesico sul target;
3. inizializza `q` e `qdot` del modello dal riferimento al tempo scelto;
4. inizializza il lato biologico dai dati prescribed allo stesso tempo;
5. resetta memoria dei controllori, SO e rilevatore eventi.

### Incoerenza iniziale SEA individuata

Durante il reset:

- `motor_angle` SEA viene posto uguale all'angolo articolare, annullando la
  deflessione iniziale della molla;
- gli altri stati plugin vengono inizializzati a zero;
- `motor_speed` resta quindi inizialmente zero.

Se il reset casuale avviene durante una fase in cui il giunto possiede velocita
non nulla, si ottiene:

```text
joint_speed != 0
motor_speed = 0
```

Questa condizione puo introdurre un transitorio artificiale proprio all'inizio
di ogni episodio casuale.

### Prerequisiti prima dell'abilitazione

`random_init=true` e raccomandato per evitare la memorizzazione del prefisso
iniziale e imparare una policy realmente ciclica, ma deve essere abilitato
solamente dopo:

1. rimozione della phase normalizzata dell'episodio dalle osservazioni actor;
2. inizializzazione coerente di `motor_speed` rispetto alla velocita del giunto;
3. valutazione di una deflessione iniziale SEA coerente con la coppia richiesta;
4. smoke test su molte phase casuali;
5. verifica che non compaiano transitori di reset nelle prime decine di
   millisecondi.

La modifica minima da validare e:

```text
motor_speed iniziale = joint_speed iniziale
```

ma la soluzione fisicamente completa potrebbe richiedere anche:

```text
motor_angle = joint_angle + tau_initial / K
```

se si vuole inizializzare la molla sulla coppia di equilibrio invece che a
deflessione nulla.

## Phase osservata dalla policy

Attualmente l'attore riceve sia:

```text
phase
phase_sin
phase_cos
```

basate sul tempo normalizzato dell'episodio, sia:

```text
gait_phase
gait_phase_sin
gait_phase_cos
```

basate sul gait clock ciclico.

La phase di episodio introduce una dipendenza non desiderata dalla durata del
rollout. Cambiando `episode_duration`, la stessa gait phase viene associata a
una diversa phase di episodio, modificando l'input della policy.

Per una policy periodica e utilizzabile su orizzonti arbitrari, la proposta e:

- rimuovere `phase`, `phase_sin`, `phase_cos`;
- rimuovere anche il valore scalare discontinuo `gait_phase`;
- mantenere `gait_phase_sin` e `gait_phase_cos`.

La coppia seno/coseno rappresenta la phase ciclica senza discontinuita al wrap
tra `1` e `0`.

Il target imitativo puo continuare a usare internamente:

```text
target_phase = (sound_gait_phase - joint_phase_shift) modulo 1
```

senza esporre il target direttamente all'attore.

## Separazione della reward imitativa

### Quattro segnali distinti

Per evitare ambiguita bisogna distinguere:

```text
q_target : target phase-based della gamba sana da imitare
q_cmd    : comando grezzo prodotto dalla policy
q_ref    : riferimento fisicamente governato e servito al controllore
q_actual : cinematica realmente simulata
```

La reward V3 valuta principalmente:

```text
q_actual vs q_target
q_actual vs q_ref
```

Manca una misura esplicita della qualita del trajectory generator:

```text
q_ref vs q_target
```

### Perche il termine diretto e necessario

Caso 1:

```text
q_target = -60 deg
q_ref    = -20 deg
q_actual = -22 deg
```

Il controllore segue bene, ma il generatore e sbagliato.

Caso 2:

```text
q_target = -60 deg
q_ref    = -60 deg
q_actual = -30 deg
```

Il generatore e corretto, ma l'esecuzione dinamica non segue.

Con la reward attuale entrambi i casi peggiorano l'imitazione effettiva, ma non
sono separati chiaramente. Aggiungere `q_ref vs q_target` rende diagnosticabile
e ottimizzabile direttamente il lavoro del trajectory generator.

### Struttura proposta

Definire tre loss separate:

```text
L_generator = errore normalizzato tra served reference e target imitativo
L_task      = errore normalizzato tra cinematica effettiva e target imitativo
L_execution = errore normalizzato tra cinematica effettiva e served reference
```

Tutte devono includere almeno posizione e velocita coerenti:

```text
L = w_q * errore_posizione^2 + w_v * errore_velocita^2
```

con normalizzazione e pesi distinti per knee e ankle. Il knee necessita
probabilmente di un peso maggiore, poiche e il giunto con imitazione peggiore,
mentre l'ankle e gia relativamente ben appreso.

Una composizione iniziale, da trattare come punto di partenza e non come tuning
approvato, potrebbe essere:

```text
60-65% qualita del riferimento generato
20-25% imitazione cinematica effettiva
10-15% tracking del riferimento servito
- penalita fisiche e di sicurezza
```

Il termine `L_execution` non deve dominare: il V3 ha gia dimostrato che il
controllore segue bene il served reference. Un peso eccessivo potrebbe premiare
riferimenti facili ma poco imitativi.

### Penalita da mantenere o migliorare

Mantenere:

- saturazione reale `tau_input`;
- torque error SEA;
- motor speed, acceleration e power;
- penetrazione GRF;
- limiti articolari e safety termination.

Migliorare:

- penalita sul jerk servito;
- penalita sugli interventi del governor;
- penalita sul salto del primo comando rispetto allo stato iniziale;
- metrica di clipping dell'azione grezza;
- supporto/carico protesico durante stance e swing.

La penalizzazione del jerk non deve sostituire il reference model C2: deve
premiare soluzioni naturalmente regolari, mentre il reference model garantisce
il vincolo fisico minimo.

## `gamma`

E stato deciso di mantenere:

```yaml
gamma: 0.95
```

Va ricordato che l'orizzonte temporale fisico associato a `gamma` dipende dalla
durata del policy step:

```text
orizzonte caratteristico ~= 1 / (1 - gamma) policy step
```

Con `gamma=0.95`:

```text
20 step caratteristici
```

Quindi:

| Policy step | Orizzonte fisico caratteristico |
|---:|---:|
| `10 ms` | circa `0.20 s` |
| `20 ms` | circa `0.40 s` |
| `50 ms` | circa `1.00 s` |

Mantenere `gamma` invariato non significa mantenere invariato l'orizzonte
fisico se cambia `segment_duration`. Questo effetto deve essere considerato
nell'interpretazione dei confronti tra frequenze del generatore.

## Durata degli episodi e curriculum

### Chiarimento

La proposta:

```text
inizialmente episodi da 2-3.5 s, poi 5 s
```

non indicava una modifica dinamica automatica della durata dentro lo stesso
training.

La strategia proposta consiste in training sequenziali, mantenendo invariati:

- observation space;
- action space;
- rete;
- reward;
- reference model.

Esempio:

```text
V4-A: training/smoke con episodi da 2.0 s
V4-B: resume del checkpoint con episodi da 3.5 s
V4-C: resume del checkpoint con episodi da 5.0 s
```

### Perche non usare subito un curriculum dinamico

Un cambio automatico della durata durante lo stesso run rende piu difficile
attribuire miglioramenti o regressioni a:

- apprendimento della policy;
- cambio della distribuzione temporale;
- maggiore esposizione a contatti e penetrazione;
- diversa frequenza di reset;
- maggiore probabilita di failure.

I training sequenziali consentono invece di validare ogni stadio e conservare
checkpoint chiaramente confrontabili.

### Prerequisito fondamentale

Il resume tra durate diverse e sensato solamente dopo aver rimosso la phase di
episodio dalle osservazioni actor. Altrimenti cambiare `episode_duration`
modifica il significato degli input della policy.

## Strategia V4 proposta

### Fase A - Correzioni strutturali

1. Rimuovere dall'attore la phase normalizzata dell'episodio.
2. Mantenere solamente `gait_phase_sin` e `gait_phase_cos` come clock ciclico.
3. Implementare reference model del terzo ordine C2/jerk-limited.
4. Ridurre l'azione a un comando posizione per giunto.
5. Correggere l'inizializzazione di `motor_speed` e valutare la deflessione
   iniziale coerente.
6. Aggiungere loss diretta `served reference vs imitation target`.

Queste modifiche cambiano observation/action contract e dinamica
azione-riferimento. Il primo V4 deve quindi partire da zero; non e corretto
continuare direttamente il checkpoint V3.

### Fase B - Oracle e smoke test

Prima del training lungo:

1. eseguire oracle imitativo con reference model C2;
2. confrontare generator rate `20/50/100 Hz`;
3. verificare assenza di componenti dominanti introdotte dalla frequenza di
   aggiornamento;
4. verificare tracking `actual - served`;
5. eseguire reset su molte gait phase casuali;
6. verificare i primi `100-250 ms` di ogni episodio;
7. eseguire un training breve da circa `10` iterazioni.

### Fase C - Training sequenziale

Configurazione iniziale conservativa:

```yaml
ppo:
  gamma: 0.95

simulation:
  episode_duration: 2.0
  random_init: true
  policy_knots: 1
```

`segment_duration` deve essere scelto dai test della fase B, non fissato prima.

Se i gate vengono superati:

```text
2.0 s -> 3.5 s -> 5.0 s
```

usando checkpoint separati e validazione deterministica tra uno stadio e il
successivo.

## Gate di validazione proposti

### Continuita e spettro del riferimento

- `q_ref`, `qdot_ref`, `qddot_ref` continui ai confini di segmento;
- jerk entro i limiti configurati;
- forte riduzione della componente periodica indotta dal policy step;
- nessun picco dominante vicino alle risonanze SEA introdotto dal generatore.

### Tracking e imitazione

- knee served-reference tracking RMSE inferiore a circa `3 deg`;
- ankle served-reference tracking RMSE inferiore a circa `1 deg`;
- knee imitation RMSE inferiore al V3 (`15.55 deg`) con obiettivo intermedio
  inferiore a `10 deg`;
- miglioramento della correlazione knee rispetto al V3 (`0.608`);
- ankle imitation non peggiore del V3.

### SEA

- clamp interno knee e ankle pari a zero o trascurabile;
- riduzione sostanziale di `tau_input_raw` RMS e torque error knee;
- riduzione del contenuto ad alta frequenza di `tau_ref`, `tau_input` e
  `motor_speed_dot`;
- nessun peggioramento nascosto tramite perdita di tracking.

### Reset e orizzonte

- nessun grande swing causato dal primo comando;
- nessun transitorio SEA artificiale dovuto al reset casuale;
- episodi casuali stabili su diverse gait phase;
- rollout da `5 s` completo senza terminazione per penetrazione;
- almeno un ciclo protesico completo osservabile e coerente.

### Robustezza del training

- smoke test breve prima del training completo;
- valutazione deterministica dei checkpoint;
- confronto su piu seed dopo la validazione della configurazione;
- selezione del checkpoint tramite gate fisici e imitativi, non solamente
  tramite episode return.

## File analizzati

- `config.py`
  - timing `dt`, `T_control`, `integration_dt`;
  - gain cascade protesici;
  - configurazione SEA e LPF.

- `simulation_runner.py`
  - callback outer a ogni finestra `T_control`;
  - integrazione `rk4_bypass`;
  - reset dello stato e inizializzazione SEA.

- `prosthesis_controller.py`
  - calcolo del cascade protesico;
  - aggiornamento della memoria del controllore a ogni callback.

- `Trajectory Generator/osim_trj_cmc_like.py`
  - mapping azione-segmento;
  - tre knot policy e derivate;
  - reference LPF/governor;
  - reset casuale;
  - phase osservate;
  - reward term e target imitativo.

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - composizione della reward imitation;
  - tracking e penalita SEA.

- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - configurazione PPO e simulation corrente.

- `Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win/training_cfg.resolved.yaml`
  - configurazione effettiva del training V3.

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`
  - proprieta effettive dei SEA PI.

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`
  - setup effettivamente usato dal V3.

- output del rollout diagnostico V3 e report tecnici precedenti sulle bande
  motor driver/cascade.

## File modificati

- `reports/user/2026-06-13_progettazione_training_imitativo_v4_frequenze_reference_model_c2.md`
  - nuovo report progettuale dettagliato.

Nessun file Python, modello `.osim`, plugin C++ o parametro di training e stato
modificato durante questa analisi.

## Test e verifiche eseguite

Verifiche read-only:

- lettura della configurazione risolta del run V3;
- verifica del setup XML effettivamente utilizzato;
- ricostruzione della catena `step -> step_until -> T_control -> RK4`;
- verifica che il setup XML non sovrascriva i timing del simulatore;
- verifica dei timestamp STO a intervalli di `1 ms`;
- verifica del mapping dei tre knot dentro ogni segmento da `10 ms`;
- rilettura del reference governor e della continuita garantita;
- rilettura della logica `random_init` e dell'inizializzazione SEA;
- verifica dei gain e degli stati del modello SEA PI corrente;
- confronto con le precedenti analisi di banda e chattering.

Non sono stati eseguiti nuovi rollout o training, poiche il lavoro corrente e
stato esclusivamente progettuale e diagnostico.

## Decisioni raggiunte

- [x] Mantenere `gamma=0.95`.
- [x] Non descrivere il motor driver SEA come callback digitale a `1 kHz`.
- [x] Distinguere frequenza di callback, passo numerico e banda dinamica.
- [x] Non fissare ancora la frequenza definitiva del trajectory generator.
- [x] Richiedere un riferimento C2/jerk-limited prima di confrontare le
      frequenze del generatore.
- [x] Considerare valida la generazione del solo comando posizione solamente se
      `q_ref`, `qdot_ref` e `qddot_ref` sono prodotti da un reference model
      dinamico coerente.
- [x] Preferire training sequenziali per aumentare la durata degli episodi,
      invece di un curriculum dinamico immediato.

## TODO

### Modifiche strutturali V4

- [ ] Rimuovere `phase`, `phase_sin` e `phase_cos` dalle osservazioni actor.
- [ ] Valutare la rimozione del valore scalare `gait_phase`, mantenendo
      `gait_phase_sin` e `gait_phase_cos`.
- [ ] Implementare un reference model del terzo ordine C2/jerk-limited.
- [ ] Ridurre l'azione policy a un comando posizione per giunto.
- [ ] Eliminare la dipendenza dalle derivate numeriche dei knot policy.
- [ ] Correggere l'inizializzazione di `motor_speed` nei reset casuali.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta.
- [ ] Aggiungere `served reference vs imitation target` alla reward.
- [ ] Normalizzare e pesare separatamente le loss knee/ankle.
- [ ] Aggiungere diagnostica e penalita sul jerk servito.
- [ ] Aggiungere metrica/reward minima di supporto e carico protesico.

### Validazione

- [ ] Creare oracle C2 per confrontare generator rate `20/50/100 Hz`.
- [ ] Misurare spettro e bande energetiche di riferimento e segnali SEA.
- [ ] Validare reset casuali su molte gait phase.
- [ ] Verificare assenza di swing iniziale e transitori SEA di reset.
- [ ] Eseguire smoke training V4 da circa `10` iterazioni.
- [ ] Eseguire training V4 da zero dopo la modifica dell'action/observation
      contract.
- [ ] Eseguire training sequenziali `2.0 s -> 3.5 s -> 5.0 s`.
- [ ] Validare rollout da `5 s` senza terminazione per penetrazione.
- [ ] Confrontare almeno tre seed dopo il superamento dei gate iniziali.

