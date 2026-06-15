# Analisi frequenziale dettagliata della configurazione PI

Data: 2026-06-14

## Problema

Dopo l'allineamento Windows del plugin PI e la promozione della configurazione
AB06 PI a default, era necessario caratterizzare in modo completo il regime
frequenziale dell'intera catena protesica. L'obiettivo non e' soltanto
confrontare PI e PD, ma determinare quali dinamiche vedono realmente il
trajectory generator, l'outer cascade e il motor driver.

L'analisi deve distinguere:

- frequenza di callback digitale;
- banda dinamica dell'outer cascade;
- dinamica continua del motor driver PI;
- risonanza meccanica rotore-molla;
- frequenza di aggiornamento e banda del riferimento generato.

L'analisi parte dal plant fisico, risale attraverso motor driver e outer
cascade, e termina sul reference model usato dal trajectory generator.

## Soluzione

E' stata costruita un'analisi frequenziale autonoma e multilivello della
configurazione PI default. Per ogni livello sono state derivate o stimate le
dinamiche rilevanti, separando:

- frequenze naturali, poli, zeri e bande `-3 dB`;
- dinamica isolata del motor driver e comportamento accoppiato whole-body;
- frequenza di aggiornamento digitale e banda del riferimento realmente
  servito;
- regime lineare non saturo e limiti introdotti da clamp e anti-windup.

I risultati analitici sono stati confrontati con i parametri del setup attivo,
con i range di inerzia effettiva gia' validati e con la simulazione Windows PI
`11.99-16.0 s`.

## Architettura della catena analizzata

```text
policy / trajectory generator
        |
        v
segmento raw di posizione protesica
        |
        v
reference model causale + governor
        |
        v
q_ref, qdot_ref, qddot_ref serviti
        |
        v
outer cascade position-P / velocity-PI
        |
        v
tau_ref = u * F_opt
        |
        v
motor driver PI nel plugin
        |
        v
rotore + molla SEA
        |
        v
tau_spring applicata al giunto e dinamica whole-body
```

Sono quindi presenti almeno quattro scale dinamiche diverse:

1. dinamica del riferimento servito;
2. outer cascade sul giunto protesico;
3. dinamica meccanica SEA rotore-molla;
4. motor driver PI interno.

La frequenza di aggiornamento digitale e la banda dinamica non sono la stessa
cosa. Un componente puo' essere valutato a `1 kHz` ma avere banda utile di
pochi hertz; analogamente, la policy puo' aggiornarsi a `100 Hz` producendo un
riferimento servito con banda molto piu' bassa.

## Configurazione analizzata

Setup e modello default:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim
```

### Outer loop cascade

```text
Knee:  Kp_outer=18.85,  Kp_inner=29.2,    Ki_inner=1377.0
Ankle: Kp_outer=47.125, Kp_inner=2.8275,  Ki_inner=213.0
```

Legge:

```text
qdot_cas = qdot_ref + Kp_outer * (q_ref - q)
tau_cmd  = Kp_inner * (qdot_cas - qdot)
         + Ki_inner * integral(qdot_cas - qdot) dt
```

### Motor driver PI effettivo

```text
tau_input = tau_ref
          + Kp * (tau_ref - tau_spring)
          + Ki * xi
          - Kd * omega_m

xi_dot = tau_ref - tau_spring
```

| SEA | Kp | Kd | Ki | K | Jm | Bm |
|---|---:|---:|---:|---:|---:|---:|
| Knee | 18.0 | 11.0 | 190.0 | 321 | 0.01 | 0.10 |
| Ankle | 11.3 | 11.0 | 123.0 | 500 | 0.01 | 0.10 |

I clamp e l'anti-windup rendono il sistema non lineare vicino alla
saturazione. I risultati frequenziali seguenti descrivono il regime
linearizzato non saturo, coerente con la full run validata.

## Ipotesi e dominio di validita'

L'analisi usa modelli linearizzati locali:

- motor driver analizzato con `theta_joint` bloccato oppure trattato come
  disturbo esogeno;
- outer velocity loop approssimato con plant scalare `1/(J_eff*s)`;
- parametri e inerzie considerati costanti nel punto operativo;
- clamp, anti-windup, limiti del governor e saturazioni esclusi dalle funzioni
  di trasferimento lineari;
- contatto piede-suolo e accoppiamento whole-body non linearizzati
  esplicitamente.

Queste ipotesi sono adeguate per identificare poli, bande e zone spettrali
critiche. Non sostituiscono la validazione in simulazione quando:

- `u` o `tau_input` raggiungono i clamp;
- l'integratore viene congelato dall'anti-windup;
- il governor limita velocita' o accelerazione;
- cambia la fase di contatto;
- la dinamica del knee cambia per accoppiamento con gli altri DOF.

## Livello 0: plant fisico SEA

La dinamica del rotore senza controllo, con il giunto trattato come posizione
imposta, e':

```text
tau_spring = K * (theta_m - theta_j)

Jm * theta_m_ddot + Bm * theta_m_dot + K * (theta_m - theta_j)
    = tau_input
```

Ponendo `theta_j=0`, la funzione di trasferimento tra coppia motore e coppia
elastica e':

```text
tau_spring / tau_input = K / (Jm*s^2 + Bm*s + K)
```

La frequenza naturale meccanica e lo smorzamento passivo sono:

```text
omega_mech = sqrt(K/Jm)
zeta_mech  = Bm / (2*sqrt(K*Jm))
```

| SEA | omega_mech | f_mech | zeta_mech | Periodo |
|---|---:|---:|---:|---:|
| Knee | `179.16 rad/s` | `28.51 Hz` | `0.0279` | `35.1 ms` |
| Ankle | `223.61 rad/s` | `35.59 Hz` | `0.0224` | `28.1 ms` |

Il plant passivo e' quindi poco smorzato. Le frequenze `28-36 Hz` non sono la
banda del controllore, ma modi fisici che possono essere eccitati da:

- riferimento con jerk o accelerazione impulsiva;
- feedback `qdot` rumoroso;
- impatti e transizioni di contatto;
- armoniche della frequenza di aggiornamento del trajectory generator.

### Limite indicativo imposto dal clamp motore

Per uno step a piena scala `tau_ref=F_opt`, il clamp `tau_input=500 Nm`
introduce un limite indicativo alla frequenza naturale ottenibile senza
richiedere istantaneamente piu' coppia del disponibile:

```text
omega_n,max ~= sqrt(500*K / (Jm*F_opt))
```

| SEA | omega_n,max | f_n,max |
|---|---:|---:|
| Knee | `400.62 rad/s` | `63.76 Hz` |
| Ankle | `316.23 rad/s` | `50.33 Hz` |

La dinamica rapida corrente del motor driver, circa `124 Hz`, supera questi
limiti indicativi per uno step a piena scala. Questo non implica che la
configurazione sia sempre saturata: significa che la banda corrente e'
utilizzabile solo per riferimenti abbastanza piccoli e regolari. La full run
prescribed validata non raggiunge il clamp.

## Motor driver PI

Con `theta_joint` bloccato, il polinomio caratteristico del driver e':

```text
Jm*s^3 + (Kd+Bm)*s^2 + (1+Kp)*K*s + Ki*K = 0
```

La funzione di trasferimento di tracking di coppia e':

```text
tau_spring / tau_ref =
    K * ((1+Kp)*s + Ki)
    -------------------------------------------------------
    Jm*s^3 + (Kd+Bm)*s^2 + (1+Kp)*K*s + Ki*K
```

La funzione di trasferimento tra velocita' del giunto, trattata come disturbo,
e coppia elastica e':

```text
tau_spring / omega_joint =
    -K*s*(Jm*s + Kd + Bm)
    -------------------------------------------------------
    Jm*s^3 + (Kd+Bm)*s^2 + (1+Kp)*K*s + Ki*K
```

Una rappresentazione state-space con stato
`x=[theta_m, omega_m, xi]` e' invece:

```text
x_dot = A*x + B_tau*tau_ref + B_joint*[theta_j, omega_j, ...]

A = [ 0                         1               0     ]
    [ -(1+Kp)K/Jm       -(Kd+Bm)/Jm          Ki/Jm   ]
    [ -K                        0               0     ]
```

Questa struttura mostra direttamente:

- una coppia di poli rapidi associata a rotore, molla e damping attivo;
- un polo reale lento associato all'integratore di errore di coppia;
- uno zero PI nel canale di tracking, quasi sovrapposto al polo lento.

### Poli e zero

| Grandezza | Knee | Ankle |
|---|---:|---:|
| Poli complessi | `-549.91 +/- 544.33j rad/s` | `-549.91 +/- 549.00j rad/s` |
| Frequenza naturale coppia complessa | `123.15 Hz` | `123.67 Hz` |
| Frequenza oscillatoria smorzata | `86.63 Hz` | `87.38 Hz` |
| Polo reale PI | `1.621 Hz` | `1.621 Hz` |
| Zero PI `Ki/(1+Kp)` | `1.592 Hz` | `1.592 Hz` |
| Banda tracking coppia `-3 dB` | `124.65 Hz` | `125.68 Hz` |
| Smorzamento coppia complessa | `0.711` | `0.708` |
| Costante di tempo inviluppo rapido | `1.82 ms` | `1.82 ms` |
| Assestamento indicativo `4*tau` | `7.27 ms` | `7.27 ms` |
| Costante di tempo polo reale | `98.16 ms` | `98.18 ms` |

Il polo lento e lo zero PI sono quasi coincidenti. Di conseguenza:

- il polo a `1.62 Hz` descrive la dinamica dell'azione integrale;
- non rappresenta la banda complessiva del motor driver;
- il tracking di coppia resta dominato dalla coppia rapida attorno a
  `123-124 Hz`.

Questa precisazione corregge la lettura troppo conservativa secondo cui il PI
renderebbe l'intero motor driver lento a circa `1.6 Hz`.

### Interpretazione della risposta in frequenza

Il tracking `tau_spring/tau_ref` presenta:

- guadagno unitario a frequenza zero;
- picco molto contenuto, circa `+0.16 dB`, attorno a `13-15 Hz`;
- attenuazione di `-3 dB` soltanto attorno a `125 Hz`;
- nessun forte picco risonante nel canale di tracking isolato.

Il fatto che la banda di tracking sia larga non significa che sia desiderabile
fornire riferimenti fino a `125 Hz`. Il driver seguirebbe tali componenti
trasferendo energia verso rotore, molla e giunto, mentre la dinamica meccanica
e il contatto whole-body presentano modi critici molto piu' bassi.

### Stabilita' del PI e margine sull'integratore

Per il polinomio cubico, la condizione Routh sul guadagno integrale e':

```text
Ki < (Kd+Bm)*(1+Kp)/Jm
```

| SEA | Ki corrente | Ki limite Routh | Rapporto |
|---|---:|---:|---:|
| Knee | `190` | `21090` | `0.0090` |
| Ankle | `123` | `13653` | `0.0090` |

Il PI e' quindi molto lontano dal limite lineare di instabilita'. Il rischio
operativo principale non e' una perdita di stabilita' Routh causata da `Ki`,
ma l'eccitazione delle dinamiche fisiche e l'ingresso nei clamp.

### Clamp e anti-windup

Il contributo integrale e' limitato a `+/-100 Nm`:

```text
|xi|max = integral_torque_limit / Ki
```

| SEA | Limite `|xi|` |
|---|---:|
| Knee | `0.5263` |
| Ankle | `0.8130` |

Quando il contributo integrale o `tau_input_raw` raggiungono i rispettivi
limiti e l'errore spinge nella stessa direzione, `xi_dot` viene congelato.
In tale regime il sistema non segue piu' la funzione di trasferimento lineare
descritta sopra.

Nella full run Windows `11.99-16.0 s`:

- nessun clamp `tau_input` e' stato raggiunto;
- gli stati integrali sono rimasti finiti e variabili;
- il massimo contributo integrale ricostruibile dagli stati e' rimasto circa
  `31.0 Nm` al knee e `17.0 Nm` all'ankle, ampiamente sotto `100 Nm`.

### Confronto col driver PD

Con gli stessi `Kp` e `Kd`, il PI modifica il modulo del tracking di coppia
rispetto al PD di meno di circa `0.16 dB`. La banda rapida rimane quindi quasi
invariata.

La differenza principale e' a bassissima frequenza:

- il PD presenta un errore di coppia a regime quando il giunto si muove a
  velocita' costante;
- il PI carica l'integratore e annulla tale bias a regime;
- l'effetto del PI sul rigetto del disturbo e' significativo soprattutto
  sotto circa `1-3 Hz`;
- sopra circa `6 Hz`, PI e PD hanno sensibilita' molto simile.

Bias del PD per velocita' articolare costante:

```text
Knee:  -(Kd+Bm)/(1+Kp) = -0.584 Nm/(rad/s)
Ankle: -(Kd+Bm)/(1+Kp) = -0.902 Nm/(rad/s)
```

Il PI elimina questo bias a regime. Nelle validazioni precedenti cio' ha
ridotto il torque-error RMS di circa `14%` al knee e `10%` all'ankle, senza
modificare sostanzialmente il tracking cinematico.

Il confronto PD/PI e' quindi una conseguenza secondaria dell'analisi completa:
la configurazione PI conserva la dinamica rapida del driver e aggiunge
accuratezza quasi-statica, senza introdurre un filtro efficace contro
componenti nella banda `20-100 Hz`.

## Interazione tra driver e risonanza meccanica

Il motor driver sposta i poli del sistema isolato dalla risonanza passiva
`28-36 Hz` alla coppia chiusa attorno a `123-124 Hz`, con modo oscillatorio
smorzato attorno a `87 Hz`. Questo risultato vale nell'analisi isolata con
giunto imposto.

Nel modello whole-body, invece, la risonanza meccanica originale resta
rilevante perche':

- `theta_joint` non e' realmente bloccato;
- il torque SEA agisce su un giunto con inerzia variabile;
- il contatto col terreno introduce shock e variazioni di vincolo;
- il velocity loop esterno usa direttamente `qdot` del giunto come feedback.

Le diagnosi precedenti hanno infatti osservato:

- picco knee vicino a `28-35 Hz` durante riferimenti RL aggressivi;
- rumore ankle legato a heel-strike e bassa inerzia effettiva;
- amplificazione del rumore `qdot` attraverso `Kp_inner` del cascade.

Quindi non esiste contraddizione tra una funzione di tracking del driver ben
smorzata e la presenza di oscillazioni whole-body nella banda `20-40 Hz`:
sono canali e condizioni al contorno differenti.

## Outer cascade

L'outer cascade riceve `q_ref` e `qdot_ref`, genera un riferimento di velocita'
corretto dalla posizione e chiude un PI sulla velocita':

```text
e_q      = q_ref - q
qdot_cas = qdot_ref + Kp_outer*e_q
e_v      = qdot_cas - qdot
tau_ref  = Kp_inner*e_v + Ki_inner*integral(e_v)dt
```

Se il motor driver e' approssimato come ideale nella banda dell'outer e il
giunto e' rappresentato da:

```text
qdot / tau_ref = 1 / (J_eff*s)
```

il velocity loop ha:

```text
C_v(s) = Kp_inner + Ki_inner/s

qdot / qdot_cas =
    Kp_inner*s + Ki_inner
    ----------------------------------------
    J_eff*s^2 + Kp_inner*s + Ki_inner
```

La posizione ha una frequenza caratteristica nominale data da `Kp_outer`:

| Loop posizione | rad/s | Hz |
|---|---:|---:|
| Knee | `18.85` | `3.00` |
| Ankle | `47.125` | `7.50` |

Il velocity PI introduce inoltre uno zero:

```text
omega_z,v = Ki_inner / Kp_inner
```

| Velocity PI | Zero [rad/s] | Zero [Hz] |
|---|---:|---:|
| Knee | `47.16` | `7.51` |
| Ankle | `75.33` | `11.99` |

Per il velocity PI, assumendo il plant scalare `1/(J_eff*s)`:

```text
omega_n = sqrt(Ki_inner/J_eff)
zeta    = Kp_inner / (2*sqrt(J_eff*Ki_inner))
```

Le inerzie effettive OpenSim precedentemente validate producono:

| Giunto / ipotesi | J_eff | f_n velocity | zeta velocity | Banda `-3 dB` indicativa |
|---|---:|---:|---:|---:|
| Knee, tutti i DOF liberi | `0.1597` | `14.78 Hz` | `0.984` | `36.3 Hz` |
| Knee, sola coppia protesica libera | `0.4450` | `8.85 Hz` | `0.590` | `16.9 Hz` |
| Knee, altri DOF bloccati | `0.4938` | `8.40 Hz` | `0.560` | `15.8 Hz` |
| Ankle, tutti i DOF liberi | `0.00989` | `23.35 Hz` | `0.974` | `56.9 Hz` |
| Ankle, sola coppia protesica libera | `0.01000` | `23.23 Hz` | `0.969` | `56.4 Hz` |
| Ankle, altri DOF bloccati | `0.01110` | `22.05 Hz` | `0.920` | `51.9 Hz` |

La banda `-3 dB` del velocity loop e' piu' alta della frequenza naturale
perche' la funzione chiusa contiene lo zero PI. Per interpretare la dinamica
transitoria e' quindi piu' utile osservare insieme poli, zero e risposta
whole-body.

Considerando anche il position loop, i poli dominanti della cascata completa
sono indicativamente:

| Cascata completa | Polo lento dominante | Coppia rapida |
|---|---:|---:|
| Knee, range J_eff | circa `2.6-2.8 Hz` | circa `9-15 Hz` |
| Ankle, range J_eff | circa `6.4 Hz` | circa `24-25 Hz` |

Il knee non e' ben rappresentato da una singola inerzia scalare: la sua banda
dipende fortemente dall'accoppiamento con gli altri DOF.

### Sensibilita' del comando outer

Il comando normalizzato e' `u=tau_ref/F_opt`. La sensibilita' proporzionale
immediata all'errore di velocita' e':

```text
du/de_v = Kp_inner/F_opt
```

| Giunto | Kp_inner/F_opt | Ki_inner/F_opt | Kp_inner*Kp_outer/F_opt |
|---|---:|---:|---:|
| Knee | `0.292` per rad/s | `13.77` | `5.504` per rad |
| Ankle | `0.01131` per rad/s | `0.852` | `0.533` per rad |

Il knee e' circa `25.8x` piu' sensibile dell'ankle all'errore di velocita'
istantaneo. Questo spiega perche' un riferimento protesico a banda larga o un
feedback `qdot` rumoroso tendono a saturare prima il knee, anche se l'ankle ha
inerzia effettiva molto piu' bassa.

### Non-linearita' dell'outer

L'integrale velocity e' limitato in coppia:

```text
Knee:  |Ki_inner*xi_v| <= 50 Nm   -> |xi_v| <= 0.0363
Ankle: |Ki_inner*xi_v| <= 200 Nm  -> |xi_v| <= 0.9390
```

Inoltre `u` e' limitato a `[-1,1]`. Quando questi limiti sono attivi, poli e
bande del modello lineare non descrivono piu' correttamente il comportamento.

## Timing digitale e integrazione numerica

La configurazione CMC-like usa:

```text
T_control         = 0.001 s
integration_dt    = 0.001 s
integration_scheme= rk4_bypass
sea_forward_mode  = plugin
```

Interpretazione corretta:

- outer cascade, inverse dynamics e static optimization vengono ricalcolati
  ogni `1 ms`, quindi a `1 kHz`;
- `tau_ref` viene mantenuto costante tra due callback outer, secondo uno
  zero-order hold;
- il motor driver e' una ODE continua valutata dal plugin durante RK4;
- le valutazioni interne di RK4 non sono callback digitali aggiuntivi;
- la frequenza di Nyquist degli output e del controllo outer e' `500 Hz`.

### Risoluzione temporale dei modi

| Frequenza | Campioni per periodo a 1 kHz | Fase ZOH indicativa |
|---:|---:|---:|
| `3 Hz` | `333` | `-0.54 deg` |
| `6 Hz` | `167` | `-1.08 deg` |
| `15 Hz` | `66.7` | `-2.7 deg` |
| `25 Hz` | `40` | `-4.5 deg` |
| Knee meccanica `28.5 Hz` | `35.1` | `-5.1 deg` |
| Ankle meccanica `35.6 Hz` | `28.1` | `-6.4 deg` |
| Modo driver `87 Hz` | circa `11.5` | circa `-15.7 deg` |
| Banda driver `125 Hz` | `8` | `-22.5 deg` |
| Nyquist `500 Hz` | `2` | `-90 deg` |

Il passo da `1 ms` rappresenta molto bene outer e risonanze meccaniche, ma
offre soltanto circa otto campioni per periodo vicino alla banda rapida del
driver. Questo e' sufficiente per la configurazione validata, ma riduce il
margine disponibile per aumentare ulteriormente la banda interna o per
iniettare contenuto vicino a `125 Hz`.

## Trajectory generator e riferimento servito

La frequenza del trajectory generator definisce ogni quanto la policy produce
un nuovo segmento. Non definisce direttamente la banda del riferimento
inseguito dall'outer.

Configurazione training corrente:

```text
segment_duration = 0.010 s
policy rate      = 100 Hz
policy_knots     = 3
```

I tre knot vengono prodotti simultaneamente dalla stessa azione policy. Non
corrispondono a tre callback separate della rete.

### Origine delle componenti spettrali

Anche se la posizione e' continua, ogni confine di segmento puo' introdurre:

- variazione della velocita';
- discontinuita' o rapido cambio dell'accelerazione;
- jerk impulsivo;
- armoniche della frequenza di aggiornamento policy.

Quindi una policy a `100 Hz` puo' introdurre una riga spettrale a `100 Hz` e
armoniche, anche se il moto fisiologico desiderato e' sotto `6 Hz`.

### Reference model protesico online

Nell'env RL il riferimento raw della policy attraversa un modello causale del
secondo ordine:

```text
qf_ddot = wn^2*(q_target - qf) - 2*zeta*wn*qf_dot

wn   = 2*pi*6 rad/s
zeta = 1
```

Funzione di trasferimento ideale:

```text
qf / q_target = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
```

Con `zeta=1`, il parametro chiamato cutoff `6 Hz` e' la frequenza naturale,
non la frequenza `-3 dB`:

```text
f_-3dB = 0.6436 * 6 Hz = 3.86 Hz
```

Attenuazione ideale del reference model:

| Frequenza | Attenuazione |
|---:|---:|
| `3 Hz` | `-1.94 dB` |
| `3.86 Hz` | `-3.01 dB` |
| `6 Hz` | `-6.02 dB` |
| `20 Hz` | `-21.66 dB` |
| `25 Hz` | `-25.28 dB` |
| `28.5 Hz` | `-27.45 dB` |
| `35.6 Hz` | `-31.17 dB` |
| `50 Hz` | `-36.96 dB` |
| `100 Hz` | `-48.91 dB` |

Questo reference model e' la protezione principale contro l'eccitazione delle
risonanze, non il PI del motor driver.

### Differenza rispetto al filtro IK

Il filtro IK sperimentale e' un Butterworth ordine 4 offline e zero-phase con
cutoff `6 Hz`. Il reference model protesico e' invece:

- ordine 2;
- causale;
- criticamente smorzato;
- con lag di fase;
- con `-3 dB` effettivo circa a `3.86 Hz`.

I due riferimenti sono quindi entrambi limitati in banda, ma non hanno la
stessa risposta in modulo e fase.

### Continuita' del riferimento

Il reference model mantiene stato continuo di posizione e velocita' tra i
segmenti e il governor limita velocita' e accelerazione. Tuttavia la
ricostruzione corrente con spline Hermite non garantisce globalmente
continuita' C2 dell'accelerazione ai confini.

Per questo la frequenza di policy resta una variabile da validare anche con il
filtro attivo: un riferimento non pienamente C2 puo' ancora produrre jerk e
stress SEA ai confini di segmento.

## Gerarchia frequenziale aggiornata

| Livello | Knee | Ankle |
|---|---:|---:|
| Reference model online: `-3 dB` | `3.86 Hz` | `3.86 Hz` |
| Parametro naturale reference model | `6 Hz` | `6 Hz` |
| Outer position correction | circa `3.0 Hz` | circa `7.5 Hz` |
| Outer cascade rapido | circa `9-15 Hz` | circa `24-25 Hz` |
| Risonanza meccanica SEA | `28.5 Hz` | `35.6 Hz` |
| Motor driver: modo oscillatorio | `86.6 Hz` | `87.4 Hz` |
| Motor driver: frequenza naturale | `123.1 Hz` | `123.7 Hz` |
| Motor driver: banda `-3 dB` | `124.6 Hz` | `125.7 Hz` |
| Policy corrente | `100 Hz` | `100 Hz` |
| Callback outer / passo RK4 | `1000 Hz` | `1000 Hz` |

Il polo/zero PI a circa `1.6 Hz` va interpretato come azione integrale lenta,
non come limite della banda complessiva del driver.

La separazione non e' uniforme:

- knee: l'outer rapido resta generalmente sotto la risonanza meccanica, con
  margine dipendente dall'accoppiamento;
- ankle: la coppia rapida dell'outer, circa `24-25 Hz`, e' relativamente vicina
  alla risonanza meccanica a `35.6 Hz`;
- motor driver e' molto piu' rapido dell'outer, ma la sua banda ampia gli
  consente di trasmettere richieste indesiderate invece di filtrarle;
- il reference model online e' il vero elemento che separa la banda
  cinematica dalla banda di risonanza.

## Impatto sulla frequenza del trajectory generator

L'introduzione del PI non rende sicuro un riferimento ad alta frequenza:

- il PI migliora precisione e rigetto dei disturbi lenti;
- non attenua significativamente la banda `20-100 Hz`;
- la banda rapida del motor driver resta circa `125 Hz`;
- discontinuita' ripetute a `100 Hz` restano vicine al modo rapido del driver;
- componenti o armoniche tra `20` e `40 Hz` restano vicine alle risonanze SEA.

Il trajectory generator puo' essere aggiornato anche piu' velocemente
dell'outer position loop, ma il riferimento realmente servito deve essere
band-limited e regolare.

### Analisi delle frequenze candidate

#### Policy a 20 Hz

```text
segment_duration = 50 ms
```

- frequenza fondamentale sotto le risonanze SEA;
- seconda armonica a `40 Hz`, vicina/sopra le risonanze;
- aggiornamento lento rispetto all'outer ankle;
- maggiore latenza di adattamento;
- richiede comunque continuita' ai confini per evitare armoniche.

#### Policy a 25 Hz

```text
segment_duration = 40 ms
```

- molto vicina alla risonanza knee a `28.5 Hz`;
- armoniche facilmente dentro la banda `20-100 Hz`;
- non raccomandata come frequenza nominale senza evidenza sperimentale forte.

#### Policy a 50 Hz

```text
segment_duration = 20 ms
```

- sopra le risonanze meccaniche ma ancora ben dentro la banda del driver;
- seconda armonica a `100 Hz`, vicina al modo rapido del driver;
- buon compromesso potenziale tra latenza ed espressivita', ma non e'
  automaticamente sicura.

#### Policy a 100 Hz

```text
segment_duration = 10 ms
```

- vicina al modo oscillatorio del driver `86-87 Hz`;
- dentro la banda di tracking di coppia del driver;
- puo' produrre una componente ripetitiva a `100 Hz`;
- e' utilizzabile soltanto se il reference model attenua fortemente la riga a
  `100 Hz` e i confini non introducono jerk rilevante.

### Aliasing e osservabilita'

Con output e callback a `1 kHz`, le candidate `20/50/100 Hz` sono tutte sotto
Nyquist e osservabili. Il problema principale non e' aliasing digitale diretto,
ma:

- energia introdotta alla fondamentale e alle armoniche;
- intermodulazione con gait e contatto;
- modifica dello spettro dovuta ai limiti non lineari;
- trasmissione di componenti indesiderate dal driver a banda larga.

Contenuto sopra `500 Hz` eventualmente prodotto da discontinuita' o shock non
e' rappresentabile correttamente negli output a `1 ms` e puo' ripiegarsi o
manifestarsi come stress numerico. La continuita' C2 e il limite sul jerk
servono anche a prevenire questa situazione.

### Conclusioni operative

- `100 Hz`: utilizzabile solo se il riferimento servito e' C2, jerk-limited e
  con energia residua trascurabile vicino a `87-125 Hz`;
- `50 Hz`: non automaticamente sicuro, perche' cade tra risonanza meccanica e
  modo rapido del driver;
- `25 Hz`: sconsigliato come scelta diretta perche' vicino alla risonanza knee;
- `20 Hz`: puo' ancora eccitare la risonanza tramite armoniche se i confini di
  segmento non sono regolari.

La frequenza di aggiornamento non deve quindi essere usata come sostituto di un
reference model fisico.

## Strategia raccomandata

La scelta della frequenza definitiva deve avvenire dopo avere reso il
riferimento servito almeno C2 e jerk-limited. Successivamente confrontare in
condizioni controllate:

```text
20 Hz  -> segment_duration = 0.050 s
50 Hz  -> segment_duration = 0.020 s
100 Hz -> segment_duration = 0.010 s
```

Per ogni variante misurare:

- FFT/PSD di `q_ref`, `qdot_ref`, `qddot_ref` e jerk;
- energia nelle bande `20-40 Hz`, `50-100 Hz` e oltre `100 Hz`;
- tracking `actual - served` e imitazione `served - target`;
- `tau_ref`, `tau_input`, torque error, `motor_speed` e `motor_speed_dot`;
- saturazioni e attivazione degli anti-windup;
- penetrazione e continuita' del contatto GRF.

### Segnali da distinguere

L'analisi deve salvare separatamente:

```text
q_cmd_raw      : comando prodotto direttamente dalla policy
q_ref_served   : riferimento dopo reference model e governor
q_actual       : stato del giunto simulato
tau_ref        : coppia richiesta dall'outer cascade
tau_spring     : coppia realmente trasmessa dalla molla
tau_input      : coppia motore calcolata dal plugin
```

Senza questa separazione non e' possibile attribuire correttamente una
componente spettrale a policy, reference model, outer o driver.

### Bande di integrazione consigliate

| Banda | Significato principale |
|---|---|
| `0-6 Hz` | cinematica utile e gait |
| `6-20 Hz` | transizione tra riferimento e outer |
| `20-40 Hz` | risonanze meccaniche SEA |
| `40-80 Hz` | chattering e armoniche intermedie |
| `80-140 Hz` | modo rapido e banda del motor driver |
| `140-500 Hz` | contenuto numerico/impulsivo sotto Nyquist |

### Gate minimi

Una frequenza candidata e' accettabile soltanto se:

- non introduce picchi dominanti nella banda `20-40 Hz`;
- riduce fortemente la riga alla frequenza policy dopo il reference model;
- non produce energia crescente nella banda `80-140 Hz`;
- non causa clamp `tau_input` o anti-windup persistente;
- mantiene il tracking `actual-served` entro le soglie definite;
- migliora o preserva l'imitazione `served-target`;
- completa rollout lunghi senza instabilita' GRF.

### Ordine degli esperimenti

1. Oracle con traiettoria nota, stessa traiettoria campionata a `20/50/100 Hz`.
2. Confronto con reference model attivo e disattivo per misurarne
   l'attenuazione reale.
3. Rollout deterministico con policy congelata.
4. Rollout su piu' gait phase e contatti iniziali.
5. Solo dopo i gate precedenti, training breve comparativo.

## File e verifiche

Nessun file di codice e' stato modificato per questa analisi.

Sorgenti verificate:

- `config.py`;
- `prosthesis_controller.py`;
- `tools/sea_plugin_relative_d/SeriesElasticActuator.cpp`;
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`;
- `Trajectory Generator/osim_trj_cmc_like.py`;
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`;
- report precedenti su PI, cascade, inerzia effettiva e trajectory generator.

Calcoli eseguiti:

- radici del polinomio caratteristico PI;
- zero PI;
- banda `-3 dB` del tracking di coppia;
- confronto frequenziale PI/PD;
- risonanza meccanica rotore-molla;
- poli indicativi dell'outer cascade usando i range validati di `J_eff`.
- sensibilita' normalizzata del comando outer;
- risoluzione temporale e fase indicativa dello zero-order hold;
- risposta ideale del reference model online del secondo ordine.

## Esito

La catena completa presenta la seguente struttura:

```text
reference model utile:       circa 0-4 Hz a -3 dB
outer position/cascade:      circa 3-25 Hz, dipendente dal giunto
risonanza meccanica SEA:     28-36 Hz
modo rapido motor driver:    87-125 Hz
policy corrente:             100 Hz
callback e integrazione:     1000 Hz
```

Il PI interno garantisce accuratezza quasi-statica e un tracking di coppia
molto rapido. Proprio per questo non va considerato un filtro protettivo:
trasmette efficacemente anche richieste non desiderate nella banda alta.

La protezione principale e' il reference model causale, che con parametro
`6 Hz` e `zeta=1` ha in realta' `-3 dB` a circa `3.86 Hz` e attenua
fortemente le risonanze e la riga policy a `100 Hz`.

La frequenza definitiva del trajectory generator deve essere scelta soltanto
dopo avere garantito continuita' C2, limite sul jerk e verifica spettrale del
riferimento realmente servito.

## TODO

- Implementare e validare il reference model C2/jerk-limited prima di fissare
  la frequenza definitiva del trajectory generator.
- Confrontare sperimentalmente `20/50/100 Hz` con metriche spettrali e
  dinamiche identiche.
