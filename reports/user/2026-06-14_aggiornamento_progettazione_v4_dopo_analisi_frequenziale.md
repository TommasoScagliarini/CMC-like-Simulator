# Aggiornamento progettazione V4 dopo l'analisi frequenziale

**Data:** 2026-06-14

## Problema

La progettazione preliminare del prossimo training imitativo V4 aveva
individuato come priorita:

- separare la frequenza di aggiornamento della policy dalla banda del
  riferimento realmente servito;
- mantenere il motor driver PI e i controllori SEA validati;
- eliminare il chattering generato dai segmenti policy;
- passare a un riferimento continuo almeno in accelerazione;
- valutare se mantenere la policy a `100 Hz` oppure ridurne la frequenza.

Successivamente e stata redatta una caratterizzazione frequenziale piu completa:

`reports/user/2026-06-14_analisi_frequenziale_configurazione_pi.md`

Era quindi necessario confrontare tale analisi con le raccomandazioni
precedenti e aggiornare le decisioni progettuali del V4.

## Sintesi della nuova analisi frequenziale

La nuova analisi distingue correttamente:

- callback digitale;
- passo numerico;
- banda dinamica;
- poli e zeri dei controllori;
- risonanze meccaniche;
- banda del riferimento servito.

La gerarchia frequenziale aggiornata e:

| Livello | Knee | Ankle |
|---|---:|---:|
| Reference model online attuale, banda `-3 dB` | `3.86 Hz` | `3.86 Hz` |
| Parametro naturale reference model attuale | `6 Hz` | `6 Hz` |
| Outer position correction | circa `3.0 Hz` | circa `7.5 Hz` |
| Coppia rapida cascata completa | circa `9-15 Hz` | circa `24-25 Hz` |
| Risonanza meccanica SEA | `28.5 Hz` | `35.6 Hz` |
| Modo oscillatorio motor driver | `86.6 Hz` | `87.4 Hz` |
| Banda tracking motor driver `-3 dB` | `124.6 Hz` | `125.7 Hz` |
| Policy corrente | `100 Hz` | `100 Hz` |
| Callback outer e passo RK4 | `1000 Hz` | `1000 Hz` |

Il motor driver PI:

- migliora accuratezza e rigetto dei disturbi lenti;
- conserva una banda di tracking di coppia molto ampia;
- non costituisce un filtro efficace nella banda `20-100 Hz`;
- puo trasmettere al SEA e al giunto richieste indesiderate ad alta frequenza.

Il reference model causale e quindi il principale elemento che deve separare la
cinematica utile dalle risonanze e dalla dinamica rapida del motor driver.

## Cosa cambia rispetto all'analisi precedente

### Banda dell'outer cascade

La precedente sintesi descriveva il velocity loop soprattutto attraverso valori
nell'ordine di `7.5-12 Hz`. Tale descrizione era incompleta.

L'analisi aggiornata mostra che:

- il knee possiede una dinamica rapida indicativa attorno a `9-15 Hz`;
- l'ankle possiede una dinamica rapida attorno a `24-25 Hz`;
- la banda `-3 dB` del velocity loop puo essere ancora maggiore a causa dello
  zero PI.

Quindi l'outer e capace di reagire a componenti relativamente rapide,
soprattutto all'ankle. Questa capacita non deve essere interpretata come
autorizzazione a fornire un riferimento a banda larga: componenti rapide
possono avvicinarsi alle risonanze meccaniche SEA.

### Ruolo del motor driver PI

Il polo e lo zero PI attorno a `1.6 Hz` descrivono principalmente l'azione
integrale lenta e quasi si cancellano nel canale di tracking.

Non limitano la banda complessiva del motor driver, che rimane circa:

```text
125 Hz a -3 dB
```

Il driver PI non protegge quindi l'intera catena dalle richieste ad alta
frequenza. Al contrario, il tracking rapido gli consente di trasferirle
efficacemente.

### Frequenza della policy

La nuova analisi conferma che non esiste una frequenza policy automaticamente
sicura:

- `25 Hz` e vicina alla risonanza knee;
- `20 Hz` puo eccitare le risonanze tramite armoniche;
- `50 Hz` resta dentro la banda del driver e possiede seconda armonica a
  `100 Hz`;
- `100 Hz` e vicina al modo rapido del driver, ma puo essere utilizzata se la
  riga a `100 Hz` viene sufficientemente attenuata prima dell'outer.

La frequenza policy non deve quindi essere usata come sostituto di un
reference model fisico.

## Decisione aggiornata sulla policy a 100 Hz

Per il primo V4 si raccomanda di mantenere:

```text
segment_duration = 0.010 s
policy rate      = 100 Hz
```

Motivazioni:

1. permette di isolare l'effetto del nuovo reference model senza cambiare
   contemporaneamente anche la frequenza decisionale;
2. conserva una risposta rapida della policy a stato, GRF e contatti;
3. il problema osservato nel V3 non e la callback a `100 Hz` in se, ma la sua
   presenza nello spettro di `qddot_ref` e `tau_ref`;
4. ridurre la frequenza non garantisce automaticamente minore eccitazione delle
   risonanze.

Il mantenimento dei `100 Hz` non costituisce ancora una decisione definitiva
per tutti i training futuri. E la configurazione iniziale piu pulita per
validare il nuovo reference model.

## Limite del reference model attuale

Il reference model online attuale e del secondo ordine:

```text
qf_ddot = wn^2 * (q_target - qf) - 2*zeta*wn*qf_dot

wn   = 2*pi*6 rad/s
zeta = 1
```

La funzione di trasferimento posizione e:

```text
Qref(s) / Qcmd(s) =
wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
```

Con `zeta=1`, il parametro nominale `6 Hz` non e la banda `-3 dB`. La banda
effettiva e:

```text
f_-3dB = 0.6436 * 6 Hz = 3.86 Hz
```

Questo modello attenua fortemente la posizione servita alle alte frequenze. A
`100 Hz`, l'attenuazione ideale di `q_ref` e circa `-49 dB`.

Tuttavia il problema del V3 non riguarda solamente il modulo di `q_ref`.

Il trasferimento dal comando posizione all'accelerazione e:

```text
Qddot_ref(s) / Qcmd(s) =
s^2 * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
```

Per frequenze molto elevate:

```text
Qddot_ref(s) / Qcmd(s) -> wn^2
```

Quindi:

- la posizione viene fortemente attenuata;
- le variazioni rapide del comando possono ancora produrre accelerazione
  significativa;
- la ricostruzione tramite spline Hermite garantisce continuita C1, ma non
  continuita globale C2;
- `qddot_ref` puo cambiare bruscamente ai confini dei segmenti;
- il jerk puo contenere componenti impulsive o molto elevate.

Questo spiega perche nel rollout V3 compare una componente vicina a `100 Hz`
in `qddot_ref` e `tau_ref`, nonostante il filtro di posizione sia attivo.

## Reference model V4 raccomandato

### Struttura

Si raccomanda un reference model causale del terzo ordine con stato:

```text
x_ref = [q_ref, qdot_ref, qddot_ref]
```

e comando policy:

```text
q_cmd
```

Una possibile struttura lineare e un Butterworth del terzo ordine:

```text
qref''' =
    wc^3 * (q_cmd - q_ref)
    - 2*wc^2 * qdot_ref
    - 2*wc * qddot_ref
```

con:

```text
wc = 2*pi*fc
```

La funzione di trasferimento e:

```text
Qref(s) / Qcmd(s) =
wc^3 / (s^3 + 2*wc*s^2 + 2*wc^2*s + wc^3)
```

In questo caso:

```text
Qddot_ref(s) / Qcmd(s) -> wc^3/s -> 0
```

alle alte frequenze. Rispetto al modello di secondo ordine, viene quindi
attenuata anche l'accelerazione prodotta dalle variazioni rapide del comando.

### Continuita

Il modello del terzo ordine permette di ottenere:

- `q_ref` continua;
- `qdot_ref` continua;
- `qddot_ref` continua;
- jerk finito e limitabile;
- stato coerente mantenuto tra azioni policy.

Il riferimento servito non deve piu dipendere dalla derivazione numerica dei
knot prodotti dalla policy.

### Limiti fisici

Il reference model deve includere limiti espliciti:

```text
|qdot_ref|  <= velocity_limit
|qddot_ref| <= acceleration_limit
|qref'''|   <= jerk_limit
q_min <= q_ref <= q_max
```

Il filtro dinamico determina la risposta in frequenza nominale. I limiti
gestiscono invece grandi transitori e richieste non lineari della policy.

I limiti non sostituiscono la banda e la banda non sostituisce i limiti.

## Banda iniziale raccomandata

Per il primo V4 si raccomanda:

```text
banda reale -3 dB iniziale: circa 4 Hz
```

Motivazioni:

1. il reference model attuale ha gia una banda effettiva di circa `3.86 Hz`;
2. una banda iniziale di `4 Hz` mantiene comparabile il contenuto cinematico
   rispetto al V3;
3. consente di attribuire le differenze osservate principalmente alla
   continuita C2 e al limite sul jerk;
4. evita di aumentare contemporaneamente anche l'autorita dinamica del
   riferimento;
5. il polo dominante knee e circa `2.6-2.8 Hz`, quindi aumentare subito la
   banda potrebbe peggiorare tracking e stress knee.

Dopo la validazione a `4 Hz`, confrontare:

```text
fc = 4 Hz
fc = 6 Hz
```

Il confronto deve essere effettuato misurando il riferimento realmente servito,
non affidandosi solamente al parametro nominale del filtro.

## Azione policy raccomandata

Per il primo V4 la policy dovrebbe produrre:

```text
q_cmd_knee
q_cmd_ankle
```

quindi due sole azioni per step.

La policy non deve piu produrre tre knot futuri e relative derivate implicite.
Il reference model del terzo ordine e responsabile di trasformare i comandi
discreti a `100 Hz` in:

```text
q_ref
qdot_ref
qddot_ref
```

fisicamente coerenti e band-limited.

Per mantenere l'osservazione Markoviana, l'attore dovrebbe ricevere anche lo
stato del reference model:

```text
q_ref
qdot_ref
qddot_ref
```

oppure una rappresentazione equivalente sufficiente a ricostruirne lo stato.

## Configurazione iniziale V4

La configurazione concettuale iniziale diventa:

```yaml
ppo:
  gamma: 0.95

simulation:
  segment_duration: 0.01
  policy_knots: 1
  episode_duration: 2.0

  pros_ref_model: butterworth3_jerk_limited
  pros_ref_cutoff_hz: 4.0

  pros_knee_ref_velocity_limit_rad_s: 6.0
  pros_ankle_ref_velocity_limit_rad_s: 3.5
  pros_knee_ref_acceleration_limit_rad_s2: 60.0
  pros_ankle_ref_acceleration_limit_rad_s2: 55.0

  pros_knee_ref_jerk_limit_rad_s3: da tarare
  pros_ankle_ref_jerk_limit_rad_s3: da tarare
```

Questa configurazione e progettuale. I nuovi campi non sono ancora
implementati nella configurazione YAML corrente.

## Gate spettrali aggiornati

Il V4 a `100 Hz` e accettabile solamente se il reference model dimostra:

- riga a `100 Hz` trascurabile in `qddot_ref`;
- nessun picco dominante nella banda `20-40 Hz`;
- nessuna crescita indesiderata nella banda `80-140 Hz`;
- jerk finito e contenuto entro i limiti configurati;
- nessun clamp persistente di `tau_input`;
- nessun anti-windup persistente;
- tracking `actual - served` preservato;
- imitazione `served - target` migliorata;
- rollout lunghi completi senza instabilita GRF.

Le bande da integrare e confrontare restano:

| Banda | Interpretazione |
|---|---|
| `0-6 Hz` | cinematica utile e gait |
| `6-20 Hz` | transizione riferimento/outer |
| `20-40 Hz` | risonanze meccaniche SEA |
| `40-80 Hz` | chattering e armoniche intermedie |
| `80-140 Hz` | modo rapido e banda motor driver |
| `140-500 Hz` | contenuto numerico o impulsivo sotto Nyquist |

## Strategia di validazione

### Fase 1 - Reference model isolato

1. Applicare la stessa traiettoria nota come comando `q_cmd`.
2. Confrontare reference model attuale e nuovo modello C2.
3. Misurare `q_ref`, `qdot_ref`, `qddot_ref` e jerk.
4. Verificare continuita ai confini dei segmenti.
5. Verificare attenuazione effettiva della riga a `100 Hz`.

### Fase 2 - Oracle OpenSim

1. Mantenere policy rate a `100 Hz`.
2. Usare un comando imitativo noto.
3. Confrontare `fc=4 Hz` e `fc=6 Hz`.
4. Misurare tracking, stress SEA e GRF.
5. Tarare jerk limit senza degradare l'imitazione.

### Fase 3 - Training V4

1. Rimuovere i tre knot policy.
2. Addestrare da zero con singolo comando posizione per giunto.
3. Mantenere inizialmente `100 Hz` e `gamma=0.95`.
4. Eseguire smoke training prima del run lungo.
5. Valutare altre frequenze policy solo dopo la validazione del riferimento
   C2.

## Soluzione e decisioni raggiunte

- [x] Il nuovo report frequenziale non invalida la separazione tra policy rate
      e served-reference bandwidth.
- [x] Il motor driver PI non deve essere considerato un filtro protettivo
      contro componenti `20-100 Hz`.
- [x] Il reference model e il principale elemento di separazione frequenziale.
- [x] La policy a `100 Hz` viene mantenuta come prima configurazione V4.
- [x] I tre knot futuri vengono considerati una complessita non necessaria per
      il V4.
- [x] La banda reale iniziale raccomandata per il nuovo reference model e circa
      `4 Hz`.
- [x] Il nuovo reference model deve essere C2, del terzo ordine e jerk-limited.
- [x] Dopo la validazione a `4 Hz`, confrontare una variante a `6 Hz`.

## File analizzati

- `reports/user/2026-06-14_analisi_frequenziale_configurazione_pi.md`
- `reports/user/2026-06-13_progettazione_training_imitativo_v4_frequenze_reference_model_c2.md`
- `reports/user/2026-06-01_filtro_riferimento_6hz_lato_simulatore.md`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `config.py`
- `simulation_runner.py`
- `prosthesis_controller.py`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`

## File modificati

- `reports/user/2026-06-14_aggiornamento_progettazione_v4_dopo_analisi_frequenziale.md`
  - nuovo report di aggiornamento progettuale.

Nessun file Python, plugin C++, modello `.osim` o parametro di training e stato
modificato durante questa attivita.

## Test e verifiche eseguite

Verifiche read-only:

- lettura e confronto della nuova analisi frequenziale;
- verifica delle formule del reference model corrente;
- verifica della banda effettiva `-3 dB` del modello corrente;
- confronto asintotico del trasferimento verso `qddot_ref` per modelli di
  secondo e terzo ordine;
- confronto delle bande outer, risonanze SEA e banda del motor driver;
- verifica dei parametri correnti in configurazione e ambiente RL.

Non sono stati eseguiti nuovi rollout o training.

## TODO

### Implementazione V4

- [ ] Implementare reference model del terzo ordine C2/jerk-limited.
- [ ] Esporre il tipo di reference model e la banda reale in
      `training_cfg.yaml`, configurazione risolta e rollout.
- [ ] Esporre limiti jerk separati per knee e ankle.
- [ ] Ridurre l'azione policy a un comando posizione per giunto.
- [ ] Rimuovere la generazione e derivazione dei tre knot futuri.
- [ ] Esporre all'attore lo stato necessario del reference model.
- [ ] Aggiungere logging esplicito di `q_cmd_raw`, `q_ref_served`,
      `qdot_ref_served`, `qddot_ref_served` e jerk.
- [ ] Aggiungere loss diretta `served reference vs imitation target`.
- [ ] Rimuovere la phase di episodio dalle osservazioni actor.
- [ ] Correggere l'inizializzazione SEA prima di abilitare `random_init=true`.

### Validazione

- [ ] Confrontare reference model attuale e C2 con la stessa traiettoria nota.
- [ ] Validare continuita C2 ai confini dei segmenti.
- [ ] Verificare attenuazione reale della riga a `100 Hz` in `qddot_ref`.
- [ ] Confrontare `fc=4 Hz` e `fc=6 Hz` tramite oracle OpenSim.
- [ ] Tarare i limiti jerk per knee e ankle.
- [ ] Verificare energia nelle bande `20-40 Hz` e `80-140 Hz`.
- [ ] Verificare clamp, anti-windup, torque error e stress motore.
- [ ] Eseguire smoke training V4 da zero mantenendo policy a `100 Hz`.
- [ ] Valutare `20/50/100 Hz` solamente dopo il superamento dei gate C2.

