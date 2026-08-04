# Detector virtuale heel/toe nella FSM: implementazione e validazione

Data: 2026-07-21

## Sintesi esecutiva

È stato implementato un detector protesico semplice basato su due sensori
virtuali distinti, uno nella regione del tallone e uno nella regione della
punta. I due sensori:

- misurano soltanto il carico normale locale;
- non generano GRF e non applicano forze al modello;
- alimentano la FSM del gait cycle già esistente;
- non introducono una seconda macchina a stati;
- non modificano plugin SEA, controllore SEA, reward, corridor o schema actor.

L'implementazione è intenzionalmente ancora **sperimentale e non promossa**.
Sono disponibili tre modalità:

| Modalità | Eventi che comandano la FSM | Scopo |
|---|---|---|
| `legacy_events` | detector aggregato storico | comportamento corrente e default |
| `shadow` | detector storico | acquisizione heel/toe senza interferenza |
| `two_sensor` | due sensori heel/toe | nuovo metodo sperimentale attivo |

Il risultato tecnico deve essere distinto su tre livelli:

1. **Logica heel/toe e FSM: PASS semantico.** Le regole sono corrette nei test
   sintetici e non introducono una seconda macchina a stati.
2. **Timing del detector: PASS soltanto in-scope sulla finestra breve a 10 ms,
   non ancora congelabile.** La replica a 1 ms fallisce marginalmente; soprattutto,
   il protocollo full-span su 23 cicli development fallisce sia con le soglie
   5/2 N sia con una griglia a soglia bassa. Validation e sealed non sono stati
   aperti.
3. **Checkpoint `best` congelato con detector attivo: incompatibile allo stato
   attuale.** Il rollout nominale termina al passo 198 per penetrazione GRF di
   25,98 mm, oltre il gate di 25 mm.

La validazione del detector non richiede training: nessun actor, checkpoint,
reward o PPO è stato usato nei replay prescribed/forward-state o negli sweep.
Il training diventa una domanda successiva e separata, relativa alla
compatibilità della policy con un detector che dovrà prima essere congelato.

Questo non cancella la robustezza ottenuta finora: in modalità `shadow` il
checkpoint `best` riproduce esattamente il rollout storico, numero per numero.
La configurazione di training resta esplicitamente su `legacy_events`.

## Problema affrontato

Il detector storico identificava alcuni HS usando l'evento del contatto
aggregato del piede. Nei trace osservati, tre HS successivi al primo cadevano
mentre:

- il sensore virtuale del tallone era sostanzialmente scarico;
- il carico era presente soprattutto sulla punta;
- il ginocchio served era ancora molto flesso.

I tre casi più evidenti del rollout nominale del checkpoint `best` erano:

| HS legacy [s] | carico tallone [N] | carico punta [N] | ginocchio served [deg] |
|---:|---:|---:|---:|
| 15,4889 | ~0 | 15,51 | -45,44 |
| 16,8799 | ~0 | 16,49 | -43,33 |
| 18,5089 | ~0 | 15,69 | -46,39 |

Il comportamento era quindi coerente con un contatto forefoot-first
interpretato impropriamente come HS. Il TO storico risultava invece molto più
coerente con la cinematica osservata.

## Soluzione implementata

### Due canali detector-only

Il percorso online GRF espone ora, oltre agli aggregati left/right già
esistenti, canali espliciti per:

```text
left_heel
left_toe
right_heel
right_toe
```

Ogni canale contiene nome del componente, lato, regione, carico normale,
penetrazione geometrica e flag di contatto. I valori non finiti causano un
errore fail-closed prima del clamp.

I componenti del profilo detector continuano a essere creati come
`appliesForce = false`. Il loro output è dunque un segnale di misura: non entra
nell'equazione dinamica e non può sostituire o generare la GRF.

Il profilo attuale usa posizioni ragionevoli sul piede virtuale AB06, senza
pretendere di rappresentare un futuro dispositivo reale:

| Sensore | Frame | Posizione locale [m] | Raggio [m] |
|---|---|---|---:|
| heel sinistro | `/bodyset/foot_l` | `[-0.10141, -0.03282, 0.01400]` | 0,02291 |
| toe sinistro | `/bodyset/foot_l` | `[0.06914, -0.07401, -0.04686]` | 0,02291 |

La geometria del profilo resta marcata `preliminary`: è sufficiente per questa
validazione simulativa, ma non è una specifica hardware.

### Integrazione nella FSM esistente

La classe `ProstheticPhaseFSM` resta l'unica autorità sul gait cycle. Sono
stati aggiunti alla stessa classe due latch di contatto con:

```text
soglia ON  = 5 N
soglia OFF = 2 N
dwell      = 30 ms
```

Le due soglie realizzano un'isteresi semplice; il dwell richiede che la
condizione persista prima di confermare il cambio di contatto.

La logica implementata è:

- il detector si arma per un nuovo HS soltanto dopo che heel e toe sono entrambi
  stabilmente scarichi;
- il solo contatto della punta produce la diagnostica `forefoot_first`, ma non
  può produrre HS;
- se in seguito si attiva stabilmente il tallone, l'onset del tallone diventa il
  candidato HS;
- il candidato passa attraverso gli handler e i gate già esistenti della FSM;
- il TO viene emesso soltanto quando heel e toe sono entrambi stabilmente OFF;
- un reset in piena stance entra nello stato stance esistente senza inventare
  un HS, senza credito e senza incrementare i contatori;
- il primo TO dopo tale reset chiude soltanto il segmento parziale non
  accreditato.

Gli stati numerici della FSM, gli handler HS/TO, i timeout, il journal delle
transizioni e i gate anti-fake-cycle non sono stati sostituiti.

### Contratto actor preservato

Le otto feature della FSM già presenti nell'osservazione actor mantengono nome,
ordine e significato strutturale. L'intero actor resta di 35 feature e
l'osservazione completa di 84 feature in tutte le modalità.

I carichi heel/toe e la diagnostica dettagliata non sono stati aggiunti
all'actor: sono disponibili nel trace e nell'`info` di rollout. Questo evita di
rendere incompatibili i checkpoint esistenti.

Quando `two_sensor` è attivo, anche i pulse HS/TO e il gait clock protesico
vengono aggiornati soltanto dalle transizioni realmente accettate dalla FSM.
Gli eventi legacy sinistri vengono rifiutati, impedendo una fusione ambigua dei
due detector.

### Protezioni operative

- Il default generale e il file ex-novo restano `legacy_events`.
- `shadow` e `two_sensor` sono opt-in espliciti.
- Le modalità nuove richiedono `segment_duration <= sensor_dwell_s`; la
  configurazione validata usa 10 ms.
- L'ambiente fallisce prima del primo segmento se non trova esattamente un
  sensore `left_heel` e uno `left_toe`.
- Le soglie sono registrate nei summary di training e rollout.
- I trace separano eventi legacy, edge grezzi, candidati sensoriali e
  transizioni accettate.

## Strategia di test e validazione

### 1. Test unitari della logica

Sono stati verificati:

- configurazioni e input invalidi fail-closed;
- identità dello schema actor nelle tre modalità;
- retrocompatibilità della modalità legacy;
- HS da tallone e TO da entrambi i sensori OFF;
- punta sola incapace di generare HS;
- heel successivo a un forefoot-first capace di generare il vero HS;
- contatto simultaneo senza duplicazioni;
- isteresi, dwell e re-contact in stance;
- bootstrap a metà stance senza evento sintetico o reward;
- shadow non interferente;
- disponibilità degli edge grezzi anche quando lo stato legacy maschera il
  controfattuale.

Esito: **11/11 PASS**.

Il data path dei sensori, il core online GRF e il matching degli eventi hanno
superato complessivamente **23 test**. Le regressioni della reward e della
morfologia hanno superato rispettivamente **32/32**, **11/11** e **19/19** test.
Lo smoke test della FSM nel vero ambiente OpenSim è passato.

### 2. Replay indipendente sui dati prescribed

Il validatore in `validation/` ricostruisce i carichi separati heel/toe lungo
l'IK prescribed e li passa alla stessa `ProstheticPhaseFSM` usata in
produzione. Mantiene tutti i parametri runtime correnti e forza soltanto
`event_source=two_sensor`, perché il training corrente resta intenzionalmente
su `legacy_events`. Ginocchio e caviglia prescribed sono forniti ai gate
cinematici della FSM.

Il riferimento temporale primario è esclusivamente la GRF prescribed indicata
dall'`ExternalLoads`. Le etichette EPIC `gcLeft` sono usate soltanto come audit
secondario hash-pinned; i TO `gcLeft`/`gcRight`, essendo identici, non sono stati
usati come riferimento side-specific.

Risultato al passo runtime di 10 ms:

| Evento | Reference | Predetti | Precision | Recall | Errore massimo |
|---|---:|---:|---:|---:|---:|
| HS | 5 | 5 | 1,00 | 1,00 | 47,93 ms |
| TO | 4 | 4 | 1,00 | 1,00 | 76,85 ms |

Sono stati osservati i quattro cicli completi attesi, senza evento invalido,
timeout, transizione sensoriale non accettata o HS privo del carico heel.
Esito del gate primario: **PASS**.

La validazione è stata poi estesa dalla sola identificazione degli eventi alla
classificazione sample-by-sample delle fasi. Il riferimento causale è:

```text
stance <=> Fy prescribed sinistra > 20 N
swing  <=> Fy prescribed sinistra <= 20 N
```

Il tratto iniziale già in stance è riportato ma escluso dal gate stretto, perché
il relativo HS precede l'inizio della finestra. Dal primo HS completo in poi:

| Metrica di fase a 10 ms | Risultato |
|---|---:|
| campioni corretti nel tratto stretto | 692 / 706 |
| agreement grezzo stance/swing | 98,02% |
| agreement lontano dalle finestre HS/TO | 570 / 570 = 100% |
| mismatch fuori dalle finestre già fissate | 0 |
| transizioni che mantengono lo stato fino alla successiva | 9 / 9 |

I 14 mismatch grezzi sono tutti confinati attorno ai cambi di fase. Non è stata
aggiunta una nuova soglia post-hoc: le sole finestre ammesse derivano dai gate
temporali già fissati, 50 ms per HS e 80 ms per TO, più il dwell causale di
30 ms.

Sui quattro cicli completi, gli errori massimi sono:

| Quantità | Errore massimo |
|---|---:|
| durata ciclo | 10,99 ms |
| durata stance | 39,98 ms |
| durata swing | 40,89 ms |
| stance duty factor | 2,43 punti percentuali |

È importante distinguere due tempi. `event_time_s` registra l'onset fisico del
crossing heel/toe; `confirmed_time_s` è il momento, 30 ms dopo, in cui il dwell
è soddisfatto e la FSM cambia causalmente stato. Il primo misura l'accuratezza
fisica attribuita all'evento; il secondo descrive ciò che diventa disponibile
alla policy.

È stato eseguito anche un test di sensibilità non gating a 1 ms. Conteggi,
ordine e semantica restano corretti: 5 HS, 4 TO, quattro cicli validi e nessuna
anomalia FSM. Tuttavia:

- l'ultimo HS anticipa il riferimento di 53,93 ms, superando il limite di
  50 ms di 3,93 ms;
- il primo TO anticipa il riferimento di 80,85 ms, superando il limite di
  80 ms di 0,85 ms.

I valori 3,93 ms e 0,85 ms sono gli **sforamenti delle tolleranze**, non gli
errori totali, che sono rispettivamente 53,93 ms e 80,85 ms. Poiché il matcher
è one-to-one e stretto, un evento appena fuori finestra diventa sia una
predizione non abbinata sia un riferimento non abbinato. Ne risultano
precision/recall 0,80 per HS e 0,75 per TO nonostante conteggi e ordine siano
esatti.

La sensitivity conserva invece un phase-state agreement grezzo del 97,80%,
un agreement settled del 100%, zero mismatch fuori finestra e gate semantico e
di fase entrambi PASS. È quindi classificata precisamente come **timing FAIL,
semantic PASS, phase-state PASS**.

Il passaggio da 10 ms a 1 ms anticipa gli onset di altri 0–7 ms: la griglia più
fitta localizza prima il crossing del sensore virtuale. Il bias di fondo non è
un bug del dwell, ma deriva dal confronto tra due definizioni fisiche diverse:
carico locale heel/toe a soglia bassa contro crossing a 20 N della GRF
prescribed. Il risultato runtime è un PASS reale, ma con margini temporali
ridotti: 2,07 ms per il peggior HS e 3,15 ms per il peggior TO.

Questa validazione dimostra coerenza sulla sequenza AB06 usata. Non dimostra
ancora generalizzazione indipendente ad altri trial o modelli, anche perché il
profilo geometrico preliminare dichiara una calibrazione originaria rispetto
alla GRF prescribed.

### 2a. Detector e policy sono due validazioni diverse

La domanda «il detector riconosce correttamente HS, TO e stance/swing?» non
richiede training. Si risolve riproducendo i due carichi virtuali attraverso la
FSM e confrontando eventi e fase con una reference prescribed indipendente dal
segnale detector durante il replay.

La domanda «il checkpoint addestrato con `legacy_events` resta stabile quando
riceve la nuova temporizzazione?» richiede invece un rollout della policy ed
eventualmente, soltanto dopo il freeze del detector, un adattamento.

Il fallimento fisico del checkpoint `best` con `two_sensor` non invalida quindi
il detector. Dimostra che la coppia `checkpoint best + nuovo detector` non è
compatibile allo stato attuale. Viceversa, un PASS del detector non renderebbe
automaticamente deployable quella coppia.

### 2b. Come era stato validato il detector storico

Il detector aggregato fu validato senza PPO mediante
`validation/validate_online_grf_events.py`:

- AB06, finestra 11,99–21,0 s e cadenza 1 ms;
- reference sinistra `Fy prescribed > 20 N`;
- matching one-to-one con 50 ms per HS e 80 ms per TO;
- soglia bassa 15 N, conferma 120 N, durata minima 30 ms e media mobile causale
  di 100 ms;
- replay IK e successiva conferma sugli stati forward dello stesso trial.

Il profilo corrente e quello storico hanno lo stesso SHA-256
`61ea948a...fd99e`: geometria e contact law sono byte-per-byte identiche. Il
confronto old/new cambia quindi la logica detector, non il profilo fisico.

La procedura storica era una valida verifica funzionale, ma non un holdout:
lo sweep e il risultato IK usavano la stessa finestra, il profilo era già
`calibrated_against_prescribed_grf` e gli stati forward seguivano lo stesso
trial. Inoltre il segnale aggregato non poteva verificare il requisito
semantico «HS soltanto da tallone». Infine, il `legacy_events` corrente non
replica esattamente quel candidato storico: risolve a circa 20/20 N, 50 ms e
nessuno smoothing.

Per un confronto omogeneo viene escluso l'HS artificiale a 11,99 s: la finestra
inizia già in stance. Il detector storico lo contava, mentre il nuovo esegue un
bootstrap parziale senza inventare né premiare un evento.

| Detector | Input | Passo | HS errore massimo | TO errore massimo | Esito timing |
|---|---|---:|---:|---:|---|
| aggregato storico | IK prescribed | 1 ms | 13 ms | 26 ms | PASS |
| `two_sensor` 5/2 N | IK prescribed | 1 ms | 53,93 ms | 80,85 ms | FAIL marginale |
| aggregato storico | forward states | 1 ms | 13 ms | 27 ms | PASS |
| `two_sensor` 5/2 N | forward states | 1 ms | 53,93 ms | 80,85 ms | FAIL marginale |
| `two_sensor` 5/2 N | IK prescribed | 10 ms | 47,93 ms | 76,85 ms | PASS runtime |
| `two_sensor` 5/2 N | forward states | 10 ms | 47,93 ms | 76,85 ms | PASS runtime |

### 2c. Replay sugli stessi forward states storici

Il nuovo validatore ricostruisce i carichi heel/toe da
`results/sim_output_states.sto`, lo stesso file usato nella verifica storica,
e attraversa la vera FSM. Stati e `run_status` sono hash-pinned.

A 10 ms ottiene 5/5 HS e 4/4 TO, quattro cicli, zero invalidi e 100% di
agreement fuori dalle finestre di transizione. A 1 ms conserva conteggi,
ordine, semantica e fase, ma riproduce il FAIL marginale di 53,93/80,85 ms.

Le metriche di contatto devono restare separate:

| Segnale, forward states a 1 ms | Precision | Recall | F1 | IoU |
|---|---:|---:|---:|---:|
| aggregato storico smussato > 15 N | 0,9051 | 0,9872 | 0,9443 | 0,8945 |
| unione dei latch heel/toe | 0,8862 | 0,9734 | 0,9278 | 0,8653 |
| stato `STANCE_AFTER_HS` della FSM | 0,9900 | 0,9734 | 0,9816 | 0,9640 |

Il primo è un carico aggregato smussato, il secondo un'unione di due latch, il
terzo lo stato confermato della FSM: non sono la stessa quantità. Il file
forward non è inoltre un gait indipendente: segue lo stesso prescribed con una
piccola perturbazione d'integrazione (RMSE 0,137° al ginocchio e 1,134° alla
caviglia). È un test di consistenza del data path, non di generalizzazione.

### 2d. Protocollo full-span detector-only

Per non ereditare il selection bias storico sono stati fissati tre blocchi
cronologici prima del test:

| Blocco | Finestra | Cicli completi | Uso |
|---|---:|---:|---|
| development | 11,99–50 s | 23 | scelta del candidato |
| validation | 50–100 s | 51 | candidato congelato + baseline |
| sealed | 100–155,045 s | 48 | apertura unica dopo PASS validation |

Nessun checkpoint, actor, reward o PPO è importato dall'harness. Il dwell resta
fisso a 30 ms e i due carichi sono calcolati una sola volta per candidato: le
soglie non generano GRF e non alterano la dinamica.

#### V1: soglie 5–20 N

La griglia preregistrata di 20 coppie fallisce già in development. La baseline
5/2 N conserva esattamente 24 HS, 23 TO e 23 cicli, senza invalidi o timeout,
ma ha:

- errore onset HS massimo 280,12 ms;
- errore onset TO massimo 92,52 ms;
- precision/recall strette 0,7917/0,7917;
- 60 mismatch di fase fuori dalle finestre fissate;
- F1/IoU dello stato FSM pari a 0,9599/0,9228.

Il problema non è un errore del matcher. In tre cicli il riferimento GRF e le
etichette EPIC concordano sull'HS, mentre il tallone virtuale porta un carico
reale ma inferiore a 5 N:

| HS prescribed [s] | heel [N] | toe [N] | ritardo crossing heel 5 N |
|---:|---:|---:|---:|
| 37,26985 | 0,70 | 63,58 | 233 ms |
| 38,85601 | 1,54 | 101,45 | 207 ms |
| 43,52988 | 1,72 | 66,24 | 271 ms |

#### V2 development: soglie basse

Senza aprire validation o sealed è stata preregistrata una seconda griglia:
ON 0,5–3 N, OFF 0,01–0,25 N, più il comparatore 5/2 N. Anche V2 fallisce il
gate onset; nessun candidato viene congelato.

Il miglior punto diagnostico è 0,5/0,01 N, a pari timing con gli altri OFF
bassi:

| Quantità | Risultato |
|---|---:|
| HS / TO / cicli | 24 / 23 / 23, tutti esatti |
| invalidi / timeout / candidati rifiutati | 0 / 0 / 0 |
| onset HS massimo | 72,96 ms, limite 50 ms |
| onset TO massimo | 92,52 ms, limite 80 ms |
| precision / recall strette | 0,50 / 0,50 |
| mismatch fase fuori finestra | 2 |
| F1 / IoU stato FSM | 0,9727 / 0,9469 |
| errore massimo al tempo di conferma | 42,96 ms HS / 62,52 ms TO |

Precision e recall scendono pur con conteggi e ordine esatti perché il matcher
stretto considera non abbinati gli onset appena oltre le finestre. I tempi di
conferma sono una diagnostica, non il gate V2: sostituirli post-hoc agli onset
avrebbe cambiato la definizione dopo aver visto i risultati.

La conclusione full-span è quindi **development FAIL**. Validation e sealed
non sono mai stati letti o creati. Il detector resta sperimentale e le soglie
non vengono promosse nel runtime.

Il prossimo design deve decidere esplicitamente, prima di un nuovo protocollo,
se l'evento online sia ancorato all'onset fisico retrodatato oppure al tempo di
conferma causale realmente disponibile alla FSM. Le alternative semplici da
valutare sono: usare la conferma come timestamp operativo; mantenere l'onset ma
aggiungere un filtro causale per sensore; oppure rivedere la geometria/contact
law preliminare. Questa scelta modifica la semantica temporale del detector e
non viene assunta automaticamente in questo report.

### 3. Rollout `best` in shadow

Il checkpoint `rl_module_best` è stato eseguito per 500 step in modalità
`shadow`. Il risultato coincide esattamente con il rollout storico nominale:

| Metrica | Storico | Shadow |
|---|---:|---:|
| step | 500 | 500 |
| return | 52,4269395298 | 52,4269395298 |
| penetrazione massima | 24,2174 mm | 24,2174 mm |
| reserve massima | 493,4498 Nm | 493,4498 Nm |
| HS / TO / cicli | 4 / 3 / 3 | 4 / 3 / 3 |
| eventi invalidi | 1 | 1 |
| action clip | 0 | 0 |
| fine episodio | time limit | time limit |

Questo dimostra che sensori, acquisizione e diagnostica non modificano il
comportamento fisico quando restano in shadow.

Il replay controfattuale dei carichi dello stesso trace attraverso la FSM
`two_sensor` produce i veri HS ai tempi:

| HS two-sensor [s] | carico heel all'onset [N] | ginocchio served [deg] |
|---:|---:|---:|
| 15,6669 | 10,97 | -18,91 |
| 17,1569 | 5,19 | -11,09 |
| 18,7069 | 8,09 | -17,23 |

Tutti gli otto hard-check del trace passano: schema completo, nessun HS da toe
solo, HS con onset heel causale, TO con entrambi i latch OFF e ordinamento
HS/TO valido, sia per i candidati sia per le transizioni accettate.

### 4. Rollout `best` con `two_sensor` attivo

Il rollout nominale con il detector nuovo autoritativo mostra:

| Metrica | Risultato |
|---|---:|
| step eseguiti | 198 / 500 |
| return | -5,8691 |
| penetrazione massima | 25,9772 mm |
| gate hard penetrazione | 25 mm |
| reserve massima | 459,8983 Nm |
| HS / TO / cicli | 1 / 1 / 0 |
| eventi invalidi | 0 |
| motivo terminazione | `grf_penetration` |

La semantica del detector passa ancora tutti gli otto hard-check, ma il gate
fisico fallisce. Il detector non è quindi promosso.

L'evidenza diretta è una distribuzione diversa delle feature temporali: il
checkpoint è stato addestrato con gli eventi legacy. Cambiare il momento di HS
modifica pulse, stato FSM, progresso del ciclo e gait clock, pur lasciando
invariati dimensione e ordine del vettore. Le azioni divergono e la policy
congelata non è robusta allo switch. È quindi ragionevole inferire che l'actor
si sia accoppiato anche alla temporizzazione legacy, ma il rollout da solo non
permette di attribuire causalmente il fallimento a una singola feature.

Non sono stati introdotti finti HS al reset, progressi artificiali o soglie
rilassate per forzare il checkpoint a passare: avrebbero nascosto il problema
invece di risolverlo.

### 5. Matrice di robustezza preregistrata

È stato preparato e validato un harness no-clobber per 72 casi di sviluppo:

```text
2 checkpoint × 3 modalità × 3 start ×
(1 deterministico + 3 seed stocastici) = 72 rollout
```

Sono fissati hash dei checkpoint e degli artefatti storici, soglie, cadenza di
10 ms e seed di sviluppo 123–125. I seed held-out 126–128 sono sigillati e non
possono essere generati dall'harness di sviluppo. Il dry-run è `READY` e tutti
i contratti passano.

La matrice completa non è stata lanciata dopo il fallimento del primo smoke
nominale attivo: il gate di promozione era già impossibile da superare. Eseguire
altri 71 rollout non avrebbe potuto rendere promuovibile il checkpoint e
avrebbe consumato i test senza cambiare la decisione. La matrice resta pronta
per il checkpoint adattato successivo.

## Decisione di validazione

| Oggetto | Stato |
|---|---|
| acquisizione heel/toe detector-only | validata |
| regole HS/TO nella FSM esistente | validate |
| actor schema e retrocompatibilità legacy | validate |
| uso shadow sul checkpoint corrente | validato, non interferente |
| finestra breve prescribed a cadenza runtime | PASS in-scope |
| finestra breve / forward sensitivity 1 ms | FAIL temporale marginale |
| full-span V1 5–20 N, development | FAIL |
| full-span V2 0,5–3 N, development | FAIL; nessun candidato congelato |
| full-span validation 50–100 s | non aperta per fail-closed |
| full-span sealed 100–155,045 s | non aperta per fail-closed |
| checkpoint `best` congelato con detector attivo | compatibility/physical FAIL |
| training necessario per validare il detector | no |
| promozione di `two_sensor` a default/training | respinta |

La conclusione corretta non è che il detector abbia cancellato la robustezza
della simulazione. La robustezza storica è preservata perché il percorso
legacy è invariato e lo shadow è numericamente identico. Sono invece due i
blocchi ancora aperti: il detector non ha superato il freeze full-span e il
checkpoint corrente non è compatibile con la modalità sperimentale. Non è
quindi ancora sensato iniziare training sul nuovo detector: prima va risolta e
congelata la sua semantica temporale.

## File modificati o introdotti

### Runtime e configurazione

- `online_grf.py`: esposizione additiva dei canali detector heel/toe e controlli
  sui valori non finiti;
- `simulation_runner.py`: propagazione dei canali senza modificarne la funzione
  detector-only;
- `Trajectory Generator/prosthetic_phase_fsm.py`: latch, isteresi, dwell e
  mapping dei due sensori sugli stati esistenti;
- `Trajectory Generator/osim_trj_cmc_like.py`: selezione legacy/shadow/two-sensor,
  fail-fast del setup e separazione degli stream evento;
- `Trajectory Generator/baseline_MLP/training_config.py`;
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`;
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`;
- `Trajectory Generator/baseline_MLP/rollout_eval.py`.

Gli ultimi quattro file registrano e propagano modalità e soglie. Il file
ex-novo resta intenzionalmente su `legacy_events`.

### Test e validazione

Tutti i nuovi script di validazione risiedono in `validation/`:

- `test_prosthetic_phase_fsm_two_sensor.py`;
- `test_detector_sensor_data_path.py`;
- `test_two_sensor_prescribed_phase_gate.py`;
- `validate_two_sensor_forward_states.py`;
- `test_two_sensor_forward_states.py`;
- `sweep_two_sensor_prescribed_thresholds.py`;
- `test_two_sensor_prescribed_threshold_sweep.py`;
- `two_sensor_prescribed_threshold_sweep_protocol.json`;
- `two_sensor_prescribed_threshold_sweep_protocol_v2.json`;
- `validate_two_sensor_rollout_trace.py`;
- `validate_two_sensor_prescribed_replay.py`;
- `heel_detector_robustness_ab.py`;
- `test_heel_detector_robustness_ab.py`;
- `heel_detector_robustness_ab_protocol.json`.

La suite mirata finale su FSM, data path, replay prescribed, forward-state e
protocollo threshold ha superato **30/30 test**. Sono stati eseguiti inoltre
`ruff`, `py_compile`, validazione JSON e `git diff --check`: tutti PASS. È stato
anche verificato esplicitamente che le directory V2 `validation/` e `sealed/`
non esistano, quindi i due blocchi non sono stati aperti accidentalmente.

## Artefatti principali

- shadow nominale e trace completo:
  `validation/heel_detector_validation_runs/2026-07-21_best_nominal_shadow/`;
- rollout nominale attivo:
  `validation/heel_detector_validation_runs/2026-07-21_best_nominal_two_sensor/`;
- replay prescribed 10 ms:
  `validation/two_sensor_prescribed_replay_runs/2026-07-21_ab06_phase_runtime10ms_v7/`;
- sensitivity prescribed 1 ms:
  `validation/two_sensor_prescribed_replay_runs/2026-07-21_ab06_phase_sensitivity1ms_v5/`;
- forward states alla cadenza runtime:
  `validation/two_sensor_forward_states_runs/2026-07-21_results_sim_output_states_runtime10ms/`;
- forward states sensitivity nativa:
  `validation/two_sensor_forward_states_runs/2026-07-21_results_sim_output_states_native1ms_sensitivity/`;
- full-span V1 development FAIL:
  `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v1_report/development/`;
- full-span V2 development FAIL e plot diagnostici:
  `validation/two_sensor_prescribed_threshold_sweep_runs/2026-07-21_fullspan_v2_report/development/`.

Il plot più utile per la decisione onset/conferma è
`diagnostic_or_selected_event_errors.png` nella cartella V2: mostra gli onset
fuori tolleranza e, separatamente, i tempi causali di conferma entro tolleranza.

## TODO e prossimo esperimento

1. Scegliere esplicitamente la semantica temporale successiva: onset fisico
   retrodatato oppure conferma causale disponibile online. In alternativa,
   mantenere l'onset e testare un semplice filtro causale o una revisione della
   geometria preliminare.
2. Preregistrare la scelta in un nuovo protocollo e usare ancora soltanto il
   development 11,99–50 s. Non aprire validation/sealed finché il gate non
   passa senza tolleranze post-hoc.
3. Congelare soglie, dwell, semantica e hash prima di aprire validation e poi
   sealed una volta sola.
4. Soltanto dopo il freeze del detector, adattare o fine-tunare una copia del
   checkpoint mantenendo recuperabile il `best` corrente.
5. Ripetere lo smoke nominale con gli stessi gate fisici; se passa, eseguire la
   matrice preregistrata di 72 casi e infine il gate policy held-out.

Fino ad allora il nuovo detector deve restare disponibile per shadow e
diagnostica, ma non deve ancora pilotare un nuovo training né sostituire il
detector legacy nel percorso validato.

## Addendum V3 confirmed-time

Il TODO sulla semantica temporale è stato successivamente eseguito con il
protocollo V3. `confirmed_time_s` è diventato il timestamp primario e l'onset è
rimasto diagnostico. Il development e la sensitivity a 1 ms sono PASS con il
candidato `ON=0.50 N / OFF=0.25 N`, ma il holdout cronologico `50-100 s` è
FAIL per quattro HS tardivi e un successivo HS non confermato che porta la FSM
in timeout. Il blocco sealed `100-155.045 s` non è stato aperto.

Risultati, causa fisica e TODO aggiornati sono documentati in:

`reports/user/2026-07-21_detector_virtuale_v3_confirmed_time_holdout.md`.
