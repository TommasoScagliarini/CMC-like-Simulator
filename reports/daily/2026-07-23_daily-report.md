# Daily Report - 2026-07-23

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Separazione netta tra GRF primaria e detector heel/toe](../user/2026-07-23_separazione_grf_primaria_detector_two_sensor.md)
- [Warm start H0 con detector V13 e Morphology Corridor](../user/2026-07-23_warm_start_h0_v13_corridor_monitoraggio.md)

## Sintesi

La giornata ha separato in modo esplicito il carico fisico continuo della GRF
primaria dai segnali locali heel/toe del detector. Il detector era gia'
`appliesForce=false`, ma il runtime poteva preferire il suo aggregato per
`normal_force_bw` e `in_contact`; questo alterava FSM, observation e reward e,
con un detector left-only, poteva azzerare il lato destro privilegiato del
critic. Il routing e' stato corretto senza modificare geometria, reward,
checkpoint, plugin C++ o semantica SEA.

In parallelo e' stata resa non ambigua la CLI di inizializzazione PPO:
`--warm-start` ripristina il checkpoint H0 completo, `--warm-start-raw`
mantiene il vecchio trapianto actor-only e `--resume-from` resta il resume
tecnico generale. E' stato quindi monitorato un pilot H0 con V13 autoritativo e
Morphology Corridor `event_anchored` a peso `0.05`.

Quel pilot non costituisce evidenza promuovibile: era stato avviato prima della
correzione del routing, usa V13 non promosso dopo un sealed FAIL e cambia
contemporaneamente detector e reward. Al momento del report era arrivato
all'iterazione logica 36 senza crash, ma con episodi medi di circa 37 step,
contro circa 410 step del comparatore allineato, e con molte piu' terminazioni
per penetrazione. Il risultato resta diagnostico; non dimostra che la GRF
primaria sia errata e non permette di attribuire il degrado al solo detector o
al solo corridor.

## 1. Problema e correzione del routing dual-stream

Il comportamento precedente dell'environment era, in sostanza:

```text
se e' disponibile l'aggregato detector:
    usalo come sorgente di carico/contact
altrimenti:
    usa la GRF primaria
```

La separazione meccanica non era in discussione: le sfere detector non
applicavano forze al plant. Il difetto era nel consumo dei segnali. La lettura
aggregata delle sfere poteva sostituire la sorgente fisica per:

- `online_left_normal_grf_bw` e `online_left_in_contact`;
- evidenza continua di carico/contact della FSM;
- feature load/contact dell'actor e del critic;
- termini reward di supporto, overload e swing unloading;
- lato destro privilegiato del critic quando il profilo detector era
  left-only.

Il nuovo contratto runtime e':

| Segnale o funzione | Sorgente autorevole |
| --- | --- |
| Forze applicate e penetrazione | GRF primaria |
| `normal_force_bw` e `in_contact` | GRF primaria |
| Feature continue load/contact actor e critic | GRF primaria |
| Evidenza continua load/contact della FSM | GRF primaria |
| Carichi locali heel/toe | detector virtuale |
| Candidati e transizioni HS/TO two-sensor | detector + FSM |

In `Trajectory Generator/osim_trj_cmc_like.py`, il vecchio
`_phase_grf_sides()` e' stato sostituito da
`_physical_online_grf_sides()`, che restituisce soltanto `_online_grf` e non
effettua fallback sul detector. `_update_phase_fsm()` usa la GRF primaria per
carico/contact e conserva heel/toe dal detector; `_online_gait_info()` usa la
GRF primaria per entrambi i lati.

In `simulation_runner.py`, l'aggregato detector resta utilizzabile soltanto
come sorgente event-only sui lati realmente presenti. Un detector sinistro non
puo' piu' azzerare il flusso destro: il lato assente conserva la GRF primaria.

Forma, ordine e dimensionalita' delle observation sono invariati. Non sono
stati modificati plugin C++, comandi SEA, profili detector, corridor, reward o
checkpoint.

## 2. Strategia di verifica del routing

I test mettono deliberatamente in contraddizione GRF primaria e detector. I
gate chiudono se il detector torna a essere usato come fallback o come sorgente
del segnale fisico continuo. Sono stati verificati:

- invarianza delle feature load/contact rispetto a carico, posizione e
  copertura laterale del detector;
- GRF primaria alla FSM per `normal_force_bw` e `in_contact`, con heel/toe
  ancora provenienti dal detector;
- assenza di fallback silenzioso quando manca la GRF primaria;
- conservazione della sorgente primaria destra con detector left-only.

La correzione non crea incompatibilita' strutturale con H0, perche' lo schema
observation non cambia. Cambia pero' la distribuzione di alcune feature
rispetto ai run che avevano consumato il routing errato; la compatibilita'
comportamentale di H0 richiede quindi un nuovo frozen-policy a zero update.

## 3. Inizializzazione H0 e pilot V13/corridor

La CLI di `train_ppo_mlp.py` distingue ora:

- `--warm-start`: restore RLlib completo da H0, inclusi actor, critic,
  optimizer, stato PPO e contatori;
- `--warm-start-raw`: trapianto actor-only in un Algorithm nuovo;
- `--resume-from`: ripresa generale di un checkpoint arbitrario.

Il checkpoint canonico usato da `--warm-start` e':

```text
validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

Il pilot ha riutilizzato lo snapshot risolto del 15 luglio, mantenendo 12
EnvRunner, tre start, interleaving esatto, batch reale da 4608 step, minibatch
512, una epoch PPO, learning rate `5e-7`, clip `0.05`, KL coefficient `1.0` e
`freeze_logstd`. Le sole variazioni dichiarate erano V13 in modalita'
`two_sensor` e corridor `event_anchored` con peso `0.05`.

Al momento del monitoraggio:

| Evidenza | Stato |
| --- | --- |
| Aggiornamenti logici completati | fino a iterazione 36 |
| `checkpoint_last` | iterazione 36 |
| `checkpoint_best` | iterazione 24 |
| Crash, timeout o restart | nessuno osservato |
| Bilanciamento esatto e gate KL | PASS |
| Return medio a iterazione 36 | `-2.437631` |
| Durata episodio media | `37.10` step |
| KL massimo | `0.000408`, sotto il limite `0.01` |

Il comparatore senza V13/corridor, alla stessa iterazione, aveva return medio
`32.783764` e durata `410.41` step. Il degrado era gia' visibile nel primo
batch raccolto da H0. Poiche' il corridor non modifica direttamente dinamica o
terminazioni, il dato indica una reale incompatibilita' di comportamento/FSM,
ma il disegno confonde detector e reward e non consente attribuzione causale.

## 4. Decisione e scope chiuso

```text
routing primary GRF / detector       = CORRETTO E TESTATO
detector appliesForce                = false, INVARIATO
observation layout                   = INVARIATO
semantica SEA / plugin C++           = INVARIATA
V13 promotion                        = NO, sealed FAIL storico
pilot V13 + corridor                 = DIAGNOSTICO NON PROMOVIBILE
compatibilita' comportamentale H0    = NON ANCORA DIMOSTRATA
causa detector vs corridor           = NON ATTRIBUIBILE
```

Il training gia' avviato non e' stato fermato o riavviato durante la correzione.
Worker gia' importati e worker eventualmente riavviati avrebbero potuto
consumare versioni diverse del modulo; anche per questo il run non puo' essere
usato come evidenza scientifica della nuova separazione.

## File modificati

### Routing e regressioni dual-stream

- `Trajectory Generator/osim_trj_cmc_like.py`
- `simulation_runner.py`
- `validation/test_detector_sensor_data_path.py`

### Warm start, resume e configurazione training

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/commands.txt`
- `validation/test_training_resume.py`
- `validation/validate_training_config.py`

## Test e verifiche

Per il routing dual-stream:

- `15` test data path detector PASS;
- `17` test FSM two-sensor PASS;
- `7` test online GRF core PASS;
- `1` test environment prescribed FSM PASS;
- `32` test reward PASS;
- totale mirato dichiarato: `72 PASS`;
- smoke OpenSim prescribed CMC-like FSM, `py_compile`, Ruff e
  `git diff --check`: PASS.

Per warm start e training:

- `45` test mirati PASS per parser, resume, warm-start actor-only, preflight e
  sampling bilanciato;
- smoke `validate_training_config.py`, compilazione Python,
  `git diff --check` e CLI `--help`: PASS;
- checkpoint H0 verificato come checkpoint RLlib valido;
- processi, watchdog, milestone, checkpoint e metriche monitorati fino
  all'iterazione 36.

I conteggi delle due suite sono riportati separatamente perche' i report non
attestano che siano disgiunti.

## TODO chiusi il 2026-07-23

- [x] Correggere il routing runtime affinche' carico continuo e contact
  provengano sempre dalla GRF primaria, senza fallback sul detector.
- [x] Aggiungere regressioni dual-stream, incluso il caso detector left-only,
  mantenendo i due sensori detector-only.
- [x] Eliminare l'ambiguita' tra restore completo H0, warm start actor-only e
  resume tecnico mediante flag CLI distinti.

## TODO aperti e propagati

### Detector e compatibilita' H0

- [ ] Creare ogni nuova revisione del detector come ramo separato, senza
  sovrascrivere o reinterpretare V9/V13.
- [ ] Portare l'errore HS sotto `50 ms` sia sulla finestra iniziale sia sul
  development, conservando TO, conteggi, ordine, continuita', clear causale,
  assenza di invalidi e coerenza `10 ms / 1 ms`.
- [ ] Motivare fisicamente e preregistrare ogni nuova revisione; evitare altro
  micro-tuning cieco sullo stesso blocco.
- [ ] Congelare detector, routing e protocollo prima di validation/holdout e
  sealed; non aprire dati protetti prima di un PASS development.
- [ ] Eseguire da un processo nuovo un frozen-policy H0 a zero update con GRF
  primaria fissa e detector assente, shadow e autoritativo.
- [ ] Verificare in shadow l'invarianza numerica di observation load/contact,
  azioni, reward fisico e terminazioni.
- [ ] Quantificare separatamente l'effetto degli eventi two-sensor soltanto con
  un profilo che abbia superato development, validation e sealed.
- [ ] Rivalidare H0 prima di usare nuovamente `--warm-start` con il detector.
- [ ] Conservare `legacy_events` come default fino al completamento di timing,
  robustezza e gate di promozione.

### Pilot V13/corridor e ablation

- [ ] Lasciare completare il pilot fino all'iterazione 51 e documentare il suo
  `summary.json`, mantenendolo diagnostico e non promuovibile.
- [ ] Eseguire un A/B `H0 + V13/two_sensor` con corridor disattivato e tutti gli
  altri parametri invariati.
- [ ] Eseguire un A/B `H0 + detector precedente` con corridor `0.05` e tutti gli
  altri parametri invariati.
- [ ] Confrontare return, durata, cause di terminazione, HS/TO e componenti
  reward; decidere soltanto dopo se adattare H0 o rivedere detector/FSM.
- [ ] Eseguire rollout deterministici di `checkpoint_best` e
  `checkpoint_last`, con plot e confronto al pilot del 15 luglio.
- [ ] Non promuovere checkpoint prodotti dal pilot V13/corridor con il routing
  precedente.

### Morphology Corridor

- [ ] Completare un rollout live shadow da 500 step della modalita'
  complete-segment in una sessione OpenSim pulita e ripetere il confronto A/B.
- [ ] Scegliere il bordo phase-dependent definitivo e decidere se la sicurezza
  debba essere immediata o retrospettiva.
- [ ] Implementare e testare la riscrittura complete-segment/complete-episode
  delle reward prima del GAE, inclusi segmenti incompleti, timeout e bootstrap.
- [ ] Definire `WAIT_HS`, disponibilita' morfologica, stato temporale robusto e
  gate per il regime ankle lento.
- [ ] Solo dopo questi gate, eseguire un A/B corto a morphology weight
  `0 / 0.0025 / 0.005`, con checkpoint, start, seed, batch e learning rate
  invariati.
- [ ] Conservare ogni update e monitorare loss morphology, uscite corridor,
  cicli, penetrazione, return, SEA, reserve e drift actor.
- [ ] Aggiungere logging per evento e validare il pilot su start/seed held-out
  e almeno un profilo o modello esterno ad AB06 prima di training lungo.

### Training, reward e robustezza

- [ ] Non avviare training promuovibile sul detector finche' detector e routing
  non superano i gate prescribed; la validazione detector resta indipendente
  dal PPO.
- [ ] Conservare H0, V9/V13 e rollback come artefatti immutabili e
  confrontabili; ripartire da H0 o da una sorgente preregistrata.
- [ ] Valutare i checkpoint con multistart, seed held-out, worst-case recovery,
  cicli, penetrazione, reserve, SEA e clipping, non solo return nominale.
- [ ] Non cambiare reward, hard limit `25 mm` o feature actor per forzare la
  compatibilita' del detector.
- [ ] Separare gli effetti di reward, GRF online, reserve e timing FSM prima di
  attribuire causalmente i failure.
- [ ] Mantenere exact-start, compaction, interleaving, singola epoca e gate
  reserve condition-matched; concentrare recovery sugli step `210-230`.
- [ ] Preregistrare la non-regressione reserve e mantenere i seed `126-128`
  sigillati fino al PASS development completo.
- [ ] Eseguire un nuovo pilot controllato prima di un altro run lungo e
  analizzare return e advantage separatamente per start.
- [ ] Non riusare la stessa trace/seed per ulteriori dimezzamenti di learning
  rate o proiezioni; raccogliere recovery data event-aligned indipendenti.
- [ ] Riprovare `sigma=0.0075` su almeno tre seed per start solo dopo un PASS a
  `sigma=0.005`; differire la memoria ricorrente finche' non emerga un limite
  sequenziale reale.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
  multi-ciclo.

### Audit ex-novo, deployment e hardware

- [ ] Confrontare H0 e logical 24 con protocollo identico e overlay di azioni,
  served, cinematica, SEA, GRF, eventi e reward.
- [ ] Eseguire ablation progressive del plant prescribed e training A/B da H0
  modificando un solo gruppo reward per volta.
- [ ] Congelare ordine/scaling feature, frequenza, unita', normalizzazione BW,
  filtri, FSM, reset, governor, limiti, fallback e watchdog del contratto
  hardware.
- [ ] Validare export actor-only, equivalenza host-target, latenza worst-case,
  HIL e poi human-in-the-loop.
- [ ] Validare detector con rumore e delay realistici e ottenere ground truth
  localizzata, oppure dichiarare esplicitamente la semantica
  `initial_contact`.
- [ ] Portare la GRF online a production-ready mantenendola separata dai due
  sensori detector; mantenere le reserve come gate fisico finche' l'origine del
  residuo non e' attribuita.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta.
