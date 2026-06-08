# Dalla reward imitativa alle GRF online: direzione GLiDE-like

## Obiettivo emerso

L'obiettivo del Trajectory Generator non è riprodurre il più fedelmente possibile
la traiettoria IK sperimentale, ma generare traiettorie protesiche **ex novo** che
siano biomeccanicamente e dinamicamente adatte al cammino.

La traiettoria sperimentale può essere utile come inizializzazione, riferimento
diagnostico o sorgente di stati iniziali, ma non dovrebbe definire l'obiettivo
finale della policy.

## Punto di partenza: reward attualmente imitativa

La reward corrente della baseline MLP combina principalmente:

- `tracking_score`: premia la capacità della dinamica simulata di seguire la
  traiettoria protesica comandata;
- `reference_score`: premia la vicinanza della protesi alla IK sperimentale;
- `bio_score`: premia la vicinanza delle coordinate biologiche alla IK
  sperimentale;
- penalità su effort SEA, smoothness, saturazione, stati unsafe e riferimento
  fuori banda.

Inoltre, il default `action_mode="delta"` genera una correzione attorno alla IK
sperimentale. La rete è quindi vincolata all'imitazione sia dalla reward sia
dallo spazio delle azioni.

`tracking_score` non è necessariamente imitativo: può rimanere utile come misura
di quanto bene il SEA realizza una traiettoria generata dalla policy. Sono invece
fortemente imitativi `reference_score`, `bio_score` e l'action space `delta`.

## Prime strategie considerate

### Pretraining imitativo seguito da RL task-based

La rete viene inizialmente addestrata a riprodurre una gait sperimentale valida,
poi viene fine-tuned con una reward funzionale priva di imitazione.

Vantaggi:

- inizializzazione stabile;
- minore probabilità di caduta nelle prime fasi;
- apprendimento iniziale più semplice.

Rischio:

- la policy può rimanere intrappolata vicino alla gait imitata.

### Curriculum della reward

I pesi imitativi vengono ridotti progressivamente durante il training, mentre
aumentano i termini task-based.

Vantaggi:

- transizione graduale dalla gait nota alla scoperta autonoma;
- minore rischio di collasso improvviso.

Rischio:

- lo scheduling dei pesi diventa un nuovo elemento critico da tarare;
- se l'action space resta `delta`, la policy rimane comunque legata alla IK.

### Altre opzioni individuate

- RL task-based completamente da zero;
- trajectory optimization non imitativa seguita da distillazione/RL;
- policy goal-conditioned su velocità, cadenza, simmetria, effort o livello di
  assistenza;
- generazione di traiettorie assolute tramite spline o parametri funzionali di
  gait, invece di delta rispetto alla IK.

## Ispirazione dal paper P12: GLiDE

Il paper P12, **GLiDE: Generalizable Quadrupedal Locomotion in Diverse
Environments with a Centroidal Model**, propone una soluzione gerarchica:

```text
policy RL high-level
    -> accelerazioni lineari e angolari desiderate del corpo
    -> QP
    -> ground reaction forces desiderate e fisicamente ammissibili
    -> controllo low-level sul modello completo
```

La policy non imita una traiettoria articolare. Opera invece in uno spazio delle
azioni astratto e funzionale. Il QP converte i comandi in GRF rispettando vincoli
come:

- piedi attualmente in stance;
- cono d'attrito/no-slip;
- forza verticale minima;
- regolarizzazione delle forze;
- limiti geometrici delle gambe tramite il foot-placement layer.

GLiDE usa poi Jacobian transpose per realizzare le forze in stance e tracking di
una spline per i piedi in swing.

La reward è molto semplice e task-based: premia velocità desiderata, altezza del
corpo e orientamento stabile. Non contiene una traiettoria articolare normativa
da imitare.

## Lezione principale di GLiDE

La lezione più importante non è soltanto ridurre i termini imitativi della
reward. Una reward funzionale semplice funziona quando l'architettura impedisce
o rende costosi i comportamenti non fisici.

Nel paper:

- il gait pattern è prestabilito;
- il foot placement è strutturato;
- il QP impone vincoli sulle GRF;
- il controllore low-level realizza il comando astratto.

Con la stessa reward applicata direttamente alle coppie articolari, GLiDE
osserva gait ad alta frequenza poco naturali. Quindi, nel nostro progetto,
azzerare semplicemente `reference_score` e `bio_score` senza introdurre struttura
e vincoli rischierebbe di produrre traiettorie inadatte.

## Trasferimento GLiDE-like al Trajectory Generator

Una prima architettura applicabile alla protesi potrebbe essere:

```text
policy high-level
    -> parametri funzionali della gait protesica
    -> generatore spline/vincoli di fattibilità
    -> traiettoria assoluta knee/ankle
    -> SEA low-level
    -> dinamica completa OpenSim
```

La policy potrebbe generare:

- picco di flessione del ginocchio e relativo timing;
- estensione durante stance;
- ampiezza e timing del push-off della caviglia;
- durata delle fasi;
- lavoro o impulso protesico desiderato.

Il livello di trascrizione, equivalente concettuale del QP GLiDE, dovrebbe
garantire:

- limiti di posizione, velocità, accelerazione e jerk;
- continuità tra segmenti;
- compatibilità con la banda passante SEA;
- limiti di coppia, potenza e saturazione;
- eventuali vincoli derivati dal contatto e dalla stabilità.

La traiettoria dovrebbe essere assoluta (`action_mode="absolute"` o un nuovo
action space parametrico), non un delta attorno alla IK.

## Reward funzionale proposta

La reward finale dovrebbe misurare la qualità del comportamento ottenuto:

- stabilità e orientamento del bacino/COM;
- velocità o avanzamento desiderato;
- regolarità e qualità del passo;
- simmetria delle GRF e degli impulsi;
- effort muscolare e uso delle reserve;
- energia e potenza SEA;
- saturazione e fattibilità del tracking;
- smoothness della traiettoria;
- rispetto dei limiti articolari e assenza di stati unsafe.

La IK sperimentale potrebbe restare solo:

- nel pretraining;
- come riferimento diagnostico;
- per inizializzare gli episodi;
- eventualmente come regolarizzazione debole nelle prime fasi del curriculum.

## Limite fondamentale scoperto: GRF sperimentali imposte

Il simulatore attuale non calcola autonomamente le GRF risultanti dal contatto
piede-terreno. `model_loader.py` carica un file `.mot` e costruisce
`ExternalForce` dipendenti dal tempo.

Di conseguenza:

- le GRF applicate restano quelle della gait sperimentale;
- cambiando la traiettoria protesica, il terreno continua ad applicare forze
  registrate per un'altra traiettoria;
- la reward non può stabilire in modo affidabile se una nuova traiettoria
  produca davvero un passo fisicamente valido;
- simmetria, impulso propulsivo e qualità del contatto non emergono dalla
  dinamica generata dalla policy.

Questo limita profondamente la possibilità di apprendere traiettorie realmente
ex novo.

## Distinzione necessaria: QP e contatto online

Il QP GLiDE non sostituisce direttamente un modello di contatto fisico.

### GRF desiderate dal QP

Il QP sceglie forze desiderate che realizzano un obiettivo sul COM/corpo e
rispettano vincoli di fattibilità. Sono comandi o target di controllo.

### GRF effettive del contatto

Le GRF effettive devono risultare dall'interazione tra:

- geometria del piede;
- terreno;
- stato del corpo;
- traiettorie articolari;
- attuazione muscolare e protesica.

Per valutare correttamente una traiettoria ex novo, queste forze devono essere
calcolate online dal simulatore.

## Modifica necessaria al simulatore

La direzione proposta richiede una modalità di simulazione con GRF online:

1. disabilitare o rimuovere gli `ExternalLoads` sperimentali;
2. aggiungere e calibrare modelli di contatto piede-terreno OpenSim;
3. calcolare e registrare le GRF durante l'integrazione;
4. rilevare online stance, swing, heel strike e toe-off;
5. usare le GRF online nella reward e nelle metriche di gait;
6. rendere coerenti inverse dynamics, Static Optimization e integrazione con le
   nuove forze di contatto.

Il QP GLiDE-like può essere aggiunto come:

- planner di GRF desiderate;
- livello di trascrizione tra obiettivi funzionali e comandi;
- misura di fattibilità;
- sorgente di reward basata sull'errore tra GRF desiderate ed effettive.

## Architettura obiettivo

```text
obiettivi di locomozione
    -> policy high-level task-based
    -> accelerazioni COM / parametri funzionali / GRF desiderate
    -> QP o generatore vincolato
    -> traiettorie assolute protesiche
    -> SEA + muscoli + reserve
    -> OpenSim con contatto piede-terreno online
    -> GRF effettive e metriche di gait
    -> reward funzionale
```

Questa struttura mantiene la separazione già presente tra generatore
high-level e controllore SEA low-level, ma sostituisce il vincolo imitativo con
obiettivi e vincoli fisici.

## Strategia di sviluppo raccomandata

### Fase 1 - Reward e action space

- introdurre traiettorie assolute o parametriche;
- mantenere temporaneamente la IK per inizializzazione;
- separare chiaramente tracking di fattibilità e imitazione;
- aggiungere un curriculum che riduca `reference_score` e `bio_score`.

### Fase 2 - Contatto e GRF online

- creare una modalità alternativa del simulatore senza GRF sperimentali;
- aggiungere contatti piede-terreno;
- validare le GRF online contro i dati sperimentali sulla gait nota;
- mantenere entrambe le modalità per confronto e regressione.

### Fase 3 - Reward task-based

- introdurre velocità, stabilità, simmetria degli impulsi, effort, energia e
  qualità del passo;
- rimuovere progressivamente i termini imitativi;
- verificare che la policy non sfrutti comportamenti non desiderati.

### Fase 4 - Livello GLiDE-like

- introdurre un QP o generatore vincolato per trasformare obiettivi funzionali
  in target fisicamente realizzabili;
- confrontare training full OpenSim con pretraining su un modello ridotto;
- effettuare fine-tuning e validazione sul modello OpenSim completo.

## File analizzati

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/literature/notes/P12_glide_quadrupedal_centroidal.md`
- `model_loader.py`
- `inverse_dynamics.py`
- `simulation_runner.py`
- `config.py`

## Verifiche e analisi eseguite

- verificata la composizione della reward corrente e la presenza dei termini
  imitativi;
- verificato che il default dell'action space sia `action_mode="delta"`;
- letto il paper GLiDE e analizzati action space, QP, reward e trasferimento al
  modello completo;
- verificato che le GRF attuali siano caricate da file come `ExternalForce`;
- verificato che tali forze partecipino alla dinamica e all'inverse dynamics;
- distinto il ruolo delle GRF desiderate dal QP da quello delle GRF effettive
  generate dal contatto.

Non sono state implementate modifiche al simulatore o alla reward in questa
fase: il risultato è una proposta architetturale.

## TODO aperti

- Progettare una modalità `online_contact` alternativa agli `ExternalLoads`.
- Identificare e calibrare il modello di contatto piede-terreno OpenSim più
  adatto al modello AB06.
- Definire formalmente le metriche di suitability: velocità, stabilità,
  simmetria, effort biologico, energia SEA e qualità del passo.
- Progettare l'action space assoluto/parametrico e il livello di trascrizione
  vincolato.
- Definire il curriculum da reward imitativa a reward task-based.
- Validare le GRF online sulla gait sperimentale prima di usarle per traiettorie
  ex novo.
- Valutare un modello ridotto per pretraining GLiDE-like e fine-tuning OpenSim.
- Il training reale controllato da 50 iterazioni dopo la correzione
  `terminated`/`truncated` è stato completato il 2026-06-07. Restano training
  full-gait e miglioramento del critic.
- Tarare `oob_weight`, completare il reward rebalancing F2, portare le gait
  metrics RMSE/Symmetry/Trend e integrare la SNN come custom RLModule.
