# Confronto offline degli storici RIGHT39, LEFT39 e V26

Data: 23 settembre 2026. Questo memo riguarda l'apprendimento e i contatori di
training. Non contiene nuovi rollout né modifiche alla produzione.

**La 39D addestrata da zero sul riferimento destro recupera il livello di
completamento storico a esperienza comparabile. Il divario rispetto alla LEFT
compare prima della sua ripresa dalla 25, quindi quella ripresa non può essere
l'unica origine della differenza.** Non è una prova di equivalenza statistica,
né separa il contributo di traiettoria, clock e normalizzazione.

## Integrità e condizioni

- RIGHT ha concluso 40 iterazioni, senza restart, crash o iterazioni saltate,
  dalle 12:50:30 alle 18:18:21 CEST; durata 19.667,86 s, circa 5 h 28 min.
- 163.840 transizioni, tutte con sei ingressi prescribed/clock visibili. Zero
  telemetria mancante. Actor medio e critic aggiornati in tutte le iterazioni;
  loss ed entropia finite. I 48 hash del manifest di lancio restano invariati.
- I digest iniziali di actor completo, parte media e critic RIGHT coincidono
  esattamente con quelli iniziali LEFT nei log precedenti al primo update.
- Confrontate le configurazioni risolte dentro i checkpoint LEFT25 e RIGHT40:
  esclusa la provenance, cambiano soltanto il budget 25→40 e
  `target_availability.target_reference` da assente/default LEFT a `right_v26`.
  Nel checkpoint LEFT40 il solo cambiamento rispetto a LEFT25 è il budget.
- Le prime 25 righe LEFT coincidono con la copia conservata prima della ripresa.
  La ripresa interrompe la continuità degli episodi fisici dei worker; lo stato
  learner era stato verificato nel report del 21 settembre.
- Non si sostiene che i sorgenti dei due run siano tutti byte-identici: il
  supporto RIGHT e ulteriore telemetria sono successivi. Tra i sette elementi
  condivisi dai piccoli manifest originari cambiano `env.py`, `phase_target.py`,
  `target_availability.py` e `train_ppo_mlp.py`; il preflight e i 107 test di
  regressione del lancio verificano il nuovo percorso. La ricetta, FSM v3,
  governor, SEA, soglie e parametri PPO risolti sono gli stessi.

## Confronto a pari esperienza

Le tre finestre contengono esattamente 20.480 transizioni. I contatori sono
cumulativi: ogni conteggio è la differenza fra estremi, non la somma delle righe.
Return e durata sono invece medie aritmetiche delle metriche per iterazione;
non hanno necessariamente gli stessi denominatori dei contatori di fine episodio.

| Metrica | RIGHT39 36–40 | LEFT39 36–40 | V26 29–33 |
|---|---:|---:|---:|
| Esperienza cumulativa finale | 163.840 | 163.840 | 163.212 |
| Episodi completi | **39/43 = 90,70%** | **19/49 = 38,78%** | **38/43 = 88,37%** |
| Penetrazioni ordinarie | 3 | 29 | 5 |
| Penetrazioni hard | 1 | 1 | Non registrate con questa semantica |
| Timeout swing | 0 | 0 | 0 |
| Return medio delle iterazioni | 204,27 | 184,65 | 195,66 |
| Durata media delle iterazioni | 4,886 s | 4,082 s | 4,827 s |
| Update di minibatch cumulativi | 3.240 | 3.240 | 3.224 |

RIGHT e LEFT hanno esattamente lo stesso budget. V26 ne differisce di sole
628 transizioni, −0,38%. Il suo oversampling storico spiega perché il confronto
corretto sia contro la 33 e non contro la 40. A V26 iter40 le transizioni erano
191.884; anche la finestra V26 36–40 ha 39/43 episodi completi, ma usa più esperienza.

Il best RIGHT è la 35, return 218,86, a 143.360 transizioni. Il best LEFT entro
la 40 era la 29, return 207,99, a 118.784 transizioni. Il miglior V26 entro il
budget della 33 è la stessa 33, return 211,13. I ritorni LEFT e RIGHT non sono
misure identiche di qualità fisica perché il target e le escursioni sono diversi.
La selezione del best è inoltre basata sulla raccolta esplorativa della iterazione,
mentre l'export contiene i pesi successivi all'update PPO.

## Progressione: il recupero non dipende da una sola finestra fortunata

| Blocco | RIGHT completi | LEFT completi | RIGHT penetrazioni | LEFT penetrazioni |
|---|---:|---:|---:|---:|
| 1–5 | 1/112 = 0,9% | 2/119 = 1,7% | 96 | 107 |
| 6–10 | 7/71 = 9,9% | 3/81 = 3,7% | 31 | 48 |
| 11–15 | 13/60 = 21,7% | 4/72 = 5,6% | 9 | 46 |
| 16–20 | 26/50 = 52,0% | 10/66 = 15,2% | 12 | 47 |
| 21–25 | 32/42 = **76,2%** | 19/51 = **37,3%** | 7 | 32 |
| 26–30 | 38/43 = 88,4% | 19/40 = 47,5% | 4 | 20 |
| 31–35 | 35/40 = 87,5% | 30/50 = 60,0% | 5 | 18 |
| 36–40 | 39/43 = 90,7% | 19/49 = 38,8% | 4 | 30 |

La finestra mobile di cinque iterazioni RIGHT supera per la prima volta:

- il 50% alla 19, 77.824 transizioni, restando sopra in tutte le finestre successive;
- l'80% alla 27: 35/41 = 85,4%, 110.592 transizioni. Tutte le finestre fino alla 40
  restano sopra l'80%; non tutte sopra il 90%.

LEFT non raggiunge mai l'80% nelle prime 40 iterazioni. V26 lo raggiunge alla
31, 155.020 transizioni, con 36/43 = 83,7%. Sono soglie descrittive calcolate
a posteriori, non criteri di successo prefissati. Il confronto già entro 25
iterazioni è comunque informativo, prima che la ripresa LEFT entri in gioco.

## Esplorazione e learner

| Metrica media nella finestra a pari esperienza | RIGHT36–40 | LEFT36–40 | V26 29–33 |
|---|---:|---:|---:|
| Entropia gaussiana | **1,8767** | 2,2010 | 2,0689 |
| KL | 0,009930 | 0,009922 | 0,010061 |
| VF loss | 2,9730 | 2,9540 | 3,0211 |
| Varianza spiegata critic | 0,3561 | 0,1635 | 0,1287 |

Non emerge un learner fermo o numericamente instabile. Il controllo RIGHT
apprende una distribuzione delle azioni meno dispersa, pur usando la stessa
configurazione dell'esplorazione e lo stesso coefficiente di entropia zero.
È coerente con una soluzione più regolare, ma non dimostra che la riduzione
di rumore sia la causa indipendente del recupero. Cambiano gli stati visitati
e la media appresa. La deviazione standard per giunto e il clipping richiedono
l'audit del checkpoint sugli stati di rollout; l'entropia aggregata non li identifica.

Il JSON riporta anche un semplice proxy sigma geometrico ricavato dall'entropia
con `exp((H−log(2πe))/2)`, rispettivamente 0,6184/0,7273/0,6808. Non è la media
aritmetica della sigma di un singolo giunto, né un test stocastico della policy congelata.

La telemetria rileva 165 righe con resync RIGHT e 133 LEFT nelle prime 40
iterazioni. Sono righe per transizione, non un conteggio di episodi unici né una
classifica di salute: non si conclude che più resync causino più robustezza.
La telemetria storica V26 mancante resta `null`, non zero.

## Loss imitativa e contatto

Valori delle medie mobili TensorBoard, mediati sulle finestre comparabili:

| Metrica | RIGHT36–40 | LEFT36–40 | V26 29–33 |
|---|---:|---:|---:|
| Loss posizione ginocchio normalizzata | 0,04412 | 0,03727 | 0,04183 |
| Loss posizione caviglia normalizzata | 0,04625 | 0,02423 | 0,04411 |
| Score imitazione effettiva | 0,71161 | 0,78236 | 0,71843 |
| Score imitazione reference servita | 0,58349 | 0,67029 | 0,58979 |
| Loss penetrazione | **0,00151** | **0,01627** | 0,00283 |
| Loss momento GRF caviglia | 0,12695 | 0,17001 | 0,29436 |

RIGHT e V26 hanno lo stesso template e le stesse escursioni; le loss imitazione
sono vicine a parità di esperienza. LEFT può mostrare una loss relativa più bassa
e contemporaneamente più fallimenti. Non è una contraddizione: cambia il movimento
richiesto e la conversione fra errore in gradi ed errore normalizzato, oltre agli
stati visitati. La loss di penetrazione LEFT è circa 10,8 volte quella RIGHT in
questa finestra. Non va interpretata come probabilità di penetrazione né come
rapporto dei picchi geometrici.

Questo test sostiene che la struttura 39D attuale è capace di apprendere da zero
una soluzione robusta durante la raccolta sul compito storico RIGHT, con FSM v3
e gestione dei fallimenti attuali. Riduce la plausibilità di un limite generale
della dimensione della rete, della FSM v3 o del learner. Non identifica ancora
quale componente del passaggio al prescribed LEFT renda il compito più difficile:
traiettoria, fase, velocità e normalizzazione sono cambiate insieme.

## Limiti e artefatti

Un solo seed di training per condizione, uno start e policy in continua evoluzione:
non trattare le 43 terminazioni come 43 repliche indipendenti del training, né
presentare 90,7% e 88,4% come equivalenza statistica. Alcuni fallimenti storici
facevano morire worker e potevano non apparire nei contatori. Il completamento
significa arrivare a 500 step, non assenza di penetrazione o coppie corrette.
Il rollout storico best87 ha inoltre più esperienza del nuovo best35.

Artefatti locali riproducibili:

- `analyze.py`: lettura JSONL/config risolte/TensorBoard e audit; non simula episodi.
- `metrics.json`: metriche, conteggi, configurazioni e limiti.
- `iterations.csv`: 120 righe, prime 40 iterazioni dei tre run.
- `tensorboard_selected.json`: nove serie selezionate per run, indicizzate per transizioni.
- `input_hashes.json`: digest degli input letti; `analysis_stdout.json`: ricevuta.
- `plot/09_23_2026_right_reference_comparison/training_by_experience.png`.
- `plot/09_23_2026_right_reference_comparison/learner_and_losses.png`.

Fonti contestuali: report di avvio del 23 settembre; report risultati LEFT40 e
cause delle differenze del 21 settembre; checkpoint e log originali V26 del
20 agosto. La cinematica, le coppie e la validazione del best35 vengono analizzate
separatamente dal processo principale.
