# Risultati del training unified39 RIGHT a 40 iterazioni

Data: 23 settembre 2026. Run concluso alle 18:18:21 CEST, dopo 5 h 27 min 48 s.

**Il riferimento destro recupera il completamento durante training e riduce
fortemente i picchi positivi di coppia alla caviglia. La qualità cinematica e
la regolarità degli eventi non sono ancora equivalenti al V26 finale.** Il
risultato restringe il problema del precedente training sinistro al compito
prescribed e alla soluzione appresa; non sostiene un'incapacità generale della
rete 39D o della pipeline attuale di apprendere.

Sono state analizzate tutte le iterazioni e svolti due nuovi rollout
deterministici, best35 e last40. Entrambi completano 500 passi, 5 s. I sette
grafici canonici di ciascuno sono nel root `plot/`. Nessun ulteriore training,
oscuramento, modifica della FSM o modifica dei controllori è stato avviato.

## Esperimento e controlli

Il run parte da pesi casuali, seed 123: non è una continuazione della V26
trasferita né dello smoke. Actor e critic iniziali coincidono con quelli del
precedente run LEFT39. Actor 39, critic 88, due layer da 256 tanh; FSM v3,
start fisso a 12,99 s, batch effettivo 4096, minibatch 512, dieci epoche,
clock e quattro prescribed visibili al 100%, velocità non divise per dieci,
slew disattivato, stesso governor e stessa formula reward.

Confrontando le configurazioni risolte, escludendo la provenienza, cambiano
soltanto il budget del run iniziale e `target_reference: right_v26`. Questo
selettore cambia insieme **traiettoria target, clock, velocità e intervalli
di normalizzazione derivati dal target**. Non isola separatamente questi
quattro effetti. Gli shift RIGHT sono quelli V26: 0,465 al ginocchio e 0,452
alla caviglia; il clock sin/cos dell'actor è la fase destra non sfasata.

Verifiche conclusive:

- 40/40 iterazioni, nessun restart o iterazione saltata; 163.840 transizioni.
- Aggiornamenti di actor, critic e logstd; loss finite; 3.240 passi Adam.
- Sei canali visibili su tutte le transizioni, nessuna telemetria mancante.
- 48 hash degli input congelati all'avvio invariati.
- Nei nuovi rollout target q/qdot uguali ai campioni V26, clock coerente,
  velocità a scala 1 e disponibilità corretta.
- Reward ricostruite dalle tracce e azioni riprodotte offline entro la
  tolleranza numerica; checkpoint e input originali non modificati.

Gli smoke precedenti avevano superato 107 test e verificato target, maschera,
training, ripresa Adam e ricaricamento. Questa analisi non li rilancia: verifica
gli artefatti effettivi del run completo.

## Completamento a parità di esperienza

Il confronto principale usa la stessa finestra di **20.480 transizioni**.
I due run 39D terminano a 163.840 campioni; il V26 alla 33 ne aveva 163.212,
soltanto 628 in meno. I checkpoint V26 intermedi non sono conservati: questa
comparazione usa i log di training, non un rollout della V26 alla 33.

| Metrica | RIGHT39, iter 36–40 | LEFT39, iter 36–40 | V26, iter 29–33 |
|---|---:|---:|---:|
| Episodi completi registrati | **39/43, 90,7%** | **19/49, 38,8%** | **38/43, 88,4%** |
| Terminazioni per penetrazione | 4, di cui 1 hard | 30, di cui 1 hard | 4 |
| Altre terminazioni | 0 | 0 | 1 timeout stance |
| Durata media delle iterazioni | **4,886 s** | 4,082 s | 4,827 s |
| Return medio delle iterazioni | 204,27 | 184,65 | 195,66 |
| Entropia media | 1,877 | 2,201 | 2,069 |

I completamenti sono calcolati dalle differenze dei contatori cumulativi di
fine episodio. Return e durata sono medie dei valori per iterazione, con
denominatori diversi: non vanno convertiti direttamente in percentuali di
successo. I guasti fatali storici V26 possono non comparire nei suoi contatori.

Il miglioramento non è limitato alle ultime cinque iterazioni. RIGHT supera
l'80% nella finestra mobile di cinque iterazioni alla **27**, mantenendolo
fino alla 40. LEFT non raggiunge l'80% entro la 40. Prima della ripresa LEFT
dalla 25, le iterazioni 21–25 mostrano già **32/42 = 76,2% RIGHT contro
19/51 = 37,3% LEFT**: il riavvio fisico degli episodi alla ripresa non può
essere l'unica spiegazione del divario.

Il best di training resta la **35**, return 218,86. Alla 40 il return è 191,82;
questo calo isolato non cancella la tenuta del completamento nella finestra.

![Training a esperienza comparabile](../../plot/09_23_2026_right_reference_comparison/training_by_experience.png)

La loss di penetrazione TensorBoard nelle ultime cinque iterazioni è circa
0,00151 per RIGHT e 0,01627 per LEFT. Le loss di posizione normalizzate RIGHT
sono vicine a quelle V26 a esperienza simile. LEFT aveva in alcuni casi loss
imitative inferiori, pur interrompendosi molto più spesso: uno score imitativo
relativo migliore non garantisce una migliore coordinazione del contatto.
Quelle loss sono inoltre medie mobili sugli stati visitati e su target diversi.

## Rollout: miglioramento reale, ma qualità non ancora recuperata

I cinque rollout confrontati usano lo stesso start fisico e durano tutti 5 s.
RIGHT35 e RIGHT40 sono nuovi; gli altri sono gli originali archiviati.
V26best87 e LEFTbest72 hanno ricevuto più esperienza. Gli RMSE sono sempre
rispetto al proprio target: RIGHT e V26 condividono esattamente il target,
mentre LEFT ne usa uno diverso.

| Metrica | RIGHT best35 | RIGHT last40 | LEFT last40 | LEFT best72 | V26 best87 |
|---|---:|---:|---:|---:|---:|
| Return deterministico | 261,94 | **286,32** | 222,15 | 200,90 | 363,79 |
| RMSE ginocchio–target | 9,59° | 9,39° | 6,16° | 6,56° | **6,91°** |
| RMSE caviglia–target | 3,63° | **2,76°** | 5,64° | 5,98° | **1,46°** |
| Massima flessione ginocchio | 51,51° | 48,36° | 44,58° | 48,29° | 63,78° |
| Flessione massima richiesta | 64,32° | 64,32° | 56,23° | 56,23° | 64,32° |
| Picco positivo coppia caviglia | **5,66 Nm** | **3,30 Nm** | 37,22 Nm | 47,44 Nm | 10,19 Nm |
| RMS coppia ginocchio sopra 20 Hz, dopo 200 ms | **3,07 Nm** | **3,10 Nm** | 3,67 Nm | 4,11 Nm | 3,28 Nm |
| Cicli validati dalla FSM | **1** | **1** | 2 | 2 | 2 |

La caviglia migliora nettamente rispetto alle policy sinistre, soprattutto
alla 40. I grandi picchi positivi non compaiono nei nuovi episodi. Questo
non implica che l'intera curva di coppia sia migliore di V26: dopo i primi
200 ms la coppia RMS della caviglia è 4,34/6,13 Nm per RIGHT35/40 contro
3,62 Nm V26, con escursioni negative più pronunciate. Il confronto usa la
coppia elastica della molla, con il segno dei plot canonici.

Il chattering del ginocchio non scompare, ma la misura sopra 20 Hz è ormai
vicina a quella storica e inferiore alla LEFTbest72. Si tratta di un proxy
spettrale calcolato con identico filtro Butterworth di ordine 4, non di una
dimostrazione di assenza di oscillazioni o di qualità fisiologica.

**Il ginocchio è la debolezza cinematica più evidente:** RIGHT non recupera
ancora il picco di flessione richiesto. La 40 migliora lievemente l'RMSE
complessivo rispetto alla 35, ma ne riduce ulteriormente il picco. Non c'è
quindi un checkpoint migliore su tutti i criteri. Il best35 è il best secondo
la raccolta di training; la 40 è più interessante per accuratezza della
caviglia e return del rollout deterministico.

![Confronto RIGHT e V26](../../plot/09_23_2026_right_reference_comparison/right_vs_v26_deterministic.png)

## Contatto e FSM: il 90,7% non certifica la regolarità del passo

Entrambi i nuovi episodi arrivano al limite temporale, ma la FSM riconosce
solo **un ciclo valido**. Best35 registra due eventi invalidi e due
risincronizzazioni; last40 tre eventi invalidi e una risincronizzazione.
Le vecchie LEFT40 e LEFT72 registravano due cicli, senza questi eventi nel
rollout deterministico. V26best87 registra due cicli e zero eventi invalidi;
i suoi campi aggiuntivi di resync non erano presenti nel formato storico.

La causa immediata è una sequenza di HS/TO irregolare: alcuni heel strike
arrivano troppo presto dopo un toe off. La v3 scarta l'evento, può recuperare
la fase tramite risincronizzazione e degrada il ciclo interessato. L'episodio
prosegue, ma il ciclo non ottiene la validazione completa. Non significa che
la gamba abbia eseguito un unico movimento periodico nei cinque secondi.

Nel best35, per esempio, al TO di 15,866 s segue un HS rifiutato registrato
al confine di step di 16,000 s: l'intervallo è inferiore ai 200 ms minimi di
swing. Dopo 80 ms di discordanza fra detector binario e FSM avviene il resync
a 16,080 s; l'HS successivo a 17,081 s non accredita il ciclo degradato.
Nella 40 un episodio analogo comincia con TO a 14,284 s e HS rifiutato
registrato a 14,400 s. Il resync usa lo stato di stance del detector binario,
non soltanto la presenza di forza verticale. Queste sequenze localizzano
l'irregolarità ma non dimostrano da sole un bug del detector.

La reward imitativa concordata non penalizza direttamente questi eventi FSM:
non è stata aggiunta una correzione automatica dei pesi o cambiata la versione
della FSM. La regolarità degli eventi resta un criterio di valutazione distinto
dal completamento e dalla fedeltà cinematica.

Anche la penetrazione va letta con attenzione. Il massimo totale ai campioni
della reward è circa 14,79 mm per entrambi i nuovi rollout e per V26; nei
file STO arriva a 16,24 mm, dominato dal transitorio iniziale comune.
Escludendo i primi 200 ms, i massimi STO sono:

| RIGHT35 | RIGHT40 | LEFT40 | LEFT72 | V26best87 |
|---:|---:|---:|---:|---:|
| **10,77 mm** | **14,53 mm** | 18,21 mm | 21,84 mm | **11,17 mm** |

I nuovi episodi restano sotto la soglia di penalizzazione di 20 mm. Non sono
episodi a penetrazione geometrica zero. Best35 è migliore della 40 su questa
misura, mentre la 40 è migliore nell'accuratezza della caviglia.

## Dove resta l'errore e cosa spiega la reward

I SEA inseguono bene la reference servita. In RIGHT35/40, RMSE giunto–reference
circa **0,16° al ginocchio e 0,29–0,30° alla caviglia**. Gli errori
reference–target sono invece circa 9,59/9,38° e 3,66/2,78°. Il grosso
dell'errore resta quindi nella reference prodotta dalla combinazione policy e
governor, a monte dell'inseguimento SEA.

Sulla stessa finestra temporale 13,30–17,75 s, il migliore riallineamento
target–reference è **0 ms al ginocchio e +50 ms alla caviglia** per entrambe
le policy RIGHT. La correzione descrittiva del ritardo porta il RMSE caviglia
da 3,85 a 3,49° nella 35 e da 2,92 a 2,30° nella 40, su quella finestra.
Resta un errore non spiegato dal semplice shift; il deficit del ginocchio
non viene corretto traslando nel tempo il segnale. Queste sono misure sulle
tracce, non una prova che modificare il filtro produca lo stesso beneficio.

La penalità sul momento GRF alla caviglia è **zero** nei nuovi rollout,
contro **40,43** punti in LEFT40 e **43,11** in LEFT72. Non è stata rimossa:
non si attiva perché il contatto risultante non supera la sua condizione
sfavorevole. Questo termine riguarda il momento della forza di contatto,
non direttamente il segno della coppia SEA.

Il divario residuo di return rispetto a V26 è diverso: RIGHT40 ha minore
score imitativo e maggiore penalità di variazione dei comandi (**81,14**
contro **61,41** punti V26). Per RIGHT35 tale penalità è 77,19. Non emerge
un bug della formula reward: una policy può arrivare a fine episodio e
restare ancora lontana dalla soluzione storica su escursione, regolarità
e compensazione del filtraggio.

Sugli stati dei rispettivi rollout deterministici, la std media appresa
delle azioni è **0,69/0,60** nella 35 e **0,67/0,54** nella 40, ordine
ginocchio/caviglia; LEFT40 è a 0,86/0,93 e LEFT72 a 0,94/1,03. È compatibile
con un'esplorazione meno perturbante, ma non ne dimostra il contributo causale.
Questi valori non sono quelli delle precedenti batterie stocastiche: stati
e finestra temporale sono differenti. Non sono stati eseguiti nuovi test
rumorosi sul checkpoint congelato in questa analisi.

## Interpretazione e seguito consigliato

1. **La struttura 39D riesce ad apprendere una soluzione molto più stabile
   sul compito RIGHT.** A pari esperienza il completamento è vicino al V26,
   pur mantenendo FSM v3 e gestione attuale dei fallimenti. Le differenze
   comuni del contenitore non impediscono questo risultato.
2. **La scelta LEFT cambia un compito di controllo, non soltanto il nome dei
   dati.** Cambiano la forma della traiettoria, il clock e le scale relative
   dell'obiettivo. La normalizzazione LEFT era presente e corretta: non è
   stata corretta o eliminata in questo test. Le prove storiche di comando
   diretto LEFT mostrano inoltre che il movimento non è semplicemente
   impossibile da eseguire; è il suo apprendimento a restare meno riuscito.
3. **Non abbiamo ancora recuperato tutta la V26.** Completamento e picchi
   anomali della caviglia migliorano, ma flessione del ginocchio e sequenze
   di contatto richiedono ancora lavoro. Il confronto cinematico disponibile
   è contro best87, molto più addestrata: non prova che RIGHT40 sia in ritardo
   rispetto a V26 alla stessa esperienza.
4. **Il proseguimento RIGHT è ora motivato meglio dell'oscuramento immediato
   della vecchia LEFTbest72.** Prima di scegliere un checkpoint per oscuramento
   valuterei best35 e last40 con i seed di rumore già usati per V26, contando
   anche cicli validi e resync. Un'eventuale continuazione può verificare se
   si recuperano flessione e regolarità senza perdere completamento. Se
   l'obiettivo finale resta il prescribed sinistro, il trasferimento da una
   buona policy RIGHT al nuovo target è un esperimento separato da preparare.

Un seed per training, un solo start e due rollout deterministici non
dimostrano equivalenza statistica o robustezza generale. Il 90,7% descrive
episodi raccolti durante l'evoluzione della policy, non una probabilità di
successo misurata con un checkpoint congelato.

## Artefatti e file introdotti

- [Sette grafici canonici RIGHT best35](../../plot/09_23_2026_39D_right_best35/).
- [Sette grafici canonici RIGHT last40](../../plot/09_23_2026_39D_right_last40/).
- [Confronto grafico RIGHT/LEFT](../../plot/09_23_2026_right_reference_comparison/right_vs_left_deterministic.png).
- [Metriche complete dei rollout](<../../Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/rollout_metrics.json>).
- [Curve, metriche e audit del training](<../../Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/training_history/findings.md>).
- [Sequenze degli eventi e audit FSM](<../../Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/fsm_audit/metrics.json>).
- [Verifiche finali di integrità](<../../Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/final_verification.json>).
- [Metodo e riproduzione](<../../Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/README.md>).

Nuovi file: script e ricevute in `validation/right_reference_results_2026-09-23/`,
due cartelle di rollout, quattordici plot canonici e figure comparative in
root `plot/`, questo report. Nessun sorgente di produzione modificato durante
l'analisi. Entrambi i set canonici riportano `No missing channels`.

## TODO

- Valutazione stocastica appaiata di best35 e last40, con criteri di ciclo
  valido e risincronizzazione oltre al completamento.
- Decidere se continuare RIGHT per recuperare flessione e regolarità;
  nessuna continuazione oltre 40 avviata in questa analisi.
- Mantenere separati eventuale adattamento al target LEFT e oscuramento.
- Eventuali repliche indipendenti e margini predefiniti per una verifica
  formale di equivalenza statistica.
