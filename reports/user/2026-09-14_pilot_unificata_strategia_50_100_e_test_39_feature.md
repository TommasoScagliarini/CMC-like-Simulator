# Pilot della rete unificata, strategia 50→100 e futura variante a 39 feature

**Data:** 2026-09-14  
**Instruction check token:** `CMC_AGENT_OK_2026`  
**Stato:** pilot V1 a 35 ingressi completato a 5 iterazioni; sei rollout e analisi storica completati. Prosecuzione a 50/100 e variante a 39 ingressi ancora da eseguire.

## 1. Decisione e ordine delle attività

L'utente propone di valutare prima il potenziale della rete unificata attuale
con un pretraining più lungo, mantenendo la baseline storica intatta:

1. Riprendere il checkpoint 5 della V1 fino a **50 iterazioni totali**:
   altre **45**, conservando actor, critic e optimizer tramite resume completo.
2. Valutare i risultati intermedi.
3. Proseguire per **altre 50 iterazioni**, fino a **100 totali**.
4. Valutare i risultati finali e il progresso rispetto a 5 e 50.
5. Svolgere i test di fattibilità della variante con prescribed osservabile
   e mascheramento, descritti nel [protocollo precedente](2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).

**Chiarimento esplicito dell'utente:** la variante a **39 feature va creata
quando si svolgeranno quei test**, in parallelo alla V1 a 35, conservandola.
La richiesta non comporta la creazione della variante durante questo
`create_report`. Nessuna implementazione della variante è stata avviata.
La scelta documentata è 39 ingressi; un eventuale quarantesimo indicatore
di disponibilità rimane una proposta precedente, non una decisione adottata.

Il conteggio 45 + 50 rende coerenti i traguardi di 50 e 100. Il training
effettivamente concluso resta quello di cinque iterazioni. La redazione
attuale modifica soltanto questo report e il daily per propagare i TODO.

## 2. Problema e avvio del pilot corretto

Il primo comando utente falliva prima dell'allenamento perché utilizzava
Python dell'ambiente base, privo di Ray. Il rilancio ha usato Python 3.10.20
di `envCMC-rllib`, con OpenSim 4.5.2, Ray 2.55.1 e PyTorch 2.10.0.

Durante il primo tentativo con l'ambiente corretto è emerso un secondo
problema: due guardie terminali morphology restavano attive in imitazione,
nonostante il relativo obiettivo fosse escluso dalla reward. Il tentativo
è stato interrotto deliberatamente e archiviato con il checkpoint disponibile.
La correzione nel solo wrapper unificato limita quelle guardie a `ex_novo`,
conservando diagnostica e protezioni comuni. Il pilot è ripartito da pesi
nuovi con seed 123.

Nel confronto sul medesimo primo batch, il digest dell'actor aggiornato
risulta identico: la vecchia guardia aveva sovrascritto l'etichetta di una
terminazione fisica già presente. Il fix risolve quindi un errore potenziale
di isolamento, coperto dai test; non dimostra una variazione dei ritorni
di quel batch specifico.

Il [pilot corretto](<../../Trajectory Generator/unified_MLP/runs/training/pretraining_v1_pen1_pilot_2026-09-14/summary.json>)
ha completato **5/5 iterazioni**, **23 040 transizioni**, senza crash, restart
o iterazioni saltate, dalle 13:11:59 alle 13:57:56 CEST. Il tempo misurato
del processo è **2753.43 s**, circa 45 min 53 s.

Configurazione conservata: actor 35 ingressi, due layer tanh da 256, target
sinistro nella reward, 12 EnvRunner, batch 4608 con 1536 transizioni per start,
minibatch 512, 10 epoche, LR 1e-4, clipping PPO 0.2, logstd apprendibile e peso
penetrazione 1.0. Detector, FSM actor v3, slew limiter, C2 e SEA sono quelli
dello snapshot unificato. L'ex-novo resta separato, con peso penetrazione 0.05.

| Iterazione | Return medio training | Durata media episodio [s] | Tempo iterazione |
|---|---:|---:|---:|
| 1 | -84.25 | 1.27 | 9 min 13 s |
| 2 | -38.22 | 2.92 | 9 min 07 s |
| 3 | -16.17 | 2.61 | 9 min 25 s |
| 4 | -27.78 | 2.84 | 9 min 01 s |
| 5 | -157.75 | 2.02 | 8 min 52 s |

Il best secondo la metrica di training è il checkpoint 3; l'ultimo è il 5.
Il return aggregato RLlib dipende dagli episodi completati e dalle finestre
di aggregazione: non rappresenta una valutazione deterministica con uguale
peso dei tre start. La sua discesa finale ha motivato il confronto successivo.

## 3. Confronto deterministico dei checkpoint 3 e 5

Sei rollout con stesso seed, fisica, configurazione, tre start e limite
500 step/5 s hanno completato l'esecuzione senza crash. Il tracking confronta
actual, riferimento servito dopo filtro e prescribed sinistro allo stesso
timestamp post-step, senza riallineamenti di fase.

| Start | Durata CP3 / CP5 [s] | RMSE ginocchio CP3 → CP5 [°] | RMSE caviglia CP3 → CP5 [°] | Penetrazione max CP3 → CP5 [mm] |
|---|---:|---:|---:|---:|
| Early | 5.00 / 5.00 | 26.33 → 24.97 | 12.96 → 12.27 | 20.26 → 23.98 |
| Center | 2.80 / 2.80 | 25.65 → 25.27 | 13.03 → 11.30 | 20.36 → 24.86 |
| Late | 2.83 / 2.83 | 24.61 → 23.63 | 12.62 → 10.75 | 18.71 → 18.77 |

Early raggiunge il limite temporale. Center e late terminano per timeout
swing, con un toe-off ciascuno. **Zero heel strike e zero cicli validi in
tutti i rollout**; periodo online sempre zero. Il checkpoint 5 migliora
il tracking angolare, ma non durata o cicli e aumenta la penetrazione.
Il tracking delle velocità non migliora uniformemente.

La RMSE actual–served è soltanto **0.25–0.29° al ginocchio e 0.16–0.40°
alla caviglia**. I giunti seguono il riferimento servito; lo scostamento
maggiore è già presente fra quel riferimento e il prescribed. Nessun
clipping delle azioni o saturazione interna `tau_input` registrati.

Anche le cinque pubblicazioni TensorBoard del training riportano zero HS,
cicli e periodo online. Sono però **medie mobili su 100 righe per worker**,
non un conteggio esaustivo di tutte le transizioni. L'assenza esatta dei
cicli è verificata nelle sei tracce complete; non va estesa automaticamente
a ogni istante del training usando soltanto quei valori TensorBoard.

Il periodo non disponibile è un'osservazione, non una diagnosi causale:
queste prove non distinguono ancora allenamento insufficiente, limiti di
osservabilità e altre difficoltà di controllo o ottimizzazione. Gli altri
segnali causali e la FSM restano disponibili all'actor.

Artefatti: [tabelle complete](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/comparison.md>),
[grafici](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/tracking_angles.png>),
[manifest ed esito dei controlli](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/completion_verification.json>).

## 4. Precedente V26 e motivazione del budget

È stato esaminato il run storico
`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`, senza modificarlo.
Nelle prime cinque iterazioni le durate medie erano **2.12, 2.49, 2.30,
2.30 e 2.75 s**: anche quella policy era ancora immatura. Gli indicatori
TensorBoard dei cicli erano però già positivi in tutte e cinque le
pubblicazioni. Questo prova la presenza di alcuni cicli nel training V26,
senza quantificarne il totale né attestare cammino robusto. Non è stato
trovato un rollout deterministico V26 delle prime cinque iterazioni.

A 50 iterazioni il V26 registrava durata media **4.81 s** e return **225.76**;
il best di training arrivò a **87**, con return **271.00**. Il relativo
rollout deterministico storico completò 5 s e due cicli validi. Questo
precedente sostiene l'utilità di un controllo a 50 e di un budget fino a 100,
ma non garantisce lo stesso esito nell'unificata.

Il confronto non isola la sola rimozione dei target: cambiano anche start,
FSM actor storica v2 rispetto a v3, supervisione controlaterale rispetto
a sinistra e slew limiter. I campioni delle prime cinque iterazioni sono
26 782 nel V26, con raccolta non esatta, contro 23 040 nell'unificata.

## 5. ETA e valutazioni previste

La stima usa il pilot sulla stessa macchina:
**2753.43 / 5 = 550.69 s per iterazione**, inclusi gli overhead del processo.

| Traguardo totale | Transizioni totali | Da pesi nuovi | Residuo dal checkpoint 5 |
|---|---:|---:|---:|
| 50 iterazioni | 230 400 | 7 h 39 min | 6 h 53 min: altre 45 |
| 100 iterazioni | 460 800 | 15 h 18 min | 14 h 32 min: altre 95 |

Il tratto **50→100** richiede circa **7 h 39 min**. I rollout di valutazione
sono aggiuntivi; i tempi restano estrapolazioni del costo osservato e possono
variare durante l'apprendimento.

Alle due valutazioni si confronteranno ultimo checkpoint e candidato migliore
sui medesimi tre start, con criteri fissati prima della selezione: tracking
actual/target e served/target, cicli ed eventi validi, primo periodo online,
durate e terminazioni, penetrazione e diagnostica SEA. Avvio e regime vanno
distinti. Un episodio lungo senza cicli non dimostra qualità del cammino.
La configurazione della V1 resta costante fra 5, 50 e 100 per rendere
interpretabile l'effetto dell'esperienza aggiuntiva.

Fonti e calcoli: [confronto storico ed ETA](<../../Trajectory Generator/unified_MLP/runs/validation/pretraining_v26_comparison_eta_2026-09-14/comparison_eta.md>).

## 6. Test futuri e variante a 39 feature

La creazione avverrà **nella fase dei test di fattibilità, dopo le valutazioni
della V1 previste dalla strategia**, in un perimetro parallelo sotto
`Trajectory Generator/`. V1 a 35, baseline storica e loro checkpoint restano
conservati. La directory della variante non è stata creata in questa attività.

Le quattro feature aggiuntive sono posizioni e velocità prescribed sinistre
di ginocchio e caviglia al tempo corrente. La variante manterrà lo stesso
schema a 39 nelle proprie fasi imitativa ed ex-novo. Il mascheramento agisce
sulla copia destinata alla rete; il target originale della reward imitativa
resta disponibile. Non cambia la prescrizione biologica del simulatore.

Il protocollo comprende:

- **Osservabilità:** dati allineati temporalmente e traiettorie indipendenti,
  distinguendo ricostruibilità del target, fase iniziale e controllo in simulazione.
- **Isolamento software:** a maschera chiusa, cambiare le copie dei target
  non deve alterare output né aggiornamenti ex-novo; verificare anche critic,
  normalizzazione, loss e ripristino. Con maschera aperta serve un controllo
  positivo. La disponibilità usata nella raccolta deve restare la stessa
  durante le epoche PPO dello stesso campione.
- **Confronto A/B/C:** A sempre visibile; B sempre mascherata; C curriculum
  fino al mascheramento completo. Architettura a 39, inizializzazioni abbinate,
  budget, reward, start, fisica e detector comuni. Valutare tutti anche senza
  target visibili. La V1 a 35 è un riferimento aggiuntivo e non sostituisce B.
- **Trasferimento:** verificare caricamento diretto, sigma 0.010, critic e
  optimizer nuovi, warm-up critic-only e resume; confrontare il successivo
  ex-novo inizializzato da B e C. Il vantaggio cercato deve sopravvivere alla
  rimozione dell'aiuto prescribed.

Schema e meccanismo devono essere implementati nella variante: non sono
ottenibili aggiungendo un flag di resume al checkpoint V1. Curriculum,
budget dei confronti, seed e soglie qualitative restano da definire prima
dell'esperimento. Un singolo run di 100 iterazioni non costituisce una prova
di impossibilità informativa o una qualifica statistica di parità.

## 7. File modificati e verifiche eseguite

Lavoro precedente di questa sessione, già presente prima del report:

- `unified_MLP/reward_function.py` e `tests/test_env_reward.py`: isolamento
  delle terminazioni morphology, con 18 regressioni comportamentali.
- `unified_MLP/validation/compare_pretraining_checkpoints.py` e
  `tests/test_checkpoint_comparison.py`: analizzatore offline e sei test.
- `unified_MLP/validation/plot_pretraining_checkpoints.py`: figure PNG/PDF.
- `unified_MLP/README.md` e `VALIDATION.md`: ambiente Python, correzioni,
  completamento del pilot, confronto e comandi di analisi.
- Artefatti dedicati in `unified_MLP/runs/`: pilot, tentativi archiviati,
  rollout, metriche, grafici e calcolo ETA.

Verifiche già eseguite su macOS arm64: **100 test della suite in 4.20 s**
dopo il fix reward; **sei nuovi test dell'analizzatore passati separatamente**;
pilot 5/5; reload dell'export in processo nuovo; sei rollout con exit code 0,
tracce complete e 17 file `.sto` ciascuno; grafici controllati visivamente;
hash degli export 3/5 invariati e confronto Git della baseline tracciata
senza differenze fuori dall'area unificata e dai report. Non si afferma
l'esecuzione di una nuova suite complessiva da 106 test.

Con questo `create_report` si crea il presente file e si aggiorna
il [daily](../daily/2026-09-14_daily-report.md). I controlli della redazione
riguardano collegamenti locali, propagazione dei TODO e conservazione delle
sorgenti preesistenti. Nessun nuovo training, rollout, test della rete o
implementazione a 39 feature viene eseguito. Windows live rimane da verificare;
predittività e compensazione del ritardo restano rinviate.

## 8. TODO da propagare

- [ ] Riprendere la V1 a 35 feature dal checkpoint 5 fino a 50 iterazioni totali, conservando configurazione e stato completo del training.
- [ ] Fissare i criteri e svolgere la valutazione intermedia a 50 sui tre start, includendo tracking, cicli, primo periodo online, terminazioni e penetrazione.
- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.
- [ ] Creare la variante parallela a 39 feature soltanto nella successiva fase dei test di fattibilità, conservando V1 a 35 e baseline storica; nessuna implementazione ora.
- [ ] Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.
- [ ] Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.
