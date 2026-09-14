# Rete unificata V1 e protocollo di parità qualitativa con la baseline

**Data:** 2026-09-06  
**Instruction check token:** `CMC_AGENT_OK_2026`  
**Stato:** pipeline V1 implementata e verificata con test e smoke su macOS; confronto qualitativo completo ancora da realizzare.

La rete unificata permette pretraining imitativo e training ex-novo con lo
stesso actor e senza adattamento intermedio. La baseline è conservata. Le
verifiche eseguite dimostrano il funzionamento della pipeline; **non dimostrano
ancora una qualità del cammino pari alla baseline**.

Questo report raccoglie la consegna V1 e la successiva discussione sul metodo
di confronto. Il benchmark descritto è una proposta operativa: non è stato
implementato o eseguito con la richiesta `create_report`.

## 1. Problema e obiettivo

La pipeline storica richiedeva il passaggio da un actor imitativo a uno
ex-novo attraverso adattamenti del contratto delle osservazioni. L'obiettivo
dell'utente è eliminare questo passaggio, conservando la qualità dei risultati
della baseline e mantenendo quest'ultima disponibile per utilizzi futuri.

La nuova rete parte da pesi nuovi e usa come supervisione il prescribed della
gamba sinistra del setup AB06. Conservare capacità dell'MLP, fisica e detector
rende possibile un confronto controllato, ma non garantisce che un nuovo
addestramento raggiunga automaticamente la stessa prestazione: cambiano i
pesi iniziali e il percorso di apprendimento.

Occorre quindi verificare separatamente:

- correttezza della pipeline e del passaggio di modalità;
- qualità del comportamento nel simulatore dopo l'addestramento;
- quantità di dati e risorse necessarie per raggiungere quella qualità.

## 2. Soluzione implementata nella V1

La nuova pipeline vive in
[Trajectory Generator/unified_MLP](<../../Trajectory Generator/unified_MLP/README.md>).
La baseline del commit `8ef48f914898fff8431cf8d164554448dbe35947`, il runtime
condiviso e il plugin C++ non sono stati modificati durante l'implementazione.
Output, TensorBoard e checkpoint hanno directory dedicate; il registro dei
run della baseline non viene aggiornato dalla nuova pipeline.

| Componente | Contratto della V1 |
|---|---|
| Actor | 35 osservazioni, due layer tanh da 256, due azioni assolute knee/ankle; testa gaussiana con due medie e due logstd |
| Osservazioni | Stesso ordine, unità e normalizzazione nelle due fasi; target imitativi esclusi dall'actor; due colonne storiche del clock prescribed a zero |
| Supervisione | `pros_knee_angle` e `pros_ankle_angle` dal prescribed sinistro via `base_kin`, posizioni e velocità al tempo corrente; errori espliciti per target mancanti o non finiti |
| GRF e detector | GRF fisiche online sinistre, prescribed destre, detector analogico, heel/toe V25, FSM V26 e actor FSM v3 conservati |
| Controllo | Frequenza policy, slew limiter, filtro C2 a 4 Hz, limiti SEA e reset della configurazione scelta conservati |
| Reward | `reward_mode: imitation | ex_novo`; obiettivi esclusivi, con penalità comuni e protezioni attive in entrambe le fasi |
| Transizione | `--init-actor-from`: copia diretta dell'actor unificato con verifica del contratto, critic e optimizer nuovi, sigma 0.010 applicata separatamente |
| Warm-up e resume | Una prima iterazione critic-only con actor e logstd congelati; `--resume-from` ripristina la stessa fase senza ripetere inizializzazione o warm-up |

L'ambiente e i parametri ex-novo derivano dal run
`MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01`: morphology knee/ankle
5.0/5.0, sigma 0.010 congelata e parametri PPO conservati. Il pretraining usa
LR `1e-4`, 10 epoche, clip PPO `0.2`, logstd apprendibile e pesi imitativi
della baseline imitativa. Entrambi i preset completi usano batch 4608 e
12 EnvRunner, con raccolta bilanciata sui tre start.

Configurazioni risolte e provenienza sono salvate nei run. **Actor predittivo,
compensazione del ritardo e training lunghi restano esclusi dalla consegna V1.**

## 3. Test e verifiche già eseguiti

Il dettaglio e i collegamenti agli artefatti originali sono in
[VALIDATION.md](<../../Trajectory Generator/unified_MLP/VALIDATION.md>).

| Verifica | Risultato effettivo |
|---|---|
| Suite automatica | 82 test superati in 3.95 s su macOS, con il comando documentato e senza impostare `PYTHONPATH` |
| Ambiente reale | Tre start: 35 feature actor e 84 osservazioni totali; osservazioni esattamente uguali nelle due modalità al reset e dopo lo stesso primo comando; target sinistro e clock verificati |
| Smoke principale | Una iterazione imitativa, una critic-only e una ex-novo; ciascuna con 192 transizioni, tre worker, minibatch 64 e un'epoca: 576 transizioni totali |
| Transizione | Copia esatta dell'actor; rete delle medie invariata dopo l'applicazione di sigma; actor invariato e critic aggiornato durante il warm-up; medie aggiornate durante PPO ex-novo |
| Resume in processo nuovo | Checkpoint ripristinato all'iterazione 2, digest actor/critic coincidenti, warm-up già completato; zero nuovi aggiornamenti perché il totale richiesto era già raggiunto |
| Verifica multi-epoca aggiuntiva | Una iterazione imitativa con batch 192 e 10 epoche: 1920 righe elaborate, 30 minibatch, bilanciamento corretto e zero KL non finiti |
| Isolamento | Controllo Git conclusivo dell'implementazione senza modifiche esterne a `unified_MLP/` |

I controlli live sono stati eseguiti su macOS arm64 con Python 3.10.20,
Ray 2.55.1 e PyTorch 2.10.0 nell'ambiente `envCMC-rllib`. I percorsi e i comandi
Windows sono predisposti; **non è stata eseguita una verifica dinamica Windows**.

I tre rollout deterministici del checkpoint dello smoke hanno prodotto:

| Start offset | Passi | Termine | HS / TO / cicli validi | Penetrazione massima |
|---|---:|---|---:|---:|
| Early, 1.756870983805102 s | 500 | Limite temporale previsto | 0 / 0 / 0 | 15.469 mm |
| Center, 1.956870983805102 s | 294 | Timeout dello swing | 0 / 1 / 0 | 21.484 mm |
| Late, 2.156870983805102 s | 295 | Timeout dello swing | 0 / 1 / 0 | 27.943 mm |

**Nessun rollout dello smoke ha completato un ciclo valido.** Lo start early
raggiunge cinque secondi con ritorno zero e senza eventi validi: questo mostra
perché la sola durata dell'episodio non basta a certificare il cammino.
Durante la raccolta stocastica ex-novo sono inoltre intervenute protezioni
per penetrazione, documentate nella validazione.

## 4. Baseline qualitativa identificata

Il riferimento proposto è `rl_module_best` del run
`MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01`, corrispondente
all'iterazione logica **47**; il checkpoint finale del run è all'iterazione 51.
La scelta del best come riferimento del benchmark è proposta, non una nuova
selezione già validata su tutti gli start.

Nel [rollout storico centrale del 2 settembre](<../../Trajectory Generator/runs/rollout/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-02_best_rollout/rollout_summary.json>)
questo checkpoint, con azioni deterministiche, ha ottenuto:

- 500 passi e conclusione per limite temporale, senza terminazione di protezione;
- 3 heel-strike, 3 toe-off, 2 cicli validi e 0 eventi invalidi;
- ritorno totale `27.423920791319738`;
- penetrazione massima `24.627298182665364 mm`;
- nessun clipping delle azioni.

Nella ricerca effettuata non sono stati trovati rollout early e late dello
stesso checkpoint. Il riferimento completo sui tre start resta da costruire.
Questi risultati storici e quelli dello smoke non costituiscono ancora un
benchmark a parità di addestramento e protocollo di valutazione.

## 5. Strategia proposta per verificare la parità

### 5.1 Fissare riferimento e condizioni di confronto

Registrare il checkpoint baseline scelto, il suo hash e la configurazione
risolta. Eseguire ciascuna rete tramite il proprio driver, mantenendo gli
artefatti baseline in lettura e scrivendo le nuove valutazioni in directory
dedicate alla rete unificata.

Le condizioni da rendere identiche sono modello, cinematica, GRF, detector,
reset, start, durata, frequenza policy, governor, SEA e protezioni. Eventuali
differenze di configurazione devono essere esplicitate prima di confrontare
i risultati.

### 5.2 Procedere con una matrice di prove comune

1. Completare il riferimento baseline con rollout deterministici di 500 passi
   ai tre start; riprodurre anche il centrale nello stesso protocollo.
2. Addestrare la rete unificata con valutazioni periodiche durante pretraining
   ed ex-novo, conservando checkpoint e metriche delle due fasi.
3. Valutare i candidati sugli stessi tre casi deterministici.
4. Estendere il confronto a prove stocastiche con sigma 0.010 e sequenze di
   rumore accoppiate. Lo stesso seed da solo non garantisce la stessa sequenza
   se i due driver consumano il generatore casuale in modo diverso: va
   verificata o registrata la sequenza effettivamente applicata.
5. Riservare alcuni start alla valutazione finale, senza usarli per selezionare
   o correggere i checkpoint. Questo misura robustezza su condizioni AB06
   diverse da quelle usate nella selezione; non dimostra generalizzazione
   ad altri soggetti o modelli.

Il numero delle repliche e il budget di training non sono ancora fissati.
Ripetere il medesimo rollout deterministico serve alla riproducibilità;
per valutare la variabilità dell'apprendimento servono anche seed di training
indipendenti. Intervalli d'incertezza e repliche sono coerenti con le
raccomandazioni di [Agarwal et al., NeurIPS 2021](https://arxiv.org/abs/2108.13264)
per la valutazione affidabile del reinforcement learning.

### 5.3 Calcolare metriche comuni dai segnali fisici

Il valutatore proposto deve leggere gli output di entrambe le pipeline e
applicare le stesse formule, unità, finestre e regole sui campioni mancanti.
Le tracce policy e gli output `.sto` già contengono gran parte dei dati.

| Aspetto | Metriche da confrontare |
|---|---|
| Continuità del cammino | Episodi completati, durata prima del fallimento, cicli validi, eventi invalidi, motivi di terminazione e timeout |
| Forma knee/ankle | Escursioni, picchi, errore e permanenza nel corridoio; risultati separati per giunto |
| Temporizzazione | Durata del ciclo, stance/swing, tempi di HS e TO |
| Tracking | Actual–served e actual–prescribed sinistro, con posizioni e velocità dello stesso riferimento comune |
| Contatto | Picchi di penetrazione e tempo sopra le soglie, carico, supporto e scivolamento |
| Controllo e supporto biologico | Saturazioni, coppie e potenza SEA, regolarità dei comandi e uso delle reserve |

La cinematica va mostrata sia nel tempo reale, per conservare gli errori di
temporizzazione, sia normalizzata sul ciclo, per confrontare la forma.
L'errore rispetto al prescribed va misurato senza traslazioni temporali
ottimizzate a posteriori nel risultato principale. Un'eventuale analisi del
ritardo resta diagnostica e non introduce compensazione nella V1.

La reward totale è un indicatore secondario. Le loss imitative storiche non
sono direttamente confrontabili con quelle nuove perché la supervisione è
cambiata: per il benchmark serve un riferimento fisico comune.

Per le metriche del corridoio vanno riportati cicli valutabili, campioni
considerati e campioni scartati. Un episodio senza cicli validi non può
ottenere un buon giudizio grazie all'assenza di campioni valutabili. Analogamente,
una penetrazione media inferiore in un episodio terminato presto non dimostra
da sola un miglioramento rispetto a un episodio completo.

### 5.4 Definire il criterio di accettazione prima dei risultati

L'obiettivo operativo è una qualità almeno pari alla baseline entro margini
di regressione prestabiliti. La rete candidata deve conservare completamento
e cicli validi sui casi nominali, rispettare le protezioni e non peggiorare
oltre tali margini cinematica, temporizzazione e carichi.

I margini numerici devono derivare dalle tolleranze del progetto e dalla
variabilità osservata della baseline. **Non sono stati ancora fissati e non
vanno scelti dopo aver visto il risultato della rete candidata.** Le protezioni
fisiche esistenti restano invariate; superare un confronto relativo non
equivale a una certificazione biomeccanica o di impiego reale.

Il report finale dovrà mostrare risultati per start e per giunto, casi peggiori
e incertezza delle differenze. Dati insufficienti per escludere un peggioramento
producono un esito inconclusivo, non una dichiarazione di parità.

### 5.5 Separare qualità finale e costo di apprendimento

Il run baseline comprende 50 aggiornamenti nuovi, alle iterazioni logiche
2–51, ma parte da un actor con una storia precedente di pretraining e
adattamento. La rete unificata parte invece da pesi nuovi.

Occorre riportare transizioni e aggiornamenti delle diverse fasi, oltre al
tempo di calcolo. Un confronto basato soltanto su «50 iterazioni contro 50»
non misura lo stesso investimento di training. Il recupero della qualità
baseline e l'efficienza del nuovo percorso vanno presentati separatamente.

## 6. File introdotti e perimetro delle modifiche

La consegna V1 ha introdotto file esclusivamente sotto
`Trajectory Generator/unified_MLP/`:

- `env.py`, `env_factory.py`, `reward_function.py`: ambiente dedicato,
  supervisione sinistra e separazione degli obiettivi;
- `asymmetric_rl_module.py`, `actor_transition.py`, `training_runtime.py`,
  `runtime_contract.py`: actor, transizione, warm-up, resume e compatibilità;
- `train_ppo_mlp.py`, `rollout_eval.py`, `training_config.py`, `tb_logging.py`,
  `_bootstrap.py`, `__init__.py`: driver e infrastruttura dedicati;
- `configs/pretraining.yaml`, `configs/ex_novo.yaml`, i due preset smoke e
  `configs/provenance.json`: configurazioni e provenienza;
- `tests/`, `validation/check_environment.py`, `README.md`, `VALIDATION.md`:
  test, verifica dinamica e documentazione;
- `runs/`: artefatti locali delle prove, ignorati da Git secondo le regole
  esistenti del repository.

La discussione sulla parità ha comportato lettura degli artefatti e definizione
del protocollo. Con `create_report` viene aggiunto soltanto questo Markdown
in `reports/user/`; nessun nuovo benchmark, training, rollout o modifica
del codice è stato eseguito per redigerlo. I risultati dei test qui riportati
appartengono alla consegna V1 già documentata, non a una nuova esecuzione.

## 7. TODO da propagare nei daily report

- [ ] Fissare checkpoint baseline, manifest degli input e protocollo comune
  di confronto, mantenendo intatti gli artefatti storici.
- [ ] Implementare il valutatore comune sotto `Trajectory Generator/unified_MLP/`,
  con metriche per start e per giunto, copertura dei cicli e gestione esplicita
  dei fallimenti e dei dati mancanti.
- [ ] Completare e riprodurre il benchmark baseline deterministico sui tre start.
- [ ] Definire prima della selezione finale margini di regressione, condizioni
  riservate alla valutazione, numero di repliche e budget di addestramento.
- [ ] Eseguire un vero pretraining e training ex-novo della rete unificata,
  con valutazioni periodiche; attività successiva allo smoke V1, non eseguita.
- [ ] Eseguire il confronto deterministico e stocastico finale, includendo
  variabilità tra seed di training e incertezza delle differenze.
- [ ] Produrre il verdetto quantitativo di parità, regressione o risultato
  inconclusivo, insieme al costo necessario per raggiungere la qualità ottenuta.
- [ ] Eseguire le verifiche live su Windows per attestare anche quella piattaforma.
- [ ] Riprendere actor predittivo e compensazione del ritardo in una versione
  successiva; restano esclusi dal recupero della parità della V1 attuale.

## 8. Riferimenti

- [Guida operativa della rete unificata](<../../Trajectory Generator/unified_MLP/README.md>)
- [Verifiche effettive della V1 e artefatti dello smoke](<../../Trajectory Generator/unified_MLP/VALIDATION.md>)
- [Provenienza dei preset](<../../Trajectory Generator/unified_MLP/configs/provenance.json>)
- [Metadati del checkpoint best baseline](<../../Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/checkpoint_best_meta.json>)
- [Rollout baseline centrale](<../../Trajectory Generator/runs/rollout/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-02_best_rollout/rollout_summary.json>)
- [Progetto iniziale del 31 agosto](2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md): contesto storico; la V1 implementata segue le successive istruzioni dell'utente che rinviano predittività e compensazione del ritardo.
