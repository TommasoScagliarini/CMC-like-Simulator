# Audit del corridor morfologico rispetto alle cinematiche prescribed e V26

**Data:** 2026-08-30  
**Stato:** analisi completata; nessuna modifica applicata a training, reward o profilo  
**Oggetto:** stabilire se le cinematiche V26 siano una conseguenza inevitabile del modello corrente e se il morphology corridor sia stato progettato in modo errato

## 1. Risposta sintetica

L'osservazione visiva è corretta, ma va completata con l'informazione di fase:

> Le cinematiche V26 hanno una forma generale simile alle cinematiche CMC-like
> prescribed, ma le caratteristiche principali sono eseguite troppo tardi nel
> ciclo. Anticipando offline le curve V26 di circa `0.09` cicli per l'ankle e
> `0.106` cicli per il knee, la correlazione con il prescribed sale a circa
> `0.91` per entrambi i giunti.

Con un periodo V26 di circa `1.51 s`, lo sfasamento equivale a circa:

- `135 ms` per l'ankle;
- `160 ms` per il knee.

La conclusione è duplice:

1. **È coerente che il V26 conservi una forma prescribed-like**, perché deriva
   da un warm start imitativo e le 50 iterazioni ex-novo hanno modificato
   pochissimo la policy.
2. **Quella forma ritardata non è imposta dalla dinamica del modello.** Le
   simulazioni CMC-like prescribed mostrano che il modello e il SEA riescono a
   seguire cinematiche dentro il corridor con errori piccoli. Nel V26 la
   cinematica fisica segue molto bene la `served reference`; quindi l'errore è
   già presente nella reference prodotta/consegnata dalla pipeline policy e
   reference governor.

Il corridor non risulta corrotto da errori di segno, unità, lato o sorgente.
È costruito correttamente dagli stessi dati AB06. Tuttavia, **la sua attuale
formulazione operativa come reward è incompleta per il problema corrente**:

- usa un unico profilo medio per cicli con cadenze molto diverse;
- valuta una reference soggetta a ritardo con un target phase-locked;
- la reward causale viene consegnata con un ulteriore ritardo esplicito;
- la loss è nulla dentro la banda e reagisce soltanto dopo l'uscita;
- il solo aumento del peso non corregge né lo sfasamento né la scarsa
  esplorazione/variazione della policy.

Pertanto il verdetto corretto non è «il corridor è completamente sbagliato»,
ma:

> Il profilo è valido come envelope descrittivo e diagnostico AB06; il contratto
> con cui viene usato come obiettivo causale di training non è adeguatamente
> condizionato per cadenza e latenza.

## 2. Domande esaminate

L'audit ha separato quattro ipotesi che dai soli grafici possono sembrare
equivalenti:

1. il modello corrente non è fisicamente capace di seguire il corridor;
2. il corridor usa segni, unità, lato o dati sbagliati;
3. il V26 genera una forma plausibile, ma sfasata rispetto agli eventi HS/TO;
4. il corridor medio non rappresenta correttamente il regime di cadenza del
   rollout corrente.

I risultati escludono le prime due ipotesi come causa principale e confermano
una combinazione delle ultime due.

## 3. Dati confrontati

### 3.1 Rollout ex-novo V26

Sono stati confrontati i rollout deterministici dei quattro training, tutti
derivati dallo stesso candidato J20:

| Peso morphology | Rollout |
|---:|---|
| `0.0025` | `Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_pilot_2026-08-28_best_rollout` |
| `0.025` | `Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_morphology_0p025_50updates_2026-08-28_best_rollout` |
| `0.10` | `Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_morphology_0p10_50updates_2026-08-28_best_rollout` |
| `1.05` | `Trajectory Generator/runs/rollout/MLP_ExNovo_V26C_J20_morphology_1p05_50updates_2026-08-29_best_rollout` |

Il confronto principale usa la `served reference`, cioè il segnale realmente
valutato dalla reward morfologica, e la cinematica fisica registrata dal
simulatore.

### 3.2 Simulazioni CMC-like prescribed

Sono stati usati due livelli di evidenza:

1. simulazione completa con reference prescribed:
   - `results/sim_output_kinematics.sto`;
   - `results/sim_output_gait_events.csv`;
   - quattro cicli protesici sinistri completi;
2. smoke più recente con controller prescribed e reference esplicita:
   - `results/hybrid_profile_ab_residual_tangent_v2/sim_output_kinematics.sto`;
   - `results/hybrid_profile_ab_residual_tangent_v2/sim_output_kinematics_reference.sto`.

Il secondo artefatto dura soltanto `1.999 s` e non contiene un intero ciclo
sinistro: è stato usato esclusivamente per verificare la fattibilità del
tracking, non per l'analisi morfologica phase-normalized.

### 3.3 Profilo morphology

Profilo analizzato:

`Trajectory Generator/baseline_MLP/morphology_profiles/ab06_prosthetic_event_warped_mean_std_corridor.json`

Sorgenti dichiarate e verificate nei metadati:

- IK: `models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`;
- GRF: `models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot`;
- lato protesico: sinistro;
- lato GRF per HS/TO: sinistro;
- unità runtime: radianti;
- ankle: segno OpenSim raw, positivo in dorsiflessione;
- knee: segno OpenSim raw, invertito soltanto nei plot flexion-positive;
- cicli inclusi: `123`;
- mapping: stance `HS->TO` e swing `TO->HS` riscampionati separatamente;
- fase TO canonica: `alpha = 0.6223299989`.

Non è emerso alcun mismatch di soggetto, laterality, unità o segno.

## 4. Metodo di confronto

Per evitare che durate diverse alterassero il confronto, ciascun ciclo completo
è stato riscampionato su una griglia comune di 201 punti:

- stance reale `HS->TO` mappata su `[0, alpha]`;
- swing reale `TO->HS` mappato su `[alpha, 1]`;
- curve medie calcolate sui cicli completi disponibili;
- correlazione e RMSE calcolate nel segno OpenSim raw;
- ricerca dello shift ottimo eseguita tra `-0.25` e `+0.25` cicli con passo
  `0.001`.

Sono inoltre stati calcolati:

- errore di tracking tra prescribed e cinematica fisica;
- errore di tracking tra V26 served e cinematica fisica;
- frazione fuori corridor con mapping retrospettivo esatto;
- statistiche separate dei 123 cicli sorgente in tre fasce di periodo.

I valori indicati «al TO» nelle tabelle phase-normalized sono valutati sul
punto della griglia `phase = 0.620`, molto vicino all'ancora esatta
`0.6223299989`. Il profilo JSON conserva anche l'ancora esatta.

## 5. Il modello corrente può seguire il prescribed

### 5.1 Errori fisici di tracking

| Esperimento | Knee RMSE | Ankle RMSE |
|---|---:|---:|
| CMC-like completo: actual vs IK prescribed | `0.00310 rad` (`0.178°`) | `0.02020 rad` (`1.158°`) |
| Prescribed controller recente: actual vs reference | `0.00439 rad` (`0.251°`) | `0.00546 rad` (`0.313°`) |
| V26 peso `1.05`: actual vs served | `0.00995 rad` (`0.570°`) | `0.00881 rad` (`0.505°`) |

Il test prescribed recente raggiunge praticamente l'intero intervallo ankle
richiesto:

| Segnale | Minimo | Massimo |
|---|---:|---:|
| reference | `-0.10935 rad` | `+0.39159 rad` |
| actual | `-0.11210 rad` | `+0.39200 rad` |

Questo esclude che la mancata plantarflessione V26 sia necessaria per limiti
meccanici del modello o del SEA. Il modello può raggiungere e seguire una
reference plantarflessa quando tale reference gli viene servita al momento
corretto.

### 5.2 Prescribed rispetto al corridor

Sui quattro cicli completi della simulazione prescribed, usando il mapping
retrospettivo esatto:

| Segnale | Knee fuori corridor | Ankle fuori corridor | Escursione ankle massima |
|---|---:|---:|---:|
| IK prescribed | `0.00%` | `3.48%` | `0.05497 rad` (`3.15°`) |
| cinematica fisica CMC-like | `0.00%` | `1.99%` | `0.02779 rad` (`1.59°`) |

La correlazione tra la curva media fisica prescribed e la media globale del
corridor è:

- knee: `0.953`;
- ankle: `0.912`.

Il prescribed lento non coincide con la media globale, ma è quasi interamente
contenuto nella banda configurata. Questo dimostra che il corridor non è
intrinsecamente incompatibile con il modello corrente.

## 6. La somiglianza V26-prescribed è reale, ma nasconde uno sfasamento

Prendendo il V26 con peso `1.05`:

| Giunto | Correlazione senza shift | Shift che minimizza RMSE | Correlazione dopo shift |
|---|---:|---:|---:|
| knee | `0.598` | anticipo V26 di `0.106` cicli | `0.911` circa |
| ankle | `0.617` | anticipo V26 di `0.089` cicli | `0.911` circa |

Quindi la percezione visiva dell'utente è fondata: le curve appartengono alla
stessa famiglia di forme. Tuttavia il corridor è legato agli eventi del passo,
e una forma corretta eseguita `135-160 ms` più tardi non è biologicamente
equivalente. In particolare il timing del toe-off determina quando la caviglia
deve produrre plantarflessione e potenza.

### 6.1 Estremi di fase

| Segnale medio | Picco dorsiflessione ankle | Minimo ankle | Massima flessione knee, segno raw |
|---|---:|---:|---:|
| CMC-like prescribed actual | fase `0.460` | fase `0.635` | fase `0.685` |
| V26 served, peso `1.05` | fase `0.530` | fase `0.825` | fase `0.815` |

La caviglia V26 non si limita a ridurre l'ampiezza: mantiene la dorsiflessione
attraverso la regione del toe-off e porta il minimo molto più avanti nello
swing.

### 6.2 Valore ankle in prossimità del TO

| Curva | Ankle a fase `0.620` |
|---|---:|
| media globale del profilo | `-0.254 rad` circa |
| CMC-like prescribed actual | `-0.124 rad` |
| media dei cicli sorgente lenti | `-0.082 rad` |
| V26 served, peso `1.05` | `+0.128 rad` |

Poiché il segno positivo è dorsiflessione, la differenza prescribed-V26 è
circa `0.253 rad`, cioè `14.5°`, e ha anche segno funzionale opposto al TO.

All'ancora TO esatta, la media globale del profilo è `-0.2622 rad`; con la
configurazione ankle `0.6 sigma + 7.5°`, il bordo superiore è circa
`-0.0623 rad`. La reference V26 rimane quindi chiaramente oltre il bordo anche
tenendo conto della larghezza del corridor.

## 7. Perché aumentare il peso non ha corretto il problema

Tra il training base (`0.0025`) e il training con peso `1.05`, cioè con un peso
420 volte maggiore, le curve medie sono quasi identiche:

| Giunto | RMSE tra le due served | Correlazione |
|---|---:|---:|
| knee | `0.000477 rad` (`0.027°`) | `0.9999989` |
| ankle | `0.000654 rad` (`0.037°`) | `0.9999891` |

La ricostruzione del plot 8 per il peso `1.05` ha confermato:

| Giunto | Campioni fuori | Frazione fuori | Massima escursione |
|---|---:|---:|---:|
| knee | `10/372` | `2.69%` | `0.07922 rad` |
| ankle | `62/372` | `16.67%` | `0.20247 rad` |

Il corridor ricostruito coincide con i bound salvati nel trace entro
`2.22e-16 rad`; non si tratta quindi di un errore del plot.

Un peso più grande moltiplica una loss già calcolata, ma non cambia:

- il momento in cui la reference viene prodotta;
- la dinamica del reference governor `butterworth3_jerk_limited`;
- il ritardo esplicito di consegna della reward morfologica, configurato a
  `0.04 s`;
- il fatto che la loss sia esattamente zero dentro la banda;
- la distribuzione di esplorazione e la piccola variazione effettiva della
  policy nelle 50 iterazioni.

L'aumento del peso era dunque un test utile: ha falsificato l'ipotesi che il
problema fosse soltanto una pressione morfologica troppo debole.

## 8. Limite reale del corridor: dipendenza dalla cadenza

Il profilo unico aggrega cicli molto eterogenei:

| Statistica periodo dei 123 cicli | Valore |
|---|---:|
| minimo | `0.879 s` |
| media | `1.128 s` |
| mediana | `1.022 s` |
| 95° percentile | `1.582 s` |
| massimo | `1.669 s` |

Il rollout V26 e la simulazione prescribed completa operano invece nel regime
lento, circa `1.5-1.6 s` per ciclo. Separando i dati sorgente:

| Regime | Cicli | Periodo medio | Ankle vicino al TO | Minimo ankle medio |
|---|---:|---:|---:|---:|
| `< 1.2 s` | `75` | `0.964 s` | `-0.318 rad` | `-0.390 rad` |
| `1.2-1.4 s` | `26` | `1.239 s` | `-0.212 rad` | `-0.276 rad` |
| `1.4-1.7 s` | `22` | `1.557 s` | `-0.082 rad` | `-0.095 rad` |

Il profilo globale è numericamente dominato dai `75` cicli veloci. La
plantarflessione AB06 cambia quindi fortemente con la cadenza: chiedere a un
ciclo da `1.55 s` di inseguire la media di tutti i cicli introduce un bias.

Questo è un limite di progettazione reale: **un singolo corridor phase-only non
è sufficiente a descrivere l'intera distribuzione AB06**. Occorre almeno
condizionarlo sul periodo, oppure su periodo di stance e swing.

Tuttavia questo risultato non assolve la V26 corrente. Anche rispetto alla
media lenta (`-0.082 rad` vicino al TO), la V26 è a `+0.128 rad`: resta una
differenza di circa `0.210 rad` (`12.1°`) e di segno funzionale errato. La
cadenza spiega una parte del mismatch con la media globale, non lo sfasamento
V26 completo.

## 9. Il corridor è stato progettato male?

### 9.1 Componenti risultate corrette

- soggetto e modello: AB06;
- lato protesico e GRF: sinistro;
- coordinate e unità: coerenti con OpenSim runtime;
- segno ankle: positivo in dorsiflessione sia nel profilo sia nella served;
- inversione knee: solo cosmetica nei plot;
- costruzione event-warped HS->TO e TO->HS: deterministica e già validata;
- corridor runtime nel plot: ricostruzione numericamente esatta;
- fattibilità fisica: dimostrata dalle simulazioni prescribed.

Quindi non vi è evidenza per scartare il profilo come artefatto errato.

### 9.2 Componenti non adeguate all'uso di training attuale

1. **Profilo non condizionato sulla velocità/cadenza.** Una sola media combina
   forme ankle significativamente diverse.
2. **Obiettivo phase-locked contro una catena ritardata.** Il corridor valuta
   ciò che dovrebbe avvenire ora, mentre la forma V26 arriva circa
   `135-160 ms` più tardi.
3. **Reward causale ritardata.** La modalità corrente è
   `event_anchored_causal_delayed_experimental` con `0.04 s` di delay.
4. **Segnale tardivo al bordo.** Dentro la banda la loss è zero; il gradiente
   compare solo dopo l'uscita.
5. **Nessuna separazione tra target actor e reference servita.** Il training
   non distingue chiaramente errore della policy, ritardo del governor ed
   errore fisico del SEA.
6. **Peso globale knee+ankle.** Il problema dominante è l'ankle al toe-off, ma
   il moltiplicatore agisce sulla loss combinata.

Per questo il corridor è appropriato come **criterio diagnostico finale**, ma
la reward va riprogettata prima di continuare a scalarne il peso.

## 10. Strategia raccomandata

### Fase A — non modificare ancora la rete o il warm start

Il prescribed dimostra che il target è realizzabile dal modello corrente.
Rifare subito il warm start non risolverebbe il disallineamento del contratto
di fase/reward. Anche aumentare ulteriormente il peso globale non è supportato
dai risultati.

### Fase B — audit offline del target morphology

Costruire e confrontare, senza training:

1. corridor globale attuale;
2. corridor del regime lento `1.4-1.7 s`;
3. corridor condizionato in modo continuo su periodo, stance e swing.

Ogni candidato deve essere validato su cicli held-out per verificare copertura,
non soltanto aderenza ai dati usati per costruirlo. Non bisogna allargare la
banda a posteriori solo per fare entrare la V26.

### Fase C — separare geometria finale e segnale apprendibile

Mantenere due misure diverse:

- **acceptance metric:** served reference contro corridor retrospettivo esatto
  HS->TO->HS, senza possibilità di alterare la reward passata;
- **training signal:** target actor o served predetta contro un corridor
  anticipato/condizionato, con credito assegnato alla transizione che ha
  originato la reference.

La misura finale deve restare event-aligned: spostare semplicemente il
corridor in avanti per accettare una plantarflessione tardiva renderebbe il
gate biologico meno significativo.

### Fase D — quantificare la latenza prima di un nuovo training lungo

Eseguire ablation a policy congelata:

1. misurare separatamente actor target -> served reference;
2. misurare served reference -> coordinata fisica;
3. ripetere con reference governor invariato e con parametri di cutoff/slew
   ankle controllati;
4. eseguire uno sweep di anticipo `0-180 ms` sul solo calcolo della loss;
5. verificare se l'anticipo che minimizza l'errore coincide con i `135-160 ms`
   osservati.

Il tracking served -> actual è già piccolo; l'attenzione principale deve quindi
andare alla catena actor target -> served e all'assegnazione temporale della
reward.

### Fase E — pilot brevi e preregistrati

Solo dopo A-D:

1. partire dallo stesso checkpoint J20 per mantenere il confronto controllato;
2. usare una loss ankle-specific e cadence-conditioned;
3. introdurre una fascia interna soft, così il gradiente compare prima del
   bordo;
4. usare un segnale predittivo/phase-lead compensato per il governor;
5. eseguire pilot da `5-10` iterazioni prima di un nuovo run da `50`;
6. selezionare i checkpoint usando metriche di toe-off e corridor, non solo il
   return totale.

Metriche minime da preregistrare:

- ankle al TO e differenza dal profilo del regime corretto;
- fase del massimo e del minimo ankle;
- frazione e massima escursione fuori corridor;
- RMSE actor target -> served;
- RMSE served -> actual;
- return, lunghezza episodio, penetrazione GRF e validità FSM;
- confronto con J20 e con tutti i V26 già eseguiti.

## 11. Limiti dell'inferenza

Il prescribed e il corridor derivano dallo stesso dataset AB06. Il loro accordo
non è quindi una validazione biologica esterna indipendente; dimostra due cose
più circoscritte ma decisive:

1. coerenza interna tra profilo AB06 e prescribed AB06;
2. capacità del modello corrente di realizzare quelle cinematiche.

Per affermare che il corridor sia universalmente biologico serviranno dati
held-out e, in futuro, la generalizzazione al dataset EPIC già prevista nella
roadmap. Questo limite non cambia la diagnosi sul V26: il suo toe-off resta
tardivo anche rispetto al sottogruppo AB06 lento più pertinente.

Inoltre, le percentuali retrospettive esatte non devono essere confrontate
direttamente con quelle del plot 8 causale. Il plot 8 usa la fase e la
disponibilità runtime delayed; l'audit offline usa segmenti completi e conosce
gli endpoint futuri. La discrepanza tra le due misure è precisamente uno dei
motivi per cui diagnostica finale e reward online devono essere separate.

## 12. Verifiche eseguite

- audit dei metadati e della provenance del profilo morphology;
- verifica di laterality, unità e convenzioni di segno;
- confronto actual-vs-prescribed sulla simulazione completa;
- confronto actual-vs-reference sul controller prescribed recente;
- confronto actual-vs-served per tutti e quattro i rollout V26;
- event warping retrospettivo dei quattro cicli prescribed completi;
- event warping dei cicli V26 completi disponibili nel trace;
- sweep di phase shift con passo `0.001` cicli;
- stratificazione dei 123 cicli sorgente in tre regimi di periodo;
- rigenerazione diagnostica del plot 8 del run `morphology_weight=1.05`;
- verifica dei bound del corridor contro il trace entro roundoff
  (`<= 2.22e-16 rad`).

## 13. File modificati

Nell'audit iniziale era stato aggiunto soltanto il presente report:

`reports/user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md`

Il successivo addendum frozen-policy ha aggiunto:

- `validation/audit_v26_frozen_policy_latency.py`;
- `validation/v26_reference_latency_audit/2026-08-30_frozen_policy_latency_decomposition.json`;
- `plot/08_30_2026_v26_reference_latency_audit/01_frozen_policy_latency_decomposition.png`.

Non sono stati modificati:

- codice Python di runtime o training;
- configurazioni YAML/JSON;
- checkpoint;
- profilo morphology;
- plugin C++;
- semantica del SEA;
- training o rollout esistenti.

## 14. Riferimenti storici

- [Integrazione iniziale del morphology corridor](2026-06-29_integrazione_corridor_morphology_reward_exnovo.md)
- [Implementazione e validazione event-anchored](2026-07-20_corridor_morfologico_event_anchored_implementazione_validazione.md)
- [Analisi retrospettiva e limite causale del corridor](2026-07-20_corridor_morfologico_retrospettivo_sperimentale.md)
- [Guida della pipeline V26/J20](2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md)
- [Rollout base V26, plot canonici e plot 8](2026-08-28_rollout_best_exnovo_v26c_j20_plot_canonici_e_corridor_morfologico.md)

## 15. Decisione architetturale proposta

La decisione suggerita, da sottoporre ad approvazione prima di modificare il
training, è:

> Conservare il corridor attuale come baseline diagnostica; non aumentare
> ulteriormente il peso; progettare un corridor cadence-conditioned e un
> segnale di training predittivo che compensi la latenza, mantenendo come gate
> finale la served reference valutata con phase retrospettiva esatta.

Questa strategia corregge il contratto causale senza trasformare il corridor in
una banda costruita a posteriori per accettare il comportamento corrente.

## 16. Addendum: scomposizione della latenza con policy congelata

Dopo il report iniziale è stata eseguita la Fase D proposta, senza rilanciare
PPO e senza modificare la policy. Per ognuno dei quattro rollout deterministici
sono stati confrontati, allo stesso timestamp di fine step da `10 ms`:

1. target raw dell'actor convertito in radianti;
2. target dopo lo slew limiter;
3. reference servita dal filtro C2 di terzo ordine a `4 Hz`;
4. coordinata fisica OpenSim.

### Ritardi diretti della catena

I risultati sono identici, alla risoluzione di `10 ms`, per tutti e quattro i
pesi morphology:

| Tratto | Knee | Ankle |
|---|---:|---:|
| actor raw -> dopo slew | `10 ms` | `20 ms` |
| dopo slew -> served | `80 ms` | `80 ms` |
| served -> coordinata fisica | `0 ms` | `0 ms` |
| actor raw -> served complessivo | `90 ms` | `100 ms` |

I limiti hard del governor su velocità, accelerazione e jerk non sono mai stati
attivati. Gli `80 ms` sono quindi associati alla dinamica low-pass del filtro,
non a saturazioni hard.

### Ritardo rispetto al prescribed

La scomposizione phase-normalized rispetto alla cinematica fisica prescribed
ha prodotto:

| Contributo diagnostico | Knee | Ankle |
|---|---:|---:|
| ritardo già presente nel target actor | `62.0 ms` | `25.7 ms` |
| quota aggiunta dallo slew | `15.1 ms` | `30.3 ms` |
| quota aggiunta dal filtro | `83.2 ms` | `78.7 ms` |
| quota fisica successiva | `0 ms` | circa `0 ms` |

Le differenze sono ottenute da shift ottimizzati separatamente e vanno lette
come decomposizione diagnostica, non come identità causale perfettamente
additiva. Il confronto diretto nel tempo conferma però lo stesso ordine e
localizza robustamente la quota dominante nel filtro.

### Evidenza specifica al toe-off

Per il run con peso `1.05`, vicino al TO:

| Segnale ankle | Valore |
|---|---:|
| prescribed CMC-like actual | `-0.1242 rad` |
| actor raw target | `-0.1276 rad` |
| target dopo slew | `-0.0222 rad` |
| served dopo filtro | `+0.1284 rad` |
| coordinata fisica | `+0.1273 rad` |

Il target raw dell'actor è quindi già praticamente corretto al TO. La perdita
della plantarflessione avviene soprattutto nella trasformazione actor -> slew
-> filtro. La coordinata fisica riproduce fedelmente la served errata.

### Conseguenza sulla reward

Questo addendum precisa la decisione architetturale:

- il corridor finale non deve essere spostato;
- la penalità served-vs-corridor resta una metrica finale corretta;
- per il training va corretta l'assegnazione temporale della penalità;
- il campione morphology è inoltre emesso `40 ms` dopo il proprio timestamp,
  quindi la loss viene oggi associata a una transizione PPO successiva;
- un segnale ausiliario dovrebbe valutare la served futura predetta dal
  governor oppure riassegnare la loss alla transizione che l'ha originata.

L'audit non autorizza ancora una specifica implementazione. Stabilisce però
che aumentare nuovamente il peso sulla loss servita e ritardata non affronta la
causa dominante.

### Verifiche dell'addendum

- quattro rollout su quattro con la stessa decomposizione temporale;
- `py_compile` dello script diagnostico: PASS;
- JSON verificato con assertion strutturali e di stabilità dei lag: PASS;
- plot PNG `2160x1620` generato e ispezionato visivamente: PASS;
- nessun training eseguito;
- nessun checkpoint, profilo o file runtime modificato.
