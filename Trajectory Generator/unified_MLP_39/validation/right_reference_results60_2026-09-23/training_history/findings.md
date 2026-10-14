# RIGHT39 fino a 60: storico, integrità della ripresa e confronto per esperienza

Analisi offline del 23 settembre 2026. Il run mantiene il nome originale `MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23`, ma contiene 60 iterazioni. Non sono stati avviati training o rollout né modificati codice di produzione, FSM, reward o dati storici. Questo memo riguarda il training; la valutazione dei checkpoint congelati è separata.

## Risultato principale

La continuazione concordata 40→60 è completata correttamente. Il nuovo best secondo il criterio di selezione del trainer è **58**, return medio 251,9742 e durata media 500 step; la 60 ha return 243,6451 e durata 500. Il budget totale è 245.760 transizioni. Il processo è durato 2 h 38 min 40 s, dalle 19:47:28 alle 22:26:11 locali, senza riavvii, crash o iterazioni saltate.

Nel nuovo segmento si completano **153/157 episodi, 97,45%**. Le quattro terminazioni anticipate sono penetrazioni ordinarie, una nelle iterazioni 54, 56, 57 e 59; nessuna penetrazione hard o timeout registrato. La coda non è perfetta: nelle ultime cinque iterazioni i completi sono **37/40, 92,5%**. Questa proporzione è nettamente superiore alla LEFT39 allo stesso budget, ma non dimostra da sola equivalenza qualitativa o statistica con V26.

## Conteggio corretto e media dei return

Le cause di fine episodio nei JSONL sono contatori cumulativi. Per una finestra [a,b] si usa `counter(b) - counter(a-1)`, anche attraversando la ripresa da 40. Non si sommano i valori grezzi dei contatori.

Le iterazioni **41, 43, 46, 49 e 51** non terminano episodi e hanno return/durata nativi null. Le 13 simulazioni fisiche ripartono al resume; ciascun batch per worker è più corto dei 500 step di un episodio, e i worker inizialmente sincronizzati possono concludere tutti insieme in un batch successivo. Questi null non sono perdite di telemetria. I valori proiettati per la console non vengono usati né per le medie scientifiche né per il best: in particolare, il valore proiettato 254,2626 della 49 **non** supera scientificamente il best 58.

Le medie aggregate riportate sotto sono `sum(episode_start_return_sum)/sum(episode_start_episode_count)` e analogamente per la durata. I callback includono i frammenti precedenti di ciascun episodio. I conteggi coincidono esattamente con le differenze delle cause di termine in tutte le finestre analizzate. Le somme provengono dai scalar TensorBoard in float32; l'ultima cifra non va interpretata come precisione fisica.

La media aritmetica dei return per iterazione darebbe lo stesso peso a una iterazione con un solo fallimento (54) e a una con undici episodi completi (58). Non è la media degli episodi. Anche moltiplicare sempre la media nativa per il numero di nuove terminazioni non ricostruisce necessariamente la somma dei callback negli storici LEFT/V26: alcune medie native aggregano finestre differenti. Lo script conserva entrambe le stime e la loro discrepanza; usa le somme esplicite dei callback per le tabelle. Per esempio, LEFT56–60 differisce di circa 2 punti di return e 3,41 step tra le due stime.

| Finestra RIGHT39 | Completi / terminati | Completamento | Return per episodio | Durata per episodio |
|---|---:|---:|---:|---:|
| 36–40 | 39/43 | 90,70% | 203,92 | 4,890 s |
| 41–50 | 78/78 | 100,00% | 232,54 | 5,000 s |
| 51–60 | 75/79 | 94,94% | 237,48 | 4,881 s |
| 56–60 | 37/40 | 92,50% | 234,66 | 4,812 s |
| 41–60 | 153/157 | 97,45% | 235,03 | 4,940 s |

La media semplice sulle 15 iterazioni 41–60 con metrica disponibile è invece 226,45 punti e 4,750 s: è una quantità diversa e non va presentata come media di 157 episodi. Le durate in secondi usano 0,01 s/step.

## Confronto a esperienza quasi uguale

RIGHT60 e LEFT60 hanno 245.760 transizioni. Il V26 più vicino è l'iterazione **53**, con 245.132, soltanto 628 in meno (0,256%). Le finestre finali di cinque, dieci e venti iterazioni hanno rispettivamente 20.480, 40.960 e 81.920 transizioni in tutte e tre le condizioni. Questo confronto evita di equiparare impropriamente 60 iterazioni RIGHT e 60 V26, poiché V26 aveva raccolto più esperienza nelle iterazioni iniziali.

| Finestra | RIGHT39 | LEFT39 | V26 storico |
|---|---:|---:|---:|
| Ultime 5: iterazioni | 56–60 | 56–60 | 49–53 |
| Completi | **37/40 (92,50%)** | 25/44 (56,82%) | 40/41 (97,56%) |
| Return per episodio | 234,66 | 219,97 | 233,85 |
| Durata per episodio | 4,812 s | 4,395 s | 4,967 s |
| Ultime 10: completi | **75/79 (94,94%)** | 54/90 (60,00%) | 80/83 (96,39%) |
| Return per episodio | 237,48 | 220,34 | 227,54 |
| Durata per episodio | 4,881 s | 4,499 s | 4,956 s |
| Ultime 20: completi | **153/157 (97,45%)** | 100/174 (57,47%) | 158/167 (94,61%) |
| Return per episodio | 235,03 | 213,98 | 220,35 |
| Durata per episodio | 4,940 s | 4,401 s | 4,908 s |

Nelle ultime cinque: RIGHT ha 3 penetrazioni ordinarie; LEFT 18 ordinarie e 1 hard; V26 1 timeout stance. Nelle ultime venti: RIGHT 4 penetrazioni; LEFT 70 ordinarie, 2 hard, 1 divergenza articolare e 1 `invalid_binary`; V26 8 penetrazioni e 1 timeout stance. I tipi di termine e la gestione degli errori storici non sono perfettamente omogenei tra V26 e implementazione attuale.

Il best entro questo budget è RIGHT58 (251,97, 237.568 transizioni), LEFT57 (243,06, durata 448,86 step) e V2651 (241,66, durata 500). Il trainer seleziona il checkpoint aggiornato con il return raccolto prima dell'aggiornamento PPO: il best di training richiede comunque valutazione separata a policy congelata.

## Entropia, critic, KL e obiettivo appreso

| Ultime 5 a budget equivalente | RIGHT39 | LEFT39 | V26 |
|---|---:|---:|---:|
| Entropia | 1,6541 | 2,1336 | 1,9260 |
| Varianza spiegata critic | 0,2947 | 0,3852 | 0,2087 |
| Value loss | 2,7915 | 2,7109 | 2,7615 |
| KL medio | 0,01120 | 0,00931 | 0,01186 |

Il critic rimane rumoroso e spiega solo una parte della variabilità dei return; i dati non mostrano un fallimento numerico o un'esplosione della KL. La varianza spiegata del critic non misura direttamente robustezza fisica. Le componenti policy e value restano finite e cambiano a ogni iterazione; attore e critic vengono aggiornati.

L'entropia RIGHT continua a diminuire (1,8767 nelle 36–40 → 1,6541 nelle 56–60). Quindi la maggiore percentuale di completamento osservata durante training non dimostra da sola robustezza a una perturbazione assoluta fissa: può riflettere insieme una policy migliore e una diversa distribuzione di esplorazione. Non sono stati salvati scalar training delle due deviazioni standard articolari. Lo script fornisce soltanto il proxy geometrico `exp((H-log(2*pi*e))/2)`, appropriato alla gaussiana bidimensionale nei suoi limiti: 0,5533 per RIGHT, 0,7032 per LEFT e 0,6339 per V26 nelle ultime cinque. Non è una sigma specifica di ankle/knee né la deviazione dei comandi fisici filtrati.

La telemetria delle loss suggerisce un compromesso ancora aperto:

| Media mobile TensorBoard | RIGHT36–40 | RIGHT56–60 | V2649–53 |
|---|---:|---:|---:|
| Loss posizione knee normalizzata | 0,04412 | **0,02281** | 0,03416 |
| Loss posizione ankle normalizzata | 0,04625 | **0,02548** | 0,03253 |
| Loss velocità knee normalizzata | 0,02952 | 0,01794 | 0,02404 |
| Loss velocità ankle normalizzata | 0,05962 | 0,04310 | 0,05710 |
| Score imitazione effettiva | 0,71161 | **0,80886** | 0,75903 |
| Score imitazione riferimento servito | 0,58349 | 0,70747 | 0,63822 |
| Loss penetrazione | 0,001508 | **0,0000966** | 0,001098 |
| Loss inversione momento caviglia | 0,12695 | **0,54419** | 0,22386 |
| Loss variazione comandi | 3,45218 | 3,44717 | 3,47282 |

Le loss di posizione RIGHT diminuiscono di circa 48,3% al ginocchio e 44,9% alla caviglia. Ma **la loss di inversione del momento alla caviglia cresce di 4,29 volte**, mentre la loss di variazione dei comandi è sostanzialmente ferma. Questo è un segnale concreto per verificare le coppie e la regolarità dei nuovi rollout: un obiettivo imitativo migliore non garantisce coppie migliori. Non è, da solo, una misura del picco SEA né una prova di causa meccanica. Nel sidecar congelato il peso della loss di inversione è 0,1 e quello della variazione comandi è 0,05: l'inversione è dunque penalizzata, non ignorata. La sua penalità pesata media sale da circa 0,01270 a 0,05442 mentre lo score imitativo cresce; questo è compatibile con un compromesso dell'obiettivo, senza dimostrare da solo quale meccanismo fisico lo produca.

Le loss sono statistiche mobili sugli stati visitati, non errori misurati sulle stesse identiche traiettorie. LEFT usa target e range differenti: il confronto diretto delle sue loss normalizzate non equivale a confrontare errori in gradi. RIGHT e V26 condividono il riferimento destro, ma continuano a visitare stati diversi.

## Audit della ripresa

Tutti i controlli implementati in `analyze.py` sono passati:

- 145 input congelati, 36 file checkpoint40, 61 file archivio pre-resume e 36 file checkpoint60 corrispondono ai rispettivi manifest SHA-256; anche i manifest originali corrispondono alla ricevuta di congelamento.
- Il prefisso originale del JSONL delle iterazioni 1–40 è identico byte per byte; le righe decodificate coincidono con l'archivio.
- Attore, mean head e critic ripristinati dalla 40 coincidono con l'inizio della 41; nessuna reinizializzazione dell'attore.
- Tutti i 12 stati Adam sono collegati ai parametri live, con momenti e digest identici all'origine. Step Adam: **3240 alla 40 → 3321 alla 41 → 4860 alla 60**.
- Stato adattivo PPO ripristinato dal sidecar; nessun riavvio o salto. KL coefficient nel segmento 41–60 circa 0,2; learning rate 0,0001 ed entropy coefficient 0.
- Tutte le 245.760 transizioni hanno target/clock visibili. Nel nuovo segmento: 81.920 righe osservate, zero righe mancanti, 170 righe con evento di resync, zero righe con timeout stance. Le 170 righe non equivalgono a 170 episodi e non certificano né escludono cicli validi.

Il ripristino è completo per lo stato di apprendimento, ma non per lo stato fisico delle simulazioni o una prosecuzione bit-identica delle RNG. La sincronizzazione degli episodi dopo il resume è un effetto della ripartenza fisica, non un reset di Adam.

## Limiti per spiegare causalmente la differenza

L'insieme dei dati sostiene che **39D può apprendere da zero una policy con completamento di training vicino al V26 usando il riferimento destro**. Non emerge un'incapacità generale della rete a 39 ingressi o del contenitore attuale ad apprendere. Questo rafforza l'utilità del test sul riferimento, ma non identifica da solo una singola causa della precedente inferiorità LEFT.

L'intervento destro cambia congiuntamente forma d'onda prescribed, clock e normalizzazione derivata dai range. Inoltre LEFT25→40 subì il difetto di restore Adam documentato il 22 settembre: dalla 25 con step2025 si ripartì alla 26 con step81; alla 40 Adam era a1215. RIGHT40→60 conserva invece tutti gli stati. LEFT40→60 ebbe inoltre l'interruzione diagnostica dopo la 42 e un resume successivo. Questi fattori impediscono di attribuire tutta la differenza al solo lato dei dati. Già prima della 25 il vantaggio RIGHT sul completamento era presente nei confronti precedenti, ma un solo seed non consente una decomposizione causale quantitativa.

Per V26 restano differenze di FSM, semantica/ordine degli ingressi e gestione dei fallimenti: alcuni worker fatalmente interrotti nello storico possono non contribuire ai contatori delle terminazioni. I completamenti non attestano due cicli validi, assenza di piccole penetrazioni, qualità cinematica o qualità delle coppie. I nuovi rollout deterministici/stocastici sono quindi necessari per interpretare il progresso del training e sono analizzati separatamente dal responsabile del report.

Nessun intervallo di equivalenza o test di superiorità viene calcolato trattando questi episodi come repliche indipendenti: provengono da un solo training seed per condizione e da un unico start, e attraversano aggiornamenti di policy.

## Riproduzione e fonti

Eseguire, dalla root del repository:

```sh
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/unified_MLP_39/validation/right_reference_results60_2026-09-23/training_history/analyze.py'
```

Output locali: `metrics.json` (finestre, audit, metodi e limiti), `iterations.csv` (righe 1–60 dei tre training), `tensorboard_selected.json`, `input_hashes.json`. Grafici in root: `plot/09_23_2026_right60_comparison/training_by_experience.png` e `imitation_and_contact_losses.png`; questi sono confronti di training, non i sette plot canonici dei rollout.

Input principali:

- `unified_MLP_39/runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23/`: `train_iterations.jsonl`, `summary.json`, TensorBoard e checkpoint40/41/60.
- `unified_MLP_39/runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21/`: storico e TensorBoard, troncati alla 60 per il confronto.
- `runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/`: storico e TensorBoard; il budget più vicino è53.
- `unified_MLP_39/runs/plans/right_reference_resume60_2026-09-23/`: `status.json`, manifest, ricevuta, archivio `pre_resume_40`, `resume_verified.json`, `optimizer_progress.json`.
- `unified_MLP_39/tb_logging.py:on_episode_end` per la semantica delle somme complete; `train_ppo_mlp.py:_console_episode_metrics` per la distinzione tra valori nativi e proiezioni.
- `unified_MLP_39/validation/right_reference_results_2026-09-23/training_history/`: helper di sola lettura e precedente confronto a40; gli artefatti precedenti non sono stati sovrascritti.
- `reports/user/2026-09-22_diagnosi_completamento_e_ripresa_unified39.md` e `2026-09-22_robustezza_unified39_diagnosi_checkpoint60.md` per i confondenti delle riprese LEFT.

Tutti i percorsi sopra, eccetto `reports/` e `plot/`, sono relativi a `Trajectory Generator/`.
