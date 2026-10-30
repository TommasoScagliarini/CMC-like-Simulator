# Unified 39D: risultati finali a 100 iterazioni e diagnosi della robustezza

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 22 settembre 2026. Training concluso alle **16:51:07 CEST**; valutazioni e pipeline concluse alle **17:24:50 CEST**.

## Esito

La continuazione fino a 100 non ha recuperato la qualità complessiva del V26 del 20 agosto. La rete produce traiettorie complete nel rollout deterministico, migliora alcuni aspetti della caviglia, ma il completamento durante il training non si consolida e resta lontano dai contatori storici V26 anche a quantità di esperienza quasi uguale.

L’indizio sperimentale più forte riguarda l’interazione fra distribuzione delle azioni, clipping e contatto: al checkpoint 100, dimezzare la deviazione standard in valutazione porta i completamenti osservati da **3/5 a 5/5** e migliora il tracking su intervalli temporali identici. Non è ancora una dimostrazione di robustezza generale, né una spiegazione unica del divario con V26. Anche V26 aveva gaussiane ampie e clipping.

**Non proseguirei invariato oltre 100 come primo passo.** Prima separerei l’effetto della variabilità delle azioni da quello del loro comando medio dopo clipping, e valuterei il checkpoint best72 con lo stesso protocollo. Nessun nuovo training o rollout fisico è stato avviato durante questa analisi.

## Integrità e configurazione

Sono presenti 100 iterazioni consecutive e **409.600 transizioni con prescribed e clock visibili**. Ogni batch contiene 4096 campioni. Gli audit delle riprese 42→60, 60→80 e 80→100 confermano la continuità dello stato Adam; gli audit di ripresa preservano anche pesi e coefficiente KL. L’ultimo processo termina con `ok=true` e `stop_reason=completed`.

Configurazione concordata invariata: FSM v3, target sinistro guidato dalla fase sinistra senza shift, velocità prescribed a scala 1, start unico, slew limiter disattivato e stesso filtro della prova allineata. I sei ingressi prescribed/clock sono verificati nelle tracce deterministiche; i target sinistri delle tappe 40/60/80/100 sono numericamente identici agli stessi istanti.

La storia non equivale a una singola esecuzione senza interruzioni: il difetto Adam della ripresa 25→40 è stato corretto solo nella continuazione successiva; gli RNG e gli episodi OpenSim interrotti non vengono ricostruiti. Le iterazioni 41–42 non hanno telemetria dettagliata recuperabile a causa del difetto di serializzazione già corretto. Questi limiti restano rilevanti per confronti causali fra ricette di training.

## Completamento e quantità di esperienza

| Tappa 39D | Ultime cinque iterazioni | Completi | Transizioni cumulative |
|---|---|---:|---:|
| 40 | 36–40 | 19/49 = 38,8% | 163.840 |
| 60 | 56–60 | 25/44 = 56,8% | 245.760 |
| 80 | 76–80 | 27/48 = 56,3% | 327.680 |
| 100 | 96–100 | 25/44 = 56,8% | 409.600 |

L’intero tratto 42→60 registra 92/160 episodi completi (57,5%); il 60→80, 114/178 (64,0%); l’80→100, 98/179 (54,7%). I miglioramenti intermedi non diventano una progressione stabile. Questi sono episodi conclusi, non tutti quelli ancora aperti al termine di ogni processo.

Alla stessa quantità approssimativa di esperienza, il V26 raggiunge la iterazione **92 con 408.342 campioni**. La sua finestra 88–92 registra **37/40 completi, 92,5%**, contro il 56,8% della 39D nella finestra 96–100. Entrambe le finestre comprendono 20.480 transizioni; gli estremi differiscono di 1258 campioni. Alla propria iterazione 100 il V26 ha 441.110 campioni e 38/43 completi nelle ultime cinque iterazioni (88,4%).

La differenza di esperienza non basta quindi a spiegare il divario osservato. Tuttavia i worker storici potevano perdere alcuni fallimenti, mentre l’implementazione attuale conserva più terminazioni: queste percentuali sono descrittive e non una stima corretta di un effetto causale dell’architettura.

Il checkpoint selezionato come **best di training è il 72, return 272,89**. È selezionato dal return degli episodi raccolti durante il training, non da una valutazione indipendente della policy salvata. Il protocollo fisico 40/60/80/100 non include una valutazione specifica della 72: non è lecito dichiararla già la migliore camminata.

## Traiettorie deterministiche

Tutti i rollout in tabella completano 500 passi, pari a 5 s. I RMSE confrontano il giunto con il proprio target; il target V26 è differente da quello sinistro della 39D.

| Checkpoint | Return | RMSE ginocchio | RMSE caviglia | Penetrazione massima | Cicli FSM validi |
|---|---:|---:|---:|---:|---:|
| 39D 40 | 222,15 | 6,16° | 5,64° | 18,21 mm | 2 |
| 39D 60 | 254,16 | 6,42° | 5,13° | 16,63 mm | 2 |
| 39D 80 | 203,83 | 5,42° | 7,01° | 19,60 mm | 2 |
| 39D 100 | 229,02 | 6,51° | 4,91° | 21,85 mm | 1 |
| V26 best87 | 363,79 | 6,91° | 1,46° | 14,80 mm | 2 |

La 100 non domina gli altri checkpoint: migliora il RMSE della caviglia rispetto alla 40 di circa il 12,8%, ma peggiora ginocchio, picco di penetrazione e regolarità FSM. La 60 conserva il return deterministico più alto e la penetrazione più bassa fra le quattro tappe; la 80 ha il miglior RMSE del ginocchio ma il peggiore della caviglia.

Il ginocchio della 100 raggiunge **45,20° di flessione**, rispetto ai **56,23° richiesti**. Nel V26, il picco era 63,78° su 64,32° richiesti. Il RMSE del ginocchio apparentemente vicino o inferiore a V26 non dimostra una riproduzione altrettanto buona del profilo: il picco resta attenuato.

La caviglia della 100 raggiunge valori negativi, da **−7,70° a +30,76°**, rispetto al target da −6,61° a +22,05°. È superata la mancata escursione negativa globale osservata alla 40, ma restano sovraelongazione positiva e differenze fra i cicli. Un minimo corretto in un ciclo non garantisce che tutti i minimi siano corretti. Il RMSE della caviglia nei cinque secondi successivi vale 1,88°, 3,55°, 2,26°, 7,67°, 6,39°: l’errore cresce nella seconda parte dell’episodio.

Il ciclo FSM mancante alla 100 ha una sequenza identificabile: HS rifiutato come troppo precoce allo step 165, resync allo step 173, ciclo successivo scartato come degradato allo step 264. Il rollout arriva comunque alla fine. Questo documenta un’irregolarità del contatto/rilevamento, non dimostra un nuovo bug della FSM.

![Confronto finale delle traiettorie e del completamento](../../plot/09_22_2026_unified39_final_comparison/comparison.png)

## Reference generata, inseguimento e coppie

Alla 100 il RMSE **giunto–reference servita** è appena **0,31° al ginocchio e 0,47° alla caviglia**. Il RMSE **reference servita–target** è invece **6,55° e 4,79°**. Quindi gran parte dell’errore è già nella reference generata/filtrata: il basso livello la segue abbastanza fedelmente nel rollout deterministico.

Questo non separa ancora il contributo della policy da quello del filtro, né esclude errori transitori di inseguimento sotto perturbazione. Non giustifica, da solo, una modifica dei guadagni SEA o la rimozione del filtro.

Persistono le coppie positive alla caviglia nella parte finale dei grafici e il chattering al ginocchio. La ricostruzione della reward dalle tracce conferma una penalità cumulativa sul segno del momento GRF alla caviglia di **55,32 alla 100**, contro 18,84 alla 60, 40,43 alla 40 e circa 0,01 nel V26. È una penalità sul momento di contatto; non coincide con una penalità diretta sulla coppia SEA positiva.

Alla 100 la reward base cumulativa vale 284,84; sottraendo 55,32 di penalità sul momento, 0,24 di penetrazione e 0,26 di fuori intervallo si ottiene il return 229,02. La qualità imitativa base migliora rispetto alla 40, ma aumenta il costo legato alla geometria del contatto. Non è stato identificato un nuovo difetto del segno del comando SEA.

## Rumore: risultato appaiato e limite dell’interpretazione

| Checkpoint | Completi, rumore originale | Completi, rumore dimezzato | Return medio originale | Return medio dimezzato |
|---|---:|---:|---:|---:|
| 40 | 5/5 | 5/5 | 231,84 | 284,20 |
| 60 | 4/5 | 4/5 | 264,50 | 271,04 |
| 80 | 2/5 | 5/5 | 265,17 | 290,64 |
| 100 | 3/5 | 5/5 | 191,37 | 293,80 |

Alla 100 i fallimenti con rumore originale avvengono ai passi **297 e 176**, per penetrazione; i corrispondenti rollout a rumore dimezzato completano entrambi 500 passi. I fallimenti alla 80 erano molto più tardivi, fra 467 e 490 passi. Il piccolo campione non consente una graduatoria definitiva della robustezza dei checkpoint, ma non mostra un consolidamento monotono.

Per evitare che il RMSE sembri migliore solo perché un episodio si interrompe prima, il confronto è ripetuto sui medesimi primi **176 passi** di tutte le traiettorie della 100:

| Media dei RMSE per rollout, sul prefisso comune | Rumore originale | Dimezzato |
|---|---:|---:|
| Ginocchio | 7,68° | 5,48° |
| Caviglia | 4,13° | 2,73° |

Il miglioramento vale in tutte e cinque le coppie per entrambi i giunti, anche estendendo ciascuna coppia al suo massimo prefisso comune. Il prefisso globale copre però solo 1,76 s, non l’intero cammino.

Le componenti delle azioni tagliate ai limiti sono il 37,2% con rumore pieno e il 27,0% con quello dimezzato, aggregando i passi osservati dei cinque rollout. Il dato 0% nei riepiloghi della telemetria di training riguarda il clipping nell’ambiente, dopo il connector RLlib, e non misura questo clipping a monte.

## La distribuzione appresa cambia durante il training

All’identica osservazione iniziale 39D, verificata esattamente uguale nelle quattro tappe:

| Iterazione | Media azione caviglia prima del clipping | Deviazione standard appresa |
|---|---:|---:|
| 40 | 0,704 | 0,989 |
| 60 | 1,051 | 1,177 |
| 80 | 1,263 | 1,399 |
| 100 | 1,480 | 2,136 |

L’intervallo applicabile è [-1, 1]. È quindi documentato un aumento della dispersione in almeno questo stato, insieme a una media fuori intervallo; non si tratta soltanto di visitare stati diversi. La dispersione non cresce uniformemente in tutti gli stati.

Una gaussiana tagliata ai limiti ha un comando atteso diverso dalla propria media tagliata: **E[clip(μ + σz)] non coincide in generale con clip(μ)**. Sugli stessi stati dei rollout a rumore pieno della 100, dimezzare σ modifica il comando medio atteso di una quantità equivalente, in valore assoluto medio, a circa **3,3° dell’endpoint grezzo** per ciascun giunto. Il filtro successivo ne modifica l’effetto fisico.

Questo significa che il vantaggio del rumore dimezzato può derivare sia da perturbazioni meno ampie sia da un diverso comando medio. Il deterministico, il rumore pieno e quello dimezzato non sono tre versioni identiche della stessa traiettoria con una semplice oscillazione sovrapposta.

**Controllo storico necessario:** anche il V26 riuscito aveva gaussiane ampie e distorsione da clipping. L’audit storico del best87 riporta deviazioni standard medie 1,03/0,75 e differenza RMS fra endpoint atteso stocastico e deterministico di 7,04°/5,42° per ginocchio/caviglia. Per la 100, sui propri stati deterministici, la seconda misura vale 8,25°/10,60°. Le visite di stato e i target differiscono: non è un confronto causale. Il clipping non è un nuovo bug sufficiente a spiegare tutto il divario.

## Come si sviluppano i fallimenti

Nell’ultimo tratto di training, **80/81 fallimenti sono per penetrazione**, e 77/81 terminano nella fase FSM di appoggio. Fra i 98 episodi completi, 21 raggiungono almeno 26 mm e 11 almeno 27 mm. Il tempo mediano dal primo superamento di 20 mm osservato nella coda alla terminazione per penetrazione è circa 55 ms. La misura usa la coda di 50 passi, non tutta la storia dell’episodio.

Il confronto dei rollout appaiati della 100 mantiene uguali tempo e target prescritto:

| Seed, istante di arresto con rumore pieno | Penetrazione pieno / dimezzato | Carico pieno / dimezzato | Errore caviglia pieno / dimezzato |
|---|---:|---:|---:|
| 126, passo 297 | 28,04 / 14,42 mm | 0,92 / 0,51 BW | −6,61° / +2,58° |
| 127, passo 176 | 28,53 / 11,80 mm | 1,81 / 0,31 BW | −9,02° / −2,99° |

BW indica la forza espressa in multipli del peso corporeo. Nel seed 126 emerge un atterraggio più tardivo e rapido, con resync vicino alla fine. Nel seed 127 il piede carica fortemente mentre la FSM è ancora in swing, senza resync. Non c’è quindi un unico evento FSM comune ai due fallimenti. Le storie delle azioni hanno già fatto divergere gli stati: il confronto non isola un singolo angolo o un singolo comando responsabile.

![Sviluppo dei due fallimenti e confronto con rumore dimezzato](../../plot/09_22_2026_unified39_final_comparison/paired_failures.png)

Nei dati di training, soltanto 4/81 fallimenti dell’ultimo tratto hanno un resync nei precedenti 100 ms e 5/81 un cambio dei pesi. Questi eventi immediati non spiegano la maggioranza dei fallimenti. La loro assenza vicina alla fine non esclude effetti antecedenti.

## Strategia successiva proposta

1. **Valutare best72 e conservare la 60 come confronto**, con lo stesso protocollo e più seed appaiati. La 100 non è automaticamente il checkpoint migliore.
2. **Separare variabilità e media dopo clipping**: confrontare il deterministico usuale `clip(μ)` con rollout che applicano il valore atteso dell’azione tagliata per σ originale e dimezzata, ricalcolato a ogni stato raggiunto. Questo aiuta a distinguere una media di comando poco adatta da perturbazioni casuali troppo ampie. Non elimina tutti gli effetti non lineari del sistema.
3. Solo dopo, scegliere una prova controllata sulla gestione dell’esplorazione in training, mantenendo target, clock, scala delle velocità e FSM v3. Le prove attuali non dimostrano che un training con σ ridotta converga meglio: la variazione è stata applicata solo in valutazione.

Non partirei dall’oscuramento dei prescribed o da una nuova modifica di FSM, reward o filtro: la baseline con ingressi visibili non ha ancora raggiunto l’obiettivo. Il target sinistro resta deliberatamente diverso e la sua fattibilità è già stata verificata dal controllo oracle precedente. La normalizzazione è presente; queste analisi non identificano una sua assenza.

## Artefatti e verifiche

- [Set dei sette grafici canonici della 100](../../plot/09_22_2026_3/); [tappa 60](../../plot/09_22_2026_1/) e [tappa 80](../../plot/09_22_2026_2/).
- [Metriche finali riproducibili — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-48a90259538f0d09>), script `analyze_final.py` e hash degli input nella stessa cartella.
- [Audit del rumore e del clipping — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-a8802777889331ef>), con script e JSON.
- [Esiti della pipeline — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-46df9dc9bb280b63>), con `results.json`, audit delle riprese, telemetria e ricevute dei grafici.
- [Report precedente della diagnosi alla 60](2026-09-22_robustezza_unified39_diagnosi_checkpoint60.md) e [correzioni della ripresa](2026-09-22_diagnosi_completamento_e_ripresa_unified39.md).

Verifiche eseguite: storico 1–100 contiguo; batch e visibilità 100%; audit Adam delle tre continuazioni; ricostruzione della reward deterministica con errore inferiore a 10⁻⁶; identità del target sinistro fra le tappe; visibilità e valori degli ingressi clock/prescribed; osservazioni iniziali e rumore appaiati; confronto degli errori sul medesimo intervallo; mappatura azione→endpoint; lettura dei JSON e controllo visivo dei grafici di sintesi e dei canonici pertinenti.

I tre tratti con telemetria hanno ciascuno 13 worker, nessun duplicato e nessun record invalido. Le statistiche riguardano episodi conclusi e salvati. Una sola replica di training, cinque seed di azione e un unico start non consentono una conclusione di equivalenza statistica con V26.

File aggiunti in questa analisi: presente report, `analyze_final.py`, `noise_audit.py`, JSON/Markdown degli audit e due figure di confronto sotto `plot/09_22_2026_unified39_final_comparison/`. Nessun cambiamento al codice di training, ai checkpoint, alla configurazione fisica o al plugin SEA.

## TODO aggiornati da propagare al daily report

- [x] Completare training e verifiche separate alle tappe 60/80/100.
- [x] Generare i sette grafici canonici di ogni tappa nel root `plot/`.
- [x] Interpretare il completamento, il rumore e il contatto con i dati disponibili; confronto appaiato prima dei due fallimenti della 100 eseguito.
- [x] Documentare la differenza fra clipping originale e clipping osservato nell’ambiente.
- [ ] Valutare il best72 e ampliare i seed prima di scegliere definitivamente il checkpoint.
- [ ] Eseguire il test sul comando atteso dopo clipping e distinguere sperimentalmente le cause; proposte non ancora avviate.
- [ ] Estendere il confronto alla stessa fase ai fallimenti di training, oltre ai due casi appaiati studiati.
- [ ] Rendere esplicita nei futuri riepiloghi automatici la semantica delle due misure di clipping.
- [ ] Eseguire un confronto con più seed di training e un protocollo omogeneo se l’obiettivo resta dimostrare equivalenza statistica con V26.

Questa lista aggiorna i TODO dei report precedenti: le tappe sono concluse, mentre restano aperte l’identificazione causale e la validazione della robustezza.
