# Ritardo target → endpoint → reference servita → giunto

Data:2026-09-22. Analisi offline delle tracce archiviate; nessun training, episodio simulato, modifica al controller o alla FSM. Script:[analyze.py](analyze.py); risultati:[metrics.json](metrics.json), [compact.json](compact.json).

## Conclusione

Il governor produce un ritardo misurabile di circa80ms quando riceve direttamente la traiettoria target. Lo fa sia col prescribed sinistro sia col target destro storico. La policy40 sta già compensando buona parte di quel ritardo: nella reference servita rimangono circa30ms al ginocchio e20ms alla caviglia. Il tracking della reference da parte dei giunti non aggiunge un ritardo risolvibile alla griglia10ms.

La differenza di qualità non è principalmente una semplice traslazione nel tempo: dopo aver allineato la reference, gli errori restano5,94° al ginocchio e5,66° alla caviglia. Persistono la flessione insufficiente del ginocchio e la caviglia troppo positiva nelle valli del target.

## Metodo e convenzioni

- Segnali letti da500 righe per rollout, intervallo10ms. Confronto primario sullo stesso intervallo13,30–17,75s,446 campioni, così il transitorio iniziale è escluso e ogni lag usa gli stessi campioni.
- Ricerca discreta di lag da−200 a+200ms, passo10ms; riportati sia minimoRMSE sia massima correlazione. **Lag positivo significa che l'uscita è in ritardo**: confronto `input(t)` con `output(t+lag)`.
- Verifica su ciascuno dei due cicli completi, definiti dai wrap della fase target; confronto anche per quarti di fase. Non è una stima fondata soltanto sulla correlazione globale.
- Non si interpola l'endpoint rumoroso fra campioni: l'interpolazione a1ms potrebbe ridurne artificialmente le oscillazioni e falsare il minimoRMSE.
- L'endpoint è quello accettato dopo clipping dell'azione, datato al termine del segmento (`policy_segment_times[-1]`). La decisione NN era stata presa10ms prima. Datandolo alla decisione, l'anticipo endpoint-target aumenta10ms e il ritardo endpoint-served aumenta10ms; il ritardo target-served resta invariato.
- Nessuno slew limiter in questi rollout. All'interno del segmento il riferimento raw interpola dall'ancora servita al nuovo endpoint. Pertanto il numero endpoint-served include questa costruzione e il governor: non va interpretato come ritardo puro e costante del solo filtro lineare.

## Risultati cinematici

| Rollout | Ritardo knee / ankle | RMSE knee prima → allineato | RMSE ankle prima → allineato |
|---|---:|---:|---:|
| 39Diter25 |40 /30ms|7,78° →7,36°|5,66° →5,53°|
| 39Dbest29 |30 /30ms|6,67° →6,31°|6,47° →6,32°|
| 39Dlast40 |30 /20ms|6,36° →5,94°|5,72° →5,66°|
| V26best87 |−10 /0ms|6,63° →6,53°|1,42° →1,42°|
| OracleLEFT |80 /80ms|6,49° →0,39°|4,33° →0,29°|
| OracleRIGHT |80 /80ms|8,12° →0,47°|3,17° →0,24°|

QuestiRMSE usano l'intervallo comune; sono quindi leggermente diversi da quelli full-episode dei report precedenti. Il V26 usa il proprio target destro e il checkpoint87, quindi non è un confronto alla stessa maturità del training.

Nella40 il miglior shift elimina **12,6% dell'errore quadratico del ginocchio e2,2% di quello della caviglia**. È una decomposizione descrittiva dell'errore, non una quota causale identificata del governor. La correlazione target-served passa a0,967/0,958, ma forma e ampiezza rimangono diverse.

Nei due cicli completi della40, i minimiRMSE danno rispettivamente50/30ms al ginocchio e20/30ms alla caviglia. La variabilità del ritardo e della forma rende inappropriato trattare tutto l'episodio come una curva traslata di una costante esatta. Nei due oracle,80ms resta il miglior lag in entrambi i cicli e in entrambi i giunti; correlazioni allineate superiori a0,999.

Nella40 l'endpoint tende ad anticipare il target: circa80ms knee e30–40ms ankle, usando il suo timestamp di destinazione. Tuttavia la correlazione dell'endpoint col target è soltanto0,394/0,459: gli endpoint oscillano molto e quel lag è una descrizione debole, non una misura precisa della capacità predittiva. Endpoint-served dà circa100ms knee e60–70ms ankle; questi ritardi non devono essere sommati meccanicamente con gli altri.

Served-actual nella40: miglior lag0ms alla risoluzione10ms, RMSE0,248°/0,362° sull'intervallo comune. Il ritardo significativo precede la cinematica del giunto, non deriva dal tracking SEA. Il jerk limiter è attivo mediamente per0,493 della metrica interna dei segmenti della40, contro0 nei due oracle: il trasferimento dei comandi NN rumorosi non è una semplice traslazione lineare. Limiti di velocità e accelerazione non intervengono nella40.

## Impatto sulla reward: verifica senza simulazione

La reward valuta target e traiettoria allo **stesso istante**, quindi penalizza anche il ritardo; non cerca un allineamento prima di calcolare l'errore. Per ogni giunto confronta q e qdot con le rispettive normalizzazioni e pesi; la componente imitativa è:

`0.2 / (1 + 4 * loss_actual) + 0.8 / (1 + 8 * loss_served)`.

La ricostruzione offline delle loss usando q/qdot registrate concorda col runtime entro8×10⁻⁸. Ho quindi traslato insieme q e qdot di ciascun giunto con il lag che minimizza il suoRMSE di posizione, lasciando target e scale invariati. Questa è esclusivamente una rivalutazione dei segnali registrati: contatti, penalità, dinamica e FSM **non** vengono ricalcolati.

| Rollout | Componente imitativa media originale | Dopo allineamento offline |
|---|---:|---:|
| 39Dlast40 |0,6570|0,6498|
| V26best87 |0,8591|0,8619|
| OracleLEFT |0,7684|0,9972|
| OracleRIGHT |0,7388|0,9961|

La sola correzione del lag migliora radicalmente l'imitazione degli oracle, le cui curve erano quasi identiche al target ma in ritardo. **Non migliora automaticamente la componente imitativa della policy40**, anche se riduce ilRMSE: la trasformazione reciproca è non lineare e si applica a ogni campione dopo aver combinato i giunti. Ridurre l'errore quadratico medio non garantisce aumentare la media di quel punteggio. Nella40 lo shift riduce la loss media ma redistribuisce gli errori, con effetto netto−0,00725 sul punteggio medio (−3,24 punti sui446 campioni). Questo non è una previsione di return fisico né una raccomandazione a spostare il target.

## Interpretazione rispetto al31agosto

Il report31agosto misurava90–100ms complessivi con slew attivo, dei quali80ms dopo-slew→served. Qui lo slew è disattivato e gli oracle confermano circa80ms della catena di filtraggio. Il **ritardo del filtro** e il **ritardo residuo rispetto al target dopo l'azione anticipatoria della policy** sono due numeri distinti. Attribuire alla40 un disallineamento residuo generale di80–100ms confonderebbe queste due quantità.

I risultati suggeriscono che il filtro costituisca una dinamica che la policy deve anticipare, già in parte appresa. Non spiegano il gap rispetto al V26 con un nuovo filtro più lento: la catena è la stessa e il V26 mostra che è compensabile. L'errore residuo di forma/ampiezza e il compromesso con le altre componenti di reward richiedono più attenzione del solo shift temporale.

## Figure diagnostiche

- [Target e reference prima/dopo allineamento](alignment_diagnostic.png).
- [CurveRMSE e correlazione su tutti i lag](lag_curves.png).

Sono figure di diagnosi separate: nessun grafico canonico o risultato di training è stato modificato. Un solo rollout deterministico per checkpoint e due cicli completi non consentono inferenze statistiche generali.
