# Strategia dalla best72: confronto con V26 e oscuramento dei prescribed

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 23 settembre 2026. Questo report registra la discussione e le verifiche documentali; non autorizza né avvia i nuovi esperimenti proposti.

## Stato del progetto e scelta dell’utente

L’utente sceglie **best72** come checkpoint di lavoro della unified 39D. Considera le traiettorie cinematiche qualitativamente accettabili, pur inferiori alla baseline imitativa V26 del 20 agosto. Restano curve di coppia peggiori, con picchi positivi alla caviglia e chattering al ginocchio, e soprattutto un problema irrisolto di robustezza.

La scelta della 72 è una decisione pratica dell’utente. Non implica che sia risultata superiore a tutte le altre tappe nelle valutazioni indipendenti. Occorre distinguere due obiettivi: spiegare il divario rispetto a V26 e ottenere un controllore autonomo migliore partendo dalla soluzione attuale.

## Valutazione del best72 già completata

Il 22 settembre, dalle 18:27 alle 19:00, sono stati eseguiti undici rollout: uno deterministico e cinque coppie stocastiche con rumore originale/dimezzato, seed 123–127, medesimo start. Non è stato svolto un nuovo training.

| Indicatore | Iterazione 60 | Best72 | Iterazione 100 |
|---|---:|---:|---:|
| Return deterministico | 254,16 | 200,90 | 229,02 |
| RMSE ginocchio rispetto al target | 6,42° | 6,56° | 6,51° |
| RMSE caviglia rispetto al target | 5,13° | 5,98° | 4,91° |
| Penetrazione massima deterministica | 16,63 mm | 21,83 mm | 21,85 mm |
| Cicli FSM validi nel deterministico | 2 | 2 | 1 |
| Completi con rumore originale | 4/5 | 3/5 | 3/5 |
| Completi con rumore dimezzato | 4/5 | 4/5 | 5/5 |

Il deterministico della 72 completa tutti i 500 passi. I tre fallimenti stocastici sono legati alla penetrazione. I cinque seed non permettono una graduatoria definitiva della robustezza né dimostrano equivalenza statistica con V26. Il return che seleziona il best in training e il risultato di una valutazione della policy salvata sono misure diverse.

Pesi, contratto e configurazione del best corrispondono esattamente al milestone 72. Il checkpoint rimane preservato.

## Che cosa sappiamo sulle differenze

Le analisi precedenti hanno verificato architettura effettiva equivalente, arrivo corretto di prescribed e clock, normalizzazione presente e formula della reward coerente sulle stesse loss. L’errore cinematico prevalente è già nella reference servita, a monte dell’inseguimento SEA; questo non separa ancora il contributo della policy da quello del filtro.

Il target sinistro cambia il problema fisico: richiede, fra l’altro, angoli negativi della caviglia non richiesti dal target destro storico. Le prove con comando diretto del riferimento sinistro ne hanno mostrato la fattibilità nello start esaminato, senza dimostrarne uguale facilità di apprendimento o robustezza all’esplorazione.

Distribuzione delle azioni e clipping sono una pista concreta: nelle ultime tappe il rumore dimezzato migliora diversi esiti e, alla 100, anche il tracking sugli stessi intervalli temporali. Ma ridurre la deviazione standard cambia anche il comando medio dopo clipping. Inoltre gaussiane ampie e clipping erano presenti nel V26 riuscito: non costituiscono da soli una nuova causa software sufficiente.

Restano altri fattori: FSM v3 durante le traiettorie perturbate, diversa gestione dei fallimenti dei worker, inizializzazione semanticamente differente quando gli ingressi vengono riordinati e difetto Adam della ripresa 25→40, corretto solo nella continuazione successiva. Non è stata identificata una singola causa radice del divario.

## Test proposto: prescribed destro sfasato nella 39D

Il test resta rilevante perché domanda se la pipeline 39D attuale recuperi il comportamento storico quando affronta lo stesso compito imitativo del V26.

La variante deve ripristinare coerentemente clock destro, template storico, sfasamenti effettivamente utilizzati, posizioni, velocità e normalizzazione della reward associata a quel riferimento. Non basta sostituire i quattro ingressi lasciando il target della reward sinistro. Va riprodotto il riferimento storico concreto, senza presumere che uno shift generico di 0,5 cicli sia identico agli shift calibrati V26.

Per studiare l’apprendimento è proposto un training da inizializzazione nuova. Adattare direttamente la best72 sinistra al destro misurerebbe anche la capacità di trasferimento della policy già appresa.

Il confronto più controllato è una coppia di run sinistro/destro con identici pesi iniziali, stesso codice attuale, quantità di campioni, esplorazione, start, FSM v3, filtro e controllori. Si modifica coerentemente il solo riferimento prescritto e la sua rappresentazione. Il vecchio run sinistro può fungere da confronto preliminare, ma non da controllo perfetto, soprattutto per il difetto Adam della ripresa 25→40. Non si propone di ripristinare bug storici o cambiare la versione della FSM.

### Budget minimo informativo

Ricalcolo dallo storico V26 originale, usando i contatori degli episodi conclusi nelle ultime cinque iterazioni:

| Iterazione V26 | Episodi completi | Campioni cumulativi | Iterazioni equivalenti a batch 4096 |
|---|---:|---:|---:|
| 20 | 19/49 = 38,8% | 103.032 | 25,15 |
| 25 | 50/63 = 79,4% | 130.444 | 31,85 |
| 33 | 38/43 = 88,4% | 163.212 | 39,85 |

La proposta è **prima lettura a 32 e budget diagnostico di 40 iterazioni**, non subito 100. Un risultato negativo dopo sole 20–25 iterazioni attuali sarebbe poco discriminante. Anche 40 non garantiscono convergenza e un risultato ambiguo non dimostrerebbe incapacità della rete.

La valutazione deve includere completamento, penetrazione, qualità cinematica rispetto al proprio riferimento e coppie; non soltanto return o RMSE di episodi con durate diverse. Le percentuali storiche possono risentire di fallimenti dei worker non registrati.

Stime basate sui tempi recenti del MacBook: **5–6 ore per un run da 40 con valutazioni**, esclusa la preparazione software; circa **10–12 ore per la coppia sinistro/destro**. Sono ordini di grandezza, non tempi garantiti. Le prove non sono state avviate né configurate in questa discussione.

### Controllo preliminare senza apprendimento

È stata proposta anche una verifica più economica: riutilizzare la policy V26 nella 39D con il riferimento destro e una rimappatura esplicita e verificata degli ingressi. Non usa un teacher e non richiede iterazioni di training; verifica se una soluzione già appresa funziona nel contenitore attuale.

Occorre preparare e verificare la conversione, confrontare le uscite per osservazioni semanticamente uguali e poi valutare fisicamente la policy. Non è un test già pronto o già eseguito. Un esito positivo restringerebbe i dubbi sull’esecuzione, ma non dimostrerebbe che un nuovo training sinistro debba convergere allo stesso risultato.

## Alternativa: ex novo e oscuramento dalla best72

È ragionevole tentare l’ex novo senza pretendere prima un’imitazione perfetta, se la qualità della 72 è accettata come punto di partenza. Il miglioramento osservato storicamente con V26 rende l’ipotesi plausibile, ma non garantisce che si ripeta.

Occorre distinguere tre interventi:

- **Oscuramento con reward imitativa:** clock e quattro target spariscono dall’osservazione, ma il riferimento continua a guidare la reward. La rete deve ricostruire il movimento dagli altri ingressi; non riceve automaticamente nuovi incentivi a migliorare coppie o robustezza.
- **Reward ex novo:** cambiano gli obiettivi ottimizzati. Può favorire contatti e comandi migliori, anche accettando una diversa aderenza al prescribed, secondo i termini effettivamente configurati.
- **Gestione dell’esplorazione:** cambia la distribuzione dei comandi applicati. Un miglioramento dopo il passaggio ex novo potrebbe dipendere anche da questo intervento.

Il curriculum attuale è **episodico**: tutti e sei i canali sono visibili oppure azzerati per l’intero episodio. La progressione riguarda la probabilità di visibilità; non attenua continuamente l’ampiezza dei segnali. Clock e quattro prescribed restano un unico gruppo.

### Limite dell’implementazione attuale

Oggi `reward_mode=ex_novo` impone visibilità zero sia nell’ambiente sia nel modulo actor/critic. Quindi **ex novo con ingressi ancora visibili, seguito da oscuramento progressivo, richiede una modifica esplicita della transizione**; non basta cambiare una probabilità nello YAML.

Il preset `configs/ex_novo.yaml` introdurrebbe inoltre molte variazioni rispetto alla 72: deviazione standard fissa 0,01, batch 4608, una sola epoca PPO, learning rate 5×10⁻⁷, tre start e slew 2,5/2 rad/s. Il divisore delle velocità prescribed è 10, mentre il contratto della 72 usa 1. Non va utilizzato invariato come se fosse una continuazione che modifica soltanto la reward.

La transizione deve mantenere espliciti provenienza del checkpoint, trattamento del critic e dell’ottimizzatore, distribuzione esplorativa e disponibilità dei canali. I controlli di compatibilità delle riprese non vanno aggirati.

### Percorso graduale proposto

Prima adattare la 72 alla reward ex novo mantenendo temporaneamente i sei canali visibili, con gestione dell’esplorazione esplicita e le altre impostazioni mantenute quanto possibile costanti. Se il comportamento è promettente, biforcare uno stesso checkpoint: un ramo conserva la visibilità, l’altro introduce il curriculum di oscuramento. I due rami devono condividere esplorazione e budget per attribuire l’effetto al masking.

L’avanzamento del curriculum andrebbe deciso usando anche valutazioni separate degli episodi oscurati: completamento, margine dalla penetrazione, regolarità dei contatti e qualità dei comandi. Il solo trascorrere delle iterazioni o il return medio misto non basta. Tolto il clock prescritto, la rete deve ricavare la progressione del passo dai segnali online rimasti; è una difficoltà aggiuntiva, non una cura automatica della robustezza.

Un confronto causale più completo richiederebbe anche una continuazione imitativa visibile con la stessa gestione di esplorazione e stato di apprendimento. Il percorso sequenziale proposto è un compromesso economico, non un fattoriale completo.

## Raccomandazione e stato delle decisioni

La best72 è il checkpoint operativo scelto dall’utente. Per l’obiettivo di **capire le motivazioni**, la raccomandazione dell’assistente è prima il controllo sul riferimento destro, eventualmente preceduto dalla verifica della policy V26 rimappata, e poi il ramo ex novo dalla 72.

Se viene privilegiato il progresso verso un controllore autonomo, si può anticipare l’ex novo, accettando che il suo eventuale successo non spiegherà da solo il divario storico. Non è necessario ottenere perfetta parità imitativa prima di sperimentare questa strada.

La sequenza futura non è ancora stata scelta o avviata. Nessuna nuova modifica a FSM, filtro, reward, maschera o configurazione di training è stata applicata durante questa discussione o per la creazione del report.

## Fonti, file e verifiche

- [Risultati finali a 100 e diagnosi](2026-09-22_risultati_finali_unified39_100iter.md).
- [Audit delle differenze rispetto a V26](2026-09-21_cause_differenze_unified39_v26.md).
- [Valutazione fisica del best72 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-8ec4e895ce608a8c>).
- [Set canonico best72](../../plot/09_22_2026_4/); [set iterazione 60](../../plot/09_22_2026_1/).
- [Disponibilità e curriculum — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-ba00f840c70b0711>), [maschera nel modulo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-f221263086aea844>) e [preset ex novo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-a71f16657df74cfd>).

Per questa discussione sono stati riletti report, risultati dei rollout e codice della transizione. Per il report sono stati verificati nuovamente i risultati del best72, il vincolo di oscuramento in ex novo, i parametri del preset e i contatori V26 alle iterazioni 20/25/33. Nessun nuovo test fisico o training è stato eseguito; le prove citate sono quelle archiviate. È stato aggiunto soltanto questo file Markdown, con controllo dei collegamenti locali.

## TODO aggiornati da propagare al daily report

- [x] Completare training e valutazioni 60/80/100, con sette grafici canonici per tappa.
- [x] Valutare best72 con il protocollo di undici rollout e salvare i sette plot canonici.
- [x] Registrare la scelta dell’utente di usare best72 come checkpoint di lavoro.
- [ ] Scegliere la priorità fra diagnosi sul riferimento destro e passaggio ex novo; le sequenze restano proposte.
- [ ] Se scelto, preparare il controllo della policy V26 rimappata e/o il confronto di training sinistro/destro da inizializzazione nuova, con verifica a 32 e budget proposto 40.
- [ ] Se scelto, separare nel software reward, disponibilità dei sei canali ed esplorazione per consentire ex novo visibile e successivo curriculum, preservando i controlli di compatibilità.
- [ ] Ampliare i seed di valutazione: la prova best72 è conclusa, l’estensione oltre cinque seed per condizione resta aperta.
- [ ] Eseguire il confronto sul comando atteso dopo clipping per separare effetti sulla media e sulla variabilità; proposta precedente non ancora svolta.
- [ ] Estendere il confronto alla stessa fase ai fallimenti di training, oltre ai casi appaiati già studiati.
- [ ] Esplicitare nei riepiloghi automatici la distinzione fra clipping originale delle azioni e clipping osservato nell’ambiente.
- [ ] Prevedere più seed di training e un protocollo omogeneo se resta richiesto dimostrare equivalenza statistica con V26.

Questa lista aggiorna quella del report finale del 22 settembre: la valutazione best72 è completata, mentre l’identificazione causale e la robustezza generale restano aperte.
