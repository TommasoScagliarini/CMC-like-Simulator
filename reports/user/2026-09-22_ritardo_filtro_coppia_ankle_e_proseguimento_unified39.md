# Ritardo, coppia della caviglia e proseguimento della unified39

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 22 settembre 2026. Run: `MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21`, completato fino alla 40. Analisi offline; nessun nuovo training o rollout fisico avviato.

## Valutazione

**È ragionevole completare il test fino a 100 iterazioni, riprendendo il checkpoint completo della 40 e mantenendo la configurazione concordata.** Il miglioramento cinematico rispetto alle precedenti prove 39D è concreto. Le verifiche attuali indicano che il ritardo del governor è già parzialmente compensato dalla rete e che la correzione storica del momento GRF alla caviglia è presente e funzionante. Non emerge una correzione software necessaria prima di una continuazione.

La proposta è un test di maturazione della policy, non una previsione di recupero garantito. Restano il completamento esplorativo basso e il burst positivo della caviglia. Valutazioni deterministiche alla 60, 80 e 100 permetterebbero di distinguere un progresso reale da una variazione del ritorno di training. Nessuna continuazione è stata avviata.

## Collocazione dei sette grafici canonici

Spostata integralmente la cartella indicata dall'utente in [plot/09_21_2026_1](../../plot/09_21_2026_1). Conservati senza variazioni tutti gli otto file originali: sette PNG e `missing_channels.txt`, verificati tramite SHA256.

Aggiornati i sette collegamenti nel report del 21 settembre e la ricevuta del rollout. Una copia della ricevuta è presente accanto alle figure. `render_canonical_rollout.py` ora salva i canonici sotto `plot/` nel root; la regola è registrata anche in `AGENTS.md`. I grafici diagnostici di questa analisi restano nel relativo perimetro di validazione.

## Quanto conta il ritardo

Il riferimento dopo governor è confrontato con il prescribed al medesimo istante nella reward. Un ritardo non compensato contribuisce quindi all'errore di posizione e velocità; può inoltre spostare il gesto rispetto al contatto. Non è un ritardo da dichiarare ininfluente per principio.

Va però distinto il ritardo della catena dal ritardo residuo di una policy che può anticipare. Nei rollout diagnostici già prodotti il 21 settembre, che comandavano direttamente il target, il governor introduce circa **80 ms** per entrambi i giunti. Dopo riallineamento offline, l'RMSE della reference sinistra scende da 6,49° a 0,39° al ginocchio e da 4,33° a 0,29° alla caviglia: in quella prova il ritardo domina l'errore.

Nella policy alla 40 il risultato è diverso:

| Confronto reference servita–target | Ginocchio | Caviglia |
|---|---:|---:|
| Ritardo residuo globale | **30 ms** | **20 ms** |
| Ritardo nei due cicli completi | 50 / 30 ms | 20 / 30 ms |
| RMSE senza riallineamento | 6,36° | 5,72° |
| RMSE dopo migliore traslazione offline | **5,94°** | **5,66°** |
| Riduzione dell'RMSE | 6,5% | 1,1% |

Le cifre usano la stessa finestra 13,30–17,75 s, 446 campioni, per tutti gli shift; sono dunque diverse dagli RMSE sull'intero episodio riportati in precedenza. La scansione copre ±200 ms a passi di 10 ms, senza interpolare gli endpoint oscillanti della policy. Minimo RMSE e massima correlazione concordano per la reference della 40; sono verificati anche i due cicli completi separatamente.

Il raw endpoint mostra anticipo rispetto al target, coerente con una compensazione appresa; è però oscillante e poco correlato, quindi non permette una somma rigorosa di ritardi fissi. Endpoint e target sono riferiti al tempo finale del segmento: il comando viene scelto 10 ms prima. Il ritardo del filtro dipende inoltre dalla frequenza e dai limiti del governor. La frazione diagnostica media di jerk limiting è circa 0,493 nella 40, contro zero nei due comandi diretti; non tutto il percorso della policy si comporta quindi come un puro ritardo lineare.

Il passaggio reference servita → giunto non aggiunge ritardo rilevabile alla risoluzione di 10 ms. Gli errori su quella finestra sono circa 0,25°/0,36°. Nel V26 best87 il ritardo residuo della reference rispetto al proprio target era circa −10 ms/0 ms: il filtro comune era compensato meglio, soprattutto alla caviglia.

**Conclusione:** il ritardo ha effetti, ma una semplice traslazione della curva attuale recupera poco errore. Restano soprattutto forma, offset ed escursione. Non è giustificato attribuire il difetto attuale a 80 ms interamente non compensati o eliminare subito il filtro.

Un controllo aggiuntivo ricalcola solo la componente imitativa sui dati registrati, spostando insieme q e qdot dei due giunti degli shift che minimizzano il rispettivo RMSE di posizione. Nella 40 lo score medio passa da 0,6570 a 0,6498: **non migliora**. L'ottimo dell'RMSE di posizione non coincide con quello dello score non lineare che considera anche le velocità. Questa è una diagnosi sui trace, non il ritorno atteso di una nuova simulazione: contatti, dinamica e penalità fisiche non sono ricalcolati.

[Immagine rimossa: Riallineamento diagnostico — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-301a2a4ef7f31811>)

## Perché la coppia della caviglia diventa positiva

Il fenomeno è lo stesso meccanismo diagnosticato nei report di giugno. Il momento della GRF rispetto all'asse della caviglia diventa negativo; il SEA produce coppia positiva quasi uguale per mantenere la reference.

Nel picco dell'ultimo appoggio:

- `tau_spring` = **+37,22 Nm**, a 17,532 s;
- `tau_ref` = **+37,21 Nm** nello stesso campione;
- momento GRF vicino al picco = **−36,36 Nm** nel trace, −36,75 Nm nella ricostruzione simultanea già eseguita;
- carico normale a 17,530 s = 539,19 N, penetrazione 17,37 mm;
- componente proporzionale cascade = 1,41 Nm, integrale = 35,80 Nm, senza clamp integrale o saturazione nel burst.

Sui campioni caricati, la correlazione fra coppia SEA e opposto del momento GRF è **0,999765**. L'errore RMS del tracking di coppia dopo il transiente è 0,428 Nm. Non emerge un errore di segno nel plot o un attuatore che non segue il comando. La leva del contatto può essere sfavorevole anche sotto la soglia di penetrazione di 20 mm.

Il burst finale oltre +8 Nm occupa circa 17,183–17,808 s; all'ultimo campione è già sceso a +0,69 Nm. Non tutte le coppie positive sono patologiche: anche il V26 ha piccoli transitori positivi.

[Immagine rimossa: Bilancio di coppia nell'ultimo appoggio — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-efab278d5fecf67f>)

## Quale correzione storica era stata adottata

La [diagnosi del 22 giugno](2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md) identificò la relazione COP–momento GRF–coppia SEA. La [correzione dello stesso giorno](2026-06-22_reward_guard_momento_grf_caviglia.md) introdusse una penalità sul momento GRF inferiore a −8 Nm sotto almeno 50 N di carico, inizialmente con peso 0,25. Non modificò il segno del controller né impose un divieto generale di coppia positiva.

Il pacchetto forte migliorò la caviglia ma peggiorò ginocchio e carico. Il [risultato del 24 giugno](2026-06-24_analisi_rollout_grf_soft.md) mostrò un compromesso migliore con peso momento **0,10** e penetrazione **1,0**. Sono esattamente i pesi attivi nel V26 e nella 39D attuale, con le stesse soglie.

La penalità è realmente applicata: costa **40,43 punti** nel rollout della 40. Alla 29 costava 69,85, alla 25 quasi zero. La correzione non è stata persa; la nuova policy non ha ancora consolidato il comportamento che evita di attivarla. L'andamento non è monotono.

Non c'è quindi una patch storica da riapplicare al SEA. La strada coerente è verificare se l'apprendimento corregge la traiettoria sotto carico con l'incentivo già presente. Se il burst persiste, una successiva variazione del peso andrà provata separatamente: aumentarlo direttamente a 0,25 riproporrebbe un compromesso che a giugno aveva già penalizzato il ginocchio.

La probe statica del 21 settembre resta una conferma locale: correggere solo la caviglia da 14,62° al target 9,33° riduce il momento da −36,75 a −4,18 Nm e azzera la penalità. Non serve invertire la coppia del SEA per risolvere quel contatto.

## Chattering del ginocchio e ruolo dell'ex-novo

È ragionevole tenere il chattering come obiettivo secondario durante questo confronto imitativo. Con la stessa misura offline su 13,15–17,95 s, filtro passa-alto Butterworth di quarto ordine a 20 Hz e fase zero:

| RMS della componente rapida | 39D iter40 | V26 imitativo best87 | Comando diretto LEFT |
|---|---:|---:|---:|
| Coppia elastica ginocchio | **3,66 Nm** | **3,28 Nm** | 0,15 Nm |
| Coppia motore interna ginocchio | **13,32 Nm** | **12,40 Nm** | 0,47 Nm |

Il chattering della 40 è vicino a quello presente anche nel buon V26 imitativo, pur essendo maggiore la coppia complessiva. La stessa catena produce una componente rapida molto minore con il comando diretto: il fenomeno non è inevitabile in ogni traiettoria.

La regolarità può migliorare con una policy diversa e nell'ex-novo, ma non è automatica. Il [report del 20 luglio](2026-07-20_audit_exnovo_prescribed_reward_deployability_checkpoint_best.md) attribuiva la regolarità all'intera catena: prior imitativo, costo di variazione dei comandi, slew/governor e controllori. Non la attribuiva soltanto al passaggio alla reward ex-novo. Per questa continuazione non emerge una necessità di modificare il plugin o i guadagni.

## Come valutare l'eventuale continuazione

La 40 ha un RMSE sul proprio target di 6,16°/5,64°, rispetto ai circa 12–14°/7,5–8,5° dei precedenti rollout 39D completi. È un miglioramento descrittivo consistente con i grafici, ma gli start e le ricette delle prove precedenti differivano: non è una misura causale di una singola modifica.

Proposta operativa, non avviata:

1. Ripristinare lo stato completo del learner alla 40 e proseguire fino alla 100, mantenendo FSM v3, start unico, prescribed e clock visibili, scala velocità 1, reward e governor concordati.
2. Valutare checkpoint alla 60, 80 e 100 allo stesso start, salvando i sette canonici in `plot/`.
3. Confrontare RMSE e escursione, ritardo residuo, completamento degli episodi esplorativi, penetrazione, penalità di momento e area di coppia positiva sotto carico. Il solo ritorno medio non basta a scegliere il checkpoint.
4. Se la cinematica migliora ma il contatto peggiora, trattare il compromesso con un esperimento separato, senza modificare silenziosamente questa continuazione.

Il completamento esplorativo recente resta 38,8% contro 88,4% del V26 a esperienza simile: è la principale riserva sulla robustezza. A 100 iterazioni la 39D avrebbe 409.600 transizioni contro 441.110 nello storico V26 a 100, circa il 7,1% in meno. Il numero nominale di iterazioni non costituisce una parità esatta di dati né una prova di equivalenza statistica.

## File e verifiche

- Spostati e verificati gli otto file originali della cartella dei canonici; aggiornate ricevute e collegamenti.
- Modificati `AGENTS.md` e `unified_MLP_39/validation/render_canonical_rollout.py` per la destinazione dei grafici; compilazione sintattica riuscita.
- Nuove analisi riproducibili e metriche in [phase_delay_torque_2026-09-22 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-6dda9a1696b284ae>): audit ritardo su sei trace, audit coppia su sette rollout, analisi della componente rapida e ricevuta dello spostamento.
- Ricalcolo delle loss imitativo/served coerente con le registrazioni entro 8×10⁻⁸; controlli temporali e finestre di confronto invarianti.
- Nessuna modifica ai dieci sorgenti congelati del training, alla reward, alla configurazione operativa o ai controllori. Nessun nuovo processo di training o simulazione.
