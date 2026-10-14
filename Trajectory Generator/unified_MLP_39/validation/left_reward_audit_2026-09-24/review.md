# Revisione indipendente dell'audit reward LEFT

Data: 24 settembre 2026. Revisione dei sorgenti diagnostici e degli artefatti; nessuna simulazione, training o modifica di produzione. Il controllo indipendente è stato eseguito con un singolo thread numerico. Le sole modifiche del revisore sono questo memo.

## Esito

**PASS per contabilità, finestre temporali e conclusioni descrittive principali**, con i limiti interpretativi sotto. Non emerge un errore che invalidi le metriche principali. L'audit localizza compromessi e possibili interventi da provare; non dimostra che un particolare peso abbia causato il divario LEFT, né che una modifica migliorerebbe l'apprendimento.

## Controlli effettuati

- Ricostruita indipendentemente dai CSV l'identità `1 − deficit imitativi − penalità base + correzione clipping − penalità postclip = reward` per tutte le 5.000 righe deterministiche. Errore massimo 4,44×10⁻¹⁶.
- Ricomposte le somme dei dieci intervalli di fase per ogni dataset: coincidono con i totali dell'episodio entro 5,12×10⁻¹³. Gli intervalli usano la fase prescribed del ginocchio, non la fase fisica FSM; gli intervalli RIGHT non rappresentano automaticamente le stesse fasi fisiche LEFT.
- Verificati i numeri LEFT72: return200,8952; deficit di posizione ankle137,2888 e knee38,2728; costo command-rate77,8315 e anti-flip43,1080. Oracle LEFT366,7745. Tutti questi episodi hanno500step e lo stesso target LEFT.
- Clipping inferiore della base: LEFT25 ha6step con correzione cumulativa+0,119863; gli altri nove deterministici hanno0step. Questa osservazione da sola non copre gli stati esplorativi dei training iniziali.
- Nella sensibilità, il command-rate è ricostruito come media di sei termini. LEFT72 riceve73,2800 dei77,8315punti da `segment_delta_loss`; la sua definizione è endpoint policy meno ancoraggio corrente filtrato, non differenza tra endpoint consecutivi. Cap per giunto: knee217/500=43,4%, ankle320/500=64%; RIGHT60:49,4%/34%.
- La diagnostica fisica esclude correttamente l'ultima riga hard duplicata del seed127, conservandola nella somma del ritorno. Il prefisso comune dei casi LEFT è303passi; le finestre appaiate finali rispettano la disponibilità del comparatore, inclusa la finestra accorciata a16passi per il fallimento sigma0,5.
- Ricontrollati i manifest dopo gli aggiustamenti:23input main,35physics,34sensitivity e15 nel nuovo probe dei training iniziali, tutti corrispondenti agli SHA-256 salvati.

## Correzioni minime segnalate

1. **Etichetta del grafico dei deficit:** la barra non sottrae la correzione del clipping, dunque non è esattamente `500−return` per LEFT25. L'autore ha corretto l'etichetta in «Deficit/costi prima della correzione clipping»: correzione verificata nel sorgente.
2. **Sensibilità locale dello score e della reward:** le prime formule non includevano l'indicatore che la base fosse interna ai limiti. La versione aggiornata moltiplica le derivate per `baseline_clip_interior` e usa i parametri della configurazione, correttamente distinguendo i due step clippati di noise0,5 seed123. Sono derivate del solo scalare rispetto alle loss aggregate, non gradienti PPO o rispetto alle azioni.
3. **Linguaggio sul seed125:** nella prima versione di `physics/findings.md`, «La reward non premia il collasso osservato» era troppo generale. L'autore ha adottato la formulazione raccomandata: «Nella finestra finale la reward è già fortemente negativa; il caso non supporta una mancata penalizzazione locale». Correzione verificata. Le traiettorie alternative hanno stati e storie differenti e non costituiscono un confronto causale del return globale.

Tutte e tre le precisazioni segnalate sono state recepite senza modificare la produzione.

## Estensione verificata: prime25iterazioni di training

Letti `physics/training_initial_probe.py`, JSON e memo; verificata indipendentemente l'identità lineare della correzione del clipping sulle50righe aggregate LEFT/RIGHT. I valori arrivano dagli stream `reward_components` realmente utilizzati dalla pipeline, non dagli score grezzi interni all'ambiente. La decomposizione reward base meno penalità postclip torna entro2,07×10⁻⁸, compatibile con scalar TensorBoardfloat32.

| Finestra | Clip lift LEFT | Clip lift RIGHT | Completi LEFT | Completi RIGHT |
|---|---:|---:|---:|---:|
| 1–5 | 0,049115 | 0,080634 | 2/119 | 1/112 |
| 21–25 | 0,001145 | 0,004405 | 19/51 | 32/42 |

Il clipping esiste dunque nella prima fase di entrambi i training; non è corretto estendere retroattivamente l'assenza di clipping dei rollout maturi. Il suo lift aggregato è maggiore nel RIGHT in tutte e cinque le finestre, mentre il completamento RIGHT successivamente migliora di più: questo non identifica il clipping come meccanismo selettivo del peggioramento LEFT. Non lo esclude come contributo comune o tramite interazioni.

Il calcolo usa una ricombinazione lineare di medie con lo stesso operatore, non il clipping della media della base. I dati sono medie mobili100step aggregate dai worker e poi mediate sulle cinque iterazioni; non sono medie esatte dei4096step di ogni batch. Non consentono di contare quante transizioni fossero clippate o localizzarle per fase. Il probe documenta correttamente tutti questi limiti e conserva il confronto prima del difetto di restore Adam.

## Limiti da preservare nel report

- La ripartizione proporzionale dei deficit non lineari è un'identità contabile valida, ma non un'attribuzione causale unica. I137,29punti ankle non sono il guadagno realizzabile correggendo solo la caviglia, né provano che il suo peso sia adeguato o insufficiente.
- Cambiare il peso command-rate o la sharpness assegna punteggi diversi alle medesime traiettorie. Un incremento del ritorno ricalcolato non prevede migliore cinematica, coppie o completamento dopo riaddestramento. Lo stesso vale per il controfattuale con errore served ridotto e costi fisici/comandi congelati: è una sensibilità condizionale, non una traiettoria dinamicamente realizzata.
- L'oracle LEFT riceve un punteggio maggiore della policy attuale e mostra che il ranking grossolano di questi due esempi non è invertito. Ha però preview10ms, controllo differente, penetrazione e carichi elevati; non prova ottimalità della reward o robustezza generale.
- Il seed127 sigma1 ha reward media+0,485 negli ultimi20step fisici contro−0,349 del deterministico che completa. Ciò mostra un compromesso locale tra imitazione e contatto. Non dimostra che la reward scontata preferisca fallire: la riga hard successiva ha reward−1,900799, con `safety_term=2,0`, e la terminazione elimina reward future.
- Nel segmento hard, i termini di contatto azzerati non sono un campione fisico valido. La sottrazione della penalità teorica alla soglia30mm è soltanto un calcolo scalare ipotetico; non ricostruisce il sottopasso mancante né dimostra il ritorno che avrebbe avuto una transizione completa.
- Cap del termine di comando, clipping delle azioni, compressione non lineare dello score e clipping della reward base sono meccanismi diversi: non chiamarli indistintamente «saturazione della reward».
- Confronti di fasi, campioni e cinque seed sono osservazioni correlate di poche policy e uno start. Non sono repliche indipendenti del training né test di equivalenza statistica. La prosecuzione LEFT25→40 contiene inoltre il difetto storico di restore Adam.

## Conclusione utilizzabile

La reward risulta calcolata come configurata e mantiene un segnale imitativo sostanziale; l'audit non sostiene una scomparsa generalizzata del segnale dovuta al clipping nei checkpoint maturi. Sono invece osservabili compromessi tra errori della reference, regolarizzazione dei comandi e sicurezza del contatto, che giustificano un esperimento mirato senza ancora identificare un peso da cambiare. Un warm-start riuscito e un beneficio di una reward rivista resterebbero risultati compatibili, non spiegazioni mutuamente esclusive.
