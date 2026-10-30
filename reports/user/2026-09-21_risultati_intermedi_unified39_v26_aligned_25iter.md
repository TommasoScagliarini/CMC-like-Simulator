# Risultati intermedi della 39D riallineata al V26 — 25 iterazioni

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 21 settembre 2026.

## Valutazione

**Il risultato giustifica il proseguimento di questa configurazione. L'imitazione viene appresa, ma la qualità e la robustezza del V26 non sono ancora dimostrate.** Il miglior checkpoint coincide con l'ultima iterazione, la 25. Nel rollout deterministico completa tutti i 5 secondi, senza superare la soglia di penalizzazione della penetrazione. Restano ridotta flessione del ginocchio, mancato raggiungimento della parte negativa del target di caviglia e contatti poco regolari tra cicli.

Il ritorno medio di training alla 25 è quasi uguale a quello storico alla stessa iterazione: 159,18 contro 161,62. Questo dato da solo nasconde una differenza di robustezza: nelle ultime cinque iterazioni gli episodi registrati che arrivano alla fine sono il 37,3% contro il 79,4% storico. **L'approfondimento sul campionamento chiarisce però che queste finestre non corrispondono alla stessa esperienza: a budget quasi uguale il confronto è 37,3% contro 38,8%.** Il V26 raccolse batch superiori ai 4096 nominali durante il recupero dai guasti dei worker. Restano diverse le cause di interruzione, con più penetrazioni nella 39D. I dettagli e i limiti sono riportati sotto.

Il test modifica insieme diverse impostazioni rispetto ai precedenti run 39D; permette di valutare la configurazione complessiva, senza identificare quale singola modifica abbia risolto il problema. Un seed e un rollout deterministico non consentono una conclusione di equivalenza statistica.

## Problema, configurazione e metodo

L'obiettivo è verificare se la 39D recuperi un apprendimento imitativo promettente adottando le impostazioni del V26 del 20 agosto, mantenendo la modifica deliberata del riferimento: **prescribed sinistro indicizzato dalla fase sinistra, senza sfasamento**. Per decisione dell'utente resta la **FSM v3**; pertanto non è una riproduzione integrale del software storico.

Configurazione effettiva:

- Actor a 39 ingressi, rete 2×256 tanh, critic asimmetrico; inizializzazione nuova, seed 123.
- Clock sin/cos sinistro e quattro ingressi prescribed sempre visibili; velocità prescribed con divisore 1, nessun oscuramento.
- Batch PPO 4096, minibatch 512, 10 epoche, learning rate 0,0001; 13 worker.
- Start unico con offset 1 s. Il setup inizia a 11,99 s: il reset fisico avviene a **12,99 s**, come nel V26. La verifica precedente al lancio aveva confrontato 18 componenti dello stato iniziale con differenza massima nulla.
- Nessuna inizializzazione direttamente sul target; durata massima 5 s, passo policy 0,01 s.
- Azioni assolute, slew limiter disattivato; governor e relativi limiti ripristinati alla configurazione storica.
- Dizionario dei parametri della reward allineato al V26. Target e normalizzazioni derivano dal riferimento sinistro concordato; FSM v3 e correzioni software attuali sono conservate.

Il campo condiviso `gait_clock_enable: false` disabilita il clock destro della pipeline comune. La 39D inserisce invece il **clock sinistro negli ingressi 0–1**. La sua presenza effettiva è stata controllata nel rollout: il clock dell'actor non è spento.

L'analisi usa lo storico delle 25 iterazioni, TensorBoard, audit dei batch e dei checkpoint, un nuovo rollout deterministico del best25 e la traccia storica del best87 V26. Il confronto cinematico usa lo stesso start fisico e la stessa durata, valutando ciascuna rete rispetto al proprio target. Non sono disponibili checkpoint V26 intermedi alle iterazioni 25, 32 o 35: non è possibile confrontare direttamente le due camminate alla stessa iterazione.

## Completamento e integrità del training

Run: `MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21`.

| Verifica | Risultato |
|---|---:|
| Iterazioni completate | 25/25 |
| Tempo totale | 3 h 19 min 53 s |
| Miglior iterazione | 25 |
| Miglior ritorno medio di training | 159,1827 |
| Transizioni raccolte | 102.400 |
| Transizioni con prescribed visibili | 102.400/102.400 |
| Restart / iterazioni saltate | 0 / 0 |
| Aggiornamenti actor e critic | Verificati in tutte le 25 iterazioni |
| Loss non finite / telemetria mancante | Nessuna |
| Hash dei sorgenti congelati al lancio | Tutti invariati |

La telemetria di training contiene 48 righe con evento di risincronizzazione FSM; non risultano timeout stance, discrepanze sul lato del timeout o violazioni del contratto causale della morfologia. Le 48 righe non vanno interpretate automaticamente come 48 episodi distinti. Tutti i checkpoint intermedi sono conservati.

## Andamento dell'apprendimento

| Blocco | Ritorno medio delle iterazioni | Durata media delle iterazioni | Episodi completi registrati | Terminazioni per penetrazione | Timeout swing |
|---|---:|---:|---:|---:|---:|
| 1–10 | 23,78 | 2,23 s | 5/200 = 2,5% | 155, di cui 23 hard | 40 |
| 11–20 | 71,68 | 3,10 s | 14/138 = 10,1% | 93, di cui 9 hard | 31 |
| 21–25 | 143,12 | 4,19 s | 19/51 = 37,3% | 32, nessuna hard | 0 |

Le colonne di ritorno e durata sono medie aritmetiche dei valori riportati per iterazione; le percentuali sono ricavate dai contatori di terminazione degli episodi nel blocco. Non hanno necessariamente lo stesso denominatore.

Il miglioramento è netto soprattutto dopo l'iterazione 15. Il ritorno sale da 10,90 alla prima iterazione a 159,18 alla 25; nelle ultime cinque scompaiono timeout swing e terminazioni hard. Restano frequenti le interruzioni per penetrazione ordinaria. Il best all'ultima iterazione e l'andamento recente non indicano un plateau consolidato.

Le loss di posizione normalizzate, nelle medie mobili TensorBoard, passano da 0,2053 a 0,04366 al ginocchio e da 0,3260 a 0,04245 alla caviglia. Sono diminuzioni del 78,7% e dell'87,0%, non riduzioni percentuali del RMSE espresso in gradi.

### Confronto con le prime 25 iterazioni V26

| Metrica | 39D attuale | V26 del 20 agosto |
|---|---:|---:|
| Ritorno medio all'iterazione 25 | 159,18 | 161,62 |
| Durata media all'iterazione 25 | 4,20 s | 4,46 s |
| Transizioni cumulative alla 25 | 102.400 | 130.444 |
| Ritorno medio delle iterazioni 21–25 | 143,12 | 151,50 |
| Durata media delle iterazioni 21–25 | 4,19 s | 4,71 s |
| Episodi completi registrati nelle iterazioni 21–25 | 19/51 = 37,3% | 50/63 = 79,4% |

Il nuovo run ha raccolto il 21,5% di transizioni in meno alla stessa iterazione. Il budget più vicino nel V26 è l'iterazione 20: 103.032 transizioni, ritorno 119,79, durata media 4,24 s. Quindi, a esperienza simile, il ritorno attuale è incoraggiante e la durata media è vicina. Non significa che la nuova rete sia superiore: target, FSM e gestione degli errori differiscono.

Il V26 storico aveva inoltre crash di worker che possono non comparire nei contatori di fine episodio. Le percentuali di completamento sono descrittive degli episodi registrati, non una stima corretta per ogni possibile fallimento. Le reward e le loss normalizzate usano target diversi: valori vicini non provano pari accuratezza cinematica.

### Approfondimento richiesto dall'utente: perché il budget differisce

Il confronto dei due YAML risolti conferma batch nominale **4096**, minibatch **512**, **10 epoche** e **13 worker** in entrambi. Non è stato impostato un batch più piccolo per la 39D. Tuttavia RLlib raccoglie frammenti in parallelo fino a raggiungere almeno il budget: il valore nominale non è sempre il numero effettivamente consegnato al learner.

Con tutti i 13 worker operativi, i frammenti automatici sono 316 step per un worker e 315 per gli altri dodici: `316 + 12×315 = 4096`. Nel V26 erano documentati guasti dei worker causati dalle guardie fisiche. Se un worker non restituiva il proprio frammento, il round poteva fermarsi, per esempio, a 3781 campioni. Essendo sotto 4096, RLlib richiedeva un altro round e conservava il batch risultante: `3781 + 3781 = 7562`. Il ramo di campionamento usato da PPO e il successivo passaggio dell'intero batch al learner sono verificabili nel codice RLlib installato.

Il [report del 20 agosto](2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md) documenta 43 morti worker e i round aggiuntivi. Il [resoconto del 21 agosto](2026-08-21_report_approfondito_dal_training_imitativo_v26_alla_fsm_v3.md) descrive sia questo meccanismo sia la successiva conversione delle guardie fatali in terminazioni pulite. Quelle correzioni erano state conservate nel preset attuale. Quindi l'allineamento dei parametri non riproduce il comportamento dei vecchi guasti, né il loro effetto sul campionamento. Non è stata ricostruita qui la sequenza originale di ciascun guasto per round.

La verifica sui dati del learner conferma che i campioni aggiuntivi non erano un semplice artefatto del contatore:

| Quantità nelle prime 25 iterazioni | 39D | V26 |
|---|---:|---:|
| Batch nominale | 4096 | 4096 |
| Batch effettivo all'iterazione 1 | 4096 | 5672 |
| Batch effettivo all'iterazione 24 | 4096 | 7562 |
| Batch effettivo all'iterazione 25 | 4096 | 7562 |
| Transizioni raccolte e consegnate al learner | 102.400 | 130.444 |
| Campioni in eccesso rispetto a 25×4096 | 0 | 28.044 |
| Passaggi su minibatch da 512, incluse le epoche | 2025 | 2576 |

Per ogni iterazione è stata verificata l'uguaglianza tra incremento delle transizioni e `learner_connector_sum_episodes_length_in`. I passaggi sui minibatch derivano dal contatore dei campioni elaborati dal modulo diviso per 512; includono riutilizzo nelle epoche, arrotondamenti e struttura dei batch. Non sono transizioni distinte.

La minore esperienza della 39D deriva quindi dal **maggior campionamento storico rispetto al budget nominale**, non dal fatto che gli episodi attuali terminino prima. Nel run attuale, quando un episodio termina, il worker ne inizia un altro e completa comunque il proprio frammento.

### Completamento a quantità di esperienza confrontabile

Le finestre più vicine sono le iterazioni **21–25 della 39D** e **16–20 del V26**. Entrambe contengono esattamente 20.480 transizioni; gli estremi del budget cumulativo differiscono di appena 632 campioni.

| Quantità | 39D, iterazioni 21–25 | V26, iterazioni 16–20 |
|---|---:|---:|
| Budget cumulativo nella finestra | 81.920 → 102.400 | 82.552 → 103.032 |
| Episodi completi registrati | 19/51 = **37,3%** | 19/49 = **38,8%** |
| Interruzioni per penetrazione | **32** | **13** |
| Interruzioni per timeout stance/swing | **0** | **17** |
| Durata media delle iterazioni | 4,19 s | 3,89 s |

Il divario 37,3%–79,4% a parità di numero di iterazioni resta un dato corretto, ma non basta a dimostrare un ritardo equivalente a parità di esperienza. Il confronto per campioni non mostra un grande divario nella frazione di completamento; **mostra invece un problema specifico di penetrazione**, che nella 39D sostituisce i timeout presenti nel V26. Non consente di attribuirne la causa al prescribed, alla FSM o all'esplorazione. Resta il limite dei guasti storici non registrati come fine episodio.

Il primo controllo futuro che pareggerebbe circa il budget del V26 alla 25 sarebbe la **32 della 39D**: 131.072 contro 130.444 transizioni. La proposta di arrivare alla 40 conserva il significato di confronto con il budget del V26 intorno alla 32; il checkpoint 32 diventa però una verifica intermedia particolarmente informativa. Nessun ulteriore training è stato avviato durante questo approfondimento.

[Immagine rimossa: Confronto delle prime 25 iterazioni — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-6912e4c36c25f10b>)

## Rollout deterministico del best25

Il rollout usa il checkpoint della 25 con la sua configurazione, offset 1 s e 500 step. Arriva normalmente al limite temporale: `episode_time_limit`, `terminated=false`, `truncated=true`. Quest'ultimo flag indica il limite dei 5 s, non una caduta.

| Metrica | 39D best25 | V26 best87, riferimento finale |
|---|---:|---:|
| Durata | 5,00 s | 5,00 s |
| Ritorno deterministico | 260,52 | 363,79 |
| RMSE ginocchio rispetto al proprio target | 7,58° | 6,91° |
| RMSE caviglia rispetto al proprio target | 5,45° | 1,46° |
| RMSE giunto–reference servita, ginocchio | 0,27° | 0,18° |
| RMSE giunto–reference servita, caviglia | 0,32° | 0,28° |
| Penetrazione massima | 17,88 mm | 14,80 mm |
| Cicli completi riconosciuti | 2 | 2 |

**Il V26 qui è alla best87, non alla 25.** La tabella misura la distanza dalla qualità storica finale, non il ritardo di apprendimento a parità di training. Anche la reward deterministica non va confusa con il ritorno medio del training, che include l'esplorazione.

La penetrazione attuale resta sotto i 20 mm che attivano la penalità e i 28 mm che determinano la terminazione. Non è penetrazione geometricamente nulla. Nel rollout non compaiono invalid event, timeout, risincronizzazioni o cancellazioni HS; i due cicli non dipendono da riparazioni della FSM v3.

### Difetti cinematici ancora presenti

**Ginocchio.** La flessione massima raggiunge 42,66° rispetto ai 56,23° richiesti: mancano circa 13,57° al picco. L'escursione complessiva è 29,41° contro 45,35°, circa il 65% del target. Nella parte 60–100% della fase prescritta il RMSE è 10,71°, contro 4,54° nella parte 0–60%. La vicinanza del RMSE globale al V26 non equivale a una flessione altrettanto buona: nel V26 il picco raggiungeva 63,78° contro 64,32° richiesti.

**Caviglia.** L'angolo simulato resta fra +1,75° e +23,88°, mentre il target richiede da −6,61° a +22,05°. La parte negativa della traiettoria non viene riprodotta. L'escursione è 22,13° contro 28,66°, circa il 77%. Anche qui l'errore cresce nella parte finale della fase: 6,98° contro 4,18°. I valori estremi descrivono l'intero rollout e non sono necessariamente contemporanei.

**Regolarità dei contatti.** I due cicli hanno periodi di 1,680 e 1,544 s, ma quote di stance rispettivamente del **38,1% e dell'81,0%**. Nel rollout storico sono 33,8% e 36,7%. Il secondo toe-off completo della nuova rete arriva a 16,811 s, contro 16,153 s del V26; l'heel-strike successivo è invece vicino nei due casi, 17,105 contro 17,094 s. Il problema non emerge dal solo conteggio dei cicli: l'alternanza stance/swing è ancora irregolare. Queste sono misure del rilevatore/FSM del simulatore, non una validazione clinica del passo.

[Immagine rimossa: Confronto cinematico dei rollout — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-94a694cc917347a7>)

## Cosa suggerisce la diagnosi

**Il tracking a valle riproduce bene la reference ricevuta.** Il RMSE reference–target è già 7,48° al ginocchio e 5,42° alla caviglia; quello giunto–reference è soltanto 0,27° e 0,32°. La maggior parte del difetto imitativo osservato è quindi presente nella traiettoria servita. Non emerge un'incapacità dominante del sistema SEA di seguire quella traiettoria. Questi dati non separano da soli il contributo della policy da quello del governor, che resta quello concordato.

**La rete usa gli ingressi prescribed e il clock.** I sei canali sono stati verificati su tutti i 500 step. Ricalcolando offline la policy sugli stati registrati, azzerare il clock modifica le medie delle azioni con RMS 0,113/0,136; azzerare le velocità prescribed produce 0,302/0,178. Ritardare insieme i sei ingressi di 250 ms produce 0,504/0,330. Sono misure di sensibilità, non rollout fisici con maschera: dimostrano dipendenza dagli ingressi, non correttezza completa della sincronizzazione né robustezza all'oscuramento.

**L'esplorazione può contribuire alla differenza tra training e rollout deterministico.** Le deviazioni standard delle azioni sono ancora 0,746 e 0,887 nelle coordinate normalizzate della policy. Sugli stati del rollout, la probabilità gaussiana stimata di superare i limiti delle azioni è circa 32,6% e 27,6%. È una spiegazione plausibile delle molte interruzioni nel training, non una causa dimostrata: non è stata eseguita una valutazione fisica stocastica dedicata. Non basta per attribuire tutte le penetrazioni all'esplorazione.

Rispetto alla precedente prova 39D con velocità non scalate, il quadro imitativo è migliore: il suo rollout best78 sullo start centrale aveva RMSE 12,39°/7,46° e flessione massima circa 28,92°. Qui, già alla 25, si ottengono 7,58°/5,45° e 42,66°. Tuttavia gli start e più impostazioni differiscono: è un confronto descrittivo, non un'ablazione controllata del singolo fattore.

## Prossimo controllo consigliato

Proseguirei **senza cambiare configurazione fino a 40 iterazioni totali**, riprendendo lo stato completo della 25. Sono altre 15 iterazioni, indicativamente circa 2 ore con i tempi osservati. Questa è una proposta; nell'ambito di questa analisi non è stato avviato altro training.

La scelta ha una motivazione nello storico: il V26 raggiunse per la prima volta durata media di 500 step all'iterazione 32, dopo 159.116 transizioni. La 39D arriva a 163.840 transizioni alla 40. Il raggiungimento storico dei 500 step non dimostra da solo una cinematica soddisfacente, ma offre un riferimento concreto per il budget del prossimo controllo.

Controllerei un checkpoint intermedio alla 35 e il risultato alla 40, usando lo stesso rollout e osservando insieme:

1. Completamento dei 5 s e margine sulle soglie di penetrazione.
2. Crescita della quota di episodi completi durante training, su finestre di più iterazioni.
3. Recupero della flessione del ginocchio e della parte negativa del target di caviglia.
4. Riduzione del RMSE e maggiore regolarità delle durate stance/swing.

Il test a 25 iterazioni ha già prodotto un segnale sufficiente per proseguire questa configurazione. Se il ritorno continuasse a salire senza recuperare ampiezza, contatti e accuratezza, quello sarebbe un motivo per tornare alla diagnosi della funzione obiettivo e dei comandi generati. L'equivalenza statistica richiederà successivamente più seed e un protocollo di valutazione comune con margini definiti prima del confronto.

## Artefatti, file e verifiche

- [Configurazione congelata del run — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-06e61a6e2e8d833e>).
- [Riepilogo training — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-c969496862b0aacf>).
- [Riepilogo rollout best25 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-225434faef44cfcd>).
- [Metriche complete e audit — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-5fc61642770cf868>).
- [Audit dei batch effettivi e delle finestre a pari esperienza — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-731f4410204232a1>).
- [Script riproducibile di analisi — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-89046076f6d419c5>).

Sono stati prodotti tutti i sette grafici canonici:

1. [Comandi SEA e reserve — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-4c15a64ef464c6df>).
2. [Stati di giunti e motori — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-4d1a065b46f68ab6>).
3. [Coppia, angolo e potenza nel ciclo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-8b1f1a284f386603>).
4. [Velocità e potenza nel ciclo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-c03eb8cd7af55629>).
5. [Coppia in ingresso ed errore di tracking — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-604d88e751bad8ba>).
6. [Giunti, reference e tracking SEA — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-4cbbe920deb868c0>).
7. [Imitazione del target sinistro ed errore — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-a18cbfd59dad4bd4>).

Il nome storico del file 07 contiene `sound_leg`, ma target, curve d'errore ed etichette principali usano il prescribed sinistro della reward. La riga centrale con la gamba destra è solo un riferimento visivo.

In questa analisi sono stati aggiunti lo script, le metriche e le due figure comparative nella cartella `validation/v26_aligned_25iter_2026-09-21/`, gli output del rollout best25 con i sette grafici e questo report. Non sono stati modificati codice di training, FSM, plugin o controllori SEA.

Verifiche eseguite: audit delle 25 iterazioni e degli hash, visibilità integrale dei batch, rollout OpenSim di 500 step completato, controllo dei sei ingressi prescribed, ricostruzione esatta del ritorno dalla traccia (errore numerico zero), riproduzione delle azioni della policy con errore massimo 6,56×10⁻⁷, produzione dei sette grafici e ispezione delle curve comparative e del grafico canonico di imitazione. Le sonde offline non aggiornano i pesi e non costituiscono ulteriori iterazioni di training.
