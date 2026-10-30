# Indagine generale 39D: imitazione LEFT e forma delle coppie

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 29 settembre 2026. Instruction check token: `CMC_AGENT_OK_2026`.

Stato: indagine completata, con otto rollout diagnostici. Nessun training o
cambiamento di produzione.

## Problema e vincoli dell'indagine

Per qualità l'utente intende errore e forma della cinematica e principalmente
forma della curva di coppia; V26 è il buon benchmark. Reward e rete devono
rimanere riutilizzabili, senza ottimizzare pesi o regole appositamente per AB06.
Non sono disponibili altri modelli: si indaga con LEFT e RIGHT sullo stesso
AB06 e si rinvia ogni verifica della generalizzazione fra modelli.

La capacità della 39D di eseguire V26 sullo stesso target RIGHT resta un fatto
acquisito. Il [controllo del 23 settembre](2026-09-23_v26_ordine_aritmetico_e_robustezza.md)
ha riprodotto cinematica, coppie e GRF byte per byte mantenendo l'ordine delle
operazioni originali. RIGHT60, usata come ulteriore controllo, è una diversa
policy appresa e non va confusa con quel trasferimento riuscito.

## Strategia eseguita

Sono stati separati quattro aspetti: qualità delle curve, capacità del governor,
feedback autoregressivo e preferenze della reward rispetto all'apprendimento.
Le analisi ricostruiscono cinque rollout storici, riproducono l'inferenza dei
checkpoint e usano il governor di produzione. Due nuove baseline ripetono gli
archivi esattamente: 500 azioni e osservazioni per caso e 17 STO identici per lato.

Si costruiscono poi sequenze di endpoint anticipati con **la stessa procedura
per LEFT e RIGHT**, senza cambiare limiti o cutoff del governor. Anche la formula
analitica di inversione senza ottimizzazione migliora nettamente entrambi i lati.
Le sequenze ottimizzate sono verificate nella simulazione fisica completa,
con controller, plugin, contatto, FSM e protezioni invariati.

Queste sequenze hanno accesso al target futuro: sono una prova diagnostica di
ciò che la catena può realizzare, **non una policy appresa né una tabella AB06
proposta per l'uso futuro**. Il protocollo non cambia dopo aver osservato gli
esiti: [protocollo e artefatti — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-38baf8345b574576>).

## Risultato principale

Entrambe le sequenze completano 500/500 passi. Nel LEFT, sulla stessa finestra
13,15–17,95 s:

| Indicatore | Best80 LEFT | Sequenza inversa LEFT |
|---|---:|---:|
| RMSE actual–target knee / ankle | 6,376° / 4,040° | **0,390° / 0,339°** |
| Escursione profilo knee / target | 0,750 | **1,001** |
| Errore di forma normalizzato knee / ankle | 0,362 / 0,250 | **0,021 / 0,037** |
| Residuo raw−LP20 della coppia knee, RMS | 3,783 Nm | **0,121 Nm** |
| Impulso verticale del piede | 354,28 N·s | **1166,49 N·s** |
| Reserve traslazionali pelvis, RMS vettoriale | 418,95 N | **222,41 N** |
| Penetrazione massima nello STO | 14,149 mm | 19,028 mm |

Il miglioramento cinematico conserva l'escursione del target e non deriva da
scarico del piede o maggior supporto traslazionale del bacino. Le reserve
biologiche rotazionali aumentano lievemente, del 2,3%; quelle protesiche restano
zero. La penetrazione aumenta, pur restando sotto la soglia soft esistente di 20 mm.

La coppia ankle diventa prevalentemente negativa sotto carico, seguendo il
cambiamento del momento GRF; il picco positivo nella finestra è circa 1,22 Nm.
Le curve diventano regolari. **La parità qualitativa della forma delle coppie
con V26 resta da dimostrare**: cambiano reference, carico, distribuzione della
coppia e ampiezze. Per esempio, il picco knee in valore assoluto è 68,9 Nm contro
circa 30,0 Nm V26. Questa differenza non costituisce automaticamente una
bocciatura della sequenza né giustifica imporre la curva V26 a LEFT.

![Confronto LEFT focalizzato](../../plot/09_29_2026_general_method_diagnosis/quality/LEFT_focused_comparison.png)

La figura conserva la fase prescribed: V26 usa la propria reference RIGHT ed
è un confronto descrittivo. Le bande cinematiche descrivono la variabilità fra
i due cicli disponibili, non l'incertezza statistica. Il segno knee segue i
plot canonici; i dati grezzi OpenSim sono conservati con la loro convenzione.

## Reward e implicazione per il metodo

Con la **reward LEFT invariata**, il ritorno sull'intero episodio passa da
305,06 a **478,66**. Esiste quindi una sequenza molto più precisa e regolare,
fisicamente realizzata e nettamente preferita dalla reward attuale. Per ottenere
questo ordinamento non serve un nuovo tuning dei pesi dedicato ad AB06.

Il risultato rende prioritario studiare come la rete possa apprendere la
compensazione del governor. Non dimostra che PPO la trovi automaticamente,
che quei comandi siano ricostruibili in ogni condizione dai 39 ingressi o che
la reward rappresenti perfettamente qualunque requisito sulla forma delle coppie.

Il controllo RIGHT chiarisce il limite: ottiene RMSE 0,631°/0,522° e coppie
regolari, ma penetrazione di 25,074 mm e un momento penalizzato dal termine GRF
attuale. Con lo stesso contratto pen2, ritorno 341,13 contro 324,15 di RIGHT60
e 363,79 di V26. Il ritorno runtime RIGHT 348,91 usa pen1: non va usato nel confronto comune.
Invertire il governor risolve la parte cinematica, non garantisce da solo
contatti e coppie desiderabili per qualunque reference.

## Feedback autoregressivo: risultato delle perturbazioni

Le quattro prove con impulso +0,005, per un singolo segmento da 10 ms,
raggiungono tutte i 250 passi previsti. Prima dell'intervento i 101 passi
coincidono esattamente con la propria nuova baseline. Il breve orizzonte è
distinto da una verifica di robustezza dell'intero episodio.

LEFT amplifica l'impulso nel comando successivo, a +10 ms; la differenza
articolare supera 0,1° a +60/+70 ms, prima della divergenza FSM a +470 ms.
RIGHT mostra anch'essa amplificazione, con tempi diversi. Il limiter knee
RIGHT assorbe completamente l'impulso iniziale nella reference: i due
esperimenti knee non rappresentano lo stesso disturbo del governor.

Per l'ankle, invece, il primo cambiamento di reference è praticamente identico
fra LEFT e RIGHT, senza intervento del limite di jerk ankle. Il massimo
guadagno finito del comando applicato nei primi 50 ms è circa 106,8 LEFT e
4,22 RIGHT. Gli stati fisici complessivi sono diversi: non è un confronto
causale dei soli pesi o una certificazione di stabilità.

L'analisi offline localizza molta sensibilità nelle accelerazioni della
reference servita, più che nell'ultimo endpoint. Il percorso è presente anche
nella buona V26. Le prove confermano quindi una reale amplificazione della
catena prima dei cambiamenti FSM, ma non una causa unica del divario LEFT.
[Risultati e verifiche degli impulsi — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-f91994e37ae150ae>).

## Storico e letteratura

Il precedente del limiter errato e il progetto del 31 agosto indicavano già
l'utilità di un teacher che compensasse la catena effettiva di controllo:
questa indagine ne dimostra ora la fattibilità nominale LEFT. I precedenti
di giugno e settembre sul COP spiegano perché la coppia ankle possa essere
corretta rispetto al momento esterno anche quando la sua forma è indesiderata.
La riduzione del jerk di giugno aveva inoltre ridotto l'alta frequenza
peggiorando imitazione e contatto: liscezza e qualità non sono sinonimi.

Le fonti primarie consultate sostengono la separazione fra tracking cinematico,
cinetica e dinamica del contatto, e la necessità di considerare filtri e
regolarità durante l'apprendimento. Non forniscono pesi trasferibili ad AB06:
[MIMOC](https://arxiv.org/html/2305.10989v1),
[controllo di giunti flessibili, ICRA 2022](https://www.ais.uni-bonn.de/papers/ICRA_2022_Pavlichenko.pdf),
[CAPS](https://cs-people.bu.edu/rmancuso/files/papers/ICRA21_1616_FI.pdf),
[OpenSim Moco](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1008493).
La [mappa dettagliata delle evidenze — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-e5fc4507110fe2ce>)
distingue esplicitamente risultati storici risolti, ipotesi e limiti del riuso.

## File e verifiche

Tutto il codice aggiunto vive in
`Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/`:
`governor/`, `feedback/`, `quality/`, `physical/` e `dynamic/`. I rollout nuovi
sono in `unified_MLP_39/runs/rollout/general_method_diagnosis_2026-09-29/`;
i plot nel root `plot/09_29_2026_general_method_diagnosis/`.

Verificati inferenza 1500/1500 esatta, replay del governor, gradienti e
perturbazioni finite, identità dei target, ricostruzione delle reward,
immutabilità degli input e dei checkpoint, e corrispondenza fra reference
offline e fisica. Non vengono modificati simulatori, controller, plugin,
reward, configurazioni operative o pesi. Le verifiche native sono su macOS
arm64; il codice diagnostico è portabile, ma manca una verifica live Windows.

Generati sette file canonici per ciascuno degli otto rollout, quindi 56 PNG,
oltre alle figure comparative. Nei brevi impulsi alcuni pannelli per ciclo
sono esplicitamente non disponibili perché manca un ciclo completo; l'analisi
degli impulsi usa i segnali nel tempo, senza ricostruire cicli mancanti.

La [verifica conclusiva — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-72d96d3e41c9e7b1>)
ricontrolla manifest, checkpoint, ricevute e figure.

## TODO emersi, da propagare alla prossima chiusura

- [ ] Definire un esperimento di apprendimento della compensazione del governor,
  con teacher ricalcolato dallo stato effettivamente visitato e criteri comuni
  LEFT/RIGHT; non usare il solo fit di una sequenza nominale come prova.
- [ ] Verificare il metodo su altri start e tratti disponibili di AB06,
  includendo recupero dopo perturbazione e comportamento dei contatti.
- [ ] Separare nella valutazione della coppia la regolarità dalla forma legata
  a reference/carico, senza copiare la waveform V26 o introdurre regole AB06.
- [ ] Verificare generalizzazione ad altri modelli quando saranno disponibili
  e completare la verifica nativa Windows. Non sono prerequisiti per leggere
  gli esiti diagnostici odierni.
