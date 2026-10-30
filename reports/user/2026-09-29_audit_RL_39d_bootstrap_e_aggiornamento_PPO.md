# Audit RL della 39D: bootstrap del critic e aggiornamento PPO

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 29 settembre 2026. Instruction check token: `CMC_AGENT_OK_2026`.

Stato: raccolta, aggiornamento originale, replay controllato e tutte le
13 valutazioni fisiche completati. Nessuna modifica al training di produzione.

## Problema e impostazione

L'indagine verifica perché il training RL della 39D LEFT non raggiunga ancora
la qualità imitativa e la forma delle coppie desiderate. Il precedente
[esperimento sul governor](2026-09-29_indagine_generale_39d_imitazione_e_forma_coppie.md)
aveva trovato una sequenza nominale migliore e favorita dalla reward, usando
però il target futuro. Quel risultato non dimostrava l'apprendibilità di una
policy causale con gli ingressi disponibili.

Qui non si aggiungono teacher o supervisione offline dei comandi. Restano la
reference prescribed e i canali osservativi e imitativi del training originale.
Reward, rete, governor, controllori e plugin restano invariati. Il modello disponibile è AB06: LEFT e
RIGHT sono due reference dello stesso modello, non una verifica fra modelli.
V26 rimane un benchmark descrittivo di buona qualità; le sue coppie non sono
imposte a LEFT, che ha reference e condizioni di carico diverse.

La qualità richiesta resta distinta dal ritorno: errore e forma della
cinematica, principalmente forma della curva di coppia. Il piano separa
esplorazione, preferenze dell'obiettivo, stima dei vantaggi, aggiornamento dei
pesi e comportamento fisico successivo.

## Esperimento realmente eseguito

Dal checkpoint completo LEFT best80, con peso penetrazione 2, è stata eseguita
una nuova raccolta on-policy di 4096 transizioni e una sola iterazione PPO
logica 81: 13 worker, minibatch 512, 10 epoche, gamma 0,99, lambda 0,9, seed 123.
Sono stati ripristinati actor, critic, 12 stati Adam e stato ausiliario PPO:
step Adam 4455, learning rate 0,0001 e coefficiente KL effettivo 0,3000000119.
Nessun critic rewarm. La reward è stata verificata in tutti i 13 worker fisici.

La strumentazione ha salvato episodi prima della GAE, valori critic originali,
batch dopo i connector, 81 minibatch effettivi e snapshot dei pesi. Sono presenti
4096 righe reali e 14 righe artificiali escluse dalla loss. Dei 14 frammenti,
12 sono ancora aperti al limite della raccolta e 2 terminano per penetrazione.

L'inferenza per singola riga ricostruisce esattamente logits, logprob e ratio
iniziale 1. L'inferenza sul batch introduce soltanto arrotondamenti float32,
con scarto massimo dei logits 2,15e-6. La logprob archiviata corrisponde
all'azione Gaussiana raw; usare l'azione già clippata darebbe un controllo
sbagliato. Non emerge qui un disallineamento fra policy sampler e learner.

Il primo tentativo di avvio fallì durante la costruzione dei worker, prima di
qualsiasi campione o aggiornamento, per un import mancante nel PYTHONPATH.
È conservato separatamente. Il launcher corretto ha concluso l'esperimento;
non è stata ripetuta un'iterazione scientifica fallita.

## Difetto accertato nel segnale di apprendimento

La versione installata Ray/RLlib 2.55.1 annulla il valore della prossima
osservazione quando il frammento finisce per la raccolta del batch. Il
connector marca terminale la riga artificiale di bootstrap e il calcolo dei
target elimina quel valore. La stessa composizione introduce un errore
nella propagazione precedente una vera terminazione.

Per un confine del campionamento il target dell'ultima transizione deve
includere `r + gamma * V(s_successivo)`. Il bootstrap va annullato quando la
transizione termina realmente l'episodio. Si tratta della distinzione fra
terminazione e troncamento, documentata anche da
[Gymnasium](https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/).

Esempio misurato nel primo frammento aperto: reward 0,729587,
valore successivo 42,72012. Il target stock è 0,729587; conservando il bootstrap
diventa 43,022505. Nel batch la differenza massima è 56,3589 e supera 1 nel 9,52%
delle transizioni. La media dello scarto target corretto meno stock è +1,09188.

Un riproduttore indipendente usa i connector PPO nativi, due reward `[1,2]`
e valori noti `[2,3,4]`, senza OpenSim o AB06:

| Fine del frammento | Target stock delle due transizioni | Target TD/GAE corretto |
|---|---|---|
| Raccolta interrotta o troncamento |3,079; 2,000|6,60736; 5,960|
| Vera terminazione |2,782; 2,000|3,079; 2,000|

I tre file installati coinvolti coincidono byte per byte con il tag ufficiale
Ray 2.55.1. Il progetto Ray ha corretto questi casi nella
[PR 59958](https://github.com/ray-project/ray/pull/59958), integrata il 27 maggio 2026
con commit `1927658ad6a97788bd86fc59310d1ed45a08e60e`. Metadata, patch e sorgenti
con hash sono conservati nella cartella `upstream/` dell'audit.

La verifica che la GAE registrata coincida con quella installata rimane valida
come verifica dell'acquisizione; non prova la correttezza matematica della
formula installata. Il difetto è emerso dal secondo confronto, indipendente.

![Target e vantaggi vicino al confine del primo frammento](../../plot/09_29_2026_rl_learning_audit/learner/05_fragment_boundary.png)

La discesa del target stock deriva dal confine della raccolta, pur mantenendo
le stesse reward, osservazioni e stime del critic. La differenza si propaga
anche alle decisioni precedenti attraverso la GAE.

## Che cosa sta facendo PPO sul proprio batch

| Misura sul medesimo batch e target stock fisso | Prima | Dopo l'update |
|---|---:|---:|
| Policy loss surrogate |−0,020205|−0,038008|
| MSE critic grezza |33,9655|28,9611|
| Frazione errori quadratici oltre il cap 10 |8,40%|6,98%|
| KL media rispetto alla policy iniziale |circa 0|0,011478|

L'ottimizzazione quindi aggiorna effettivamente actor e critic e migliora
l'obiettivo sul batch usato per il training. Questo non equivale a un
miglioramento fisico, né certifica la qualità dei target ottimizzati.

Usando il critic iniziale invariato contro i target con bootstrap corretto,
la MSE è 7,3679 e la quota oltre cap è 2,66%. Non è un critic diventato migliore:
è cambiato il bersaglio con cui lo si confronta. Una parte dell'apparente
grande errore del critic proviene dunque dalla costruzione errata dei target.

Il codice stock normalizza inoltre le advantages includendo le righe
artificiali, poi escluse dalla loss. Questo effetto è misurato separatamente
e non viene confuso con il difetto del bootstrap.

## Esplorazione e significato della policy nominale

Circa il 52,98% delle transizioni ha almeno una componente raw oltre i limiti:
31,98% knee e 34,38% ankle. Le sigma medie sono 0,733/0,794. Il clipping non prova
da solo che il policy gradient sia errato; cambia la relazione fra la media
della Gaussiana, l'azione attesa applicata e il rollout deterministico.

Sugli stessi stati del batch, la differenza media fra endpoint angolare atteso
e endpoint della media clippata è −3,394°/−3,009° prima dell'update. Queste
quantità sono prima del governor, non errori degli angoli articolari. Il
training ottimizza una policy stocastica; il nominale ne usa la media.

Tutti i 4096 campioni sono stati confrontati con best80 nominale agli stessi
istanti e con gli stessi target. Un frammento di 315 step migliora entrambi
gli RMSE: 4,948°/2,751° contro 5,494°/3,055°. Peggiora però la forma normalizzata
di entrambi i giunti e ha reward inferiore di 9,209. I termini imitativi
migliorano, ma l'aumento degli altri costi li supera. Un altro miglioramento
degli RMSE riguarda solo 5 step. Nessun frammento migliora contemporaneamente
la forma di entrambi i giunti o supera la reward nominale sul proprio
intervallo. Questi prefissi non sono ritorni futuri completi né una prova
d'impossibilità dell'esplorazione.

## Confronto controllato e valutazioni fisiche

L'intervento condizionato è ripartito dallo stesso stato e dagli stessi episodi.
Il ramo di controllo ha riprodotto bit per bit actor, critic, Adam, stato
ausiliario e tutte le colonne, l'ordine e i forward degli 81 minibatch originali.
Il ramo corretto ha poi modificato solo target e advantages delle transizioni
reali, conservando le altre nove colonne del batch. Il segno del vantaggio
normalizzato cambia nel **23,999%** delle righe reali. Questa quota risente
anche della centratura e della scala globali: non significa che il 24% dei
target abbia un errore grande. Non è soltanto una
differenza nella metrica del critic: cambia il segnale che indirizza l'actor.
La significatività fisica va valutata separatamente.

Sono conservati target artificiali e convenzione di normalizzazione stock,
per isolare l'effetto ai confini. Non è un backport completo della patch
upstream. Il replay usa un learner locale senza Ray, ambiente o nuova raccolta.
I target reali corretti coincidono con quelli calcolati dal codice della
correzione ufficiale sui 4096 campioni, entro 3,82e-6 di arrotondamento float32;
questa equivalenza non viene estesa alle righe artificiali o alla normalizzazione.
Il primo avvio del replay si era fermato prima di qualsiasi update perché il
restore generico arrotondava il learning rate a float32; la seconda versione
ripristina esattamente gli iperparametri originali e supera il controllo completo.

Le valutazioni eseguite sono nominale e tre flussi Gaussiani appaiati
seed 123/125/127 per prima, dopo e ramo corretto, più il nominale al primo
passaggio sui dati: 13 casi totali. Ogni policy usa la propria sigma. Sono
conservati costi e motivi delle terminazioni anticipate; cinematica e coppie
si confrontano sul supporto temporale comune. Un timeout operativo non
viene convertito in un risultato RL. I plot canonici sono nel root `plot/`.

Il nominale mantiene la pipeline deployment, necessaria a riprodurre esattamente
il benchmark archiviato. I casi rumorosi usano anche la trasformazione azioni
del connector training (`normalize_actions=True`, `unsquash_action`). Il
confronto prima/dopo/corretto avviene sempre entro lo stesso protocollo;
non si confonde la media deterministica con una stima del ritorno stocastico.

Lo snapshot `first_pass` è dopo 9 minibatch, 4608 righe elaborate: supera il
primo passaggio e include parte del successivo, non coincide con una singola
epoca esatta. I ritorni episodici riportati sono somme non scontate. Un episodio
interrotto al limite temporale non fornisce il vero ritorno futuro infinito
del critic: la sua analisi distingue orizzonte finito e bootstrap esplicito.

Il confronto nominale prima/dopo standard è già concluso: entrambi completano
500 passi, con la stessa finestra 13,15–17,95 s.

| Nominale | Best80 prima | Dopo PPO standard |
|---|---:|---:|
| Ritorno episodico |305,0597|255,1864|
| RMSE actual–target knee / ankle |6,3758° / 4,0401°|6,3277° / 5,3866°|
| Errore di forma temporale knee / ankle |0,3882 / 0,3146|0,4019 / 0,3940|
| Residuo coppia raw−LP20 RMS knee / ankle |3,783 / 0,299 Nm|3,430 / 0,315 Nm|
| Coppia RMS knee / ankle |14,245 / 5,583 Nm|17,296 / 8,995 Nm|
| Impulso verticale del piede |354,3 N·s|364,0 N·s|
| Penetrazione massima nello STO |14,149 mm|17,121 mm|

L'errore di forma qui confronta le sequenze temporali centrate e normalizzate
alla propria deviazione standard, senza riallineare ritardi. Le misure
0,362/0,250 del precedente report sul governor riguardavano invece i profili
medi in fase: non sono la stessa metrica. Il miglioramento della surrogate
sul batch stock si accompagna qui a peggioramento del nominale, soprattutto
alla caviglia. Alcune misure di regolarità delle coppie diminuiscono, mentre
le ampiezze aumentano; questo non certifica una migliore forma biomeccanica.

Anche i tre confronti rumorosi standard sono conclusi:

| Seed | Ritorno prima | Ritorno dopo standard | Completamento prima / dopo |
|---|---:|---:|---|
|123|264,2062|223,8865|500 / 500 passi|
|125|212,4523|231,4851|500 / 500 passi|
|127|229,4650|267,6200|458, penetrazione / 500 passi|
|Media descrittiva|235,3745|240,9972|2/3 / 3/3 episodi completi|

La media include il fallimento fisico e la sua penalità. L'incremento osservato
di 5,6227 non è una superiorità statistica accertata con tre seed. Mostra però
che il peggioramento nominale non coincide, in questo screening, con un calo
della media dei ritorni stocastici. Le due valutazioni misurano comportamenti
diversi della stessa policy.

Lo snapshot intermedio completa 500 passi con ritorno 272,8570 e RMSE
5,9519° / 5,1434°. Migliora il ginocchio rispetto all'iniziale, mentre la
regressione della caviglia è già presente. Il passaggio al finale accentua
la perdita del ritorno nominale. Questo controllo descrittivo non dimostra
che ridurre le epoche risolva il problema complessivo.

Il nominale corretto è concluso e completa anch'esso 500 passi. Sulla stessa
finestra temporale:

| Nominale | Best80 prima | Dopo PPO standard | Dopo correzione bootstrap |
|---|---:|---:|---:|
| Ritorno episodico |305,060|255,186|279,378|
| RMSE actual–target knee / ankle |6,376° / 4,040°|6,328° / 5,387°|6,243° / 4,707°|
| Errore di forma temporale knee / ankle |0,388 / 0,315|0,402 / 0,394|0,412 / 0,351|
| Coppia RMS knee / ankle |14,245 / 5,583 Nm|17,296 / 8,995 Nm|14,626 / 5,565 Nm|
| Residuo coppia raw−LP20 RMS knee / ankle |3,783 / 0,299 Nm|3,430 / 0,315 Nm|3,705 / 0,335 Nm|
| Impulso verticale del piede |354,28 N·s|363,99 N·s|373,05 N·s|

Il ramo corretto attenua la regressione nominale della caviglia rispetto
all'update standard, senza recuperare la qualità iniziale. Migliora l'RMSE
del ginocchio ma peggiora il suo errore di forma. Le ampiezze RMS delle coppie
tornano vicine a best80; la loro forma non risulta per questo risolta.
La variazione totale della coppia ankle filtrata a 10 Hz, normalizzata e per
secondo, sale a 2,523 contro 1,758 stock e 2,015 iniziale. Non emerge una
riduzione generalizzata delle irregolarità.

Sui target corretti fissi la loss surrogate passa da −0,036341 a −0,043537
nel controllo e −0,048638 nel ramo corretto. La loss critic effettivamente
limitata dal cap migliora di più nel ramo corretto (1,42731 contro 1,76125),
ma la MSE grezza è inferiore nel controllo (6,83727 contro 7,08265). Non c'è
un miglioramento uniforme di tutte le misure del critic. La KL finale è
0,011478 stock e 0,009844 corretta, con coefficiente identico 0,3000000119.

Anche i tre casi stocastici corretti sono conclusi, tutti a 500 passi:

| Seed | Prima | Dopo PPO standard | Dopo correzione bootstrap |
|---|---:|---:|---:|
|123|264,206|223,887|294,385|
|125|212,452|231,485|277,273|
|127|229,465|267,620|244,768|
|Media descrittiva|235,375|240,997|272,142|

La correzione migliora il ritorno rispetto al controllo in due seed su tre:
le differenze sono +70,499, +45,788 e −22,852. Rispetto a best80 il ritorno
aumenta in tutti e tre i seed, mantenendo nel confronto il fallimento fisico
del seed 127 iniziale. Non ci sono episodi censurati per problemi operativi.
Sul supporto comune con best80 migliorano RMSE e forma cinematica dei due
giunti in tutti e tre i seed. Il confronto con PPO standard è meno uniforme:
nel seed 127 peggiorano entrambi gli RMSE e la forma del ginocchio; nel seed
123 peggiora la forma della caviglia pur diminuendo il suo RMSE.

Il risultato favorisce la correzione nello screening stocastico, ma il
nominale resta peggiore di best80 e le coppie non raggiungono una qualità
dimostrata equivalente a V26. La forma non si deduce dal ritorno medio.
I confronti con il seed 127 iniziale usano il prefisso comune di 458 passi;
non vengono aggiunti campioni o cicli dopo la sua terminazione.

![Cinematica e coppie nominali prima, dopo PPO standard e dopo correzione](../../plot/09_29_2026_rl_learning_audit/corrected_comparison/nominal_before_stock_corrected.png)

Sono inoltre valutati i tre critic su 12 rollout con osservazioni complete,
separati dal batch di training. Il nominale best80 riusato dall'archivio non
contiene l'osservazione completa del critic e resta esplicitamente escluso
da questa sola analisi. I ritorni appartengono alla policy che li genera;
non sono una verità comune sul valore di policy diverse. Anche con gli
stessi pesi, il nominale deterministico differisce dal comportamento
stocastico di cui il critic stima il valore. I ritorni a orizzonte finito e
quelli completati con bootstrap del critic iniziale sono distinti. Questi
confronti non certificano una calibrazione assoluta o un ranking generale
dei critic: dati e limiti sono nel
[confronto dedicato — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-cf7ae05d793f1630>).

## Cosa permette di concludere lo storico

V26, LEFT best80 e l'audit registrano tutti Ray 2.55.1 con lo stesso commit,
learner PPO stock e connector di default. Le configurazioni storiche
confermano il campionamento a frammenti. Il difetto è quindi potenzialmente
condiviso: non basta a spiegare perché V26 riesca e LEFT sia peggiore.
Non sono stati ricostruiti i target dei batch storici V26; la versione non
esclude patch locali storiche non documentate.

Il fix del 14 luglio rimuoveva righe mascherate dopo la GAE per compattare e
bilanciare i minibatch. Non documenta una correzione del bootstrap. V26 aveva
inoltre disabilitato quella modalità di campionamento esatto. Vedere la
[ricostruzione dello stack — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-4b99a28f2b144ccd>).

## File, verifiche e limiti

Il codice e i dati diagnostici sono in
`Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/`.
Il training isolato è `runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29`;
i rollout sono in `runs/rollout/rl_learning_audit_2026-09-29` e le figure in
`plot/09_29_2026_rl_learning_audit/`.

Le verifiche coprono acquisizione senza variazioni di loss/gradienti/RNG,
restore completo, policy e logprob, ricorrenza GAE indipendente, connector
nativi, ricostruzione reward, confini dei frammenti, trasformazione delle
azioni e formula analitica della Gaussiana clippata. Gli artefatti originali
sono congelati con SHA256. Sono passati 17 test mirati: 3 del probe, 5
dell'analisi learner, 6 dell'esplorazione e 3 del replay, oltre alle verifiche
analitiche della Gaussiana clippata e al confronto con il codice Ray ufficiale.

La [verifica finale — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-566342d48456be9f>)
è passata: 125 input iniziali invariati, 113 artefatti della raccolta,
riproduzione esatta degli 81 minibatch e delle nove colonne estranee alla
correzione, ricevute di tutti i 13 casi e 91 plot canonici verificati.
Sono controllati anche gli input e gli output delle analisi numeriche,
i quattro grafici dei confronti fisici e i flussi Gaussiani appaiati.
Dodici casi completano 500 passi; il solo fallimento fisico è best80 rumoroso
seed 127, a 458 passi. Nessun timeout operativo viene usato come esito RL.

I file nuovi principali sono `run_audit.py` e `learner_probe.py` per la
raccolta, gli script `learner_analysis.py` ed `exploration_*.py` per la
diagnosi, `replay_batch.py` e `compare_replay.py` per il confronto controllato,
`evaluate.py`, i due launcher e i due script di analisi fisica per i rollout,
infine `verify_upstream_equivalence.py` e `verify_results.py` per le verifiche.
Protocollo, README, sorgenti ufficiali, test e ricevute sono nella stessa
cartella dell'audit. Nessun file della pipeline di produzione è stato
modificato per applicare la correzione.

Tutte le modifiche relative alla rete restano nella cartella Trajectory
Generator. Non sono modificati controller, plugin, reward o dipendenze
installate; il training separato da zero già attivo resta distinto dall'audit.
Le esecuzioni native sono su macOS arm64; manca una verifica live Windows.

Un batch e un aggiornamento locale da best80 non riparano 80 iterazioni
pregresse, non provano convergenza da zero e non decidono se il RL possa
raggiungere V26. Tre seed di valutazione sono uno screening descrittivo.
La pista prioritaria è ora un difetto generale del segnale di apprendimento,
senza necessità dimostrata di teacher o tuning della reward per AB06.

## Risposta al quesito sul RL e lavoro successivo

Il risultato restringe il problema: la pipeline raccoglie azioni coerenti con
la policy e PPO ottimizza il segnale ricevuto, ma quel segnale contiene un
errore riproducibile nella costruzione dei target. Non è dimostrato che alla
39D servano comandi teacher per imparare. Non è neppure dimostrato che basti
correggere questo errore per ottenere la qualità V26: l'effetto misurato è
locale e i pesi iniziali derivano già da 80 iterazioni con lo stack storico.

Il passo successivo deve rendere affidabile il calcolo del segnale RL e
misurare un apprendimento controllato su più aggiornamenti. I controlli
matematici del bootstrap si verificano su piccoli episodi sintetici senza
AB06; la qualità fisica si verifica sui dati e sul modello disponibili.
La generalizzazione ad altri modelli resterà non verificata finché questi
non saranno disponibili.

## TODO da propagare

Aggiornamento del 30 settembre, su indicazione esplicita dell’utente: la
correzione del bug PPO è rinviata a un’attività successiva e resta un TODO
aperto. Non è un prerequisito per il pilot di oscuramento da best90; la
pipeline PPO attuale resta invariata per quel pilot. Questa decisione
aggiorna la priorità operativa descritta sopra, senza modificare gli esiti
dell’audit.

- [ ] Integrare in un esperimento separato la correzione generale dei confini
  PPO, con verifica dei connector completi, restore e riproducibilità; non
  confondere il replay isolato odierno con una patch già in produzione.
- [ ] Quantificare separatamente la normalizzazione delle advantages sulle
  sole transizioni reali rispetto all'inclusione delle righe artificiali.
- [ ] Confrontare training RL standard e corretto con stessa inizializzazione,
  reward, budget e condizioni di valutazione, distinguendo continuazione da
  best80 e apprendimento da zero. Includere repliche prima di conclusioni
  sulla stabilità o superiorità del metodo.
- [ ] Proseguire la diagnosi di esplorazione, clipping e differenza fra policy
  stocastica e nominale; verificare anche il feedback autoregressivo con
  interventi separati, senza ottimizzare la reward su questo singolo caso.
- [ ] Valutare altri start e tratti di AB06, includendo recupero e carichi;
  giudicare la forma della coppia insieme a cinematica e contatto, senza
  sostituirla con una sola misura di regolarità o con il ritorno.
- [ ] Completare la verifica live Windows e il trasferimento ad altri modelli
  quando disponibili. Il teacher resta una pista eventuale distinta, non
  un prerequisito dell'indagine RL autorizzata.
