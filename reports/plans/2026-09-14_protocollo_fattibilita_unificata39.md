# Protocollo preregistrato di fattibilità della rete unificata a 39 feature

**Data:** 2026-09-14  
**Instruction check token:** `CMC_AGENT_OK_2026`  
**Stato:** protocollo definito prima degli esperimenti; variante39 e campagna non ancora implementate o eseguite con la redazione di questo documento.

## 1. Autorizzazione, ordine e obiettivo

L'utente ha chiesto di eseguire la sequenza V1 5→50, valutazione intermedia,
50→100, valutazione finale e test di fattibilità. La fase qui descritta è
quindi autorizzata: non richiede un'ulteriore approvazione. Le precedenti
frasi «non implementare ora» si riferivano alla redazione dei report, non
costituiscono un divieto per la fase 5 ora richiesta.

**La directory e il codice della variante39 saranno creati, e i relativi
esperimenti saranno avviati, soltanto dopo il completamento e la valutazione
della V1 a 100 iterazioni.** Questo documento prepara esclusivamente il
protocollo. Eventuali materiali preparatori non costituiscono già la variante
in uso. La V1 a 35 ingressi, la baseline storica e i loro checkpoint restano
conservati. Tutto il nuovo codice neurale vive in un perimetro parallelo sotto
`Trajectory Generator/`; runtime condiviso, plugin C++ e semantica SEA restano
quelli esistenti.

L'obiettivo è verificare se rendere disponibili temporaneamente quattro
target prescribed sinistri produca una capacità utile dopo la loro rimozione.
Si distinguono correttezza software, controllo autonomo nel simulatore e
vantaggio trasferibile all'ex-novo. Il presente disegno è un **pilot di
fattibilità**; non costituisce il benchmark di parità con la baseline né una
qualifica statistica o biomeccanica.

Fonti: [ordine delle attività e decisione 39 ingressi](../user/2026-09-14_pilot_unificata_strategia_50_100_e_test_39_feature.md),
[protocollo di osservabilità e mascheramento](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md),
[protocollo distinto di parità](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).

## 2. Decisioni congelate prima di A/B/C

Le scelte seguenti erano ancora aperte nei report precedenti e sono fissate
qui come default sperimentali. Sono scelte del pilot, non soglie già validate
dal progetto. Il manifest esecutivo registrerà hash del protocollo, sorgenti,
configurazioni risolte, checkpoint, input e ambiente software.

| Elemento | Decisione |
|---|---|
| Actor | 39 → 256 tanh → 256 tanh → due medie e due logstd; due azioni assolute knee/ankle |
| Canali esistenti | Le 35 feature V1 conservano ordine, valori e trasformazioni; i due slot storici del clock prescribed restano zero |
| Canali aggiuntivi | Quattro copie dei target prescribed sinistri correnti, nell'ordine knee q, knee qdot, ankle q, ankle qdot; nessun quarantesimo bit |
| Posizione nello schema | In coda al prefisso actor, prima del suffisso privilegiato; schema completo ricavato e verificato dal probe |
| Normalizzazione nuova | Fissa: centro 0, scale [1 rad, 10 rad/s, 1 rad, 10 rad/s] nell'ordine dichiarato; nessuna statistica adattiva nuova o clipping implicito |
| Maschera | Zero nella rappresentazione consegnata alla rete, applicato dopo la trasformazione fissa; scelta per episodio, conservata nei campioni PPO |
| Seed del pilot | 123; inizializzazioni actor e critic abbinate e verificate fra A/B/C |
| Budget imitativo | 50 iterazioni complete per braccio, batch 4608: 230400 transizioni per braccio |
| Raccolta | 12 EnvRunner; 1536 transizioni per ciascuno dei tre start in ogni batch |
| PPO imitativo | Preset V1: minibatch 512, 10 epoche, LR 1e-4, clip 0.2, logstd apprendibile; reward e altri parametri conservati |
| Penetrazione | Peso 1.0 imitativo, 0.05 ex-novo; soglie soft/hard 20/28 mm invariate |
| Ex-novo B/C | 1 iterazione critic-only + 10 aggiornamenti PPO: 11 totali, 50688 transizioni per braccio |
| Repliche eventuali | Seed 456 e 789, stesso protocollo, prima di sostenere una conclusione qualitativa favorevole |

Il budget complessivo del pilot è 691200 transizioni imitative e 101376
transizioni ex-novo, escluse valutazioni e diagnostica. Le ripetizioni dovute
a errori tecnici devono essere rendicontate a parte; le transizioni dei
tentativi interrotti non vanno cancellate dal costo effettivo.

La scala fissa delle nuove velocità riduce la differenza di ordine di grandezza
fra posizioni e velocità senza stimare statistiche dai campioni del teacher.
È una scelta dimensionale del presente esperimento, non una normalizzazione
già validata nella V1. Le 35 feature originali e le loro trasformazioni
preesistenti restano identiche nei tre bracci e fra le modalità. In ex-novo
nessuna statistica può dipendere dalle quattro copie nascoste del teacher.

## 3. Curriculum e riprese ai confini dei blocchi

| Iterazioni totali | A: disponibilità per episodio | B: disponibilità per episodio | C: disponibilità per episodio |
|---|---:|---:|---:|
| 1–10 | 1.00 | 0.00 | 1.00 |
| 11–20 | 1.00 | 0.00 | 0.75 |
| 21–30 | 1.00 | 0.00 | 0.50 |
| 31–40 | 1.00 | 0.00 | 0.25 |
| 41–50 | 1.00 | 0.00 | 0.00 |

Ogni braccio usa cinque lanci del trainer con traguardi totali 10, 20, 30,
40 e 50. I lanci successivi effettuano **resume completo di actor, critic,
optimizer e fase**, ricreando gli ambienti come già avviene nella V1.
La stessa segmentazione è applicata ad A, B e C, anche quando la loro
disponibilità non cambia. Non si introduce un nuovo meccanismo di reset nel
runtime condiviso. In questo modo nessun episodio con teacher visibile può
attraversare il rilancio 40→41: tutte le ultime dieci iterazioni di C sono
interamente mascherate.

Il riavvio del processo ricomincia gli episodi e non equivale alla continuità
dello stato fisico del simulatore. Questa differenza rispetto a un unico run
ininterrotto deve apparire nel manifest e nella lettura dei risultati.
Non si dichiara riproducibilità esatta del RNG o della traiettoria fisica fra
resume e run ininterrotto senza una prova specifica. Le tracce e i riepiloghi
devono rendere riconoscibili riavvii e reset.

Il livello curriculum deriva dall'iterazione globale e viene persistito;
gli RNG dedicati alla maschera devono essere serializzati e il loro restore
verificato dai test. Il seed della replica identifica l'esperimento, senza
implicare da solo identità delle sequenze di rumore o dei reset fisici.
All'interno di un episodio non si ricampiona la disponibilità. La maschera
effettiva resta quella memorizzata nella transizione durante tutte le epoche
PPO. Non sono ammessi livelli scelti adattivamente guardando i risultati.

## 4. Diagnostica delle 35 osservazioni

Prima del training comparativo, costruire un dataset dalle tracce V1
disponibili a 5, 50 e 100 e dalle loro valutazioni. Registrare checkpoint,
start, episodio, timestamp, disponibilità del periodo online e target q/qdot.
La raccolta addizionale eventualmente necessaria è diagnostica e va
rendicontata separatamente dal budget A/B/C.

Le tracce storiche contengono osservazioni pre-step e target post-step:
accoppiare i campi della stessa riga senza verifica introdurrebbe un ritardo
artificiale. La coppia valida è osservazione a `t_pre` e teacher interpolato
a `t_pre`. Dalla seconda riga si può usare il teacher post-step della riga
precedente soltanto verificando continuità dei timestamp e assenza di reset;
il primo campione va ricostruito dall'interpolatore o escluso esplicitamente.
Conservare timestamp osservazione e target nel dataset finale. Non interpolare
attraverso reset, terminazioni o dati mancanti.

Eseguire una diagnostica di ricostruzione di q/qdot del teacher con regressione
ridge e regolarizzazione preregistrata `alpha=1.0`, usando:

1. le sole 35 osservazioni istantanee;
2. gli ultimi cinque campioni causali delle stesse osservazioni, incluso
   quello corrente: a 100 Hz gli estremi distano 0.04 s;
3. un **oracolo temporale diagnostico offline**: le 35 osservazioni insieme
   al tempo assoluto e a sin/cos con periodo nominale fisso 1.58 s.

Il terzo controllo non è una fase prescribed misurata: il periodo nominale
costituisce un ausilio diagnostico temporale preregistrato. Non se ne ricava
un clock da trasferire nell'actor e non lo si descrive come segnale causale
disponibile alla policy. Un confronto ulteriore con vera fase prescribed
resta una diagnostica opzionale futura, da etichettare e rendicontare
separatamente.

Normalizzazioni del modello diagnostico vanno stimate solo sul fit. Separare
fit e valutazione per episodi e condizioni; usare come primo split i tre
fold con uno start interamente escluso dal fit. Nessun timestamp, oracolo
temporale o fase prescribed entra nei modelli causali. Escludere le finestre di storia che
attraversano reset; riportare quanti campioni vengono esclusi. Quantificare
RMSE/MAE per giunto e componente, errore rispetto al predittore costante fit
e vicini osservabili con target divergenti.

Per tutti i modelli usare la stessa coorte con storia completa: la prima
riga è esclusa per mancanza del teacher pre-step e altre tre righe per
storia insufficiente; le osservazioni iniziali rimangono utilizzabili come
contesto causale. Le quattro uscite usano le scale fisse [1,10,1,10].
Richiedere almeno tre start utilizzabili, almeno dieci campioni di fit e
cinque campioni held-out per fold; dati inferiori danno un risultato
inconclusivo, non un errore di implementazione. Questi minimi evitano fit
puramente degeneri e non garantiscono sufficienza statistica.

Separare avvio, fino al primo periodo online valido maggiore di zero, e
regime successivo.
Se mancano cicli validi, il regime è **non valutabile**: non si riempiono le
metriche con zero e non si dichiara risolta l'osservabilità. Un fit scarso
non prova impossibilità; un fit buono non prova controllo stabile. Non si
sviluppa qui un teacher di comandi o una nuova pipeline supervisionata.

## 5. Gate software prima degli esperimenti comparativi

La variante deve superare i seguenti controlli prima di avviare A/B/C:

- Contratto39 identico fra modalità lungo probe, EnvRunner, learner e
  rollout: nomi, ordine, unità, bounds, trasformazioni e prefisso actor.
  Le quattro feature hanno nomi espliciti `prescribed_left_*`; non riusare
  nomi storici che le descrivano come controlaterali.
- A stato osservabile fisso e maschera chiusa, sostituire le quattro copie
  con teacher corretto, sfasato e casuale: osservazioni mascherate, medie e
  logstd identiche; derivata degli output rispetto ai target nascosti nulla.
  Ripetere dopo update e reload in processo nuovo.
- Con maschera aperta, verificare un controllo positivo: i target possono
  influenzare l'actor e il gradiente passa nel ramo visibile. Un ramo
  disconnesso non dimostra correttezza della maschera.
- Verificare separatamente che la reward imitativa reagisca al teacher
  originale anche con actor mascherato. Maschera e curriculum non cambiano
  il prescribed biologico, gli angoli reali o il riferimento servito.
- Clonare policy, critic, optimizer, batch e casualità di un update ex-novo;
  variare soltanto copie del teacher e loro trasformazioni eventuali.
  Confrontare osservazioni utilizzate, reward, value, bootstrap, advantage,
  loss, gradienti e pesi aggiornati. Devono coincidere alla tolleranza
  dichiarata. Il test deve attraversare il percorso di preprocessing
  effettivo, non una funzione fittizia scollegata dal trainer.
- Controllare critic, normalizzazione e loss ausiliarie. I campi esistenti
  `pros_*_target` e `*_tracking_error` possono derivare dal served `self.kin`
  anziché dal teacher `base_kin`: identificarne la sorgente prima di
  classificarli come copie del teacher, senza rimozioni automatiche.
- Riutilizzare lo stesso batch mascherato in più epoche PPO. A pesi fissi,
  il likelihood ratio deve essere unitario; disponibilità e log-probabilità
  di raccolta non devono cambiare a ogni forward.
- Save/reload deve conservare contratto, normalizzazione, livello globale
  curriculum, RNG dedicato alla maschera e stato della fase. Verificare in
  particolare la ripresa 40→41 e il divieto di aprire la maschera ex-novo.
- Eseguire uno smoke reale delle due modalità e del trasferimento prima
  della campagna; conteggi delle transizioni e parità del bilanciamento
  devono corrispondere al budget dichiarato per lo smoke.

Sulla stessa piattaforma CPU richiedere uguaglianza esatta dei vettori
mascherati e degli stati per i quali il percorso numerico è identico.
Per confronti float32 non bitwise usare `atol=1e-6`, `rtol=1e-6`, riportando
la differenza massima; non finiti sono sempre failure. Per il rapporto PPO
usare la stessa tolleranza rispetto a 1. La tolleranza non autorizza a
ignorare un percorso residuo del teacher: il gate richiede anche verifica
strutturale della maschera. Ogni eccezione deve essere spiegata e risolta
prima del confronto, non mediata sui campioni.

## 6. Raccolta e valutazione A/B/C

Gli start nominali sono `1.756870983805102`, `1.956870983805102` e
`2.156870983805102` s. Modello, IK, GRF, detector, FSM actor v3, reset,
frequenza policy, slew limiter, filtro C2, SEA e protezioni sono comuni ai
tre bracci. Prima del primo update confrontare i digest dei pesi iniziali;
avere lo stesso seed senza verificarli non dimostra inizializzazione abbinata.

Salvare tutti i checkpoint e selezionare come risultato principale il
checkpoint50 di ogni braccio, senza sostituirlo con il best di training.
La reward aggregata del trainer è secondaria e non seleziona il vincitore.
Le valutazioni di confine usano checkpoint10/11, 20/21, 30/31 e 40/41:
la coppia mostra il risultato prima e dopo il primo aggiornamento con la
disponibilità ridotta. Questi checkpoint sono fissi e vanno valutati anche
se non sembrano promettenti.

Valutare A, B e C con maschera chiusa ai tre start, deterministico,
500 step/5 s, per i checkpoint di confine e per il50. Valutare A anche
visibile per distinguere la capacità assistita da quella autonoma.
Registrare la disponibilità effettiva nei batch ai cambi curriculum: una
probabilità nominale non equivale alla quota osservata di transizioni visibili.

Al checkpoint50 affiancare prove stocastiche con sigma0.010 e seed rumore
20260914, 20260915 e 20260916. Usare copie per la valutazione e conservare
intatti i checkpoint di training. Registrare o preregistrare la sequenza
di perturbazioni effettivamente applicate e accoppiarla fra i bracci;
controllare sigma e clipping, senza assumere che lo stesso seed basti.

Gli offset `1.856870983805102` e `2.056870983805102` s sono riservati alla
valutazione finale del checkpoint50. Verificarne prima solo la validità nel
dominio IK; non usarli per selezione, correzioni o curriculum. Eseguire
deterministico e le stesse tre sequenze stocastiche, con maschera chiusa.
Questi start misurano robustezza nella stessa condizione AB06, non
generalizzazione ad altri soggetti.

## 7. Metriche fisiche e criteri esplorativi

Calcolare per start, giunto e seed, separando episodio completo, finestra
comune fra candidati, avvio e regime:

- durata, completamento, motivo di terminazione, HS/TO validi e invalidi,
  cicli completi, tempo al primo periodo online valido;
- RMSE e MAE di q/qdot actual–teacher, served–teacher e actual–served,
  ampiezze e picchi; q in gradi e qdot in gradi/s nel report, con unità raw
  e conversioni documentate;
- sfasamento come diagnostica aggiuntiva, senza traslazioni temporali
  ottimizzate nelle metriche principali;
- penetrazione massima e tempo sopra 20/28 mm, saturazioni SEA,
  clipping delle azioni e regolarità dei comandi;
- coppie/potenza e supporto residuale se disponibili negli output, con
  quantità mancanti marcate esplicitamente e nessuno zero sostitutivo;
- campioni e cicli valutabili, campioni esclusi, transizioni, aggiornamenti
  e costo reale delle singole fasi.

Le metriche di tracking sono allineate allo stesso timestamp e ponderate
per la durata del passo. Mostrare la cinematica nel tempo reale; la forma
normalizzata sul ciclo è aggiuntiva e disponibile soltanto con cicli validi.
Un episodio corto non diventa migliore grazie a medie inferiori; nessun
ciclo significa assenza di evidenza del cammino, non errore di ciclo nullo.

Il gate operativo di **cammino autonomo nel pilot** richiede, per tutti i
tre start nominali deterministici a maschera chiusa: 500 step/5 s,
almeno un ciclo valido, nessuna terminazione di protezione e rispetto delle
protezioni fisiche esistenti. Non si modificano le soglie per ottenere il
passaggio. Questo gate è un criterio sperimentale minimo, non parità con
la baseline o certificazione biomeccanica.

Per il confronto C/B riportare sempre le differenze per start e giunto.
La lettura è ordinata: completamento e cicli, protezioni e saturazioni,
poi tracking e contatto sulla finestra comune. Un miglioramento del tracking
con regressione di completamento o violazione fisica non è un vantaggio
autonomo. Un segnale da replicare richiede che C mascherata superi il gate,
non perda completamento o cicli rispetto a B in alcuno start nominale,
non introduca nuove violazioni, e migliori almeno un indicatore fisico
principale. Esplicitare ogni compromesso residuo: non esiste qui una
soglia di non inferiorità per tracking o carichi validata dal progetto.

Classificazione preregistrata:

| Risultato | Interpretazione consentita |
|---|---|
| Gate software fallito | Implementazione non valida; correggere e ripetere i controlli prima di interpretare A/B/C |
| A visibile funziona, A/B/C mascherate no | Dipendenza dall'informazione disponibile; autonomia non dimostrata nel budget |
| C mascherata mostra un vantaggio rispetto a B | Segnale esplorativo da verificare con repliche e trasferimento |
| B e C funzionano, senza vantaggio chiaro di C | Fattibilità del controllo mascherato; beneficio del curriculum non dimostrato |
| Nessun braccio apprende nel budget | Inconclusivo sulla fattibilità informativa; esaminare anche reward, controllo e durata |
| Regime assente o dati insufficienti | Metrica non valutabile; nessun giudizio favorevole basato sui campioni mancanti |

## 8. Trasferimento ex-novo e repliche

Trasferire direttamente gli actor B50 e C50 a due run ex-novo nuovi,
con lo stesso budget di 11 iterazioni: una critic-only e dieci PPO.
La maschera resta sempre chiusa. Il critic e l'optimizer sono nuovi;
la configurazione sigma0.010 deve conservare la rete delle medie.
Verificare digest actor/logstd invariati nel warm-up e critic aggiornato,
poi aggiornamenti delle medie durante PPO.

Effettuare il warm-up in un primo processo fino a iterazione1 e un resume
completo in processo nuovo fino a11, uguale per B e C. Questo verifica che
il reload non riapra i target e non ripeta il warm-up. La discontinuità
degli episodi dovuta al processo nuovo va registrata come negli altri blocchi.
Valutare gli actor prima del trasferimento e dopo iterazione11 con lo stesso
protocollo finale mascherato; rendicontare separatamente il costo ex-novo.

I test software del trasferimento vanno completati anche se C non raggiunge
il gate autonomo. In quel caso il confronto di training rimane esplorativo:
non si descrive come trasferimento di una policy già qualificata.
Un errore software bloccante deve essere risolto prima di interpretare
gli aggiornamenti; il fallimento del cammino non autorizza invece a saltare
la verifica tecnica del passaggio di modalità.

Se il pilot suggerisce un vantaggio autonomo e trasferibile di C, ripetere
**prima di sostenere una conclusione qualitativa favorevole** l'intera
campagna A/B/C e il trasferimento B/C con seed456 e789, mantenendo budget,
segmentazione, curriculum e valutazioni. Riportare tutte le tre repliche,
dispersione e differenze abbinate, senza selezionare soltanto il seed migliore.
Se il pilot non mostra tale segnale, si può concludere negativamente o in
modo inconclusivo entro il budget assegnato senza presentare il singolo seed
come prova statistica. Tre seed non garantiscono da soli significatività:
l'incertezza residua deve restare esplicita.

## 9. Artefatti e chiusura

La fase 5 deve produrre manifest e configurazioni, dataset allineato,
risultati dei gate software e dello smoke, checkpoint e audit dei resume,
metriche/tracce/grafici A/B/C, confronto ex-novo B/C e un report con esiti,
limiti, costi e prove non valutabili. Ogni eventuale deviazione da questo
protocollo va datata e motivata prima dei risultati cui si applica; non
retrodatare una scelta sperimentale dopo avere visto i candidati.

Il benchmark di parità storica richiede ancora riferimento baseline ai
tre start, tolleranze di regressione motivate e confronto dell'intera
storia di apprendimento. Non viene dichiarato completato da questi test.
Il comportamento live Windows richiede un'esecuzione Windows effettiva:
path e codice portabili non costituiscono quella prova. Predittività,
compensazione del ritardo e nuovi teacher di comandi restano fuori scope.

## 10. Verifiche della redazione

Questo task introduce soltanto il presente file in `reports/plans/`.
La verifica documentale riguarda UTF-8, esistenza dei collegamenti locali e
coerenza aritmetica dei budget. Non costituisce esecuzione dei test descritti,
creazione della variante39 o avvio di nuovi esperimenti.

## 11. Precisazioni esecutive prima dei risultati della fase 5

Il coordinatore `unified_MLP/validation/run_authorized_plan.py` attende il
training a 50, esegue la valutazione, riprende fino a 100 e ne esegue la
valutazione. Solo dopo chiama `run_feasibility39.py`, che materializza il
bundle preparatorio nella directory separata `unified_MLP_39`. La preparazione
e i test del bundle in directory temporanee non avviano la campagna fisica.

- Lo smoke fisico usa 3 runner, batch192, minibatch64 e una sola epoca:
  1 iterazione imitativa e 3 ex-novo (warm-up1, PPO1 e ulteriore PPO1 dopo
  resume). Sono 768 transizioni di training, escluse sei valutazioni
  deterministiche ai tre start. Questi costi sono separati da A/B/C.
- I checkpoint11/21/31/41 sono valutati dagli artefatti trattenuti dopo il
  completamento del rispettivo blocco successivo. Le loro iterazioni e
  disponibilità restano quelle preregistrate; non vengono aggiunti update.
- I digest dei pesi salvati prima del primo update vengono confrontati
  appena sono disponibili i primi blocchi A/B/C, prima di procedere oltre10.
  Un mismatch invalida l'inizializzazione abbinata e arresta la campagna.
- Le prove stocastiche usano `stochastic_held` con hold1, un RNG gaussiano
  dedicato e copie dei moduli con sigma0.010. I checkpoint originali restano
  intatti. Il rumore standard ricostruito dalla traccia viene confrontato
  fra i bracci, con scarto massimo1e-6 sul prefisso comune.
- Il trigger automatico delle repliche è volutamente più restrittivo del
  segnale esplorativo generale: C supera il gate autonomo, non perde durata
  o cicli rispetto a B e migliora tutte le RMSE di posizione actual/teacher
  e served/teacher per entrambi i giunti e tutti i tre start nominali.
  Il criterio deve risultare soddisfatto sia dopo imitazione sia dopo
  trasferimento ex-novo. Fuori da questo caso, vengono comunque riportati
  risultati e compromessi del pilot, senza una conclusione favorevole generale.
- La disponibilità effettiva viene contata per transizione e worker; la
  copertura deve coincidere con il batch. In B, nelle ultime dieci iterazioni
  di C e nell'ex-novo non sono ammesse transizioni con target visibili.
- Gli errori tecnici arrestano i passaggi dipendenti. Un fallimento del
  cammino è invece un risultato dell'esperimento e non elimina una prova.

Verifiche preparatorie del 14 settembre: suite completa165 test superati;
ulteriori18 test dell'orchestrazione superati dopo il monitoraggio periodico.
Queste prove non attestano ancora il runtime fisico della variante39 né
il completamento della V1 a50/100.
