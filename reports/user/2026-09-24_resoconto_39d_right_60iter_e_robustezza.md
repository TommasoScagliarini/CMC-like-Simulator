# Resoconto RIGHT39 alla 60: imitazione, coppie e robustezza

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Report del 24 settembre 2026, redatto su richiesta `create_report`.
Il training è terminato il 23 settembre alle 22:26 CEST; i due rollout
deterministici e i cinque rollout con rumore sono conclusi. Questo documento
consolida il [resoconto dell’analisi](2026-09-23_risultati_39d_right_60iter.md),
che rimane preservato insieme alla sua verifica di integrità.

## Problema e strategia

Verificare se la 39D allenata sul riferimento destro, proseguita dalla 40 alla
60, si avvicina alla qualità e alla robustezza del training imitativo V26 del
20 agosto. Il limite concordato resta l’iterazione 60; il lavoro successivo
ha riguardato esclusivamente la valutazione.

La strategia comprende audit del resume, confronto degli storici a esperienza
comparabile, rollout deterministici best58/last60 con i sette grafici canonici
per ciascuno, cinque seed con rumore sul best58 e diagnosi temporale del
fallimento seed125. Completamento, cinematica, FSM e coppie sono valutati
separatamente. La diagnosi è avanzata, ma non è stata implementata una
correzione della fragilità residua.

## Esito principale

La continuazione dalla 40 alla 60 è stata utile: migliorano imitazione e
regolarità del rollout nominale, con un elevato completamento degli episodi
durante il training. La qualità V26 non è però pienamente recuperata e
ricompare una coppia positiva rilevante alla caviglia nell'ultimo appoggio.

Il **best selezionato dal training è la 58**, con ritorno 251,974. Nella
valutazione deterministica, però, **la last60 è migliore della best58** sulle
metriche cinematiche aggregate, sul ritorno e sul picco positivo della coppia
alla caviglia. Il criterio di selezione del training non è una valutazione
separata del checkpoint: gli episodi possono attraversare aggiornamenti dei
pesi e l'export viene salvato dopo l'update PPO.

Il training si è fermato esattamente alla 60, senza crash, iterazioni saltate
o prosecuzioni automatiche. Le 20 iterazioni aggiuntive hanno richiesto circa
2 h 39 min; totale 245.760 transizioni.

## Training: confronto per esperienza raccolta

Le percentuali usano differenze dei contatori di fine episodio, comprese le
riprese. Le medie di ritorno e durata sono pesate per episodio e ricavate
dalle somme dei callback, verificate con TensorBoard. Non vengono sostituite
con le proiezioni mostrate dalla console quando non termina alcun episodio.

Il confronto V26 a esperienza simile è alla sua iterazione 53, con 245.132
transizioni; RIGHT e LEFT sono alla 60, con 245.760 transizioni ciascuna.

| Finestra di training | RIGHT39 | LEFT39 | V26 |
|---|---:|---:|---:|
| Completi nelle ultime 5 iterazioni | 37/40 — 92,5% | 25/44 — 56,8% | 40/41 — 97,6% |
| Completi nelle ultime 10 | 75/79 — 94,9% | 54/90 — 60,0% | 80/83 — 96,4% |
| Completi nelle ultime 20 | 153/157 — 97,5% | 100/174 — 57,5% | 158/167 — 94,6% |
| Ritorno medio per episodio, ultime 5 | 234,66 | 219,97 | 233,85 |
| Durata media per episodio, ultime 5 | 4,812 s | 4,395 s | 4,967 s |

Nella continuazione RIGHT 41–60, i quattro fallimenti sono penetrazioni
ordinarie alle iterazioni 54, 56, 57 e 59. Nessun timeout di fase o fallimento
per penetrazione hard in questa tranche di training. La finestra 41–50 ha
78/78 episodi completi; 51–60 ne ha 75/79. Il comportamento non migliora quindi
monotonicamente e il 97,5% non va interpretato come una probabilità generale
di successo del checkpoint finale.

Rispetto alle ultime cinque iterazioni della 40, il completamento passa da
90,7% a 92,5% e il ritorno medio per episodio da 203,92 a 234,66. Le loss
cinematiche diminuiscono; l'entropia media scende da 1,877 a 1,654. Non sono
emerse instabilità numeriche di PPO. L'entropia non equivale alla deviazione
standard di un singolo giunto su una traiettoria fissata.

Le iterazioni 41, 43, 46, 49 e 51 non hanno un ritorno per episodio concluso:
i worker ripartiti insieme non hanno ancora terminato gli episodi. Non si
tratta di campioni persi. Gli 81.920 nuovi passi risultano tutti osservati,
con i sei ingressi prescribed/clock visibili al 100%.

## Rollout deterministici e grafici canonici

Due nuovi rollout dai milestone immutabili 58 e 60, ciascuno di 500 passi
allo start fisico 12,99 s. Riferimento destro, clock, configurazione e FSM v3
restano quelli del training. V26 best87 è qui il riferimento storico di
qualità, più maturo; questo confronto non sostituisce quello a pari esperienza
degli storici.

| Misura | RIGHT40 | RIGHT best58 | RIGHT last60 | V26 best87 |
|---|---:|---:|---:|---:|
| Passi completati | 500 | 500 | 500 | 500 |
| Ritorno deterministico | 286,32 | 278,72 | 324,15 | 363,79 |
| RMSE ginocchio vs target | 9,386° | 7,223° | 7,066° | 6,906° |
| RMSE caviglia vs target | 2,757° | 3,118° | 2,414° | 1,459° |
| Cicli FSM validi | 1 | 2 | 2 | 2 |
| Eventi FSM invalidi | 3 | 0 | 0 | 0 |
| Picco positivo SEA caviglia | 3,30 Nm | 31,85 Nm | 25,89 Nm | 10,19 Nm |
| Chattering knee, RMS >20 Hz dopo 0,2 s | 3,097 Nm | 3,226 Nm | 3,266 Nm | 3,280 Nm |
| Picco penetrazione STO dopo 0,2 s | 14,53 mm | 17,69 mm | 14,10 mm | 11,17 mm |
| Picco forza normale dopo 0,2 s | 348,7 N | 583,1 N | 451,0 N | 362,3 N |

La RMSE del ginocchio della 60 è vicina a V26, ma **la forma non è equivalente**:
la flessione massima è 51,44°, contro 64,32° del target e 63,78° effettivi V26.
Nella fascia di fase 60–100% la RMSE resta 8,755° contro 4,886° V26. Un numero
aggregato simile può nascondere errori distribuiti diversamente lungo il ciclo.

Il controllore segue bene la reference servita: RMSE giunto–reference alla
60 di 0,180°/0,277° per ginocchio/caviglia, praticamente uguale a V26
(0,180°/0,278°). Il divario si trova soprattutto nella traiettoria fornita
dal blocco policy/governor, senza separare causalmente i due elementi.

Il riallineamento temporale descrittivo ottimale alla 60 è 20 ms al ginocchio
e 30 ms alla caviglia. Nella finestra comune elimina circa il 7,7% e il 16,5%
dell'errore quadratico: non spiega tutto il divario, né identifica il ritardo
causale introdotto dal filtro.

Grafici salvati nella cartella root `plot/`:

- [Set canonico best58: sette PNG](../../plot/09_23_2026_39D_right_best58/)
- [Set canonico last60: sette PNG](../../plot/09_23_2026_39D_right_last60/)
- [Confronto cinematico e coppie con V26](../../plot/09_23_2026_right60_comparison/right60_vs_v26_deterministic.png)
- [Confronto con i checkpoint 35 e 40](../../plot/09_23_2026_right60_comparison/right60_vs_right40_deterministic.png)
- [Curve di training per esperienza](../../plot/09_23_2026_right60_comparison/training_by_experience.png)

## Coppie: il miglioramento della reward lascia un compromesso

I picchi positivi SEA caviglia compaiono nell'ultimo appoggio, a 4,655 s dal
reset per best58 e 4,545 s per last60: non sono transitori iniziali. Nel
campione reward vicino al picco della 60, la coppia è +25,57 Nm e il comando
richiesto +25,55 Nm, mentre il momento GRF sull'asse ankle è −24,98 Nm. Per
best58 i valori sono rispettivamente +31,49, +31,43 e −30,66 Nm.

È compatibile con una coppia che compensa il momento esterno, non con un
mancato inseguimento del SEA. Non identifica ancora quale differenza di
postura, appoggio e centro di pressione generi quel momento. Il momento GRF
qui usato è riportato al giunto e proiettato sul suo asse, distinto dal momento
rispetto all'origine del sistema ground registrato nello STO.

La reward rileva il problema: la penalità cumulativa anti-flip è 6,04 alla
60 e 19,13 alla 58, contro zero alla 40. Non impone però un vincolo assoluto.
Da 40 a 60, il ritorno aumenta di 37,83: +29,28 di contributo imitativo,
14,53 di penalità base risparmiate e circa 5,97 di penalità aggiuntive in più,
con la piccola correzione del clipping base. Il miglioramento dell'obiettivo
complessivo può quindi convivere con coppie meno desiderabili.

Questo è un compromesso osservato con reward invariata, non una prova che
un particolare peso sia errato o una diagnosi causale sufficiente per
modificarlo. Il chattering del ginocchio è invece ora vicino al livello V26
secondo la misura RMS sopra 20 Hz utilizzata per tutti i rollout.

## Robustezza a pesi fissi: il problema non è risolto

Cinque seed gaussiani 123–127 sul best58, rumore originale, seed ambiente 123
e start fisso. Ogni policy conserva la propria deviazione standard appresa;
le estrazioni gaussiane standard sono verificate identiche e confrontate con
best35, last40 e V26 best87 valutata nel contenitore attuale con FSM v3.

| Policy a pesi fissi | Episodi completi | Almeno due cicli validi | Ritorno medio | RMSE knee / ankle sul prefisso comune |
|---|---:|---:|---:|---:|
| RIGHT best35 | 5/5 | 5/5 | 210,49 | 10,99° / 4,55° |
| RIGHT last40 | 4/5 | 3/5 | 204,18 | 9,78° / 4,61° |
| RIGHT best58 | 4/5 | 3/5 | 208,98 | 8,82° / 3,65° |
| V26 best87, replay attuale | 5/5 | 3/5 | 259,06 | 6,36° / 2,88° |

Le metriche fisiche sono ricalcolate per tutti sui primi **299 passi, 2,99 s**,
escludendo la riga terminale hard che non rappresenta un nuovo passo completo.
Non vanno mescolate con le vecchie medie calcolate su 301 passi. Ritorni e
contatori di fine episodio riguardano invece l'intero episodio osservato.

Best58 termina al passo 300 sul seed125 per penetrazione hard, senza eventi
FSM invalidi o resync; last40 falliva sullo stesso seed al passo 301 per
penetrazione ordinaria. Best35 e V26 completano questo seed. Gli altri quattro
seed del best58 completano tutti 500 passi. La migliorata precisione cinematica
non ha quindi eliminato il caso critico. Cinque prove non dimostrano che una
policy abbia in generale un tasso di successo dell'80% o del 100%, né provano
equivalenza o regressione statistica.

Nel confronto del seed125 a t=15,98 s, la penetrazione registrata a 100 Hz è
15,75/24,91/27,93/22,93 mm rispettivamente per best35/last40/best58/V26.
Best58 raggiunge anche 29,95 mm nello STO a 1 kHz prima dell'interruzione:
il picco del sottopasso che attiva il guard hard da 30 mm non è necessariamente
registrato nei campioni salvati.

Il riferimento di caviglia best58 passa da 12,86° a 21,92° fra 2,90 e 2,99 s,
mentre il target passa da 12,91° a 15,11°. Al termine, la velocità effettiva
è 145,95°/s contro 60,87°/s nel V26; l'errore actual–served è −4,28° contro −1,52°.
Lo scostamento nasce già nella reference servita e l'inseguimento si deteriora
nel transitorio. La RMSE actual–target nell'ultimo decimo di secondo può
restare bassa perché questi errori di segno opposto si compensano: non è
una misura sufficiente del margine rispetto alla penetrazione.

V26 attraversa la zona oltre 20 mm quasi allo stesso istante ma recupera.
La sigma media finale della caviglia è persino inferiore nella 58 rispetto
a V26; nell’ultimo segmento completato comune, lo step 299, entrambe le
azioni della caviglia sono clippate a −1. Il comando del segmento 300
interrotto è distinto e non è quello usato in questo confronto.
Il solo ultimo rumore o una FSM irregolare non spiegano quindi il fallimento.
I dati localizzano una fragilità del transitorio di contatto e della storia
dei comandi, senza isolare una singola causa meccanica.

- [Confronto delle penetrazioni sui cinque seed](../../plot/09_23_2026_right60_comparison/noise_paired_penetration.png)
- [Metriche complete dello screening — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-e62fc8b7eac3ccdc>)
- [Diagnosi temporale del seed125 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-6ddffe966fc83eb5>)

## Verifiche, file e limiti

Nessuna modifica a produzione, plugin SEA, FSM o configurazione di training.
Script, metriche, log, ricevute e hash sono nella nuova cartella
[right_reference_results60_2026-09-23 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-e4e190e82b08736b>).

Audit del training superati: prefisso 1–40 immutato, 145 input congelati
verificati, checkpoint 40/60 e archivio preservati, actor/critic/optimizer/PPO
ripristinati correttamente. I contatori Adam crescono 3240→3321→4860, senza
reset silenzioso. Target q/qdot dei rollout RIGHT coincidono con V26 sui
500 campioni; clock coerente, sei ingressi visibili, divisore velocità 1.
Reward ricostruita entro 4×10⁻¹³, incluso clipping base verificato per passo.
Tutti i rollout hanno concluso correttamente il processo software, incluso
quello terminato per penetrazione. Gli input congelati risultano immutati;
lo screening verifica rumore, clipping, reward e target sulle tracce salvate.

Il confronto usa un solo seed di training e uno start fisico di evaluation.
Le differenze LEFT/RIGHT comprendono waveform, clock, velocità e range di
normalizzazione; le riprese storiche LEFT hanno inoltre anomalie documentate.
La FSM e la gestione dei guasti V26 storici differiscono; alcuni guasti
fatali dei worker possono mancare dai contatori. Nessuna equivalenza
statistica o singola causa radice è dimostrata da questi dati.

## Indicazione operativa

Questi risultati sono un'evidenza contro un'incapacità generale della 39D
di apprendere l'imitazione. Il riferimento destro ha prodotto un percorso di
apprendimento molto più efficace della prova sinistra, senza isolare il solo
riferimento come unica causa.

La last60 è un candidato migliore della best58 nel deterministico, ma va
valutata sugli stessi seed prima di sceglierla per l'oscuramento: i cinque
test con rumore appena conclusi riguardano la 58, non la 60. La best35 resta
un controllo utile per la robustezza. Per le coppie, il confronto da
approfondire è la sequenza reference–postura–contatto/COP–momento GRF–coppia
SEA nell'ultimo appoggio. Nessuna prosecuzione oltre la 60 o modifica della
reward è stata avviata.


## File prodotti e verifiche eseguite

Il codice di produzione, la FSM e i checkpoint non sono stati modificati.
La cartella delle verifiche è
`Trajectory Generator/unified_MLP_39/validation/right_reference_results60_2026-09-23/`.

| Artefatto | Contenuto |
|---|---|
| `training_history/analyze.py`, `metrics.json`, `findings.md` | Storici, medie per episodio e audit del resume |
| `run_evaluations.py`, `rollout_launch.json`, `rollout_execution.json` | Due rollout deterministici e relativi comandi/esiti |
| `analyze_rollouts.py`, `rollout_metrics.json`, `fsm_metrics.json` | Cinematica, coppie, FSM, reward e confronto con gli archivi |
| `noise_screening/run.py`, `analyze.py`, `metrics.json`, `verification.json` | Cinque seed best58 e confronto con quindici episodi archiviati |
| `failure_seed125/analyze.py`, `metrics.json`, `findings.md` | Confronto temporale del transitorio critico |
| `final_verification.json` | Verifica conclusiva degli artefatti dell’analisi e dei collegamenti |
| `plot/09_23_2026_39D_right_best58/` e `plot/09_23_2026_39D_right_last60/` | Quattordici grafici canonici nella root |
| `plot/09_23_2026_right60_comparison/` | Figure di confronto training, cinematica, coppie e rumore |

La verifica conclusiva ha controllato **561 voci di manifest** (con possibili
ripetizioni fra manifest), i quattordici PNG canonici e i collegamenti del
resoconto originario. I sette rollout sono terminati senza errori tecnici;
la terminazione fisica hard del seed125 resta un fallimento della policy e
non viene trasformata in un successo. In questa richiesta `create_report`
sono stati prodotti soltanto documenti, senza nuovi training o simulazioni.

## TODO e decisioni ancora aperte

- [ ] Valutare last60 sugli stessi cinque seed prima di scegliere il checkpoint
  da usare per l’oscuramento; questi test non sono ancora stati eseguiti.
- [ ] Approfondire il transitorio seed125, confrontando reference, velocità,
  contatto e margine di penetrazione su best35, last40, best58, last60 e V26.
- [ ] Isolare l’origine del momento GRF invertito nell’ultimo appoggio prima
  di proporre una correzione della reference, del governor o della reward.
- [ ] Definire criteri distinti per completamento, cicli validi, forma delle
  traiettorie e coppie prima della scelta del checkpoint e dell’oscuramento.
- [ ] Dopo questo screening, pianificare repliche e condizioni di valutazione
  sufficienti a verificare comparabilità statistica e generalizzazione.

Questi punti sono un elenco di lavoro futuro: nessuna esecuzione aggiuntiva
è stata avviata. Sono riportati anche nel
[daily del 24 settembre](../daily/2026-09-24_daily-report.md).
