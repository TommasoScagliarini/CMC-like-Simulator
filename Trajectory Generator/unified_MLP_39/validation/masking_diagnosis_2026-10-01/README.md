# Diagnosi del curriculum di oscuramento CP90 → CP190

Data: 2026-10-01. Instruction check token: `CMC_AGENT_OK_2026`.

## Domanda e perimetro

Perché CP112 e CP184 completamente oscurati perdono qualità imitativa rispetto
alla baseline CP90 visibile? Analisi del training archiviato, confronto delle
tracce e tre controlli fisici a pesi congelati. Nessun aggiornamento PPO, cambio
di reward, controller, plugin o configurazione di produzione. CP90 resta la
baseline. Il bug bootstrap già diagnosticato rimane fuori da questa attività.

## Risultato dei controlli a pesi congelati

Stesso reset, start, seed, fisica, reward e durata richiesta. I tre controlli
nuovi sono CP90 hidden, CP112 visible e CP184 visible; gli altri sono gli
archivi originali. Hash dei checkpoint verificati invariati.

| Checkpoint | RMSE knee/ankle visibile [°] | RMSE knee/ankle oscurato [°] |
|---|---:|---:|
| CP90 | 6,64 / 3,94 | 13,28 / 9,55 |
| CP112 | 7,03 / 4,27 | 13,47 / 10,63 |
| CP184 | arresto a 0,98 s | 12,43 / 8,89 |

Le coppie di RMSE coprono 13,19–17,94 s. Tutti i casi salvo CP184 visible
completano 5 s. Oscurare CP90 è già sufficiente a perdere gran parte della
qualità; CP112 visibile è ancora vicino alla baseline. CP184 hidden recupera
qualcosa rispetto a CP90 hidden, ma il trasferimento rimane insufficiente.

Riaprire i segnali a CP184 migliora il suo prefisso rispetto a hidden, senza
ripristinare CP90. Sullo stesso prefisso di 98 passi: RMSE knee/ankle
6,08/2,89° per CP90 visible, 9,87/5,16° per CP184 visible, 14,70/12,40° per
CP184 hidden. Quest'ultimo confronto usa anche il transitorio iniziale e non
è la finestra della tabella principale.

CP184 visible termina per `joint_divergence:pros_knee_angle`: actual
`+0.0004203 rad`, circa 0,024° oltre il limite zero. Negli ultimi dodici passi
il comando knee è saturato verso endpoint zero; il riferimento served arriva
a `+0.002955 rad` mentre il target è circa `-0.196779 rad`. È una fragilità
articolare reale, non un'esplosione numerica. I comandi sono deterministici.

La diagnosi sostenuta dai dati è quindi: **dipendenza iniziale dal teacher,
trasferimento incompleto al controllo oscurato e successiva erosione della
competenza visibile**. Non una scomparsa progressiva uniforme di ogni capacità.
I controlli non separano tutte le cause del cambiamento dei pesi.

Ricevute e comandi: [counterfactual_results.json](counterfactual_results.json).
Set di sette plot per ogni controllo sotto root `plot/`, con prefisso
`10_01_2026_masking_diagnosis_cp*`; il caso di 0,98 s non contiene cicli
completi, dichiarati non disponibili nelle figure per ciclo.

## Evidenza dal training: il ramo oscurato non acquisisce la qualità del visibile

RMSE fisico contro target, pooled sui campioni dei segmenti completati degli
episodi conclusi. Ordine knee / ankle, gradi:

| Probabilità episodica di oscuramento | RMSE episodi visibili | RMSE episodi oscurati | Transizioni realmente oscurate |
|---|---:|---:|---:|
| 25% | 7,83 / 4,29 | 14,27 / 9,16 | 22,28% |
| 50% | 7,47 / 5,16 | 14,38 / 9,40 | 50,86% |
| 75% | 8,50 / 5,29 | 14,13 / 9,61 | 79,52% |
| 100% | — | 14,22 / 9,23 | 100% |

Il calo della media complessiva mescola due effetti: cresce la quota di episodi
del ramo già peggiore, e cambiano i pesi della policy. La qualità hidden resta
quasi piatta; non descrive da sola una perdita graduale di una capacità hidden
precedentemente buona. Il ritorno per passo hidden cresce modestamente:
0,284 → 0,289 → 0,304 → 0,329, senza recupero comparabile degli angoli.

Le metriche sono relative alla popolazione di episodi salvati, non valutazioni
di un singolo checkpoint: gli episodi attraversano spesso aggiornamenti dei
pesi. Quelli ancora aperti ai riavvii dei blocchi non hanno un record conclusivo.
Le quote di maschera derivano invece da tutte le 409.600 transizioni raccolte.

Script, CSV, denominatori e limiti: [training/](training/README.md).
Interpretazione e sigma: [training/findings.md](training/findings.md).

## Quale anello della catena sbaglia

Confronto nominale su tempi identici 13,19–17,94 s, esclusi i primi 200 ms di
inizializzazione. Coppie fisiche campionate a 1000 Hz; cinematica delle tracce
a 100 Hz. Target registrati verificati uguali ai medesimi tempi.

| Misura | CP90 visibile | CP112 oscurato | CP184 oscurato |
|---|---:|---:|---:|
| RMSE actual–target knee [°] | 6,64 | 13,47 | 12,43 |
| RMSE actual–target ankle [°] | 3,94 | 10,63 | 8,89 |
| RMSE actual–served knee [°] | 0,18 | 0,20 | 0,22 |
| Escursione knee [°] | 41,71 | 19,04 | 21,36 |
| Escursione ankle [°] | 30,67 | 8,98 | 17,62 |
| Frazione temporale coppia ankle positiva | 57,3% | 50,0% | 43,1% |
| Area positiva coppia ankle [Nm·s] | 2,88 | 8,41 | 3,42 |
| Componente knee sopra 10 Hz [Nm RMS] | 4,69 | 3,72 | 3,73 |
| Componente knee sopra 20 Hz [Nm RMS] | 3,62 | 3,37 | 3,50 |

Il controller segue una reference generata già lontana dal target. Non emerge
una perdita primaria di tracking low-level. Il degrado include escursione e
forma, oltre alla fase: su supporto fisso 13,50–17,49 s, un riallineamento
temporale globale ottimo lascia ancora circa 10,45° di RMSE knee per CP184.

La lettura delle coppie dipende dalla misura: la porzione positiva ankle si
accorcia ma l'area non scende sotto CP90. La banda knee sopra 10 Hz diminuisce,
quella sopra 20 Hz cambia poco; il rumore di coppia motore sopra 20 Hz aumenta
da circa 13,24 a 15,32 Nm RMS. Non si può equiparare una curva apparentemente
più regolare a un miglioramento generale di controllo. Le escursioni si sono
ridotte e il contatto è cambiato.

Neppure gli endpoint della policy sono diventati più lisci: la media del
salto assoluto tra endpoint successivi knee è 0,428 / 0,459 / 0,564 rad e
ankle 0,558 / 0,829 / 0,753 rad. La frazione media di intervento del limite
di jerk cresce 0,579 → 0,678 → 0,760. I profili fisici più piatti sono il
risultato della catena filtrata; non attestano una policy intrinsecamente
meno oscillante.

Analisi riproducibile, metriche e reward: [rollouts/](rollouts/README.md).
Figure: [confronto cinematico](../../../../plot/10_01_2026_masking_diagnosis/rollout_kinematics_original_three.png).

## Contratto informativo e curriculum

La maschera azzera per l'intero episodio sei ingressi: clock prescribed
sin/cos e q/qdot target knee/ankle. La progressione aumenta la probabilità di
estrarre un episodio completamente oscurato; non rende i segnali gradualmente
più deboli. Il target della reward continua a seguire il clock prescribed.

L'actor resta un MLP feedforward 2×256 tanh con q/qdot protesiche, motori,
contatto e fase online LEFT, timer FSM e stato del controller. Questi ultimi
forniscono memoria indiretta; non c'è uno stato ricorrente appreso o tempo
episodico esplicito. Il critic perde i sei ingressi ma conserva cinematiche
biologiche/controlaterali e fase online RIGHT, potenziali proxy temporali.
La coppia sin/cos zero identifica implicitamente gli episodi oscurati.

Trasferire l'imitazione richiede quindi anche ricostruire la sincronizzazione
del riferimento. È un meccanismo plausibile per la difficoltà osservata,
non una dimostrazione di impossibilità con gli ingressi rimasti. Nessun test
di osservabilità certifica qui che quegli ingressi siano insufficienti.

Il curriculum avanza ogni 25 aggiornamenti indipendentemente dalla qualità
hidden. Il best complessivo miscela livelli di visibilità e statistiche RLlib
smussate; non seleziona il miglior imitatore completamente oscurato.

Audit read-only: 137 hash del manifest invariati; configurazioni fisiche,
reward e PPO invariate; quattro restore esatti con 12 stati Adam e stato KL.
Nessun batch visibile nel blocco finale. Non emerge un bug di applicazione
della maschera o un reset accidentale dell'apprendimento.

## Reward: perdita di segnale locale sulle flessioni grandi

La parte positiva resta 80% imitazione della reference served e 20% imitazione
fisica, con score razionali `1/(1+w*loss)` e sharpness 8/4. Si sottraggono
penalità, si clippa la base fra 0 e 1, poi si applicano i termini ulteriori.
Nessuna penalità nuova ankle-positive o knee-torque-rate è stata attivata.

Nei tre rollout originali il costo cumulativo command-rate cresce
78,17 → 87,99 → 96,47: la reference fisica più piatta non significa che la rete
abbia ridotto questo costo. Il termine dominante riguarda il salto dal punto
iniziale del segmento al nuovo endpoint, prima del governor.

Nel rollout CP184 la base reward è clippata a zero in 67 passi; tutti rientrano
nei 112 passi con target knee oltre 35°. Il ritorno di questa porzione scende
da 52,63 in CP90 a 5,69 in CP184. Piccoli miglioramenti che restano sotto la
soglia del clipping non cambiano la reward immediata. Questo è un possibile
ostacolo al recupero dei picchi, non la prova che PPO non riceva alcun segnale
futuro o che la reward preferisca globalmente il comportamento degradato.

La ricostruzione delle reward riproduce esattamente i ritorni archiviati. Il
test diagnostico precedente con inversione del governor mostrava inoltre
che la reward attuale può preferire una traiettoria molto più imitativa;
non si conclude che basti cambiarne i pesi.

## Esplorazione e PPO

KL media per blocco circa 0,0093 / 0,0086 / 0,0096 / 0,0120; explained variance
critic 0,571 / 0,756 / 0,733 / 0,609. Le medie non documentano un collasso
numerico generale, ma non escludono problemi in sottoinsiemi degli stati.

La dispersione cresce: sui medesimi stati archiviati di CP90, sigma media
knee/ankle passa da 0,887/0,722 con CP90 a 1,549/1,163 con CP184 visibile.
Il confronto tiene gli stati fissi, ma questi non sono necessariamente quelli
visitati dal nuovo checkpoint. Gli output sono gaussiani raw poi clippati:
la policy stocastica di training e il rollout che usa la media possono
divergere materialmente. Non si attribuisce il fallimento deterministico
CP184 visible a un rumore che in quel rollout non è campionato.

Una verifica della telemetria stocastica mostra un problema più marcato in
sottoinsiemi degli stati: nelle ultime fino a 50 righe degli episodi, P99
sigma knee passa da 10,34 nella baseline a 1063,20 nel blocco finale, mentre
la mediana resta circa 1. La selezione sovrarappresenta finali e fallimenti;
anche limitando ai soli episodi regolarmente completati, P99 cresce da 7,85
a 92,87. Sono unità di azione normalizzata raw, prima del clipping.

Il fallback effettivo `TorchDiagGaussian` calcola `exp(logstd)` senza clamp;
logprob ricostruite da logits e azioni raw coincidono con il dato registrato
entro 1,9e-6. Esistono campioni reali oltre mille, applicati poi come endpoint
al limite. Non è un errore di interpretazione dei log. Nel blocco finale solo
76/266 episodi salvati completano il time limit; 185 terminano per penetrazione.
Questo identifica una criticità concreta dell'esplorazione da isolare con un
controllo dedicato, non ne dimostra la causalità esclusiva sul degrado.

Le ablation offline sullo stesso CP90 modificano l'endpoint prima del governor
di circa 11°/15° RMS rimuovendo solo clock, 20°/18° rimuovendo solo target,
24°/23° rimuovendo entrambi. Sono sensibilità istantanee, non errori articolari
di rollout, e i contributi non sono additivi. Inferenza originale riprodotta
esattamente su tutte le 500 righe: [input_ablation.json](input_ablation.json).

Il bug bootstrap preesistente rimane un fattore di disturbo del segnale PPO;
è presente anche nella baseline e non isola da solo la causa del masking.

## Limiti causali

Un solo training e un solo start nominale per controllo. Mancano repliche,
una continuazione CP90 tutta visibile di pari budget, ablation fisiche
separate clock/target e un confronto con bootstrap corretto. Non confondere
correlazioni e meccanismi plausibili con cause isolate sperimentalmente.
Le verifiche native sono su macOS arm64; codice diagnostico con pathlib e
subprocess senza shell, verifica Windows live non eseguita.
