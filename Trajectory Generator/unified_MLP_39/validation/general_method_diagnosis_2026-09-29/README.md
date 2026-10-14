# Indagine generale 39D: reference sinistra, imitazione e forma delle coppie

Data: 29 settembre 2026. Instruction check token: `CMC_AGENT_OK_2026`.

**Stato: indagine completata.** Otto rollout diagnostici conclusi; nessun
training o cambiamento di produzione. Protocollo e input storici preservati.

## Domanda e vincoli

L'obiettivo richiesto è ottenere con prescribed LEFT una qualità comparabile
alla buona baseline V26: errore cinematico e forma della traiettoria, e soprattutto
forma della curva di coppia. Non si identifica la qualità con il solo ritorno,
con la sola RMSE o con l'assenza di alta frequenza.

Il metodo deve essere riutilizzabile: nessun tuning della reward per AB06,
nessuna prescrizione di segno o ampiezza della coppia ricavata da questo soggetto.
Sono disponibili due reference ma **un solo modello**, AB06. Le verifiche fra
modelli diversi restano impossibili con il materiale attuale. Non sono una
condizione per svolgere questa indagine, né una proprietà già dimostrata.

## Punto di partenza verificato

La capacità del contenitore 39D di eseguire V26 sul target RIGHT è già dimostrata
dal controllo storico del 23 settembre: ripristinando l'ordine aritmetico delle
operazioni, cinematica, coppie e GRF coincidono byte per byte con V26. RIGHT60,
usata qui come ulteriore controllo, è una policy appresa distinta e non coincide
con quel trasferimento riuscito.

Fonte: [V26, ordine aritmetico e robustezza](../../../../reports/user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).

Le due nuove baseline LEFT best80 e RIGHT60 riproducono esattamente tutte le
500 azioni, osservazioni e sequenze di segmenti archiviate. Anche i **17 file
STO di ciascun caso** sono identici agli originali: [verifica](dynamic/baseline_sto_equality.json).
Questo rende interpretabili gli interventi appaiati successivi.

## Analisi concluse

**Qualità.** I confronti mantengono fase e tempi prescritti, separano
escursione, offset, ritardo e forma normalizzata; per le coppie distinguono
profilo lento, oscillazioni e ripetibilità. Il knee LEFT best80 riproduce circa
il 75% dell'escursione del profilo medio del proprio target; V26 circa il 98,5%
del proprio. Questi confronti sono descrittivi fra reference differenti.
Il tracking actual–served di LEFT è circa 0,28°/0,34° knee/ankle, contro
6,38°/4,04° actual–target: gran parte dell'errore è già nella reference servita.
Metodo, finestre e metriche: [quality/README.md](quality/README.md).

**Governor.** La stessa procedura inversa, con gli stessi criteri e limiti,
produce endpoint regolari che fanno seguire precisamente LEFT e RIGHT al
governor di produzione. Anche la formula analitica senza ottimizzazione
riduce molto l'errore. Non serve modificare cutoff o limiti per dimostrare
questa fattibilità cinematica. Il risultato offline usa il target futuro:
è uno strumento diagnostico, non una policy appresa né una tabella proposta
per il funzionamento futuro. Metodo e numeri: [governor/README.md](governor/README.md).

**Feedback.** Il collegamento isolato attraverso l'ultimo endpoint ha guadagno
locale inferiore a uno negli stati osservati. Il percorso attraverso la
reference servita, soprattutto qddot, può invece amplificare un'azione nel
comando successivo. È presente anche nella buona V26, con guadagno medio
maggiore: non identifica una causa esclusiva LEFT e non certifica instabilità
del sistema completo. [feedback/README.md](feedback/README.md).

**Reward.** I ritorni sono ricostruiti e confrontati solo a target uguale,
con un medesimo contratto di reward. L'oracle LEFT diretto è favorito rispetto
a best80 soprattutto per il minor costo dei comandi, ma conserva ritardo e
un errore di forma peggiore: una reward maggiore non lo rende automaticamente
un teacher risolto. La prova con compensazione del governor serve a cercare
un confronto più discriminante, comprensivo della dinamica fisica.

## Prove dinamiche preregistrate

- Due repliche nominali di 500 segmenti: concluse, esatte.
- Due sequenze inverse di 500 segmenti, una per target, nella normale
  simulazione OpenSim con controller, contatto e protezioni invariati.
- Quattro impulsi di +0,005 per un solo segmento da 10 ms: knee/ankle,
  LEFT/RIGHT. Ogni prova conserva la policy congelata e copre 250 segmenti,
  di cui circa 1,49 secondi dopo l'impulso. Si confronta con la propria baseline.

Il tempo dell'impulso è scelto prima delle prove, evitando il clipping del
comando. Gli stati di LEFT e RIGHT a quel tempo non sono uguali; l'attribuzione
causale riguarda ciascuna coppia perturbato/nominale. L'orizzonte ridotto non
è una prova di completamento o robustezza dell'episodio intero.

Protocollo: [PROTOCOL.md](PROTOCOL.md). Dettagli del confronto e dei tempi:
[dynamic/README.md](dynamic/README.md). Gli output canonici sono salvati sotto
`plot/09_29_2026_general_method_diagnosis/<caso>/` nel root del repository.

## Risultato delle sequenze nella dinamica completa

Entrambe completano 500/500 segmenti. La reference effettivamente servita
coincide con il replay offline entro gli arrotondamenti float32
(errore massimo q/qdot/qddot inferiore a 5,9e-8 / 2,4e-7 / 1,9e-6).
Gli stessi checkpoint sono valutati per diagnostica, ma le loro azioni sono
sostituite dalla sequenza offline. **Non sono nuove policy apprese.**

Confronto LEFT nella finestra comune 13,15–17,95 s:

| Misura | Best80 LEFT | Sequenza inversa LEFT |
|---|---:|---:|
| RMSE giunto–target knee / ankle | 6,376° / 4,040° | **0,390° / 0,339°** |
| Escursione profilo knee / target | 0,750 | **1,001** |
| Errore di forma normalizzato knee / ankle | 0,362 / 0,250 | **0,021 / 0,037** |
| Residuo raw−LP20 della coppia knee, RMS | 3,783 Nm | **0,121 Nm** |
| Impulso verticale del piede | 354,28 N·s | **1166,49 N·s** |
| Reserve traslazionali pelvis, RMS vettoriale | 418,95 N | **222,41 N** |
| Penetrazione massima registrata a 1 kHz | 14,149 mm | 19,028 mm |

La precisione non deriva da scarico del piede o da maggiore assistenza root:
il carico aumenta e le reserve traslazionali diminuiscono. Le reserve biologiche
rotazionali aumentano lievemente, circa 2,3%; quelle protesiche restano zero.
La penetrazione aumenta, pur restando sotto la soglia soft già configurata di
20 mm. Le soglie non sono state scelte dopo aver visto la candidata.

L'ankle presenta ora una curva prevalentemente negativa sotto carico, coerente
con il cambiamento del momento GRF; il picco positivo nella finestra scende
a circa 1,22 Nm. Le coppie sono più regolari, ma **non coincidono nella forma
con V26**. In particolare, il ginocchio raggiunge circa 68,9 Nm in valore
assoluto contro circa 30,0 Nm del benchmark; la RMS è 23,6 contro 8,86 Nm.
L'aumento del carico e la diversa reference impediscono di attribuire queste
differenze alla sola qualità del controllore. Non viene certificata parità
della forma delle coppie con V26.

La reward LEFT **invariata**, sull'intero episodio di 500 passi, passa da
305,06 a **478,66**. Il confronto prova che esiste una sequenza nominale molto
più precisa e regolare, fisicamente eseguita e nettamente preferita dalla reward
attuale. Non prova l'ottimo globale, l'apprendibilità dai 39 ingressi in condizioni
generali o che qualunque ulteriore forma di coppia desiderata sia incentivata.

RIGHT è un controllo utile del limite della stessa procedura: RMSE
0,631°/0,522° e coppie lisce, ma penetrazione fino a 25,074 mm e momento del
contatto penalizzato dal termine GRF attuale alla caviglia. Con contratto comune pen2 ottiene 341,13,
contro 324,15 di RIGHT60 e **363,79 di V26**. Il suo ritorno runtime 348,91 usa
invece pen1 e non è il numero da usare nel confronto comune. L'inversione del
governor risolve la parte cinematica, non garantisce la qualità dinamica di
qualunque reference.

Risultati completi: [qualità e reward](quality/dynamic_pair_metrics.json),
[carico, contatto e reserve](physical/README.md).

## Risposta fisica agli impulsi

Tutte e quattro le prove raggiungono i 250 passi richiesti, con 101 passi
precedenti all'impulso esattamente uguali alla baseline. È un orizzonte
diagnostico di 2,5 s, non un episodio completo di 5 s o un test generale di
robustezza. L'impulso viene applicato a 14,00 s per un solo segmento da 10 ms.

| Caso | Comando differente oltre l'ampiezza dell'impulso | Differenza articolare >0,1° | Prima divergenza FSM |
|---|---:|---:|---:|
| LEFT, knee | 10 ms | 60 ms | 470 ms |
| RIGHT, knee | 220 ms | 340 ms | 680 ms |
| LEFT, ankle | 10 ms | 70 ms | 470 ms |
| RIGHT, ankle | 10 ms | 140 ms | 630 ms |

L'amplificazione precede il cambiamento degli eventi FSM: questi non possono
spiegare l'insorgenza iniziale della differenza. Nel confronto knee esiste
però un confondente verificato: il jerk limiter RIGHT assorbe completamente
l'impulso nel primo segmento, lasciando q/qdot/qddot della reference identici.
LEFT modifica subito qddot knee di circa 0,4681 rad/s².

Il caso ankle non ha quel confondente iniziale: entrambi i governor producono
circa lo stesso delta q/qdot/qddot, con delta accelerazione circa 0,4369 rad/s²
e nessun clamp del jerk ankle. Nei primi 50 ms il massimo scarto del comando
applicato, diviso per l'impulso iniziale, è circa 106,8 LEFT e 4,22 RIGHT;
entro 200 ms entrambi mostrano ampia amplificazione. Gli stati globali e le
reti rimangono differenti, quindi questi numeri non isolano l'effetto dei
pesi né costituiscono una graduatoria generale di stabilità.

I risultati localizzano una sensibilità reale della catena rete–governor–fisica.
La sola memoria dell'endpoint ha piccolo guadagno locale; il percorso che
include le accelerazioni della reference è molto più influente. Anche V26
presenta tale sensibilità offline: non è dimostrata una causa unica o esclusiva
del divario LEFT. [Metodo, risultati e limiti](dynamic/README.md).

## Interpretazione e prossima verifica

La prova LEFT dimostra l'esistenza di comandi nominali molto più precisi e
regolari, preferiti dalla reward attuale. È quindi motivato studiare un metodo
di apprendimento della compensazione del governor, prima di introdurre nuovi
pesi dedicati alla reference. La sequenza dispone del futuro: non dimostra che
la rete causale abbia imparato quei comandi, che possa ricostruirli in ogni
condizione dai propri ingressi o che restino validi dopo una perturbazione.

Il candidato RIGHT mostra che la sola precisione cinematica non garantisce
contatti desiderabili. Il metodo futuro deve verificare insieme reference,
posa del piede, contatto e coppie. Non è giustificato imporre a LEFT una curva
di coppia ricavata da V26 o fissare nuove regole specifiche per AB06.

Il passo proposto è un teacher inverse-governor che ricalcoli i comandi dallo
stato effettivamente visitato, seguito da verifica dell'apprendimento e del
comportamento in circuito chiuso. Un fit sugli stessi 500 istanti nominali non
dimostrerebbe questo risultato; i precedenti BC/DAgger mostrano precisamente
quel rischio. Questa fase di apprendimento non è stata avviata.

Non sono modificati checkpoint, reward, configurazioni di produzione,
governor, FSM, controllori o plugin. Non è avviato alcun training. Gli script
diagnostici usano Python e percorsi rilocalizzabili; le esecuzioni native qui
riguardano macOS arm64, non costituiscono verifica live Windows.
