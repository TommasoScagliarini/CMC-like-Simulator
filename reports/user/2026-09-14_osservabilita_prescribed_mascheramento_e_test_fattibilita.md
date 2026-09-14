# Osservabilità prescribed, mascheramento e test di fattibilità della rete unificata

**Data:** 2026-09-14  
**Instruction check token:** `CMC_AGENT_OK_2026`  
**Stato:** analisi e protocollo sperimentale; mascheramento e curriculum non implementati né addestrati.

La rete unificata V1 è disponibile con 35 ingressi actor e trasferimento
diretto dall'imitazione all'ex-novo. Resta da dimostrare che il pretraining
produca una base di controllo sufficientemente solida senza osservare i
target prescribed. La proposta discussa è fornire temporaneamente questi
target durante l'apprendimento, rimuovendone progressivamente la disponibilità
prima del passaggio all'ex-novo. La fattibilità software e quella qualitativa
devono essere verificate separatamente.

Questo report integra il
[report V1 e protocollo di parità del 6 settembre](2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
Non sostituisce il contratto della V1 e non autorizza implicitamente training
lunghi o modifiche alla baseline.

## 1. Problema: reward e informazione disponibile all'actor

Il prescribed sinistro AB06 fornisce posizione e velocità di
`pros_knee_angle` e `pros_ankle_angle`, lette da `base_kin.get(t)` al tempo
corrente. Nella V1 queste quattro grandezze entrano nella supervisione
imitativa, ma non nell'actor. Posizioni e velocità reali dei giunti restano
osservate.

La reward valuta il comportamento actual/target e served/target; PPO usa il
ritorno e la stima del critic per aggiornare la policy. Non è una loss
supervisionata diretta sui comandi e non aggiunge informazione all'actor al
momento della scelta dell'azione. Se lo stesso vettore osservabile richiedesse
azioni diverse a seconda di una fase nascosta, la reward da sola non
eliminerebbe tale ambiguità. Questa condizione non è ancora dimostrata per
l'intero vettore di 35 osservazioni.

Le due colonne storiche `gait_phase_sin/cos` sono entrambe zero. Le colonne
distinte `online_left_gait_phase_sin/cos` dipendono dagli eventi sinistri
heel/toe validati dalle FSM. Prima di un ciclo valido HS→TO→HS manca un periodo
misurato e la fase online resta nella rappresentazione `(0, 1)`. Il target
prescribed continua invece ad avanzare nel tempo. Stato articolare, GRF,
stato FSM e relativi timer possono comunque fornire altri indizi di fase.

L'obiettivo è mantenere lo stesso actor fra le due fasi, conservando la
capacità appresa e ottenendo autonomia dai quattro target durante l'ex-novo.

## 2. Evidenza già disponibile e suoi limiti

L'audit in sola lettura delle tre tracce dello smoke ha rilevato:

| Start | Campioni | Periodo online zero | HS / TO / cicli validi | Termine |
|---|---:|---:|---:|---|
| Early | 500 | 100% | 0 / 0 / 0 | Limite di 5 s |
| Center | 294 | 100% | 0 / 1 / 0 | Timeout swing |
| Late | 295 | 100% | 0 / 1 / 0 | Timeout swing |

Nei 1089 campioni il sin/cos online resta `(0, 1)`, mentre i target percorrono
circa 48.85° al ginocchio e 30.48–31.82° alla caviglia. Early resta `WAIT_HS`;
center e late registrano un TO e sviluppano un timer swing. I tre reset hanno
stati articolari e FSM differenti: il clock costante non implica che tutte
le osservazioni siano indistinguibili.

La policy ha ricevuto soltanto le tre iterazioni ridotte dello smoke. Questi
dati mostrano una condizione iniziale difficile, non l'impossibilità di
apprendere né la qualità raggiungibile dopo un training completo.

**Allineamento necessario:** le righe delle tracce contengono osservazioni
pre-step e target post-step. Per costruire coppie di apprendimento o analisi
di ambiguità occorre riallineare esplicitamente i timestamp. Il confronto
precedente riguarda soltanto costanza ed escursioni complessive.

Fonti: [TODO della pipeline](<../../Trajectory Generator/unified_MLP/TODO.md>)
e [VALIDATION.md con collegamenti alle tracce](<../../Trajectory Generator/unified_MLP/VALIDATION.md>).

## 3. Soluzione proposta: disponibilità progressiva dei target

Escludere i target dalla sola reward ex-novo non impedisce all'actor di usarli:
essi possono influenzare le azioni, quindi la dinamica e il ritorno task.
L'assenza di un termine imitativo non garantisce indipendenza dell'actor.

Per **mascheramento**, o oscuramento, si intende sostituire la copia dei quattro
target destinata alla rete con un vettore costante, ad esempio zero nella
rappresentazione normalizzata. Non si azzerano gli angoli reali, i file
prescribed, il riferimento usato dalla reward imitativa o la prescrizione
biologica del simulatore.

| Condizione | Quattro canali aggiuntivi actor | Supervisione della reward |
|---|---|---|
| Episodio imitativo con riferimento visibile | Posizioni e velocità prescribed sinistre correnti | Target sinistro originale |
| Episodio imitativo mascherato | Costanti indipendenti dal target | Target sinistro originale |
| Ex-novo della variante proposta | Sempre costanti | Nessun contributo imitativo |

Una variante con questi quattro ingressi avrebbe 39 feature, conservate in
entrambe le fasi. Sarebbe un nuovo contratto rispetto alla V1 a 35 ingressi:
non basta cambiare un flag della configurazione attuale. Un eventuale bit
esplicito di disponibilità porterebbe a 40 ingressi; la sua introduzione non
è stata decisa. Ordine e normalizzazione andrebbero versionati.

Il curriculum proposto riduce la percentuale di episodi con riferimento
visibile, ad esempio 100% → 75% → 50% → 25% → 0%. È una sequenza sperimentale,
non un calendario già scelto o validato. Occorre mantenere il bilanciamento
dei tre start e completare una parte del pretraining interamente mascherata.
Nell'ex-novo la maschera resta obbligatoriamente chiusa.

Questa soluzione conserva forma e pesi della rete nel trasferimento, ma
cambia l'informazione effettivamente disponibile. La progressione deve
insegnare a funzionare senza l'aiuto, evitando di considerare sufficiente
la sola compatibilità tensoriale. Il sapere appreso nel pretraining deve
persistere: l'indipendenza richiesta riguarda i valori prescribed forniti
durante l'ex-novo, non la cancellazione dell'esperienza pregressa.

Per l'indipendenza anche degli aggiornamenti occorre controllare eventuali
copie o trasformazioni dei target nel critic, nella normalizzazione e nelle
loss ausiliarie. I campi privilegiati esistenti `pros_*_target` e
`*_tracking_error` non vanno classificati dal solo nome: nel percorso
attuale sono costruiti con il riferimento servito `q_ref` da `self.kin`,
distinto dal teacher sinistro `base_kin`. Non è stata dimostrata una fuga
del teacher attraverso quei campi, né proposta la loro rimozione automatica.

## 4. Strategia e test di fattibilità proposti

### 4.1 Isolamento software e coerenza della pipeline

- Verificare schema, ordine, unità e normalizzazione identici attraverso
  probe, EnvRunner, learner e rollout nelle due modalità.
- A stato osservabile invariato, sostituire la copia dei target destinata
  alla rete con riferimenti corretti, sfasati e casuali. Con maschera chiusa
  devono coincidere medie e logstd; la derivata degli output rispetto ai
  target nascosti deve essere nulla. Ripetere dopo aggiornamenti e reload.
- Usare un controllo positivo per verificare che il ramo visibile possa
  influenzare la rete: un ramo accidentalmente disconnesso non prova la
  correttezza dell'intero curriculum.
- Confrontare due update ex-novo con policy, critic, optimizer, batch e
  casualità identici, variando soltanto i quattro target e le loro eventuali
  copie derivate. Reward, value, advantage, loss e pesi aggiornati devono
  coincidere entro la tolleranza numerica dichiarata.
- Verificare separatamente che la reward imitativa continui a utilizzare
  il target originale anche quando l'actor è mascherato.
- Conservare per ogni transizione la disponibilità usata in raccolta. Non
  ricampionare la maschera a ogni forward nelle epoche PPO; a policy invariata
  il rapporto delle probabilità dello stesso campione deve restare unitario.
  Verificare anche checkpoint dello stato curriculum e trattamento delle
  statistiche di normalizzazione.

Il riutilizzo dei campioni per più epoche è parte del metodo
[PPO di Schulman et al.](https://arxiv.org/abs/1707.06347); i controlli sulla
maschera sono requisiti progettuali proposti per questa applicazione.

I test esistenti su schema, reward, trasferimento e resume sono riutilizzabili
come base. Non coprono già il nuovo mascheramento e non ne attestano l'efficacia.

### 4.2 Diagnostica dell'osservabilità prima del training comparativo

Raccogliere traiettorie valide e coppie osservazione/target allo stesso tempo.
Valutare su episodi esclusi dal fit quanto i target sinistri siano ricostruibili
dalle sole 35 osservazioni, separando avvio e regime. Cercare osservazioni
simili associate a target differenti senza introdurre timestamp o fase
prescribed fra gli ingressi del modello causale.

Confronti diagnostici con brevi storie causali o con una fase privilegiata
possono aiutare a individuare il tipo di informazione mancante. Un fit
scarso non prova da solo impossibilità; un fit buono non prova controllo
stabile in simulazione. La valutazione va separata per episodi/condizioni,
evitando che campioni temporali adiacenti rendano il risultato ottimistico.

### 4.3 Esperimento controllato A/B/C

| Variante | Disponibilità nel pretraining | Scopo |
|---|---|---|
| A | Sempre visibile | Verificare l'apprendimento quando il riferimento è disponibile |
| B | Sempre mascherata | Controllo senza aiuto prescribed all'actor |
| C | Progressivamente ridotta, poi sempre mascherata | Misurare il beneficio trasferibile dell'aiuto iniziale |

Usare la stessa architettura sperimentale a 39 ingressi nei tre casi,
inizializzazioni abbinate per seed, stesso budget di transizioni, parametri
PPO, reward imitativa, start, fisica e detector. B mantiene le quattro
colonne costanti sin dall'inizio; la V1 originale resta conservata.

Valutare tutti i checkpoint anche con maschera chiusa. A visibile è un
controllo diagnostico, non dimostra autonomia. C che funziona senza target
e migliora B a parità di budget fornisce evidenza favorevole al curriculum.
Se funziona soltanto A visibile, resta una dipendenza dall'aiuto. Se nessuno
impara nel budget assegnato, il risultato è inconclusivo rispetto alla
fattibilità informativa: vanno esaminati anche reward, controllo e durata.

Registrare la qualità prima e dopo ciascuna riduzione di disponibilità.
Ripetere con più seed per una conclusione qualitativa; una prima prova con
un solo seed può servire da pilot, non da conferma statistica.

### 4.4 Rollout e criteri qualitativi

Valutare i tre start esistenti, offset 1.756870983805102,
1.956870983805102 e 2.156870983805102 s, con durata massima 5 s e target
mascherati. Misurare per giunto e per start:

- errore e sfasamento actual/target e served/target;
- HS/TO validi, cicli completi e tempo al primo periodo valido;
- terminazioni, copertura temporale e disponibilità delle metriche;
- penetrazione, saturazioni SEA, clipping e regolarità dei comandi.

Separare il tratto prima del primo ciclo dal regime successivo. Un episodio
di 5 s senza cicli validi non certifica il cammino; un episodio terminato
presto non diventa migliore grazie a errori medi o penetrazione più bassi.
Affiancare valutazioni deterministiche a prove stocastiche con sigma e seed
controllati e condizioni riservate alla valutazione.

Le soglie qualitative e i margini di regressione devono essere fissati prima
della selezione dei checkpoint. Il benchmark baseline completo ai tre start
resta da costruire, come documentato nel report del 6 settembre.

### 4.5 Trasferimento all'ex-novo e ripresa

Caricare direttamente l'actor C già validato senza target. Verificare critic
e optimizer nuovi, identità della rete delle medie dopo l'impostazione di
sigma 0.010, actor/logstd congelati nel warm-up e aggiornamento del critic.
Save/reload in un processo nuovo deve conservare maschera e stato della fase;
il resume non deve riaprire i target o ripetere il warm-up già concluso.

Confrontare poi l'ex-novo inizializzato da B e da C con uguale budget. Il
beneficio ricercato è una base che migliori o faciliti l'apprendimento task
anche dopo aver spento l'imitazione, non soltanto un buon score imitativo.

## 5. Alternativa discussa: avvio supervisionato

Resta aperta l'alternativa di addestrare direttamente lo stesso actor finale
a 35 ingressi con etichette teacher, eventualmente aggiungendo stati visitati
dallo student. Le etichette devono essere comandi validati nella catena
slew/C2/SEA: non si assume che copiare gli angoli prescribed dia comandi
adeguati. Un teacher più informato non risolve automaticamente ambiguità
negli ingressi dello student.

La stima discussa di mezza giornata/un giorno riguarda un primo prototipo
diagnostico, non una policy qualificata. I precedenti storici di circa 83 s
per un fit su dati già pronti e circa 25 minuti per sei rollout non includono
lo sviluppo del nuovo teacher e non stimano il tempo alla qualità desiderata.
Nessuna delle due nuove strategie è stata implementata o selezionata.

## 6. Configurazione già aggiornata e verifiche effettive

Su richiesta dell'utente, il peso `grf_penetration_weight` dei preset
imitativi completo e smoke è stato portato a **1.0**; i preset ex-novo
conservano **0.05**. Le soglie soft/hard restano **20/28 mm**. L'override è
documentato nei preset e nella provenienza; non cambia lo schema actor.

I preset completi mantengono batch 4608 e 12 EnvRunner, con 1536 transizioni
per start. Il batch 4096 della vecchia imitazione non è divisibile esattamente
fra i tre start; un suo ripristino richiederebbe ridefinire il vincolo di
bilanciamento, non è stato applicato. Lo start unico storico non coincide
con il reset comune ai tre start scelto per la rete unificata.

| Verifica | Stato effettivo |
|---|---|
| Suite aggiornata dopo il peso imitativo 1.0 | 82 test superati in 5.82 s su macOS, già documentati; non rieseguiti per scrivere questo report |
| Smoke V1 del 6 settembre | Imitazione, critic-only, ex-novo: 192 transizioni per fase; completato con peso imitativo ancora 0.05 |
| Ambiente reale V1 | Schema 35/84 e parità delle osservazioni nelle due modalità ai tre start, al reset e dopo un passo |
| Audit delle tracce | Lettura degli artefatti esistenti; nessun nuovo rollout |
| Nuovo peso 1.0 in simulazione | Non ancora verificato con un nuovo run |
| Maschera, curriculum, confronto A/B/C e trasferimento relativo | Proposti, non implementati né eseguiti |
| Qualità del cammino e parità con la baseline | Non dimostrate |
| Windows live | Non eseguito; comandi e percorsi portabili predisposti per la V1 |

I risultati dinamici storici riguardano macOS arm64. La creazione del report
ha comportato letture del repository e controlli documentali, senza nuovi
test della rete, aggiornamenti PPO o simulazioni.

Controlli della redazione superati: codifica UTF-8, esistenza dei 15
collegamenti locali complessivi nei due nuovi documenti, propagazione esatta
di tutti gli 11 TODO del presente report nel daily e assenza di differenze
dei file tracciati rispetto a `8ef48f91`; nessuna modifica staged.

## 7. File modificati e isolamento

Le modifiche già effettuate nella discussione precedente riguardano,
sotto `Trajectory Generator/unified_MLP/`, i preset `configs/pretraining.yaml`
e `configs/smoke_pretraining.yaml`, `configs/provenance.json`,
`tests/test_config.py`, `README.md`, `VALIDATION.md` e `TODO.md`, per il peso
imitativo e la documentazione dell'osservabilità. I preset ex-novo restano
a 0.05. Queste modifiche precedono il presente comando `create_report`.

Con questo comando vengono creati esclusivamente:

- il presente report utente;
- [il daily del 14 settembre](../daily/2026-09-14_daily-report.md), con la
  propagazione dei TODO richiesta da `AGENTS.md`.

La baseline del commit `8ef48f914898fff8431cf8d164554448dbe35947`, i suoi
checkpoint e registri, il runtime condiviso e il plugin C++ non sono oggetto
di modifica. Actor predittivo e compensazione del ritardo restano rinviati.

## 8. TODO da propagare

- [ ] Risolvere e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start.
- [ ] Costruire dati diagnostici temporalmente allineati e valutare osservabilità delle 35 feature su episodi indipendenti.
- [ ] Scegliere fra curriculum con target mascherati, avvio supervisionato e possibili soluzioni causali di fase/memoria sulla base di evidenze.
- [ ] Se si procede con la maschera, definire il nuovo schema, la normalizzazione, l'eventuale indicatore di disponibilità e lo stato persistente del curriculum, conservando la V1.
- [ ] Implementare e verificare isolamento dei target, coerenza raccolta/PPO e indipendenza degli aggiornamenti ex-novo dalle copie dei target.
- [ ] Definire budget, seed, progressione e criteri di accettazione prima del confronto A/B/C; eseguire poi il pilot e le repliche necessarie.
- [ ] Validare i candidati completamente mascherati ai tre start, con metriche separate di avvio, tracking, cicli validi e protezioni.
- [ ] Verificare trasferimento, warm-up e resume della variante scelta e confrontarne il beneficio nel successivo ex-novo.
- [ ] Misurare in un nuovo run imitativo l'effetto del peso di penetrazione 1.0, senza attribuirgli gli esiti dello smoke storico a 0.05.
- [ ] Completare il benchmark e il verdetto di parità qualitativa secondo il report del 6 settembre, includendo costo di apprendimento e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows della pipeline scelta; riprendere predittività e compensazione del ritardo soltanto in una versione successiva.

## 9. Riferimenti

- [Implementazione del target sinistro e invarianti V1](<../../Trajectory Generator/unified_MLP/env.py>)
- [Factory della rete unificata](<../../Trajectory Generator/unified_MLP/env_factory.py>)
- [Guida e comandi della V1](<../../Trajectory Generator/unified_MLP/README.md>)
- [Validazione e artefatti storici](<../../Trajectory Generator/unified_MLP/VALIDATION.md>)
- [TODO dell'osservabilità](<../../Trajectory Generator/unified_MLP/TODO.md>)
- [Provenienza dei preset](<../../Trajectory Generator/unified_MLP/configs/provenance.json>)
- [Fit supervisionato storico J15R1](2026-08-27_v26c_j15r1_fresh_refit_execution.md)
- [Qualifica closed-loop storica J16](2026-08-27_v26c_j16_j15r1_closed_loop_requalification_execution.md)
