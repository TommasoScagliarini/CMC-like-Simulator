# Piano operativo per il recupero della pipeline ex-novo V26 / FSM v3 / Morphology Corridor

**Data:** 22 agosto 2026  
**Stato:** approvato dall'utente il 22 agosto 2026; esecuzione per fasi fail-closed  
**Regola di attivazione:** il via esplicito è stato ricevuto il 22 agosto 2026. Ogni fase resta subordinata al PASS del gate precedente e alla revisione architetturale degli artefatti.  
**Baseline di rollback:** lineage ex-novo del 15 luglio 2026, con H0 `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last`.
**Revisione indipendente:** Claude Fable 5, effort `max`, eseguita il 22 agosto 2026 in modalità read-only. Le correzioni obbligatorie della revisione sono incorporate nel presente testo.

## 1. Scopo e risultato atteso

Lo scopo è recuperare, sotto il runtime definitivo basato su detector V26, FSM v3 e Morphology Corridor corretto, la qualità cinematica e dinamica raggiunta dalla pipeline di luglio. Il risultato non sarà considerato raggiunto sulla sola base del return PPO: il candidato dovrà generare traiettorie di knee e ankle con segno, escursione e forma compatibili con una cinematica biologica, completare cicli validi, produrre curve di coppia plausibili e mantenere sotto controllo reserve, penetrazione, clipping e diagnostica della FSM v3.

Il piano separa deliberatamente tre problemi:

1. qualità della policy media deterministica prima di PPO;
2. robustezza della policy alla distribuzione target e al rumore stocastico;
3. apprendimento PPO sotto FSM v3 e Morphology Corridor.

PPO non sarà usato per tentare di riparare una policy iniziale che non supera i gate cinematici e di ciclo.

## 2. Evidenze e diagnosi di partenza

### 2.1 Differenza sostanziale fra le due pipeline

La pipeline di luglio non era un semplice trapianto di rete. Prima del training ex-novo aveva costruito una vera policy target-domain mediante teacher prescritto AB06, adattamento phase-aligned, DAgger e recovery multi-start. Il corpus di adattamento conteneva 24.712 esempi, inclusi 16.000 anchor e 712 esempi di recovery phase-aligned. La media dell'actor era stata adattata integralmente; la deviazione standard era stata portata e qualificata a `0.005`. Il checkpoint H0 risultante camminava già prima degli update PPO. Il pilot del 15 luglio ha poi eseguito update conservativi e non ha creato da zero quella capacità.

La pipeline B0820 ha invece portato un actor imitativo V26 da 39 feature a un actor deployable da 35 feature senza un successivo refit supervisionato nel dominio finale. Il danno informativo comprende **sei segnali**, non quattro: quattro target cinematici sani eliminati (`source_only_features_dropped`) e i due ingressi `gait_phase_sin/cos` condivisi ma azzerati (`shared_features_zeroed`) perché il clock prescritto non è disponibile nel target. Il report è `validation/critic_warmup/2026-08-20_B0820_native_v26_frozen_actor_iter5/actor_transplant_report.json`. Il bridge dovrà quindi sostituire esplicitamente il clock prescritto con `online_left_gait_phase_sin/cos`, non limitarsi a riaddestrare dopo il drop dei quattro target.

L'actor V26 ha inoltre ereditato una scala stocastica state-dependent dell'ordine di `sigma ~= 0.5`: l'entropia osservata `1.35–2.15 nat` equivale a circa `sigma 0.47–0.71`, contro `0.005` esatto e bit-costante nella lineage di luglio. In B0820 `freeze_logstd` applica `detach()` alla metà log-std dei logits, ma l'encoder condiviso continua a cambiare; di conseguenza non congela la distribuzione né rende costante sigma. Nel nuovo bridge la testa log-std dovrà avere righe di peso azzerate, bias `ln(0.005)`, digest verificato ed entropia attesa `-7.7588` dopo save/reload e sincronizzazione.

Questa combinazione spiega perché B0820 arrivi a PPO con una caviglia sempre positiva, escursioni collassate e nessun ciclo qualificato. Il ginocchio non è positivo: nel best nominale resta fra `-0.513` e `-0.174 rad`, ma la ROM è soltanto `0.339 rad` contro `0.868 rad` nel riferimento di luglio e `0.997 rad` nel best imitativo V26. Il difetto corretto da misurare è quindi la forma/ROM phase-aligned, non il solo segno.

Il Morphology Corridor, con peso `0.0025`, è una guardia debole: nel best risultano 156 segmenti settled e 0 segmenti discarded, mentre a livello di **campioni** risultano 160 risolti e 332 scartati (340 nel totale della diagnostica causale). Può penalizzare deviazioni entro un contratto causale valido, ma non generare autonomamente una traiettoria biologica né correggere uno stance-lock. La forma di H0 B0820 è sostanzialmente identica sotto v2 e v3 (ROM knee `0.336 rad` in entrambi), quindi deriva dall'actor; il runtime modifica invece radicalmente terminazioni e segnali di apprendimento.

Infine, i 50 update PPO B0820 sono stati sostanzialmente un'operazione nulla: il massimo `max_minibatch_mean_kl_loss` è `5.7e-6`, il coefficiente KL decade da `0.5` a circa `8.9e-16`, e il best resta quasi identico a H0 (`-56.22` contro `-58.56`, 0 cicli, ROM knee `0.339` contro `0.336`). B0820 dimostra quindi il fallimento dell'H0 trapiantato, ma **non** consente di concludere che PPO sotto FSM v3 sia incapace di apprendere. Il critic warmup è inoltre avvenuto senza `binary_phase_actor_fsm_version: v3`, quindi su un runtime di terminazione diverso dal training successivo; questo confound deve essere eliminato.

### 2.2 Riferimenti quantitativi

Il riferimento di luglio, valutato nel rollout auditato, raggiunge `+52.42694` di return, 4 HS, 3 TO, tre cicli validi, un evento invalido, reserve massima `493.45 Nm`, `action_abs_max = 0.828` e nessun clip. Va però qualificato: il best era stato scelto per return alla logical iteration 24, fuori dalle otto milestone preregistrate; tutte le milestone erano state respinte dallo screen robusto e i seed held-out 126–128 non erano stati aperti. Per i gate di robustezza il riferimento corretto è pertanto la qualifica H0 del 13–14 luglio, non il pilot best.

Il best B0820 completa 500 step per `episode_time_limit` ma resta a `-56.2236`, non produce cicli qualificati, registra 3 HS e 1 TO v3, `invalid_event_count = 3`, reserve massima `872.52 Nm`, `action_abs_max = 1.186` e tre step clippati; la caviglia è sempre positiva e la ROM è collassata. Il detector legacy legge invece 4 HS e 3 TO sinistri regolari: i gate dovranno riportare separatamente eventi legacy ed eventi accettati dalla catena v3. `total_cancelled_transition_count = 0` è verificato nella trace; qualsiasi valore `resync_count` dovrà essere accompagnato dal campo e dall'artefatto sorgente, non soltanto riportato per memoria.

Questi valori saranno ricampionati con lo stesso protocollo prima di qualunque confronto decisionale, in modo da evitare confronti fra rollout, start, seed o runtime diversi.

## 3. Invarianti e vincoli

- Tutto il lavoro relativo a policy, rete e training resterà in `Trajectory Generator/`.
- Detector V26, FSM v3 e Morphology Corridor corretto resteranno il runtime target. Eventuali prove legacy saranno solo controlli diagnostici e non candidati finali.
- Non saranno modificati plugin C++, semantica del comando SEA, modello OpenSim o hard guard fisiche per far passare un candidato.
- Il checkpoint del 15 luglio e gli artefatti B0820 saranno trattati come baseline immutabili; ogni nuova lineage avrà directory, configurazione risolta, manifest e digest propri.
- Ogni fase sarà fail-closed: un gate fallito arresta la promozione alla fase successiva.
- Non saranno confrontati checkpoint usando protocolli differenti. Start, seed, rumore, durata, reward, soglie, GRF e modalità di rollout saranno registrati e identici fra i candidati.
- Il return sarà una metrica secondaria rispetto a cicli, cinematica, coppie e sicurezza.

## 4. Piano di esecuzione

### Fase 0 — Freeze, inventario e baseline comparabile

**Obiettivo:** creare un punto di partenza riproducibile senza modificare pesi.

Attività previste:

1. congelare digest e manifest di:
   - baseline imitativa del 24 giugno;
   - actor adattato target-domain e H0 di luglio;
   - checkpoint best/last del pilot del 15 luglio;
   - actor imitativo V26, trapianto 39→35, warmup critic e best/last B0820;
   - configurazioni V26/FSM v3/corridoio e profilo morphology;
2. produrre una matrice di provenienza che distingua pesi actor, critic, optimizer, `log_std`, observation schema, normalizzazione, runtime FSM e reward;
3. censire separatamente entrambe le run B0820 (`MLP_ExNovo_B0820_from_zero_50iter`, che in realtà riprende il warmup, e `MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter`) per evitare che il nome `from_zero` venga interpretato come inizializzazione casuale;
4. costruire una matrice esplicita delle differenze di runtime non isometriche: profilo GRF (`grf_correct_magnitude` contro `tangent_v2`, da verificare anche mediante SHA), assenza del blocco `binary_phase` nel runtime di luglio, soglie di penetrazione `15/25` contro `20/28`, timeout swing `1.1 s` contro `2.6 s`, profilo/modalità morphology e peso `0` contro `0.0025`;
5. rieseguire soltanto rollout deterministici e stocastici comparabili su nominale e ±0,20 s, includendo un controllo 39D e l'H0 luglio sotto v3 come **controlli di compatibilità**, non come confronti fra policy a parità di runtime;
6. salvare overlay sincronizzati per fase di azione raw/served, knee/ankle, coppie SEA, GRF, eventi, reserve e corridoio;
7. produrre metriche separate per ciascuno start, con attenzione specifica allo start `-0,20 s`, che in B0820 ha raccolto episodi da 500 step e return prossimo a zero ma soltanto 13 segmenti settled e 478 campioni scartati;
8. quantificare la deriva dell'actor fra H0 e best, oltre alla sola differenza di return, mediante norma/digest dei parametri, deviazione delle azioni e KL su un observation set congelato.

**Gate F0:** tutti gli artefatti devono essere risolvibili, avere digest e produrre risultati ripetibili. Le differenze di runtime devono essere classificate prima del confronto; i controlli storici non saranno dichiarati isometrici. Qualsiasi divergenza rispetto ai valori storici deve essere spiegata prima di procedere.

### Fase 1 — Ablation causale della regressione 39→35 e della scala stocastica

**Obiettivo:** confermare con prove isolate i due meccanismi principali della regressione.

Saranno costruiti, senza PPO, almeno i seguenti confronti:

| Caso | Actor/feature | Media | Sigma | Funzione |
|---|---|---|---:|---|
| A | V26 nativo 39D | originale | deterministica e nativa | controllo privilegiato di compatibilità, non policy deployable equivalente |
| B | deployable 35D | hard-drop attuale | deterministica | misura del danno dovuto al drop |
| C | deployable 35D | hard-drop attuale | `0.005` | separazione fra errore della media e rumore |
| D | deployable 35D | refit supervisionato minimo | `0.005` | prova diretta di recuperabilità |
| E | H0 luglio | originale | `0.005` | controllo storico di compatibilità, non confronto isometrico |

Le prove useranno lo stesso set di start e seed. Saranno misurati errore d'azione rispetto al teacher, ROM, velocità e jerk di knee/ankle, correlazione e inviluppo phase-aligned, attraversamenti dello zero soltanto nelle fasi in cui sono richiesti dal riferimento, HS/TO/cicli, cause di fine, reserve e coppie. Una metrica generica di “frazione positiva” non sarà usata come gate: anche il best imitativo V26 ha ankle sempre positiva e il riferimento di luglio è positivo per l'88,1% del ciclo.

**Gate F1:** procedere con il bridge supervisionato solo se il refit 35D migliora contemporaneamente la fedeltà al teacher e i gate closed-loop. Se il caso D non recupera, la diagnosi sarà riaperta prima di spendere budget in un dataset esteso.

### Fase 2 — Nuovo bridge target-domain deployable sotto runtime finale

**Obiettivo:** ricostruire il passaggio che aveva reso efficace la pipeline di luglio, questa volta direttamente nel contratto deployable 35D e con V26/FSM v3/corridoio definitivi.

Strategia primaria:

1. usare il comportamento prescritto AB06 come teacher target-domain;
2. mantenere l'actor student esclusivamente sulle 35 feature disponibili in deployment;
3. usare le quattro feature target sane, se utili, soltanto nel teacher, nel critic o nella loss supervisionata, mai come input dell'actor finale; sostituire nel contratto student anche il clock prescritto rimosso con `online_left_gait_phase_sin/cos`, portando a sei i segnali da trattare esplicitamente;
4. raccogliere dati phase-aligned sui tre start esatti e aggiungere un blocco recovery dedicato allo start `-0,20 s`, più recovery mirate attorno a TO, HS, stance timeout, resync e cancellazioni HS; almeno un offset ulteriore resterà non visto durante il fit;
5. mantenere anchor della baseline imitativa per evitare forgetting e collasso di forma;
6. eseguire DAgger per un numero e un budget di round preregistrati, piccoli e limitati, salvando ogni round e selezionando per comportamento closed-loop, non per sola loss offline;
7. adattare l'intera media dell'actor deployable, non solo un residuale terminale, salvo evidenza contraria dalle ablation;
8. imporre sigma costante azzerando le righe della testa log-std e impostandone il bias a `ln(0.005)`; verificare digest della testa, entropia `-7.7588`, valore effettivo dopo save/reload e dopo la sincronizzazione su learner ed EnvRunner. Il solo `detach()` del ramo log-std non sarà considerato un freeze sufficiente perché l'encoder è condiviso.

La dimensione iniziale del corpus seguirà la scala risultata efficace a luglio, ma non sarà copiata ciecamente. La composizione minima dovrà includere anchor, nominale, ±0,20 s, recovery e casi v3 con resync/HS cancellato. Ogni categoria avrà conteggi e split dichiarati; validation e test non saranno riutilizzati nel fit. I seed 126–128 resteranno sigillati fino alla valutazione finale e non saranno usati né per DAgger né per scegliere iperparametri o milestone.

**Gate F2 offline:** errore e distribuzione delle azioni non peggiori della baseline di luglio su anchor e sensibilmente migliori del hard-drop B0820 sui casi target.

**Gate F2 closed-loop:** almeno un candidato deve completare l'orizzonte nominale e produrre cicli validi a tutti e tre gli start deterministici. Knee e ankle devono rispettare inviluppi preregistrati di ROM, forma e correlazione phase-aligned; gli attraversamenti dello zero saranno richiesti solo se e dove imposti dal riferimento biologico. Nessun candidato che fallisca questo gate passa al critic warmup.

### Fase 3 — Qualifica pre-PPO deterministica e stocastica

**Obiettivo:** dimostrare che H0 è già una policy cinematicamente valida e sufficientemente robusta.

Protocollo previsto:

- tre start deterministici: nominale e ±0,20 s;
- almeno tre seed stocastici per start a `sigma = 0.005`;
- un set validation held-out di recovery/eventi e di offset non usato da DAgger, distinto dai seed finali sigillati 126–128;
- runtime finale con FSM v3 e Morphology Corridor strumentato, inizialmente senza attribuire al reward morphology il compito di correggere la forma.

Metriche obbligatorie:

- full episode, HS, TO e cicli validi, riportando separatamente gli eventi del detector legacy e quelli accettati/rifiutati dalla FSM v3;
- `phase_timeout_stance`, `morphology_causal_contract_failure`;
- `resync_count`, `hs_cancelled_count` e transizioni accettate/rifiutate;
- range, escursione, velocità, jerk e conformità di fase di knee/ankle; il segno sarà valutato rispetto all'inviluppo biologico fase per fase;
- correlazione e RMSE phase-aligned rispetto al riferimento biologico;
- profilo e picchi di coppia SEA, reserve e clipping;
- penetrazione, safety/support interventions e cause di terminazione;
- quota di segmenti morphology settled, discarded e causalmente validi.

**Gate F3:** 3/3 start deterministici a orizzonte con cicli validi; robustezza stocastica almeno equivalente alla **qualifica H0 del 13–14 luglio**, non al best esplorativo del pilot; assenza di collasso di ROM/forma; nessuna regressione sostanziale di reserve, penetrazione o coppie. Le soglie numeriche definitive saranno preregistrate usando la baseline di luglio ricampionata in F0. I seed sigillati 126–128 saranno aperti una sola volta per il gate finale, non durante l'ottimizzazione.

### Fase 4 — Critic warmup nel trainer definitivo

**Obiettivo:** inizializzare il critic senza cambiare l'actor qualificato.

Attività previste:

1. costruire il trainer con observation schema, connector, exact-start, V26, FSM v3, soglie di terminazione e corridoio **identici al training successivo**, includendo digest del codice e configurazione risolta; il warmup B0820 non soddisfaceva questo requisito perché non dichiarava `binary_phase_actor_fsm_version: v3`;
2. congelare realmente media actor e `log_std`;
3. eseguire un warmup critic breve con audit bit-exact dell'actor prima/dopo ogni iterazione;
4. verificare sincronizzazione dei pesi sui dodici EnvRunner remoti e sul runner locale di inference/sync prima del primo batch;
5. verificare optimizer state, learning rate e contatori al checkpoint e al resume;
6. controllare distribuzione e segno degli advantage separatamente per start. In particolare, non sarà accettato senza spiegazione un batch come la logical iteration 6 B0820, in cui allo start `-0,20 s` tutti i 1536 advantage erano positivi con media `+0.325`, segnale compatibile con il critic addestrato sulle terminazioni brevi del vecchio runtime e poi applicato a episodi da 500 step.

**Gate F4:** actor e `log_std` bit-identici; entropia coerente con sigma costante; value loss finita e stabilizzata sotto il runtime finale; advantage per-start plausibili; tutti i learner check exact-start superati; nessuna anomalia di restore, sincronizzazione o worker.

### Fase 5 — Pilot PPO conservativo con milestone promuovibili

**Obiettivo:** verificare che PPO preservi o migliori H0 senza distruggerne la cinematica.

Il primo pilot sarà breve. La configurazione di partenza resterà conservativa: exact-start bilanciato, una epoch, `freeze_logstd` verificato, guard KL fail-closed e learning rate iniziale nell'ordine della ricetta del 15 luglio. La risposta effettiva dell'actor, non il learning rate nominale da solo, determinerà la configurazione: con sigma circa `0.5` lo stesso learning rate muoveva la media circa cento volte meno che con sigma `0.005`. Non sarà avviato direttamente un nuovo run lungo.

Prima dell'avvio saranno preregistrati un intervallo KL operativo indicativo nell'ordine di `5e-4–1e-3`, coerente con gli update utili della lineage di luglio, un tetto di sicurezza e una soglia minima di attività. Se il KL resta sotto la soglia minima, il pilot sarà classificato come **update nullo** e non come evidenza di stabilità o di incapacità di PPO; B0820, con massimo `5.7e-6`, ricade in questa categoria.

Dopo ogni milestone saranno eseguiti rollout comparabili. Una milestone sarà promossa solo se:

- mantiene i gate F3;
- non peggiora materialmente le curve cinematiche e di coppia;
- non aumenta sistematicamente `phase_timeout_stance` o `morphology_causal_contract_failure`;
- `resync_count` e `hs_cancelled_count` restano interpretabili e non mostrano una deriva crescente;
- il KL per minibatch resta entro il corridoio preregistrato: sotto il tetto di sicurezza ma anche sopra la soglia minima di attività nelle milestone destinate a testare capacità di apprendimento;
- l'eventuale miglioramento del return non è ottenuto a spese di ROM, reserve o sicurezza.

Le milestone saranno valutate a iterazioni fissate prima del run, mediante rollout per-start e senza selezionare retrospettivamente il massimo return rumoroso. Se nessuna milestone supera H0, H0 resta il candidato e si evita di interpretare 50 update non migliorativi come progresso.

### Fase 6 — Attivazione e taratura del contributo morphology

**Obiettivo:** usare il corridoio per rifinire una gait già valida, non per crearla.

Il contributo morphology sarà valutato mediante ablation controllata `weight = 0` contro il peso candidato, con tutti gli altri parametri invariati. Il profilo target resterà `event_anchored`, contratto `causal_delayed`, peso iniziale `0.0025` e hard termination disabilitata, salvo modifica esplicita preregistrata. L'audit registrerà che il corridoio valuta il **riferimento servito** e non direttamente la cinematica misurata `q`; perciò il digest includerà anche tutti i flag `allow_effects` che determinano quali effetti arrivano alla policy/reward. Saranno mantenuti distinti i conteggi per segmento (`settled_segments`, `discarded_segments`) da quelli per campione (`solved`, `discarded`, `total_dropped`). Prima di aumentare il peso dovranno crescere copertura dei segmenti settled e validità del contratto causale. Se la copertura utile per campione resta insufficiente, si correggeranno copertura/contratto o dati; non si compenserà aumentando il peso.

La promozione richiederà un miglioramento delle metriche phase-aligned di knee/ankle senza regressione di cicli, coppie, reserve, penetrazione e contatori v3.

### Fase 7 — Run esteso e decisione finale

Un training più lungo sarà autorizzabile solo dopo PASS delle fasi 0–6. Il run esteso conserverà milestone frequenti, rollback automatico e valutazioni esterne al batch di training. La decisione finale confronterà:

1. nuovo H0 deployable V26/FSM v3/corridoio;
2. miglior milestone PPO della nuova lineage;
3. H0 e checkpoint best del 15 luglio ricampionati;
4. B0820 best come controllo negativo documentato.

## 5. Strategia alternativa e rollback

### Alternativa strutturale

Se il teacher privilegiato 39D è utile ma il distillato 35D non raggiunge i gate, si manterrà l'actor nativamente 35D e si sposteranno le informazioni privilegiate esclusivamente nel critic asimmetrico e/o nella loss del teacher. Non sarà ripetuto un hard-drop senza refit.

### Rollback operativo

Si ritorna alla baseline del 15 luglio come baseline primaria se si verifica una delle condizioni seguenti:

- nessun actor 35D supera F2/F3 dopo i round supervisionati preregistrati;
- la qualità cinematica recuperata richiede feature non disponibili in deployment;
- V26/FSM v3 produce un'incompatibilità dimostrata anche con il teacher prescritto;
- il Morphology Corridor non raggiunge una copertura causale sufficiente senza alterare impropriamente safety/reward;
- il nuovo candidato non eguaglia la baseline storica su cicli e forma, pur spendendo il budget sperimentale concordato.

Il rollback non implica abbandonare il corridoio: significa usare il checkpoint del 15 luglio come riferimento attivo e trattare l'integrazione V26/morphology come un problema separato di adattamento e compatibilità.

## 6. Indagine sul valore 4096 e sui contatori crescenti

### 6.1 Risultato

Nel B0820 non sono esistiti 4096 environment e il loro numero non è aumentato. La topologia effettiva era:

```text
EnvRunner remoti di sampling            = 12
environment per EnvRunner remoto        = 1
runner locale per inference/sync        = 1 (non campiona il batch remoto)
train_batch_size                        = 4608 timestep unici per update
rollout_fragment_length                 = 384 timestep per runner remoto
runners_per_start                       = 4
step per ciascuno dei 3 start           = 1536
minibatch_size                          = 512
```

Per l'utente il numero corretto di environment che campionano in parallelo è quindi **12**, non 4096 e neppure 13. La topologia RLlib comprende anche un runner locale di inference/sincronizzazione, ma quest'ultimo non aggiunge un tredicesimo frammento al batch distribuito.

La provenienza dei numeri va distinta. Il `training_cfg.resolved.yaml` B0820 contiene `num_env_runners = 12`, `train_batch_size = 4608`, `minibatch_size = 512` ed exact-start attivo. `num_envs_per_env_runner = 1` è imposto dal codice in `train_ppo_mlp.py`; `rollout_fragment_length = 384`, `runners_per_start = 4` e `1536` step/start derivano dal contratto calcolato in `start_sampling.py` e sono registrati nei summary/JSONL. Non sono tutti campi espliciti della configurazione risolta.

Il `4096` ricordato dall'utente era il **batch target di timestep**, non il numero di environment, nella pipeline imitativa V26 precedente: quella usava 13 runner ed exact-start disabilitato, quindi gli incrementi reali osservati potevano essere non costanti (`5987`, `5355`, `5357`, ecc.). Anche il pilot del 15 luglio mostra una transizione storica: la prima coordinata lifetime è `8704 = 4096 + 4608`. Con il contratto exact-start attuale, i tre start devono ricevere lo stesso numero di runner e righe. Sono stati quindi scelti 12 runner, quattro per start; `4096 mod 12 = 4`, mentre `4608 = 12 x 384 = 3 x 1536` ed è divisibile anche per il minibatch da 512.

Esistono infine altri due `4096` nella configurazione, `morphology_completed_segment_max_samples` e `morphology_causal_max_samples`: sono cap massimi di campioni delle diagnostiche morphology e non hanno alcun rapporto con il numero di environment o con il batch PPO.

### 6.2 Perché il numero osservato cresce

La metrica principale salvata e mostrata durante il run è `num_env_steps_sampled_lifetime`: è un contatore cumulativo dalla nascita del trainer/checkpoint, non la dimensione del batch corrente e non il numero di environment. B0820 riprende dal critic warmup dopo cinque iterazioni, non parte realmente “from zero”. Perciò:

```text
dopo warmup iterazione 5     5 x 4608 = 23040 step cumulativi
prima riga B0820, iterazione 6             27648
iterazione 7                               32256
...
iterazione 55                             253440
```

L'audit delle 50 righe, logical iteration 6–55, mostra un incremento unico e costante di `4608` per iterazione: da `27648` a `253440`. In tutte le righe `exact_start_balance.pass = true` e `learner_batch_pass = true`; ogni update contiene esattamente 1536 step per ciascuno dei tre start e `num_module_steps_trained = 4608`, cioè 4608 righe uniche compattate.

I contatori TensorBoard `episode_start_steps/<offset>` sono anch'essi lifetime e quindi crescono; le versioni `episode_start_steps_current/<offset>` vengono invece azzerate a ogni risultato e valgono sempre 1536. Il trainer usa inoltre il contatore lifetime come asse X di TensorBoard, scelta corretta per confrontare l'avanzamento in sample ma facile da scambiare per una dimensione variabile.

Il connector produce temporaneamente tra 4620 e 4633 righe post-GAE, quindi 12–25 righe tecniche in più; la compaction le rimuove prima dell'update e mantiene 4608 righe uniche. Anche queste righe non rappresentano nuovi environment.

Un ulteriore campo RLlib può risultare molto più grande: alla prima riga B0820 `learners/__all_modules__/num_env_steps_trained = 41580` e il corrispondente lifetime è già `249975`. Con una epoch e nove minibatch da 512, RLlib conteggia ripetutamente le circa 4620 righe pre-compaction durante le operazioni del learner (`9 x 4620 = 41580`). È quindi un contatore di **righe di ottimizzazione ripetute**, non di transizioni ambientali uniche. Per stabilire il batch realmente addestrato, il campo affidabile in questo contratto è `num_module_steps_trained = 4608`.

Sul tema restart, il supervisor registra `restart_count = 0`, `crash_restart_count = 0` e nessuna iterazione saltata; `faulthandler.log` non mostra crash o restart. Questo prova che il supervisore non ha riavviato il job, ma non dimostra in modo assoluto che nessun singolo EnvRunner sia stato sostituito: il codice abilita `restart_failed_env_runners = true` e `ignore_env_runner_failures = true`, e nella run B0820 manca un `env_runner_restart_audit.json` dedicato. Non esiste evidenza di replacement, ma l'osservabilità attuale non consente di escluderlo. Il prossimo run dovrà fallire o registrare esplicitamente ogni restart/replacement.

### 6.3 Correzione di osservabilità proposta

Prima del prossimo training saranno esposti insieme, in terminale, JSONL e TensorBoard:

- `num_env_runners_configured` e `num_env_runners_healthy`;
- `remote_sampling_env_count` e `local_inference_sync_runner_count`, senza sommarli in un'unica etichetta user-facing;
- `num_envs_per_env_runner`;
- `train_batch_size_target`;
- `env_steps_sampled_this_iter`;
- `env_steps_sampled_lifetime`;
- `learner_rows_pre_compaction`, `removed_rows` e `module_steps_trained`;
- step correnti e lifetime per ciascuno start;
- restart, replacement, worker id e generation degli EnvRunner, con artefatto `env_runner_restart_audit.json` obbligatorio anche quando i conteggi sono zero.

Le etichette user-facing distingueranno esplicitamente **environment**, **timestep unici per update**, **righe ripetute di ottimizzazione**, **episode** e **timestep cumulativi**. Un controllo fail-closed verificherà a ogni iterazione `delta sampled = 4608`, `current per start = 1536`, `module_steps_trained = 4608` e assenza di replacement non dichiarati. Questa è una modifica di logging/osservabilità, non del comportamento PPO, e sarà applicata solo dopo autorizzazione.

## 7. Artefatti e tracciabilità richiesti

Ogni fase produrrà:

- configurazione sorgente e `training_cfg.resolved.yaml`;
- manifest di feature actor/critic e normalizzazione;
- digest di actor, critic, `log_std`, optimizer e profilo morphology;
- receipt del dataset con provenienza, split e conteggi per categoria;
- audit save/reload e sincronizzazione EnvRunner;
- JSONL per iterazione con contatori current/lifetime distinti;
- rollout summary, trace complete e overlay phase-aligned;
- tabella di gate PASS/FAIL con motivazione;
- decisione di promozione o rollback, senza sovrascrivere artefatti precedenti.

## 8. Sequenza decisionale compatta

```text
freeze e ricampionamento baseline
        |
        v
ablation hard-drop / sigma
        |
        +-- nessun recupero --> riapertura diagnosi, STOP
        |
        v
bridge supervisionato 35D + DAgger target-domain
        |
        +-- gate cinematica/cicli FAIL --> rollback luglio, STOP
        |
        v
qualifica deterministica e stocastica
        |
        +-- FAIL --> nuovo round limitato o rollback
        |
        v
critic warmup bit-exact
        |
        +-- actor cambia --> STOP tecnico
        |
        v
pilot PPO breve con milestone
        |
        +-- nessuna milestone migliora H0 --> conserva H0
        |
        v
ablation morphology e solo dopo run esteso
```

## 9. Esito della revisione Claude

Claude è stato avviato da terminale con modello **Fable 5** ed effort **max**, in modalità read-only. Il verdetto iniziale è stato **“revise before approval”**: la direzione generale era corretta, ma alcune formulazioni avrebbero prodotto gate scientificamente errati o confronti non isometrici.

Le revisioni vincolanti incorporate sono:

- correzione della diagnosi cinematica: il knee B0820 non è sempre positivo, ma ha ROM collassata; il gate non usa più il segno globale della caviglia;
- riconoscimento dei sei segnali persi nel trapianto e sostituzione esplicita del clock prescritto con quello online;
- inizializzazione strutturale della testa log-std a sigma `0.005`, non semplice `detach()`;
- ripetizione del critic warmup nello stesso runtime v3 definitivo e audit per-start degli advantage;
- soglia KL minima oltre al tetto massimo, perché i 50 update B0820 sono stati quasi nulli;
- separazione fra policy controls storici e confronti realmente isometrici;
- distinzione delle unità morphology, del riferimento servito e dei flag `allow_effects`;
- audit dei 12 runner remoti più il runner locale e spiegazione separata dei contatori RLlib cumulativi/ripetuti;
- mantenimento sigillato dei seed 126–128 fino al gate finale.

Con queste correzioni, il piano è giudicato tecnicamente coerente come proposta da sottoporre all'autorizzazione dell'utente. La revisione non costituisce autorizzazione all'esecuzione.

## 10. TODO differito — Generalizzazione multi-modello con dataset EPIC

**Origine della decisione:** conversazione con l'utente del 22 agosto 2026, immediatamente precedente all'approvazione del presente piano. È stato chiarito che Teacher–Student e DAgger non sono intrinsecamente dipendenti da AB06, ma la pipeline corrente lo è statisticamente perché teacher, dinamica di raccolta, IK, GRF e Morphology Corridor provengono dallo stesso soggetto/modello. L'utente ha deciso di recuperare prima la qualità AB06 e di differire la generalizzazione.

**Dataset di riferimento:** EPIC, includendo i trial/soggetti disponibili e i relativi artefatti cinematici, dinamici, di contatto e metadata. Il lavoro futuro dovrà partire da un inventario di soggetti, task, velocità, completezza IK/ID/GRF, schema delle coordinate e compatibilità con i SEA, senza trattare più trial dello stesso soggetto come evidenza cross-subject.

TODO da propagare finché non completati:

- definire un observation/action contract canonico e adimensionale, indipendente dai nomi e dai range specifici AB06;
- censire i modelli e i trial EPIC utilizzabili, distinguendo train, validation e almeno un soggetto/modello leave-one-out sigillato;
- costruire teacher per soggetto/modello e un corpus DAgger multi-modello, con provenienza e bilanciamento dichiarati;
- decidere fra actor condizionato da descrittori fisici (massa, lunghezze, inerzie, velocità/task e parametri SEA) e policy universale con adattatore specifico del modello;
- sostituire il corridoio AB06 con un corridoio multi-soggetto normalizzato oppure con profili soggetto-specifici selezionabili senza cambiare i pesi della policy;
- preregistrare test zero-shot, leave-one-model-out e few-shot adaptation, separando compatibilità tecnica da generalizzazione scientifica;
- non dichiarare generalizzazione cross-subject, cross-task o cross-model prima del PASS su un modello EPIC mai usato da teacher, DAgger, PPO o selezione degli iperparametri.

**Artefatto TODO dedicato:** `reports/user/2026-08-22_todo_generalizzazione_multimodello_epic.md`.

Questo workstream è esplicitamente fuori dallo scope del recupero AB06 corrente e non deve introdurre confound nelle fasi F0–F7.

## 11. Condizione corrente

Il piano è autorizzato. L'esecuzione procede per fasi, con Claude Fable/max come esecutore operativo e Codex come architetto responsabile di monitoraggio, confronto tecnico e decisione sui gate. Nessuna fase successiva sarà promossa senza evidenza sufficiente del PASS della fase corrente.

## 12. Stato di esecuzione al 23 agosto 2026

La **Fase 0 è completata con PASS tecnico**. La matrice canonica contiene 58/58 job `PASS_ANALYSED`; gli artefatti finali di deriva actor e overlay hanno superato la verifica strict e il re-audit indipendente. Il report conclusivo è `reports/user/2026-08-23_fase0_gate_finale_recupero_ab06.md`.

L'esito comportamentale B0820 è **FAIL come baseline di recupero**: la regressione di forma nasce già nel transplant 39D→35D privo di refit target-domain e non viene recuperata dai 50 update PPO. La lineage del 15 luglio resta baseline di rollback.

Durante la chiusura dell'overlay sono state isolate due trace con `cycle_valid = 1` dopo una cancellazione HS senza nuova catena HS→TO→HS. I due job sono esclusi fail-closed dai prodotti phase-aligned e restano disponibili solo nel dominio temporale; il contratto runtime dovrà essere corretto o reso univoco prima dei futuri gate di fase.

Per istruzione dell'utente l'esecuzione si arresta dopo F0. **F1–F7, Teacher–Student/DAgger, critic warmup e nuovi training non sono iniziati** e richiedono un nuovo via esplicito.
