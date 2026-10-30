# Daily Report - 2026-08-25

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../cleanup/2026-10-19_baseline_validation_removal/README.md).

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Ricostruzione storica delle architetture e dei tentativi di widening](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md)
- [A0/A1 — contratto 25D e preflight](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md)
- [B0/B1 — esecuzione e NO-GO](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md)
- [B1R1 LOCO — fit e NO-GO](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md)
- [B1R1 LOCO — studio corretto](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md)
- [B1R1 — studio read-only degli split](../user/2026-08-25_v26b_b1r1_studio_split_readonly.md)
- [B1R2 — diagnostica](../user/2026-08-25_v26b_b1r2_diagnostica.md)
- [B1R2 — diagnostica rev1](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md)
- [B1R2-A — revisione indipendente e proposta](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md)
- [B1R2-B — addendum correttivo e screening](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md)
- [B1R2-B — NO-GO, ipotesi falsificata](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md)
- [Fase A — diagnostica read-only](../user/2026-08-25_v26b_fasea_diagnostica_readonly.md)
- [Fase A — rettifica e protocollo 25D](../user/2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md)
- [Ramo 35D masked — protocollo e preflight](../user/2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md)

## Sintesi

La ricostruzione storica ha corretto il percorso operativo. Le reti riuscite tra giugno e agosto mantenevano la stessa capacità — **due hidden layer tanh da 256**, con larghezze d'ingresso cambiate 31→17→39→35 — e non usavano un adapter appreso. I tentativi di widening a 512/1024 avevano superato verifiche offline ma erano falliti closed-loop per penetrazione; non vi è quindi evidenza a favore di una rete più grande.

È stato inoltre chiarito che il riferimento «training del 15 luglio» non identifica un checkpoint ex-novo promosso: quel giorno fu eseguito un pilot PPO sul lineage luglio, mentre la qualità utile proveniva dal bridge supervisionato precedente. Luglio deve essere usato come **fonte metodologica**, non come parent operativo. Il parent corrente resta esclusivamente il migliore training imitativo V26 di agosto.

I rami A0/A1, B0/B1, LOTO/LOCO e B1R1/B1R2 hanno prodotto studi, fit e revisioni, ma nessun candidato promuovibile. Il passaggio a un actor 25D separato è stato ritirato: la decisione corretta è un **solo actor 35D** con le colonne non osservabili mascherate/azzerate nello stadio base e riabilitate nello stadio Markov. Le ipotesi su split, patience, budget e learning rate esplorate nei rami B1 sono state insufficienti o falsificate.

## Lavoro svolto

- Ricostruite da codice e artefatti le architetture storiche e i fallimenti dei tentativi di allargamento.
- Progettati ed eseguiti i rami diagnostici 25D/35D masked e gli studi LOTO/LOCO/B1R1/B1R2.
- Corrette conclusioni intermedie su split, teste/coda, leakage e limite informativo.
- Stabilito che il prossimo percorso deve replicare l'ordine BASE→MARKOV di luglio sul parent agosto, senza pesi, dataset o trace luglio.
- Nessun PPO, critic warm-up o training ex-novo lanciato.

## Test e verifiche

- Verifica degli shape e delle architetture storiche direttamente nei checkpoint/artefatti.
- Controlli read-only e fit preregistrati dei rami B0/B1; tutti i NO-GO conservati senza promozione.
- Verifiche di immutabilità/no-clobber sui receipt e sui module state dei tentativi esplorativi.

## TODO completati

- [x] Escludere il widening come rimedio supportato dall'evidenza.
- [x] Ritirare l'actor intermedio 25D come artefatto separato.
- [x] Stabilire il parent operativo: imitation V26 agosto; luglio solo metodo.
- [x] Conservare LOTO, LOCO, B1R1 e B1R2 come rami futuri, non integrarli nella baseline senza approvazione dell'utente.

## TODO aperti e propagati

- [ ] Preregistrare una replica fedele BASE→MARKOV di luglio usando un unico actor 35D e il parent V26 agosto.
- [ ] Misurare sigma nel caso corrente prima delle recovery; 0,005 non va assunto.
- [ ] Risolvere il gate di forma caviglia/plantarflessione e la copertura delle fasi closed-loop senza sacrificare la preservazione nominale.
- [ ] Monitorare la penetrazione rispetto alle bande 20/25/28 mm e la copertura swing/WAIT_HS.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri e richiedono approvazione esplicita dell'utente prima di ogni integrazione.
- [ ] Generalizzazione multimodello EPIC, differita.
- [ ] Semantica `cycle_valid` dopo HS cancellato, osservabilità RLlib e TODO tecnici/amministrativi storici ancora aperti.


## Addendum retrospettivo — integrazione del 5 ottobre 2026

Questa integrazione completa il consolidamento documentale dei report utente della giornata. Riporta soltanto esperimenti, audit e verifiche già descritti nelle fonti; il 5 ottobre non sono stati eseguiti nuovi fit, training, rollout o test del simulatore. Le rettifiche successive sono dichiarate come tali e non promuovono retroattivamente candidati respinti.

<a id="integrazione-2026-10-05-2026-08-25-ricostruzione-storica-architetture-actor-e-tentativi-rete-larga"></a>

### Ricostruzione storica: architetture actor giugno–luglio, adapter, e ogni tentativo di rete più larga

Verifica di copertura: architetture 256×2 tanh, ingressi 31→17→39→35, assenza adapter e fallimenti closed-loop del widening sono già consolidati con controllo diretto degli artefatti. Il verdetto storico sul widening non prova che un fit imitativo V26B da zero più largo sia impossibile: quel caso non è stato testato.

Fonte: [2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-a0a1-protocollo-contratto25d-preflight"></a>

### V26B A0/A1 — protocollo, contratto 25D, tooling e preflight

A0/A1 preflight 25D ottiene A0 GO ma A1 BLOCKED: manca la raccolta teacher sotto quel contratto e i dati visitati da S0D sono esclusi per provenance. Il nuovo config sposta memoria controller nel critic, preservando fisica/FSM/guardie; il trapianto compensa 14 colonne rimosse e il fit sarebbe full mean con split bloccato. I test 103/103 verificano invarianti, gate e rifiuto fail-closed. Nessun fit o raccolta è stato eseguito; il ramo viene poi superseduto prima di esecuzione dal 35D masked.

File o artefatti pertinenti documentati: `v26b_amendment_a0a1_25d.json`, `v26b_contract_25d_v1.yaml`.

Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b0-b1-esecuzione-esito-nogo"></a>

### V26B — esecuzione B0/B1 nel ramo 35D-masked: **NO-GO offline**

B0 viene materializzato e B1 esegue fit LOTO da B0 byte-identico su tre anchor; maschera, clock zero, logstd e equivalenza 25D passano. Cinque gate di qualità su otto falliscono: NO-GO, nessun rollout o correzione post-hoc. Minus 020 domina l'errore, con 47,4% fuori supporto e WAIT_HS assente dalle altre due anchor: è estrapolazione di uno stato discreto mai visto, non distribuzione diversa delle label. Pin del protocollo, confronto differenziale della loss e suite di immutabilità verificano il risultato.

File o artefatti pertinenti documentati: `v26b_july_imitation_protocol_pin.json`, `v26b_b0_receipt.json`.

Fonte: [2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r1-loco-fit-esito-nogo"></a>

### V26B B1R1 — fit LOCO a 6 fold: **NO-GO offline**, ma il difetto è cambiato natura

B1R1 LOCO a sei fold passa generalizzazione 6/6 ma resta NO-GO per ricostruzione e pooled validation. La selezione pooled usa lo stesso e*=358 per tutti i fold, senza best individuali; il fit finale riparte da B0. Le 590 righe mai validate dominano l'errore e contengono tutte 11 le violazioni knee>0,15. Test dimostrano riproduzione bit-exact delle traiettorie di training e invarianti della maschera. La patience arriva solo a 42 stale su 60; le successive revisioni distinguono teste strutturalmente complete dalle 95 code, che invece ricostruiscono bene.

File o artefatti pertinenti documentati: `v26b_b1r1_exec.py`, `v26b_b1r1_receipt.json`.

Fonte: [2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r1-loco-studio-corretto"></a>

### V26B B1R1 — studio LOCO corretto: 6 fold leave-one-complete-cycle-out

Lo studio LOCO corregge supporto 23D esplicito, scaler fit solo sul train e firme discrete esatte, incluse le quattro classi di credit. WAIT_HS è un unico segmento 187 righe senza ricorrenza: un heldout è leakage, estrapolazione o stato fuori distribuzione; non esiste generalizzazione WAIT in distribution in questo corpus. LOCO valida 910/1500 righe e lascia 590 fuori; il gate WAIT è ricostruzione, mai heldout. Test 87/87 e receipt verificano geometria/embargo; perdere 20 righe WAIT nel fold 0 è il costo dichiarato poi accettato dall'architetto.

File o artefatti pertinenti documentati: `v26b_b1r1_loco_study.json`, `v26b_addendum_b1r1_blocked5_comparator.json`.

Fonte: [2026-08-25_v26b_b1r1_loco_studio_corretto.md](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r1-studio-split-readonly"></a>

### V26B B1R1 — studio read-only dello split. Causa del NO-GO B1 e proposta

Lo studio split identifica il difetto LOTO: minus 020 contiene l'unico WAIT_HS,187 righe, mentre il train del suo fold non ne contiene nessuna. Il random 80/20 offre quasi duplicati temporali,97,7% con vicino adiacente, e non misura generalizzazione. Propone blocked 5 con embargo 20 e pooled-fold early stopping, mantenendo gate numerici; la baseline kNN elevata non è un limite inferiore della rete. Test 81/81 e receipt verificano copertura e assenza training. Il progetto verrà superseduto da LOCO dopo correzioni della misura, senza cancellare la diagnosi.

File o artefatti pertinenti documentati: `v26b_b1r1_split_study.json`, `v26b_b1r1_split_study.py`.

Fonte: [2026-08-25_v26b_b1r1_studio_split_readonly.md](../user/2026-08-25_v26b_b1r1_studio_split_readonly.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r2-diagnostica"></a>

### V26B B1R2 — studio diagnostico: il difetto è di ottimizzazione, non di informazione

B1R2 iniziale esclude difetti di label/action, anchor e saturazione, ma conclude ottimizzazione non assestata usando R² in-sample 0,9021 sui gruppi piccoli. Il fit strumentato riproduce bit-exact quello pinnato e nessuna epoca 400 passa la ricostruzione completa; propone copertura 11 fold. La conclusione positiva sul residuo recuperabile è però ritirata da rev 1: in-sample su righe contigue non è una prova di informazione o capacità fuori campione. Restano validi i controlli label e invarianti, non quella causa né il relativo TODO sul 90,2%.

File o artefatti pertinenti documentati: `v26b_b1r2_diagnostics.json`, `v26b_b1r2_diagnostics.py`.

Fonte: [2026-08-25_v26b_b1r2_diagnostica.md](../user/2026-08-25_v26b_b1r2_diagnostica.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r2-diagnostica-rev1"></a>

### V26B B1R2 rev1 — revisione architettonica obbligatoria della diagnostica

B1R2 rev 1 rovescia la diagnosi con quattro correzioni: vicini lontani sono sparsità/smoothness, non collisioni; audit esaustivo 23D trova zero coppie entro 1 e−3; ridge con fold/embargo/scaler train-only dà R² fuori campione≤0,16; le teste sono cicli completi, mentre 95 code passano la ricostruzione. Test e nuova receipt preservano gli artefatti precedenti e riconfermano label, anchor e saturazione. L'errore è localizzato nelle 308 righe di testa ma la causa non è identificata; il transitorio SEA/FSM resta ipotesi non verificata.

File o artefatti pertinenti documentati: `v26b_b1r2_diagnostics.py`, `v26b_b1r2_diagnostics_rev1.json`.

Fonte: [2026-08-25_v26b_b1r2_diagnostica_rev1.md](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r2a-revisione-indipendente-e-proposta"></a>

### V26B B1R2-A — revisione indipendente, esito NO-GO e proposta del prossimo esperimento

B1R2-A usa 11 fold e budget 1200: NO-GO su sei gate. Patience seleziona 237, pur con minimo globale 1171, ma nemmeno l'oracle a epoca singola 0,04120 o per-fold 0,03599 raggiunge pooled 0,03: selection e budget da soli non chiudono il problema. Le teste producono 41,8% della MSE e confliggono localmente con ciclo 1, senza collisioni esatte; non è dimostrata non-identificabilità. Audit indipendente di receipt, curve e hash propone solo screening lr 3 e−4→1 e−4. La validation luglio su copie 32× non è un bersaglio di generalizzazione confrontabile.

File o artefatti pertinenti documentati: `module_state.pkl`.

Fonte: [2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r2b-addendum-correttivo-screening"></a>

### V26B B1R2-B — addendum correttivo: tre rettifiche vincolanti prima dell'esecuzione

L'addendum B1R2-B corregge tre errori logici prima del fit: il conflitto tra label del dataset è invariante al learning rate, dunque controllo integrità e non falsificatore; un NO-GO non dimostra limite del contratto osservativo; esaurire patience di selection non significa convergenza. Convergenza al budget richiede argmin globale≤1140 e 60 epoche successive senza miglioramento. Il confronto di lr default/esplicito verifica quattro epoche dello stesso fold, non l'intera curva del braccio A;673 check di regressione restano PASS. È una preregistrazione correttiva, senza esecuzione.

Fonte: [2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-b1r2b-esito-nogo-ipotesi-falsificata"></a>

### V26B B1R2-B — esito: **NO-GO** e **ipotesi falsificata**, con curva convergente

B1R2-B a lr 1 e−4 converge secondo la regola completa, argmin 1093≤1140, ma fallisce sei gate e entrambi falsificatori: le teste restano>.05 al proprio best e pooled selezionato 0,05204 non batte l'oracle A. Il rumore scende 44%→31,4%, mentre generalizzazione e ricostruzione peggiorano. Valori del conflitto dataset coincidono esattamente,791 check passano, artefatti restano diagnostici non deployable. È falsificata questa via dell'ottimizzazione più fine, non dimostrato un limite del contratto; budget extra non è più giustificato e nessun braccioC parte.

File o artefatti pertinenti documentati: `v26b_b1r2b_receipt.json`, `v26b_amendment_b1r2b_lr.json`.

Fonte: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-fasea-diagnostica-readonly"></a>

### Fase A — diagnostica read-only del DAgger V26B

Fase A verifica la catena provenance e distingue ultimo artefatto pulito cronologico L20 da unico init autorizzato S1A. Il confronto teacher-versus-student trova mismatch 14/62 e prefix≤61 tutto stance; la misura di eseguibilità passa aggregata ma swing Rev 4E fallisce da solo 13,5%, limite inferiore basato sul slew. Test 827/827 e confronti differenziali verificano aritmetica e membership cross-phase. L'interpretazione di quei prefix come limite del troncamento storico è errata e sarà ritirata: i numeri restano solo controfattuale teacher/student, non prova che luglio non potesse correggere swing.

File o artefatti pertinenti documentati: `v26b_fasea_diagnostics.py`, `v26b_fasea_receipt.json`.

Fonte: [2026-08-25_v26b_fasea_diagnostica_readonly.md](../user/2026-08-25_v26b_fasea_diagnostica_readonly.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-fasea-supplemento-rettifica-e-protocollo-25d"></a>

### Supplemento additivo alla Fase A — rettifica metodologica e protocollo 25D → 35D

Il supplemento Fase A corregge il call site: troncamento storico confronta recovery e nominale dello stesso attore, non teacher/student; ritira la conclusione sullo swing e TODO-8. La ricostruzione di luglio impone BASE senza proxy prima di MARKOV con anchor self-mean, recovery allineate e teacher multistart. Agosto eredita già 71,25% del contributo del primo layer sul canale controller; l'ablazione diretta degrada molto e le 25 colonne residue non sono addestrate a sostituirlo. Misure open-loop e codice sostengono la raccomandazione di riaddestrare la base, senza provare causalità unica dei collassi. Il progetto 25D separato verrà poi ritirato.

File o artefatti pertinenti documentati: `target_domain_markov_adaptation.py`, `target_domain_noise_adaptation.py`.

Fonte: [2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md](../user/2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md).

<a id="integrazione-2026-10-05-2026-08-25-v26b-ramo-35d-masked-protocollo-e-preflight"></a>

### V26B — ramo 35D-masked: quarantena del 25D, protocollo, tooling e preflight

Il 35D masked supersede A0/A1 prima di esecuzione, chiudendo il secondo contratto/pin di runtime. Due protezioni indipendenti sono verificate: input delle dieci colonne a zero e colonneW1 hard-zero/ripristinate dopo optimizer; unmask con colonne sporche fallisce. Equivalenza funzionale 25D e transizione unmask sono bit-exact su dati reali/sintetici, con controllo negativo sul gradiente. B0/B1 preflight GO85/85, dataset nativo già disponibile e nessun fit in questo stadio. Solo dopo base qualificata viene riabilitata memoria nello stesso actor; logstd/critic restano congelati e sigma va misurata.

File o artefatti pertinenti documentati: `v26b_amendment_b_masked35.json`, `v26b_addendum_a0a1_superseded.json`.

Fonte: [2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md](../user/2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md).

## TODO delle fonti — stato documentato al 5 ottobre 2026

Le chiusure sotto riportate riguardano lo scope indicato e non modificano il verdict degli esperimenti precedenti. Le voci condizionate sono rami o confronti futuri: richiedono approvazione e non costituiscono autorizzazione a eseguire. Conservazione/no-clobber dei receipt storici rimangono invarianti.

- [x] **Risolto** — Misurare sigma sul lineage corrente prima delle recovery, senza ereditare il placeholder 0,005. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga134. Qualificato circa0,005 con seed123–125 per la fase V26C; nuove policy o nuovi contesti richiedono una nuova misura. Evidenza: [2026-08-27_daily-report.md](./2026-08-27_daily-report.md), riga101.

- [ ] **Aperto** — Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga135. Nessuna chiusura esplicita distinta trovata; il problema del campo non coincide con cycle_valid e non è chiuso da successi di altri lineage. Evidenza: [2026-08-24_v26b_s0d_rollout_b3_semantica_supplemento.md](../user/2026-08-24_v26b_s0d_rollout_b3_semantica_supplemento.md), riga3.

- [x] **Risolto** — Ottenere cammino completo con almeno due cicli e plantarflessione, preservando gli start richiesti. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga136. J19B6/6 e J19C3/3 con forme adeguate chiudono il traguardo nel successore V26C; i candidati precedenti restano respinti e le bande di penetrazione hanno autorità propria. Evidenza: [2026-08-27_daily-report.md](./2026-08-27_daily-report.md), riga72.

- [ ] **Condizionato** — Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga137. Non risolto da Rev4E multivariato; nessun nuovo round è autorizzato da questo TODO. Riguarda la riapertura del ramo storico. Evidenza: [2026-08-24_v26b_rev4e_r2_replay.md](../user/2026-08-24_v26b_rev4e_r2_replay.md), riga201.

- [ ] **Condizionato** — Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga138. Il fail del candidato resta immutabile; la qualifica successiva J19 non corregge Rev4E. Il monitoraggio20/25/28mm resta un obbligo corrente distinto. Evidenza: [2026-08-24_v26b_rev4e_r2_replay.md](../user/2026-08-24_v26b_rev4e_r2_replay.md), riga203.

- [ ] **Condizionato** — Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga139. Misura riferita all'ammissione delle vecchie trace; non è una nuova autorizzazione a raccogliere o rifittare il ramo. Evidenza: [2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md](../user/2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md), riga254.

- [x] **Risolto** — Rimuovere il proxy controller ereditato durante lo stadio BASE senza rompere la transizione al futuro MARKOV. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga140. Mascheramento e colonne hard-zero verificati in B0/B1; chiusura limitata alla BASE. Non prova assenza di feedback autoregressivo in ogni actor successivo. Evidenza: [2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md), riga151.

- [x] **Risolto** — Eliminare il secondo contratto/pin di runtime del ramo 25D separato. Fonte: [2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md), riga141. Contratto unico35D chiude esplicitamente TODO-11; A0/A1 superseduto prima esecuzione. Evidenza: [2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md](../user/2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md), riga153.

- [x] **Risolto** — Identificare e trattare l'assenza WAIT_HS nel train del fold minus 020 sotto LOTO. Fonte: [2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md), riga152. Causa identificata e chiusa in progetto con copertura di stato; non trasforma il precedente LOTO FAIL in PASS. Evidenza: [2026-08-25_v26b_b1r1_studio_split_readonly.md](../user/2026-08-25_v26b_b1r1_studio_split_readonly.md), riga149.

- [x] **Risolto** — Coprire le righe non-WAIT mai validate dal LOCO a soli cicli completi. Fonte: [2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md), riga122. Il disegno11fold copre tutte1313 righe non-WAIT; questo risolve la copertura, non l'errore localizzato nelle teste. Evidenza: [2026-08-25_v26b_b1r2_diagnostica_rev1.md](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md), riga83.

- [x] **Risolto** — Decidere sulla perdita di 20 righe WAIT nel train del fold 0 per embargo. Fonte: [2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md), riga123. Costo esplicitamente accettato dall'architetto; fit finale usa tutte187 righe. Evidenza: [2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md), riga123.

- [ ] **Condizionato** — Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. Fonte: [2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md), riga124. Oracle fallisce entrambi i bracci: selection non causale. La revisione del ramo resta condizionata a riapertura approvata. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga148.

- [x] **Risolto** — Decidere sul gate 0,05 per fold rispetto alla baseline dello split bloccato candidato. Fonte: [2026-08-25_v26b_b1r1_loco_studio_corretto.md](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md), riga121. TODO-13 chiuso dal nuovo LOCO, baseline peggiore0,064 contro0,20 del design superseduto. Evidenza: [2026-08-25_v26b_b1r1_loco_studio_corretto.md](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md), riga121.

- [ ] **Condizionato** — Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. Fonte: [2026-08-25_v26b_b1r2_diagnostica.md](../user/2026-08-25_v26b_b1r2_diagnostica.md), riga167. La sola via lr più fine è falsificata; altre cause rappresentative o osservative restano aperte e richiedono autorizzazione. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga149.

- [x] **Superato** — Ritirare la conclusione che il 90,2% del residuo dimostri informazione/capacità disponibile fuori campione. Fonte: [2026-08-25_v26b_b1r2_diagnostica.md](../user/2026-08-25_v26b_b1r2_diagnostica.md), riga168. La rettifica cross-validated identifica overfitting dei valori0,9021/0,9996. Evidenza: [2026-08-25_v26b_b1r2_diagnostica_rev1.md](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md), riga19.

- [ ] **Condizionato** — Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. Fonte: [2026-08-25_v26b_b1r2_diagnostica_rev1.md](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md), riga185. Le due teste restano FAIL al proprio best in entrambi bracci; nuovi studi del ramo richiedono riapertura approvata. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga147.

- [ ] **Condizionato** — Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. Fonte: [2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md), riga198. Migliorato0,18824→0,16238 ma ancora FAIL; nessuna chiusura da actor di altro lineage. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga150.

- [ ] **Condizionato** — Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. Fonte: [2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md), riga199. È un vincolo metodologico permanente per eventuali confronti, non lavoro da rieseguire. Evidenza: [2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md), riga146.

- [x] **Superato** — Richiedere un braccio a budget adeguato solo se lo screening lr ridotto non converge. Fonte: [2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md), riga111. Chiuso: il braccio B converge; budget extra non giustificato da questo esito. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga152.

- [ ] **Condizionato** — Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. Fonte: [2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md), riga112. Vincolo per futuri confronti B1; nessuna nuova esecuzione richiesta per riscrivere artefatti storici. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga153.

- [ ] **Condizionato** — Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. Fonte: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga154. Reperto diagnostico non risolto; ramo storico non operativo. Evidenza: [2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md), riga154.

- [x] **Superato** — Ritirare la conclusione che il troncamento di luglio impedisca copertura swing: il confronto corretto è same-actor. Fonte: [2026-08-25_v26b_fasea_diagnostica_readonly.md](../user/2026-08-25_v26b_fasea_diagnostica_readonly.md), riga123. TODO-8 esplicitamente ritirato perché fondato su un confronto improprio teacher/student. Evidenza: [2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md](../user/2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md), riga253.

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).
- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).
- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).
- [ ] **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Completare il milestone di recupero AB06 qualificato e attestato come training-input-only. La ricostruzione J19–J21 sostituisce la vecchia proposta F0/F1–F7, senza dichiarare eseguite tutte quelle fasi né il benchmark generale di parità. — [fonte 2026-08-23, riga 200](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Misurare sigma sul lineage corrente prima delle recovery, senza ereditare il placeholder 0,005. — [fonte 2026-08-24, riga 211](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Ottenere cammino completo con almeno due cicli e plantarflessione, preservando gli start richiesti. — [fonte 2026-08-24, riga 213](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](../user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).
- [ ] **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](../user/2026-08-24_v26b_rev4e_r2_replay.md).
- [ ] **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).
- [ ] **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).
- [ ] **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](../user/2026-08-25_v26b_b1r2_diagnostica.md).
- [ ] **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).
- [ ] **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).
- [ ] **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).
- [ ] **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Riferimento storico al TODO giugno sulla scelta warm-start: luglio ha usato il 23 giugno grfsoft, non il 24 giugno. — [fonte 2026-08-25, riga 40](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md). **Chiuso:** Ricostruzione del25agosto documenta la sorgente effettivamente usata a luglio; non introduce una nuova selezione. ([evidenza](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md)).
- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Rimuovere il proxy controller ereditato durante lo stadio BASE senza rompere la transizione al futuro MARKOV. — [fonte 2026-08-25, riga 140](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md). **Chiuso:** Mascheramento e colonne hard-zero verificati in B0/B1; chiusura limitata alla BASE. Non prova assenza di feedback autoregressivo in ogni actor successivo. ([evidenza](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md)).
- [x] Decidere sul gate 0,05 per fold rispetto alla baseline dello split bloccato candidato. — [fonte 2026-08-25, riga 121](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md). **Chiuso:** TODO-13 chiuso dal nuovo LOCO, baseline peggiore0,064 contro0,20 del design superseduto. ([evidenza](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md)).
- [x] Ritirare la conclusione che il 90,2% del residuo dimostri informazione/capacità disponibile fuori campione. — [fonte 2026-08-25, riga 168](../user/2026-08-25_v26b_b1r2_diagnostica.md). **Superato:** La rettifica cross-validated identifica overfitting dei valori0,9021/0,9996. ([evidenza](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md)).
- [x] Coprire le righe non-WAIT mai validate dal LOCO a soli cicli completi. — [fonte 2026-08-25, riga 122](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md). **Chiuso:** Il disegno11fold copre tutte1313 righe non-WAIT; questo risolve la copertura, non l'errore localizzato nelle teste. ([evidenza](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md)).
- [x] Richiedere un braccio a budget adeguato solo se lo screening lr ridotto non converge. — [fonte 2026-08-25, riga 111](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md). **Superato:** Chiuso: il braccio B converge; budget extra non giustificato da questo esito. ([evidenza](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md)).
- [x] Identificare e trattare l'assenza WAIT_HS nel train del fold minus 020 sotto LOTO. — [fonte 2026-08-25, riga 152](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md). **Chiuso:** Causa identificata e chiusa in progetto con copertura di stato; non trasforma il precedente LOTO FAIL in PASS. ([evidenza](../user/2026-08-25_v26b_b1r1_studio_split_readonly.md)).
- [x] Eliminare il secondo contratto/pin di runtime del ramo 25D separato. — [fonte 2026-08-25, riga 141](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md). **Chiuso:** Contratto unico35D chiude esplicitamente TODO-11; A0/A1 superseduto prima esecuzione. ([evidenza](../user/2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md)).
- [x] Decidere sulla perdita di 20 righe WAIT nel train del fold 0 per embargo. — [fonte 2026-08-25, riga 123](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md). **Chiuso:** Costo esplicitamente accettato dall'architetto; fit finale usa tutte187 righe. ([evidenza](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md)).
- [x] Ritirare la conclusione che il troncamento di luglio impedisca copertura swing: il confronto corretto è same-actor. — [fonte 2026-08-25, riga 123](../user/2026-08-25_v26b_fasea_diagnostica_readonly.md). **Superato:** TODO-8 esplicitamente ritirato perché fondato su un confronto improprio teacher/student. ([evidenza](../user/2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
