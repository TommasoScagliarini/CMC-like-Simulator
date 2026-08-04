# Piano revisionato — detector GRF a due sensori fino a CORRIDOR_TRAINING_READY

> Revisione: 2026-08-03  
> Piano precedente SHA-256: 19107fe9f5bca3ebbdebc80cc2c43a2ce252450182007528f614ca1a8f92053c  
> Gate storico: ERROR — CADENCE_DEPENDENT_ORACLE_TO_SEMANTICS_UNRESOLVED  
> Trial protetti aperti prima di questa revisione: nessuno

## Sintesi

Il problema da risolvere è il ritardo del detector, non la GRF primaria online.
La GRF primaria resta intoccata e mantiene il solo claim già validato: supporto
ibrido con GRF sinistra online e GRF destra prescritta.

La soluzione usa:

- detector esattamente a due sensori non fisici, una sfera tallone e una punta,
  entrambe con appliesForce=false;
- campionamento e debounce del detector a 1 ms, mantenendo policy e azioni a
  10 ms;
- un unico oracle eventi calcolato a 1 ms e riutilizzato a ogni cadenza;
- adattamento H0_sep soltanto se H0 non è compatibile con la semantica finale
  del detector;
- validazione protetta su trial 05 e 06;
- preflight corridor causale, senza avviare PPO con morphology reward positivo.

L'errore V5 resta documentato come storico: l'oracle era stato ricostruito
separatamente alle diverse cadenze, spostando alcuni TO fino a 51 ms. Non era
un errore della GRF primaria.

## Provenienza e invarianti

- Conservare V1–V5 e i relativi risultati come storia autorevole; non creare
  V6 per la GRF primaria.
- Non modificare online_grf.py, profilo, materiale o geometria della GRF
  primaria, legge di contatto o plugin C++.
- Non cambiare la semantica SEA.
- Escludere dal claim scientifico della GRF primaria la rappresentazione COP a
  carico nullo e le opzioni tecniche del loader.
- Mantenere separati:
  - GRF primaria: carico continuo e contatto fisico;
  - detector: soli eventi discreti;
  - GRF prescritta: solo oracle e lato destro ibrido.
- Ogni fallback, sample mancante, duplicato, non monotono o non finito deve
  chiudere il gate in fail-closed.

## Sequenza dei gate

### 1. PRIMARY_CONTRACT_FROZEN

Congelare in un manifest:

- profilo primario AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json,
  SHA 09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0;
- modello runtime,
  SHA 33e67d84bf11740eac509f620a143ad3c57d98c6f765d857e69c1892513de0c1;
- sorgente, header, interfaccia e CMake del contatto, SHA rispettivamente:
  - e0022a37268b60b51f68a588ceb7c951e0a8e37d1cd0e0016135a458a0cf919a;
  - 83890c056286b6cd3eee6852901228794813f5170d89db6ef4d3c3e64e43e272;
  - 5e87f884b023c749239b4484d54ea900862d0c11879d92941ef762aa6901ec99;
  - 8a2d71eb1535888d6d90c9e8977dae31756dc9947b420b9bf221f7b85cd86913;
- dylib macOS, come attestazione separata,
  SHA 5597a59a5368825fd207753b59291e72240c1ece3aa5280804f1e9b9c7d6a2b3.

Verificare esclusivamente routing e integrità: GRF sinistra online attiva,
destra online non applicata, destra prescritta presente, detector non applicato
e nessun fallback. Non riaprire la calibrazione fisica primaria.

### 2. CANONICAL_ORACLE_FROZEN

Generare una sola ledger eventi dalla GRF verticale prescritta sinistra sul
reticolo nativo/fine a 1 ms:

- contatto quando Fy > 20 N;
- HS al primo superamento di 20 N, accettato solo se il contatto successivo
  dura almeno 50 ms; il timestamp resta quello dell'onset;
- TO al primo campione Fy <= 20 N dopo uno stance accettato, senza dwell o
  isteresi di off;
- ciclo minimo 0,30 s;
- timestamp assoluti ed eventi di bordo espliciti.

La stessa ledger deve essere consumata sia dal validatore sequenziale a 1 ms
sia dal runtime raggruppato a 10 ms; è vietato riapplicare la soglia alla GRF
per ciascuna cadenza.

Creare ora soltanto gli oracle development per 02/04/08. Per 05 e 06 congelare
algoritmo e hash delle sorgenti, ma creare l'oracle all'interno della rispettiva
apertura one-shot.

### 3. TWO_SENSOR_HIGH_RATE_DEVELOPMENT_READY

Creare un solo candidato V17 ad alta frequenza, senza sweep:

- geometria V13 invariata;
- tallone [-0.0946600475, -0.0353195377, 0.01399567];
- punta [0.11574858501553537, -0.051879237903540584, 0.0030479021621026177];
- raggio comune 0.0229053623 m;
- soglia on 0,5 N, off 0,25 N;
- dwell 0,03 s;
- sample time detector 0,001 s;
- policy step 0,01 s.

V13 resta un comparatore diagnostico non promuovibile. Non ripetere V16,
ottimizzazione geometrica o tuning del dwell. Se V17 fallisce, terminare senza
fallback o retuning.

Eseguire 02/04/08 × quattro plateau × due modalità di consumo, per 24 unità e
sei aggregati:

1. campioni processati sequenzialmente a 1 ms;
2. gli stessi campioni raggruppati in batch da 10 ms.

Gate obbligatori:

- conteggi, ordine eventi e cicli esatti;
- precision e recall pari a 1;
- F1 >= 0,95 e IoU >= 0,90;
- latenza confermata HS <= 50 ms e TO <= 80 ms;
- latenza visibile alla policy HS <= 60 ms e TO <= 90 ms;
- 0 <= delivered_time_s - confirmed_time_s <= 10 ms;
- parità esatta tra processamento sequenziale e batch per latch, onset,
  conferme, ordine, cicli e transizioni accettate;
- toe-clear >= 30 ms e controllo di prossimità mesh V13 superato;
- zero eventi non validi, timeout, transizioni non accettate, stati
  proibiti/sconosciuti, trasferimenti incompleti, entrambi i latch off o TO
  anticipati;
- tutte le metriche finite.

### 4. H0_TWO_SENSOR_CANDIDATE_READY

Valutare:

- A: H0 con eventi legacy e corridor a peso zero;
- B: detector V17 in shadow e corridor a peso zero, bit-identico ad A;
- C: detector V17 attivo e corridor a peso zero.

Per il candidato finale eseguire tre partenze deterministiche e tre partenze
con seed 123–125, per 12 casi complessivi. Ogni caso deve completare 500/500
step, contenere almeno due cicli e rispettare:

- penetrazione < 25 mm;
- zero invalidità, timeout, clipping, safety stop o non-finiti;
- nessuna regressione SEA/reserve rispetto alla baseline ammessa.

Se H0 supera C, non eseguire adattamento. Se fallisce, consentire un solo
H0_sep:

- target semantico primary_grf_split_v1+two_sensor_highrate_v1;
- actor-only supervised adaptation dell'intera rete delle medie, ancorata
  all'actor H0 congelato;
- logstd congelata;
- critic e optimizer ricreati, senza restore;
- warm-up critic massimo tre iterazioni;
- holdout finale con seed 126–128;
- audit esplicito del restore;
- nessun actor prescritto e nessun morphology reward a runtime.

Un fallimento di H0_sep è terminale. La promozione nel registry resta sospesa
fino al superamento dei trial protetti del detector.

### 5. TWO_SENSOR_EVENT_READY

Congelare candidato, codice, configurazione, oracle algorithm e receipt prima
di aprire dati protetti.

- Trial 05: apertura validation one-shot.
- Solo in caso di PASS, trial 06: apertura sealed one-shot.
- Applicare gli stessi gate development.
- Vietare modifiche, rescue, retuning o riselezione tra 05 e 06.
- Un FAIL consuma lo stage e termina il piano.
- Trial 03 e 07 restano UNALLOCATED_RESERVE_CLOSED.
- Trial 01 resta esclusivamente diagnostica storica.

Dopo il PASS di 05 e 06, promuovere atomicamente detector V17 e H0/H0_sep
eleggibile, quindi eseguire restore/save-reload a zero update.

### 6. CORRIDOR_TRAINING_READY

Preparare il corridor in modalità causale event_anchored:

- ancorare i segmenti agli onset fisici event_time_s;
- introdurre un buffer morphology fisso di 40 ms, pari a dwell massimo più
  consegna alla policy;
- non ritardare osservazioni, pulse o azioni dell'actor;
- ritardare soltanto l'emissione del morphology reward;
- conservare a ogni policy step knee/ankle effettivamente serviti;
- calcolare la loss solo su campioni vecchi almeno 40 ms e usando eventi già
  confermati/consegnati;
- a fine episodio, emettere i campioni risolti; se esiste una transizione
  pendente, scartare soltanto i campioni dall'onset pendente in avanti e
  registrarli.

Usare morphology profile
SHA 33b1dd7cb0db40110a4f9c1b8c0dd49a498662211e6e132f0f3cefe8edc02a55,
alpha=0.6223299989 e creare una configurazione candidata con
morphology_weight=0.05. La configurazione attiva rimane a peso zero e non viene
avviato alcun PPO con peso positivo.

Gate finali:

- peso zero bit-identico alla baseline;
- somma reward ritardata uguale all'oracle causale offline sui campioni
  risolti;
- replay a azioni congelate con peso positivo: cambiano solo reward e
  diagnostica, non azioni o dinamica;
- trainer zero-update con save/reload deterministico;
- nessun update PPO.

L'output terminale del piano è CORRIDOR_TRAINING_READY, non un modello
addestrato.

## Interfacce e compatibilità

- SimulationRunner.step_until() aggiunge a info:
  phase_sensor_samples = [{time_s, left_heel_normal_n, left_toe_normal_n}].
- Al reset viene campionato t0 senza attribuzione di eventi. Ogni step
  restituisce solo i campioni unici in (previous_sample_time, t_stop], inclusivo
  di t_stop: dieci campioni per un normale step da 10 ms.
- Il FSM aggiunge un'API batch per policy step: reset dei transient una sola
  volta, processamento sequenziale dei dieci campioni, aggregazione e consegna
  una sola volta.
- L'API scalare legacy resta compatibile.
- Primary load/contact e cinematica continua restano aggiornati alla cadenza
  policy esistente; il riferimento di parità deve replicare questa stessa
  semantica e variare soltanto la cadenza heel/toe.
- Ogni evento espone:
  - event_time_s: onset fisico;
  - confirmed_time_s: conferma dopo dwell;
  - delivered_time_s: boundary policy che riceve l'evento.
- Configurazione comune:
  - detector_sample_dt_s=0.001;
  - event_contract_id=primary_grf_split_v1+two_sensor_highrate_v1;
  - dopo la promozione, morphology_reward_delay_s=0.04.
- Forma e ordine delle osservazioni restano invariati, inclusi i layout 35/84.
- Codice e path devono funzionare su macOS arm64 e Windows x86. Il claim
  numerico resta macOS-only finché DLL e parità Windows non sono attestati.

## Controlli operativi

- Receipt, manifest e JSON devono essere atomici, no-clobber, strict JSON e
  privi di NaN/Inf.
- Conservare audit globale degli accessi ai dati e verifica indipendente dei
  gate.
- Il default resta legacy_events con morphology_weight=0 fino alla promozione
  atomica.
- Nessuna fase può utilizzare 03/07, modificare la GRF primaria o trasformare
  un FAIL in un nuovo ciclo di tuning.
