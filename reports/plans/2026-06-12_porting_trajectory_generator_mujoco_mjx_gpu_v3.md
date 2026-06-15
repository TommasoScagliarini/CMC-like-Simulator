# Porting Trajectory Generator su MuJoCo/MJX GPU - piano v3

> Revisione della v2 approvata. Mantiene Fase 0 bloccante, milestone engine
> prima del training, gate di parita separati e riuso di un trainer JAX
> esistente. Corregge la dinamica PI SEA, usa uno spike verticale riusabile,
> separa replay e GRF ibrida e rende quantitativi i gate di osservazione,
> reward, contatto, causalita scientifica e prestazioni. Introduce un
> checkpoint strategico prima dell'investimento completo nell'engine.

## Obiettivo e perimetro

Usare OpenSim come **oracolo scientifico** e `MuJoCo_env` come backend di
produzione parallelizzato sulla RTX 4070 tramite WSL2/JAX/MJX, preservando la
dinamica e le scelte strutturali della Trajectory Generator.

Il porting complessivo termina quando training PPO e rollout/inference MLP
girano sul backend MJX. La prima milestone consegnabile, **v1-engine**, comprende
engine MJX batched, parita e benchmark; non viene confusa con il completamento
del porting.

Dentro scope:

- stato attualmente implementato di `baseline_MLP`;
- modello e setup `AB06_SEASEA_stiff321_500_pi`;
- ambiente CMC-like, reward, osservazioni, PPO MLP e rollout.

Fuori scope:

- `Prosthesis_SNN`;
- TODO funzionali ancora aperti della Trajectory Generator;
- conversione o resume dei checkpoint RLlib esistenti.

## Fonti di verita

- **Semantica scientifica e di controllo**: setup, modello, plugin C++ e
  Trajectory Generator della repo OpenSim.
- **Backend di produzione**: `MuJoCo_env`.
- Ogni run MuJoCo deve salvare un manifest con hash delle sorgenti OpenSim,
  configurazione risolta, versione MuJoCo/JAX e precisione numerica.

La parita precedente di `MuJoCo_env` non e valida per l'obiettivo RL corrente:
era basata su `Adjusted_SEASEA - Copia_tuned.osim`, stiffness `1000/700`, mentre
`baseline_MLP` usa `AB06_SEASEA_stiff321_500_pi.osim`, stiffness `321/500`.
La validazione deve quindi ripartire dal nuovo modello.

### Dinamica PI da preservare

Esistono **due integratori distinti**, entrambi obbligatori:

1. **PI esterno cascade Python**: integra l'errore di velocita e contribuisce al
   comando normalizzato `u`, con limite, feasibility scaling e anti-windup.
2. **PI interno del plugin SEA**: integra
   `tau_ref - tau_spring` nello stato `torque_error_integral` e contribuisce a:

   ```text
   tau_input_raw =
       tau_ref
       + Kp * (tau_ref - tau_spring)
       + clamp(Ki * torque_error_integral, integral_torque_limit)
       - Kd * omega_m
   ```

   L'integrale interno si arresta quando il suo contributo o `tau_input_raw`
   saturano nella direzione dell'errore.

I due loop devono avere stato, reset, diagnostica e test separati. Nel percorso
JAX, il carry interno di ciascun SEA contiene esplicitamente
`(theta_m, omega_m, torque_error_integral)`. La derivata dell'integrale e
l'anti-windup data-dependent vengono valutati a ogni stadio/substep del metodo
di integrazione tramite `jnp.where`/`lax.cond`, mai tramite branch Python.

---

## Prerequisito - Oracle OpenSim canonico 321/500_pi

Prima della Fase 1 e di qualsiasi gate di parita, generare e congelare la
reference OpenSim contro cui saranno eseguiti i gate. La Fase 0 puo precedere
questo lavoro per evitare di produrre un oracle completo se la fattibilita GPU
riceve NO-GO.

- setup canonico:
  `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`;
- finestra completa del setup: `t_start=11.99 s`, `t_end=21.00 s`;
- finestra RL primaria deterministica: episodio da `2.0 s` a partire da
  `t=11.99 s`;
- suite di azioni scriptate condivisa: comando costante, rampa, target imitativo
  governato e stress controllato dei limiti;
- output STO completi per stati, controlli, entrambi i PI, SEA, `tau_bio`, SO,
  reserve, reward terms, GRF ed eventi;
- manifest con hash di modello, plugin caricato, setup, IK, ExternalLoads,
  reserve, profilo GRF, config Trajectory Generator e action script.

Produrre reference distinte per:

1. **replay canonico**, con GRF prescritte su entrambi i lati;
2. **hybrid canonico**, con contatto protesico online e lato sano prescritto.

Gli oracle esistenti possono essere riusati soltanto se manifest, hash, finestra
e action script coincidono. Senza questi artefatti i gate closed-loop non sono
eseguibili.

---

## Fase 0 - Spike di fattibilita GPU

La Fase 0 e un **gate-zero bloccante**. Non si costruisce il prodotto prima di
aver dimostrato che l'intero percorso necessario e realizzabile e abbastanza
veloce sulla RTX 4070.

Lo spike e una **vertical slice grezza ma riusabile**, non l'engine di Fase 2:

- usa modello reale e primitive computazionalmente rappresentative;
- puo usare configurazione hard-coded, schema observation ridotto e diagnostica
  minima;
- non deve ancora soddisfare parita scientifica, UX, recorder o API definitive;
- vive isolato sotto `validation/spikes/mjx_feasibility/`;
- le primitive scelte vengono poi promosse e indurite in Fase 2, evitando una
  seconda implementazione usa-e-getta.

### 0.1 Runtime WSL2/GPU

- Installare e verificare WSL2/Ubuntu, driver NVIDIA, JAX CUDA e `mujoco-mjx`.
- Verificare che `jax.devices()` esponga la RTX 4070.
- Eseguire `mjx.step` sotto `jit`, `vmap` e `lax.scan`.
- Registrare versioni, compile time, memoria e throughput.

### 0.2 Primitive CMC-like on-device

Verificare sotto JIT, senza host callback nel percorso caldo:

- stato, bias/passive/applied forces e accelerazioni MJX;
- metodo per ottenere la mass matrix o un'alternativa equivalente per
  inverse dynamics/outer loop;
- applicazione delle ExternalLoads prescritte;
- estrazione delle forze di contatto, GRF, penetrazione ed eventi;
- supporto/equivalenza degli attuatori muscolari;
- costruzione on-device di `A_muscle`.

Lo spike deve includere anche un contatto protesico rappresentativo della
modalita hybrid a batch `1/32/256`, con numero di sfere/geom realistico,
estrazione wrench/eventi e applicazione della forza alla dinamica. Misurare
separatamente memoria e throughput di:

1. contatto MJX nativo con array di contatto e vincoli `efc_*`;
2. port JAX on-device della legge onlineGRF custom;
3. eventuale soluzione mista contatto nativo + residuo JAX.

La Fase 0 non riceve GO sulla sola fattibilita replay: deve dimostrare che almeno
un candidato hybrid entra in memoria a batch `256`, oppure classificare
esplicitamente il batch nominale come NO-GO. Il candidato dello spike puo
essere non calibrato e non validato scientificamente: in Fase 0 serve a
misurare fattibilita, memoria e costo computazionale rappresentativi.

`ten_J` non e il requisito in se. Il gate e ottenere `A_muscle` on-device con
fedelta e throughput accettabili. Valutare in ordine:

1. Jacobian/moment arm esposti direttamente da MJX;
2. differenze finite vettorizzate via MJX;
3. moment arm precomputati/interpolati.

Ogni alternativa deve essere confrontata contro MuJoCo CPU per errore di moment
arm, `A_muscle`, coppia muscolare risultante e throughput.

### 0.3 Static optimization

- Micro-benchmark di `jaxopt.OSQP` sotto `jit`+`vmap` a batch
  `1/32/256`.
- Confrontare iterazioni fisse, warm-start e semantica del feasibility
  backtracking.
- Confrontare come criteri primari coppia generalizzata erogata
  `A_muscle @ activation`, coppia reserve, residuo di equilibrio, violazioni
  dei vincoli e valore obiettivo. Il vettore activation e diagnostico, perche
  solver diversi possono scegliere soluzioni differenti ma dinamicamente
  equivalenti in problemi degeneri o multi-ottimo.
- Il percorso produttivo non puo usare fallback SciPy nascosti.
- Un fallimento QP deve produrre diagnostica e truncation mascherata
  dell'environment interessato.

### 0.4 Precisione e composizione completa

- Verificare entrambi i PI SEA, governor e molla `321/500` a `1 ms` in float32.
- Integrare nello spike il carry SEA `(theta_m, omega_m, xi)` e l'anti-windup
  interno data-dependent dentro lo scan/stadio integrativo JAX.
- Integrare il carry governor `(qf, qf_dot)` nello scan JAX; clip di
  accelerazione/velocita e relativi reset sono branch data-dependent realizzati
  con `jnp.clip`/`jnp.where`/`lax.cond`, mai branch Python.
- Usare x64 come modalita `validation`; scegliere float32 o mixed precision per
  `training` soltanto dopo i test.
- Costruire un prototipo compilato di:
  - un substep CMC-like completo da `1 ms`;
  - un policy-step rappresentativo da `10 ms`, inclusi controlli, SO, fisica,
    observation/reward minimi e terminazioni;
  - batch `1/32/256` con reset mascherato.

### Gate Fase 0

Produrre una nota GO/NO-GO con:

- disponibilita di tutte le primitive richieste;
- strategia scelta per mass matrix/inverse dynamics, `A_muscle`, QP e contatto;
- parita numerica preliminare CPU-MJX;
- compile time, memoria e throughput del policy-step rappresentativo;
- memoria e throughput hybrid a batch `256`, includendo contatto e relativo
  wrench/residuo candidato.

Un NO-GO richiede revisione esplicita del modello o dell'architettura; non si
procede con fallback CPU nel loop di training.

---

## Fase 1 - Sincronizzazione modello e dinamica

### 1.1 Bundle e manifest

- Estendere `tools/osim2mjcf.py` affinche accetti esplicitamente modello `.osim`,
  reserve e destinazione bundle; il setup XML viene risolto da un orchestratore
  separato che passa i file reali al converter.
- Convertire `AB06_SEASEA_stiff321_500_pi.osim`.
- Importare IK `.mot`, `ExternalForces.xml`, `CMC_Actuators.xml`, profilo
  online-GRF e parametri controller correnti.
- Rigenerare coupling, metadati SEA e validazione geometrica/statica.
- Salvare manifest versionato con hash di ogni sorgente.

### 1.2 Dinamica e controlli

- Portare separatamente PI esterno cascade e PI interno SEA.
- Nel PI interno, trattare `xi` come terzo stato dinamico del SEA e applicare
  anti-windup a ogni valutazione dello stadio integrativo, coerentemente con
  `getTorqueErrorIntegralDot`.
- Portare impedance path, `Kd_Imp`, clamp, integral limits, anti-windup,
  feasibility scaling, LPF di `u`, reset e diagnostica.
- Portare outer loop biologico, inverse dynamics, static optimization e reserve
  secondo la strategia scelta in Fase 0.
- Tracciare esplicitamente ogni approssimazione del plant biologico, inclusi
  geometria, moment arm, dinamica muscolare Thelen, recruitment e reserve, con
  test isolati contro OpenSim.

### 1.3 Due modalita GRF distinte

Implementare e validare separatamente:

1. **Replay parity mode**: ExternalLoads prescritte su entrambi i lati e
   contatti disabilitati. Serve a isolare modello, SEA, ID e SO.
2. **Hybrid production mode**: contatto online applicato sul lato protesico
   sinistro e GRF prescritta sul lato sano destro.

La modalita ibrida non puo ereditare automaticamente i gate della replay:
richiede calibrazione e gate propri su GRF, eventi e penetrazione.

### 1.4 Porting e calibrazione del contatto hybrid

La fonte di verita del contatto non e un generico sphere/plane MuJoCo: e il
profilo onlineGRF JSON congelato con l'oracle e la legge
`OnlineGRFSphereHalfSpaceForce`. Questa comprende geometria a base sparsa,
Hunt-Crossley, attrito smussato, velocita del piano e, dove presente, residuo
state-only full-wrench scalato dalla forza normale e dipendente da penetrazione
e penetration-rate. Il residuo non legge tempo, fase o GRF prescritta.

Valutare e documentare tre candidati:

1. **Port JAX fedele, candidato predefinito**: implementare on-device la legge
   completa del plugin e applicare per-sfera forza e momento al corpo
   protesico.
2. **Contatto MJX nativo calibrato**: mappare geometria, friction,
   `solref`/`solimp` e treadmill velocity; e accettabile soltanto se riproduce
   anche il wrench e supera i gate C/D2 senza residui nascosti.
3. **Contatto nativo + residuo JAX**: usare il normale/contact state MJX come
   base e applicare on-device il residuo full-wrench, soltanto se la
   decomposizione e verificabile e piu veloce del port fedele.

Non assumere equivalenza diretta tra Hunt-Crossley e `solref`/`solimp`.
Calibrare su training split e validare su holdout, mantenendo separati:

1. test formula-locked con identiche cinematiche delle sfere, penetrazione e
   velocita;
2. replay cinematico dell'intera finestra canonica;
3. forward active breve, poi episodio da `2 s`;
4. perturbazioni di azione/stato non presenti nel fit;
5. benchmark hybrid batch `1/32/256`.

Confrontare per-sfera e aggregato: forza normale/tangenziale, free moment,
wrench totale, COP, penetrazione, slip, eventi, reserve root e drift dinamico.
Il profilo scelto deve essere active-validated; un profilo approvato soltanto
come `online_sensor` non puo diventare backend di training.

La decisione finale congela candidato, profilo, parametri e hash nel manifest.
Se nessun candidato supera contemporaneamente C/D2 e il gate prestazionale, la
modalita hybrid riceve NO-GO invece di essere sostituita implicitamente da
replay o GRF prescritta.

---

## Fase 2 - Ambiente CMC-like JAX/MJX batched

### API pubbliche

Introdurre:

```text
CMCEnvConfig
BatchedEnvState
reset_batch(keys, config) -> state, observation, info
step_batch(state, action) -> state, observation, reward, terminated, truncated, info
```

`BatchedEnvState` contiene almeno:

- `mjx.Data`;
- stati dei due PI SEA e del controller esterno;
- stato reference model/governor;
- warm-start static optimization;
- gait clock ed eventi;
- azione, endpoint e comando precedenti;
- RNG JAX;
- contatori, mask e motivi di fine episodio.

### Esecuzione

- Policy-step `10 ms` tramite `lax.scan` sui substep `1 ms`.
- Parallelizzazione tramite `vmap`.
- Reset selettivo tramite mask, senza ricompilazione.
- Nessun NumPy, SciPy, loop Python dinamico, conversione host o fallback CPU nel
  percorso caldo.
- Il watchdog wall-clock resta un controllo host sul batch/processo: non viene
  simulato come evento fisico per-environment dentro JIT.

### Contratto Trajectory Generator

Portare fedelmente:

- azione assoluta `(policy_knots=3, 2)` con anchor continuo;
- PCHIP JAX con la stessa regola delle pendenze del percorso OpenSim, oppure
  implementazione alternativa accettata soltanto se supera il gate
  action-to-reference;
- reference model `6 Hz`, governor persistente e limiti:
  - knee `6.0 rad/s`, `60.0 rad/s^2`;
  - ankle `3.5 rad/s`, `55.0 rad/s^2`;
- stato continuo governor `(qf, qf_dot)` integrato on-device, con clip
  data-dependent di velocita e accelerazione dentro lo scan JAX e test sui
  confini di saturazione;
- target imitativo periodico phase-based:
  - knee shift `0.465`;
  - ankle shift `0.452`;
- inizializzazione imitativa coerente;
- raw reward terms e reward centralizzata `ex_novo`/`imitation`;
- diagnostica SEA, command-rate, safety, OOB, GRF e contatto;
- terminazioni fisiche e truncation numeriche con codici interi JAX e mapping
  host verso gli `end_reason` testuali.

### Observation spec

Definire un unico schema versionato condiviso da environment, recorder e futuro
trainer:

- nomi e indici actor-realistic;
- nomi e indici critic-privileged;
- `n_actor`, `n_full`;
- unita e normalizzazione;
- feature consentite per ogni reward mode.

Lo schema deve impedire per costruzione che l'attore legga il suffisso
privilegiato.

---

## Fase 3 - Parita e benchmark della milestone v1-engine

### A. Equivalenza di formula

Con input e stato identici, confrontare OpenSim/Python-oracle, MuJoCo CPU e MJX:

- PI esterno cascade;
- PI interno SEA e anti-windup;
- action mapping e PCHIP;
- reference model/governor;
- branch e confini di clip del governor;
- target phase-based;
- static optimization, gatata primariamente su coppia generalizzata erogata,
  reserve, residuo/vincoli e obiettivo; il vettore activation e diagnostico
  salvo problemi resi univoci dalla stessa regolarizzazione;
- observation builder;
- reward terms e reward scalare;
- terminazioni.

Usare tolleranze strette definite per ogni componente e testare esplicitamente
stati di saturazione, clamp e reset.

### B1. Parita SEA isolata

Prima del closed-loop, validare ciascun SEA con plant bloccato imponendo gli
stessi `q_joint`, `qdot_joint`, `u`, stato iniziale e sequenza temporale a
OpenSim/plugin, MuJoCo CPU e MJX.

Separare due verifiche:

1. **B1a, legge istantanea**: con stato identico confrontare output, derivate e
   branch senza integrare. Questo isola la legge SEA dallo schema numerico.
2. **B1b, traiettoria integrata**: confrontare con schema, passo e punti di
   valutazione allineati tramite un harness fixed-step di riferimento.

Confrontare almeno:

- `theta_m`, `omega_m` e `torque_error_integral`;
- `tau_ref`, `tau_spring`, `tau_input_raw`, clamp e flag anti-windup;
- `sea_torque_rmse <= 0.5 Nm`;
- attivazione degli stessi branch di saturazione e anti-windup.

B1a e il gate bloccante sulla fedelta della **legge SEA**. La soglia
`sea_torque_rmse <= 0.5 Nm` si applica a B1b con integrazione allineata.
Confrontare inoltre il fixed-step produttivo JAX da `1 ms` contro
l'integrazione OpenSim/Simbody usata dall'oracle, quantificando separatamente
la quota di errore dovuta a schema, passo e tolleranze. Tale quota entra nei
gate closed-loop e non puo essere attribuita al plant muscolare o nascosta
allargando B1a.

### B2. Parita replay closed-loop

Con azioni scriptate identiche e GRF prescritte su entrambi i lati:

- `q_pros_rmse <= 0.01 rad`;
- `q_bio_rmse <= 0.02 rad`;
- `sea_torque_rmse <= 5 Nm`;
- stesso tipo di terminazione/truncation fisica o numerica;
- parita delle diagnostiche PI e assenza di drift non spiegato.

Il target `sea_torque_rmse <= 5 Nm` misura l'intero sistema e puo essere
bloccato dalla fedelta del lato biologico, anche quando B1 passa. In particolare,
una replica incompleta di geometria, moment arm, curve forza-lunghezza/velocita,
dinamica Thelen, recruitment o reserve modifica `tau_bio`, lo stato articolare e
quindi la domanda osservata dal SEA.

Se B2 fallisce, eseguire la seguente decomposizione causale prima di modificare
le soglie:

1. replay SEA plant-locked di B1;
2. replay controller-locked, imponendo stati articolari e segnali biologici
   OpenSim ma lasciando evolvere entrambi i PI SEA;
3. replay full closed-loop, confrontando in ordine moment arm, forza muscolare,
   attivazioni, `tau_bio`, reserve, stato articolare e coppia SEA.

Se B1 passa e il residuo B2 e attribuito quantitativamente al plant biologico,
la v1-engine puo avanzare soltanto come **consegna infrastrutturale
condizionata**, con mismatch e causa registrati. Non si dichiara parita
scientifica e il training di produzione resta bloccato finche:

- il gap biologico non viene chiuso; oppure
- una sensitivity/ablation dimostra che il residuo resta entro D2, non modifica
  materialmente ranking delle azioni, sicurezza o terminazioni, i segnali
  biologici divergenti restano esclusi da observation/reward, e la deroga viene
  approvata esplicitamente.

### C. Parita hybrid closed-loop

Con lato protesico a contatto online:

- contatto protesico concorde `>= 95%`;
- heel-strike/toe-off MAE `<= 20 ms`;
- normal GRF RMSE `<= 0.10 BW`;
- penetrazione sempre `< 0.028 m`;
- gate replay ancora rispettati sui segnali non dipendenti dal contatto, oppure
  deviazioni motivate e registrate.

### D1. Gate policy-relevant replay

In replay, per ogni policy-step:

- observation actor e full critic confrontate campo-per-campo;
- action-to-reference e riferimento servito confrontati campo-per-campo;
- reward scalare MAE `<= 0.01`;
- reward term attivi: errore relativo `<= 5%`, con tolleranza assoluta
  documentata per valori prossimi a zero;
- terminazioni e codici `end_reason` concordi.

### D2. Gate policy-relevant hybrid

In hybrid non si richiede uguaglianza stretta campo-per-campo dopo una
divergenza ammessa del contatto. Si richiedono invece:

- schema, indici, unita, normalizzazione, action-to-reference e governor
  identici;
- feature actor legate a contatto/eventi entro i gate temporali e GRF di C;
- observation continue confrontate tramite RMSE normalizzato sulla rispettiva
  scala configurata, `<= 10%`;
- reward term attivi confrontati tramite RMSE normalizzato, `<= 10%`, usando
  scale e floor congelati dall'oracle prima del confronto, e ritorno episodico
  con differenza relativa `<= 10%`;
- stessa categoria di terminazione e distribuzione degli `end_reason`; non e
  richiesta uguaglianza per-step dopo la prima divergenza di contatto ammessa.

Le soglie hybrid devono essere calcolate sulla suite canonica e riportate anche
per fase del passo, cosi che una media episodica non nasconda errori localizzati.

Le attivazioni/recruitment restano diagnostiche e non possono entrare in reward
o osservazioni finche il relativo mismatch non e chiuso.

### E. MuJoCo CPU contro MJX

- Policy-step N=1: `rtol=1e-4`, `atol=1e-5` in validation/x64.
- Episodi completi con azioni scriptate.
- Batch indipendenti, reset mascherati e riproducibilita per seed.
- Confronto validation/x64 contro training precision per quantificare il drift.

### Benchmark e criterio v1-engine

Il workload benchmark canonico di un `env-step` e un policy-step da `10 ms`
composto da dieci substep CMC-like da `1 ms`. Include:

- action mapping, reference model, governor e controllore esterno;
- inverse dynamics, costruzione di `A_muscle`, SO con le stesse tolleranze e
  strategia di backtracking, e reserve;
- entrambi i PI SEA, fisica, modalita GRF scelta, observation, reward e
  terminazioni.

Esclude compile, inizializzazione processo, I/O, recorder, TensorBoard e logging
host. Eseguire benchmark distinti per replay e hybrid usando stesso modello,
action script canonico, episodio da `2 s`, timestep, precisione, modalita GRF e
configurazione solver. "Stesso workload" significa stesso contratto
computazionale e stesse tolleranze, pur usando implementazioni native diverse:
nessun backend puo omettere ID, SO, contatto, observation, reward o terminazioni.

Per ogni configurazione eseguire `100` policy-step di warm-up, poi cinque
finestre misurate da `30 s`. Il throughput e:

```text
env-step/s = policy-step validi completati da tutti gli environment / wall-clock
```

Riportare mediana, minimo e massimo delle cinque finestre.

Misurare batch `1/8/32/128/256` e massimo compatibile:

- compile time;
- post-compile physics-step/s;
- policy-step/s ed env-step/s;
- memoria GPU;
- latenza batch=1;
- throughput MuJoCo CPU e OpenSim 12-worker sullo stesso workload canonico.

Il break-even bloccante della milestone v1-engine, misurato sul workload
**hybrid di produzione**, e:

```text
post-compile env-step/s MJX batch 256 >= 5x OpenSim 12-worker
```

La soglia `5x` e una decisione di prodotto fissata prima delle misure:
corrisponde ad almeno l'`80%` di riduzione del wall-clock rispetto alla baseline
OpenSim 12-worker e a una capacita equivalente di almeno `60` worker OpenSim.
Questo margine e il minimo scelto per compensare il costo di mantenere e
validare un secondo backend e lasciare headroom a policy forward, PPO update e
logging. Il benchmark replay resta diagnostico e non sostituisce quello hybrid.

Se batch 256 non entra in memoria, usare il massimo batch MJX compatibile e
registrare esplicitamente che il gate nominale non e stato eseguito; una soglia
alternativa richiede approvazione prima di osservare nuovi risultati. Se il gate
non passa, profilare e ridisegnare prima del PPO.

### Consegna v1-engine

- Engine MJX batched.
- Replay e hybrid mode.
- Harness e report di parita A-E.
- Benchmark e profiling.
- Rollout scripted batch e singolo.
- Nessuna dichiarazione di porting completo finche la Fase 4 non passa.

---

## Fase 4 - Trainer PPO JAX e completamento porting

### Selezione trainer

Prima dell'integrazione, eseguire uno spike comparativo tra **Brax training** e
**Rejax**. PureJaxRL resta riferimento implementativo ma non dipendenza, perche
non e progettato come libreria modulare. SBX non viene scelto come default
finche non dimostra un percorso completamente JIT con environment custom MJX.

Il trainer scelto deve superare:

- environment custom interamente JIT;
- azioni continue e PPO;
- actor-critic asimmetrico reale;
- corretta distinzione `terminated`/`truncated` nel bootstrap GAE;
- training e valutazione multi-env;
- checkpoint/resume;
- logging e metriche senza callback nel percorso caldo.

Scelta predefinita: **Brax training**, se supera tutti i gate; altrimenti Rejax.

### Integrazione

- Mantenere `training_cfg.yaml` come sorgente unica e salvare
  `training_cfg.resolved.yaml`.
- Mappare esplicitamente gli iperparametri RLlib PPO al trainer scelto.
- Implementare actor-realistic e critic-privileged usando lo schema observation
  versionato; non assumere che basti uno slicing senza test.
- Salvare checkpoint best/last, policy inference, `summary.json`, cronologia
  iterazioni e TensorBoard.
- Conservare supervisione, timeout host e restart da ultimo checkpoint valido.
- Registrare STO e diagnostica fuori dal loop JIT per environment selezionati.
- RLlib resta baseline OpenSim; nessun resume da checkpoint RLlib.

### Gate completamento porting

- PPO smoke simmetrico e asimmetrico.
- Checkpoint/resume e rollout deterministico batch=1.
- Rollout batch.
- Stesse action shape, observation spec, reward mode e config risolta tra
  training e rollout.
- Training breve con reward finita, parametri aggiornati e nessuna lettura
  privilegiata da parte dell'attore.
- Benchmark end-to-end comprensivo di policy forward e PPO update.

---

## Rischi dominanti e checkpoint strategico

La fedelta della legge SEA e necessaria ma non sufficiente alla parita
closed-loop. Un mismatch del plant biologico puo cambiare `tau_bio`, stato
articolare e domanda al SEA, facendo fallire B2 pur con B1 corretto.

Per questo il progetto mantiene separati:

- gate bloccante della legge SEA isolata;
- target di parita dell'intero sistema;
- avanzamento infrastrutturale condizionato;
- autorizzazione al training di produzione.

Nessuna soglia closed-loop viene allargata senza decomposizione causale,
evidenza quantitativa e approvazione esplicita.

Esiste inoltre un rischio programmatico concreto: B2 potrebbe non diventare mai
autorizzabile con il modello muscolare disponibile. In tal caso il progetto
potrebbe produrre un engine veloce ma non raggiungere l'obiettivo finale di
training/inference scientificamente autorizzati. La sensitivity/ablation
necessaria per una deroga e un sotto-progetto di ricerca, non una verifica
ordinaria da assumere implicitamente.

Dopo la prima decomposizione B2 su replay, e prima di calibrare completamente
hybrid e indurire l'intera Fase 2, produrre un checkpoint decisionale con:

- dimensione e cause del gap biologico;
- impatto misurato su azioni, observation, reward, sicurezza e terminazioni;
- stima di lavoro/costo per chiudere il gap;
- protocollo, costo e criteri di successo dell'eventuale sensitivity/ablation;
- decisione esplicita **GO scientifico**, **RESEARCH prima di continuare**,
  **engine infrastrutturale soltanto** oppure **STOP**.

Il budget e la milestone successiva vengono approvati soltanto dopo questo
checkpoint; la costruzione dell'engine completo non e usata come prerequisito
per scoprire se l'obiettivo utente e raggiungibile.

---

## Ordine operativo

1. Eseguire Fase 0 e produrre GO/NO-GO.
2. Generare, verificare e congelare gli oracle OpenSim canonici `321/500_pi`.
3. Sincronizzare modello e dinamica in replay mode.
4. Chiudere B1 e decomporre causalmente ogni residuo B2.
5. Eseguire il checkpoint strategico GO/RESEARCH/infrastruttura/STOP.
6. Chiudere parita replay prima di introdurre il contatto ibrido.
7. Se autorizzato, selezionare, calibrare e validare il candidato hybrid.
8. Costruire l'ambiente batched completo e chiudere i gate A-E.
9. Superare benchmark e consegnare v1-engine, eventualmente condizionata.
10. Selezionare e integrare il trainer PPO soltanto dopo l'autorizzazione
   scientifica prevista da B2.
11. Chiudere i gate Fase 4 e dichiarare completato il porting MLP.

## Riferimenti runtime

- [JAX installation](https://docs.jax.dev/en/latest/installation.html)
- [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
- [Brax](https://github.com/google/brax)
- [Rejax](https://github.com/keraJLi/rejax)
- [PureJaxRL](https://github.com/luchris429/purejaxrl)
