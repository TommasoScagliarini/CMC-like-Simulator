# Porting definitivo Trajectory Generator baseline_MLP su MuJoCo/MJX GPU

## Missione

Portare nella repository `MuJoCo_env` lo stato **attualmente implementato** della
Trajectory Generator `baseline_MLP`, preservando:

- dinamica e semantica di controllo policy-relevant;
- scelte strutturali di ambiente, policy, training, rollout e supervisione;
- esperienza utente semplice e riproducibile;
- compatibilita CPU/debug su Windows x86 e macOS arm64;

e usando MJX/JAX su RTX 4070 via WSL2/Linux per accelerare realmente:

- generazione di rollout;
- inference deterministica singola e batch;
- sampling e training PPO.

OpenSim resta l'oracolo scientifico. MuJoCo CPU e il backend di validazione e
debug della repository target. MJX e il backend di produzione batched.

Il porting deve ridurre il wall-clock senza complicare la vita dell'utente. Non
basta produrre un engine veloce: il lavoro e completato soltanto quando anche
training, rollout, configurazione, checkpoint, diagnostica e comandi utente sono
portati e verificati.

---

## Repository e ownership

### Repository sorgente OpenSim

```text
C:\Users\tomma\Desktop\Opensim OMNIBUS\CMC-like-Simulator - Claude
```

Responsabilita:

- fonte di verita scientifica;
- modello e setup OpenSim correnti;
- plugin SEA C++;
- CMC-like root;
- Trajectory Generator `baseline_MLP`;
- oracle e output di riferimento.

Il codice sorgente OpenSim non deve essere modificato per adattarsi a MuJoCo,
salvo strumenti di esportazione, diagnostica o documentazione esplicitamente
necessari. Non modificare la semantica del plugin C++ SEA.

### Repository target MuJoCo/MJX

```text
C:\Users\tomma\Desktop\MuJoCo_env
```

Responsabilita:

- bundle MJCF e manifest sincronizzati;
- MuJoCo CPU;
- MJX/JAX batched;
- porting del contratto Trajectory Generator;
- trainer PPO, rollout e UX di produzione;
- harness di parity e benchmark.

Tutto il nuovo codice di produzione del porting deve vivere in `MuJoCo_env`.
Gli strumenti target possono leggere la repository OpenSim tramite un
`--source-root` esplicito, ma il runtime di training/inference non deve dipendere
da import Python o file non versionati della repository sorgente.

### Stato VCS da gestire prima delle modifiche

Al momento della stesura, `MuJoCo_env` non risulta una repository Git. Prima di
modificare codice:

1. verificare nuovamente lo stato VCS;
2. ottenere approvazione esplicita prima di inizializzare Git;
3. se si procede senza Git, creare almeno un manifest read-only con hash e lista
   dei file target esistenti;
4. non sovrascrivere il bundle storico `models/SEASEA` finche il nuovo bundle
   non supera i gate statici e replay.

---

## Definizione di completamento

### Porting 1:1 completato

Il porting e completato soltanto quando:

1. il backend usa il modello/setup corrente `AB06_SEASEA_stiff321_500_pi`;
2. passano i gate scientifici strict definiti in questo documento;
3. passano i gate MuJoCo CPU contro MJX;
4. passano i gate di prestazione di engine, inference e training;
5. tutte le feature `baseline_MLP` dentro scope risultano portate nella matrice
   source-to-target;
6. l'utente dispone di comandi semplici per training, oracle, benchmark,
   rollout batch e rollout deterministico batch `1`;
7. checkpoint, auto-config rollout, supervisione e artefatti sono verificati;
8. CPU/debug e tooling condiviso passano su Windows x86 e macOS arm64; il
   percorso GPU passa su WSL2/Linux.

### Risultato condizionato che non completa il porting 1:1

Se la legge SEA passa ma il plant biologico non raggiunge la parity dinamica,
il progetto puo consegnare:

- engine infrastrutturale;
- benchmark;
- backend di ricerca/surrogate;
- protocollo di sensitivity/ablation.

Questo risultato deve essere etichettato chiaramente come
`research_surrogate_only`. Non autorizza la dichiarazione di dinamica 1:1 e non
completa l'obiettivo iniziale, anche se una policy sembra funzionare.

Una deroga scientifica puo autorizzare esperimenti separati, ma non puo
trasformare un mismatch dinamico in un PASS del porting 1:1.

### Fuori scope

- `Prosthesis_SNN`;
- TODO funzionali non ancora implementati nella `baseline_MLP`;
- conversione o resume dei checkpoint RLlib esistenti;
- modifica della semantica scientifica OpenSim per facilitare MuJoCo;
- GPU nativa Windows o macOS.

---

## Stato iniziale verificato

### Sorgente OpenSim corrente

- Setup:
  `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`.
- Modello: `AB06_SEASEA_stiff321_500_pi.osim`.
- Finestra setup: `t_start=11.99 s`, `t_end=21.00 s`.
- Policy-step: `10 ms`.
- Substep fisico validato: `1 ms`.
- Stiffness SEA: knee `321`, ankle `500`.
- Trajectory Generator corrente:
  `Trajectory Generator/baseline_MLP/`.
- Config di riferimento:
  `Trajectory Generator/baseline_MLP/training_cfg.yaml`.
- Hybrid corrente: `grf_mode=online_sensor`, contatto online applicato sul lato
  protesico sinistro, GRF prescritta sul lato sano destro.
- Profilo GRF corrente: il file referenziato dalla config risolta del run
  canonico; al momento della stesura:
  `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`.

### Target MuJoCo_env corrente

- Bundle storico convertito da `Adjusted_SEASEA - Copia_tuned.osim`.
- Stiffness SEA storica `1000/700`.
- `mujoco_env/trajectory_env.py` e ancora CPU/NumPy e usa un contratto azione,
  observation e reward piu semplice della Trajectory Generator corrente.
- `mujoco_env/mjx_runner.py` supporta step fisici raw batched, non il policy-step
  CMC-like completo.
- Le metriche precedenti di parity non sono valide per il nuovo obiettivo RL.
- Mismatch scientifici storici:
  - `sea_torque_rmse` circa `112-127 Nm`;
  - correlazione attivazioni accettabile circa `0.02`;
  - parity biologica non chiusa.

### Dinamica SEA da preservare

Esistono due integratori distinti.

1. PI esterno cascade Python:

   - integra errore di velocita;
   - produce il comando normalizzato `u`;
   - include limit, feasibility scaling, LPF e anti-windup.

2. PI interno del plugin SEA:

   ```text
   torque_error = tau_ref - tau_spring

   tau_input_raw =
       tau_ref
       + Kp * torque_error
       + clamp(Ki * torque_error_integral, integral_torque_limit)
       - Kd * omega_m
   ```

   Il carry interno per ogni SEA e:

   ```text
   theta_m
   omega_m
   torque_error_integral
   ```

   La derivata dell'integrale e l'anti-windup vengono valutati a ogni
   stadio/substep integrativo.

Nel percorso JAX, branch data-dependent di SEA e governor devono usare
`jnp.where`, `jnp.clip` o `lax.cond`, mai branch Python nel percorso JIT.

---

## Regole operative per il coding agent

1. Leggere gli `AGENTS.md` di entrambe le repository prima di ogni fase.
2. Non avanzare alla fase successiva se il relativo gate bloccante fallisce.
3. Non nascondere un fallimento con fallback CPU, SciPy o callback host nel
   percorso caldo.
4. Non modificare o rimuovere test esistenti soltanto per ottenere PASS.
5. Aggiornare i test storici `1000/700` soltanto dopo aver introdotto il nuovo
   bundle e aver documentato la nuova fonte.
6. Preservare il bundle storico finche la nuova conversione non e validata.
7. Tenere leggero `python -c "import mujoco_env"`: trainer, TensorBoard, Orbax e
   dipendenze pesanti devono essere importati lazy o da package separati.
8. Salvare per ogni run:
   - manifest;
   - config risolta;
   - versioni runtime;
   - precisione;
   - hash del bundle e delle sorgenti;
   - risultato dei gate applicabili.
9. Ogni fase deve produrre un report Markdown in:

   ```text
   C:\Users\tomma\Desktop\MuJoCo_env\reports\porting\
   ```

10. Ogni report deve includere: problema, decisioni, file modificati, comandi,
    test, risultati, gate PASS/FAIL e TODO residui.
11. Prima di introdurre un nuovo tool o un nuovo formato, verificare se esiste
    gia un equivalente nella repository target.
12. Non confondere:
    - formula parity;
    - replay dynamic parity;
    - hybrid behavioral parity;
    - performance parity;
    - feature/UX parity.

---

## Layout target raccomandato

Preservare i moduli esistenti e aggiungere responsabilita senza concentrare
tutto in `trajectory_env.py`.

```text
MuJoCo_env/
  models/
    AB06_SEASEA_stiff321_500_pi/
      model.xml
      source_manifest.json
      kinematic_couplings.json
      data/

  mujoco_env/
    config.py
    model_loader.py
    sea_plugin.py
    prosthesis_controller.py
    outer_loop.py
    inverse_dynamics.py
    static_optimization.py
    online_grf.py                 # nuovo: legge onlineGRF JAX/MuJoCo
    batched_env.py                # nuovo: API pure-JAX/MJX
    trajectory_env.py             # wrapper CPU/Gymnasium/debug
    observation_spec.py           # nuovo: schema versionato
    simulation_runner.py
    mjx_runner.py
    output.py
    parity.py

  trajectory_generator/
    baseline_mlp/
      training_cfg.yaml
      training_config.py
      reward_function.py
      trainer.py
      rollout.py
      process_watchdog.py
      progress_display.py
      tb_logging.py
      feature_parity.yaml

  scripts/
    train_ppo_mlp.py
    rollout_eval.py
    run_simulation.py
    run_mjx_batch.py
    run_parity.py
    run_benchmark.py

  tools/
    sync_opensim_bundle.py
    osim2mjcf.py
    parity_test.py
    validate_model.py

  tests/
  validation/
  reports/porting/
```

I nomi possono essere adattati ai pattern locali, ma le responsabilita devono
restare separate. `trajectory_env.py` non deve diventare un secondo
`simulation_runner.py`.

---

## Matrice source-to-target obbligatoria

Creare all'inizio:

```text
MuJoCo_env/trajectory_generator/baseline_mlp/feature_parity.yaml
```

Ogni riga deve contenere:

```text
feature
source_files
source_tests_or_reports
target_files
target_tests
status: missing | partial | implemented | verified
notes
```

La matrice deve includere almeno:

| Area | Sorgente OpenSim | Target MuJoCo_env |
|---|---|---|
| Config simulatore | `config.py`, setup XML | `mujoco_env/config.py`, bundle manifest |
| Loader e GRF | `model_loader.py`, `online_grf.py` | `model_loader.py`, nuovo `online_grf.py` |
| SEA interno | plugin `SeriesElasticActuator` | `sea_plugin.py` |
| Controller esterno | `prosthesis_controller.py` | `prosthesis_controller.py` |
| Outer loop / ID / SO | moduli root omonimi | moduli target omonimi |
| Runner CMC-like | `simulation_runner.py` | `simulation_runner.py`, `batched_env.py` |
| Contratto Trajectory | `Trajectory Generator/osim_trj_cmc_like.py` | `batched_env.py`, `trajectory_env.py` |
| Reward | `baseline_MLP/reward_function.py` | `trajectory_generator/baseline_mlp/reward_function.py` |
| Observation actor/critic | env sorgente + asymmetric module | `observation_spec.py`, trainer |
| Training config | `training_cfg.yaml`, `training_config.py` | equivalenti target |
| Training PPO | `train_ppo_mlp.py` | `trainer.py`, wrapper script |
| Rollout | `rollout_eval.py` | `rollout.py`, wrapper script |
| Supervisione | watchdog/progress sorgenti | equivalenti target |
| Logging/artefatti | TensorBoard, summary, JSONL | equivalenti target |
| UX | `README.md`, `commands.txt` | README e comandi target |

Prima di implementare, generare anche un inventario machine-readable della
baseline corrente che elenchi:

- tutte le chiavi di `training_cfg.yaml`;
- tutti gli argomenti CLI pubblici di training, rollout, oracle e diagnostica;
- file/artefatti prodotti da un training e da un rollout canonici;
- observation names/indici;
- raw reward terms;
- codici `end_reason`;
- modalita e flag GRF;
- combinazioni symmetric/asymmetric ed `ex_novo`/`imitation`.

Salvare l'inventario nella repository target e collegare ogni voce a una riga
della matrice. La lista manuale seguente e il minimo obbligatorio, non sostituisce
l'inventario completo.

Feature gia implementate da non perdere:

- azione assoluta `(policy_knots=3, 2)` con anchor continuo;
- PCHIP/action-to-reference equivalente;
- target imitativo phase-based;
- reference model/governor persistente;
- actor-realistic e critic-privileged;
- reward `ex_novo` e `imitation`;
- override reward JSON;
- warm-start imitation-to-ex-novo;
- auto-load di `training_cfg.resolved.yaml` nel rollout;
- checkpoint best/last;
- export actor inference-only senza value tower privilegiata;
- resume manuale;
- restart automatico da ultimo checkpoint valido;
- timeout e watchdog self-test;
- progress live;
- TensorBoard;
- `summary.json`, cronologia iterazioni e rollout summary;
- rollout con e senza recorder STO;
- hybrid GRF e diagnostiche gait;
- `terminated` contro `truncated` e `end_reason`;
- comandi semplici Windows, macOS e WSL2.

Il porting non puo essere dichiarato completo con righe `missing` o `partial`
nella matrice per feature dentro scope.

---

## Fase 0 - Preflight, inventario e fattibilita GPU

### 0.1 Preflight repository e runtime

Prima di modificare codice:

1. salvare stato e hash di entrambe le repository;
2. registrare le modifiche locali gia presenti;
3. verificare ambiente CPU target:

   ```text
   conda activate mujoco-cmc
   python -c "import mujoco_env"
   python -m pytest -p no:cacheprovider tests -q
   ```

4. verificare WSL2/Ubuntu, driver NVIDIA, JAX CUDA e `mujoco-mjx`;
5. verificare che `jax.devices()` esponga la RTX 4070;
6. creare la matrice `feature_parity.yaml`;
7. congelare una baseline prestazionale OpenSim e MuJoCo CPU prima delle
   ottimizzazioni.

### 0.2 Vertical slice GPU riusabile

Creare sotto `validation/spikes/mjx_feasibility/` una vertical slice grezza ma
riusabile. Non deve essere scientificamente validata o avere UX definitiva, ma
deve usare dimensioni e primitive computazionalmente rappresentative.

Deve includere in un policy-step rappresentativo da `10 ms`:

- dieci substep da `1 ms`;
- outer loop e inverse dynamics;
- costruzione di `A_muscle`;
- static optimization JAX;
- entrambi i PI SEA;
- reference governor;
- fisica MJX;
- observation/reward minimi;
- terminazioni/reset mascherati;
- contatto hybrid rappresentativo.

Verificare sotto `jit` + `vmap` + `lax.scan`, senza host callback:

- mass matrix o alternativa necessaria all'ID;
- applied/passive/bias forces;
- ExternalLoads prescritte;
- attuatori muscolari;
- moment arm e `A_muscle`;
- QP warm-started;
- applicazione di forza/momento custom;
- estrazione contatti, wrench, penetrazione ed eventi.

### 0.3 Static optimization spike

Confrontare almeno:

- `jaxopt.OSQP`;
- iterazioni fisse;
- warm-start;
- feasibility/backtracking espresso con primitive JAX.

Criteri primari di equivalenza:

- coppia generalizzata `A_muscle @ activation`;
- coppia reserve;
- residuo di equilibrio;
- violazioni dei vincoli;
- valore obiettivo.

Il vettore activation e diagnostico: solver differenti possono scegliere
soluzioni multi-ottimo diverse ma dinamicamente equivalenti.

### 0.4 Hybrid memory/compute spike

Misurare batch `1/32/128/256` e massimo compatibile per:

1. contatto MJX nativo;
2. port JAX della legge onlineGRF;
3. soluzione mista contatto nativo + residuo JAX.

Il candidato puo essere non calibrato in Fase 0. Deve pero usare numero di
sfere/geom, array di contatto e wrench rappresentativi.

Batch `256` deve essere tentato e documentato, ma non e un fine assoluto. Se non
entra in memoria, il progetto puo ricevere GO soltanto se il massimo batch
stabile `B*`:

- e almeno `32`;
- dimostra un percorso credibile verso il gate prestazionale finale;
- non richiede semplificazioni scientifiche nascoste.

### Gate Fase 0

GO soltanto se:

- tutte le primitive necessarie sono disponibili on-device;
- esiste una strategia JAX per ID, `A_muscle`, SO e contatto;
- il policy-step rappresentativo compila ed esegue;
- reset mascherati e batch indipendenti funzionano;
- esiste almeno un candidato hybrid stabile;
- memoria e throughput indicano che il target finale e plausibile.

Produrre:

```text
reports/porting/phase_0_feasibility.md
validation/spikes/mjx_feasibility/results.json
```

Un NO-GO richiede ridisegno esplicito. Non proseguire costruendo fallback CPU
nel percorso di training.

---

## Fase 1 - Oracle canonico e sincronizzazione del bundle

### 1.1 Oracle OpenSim canonico

Generare e congelare output OpenSim dal setup corrente.

Finestre:

- completa: `11.99-21.00 s`;
- primaria RL: episodio deterministico da `2.0 s` a partire da `11.99 s`;
- smoke formula/dinamica: finestre brevi sufficienti a saturazioni e transitori.

Suite azioni condivisa:

- comando costante;
- rampa;
- target imitativo governato;
- stress controllato dei limiti;
- azioni deterministiche campionate con seed congelato.

Produrre oracle separati:

1. replay con GRF prescritte su entrambi i lati;
2. hybrid OpenSim corrente con contatto online applicato a sinistra e
   prescribed a destra.

Salvare:

- stati e controlli;
- output dei due PI;
- stati/derivate/diagnostica SEA;
- `tau_bio`;
- SO, muscoli e reserve;
- raw reward terms e reward finale;
- observation actor/full;
- action-to-reference e governor;
- GRF, COP, penetrazione ed eventi;
- terminazioni e `end_reason`;
- manifest con hash e config risolta.

### 1.2 Nuovo bundle target

Non sovrascrivere `models/SEASEA`.

Creare:

```text
MuJoCo_env/models/AB06_SEASEA_stiff321_500_pi/
```

Estendere `tools/osim2mjcf.py` o creare `tools/sync_opensim_bundle.py` affinche
accetti esplicitamente:

- `--source-root`;
- setup XML;
- modello `.osim`;
- IK;
- ExternalLoads e file `.mot`;
- reserve;
- profilo onlineGRF;
- output bundle.

Il bundle deve includere `source_manifest.json` con hash di ogni input,
parametri SEA/controller, versioni converter e note sulle approssimazioni.

### 1.3 Validazione statica

Ripetere e aggiornare i gate gia esistenti:

- masse;
- CoM;
- mass matrix;
- coordinate e coupling;
- moment arm;
- muscoli e reserve;
- mapping nomi/indici;
- geometria e frame del contatto.

Ogni approssimazione di wrap object, MovingPathPoint, ConditionalPathPoint e
Thelen deve essere registrata.

### Gate Fase 1

- Oracle canonici completi e riproducibili.
- Nuovo bundle selezionabile senza eliminare il bundle storico.
- Manifest completo.
- Validazione statica PASS.
- Test target aggiornati al nuovo bundle senza rompere il vecchio percorso di
  debug fino alla migrazione esplicita.

---

## Fase 2 - Dinamica replay e parity scientifica strict

### 2.1 SEA e controller

Portare separatamente:

- PI esterno cascade;
- PI interno SEA;
- `theta_m`, `omega_m`, `torque_error_integral`;
- anti-windup;
- clamp e integral limits;
- impedance path e `Kd_Imp`;
- feasibility scaling;
- LPF di `u`;
- reset e diagnostica.

#### Gate B1a - Legge SEA istantanea

Con stato/input identici e senza integrare, confrontare OpenSim/plugin,
MuJoCo CPU e JAX:

- output;
- derivate;
- `tau_ref`;
- `tau_spring`;
- `tau_input_raw`;
- clamp;
- branch anti-windup.

B1a e il gate bloccante sulla legge. Non puo essere derogato attribuendo
l'errore al plant biologico.

#### Gate B1b - Traiettoria SEA integrata

Usare un harness fixed-step con schema, passo e punti di valutazione allineati:

- `sea_torque_rmse <= 0.5 Nm`;
- stessi eventi di saturazione/anti-windup;
- assenza di drift non spiegato.

Confrontare poi separatamente il fixed-step produttivo JAX `1 ms` con
l'integratore OpenSim/Simbody. Quantificare l'errore dovuto a schema, passo e
tolleranze; non nasconderlo nei mismatch biologici.

### 2.2 Replay CMC-like

In replay:

- ExternalLoads prescritte su entrambi i lati;
- contatti disabilitati;
- stesso stato iniziale;
- stesse azioni scriptate;
- stessi tempi e sample point.

Portare e verificare:

- kinematics/coupling;
- outer loop;
- inverse dynamics;
- static optimization;
- reserve;
- entrambi i PI;
- fisica MuJoCo CPU;
- osservazioni e reward policy-relevant.

### 2.3 Decomposizione del mismatch biologico

Se il replay full closed-loop fallisce:

1. eseguire B1 plant-locked;
2. eseguire controller-locked imponendo stati e segnali biologici OpenSim;
3. confrontare in ordine:
   - geometria e moment arm;
   - force-length/force-velocity;
   - dinamica Thelen;
   - attivazioni;
   - forza muscolare;
   - `tau_bio`;
   - reserve;
   - stato articolare;
   - coppia SEA.

Le attivazioni restano diagnostiche e non sono richieste identiche in un
problema multi-ottimo. Tuttavia un mismatch di attivazioni che cambia la
dinamica non puo essere ignorato: deve emergere nei gate su coppia, stato e
reward.

### Gate B2 - Replay dynamic parity strict

Con azioni scriptate identiche:

- `q_pros_rmse <= 0.01 rad`;
- `q_bio_rmse <= 0.02 rad`;
- `sea_torque_rmse <= 5 Nm`;
- stesso tipo di terminazione/truncation;
- `end_reason` concorde;
- nessun drift non spiegato;
- coppia SO/reserve e residui entro tolleranze congelate dall'oracle;
- reward e observation entro Gate D1.

### Checkpoint strategico

Dopo la prima decomposizione B2, prima di investire nel contatto definitivo e
nell'intero engine batched, produrre:

- dimensione e causa del gap;
- impatto su dinamica, observation, reward, azioni, sicurezza e terminazioni;
- costo stimato per chiuderlo;
- eventuale protocollo sensitivity/ablation;
- decisione:
  - `GO_STRICT`;
  - `RESEARCH_REQUIRED`;
  - `INFRASTRUCTURE_ONLY`;
  - `STOP`.

Solo `GO_STRICT` autorizza il percorso che puo concludersi come porting 1:1.
Gli altri esiti possono proseguire soltanto con obiettivo e label separati.

---

## Fase 3 - Porting e calibrazione del contatto hybrid

### Fonte di verita

La fonte di verita non e un generico sphere/plane MuJoCo. E la combinazione di:

- hybrid OpenSim corrente;
- profilo onlineGRF congelato nel manifest;
- legge `OnlineGRFSphereHalfSpaceForce`;
- lato sinistro applicato e lato destro prescritto.

La legge sorgente include:

- geometria a base sparsa;
- Hunt-Crossley;
- attrito smussato;
- surface/treadmill velocity;
- eventuale residuo state-only full-wrench scalato dalla forza normale;
- dipendenze da penetrazione e penetration-rate;
- nessuna lettura runtime di tempo, fase o GRF prescritta.

### Candidati

Valutare:

1. **Port JAX fedele**, candidato predefinito.
2. **Contatto MJX nativo calibrato**, accettabile soltanto se supera i gate senza
   perdere wrench o introdurre compensazioni nascoste.
3. **Contatto nativo + residuo JAX**, soltanto se la decomposizione e
   verificabile e vantaggiosa.

Non assumere equivalenza diretta tra Hunt-Crossley e `solref`/`solimp`.

### Protocollo calibrazione

Usare split training/holdout e mantenere separati:

1. formula-locked per-sfera;
2. replay cinematico canonico;
3. forward active breve;
4. episodio active da `2 s`;
5. perturbazioni di stato e azione non presenti nel fit;
6. benchmark batch `1/32/128/256/B*`.

Confrontare:

- forza normale e tangenziale;
- free moment;
- wrench aggregato;
- COP;
- penetrazione;
- slip;
- eventi;
- reserve root;
- drift dinamico;
- coppie protesiche;
- observation/reward policy-relevant.

Un profilo approvato soltanto come sensore non puo diventare backend di
produzione active senza gate forward active.

### Gate C0 - Formula contact-locked

Prima del forward active, alimentare la legge sorgente e il candidato target con
identiche cinematiche per-sfera, velocita, penetrazione e penetration-rate.
Confrontare in validation/x64:

- forza normale e tangenziale per-sfera;
- forza residua;
- free moment;
- punto di applicazione;
- wrench aggregato.

Per il port JAX fedele usare `rtol=1e-4`, `atol=1e-5` sulle grandezze
normalizzate con scale/floor congelati prima del test. Un contatto MJX nativo
non e tenuto alla stessa formula interna, ma deve dichiarare esplicitamente la
non-equivalenza e non puo essere scelto soltanto perche piu veloce: deve superare
holdout, perturbazioni, C e D2.

### Gate C - Hybrid behavioral parity

- contatto protesico concorde `>= 95%`;
- heel-strike/toe-off MAE `<= 20 ms`;
- normal GRF RMSE `<= 0.10 BW`;
- penetrazione sempre `< 0.028 m`;
- nessuna instabilita o compensazione reserve non spiegata;
- Gate D2;
- gate prestazionale hybrid plausibile.

Se nessun candidato passa contemporaneamente C, D2 e prestazioni, la modalita
hybrid riceve NO-GO. Non sostituirla implicitamente con replay o prescribed.

---

## Fase 4 - Ambiente CMC-like JAX/MJX batched

### API pubbliche

Implementare una API pure-JAX:

```text
CMCEnvConfig
BatchedEnvState
reset_batch(keys, config) -> state, observation, info
step_batch(state, action) -> state, observation, reward, terminated, truncated, info
```

`BatchedEnvState` contiene almeno:

- `mjx.Data`;
- stato dei due SEA interni;
- stato controller/PI esterno;
- stato reference model/governor `(qf, qf_dot)`;
- warm-start SO;
- gait clock ed eventi;
- azione, endpoint, riferimento e comando precedenti;
- RNG JAX;
- contatori, mask e codici di fine episodio.

### Esecuzione

- Policy-step `10 ms` tramite `lax.scan` su substep `1 ms`.
- Parallelizzazione con `vmap`.
- Reset selettivi mascherati senza ricompilazione.
- Nessun NumPy, SciPy, loop Python dinamico, host callback o conversione host
  nel percorso caldo.
- Un fallimento SO produce diagnostica e truncation dell'environment
  interessato, non fallback nascosto.
- Modalita:
  - `validation`: x64;
  - `training`: float32 o mixed precision approvata dai test.

### Contratto Trajectory Generator

Portare:

- azione assoluta `(policy_knots=3, 2)`;
- anchor continuo;
- PCHIP/action-to-reference equivalente;
- reference model `6 Hz`;
- governor persistente:
  - knee `6.0 rad/s`, `60.0 rad/s^2`;
  - ankle `3.5 rad/s`, `55.0 rad/s^2`;
- clip governor dentro lo scan JAX;
- target imitativo phase-based:
  - knee shift `0.465`;
  - ankle shift `0.452`;
- inizializzazione imitativa coerente;
- raw reward terms;
- reward centralizzata `ex_novo`/`imitation`;
- diagnostiche SEA, command-rate, safety, OOB, GRF e contatto;
- gait clock;
- terminazioni e truncation con codici interi JAX e mapping host.

### Observation spec

Creare uno schema versionato unico condiviso da env, recorder, trainer e
rollout:

- nomi e indici actor-realistic;
- nomi e indici critic-privileged;
- `n_actor`, `n_full`;
- unita e normalizzazione;
- feature ammesse per reward mode;
- schema version/hash nel checkpoint.

L'attore non deve poter leggere feature privilegiate. Il test non deve limitarsi
a verificare uno slicing: deve controllare gli input effettivamente consumati
dalla policy.

### Gate D1 - Policy-relevant replay

In replay, per ogni policy-step:

- observation actor/full campo-per-campo;
- action-to-reference campo-per-campo;
- riferimento servito campo-per-campo;
- reward scalare MAE `<= 0.01`;
- reward term attivi con errore relativo `<= 5%` e floor assoluto congelato;
- terminazioni e `end_reason` concordi.

### Gate D2 - Policy-relevant hybrid

In hybrid:

- schema, indici, unita, normalizzazione e governor identici;
- feature contatto/eventi entro Gate C;
- observation continue con RMSE normalizzato `<= 10%`;
- reward term attivi con RMSE normalizzato `<= 10%`;
- ritorno episodico con differenza relativa `<= 10%`;
- stessa categoria di terminazione e distribuzione degli `end_reason`;
- metriche riportate anche per fase del passo.

Non richiedere uguaglianza per-step dopo una divergenza di contatto ammessa.

### Gate E - MuJoCo CPU contro MJX

- N=1 policy-step in validation/x64: `rtol=1e-4`, `atol=1e-5`;
- episodi completi;
- batch indipendenti;
- reset mascherati;
- riproducibilita per seed;
- drift validation/x64 contro training precision quantificato;
- checkpoint state round-trip.

---

## Fase 5 - Milestone v1-engine e benchmark

### Workload canonico

Un `env-step` e un policy-step da `10 ms` composto da dieci substep CMC-like da
`1 ms`. Include:

- action mapping, reference model e governor;
- controller esterno;
- inverse dynamics;
- costruzione `A_muscle`;
- SO con stessa strategia/tolleranze;
- reserve;
- entrambi i PI SEA;
- fisica e GRF;
- observation;
- reward;
- terminazioni.

Esclude:

- compile;
- bootstrap processo;
- recorder/STO;
- TensorBoard;
- logging host.

"Stesso workload" significa stesso contratto computazionale e stesse tolleranze,
pur usando implementazioni native diverse. Nessun backend puo omettere ID, SO,
contatto, observation, reward o terminazioni.

### Protocollo benchmark

Per replay e hybrid separatamente:

- stesso modello/config/action script/episodio `2 s`;
- batch `1/8/32/128/256` e massimo compatibile;
- `100` policy-step warm-up;
- cinque finestre misurate da `30 s`;
- riportare mediana, minimo, massimo e memoria.

```text
env-step/s = policy-step validi completati da tutti gli env / wall-clock
```

Misurare:

- compile time;
- physics-step/s;
- policy-step/s;
- env-step/s;
- latenza batch `1`;
- memoria;
- throughput OpenSim 12-worker;
- throughput MuJoCo CPU;
- throughput MJX.

### Gate prestazionale v1-engine

Definire `B*` come il massimo batch stabile compatibile con la RTX 4070 tra i
batch richiesti e un eventuale massimo aggiuntivo.

Gate bloccante:

```text
post-compile hybrid env-step/s MJX a B* >= 5x OpenSim 12-worker
```

Batch `256` deve essere misurato se entra in memoria. Se non entra, il gate puo
passare a `B*` soltanto se `B* >= 32` e la deviazione e documentata. La soglia
non puo essere cambiata dopo aver visto i risultati senza approvazione.

La soglia `5x` corrisponde ad almeno `80%` di riduzione del wall-clock rispetto
alla baseline e giustifica il costo di un secondo backend.

### Consegna v1-engine

- nuovo bundle e manifest;
- engine MuJoCo CPU e MJX;
- replay e hybrid;
- API batched;
- harness parity A-E;
- feature matrix aggiornata per la parte engine;
- benchmark e profiling;
- rollout scripted batch/singolo;
- report dei gate.

La v1-engine non equivale al porting completo: mancano trainer, UX e gate
end-to-end.

---

## Fase 6 - PPO JAX, rollout e UX

### Scelta trainer

Eseguire uno spike tra Brax training e Rejax. PureJaxRL resta riferimento
implementativo, non dipendenza predefinita.

Il trainer scelto deve supportare:

- env custom completamente JIT;
- PPO continuo;
- actor-critic asimmetrico reale;
- `terminated`/`truncated` nel bootstrap GAE;
- multi-env train/eval;
- checkpoint/resume;
- logging fuori dal percorso caldo.

Scelta predefinita: Brax training se supera tutti i gate, altrimenti Rejax.

### Config e compatibilita strutturale

Portare `training_cfg.yaml` come sorgente unica. Il target puo aggiungere una
sezione backend, ma non deve obbligare l'utente a duplicare valori.

Il training salva:

```text
training_cfg.resolved.yaml
```

Il rollout dato soltanto `--checkpoint` deve auto-caricare:

- config env;
- observation schema;
- rete;
- reward;
- reward mode;
- action mode;
- profilo/semantica GRF;
- precisione necessaria;
- actor symmetric/asymmetric.

### Artefatti obbligatori

- checkpoint `best` e `last`;
- policy inference-only;
- `summary.json`;
- `train_iterations.jsonl` o equivalente;
- rollout summary;
- TensorBoard;
- manifest e schema version;
- stato supervisor/watchdog;
- config risolta.

L'export inference-only dell'attore asimmetrico non deve contenere o richiedere
il value tower privilegiato.

### Supervisione e robustezza

Portare l'idea strutturale della baseline:

- timeout startup/sampling/iteration/checkpoint/cleanup;
- watchdog con heartbeat;
- self-test watchdog;
- restart da ultimo checkpoint valido;
- limite anti-loop sui crash senza progresso;
- progress live;
- cleanup del processo figlio;
- nessun callback nel loop JIT.

Gli strumenti possono cambiare rispetto a RLlib/Ray, ma il comportamento utente
e la robustezza devono restare equivalenti.

### Comandi utente obbligatori

Fornire wrapper semplici documentati:

```text
python scripts/train_ppo_mlp.py --output-dir ...
python scripts/train_ppo_mlp.py --resume-from ... --output-dir ...
python scripts/rollout_eval.py --checkpoint ... --output-dir ...
python scripts/rollout_eval.py --checkpoint ... --output-dir ... --record-outputs
python scripts/run_parity.py ...
python scripts/run_benchmark.py ...
python scripts/run_mjx_batch.py ...
```

Documentare esempi Windows, macOS CPU/debug e WSL2 GPU. L'utente non deve
modificare manualmente path interni o rispecificare nel rollout parametri gia
presenti nel checkpoint/config risolta.

### Gate UX e feature parity

- matrice `feature_parity.yaml` senza `missing/partial` dentro scope;
- training standard avviabile con config + output dir;
- rollout standard avviabile con checkpoint + output dir;
- auto-config rollout verificato;
- override CLI/config verificati;
- warm-start imitation-to-ex-novo verificato con checkpoint target;
- symmetric e asymmetric verificati;
- recorder STO opzionale verificato;
- watchdog self-test PASS;
- restart da checkpoint PASS;
- progress e logging PASS;
- artefatti equivalenti presenti.

---

## Fase 7 - Gate finali di training e inference

I benchmark finali devono includere la policy e, per il training, l'update PPO.

### Policy-only inference

Esportare e confrontare l'attore inference-only con stessa architettura e stessi
pesi su input actor-realistic congelati:

- latenza batch `1` p50/p95 non peggiore della policy RLlib/Torch sorgente;
- throughput al batch `B*` almeno `5x` rispetto alla policy RLlib/Torch sulla
  piattaforma di riferimento;
- output deterministici equivalenti entro le tolleranze numeriche della
  conversione/export;
- nessuna dipendenza dal critic privilegiato.

### Inference deterministica batch 1

Con stessa rete, stesso episodio e recorder disabilitato:

```text
wall-clock rollout MJX batch 1 <= 0.5x wall-clock rollout OpenSim batch 1
```

Riportare anche latenza p50/p95 del policy-step e real-time factor.

### Rollout batched con policy

```text
rollout env-step/s MJX + policy a B* >= 5x OpenSim 12-worker + policy
```

### Training PPO end-to-end

Con stesso numero di env-step, stessa rete, PPO semanticamente equivalente e
logging minimo equivalente:

```text
wall-clock sample + PPO update MJX <= 0.333x baseline OpenSim/RLlib
```

Equivale ad almeno `3x` speedup end-to-end. Riportare separatamente:

- sampling;
- update PPO;
- checkpoint;
- logging;
- compile iniziale;
- memoria host/GPU.

### Gate funzionali finali

- smoke PPO symmetric e asymmetric;
- training breve con reward finita e parametri aggiornati;
- checkpoint/resume round-trip;
- rollout deterministico batch `1`;
- rollout batch;
- nessuna feature privilegiata letta dall'attore;
- stessa action shape, observation spec, reward mode e config tra train/rollout;
- nessun fallback CPU/SciPy nel percorso caldo;
- gate scientifici strict ancora PASS sul commit finale.

### Gate cross-platform

- Windows x86: import, test CPU/debug, config, parity tooling e rollout CPU.
- macOS arm64: import, test CPU/debug, config, parity tooling e rollout CPU.
- WSL2/Linux RTX 4070: MJX, benchmark, rollout e training GPU.

Le modifiche GPU-specifiche non devono rompere import o tooling condiviso sugli
altri sistemi.

---

## Ordine operativo definitivo

1. Eseguire preflight e proteggere lo stato di `MuJoCo_env`.
2. Creare la matrice source-to-target.
3. Eseguire Fase 0 e produrre GO/NO-GO GPU/hybrid.
4. Generare e congelare gli oracle OpenSim canonici.
5. Creare il nuovo bundle `321/500_pi` senza sovrascrivere quello storico.
6. Chiudere validazione statica.
7. Chiudere B1a e B1b SEA.
8. Portare replay CMC-like e chiudere B2/D1.
9. Eseguire checkpoint strategico.
10. Se `GO_STRICT`, portare e validare hybrid C0/C/D2.
11. Costruire e validare ambiente batched E.
12. Superare benchmark v1-engine e consegnare v1-engine.
13. Integrare trainer PPO, rollout, supervisione e UX.
14. Chiudere feature matrix, gate UX e cross-platform.
15. Superare benchmark finali inference/training.
16. Dichiarare completato il porting 1:1 soltanto se tutti i gate strict passano.

---

## Regole di stop ed escalation

Fermarsi e produrre un report decisionale se:

- manca una primitiva MJX necessaria;
- il contatto hybrid non e applicabile on-device;
- B1a fallisce;
- B2 non e chiudibile senza cambiare semantica OpenSim;
- il plant biologico richiede un sotto-progetto di ricerca;
- nessun candidato hybrid supera C/D2;
- il massimo batch stabile e `< 32`;
- lo speedup engine e `< 5x`;
- lo speedup training end-to-end e `< 3x`;
- il porting richiede una UX piu complessa della baseline senza beneficio
  approvato;
- Windows/macOS CPU/debug vengono rotti.

Il report deve proporre opzioni e costi. Non allargare soglie, cambiare workload
o declassare mismatch scientifici dopo aver visto i risultati senza
approvazione esplicita.

---

## Deliverable finali

### Codice e runtime

- bundle `AB06_SEASEA_stiff321_500_pi`;
- engine MuJoCo CPU;
- engine MJX batched;
- contatto hybrid;
- ambiente Trajectory Generator;
- PPO JAX;
- rollout/inference;
- supervisione;
- comandi semplici.

### Validazione

- oracle canonici;
- manifest;
- feature parity matrix;
- report A/B1/B2/C/D1/D2/E;
- report cross-platform;
- checkpoint round-trip;
- smoke training/rollout.

### Prestazioni

- benchmark engine;
- benchmark inference batch `1`;
- benchmark rollout batch;
- benchmark training end-to-end;
- compile time e memoria.

### UX e documentazione

- README aggiornato;
- comandi Windows/macOS/WSL2;
- descrizione config;
- descrizione artefatti;
- guida troubleshooting;
- limiti scientifici residui dichiarati.

---

## Riferimenti locali principali

### OpenSim source

```text
config.py
model_loader.py
online_grf.py
prosthesis_controller.py
outer_loop.py
inverse_dynamics.py
static_optimization.py
simulation_runner.py
output.py
tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/env_factory.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/process_watchdog.py
Trajectory Generator/baseline_MLP/progress_display.py
Trajectory Generator/baseline_MLP/tb_logging.py
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/baseline_MLP/commands.txt
```

### MuJoCo_env target

```text
AGENTS.md
CONTEXT.md
README.md
mujoco_env/config.py
mujoco_env/model_loader.py
mujoco_env/sea_plugin.py
mujoco_env/prosthesis_controller.py
mujoco_env/outer_loop.py
mujoco_env/inverse_dynamics.py
mujoco_env/static_optimization.py
mujoco_env/simulation_runner.py
mujoco_env/mjx_runner.py
mujoco_env/trajectory_env.py
mujoco_env/output.py
mujoco_env/parity.py
tools/osim2mjcf.py
tools/validate_model.py
tools/parity_test.py
scripts/run_simulation.py
scripts/run_mjx_batch.py
scripts/run_rl_smoke.py
tests/
```

## Riferimenti runtime

- [JAX installation](https://docs.jax.dev/en/latest/installation.html)
- [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
- [Brax](https://github.com/google/brax)
- [Rejax](https://github.com/keraJLi/rejax)
- [PureJaxRL](https://github.com/luchris429/purejaxrl)
