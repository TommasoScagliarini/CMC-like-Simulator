# Calcolo online delle GRF tramite modello di contatto

> Documento di analisi e proposta implementativa.
> Obiettivo: rendere il simulatore in grado di **calcolare le Ground Reaction
> Forces (GRF) in funzione dello stato** (contatto piede-suolo), invece di
> riprodurle da dati sperimentali prescritti. Fine ultimo: **deployability** del
> generatore di traiettorie protesiche e rilevazione **online** del gait cycle
> (heel strike) per definire i confini degli episodi RL.

## Stato implementazione

Il simulatore ora espone tre modalità selezionabili:

- `prescribed`: comportamento CMC-like storico, invariato;
- `online_sensor`: le ExternalLoads prescribed guidano la dinamica, mentre il
  contatto online viene letto come sensore e validato;
- `online`: il contatto online carica il modello; le ExternalLoads sono un
  oracle opzionale e possono essere omesse con `--no-external-loads`.

Il contatto treadmill-aware vive nel componente separato
`tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce`, non nel plugin SEA.
Geometria e materiale sono descritti da profili JSON in `online_grf_profiles/`.
Il runner salva `<prefix>_online_grf.sto` e
`<prefix>_gait_events_online.csv`.

La calibrazione/validazione con split calibration/holdout è implementata in
`validation/validate_online_grf.py`; l'inferenza iniziale dai marker è
disponibile in `validation/generate_online_grf_profile.py`.

La calibrazione intensiva per `online_sensor` è disponibile in:

- `validation/calibrate_online_grf_intensive.py`: fit nonlineare per contatto;
- `validation/calibrate_online_grf_basis.py`: ricerca di una patch sparsa con
  regressione non-negativa;
- `validation/verify_online_grf_plugin.py`: prova di equivalenza tra formula
  Python e plugin C++;
- `validation/compare_online_grf_output.py`: confronto di un rollout salvato
  con le ExternalLoads prescribed.

Il materiale può essere configurato per singola sfera e include un esponente
forza-penetrazione calibrabile, con default retrocompatibile `1.5`.

### Validazione plugin macOS arm64

Il TODO di portabilità macOS è stato chiuso il 2026-06-08. Il plugin
`OnlineGRFContact` è stato compilato in Release contro
`/Users/tommy/opensim-core-install`, installato localmente come
`plugins/libOnlineGRFContact.dylib` e verificato come Mach-O arm64.

Verifiche completate:

- `LC_RPATH` punta alla stessa installazione OpenSim usata dal binding Python;
- firma ad hoc valida e caricamento tramite `opensim.LoadOpenSimLibrary`;
- registrazione del tipo custom `OnlineGRFSphereHalfSpaceForce`;
- audit formula Python/plugin C++ equivalente entro `1e-8 N`, con errore
  massimo `1.73e-12 N`;
- smoke AB06 `online_sensor` e `online` senza ExternalLoads completati;
- smoke adapter RL onlineGRF completato con heel strike confermati.

Non sono state necessarie modifiche al sorgente C++ o al plugin SEA.

### Esito calibrazione AB06

Il miglior profilo incluso per AB06 è
`online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json`.
Su holdout del replay IK ottiene RMSE verticale `60.70 N` a sinistra e
`31.81 N` a destra, pari a `7.72%` e `4.17%` del picco. Formula Python e plugin
C++ coincidono con errore massimo osservato `0.0 N`.

La calibrazione sugli stati forward prescribed non generalizza quando lo stato
simulato diverge dall'IK: le ExternalLoads rimangono time-driven e possono
applicare una GRF non coerente con il contatto corrente. In quel regime
annullare l'errore richiederebbe usare esplicitamente tempo/oracolo e non sarebbe
più un calcolo online state-based. Per il training, `online_sensor` resta la
modalità prioritaria; `online` puro ha completato un rollout di `0.5 s`, ma con
reserve più elevate e crescenti.

### TODO prioritizzato

- La riproduzione esatta della magnitudo GRF richiede una patch plantare 3D
  e/o un correttore residuale state-based ed è rinviata.
- La priorità corrente è validare che la GRF verticale `online_sensor`
  identifichi gli heel strike senza eventi falsi o mancati e con timing
  concorde alle GRF prescribed, così da scandire i gait cycle della rete.

### Validazione heel strike AB06

Il detector streaming usa ora una doppia soglia model-specific:

- `20 N` per memorizzare il timestamp iniziale del contatto;
- `190 N` per confermare che non si tratti di un picco spurio durante lo swing;
- `0.05 s` di durata minima.

Sul replay IK completo rileva `12/12` heel strike prescribed, senza falsi
positivi o falsi negativi, con timing MAE `10 ms` e massimo `30 ms`. Sugli
stati forward prescribed di `2 s` rileva `3/3` eventi osservabili. La latenza di
conferma è distinta dal timestamp riportato ed è circa `67 ms` medi sul replay
IK. Il valore `190 N` è salvato nel profilo AB06 e deve essere ricalibrato per
ogni nuovo modello.

---

## 1. Osservazioni sul simulatore

### 1.1 Le GRF oggi sono *prescritte*, non calcolate
Il contatto col suolo non esiste come grandezza fisica calcolata. La GRF è
imposta dall'esterno tramite un `ExternalForce` legato a una `Storage` letta da
un file `.mot` sperimentale, riprodotta **in funzione del tempo** e
**indipendente dalle azioni della policy**.

- Caricamento: `model_loader.py` (~righe 707–817, 1058/1061) — la `.mot` GRF è
  ricavata dal `<datafile>` dell'`ExternalLoads` XML, caricata in
  `opensim.Storage` e legata a oggetti `ExternalForce`.
- Metadati esposti nel contesto: `ctx.grf_data_file` e
  `ctx.grf_vertical_force_columns = {"left": "<id>y", "right": "<id>y"}`.
- Rilevazione cicli (offline): `output.py` → `_cycles_from_vertical_grf`
  (righe 154–207) + `_write_gait_events_csv` (righe 210–279). Produce
  `<prefix>_gait_events.csv` con `side, cycle_start, cycle_end, cycle_duration_s,
  contact_duration_s, source_force, threshold_n, ...`.

**Conseguenza chiave:** poiché gli heel strike derivano dalla GRF prescritta e
non si spostano in base alla policy, "rilevarli online" leggendo il segnale
prescritto è **equivalente** a precalcolarli. Un calcolo online *significativo*
(state/policy-dipendente, deployment-relevant) richiede una GRF che sia funzione
dello stato → un **modello di contatto**.

### 1.2 La dinamica CMC-like dipende dalla GRF prescritta
La inverse dynamics calcola l'accelerazione baseline `q̈₀` con **tutti gli
attuatori azzerati**, via `realizeDynamics + InverseDynamicsSolver`
(`inverse_dynamics.py`, righe 1–31). Gli external loads prescritti sono già
inclusi in `q̈₀`. Poi:

    τ = M · (q̈_des − q̈₀)
    τ_bio  → muscoli + reserve (QP di static optimization)
    τ_pros → oracolo diagnostico DOF protesici

Implicazioni per il contatto:
- **A favore:** qualsiasi forza aggiuntiva (incluso un contatto state-dependent)
  viene inclusa automaticamente in `q̈₀` durante `realizeDynamics`. La matematica
  dell'ID **regge** senza riscritture.
- **Rischio:** se la GRF di contatto a un dato stato è sbagliata (geometria piede/
  pavimento non calibrata → penetrazione o distacco), `q̈₀` viene inquinato →
  `τ_bio` infeasible → **reserve che esplodono** (modo di fallimento già visto
  storicamente).
- **Nota crash:** `realizeAcceleration()` è evitato di proposito perché fa
  crashare il plugin SEA. Le forze di contatto si risolvono a stage **Dynamics**,
  quindi il bypass ID resta valido (nessun nuovo crash).

### 1.3 Catena temporale e loop
- `simulation_runner.step_until(t_stop)` (righe 255–327): finestra di controllo
  `T_control` (default 0.001 s), `integration_dt` 0.001 s, integratore
  `rk4_bypass` / semi-implicit Euler. Controlli calcolati una volta per finestra,
  poi N sub-step a ZOH.
- API pubbliche già presenti per uso RL: `reset_to_time`, `step_until`,
  `current_time`, `last_step_info`, `save_results`, `reset_outputs`.
- L'env RL (`Trajectory Generator/osim_trj_cmc_like.py`) avanza `segment_duration`
  (default 0.05 s) per step → ~50 sub-step di integrazione.

---

## 2. Osservazioni sul modello

### 2.1 Nessuna geometria di contatto presente
Il `<ContactGeometrySet>` dei modelli `.osim` è **vuoto** (solo placeholder
`<objects/>`). Va costruito da zero.

### 2.2 Versione e classi disponibili
- **OpenSim 4.5.2** (`4.5.2-2026-02-20-...`).
- Disponibili: `SmoothSphereHalfSpaceForce`, `HuntCrossleyForce`,
  `ContactSphere`, `ContactHalfSpace`, `ContactGeometry`.
- Per la simulazione *forward* si preferisce **`SmoothSphereHalfSpaceForce`**
  (forza C∞, derivabile, integra meglio e non rompe l'integratore come i contatti
  rigidi/non-smooth).

### 2.3 Il modello è asimmetrico (cruciale per dove mettere il contatto)
| Lato | Tipo | Corpi piede |
|------|------|-------------|
| **Destro** | intatto / biologico | `talus_r`, `calcn_r`, `toes_r` (piede a 3 segmenti, geometria completa) |
| **Sinistro** | **protesi** | `transfemur`, `osseo_pylon`, `tibia_pylon`, **`foot_l`** (corpo piede singolo) |

- **Lato protesico = SINISTRO** (`foot_l`). (Correzione rispetto a un'ipotesi
  precedente che lo dava a destra.)
- Il piede sano (destro) ha la geometria migliore/più semplice su cui appoggiare
  le sfere di contatto; il piede protesico è un corpo unico.
- Le colonne GRF in `ctx.grf_vertical_force_columns` sono keyed `left`/`right`.

### 2.4 Treadmill
Alcuni modelli sono su tapis roulant (`AB06_SEASEA_Threadmill`). Un
`ContactHalfSpace` statico complica la componente **tangenziale/attrito** (il
nastro si muove). La **componente verticale** — quella che serve per l'heel
strike — è invece robusta a questo. È un motivo forte per separare i due usi del
contatto (sensore verticale vs dinamica completa).

---

## 3. Il bivio progettuale: come il contatto si accoppia alla dinamica

Questa scelta determina il rischio (ordine di grandezza).

### B1 — Contatto come "sensore virtuale" (raccomandato come primo passo)
- Gli **ExternalLoads prescritti continuano a guidare la dinamica** (stabile,
  validata).
- Si aggiungono le forze di contatto al modello ma **non** le si usa per caricare
  lo scheletro nel recruitment CMC-like: se ne legge solo la **componente
  verticale** per rilevare l'heel strike online.
- La detection diventa **state/policy-dipendente** (via la posa del piede
  protesico simulato) → rilevante per il deploy — **senza** destabilizzare ID/SO.
- Rischio basso, valore immediato: si ottiene subito un detector online su cui
  allenare.

### B2 — Contatto che carica davvero lo scheletro (end-state futuro)
- Si rimuovono gli ExternalLoads prescritti; il contatto calcolato diventa la
  GRF fisica.
- Massima fedeltà al deploy, ma riscrive le assunzioni della dinamica: `q̈₀`
  contiene il contatto, la geometria piede/pavimento **deve inseguire** la
  traiettoria IK del piede o si ottengono picchi di penetrazione/distacco →
  `τ_bio` infeasible → reserve in saturazione.
- Su treadmill servirebbe anche la gestione della velocità del nastro
  (attrito tangenziale).
- **Merita un piano dedicato**, da affrontare solo dopo che il modello di
  contatto è calibrato in B1.

> **In entrambi i casi la GRF prescritta resta l'ORACOLO di validazione:** il
> contatto calcolato deve riprodurre il *timing* sperimentale degli heel strike
> prima di poter pilotare la terminazione degli episodi.

---

## 4. Proposta implementativa per il calcolo online delle GRF

### Fase A — Costruzione e calibrazione del modello di contatto (offline, no RL)

**Cosa aggiungere (programmaticamente, NON editando i `.osim`):**
- 1 `ContactHalfSpace` come pavimento (per il treadmill: piano statico
  all'altezza misurata del suolo).
- `ContactSphere` su:
  - `calcn_r` (tallone destro) e `toes_r` (avampiede destro) — lato sano;
  - `foot_l` (2–3 sfere: tallone, mesopiede, avampiede) — lato protesico.
- 1 `SmoothSphereHalfSpaceForce` per ciascuna sfera (sfera ↔ half-space).

**Dove:** in `model_loader.py`, dopo la costruzione del modello, dietro un nuovo
flag di config `enable_foot_contact` (vedi §5). Costruzione via API OpenSim
(`opensim.ContactHalfSpace(...)`, `opensim.ContactSphere(...)`,
`opensim.SmoothSphereHalfSpaceForce(...)`, `model.addContactGeometry(...)`,
`model.addForce(...)`). Così è **cross-OS** (Windows x86 + macOS arm64) e vale per
tutte le varianti di modello, senza toccare i file `.osim`.

**Parametri da calibrare:** raggi e offset delle sfere, altezza/orientamento del
piano, `stiffness`, `dissipation`, `static_friction`/`dynamic_friction`/
`viscous_friction`, parametri di transizione smooth.

**Criterio di calibrazione (oracle-anchored):** guidando il modello con la
**cinematica IK sperimentale**, la GRF verticale **calcolata** deve riprodurre la
GRF **sperimentale** (file `.mot`) in:
1. magnitudo del picco (per lato), e soprattutto
2. **timing del threshold-crossing** (heel strike / toe off) entro tolleranza
   (es. < un control step), per ogni ciclo.

**Deliverable:** script in `validation/` che sovrappone GRF calcolata vs
prescritta e riporta l'errore di timing per ciclo, per lato.

### Fase B — Detector heel-strike online (streaming)

- Estrarre il *core* di threshold-crossing oggi dentro
  `_cycles_from_vertical_grf` in una funzione condivisa, così detection
  **online** e **offline** usano un'unica definizione.
- Macchina a stati per-lato sulla forza di contatto **verticale calcolata**, con
  gli stessi parametri già in `config.py` (`grf_contact_threshold_n`,
  `grf_min_contact_duration_s`, `grf_min_cycle_duration_s`):
  - **Heel strike** = salita oltre soglia, sostenuta ≥ `min_contact` (senza
    lookahead, a differenza della versione offline che definisce il ciclo
    guardando l'HS successivo).
  - Emette evento "HS confermato" + forza verticale corrente in
    `last_step_info`.
- Lettura online della forza: `SmoothSphereHalfSpaceForce` espone il proprio
  record (oppure si sommano i contributi delle sfere) allo stato corrente, dentro
  `simulation_runner.step_until` (righe 255–327).
- **Validazione contro l'oracolo** (cicli da GRF prescritta): latenza di
  detection e tasso di falsi positivi/negativi, su rollout di policy **buone e
  cattive**, offline, prima che il detector piloti qualunque cosa.

### Fase C — Uso come trigger di episodio (lato env RL)

(Dettaglio completo nel file di piano; qui i punti che dipendono dalla GRF.)
- Sorgente selezionabile: `gait_cycle_source = "prescribed" | "contact"`
  (oracolo vs online).
- Lato di riferimento configurabile, default piede **sano (destro)** per la
  detection iniziale: è guidato dall'IK biologica → segnale stabile, evita un MDP
  non-stazionario; passare al piede protesico/`contact` quando la policy è matura.
- **Fine ciclo → troncatura SENZA penalty ma con bootstrap del valore** (il
  cammino è periodico, non assorbente: il valore all'HS va bootstrappato, non
  azzerato). Cap di durata leggermente > ciclo atteso come rete di sicurezza se
  un HS viene mancato. Divergenza/caduta → troncatura **con** penalty (invariato).
- `phase` vera = `(t − cycle_start)/(cycle_end − cycle_start)`; con
  `source="contact"` il `cycle_end` non è noto in anticipo → normalizzare con la
  durata attesa stimata dal ciclo corrente (phase 0→1 limitata).

---

## 5. Punti di intervento (riassunto)

| Area | File | Modifica |
|------|------|----------|
| Costruzione contatto | `model_loader.py` | Aggiunta programmatica di `ContactHalfSpace` + `ContactSphere` + `SmoothSphereHalfSpaceForce`, dietro flag |
| Flag config | `config.py` | `enable_foot_contact` (+ eventuali parametri contatto) |
| Detection condivisa | `output.py` | Estrarre core threshold-crossing riusabile online/offline |
| Forza online + evento | `simulation_runner.py` | Leggere forza contatto e HS in `last_step_info` durante `step_until` |
| Calibrazione/validazione | `validation/` (nuovo) | Overlay GRF calcolata vs prescritta + errore di timing |
| Trigger episodio | `Trajectory Generator/osim_trj_cmc_like.py` | `episode_mode`, `gait_cycle_source`, lato, cap, semantica truncation+bootstrap, phase vera |
| Wiring training | `Trajectory Generator/baseline_MLP/{env_factory,train_ppo_mlp,tb_logging}.py` | Passaggio flag + log lunghezza ciclo |

---

## 6. Criticità principali (sintesi)

1. **Calibrazione geometrica** = make-or-break. Geometria piede/pavimento non
   allineata all'IK → contatto sbagliato → (in B2) `τ_bio` infeasible.
2. **Accoppiamento dinamico** solo in B2; B1 lo evita tenendo i carichi prescritti
   per la dinamica e usando il contatto come sensore.
3. **MDP non-stazionario** se si rileva dal piede protesico (policy-dipendente,
   knee storicamente instabile a inizio training) → mitigare con piede sano e/o
   `source="prescribed"` finché la policy non è decente.
4. **Treadmill**: attrito tangenziale complica B2; la verticale per la detection è
   robusta.
5. **Semantica `terminated` vs `truncated`**: fine-ciclo deve bootstrappare il
   valore, non azzerarlo (adiacente al bug F5 noto nel ramo SNN, fuori scope qui;
   RLlib lo gestisce correttamente).
6. **Crash SEA su `realizeAcceleration`**: non re-innescato (contatto a stage
   Dynamics, bypass ID preservato).

## 7. Sequenza raccomandata
**A (calibra) → B (detector validato vs oracolo) → C con `source="prescribed"`
→ C con `source="contact"` → D (training).** Tenere **B1 (contatto come sensore)**
come target a breve; **B2 (dinamica di contatto completa)** rinviata a un piano
dedicato. Non è una singola PR: è un lavoro multi-fase da validare a ogni passo.
