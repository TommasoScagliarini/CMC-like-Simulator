# Daily report - 2026-06-01

## Sintesi

Giornata su tre linee:

1. infrastruttura di training SNN/PPO dentro `Trajectory Generator/Prosthesis_SNN`
   (wrapper actor-critic skrl, checkpoint inference-only, entrypoint smoke);
2. diagnosi e soluzione della saturazione del knee nell'env RL di traiettoria
   (limit-cycle ~35 Hz) tramite filtro di riferimento 6 Hz lato simulatore;
3. code review dell'intero repository con report dedicato.

Report utente prodotti oggi:

```text
reports/user/2026-06-01_actor_critic_snn_skrl_reward_chiarimento.md
reports/user/2026-06-01_checkpoint_inference_path_snn.md
reports/user/2026-06-01_training_entrypoint_ppo_snn_cmc_like.md
reports/user/2026-06-01_knee_saturazione_env_rl_limit_cycle.md
reports/user/2026-06-01_filtro_riferimento_6hz_lato_simulatore.md
```

Report di review (radice repo):

```text
BUG_6-1-26.md
```

## 1. Infrastruttura training SNN/PPO

### Wrapper actor-critic skrl
Aggiunto `ProsthesisSNNActorCritic` (`training/actor_critic.py`): backbone SNN
condiviso, due teste non-spiking LIF (policy/value), stato membrana esposto come
`rnn`, cache tra chiamata policy e value, stesso oggetto usabile come
`models["policy"]` e `models["value"]`. La policy head usa il formato di
`ProsthesisReferenceSNN`, esportabile verso `ReferenceGenerator`. Aggiunte
compatibilita' per `skrl 2.1.0`.

Chiarito che la reward dell'env **esiste gia'** (`osim_trj_cmc_like._get_reward`,
con tracking/reference/bio/effort/smoothness e truncation in `_is_truncated`):
la voce "reward minimale" significa "presente ma da validare/tarare", non
assente.

### Checkpoint inference-only
Aggiunto `build_reference_checkpoint` / `save_reference_checkpoint`
(`training/checkpoint.py`): esporta solo la rete reference (no value head, no
`log_std_parameter`, no optimizer/skrl), caricabile con
`ReferenceGenerator.from_checkpoint`.

### Training entrypoint smoke
Aggiunto `training/cmc_ppo_smoke.py`, entrypoint model-agnostic
(`--setup-xml` o setup persistito) che collega env -> RandomMemory ->
`ProsthesisSNNActorCritic` -> `PPO_SNN` -> una update PPO -> checkpoint agent ->
reference checkpoint -> reload -> predict. `ppo_snn.py` aggiornato per accettare
`trainer_cfg` dict (skrl 2.1.0). Verificato anche resume/load.

Importante (TODO propagato): l'azione dell'env e' un set di knot di traiettoria
`(policy_knots, n_coords)`, non ancora il contratto SNN inteso
`(q, qdot, qddot) x (knee, ankle)`. Mismatch ancora aperto.

## 2. Saturazione knee env RL: diagnosi e filtro 6 Hz

### Diagnosi
Il knee satura (`|u|>=0.999`) mentre l'ankle no, e i dati sperimentali non
saturano. Causa: la cascade del knee e' ~26x piu' sensibile all'errore di
velocita' dell'ankle (`Kp_inner/F_opt` = 29.2/100 vs 2.83/250). Un riferimento a
banda larga eccita una risonanza knee a **~35 Hz** -> limit-cycle bang-bang del
comando, `qddot_ref` enorme (`~1/seg^2`) che accoppia nell'inverse dynamics
biologico, recruitment infeasible, reserve a ~46x e scheletro che vibra. I dati
sperimentali non saturano perche' l'IK e' gia' filtrata a 6 Hz.

Approfondimento (b): l'oscillazione residua del knee sotto LPF sul comando e'
instabilita' whole-body **sostenuta** (non transitorio, non comando-driven):
verificato che a LPF 10 Hz `max|u|` scende a 0.63 ma `qd_cur` resta ~25 rad/s.

### Soluzione adottata
Su indicazione esplicita (la rete genera solo un riferimento smooth, il
simulatore lo filtra come l'IK), aggiunto un **filtro di riferimento 6 Hz lato
sim** in `ProstheticSegmentKinematics` (reference-model 2 ordine, `zeta=1`,
cutoff = `kinematics_lowpass_cutoff_hz`, stato continuo tra segmenti). Nuovi
campi `CMCEnvConfig`: `enable_pros_ref_lpf`, `pros_ref_lpf_cutoff_hz`,
`pros_ref_lpf_zeta`. `max_delta_rad` ripristinato a 0.35 (full authority): un
cap asimmetrico provato prima e' stato superato dal filtro.

Esiti: 0% saturazione a full authority col default, robusto a qualsiasi
`knots/seg` (salva anche `seg=0.01` che prima divergeva), giunto ~1.3 rad/s,
reserve ~2x, reward ~0.99. Il rollout esistente passa da 21/30 a **0/30**
saturati e resta bitwise-deterministico.

Decisione architetturale salvata in memoria di progetto (`env-pros-ref-6hz-filter`).

## 3. Code review repository (BUG_6-1-26)

Analisi statica di sim e trajectory generator + analisi congiunta, con verifica
a runtime dei punti chiave. Finding principali:

- **S1 [Media]** `kinematics_interpolator._read_sto` ignora il flag `inDegrees`
  (sempre deg->rad), incoerente con `output.read_sto`. Latente.
- **T1 [Media][verificato]** il checkpoint del training non pilota il provider
  d'integrazione: `SNNProsthesisReferenceProvider.get()` solleva `KeyError`
  (feature obs vs phase) e i canali sono knot/joint invece di q/qdot/qddot.
- **T2 [Media-bassa]** cache forward su `data_ptr()` in `actor_critic.compute`
  (hazard stale-pointer).
- **J1 [Media][verificato]** contratto a tre incompatibile env<->SNN<->provider;
  inoltre l'env non consuma qdot/qddot diretti (usa solo knot di q).
- Altri minori: `sea_stiffness` default fuorviante, `InverseDynamicsSolver`
  ricreato ogni frame, reshape BPTT solo `num_envs=1`, spike_grad non passato a
  rate/latency encoder, gain non per-modello. Dettagli e priorita' in
  `BUG_6-1-26.md`.

## File modificati / creati

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/actor_critic.py   (nuovo)
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/checkpoint.py     (nuovo)
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_smoke.py  (nuovo)
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py        (aggiornato)
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py
Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py
Trajectory Generator/Prosthesis_SNN/README.md
Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md
Trajectory Generator/osim_trj_cmc_like.py   (filtro 6 Hz + config; max_delta a 0.35)
validation/rl_env_knee_sat_diag.py   (nuovo)
validation/rl_env_knee_fix_sweep.py  (nuovo)
BUG_6-1-26.md  (nuovo, code review)
reports/user/2026-06-01_*.md  (5 report)
```

## Verifiche eseguite

```text
smoke_test.py (SNN)                         -> smoke tests passed
cmc_ppo_smoke (end-to-end + resume)         -> ok:true, reload finite
rl_env_smoke_ab06_pi.py                     -> rl_env_smoke_ab06_pi_ok
rl_env_rollout_ab06_pi.py --skip-cost       -> Test L 0/30 sat, Test R deterministico
rl_env_knee_sat_diag.py / knee_fix_sweep.py -> riproducono 21/30 e confermano le tabelle
py_compile sui file modificati              -> ok
contract mismatch (checkpoint->provider)    -> KeyError verificato
```

## TODO chiusi oggi

- Wrapper skrl actor-critic con backbone SNN condiviso (era aperto dal 2026-05-31).
- Training entrypoint env -> memory -> actor-critic -> PPO_SNN -> checkpoint ->
  reload (chiuso a livello smoke).
- Checkpoint inference-only e reload via `ReferenceGenerator`.
- Smoke test end-to-end training.
- Diagnosi causa saturazione knee (limit-cycle ~35 Hz) e relativo fix (filtro 6 Hz).
- Approfondimento (b): origine dell'oscillazione residua sotto LPF (instabilita'
  whole-body).
- Action design knee lato env: la saturazione e' eliminata dal filtro 6 Hz a
  full authority (sostituisce il cap `max_delta`).

## TODO aperti e propagati

### Trajectory Generator / SNN / PPO
- Creare script di training e inference **veri** (production), oltre agli smoke
  attuali (`cmc_ppo_smoke`, `examples/smoke_inference.py`). Aggiunto in
  `docs/TODO_integration.md`.
- Risolvere il **mismatch di contratto** azione/output/feature: env (knot di q,
  delta-mode) vs SNN inteso `(q,qdot,qddot)` vs provider (feature di fase). E' il
  prerequisito per un training utile (vedi BUG_6-1-26 T1/J1).
- Decidere la tecnica di training (RL puro / hybrid SL+RL / imitazione->RL /
  staged); in `TODO_integration` c'e' una decisione parziale (hybrid).
- Definire l'observation space (stato completo vs sensori deployable vs ibrido).
- Definire/validare normalizzazione input/output, feature names, unita',
  scaling, output transform, action limits, metadata checkpoint.
- Tarare/validare la reward su rollout lunghi: tracking, smoothness, effort,
  coerenza soft q/qdot/qddot, **penalita' esplicita sulla saturazione di u**,
  soglie di truncation adatte al training.
- Collegare l'export del checkpoint inference-only al training entrypoint (export
  automatico a fine training).

### Env RL / simulatore
- Validare il filtro 6 Hz con una policy reale (azioni piu' ampie della
  sinusoide 0.10) e su rollout lunghi (gait completo).
- Verificare il **lag causale** del filtro sul tracking dell'IK (l'IK e' filtrata
  a fase-zero, il riferimento online no): valutare lead/anticipo o termine reward.
- Eventuale taratura `pros_ref_lpf_cutoff_hz`/`pros_ref_lpf_zeta` se la policy
  necessita di piu' banda.
- Target/bounds biologici della static optimization all'init AB06:
  `capable_share < 1` e warning QP nei primi step (linea aperta dal 2026-05-29).
- Decidere il layout Git: confermare lo spostamento di `osim_trj_cmc_like.py` in
  `Trajectory Generator/` e aggiornare riferimenti/packaging.
- Aggiungere `build/` al `.gitignore` o decidere come gestirla.
- Decidere se includere `LLM_SIMULATOR_OVERVIEW.md` in `CONTEXT.md` o nel flusso
  `start_day`.

### Bug review (BUG_6-1-26) - da confermare/correggere
- S1: uniformare i reader `.sto` sul flag `inDegrees`.
- T2: irrobustire o rimuovere la cache su `data_ptr()` in `actor_critic`.
- T3: validare il reshape BPTT prima di passare a `num_envs > 1`.
- T4: passare lo spike_grad custom anche a rate/latency encoder.
- S2/J2: allineare `sea_stiffness` default e valutare gain per-modello.
- S3: riusare `InverseDynamicsSolver` invece di ricrearlo ogni frame.

### Propagati storici (linea controllo SEA, dal 2026-05-31 e precedenti)
- Sweep `Kp_knee_motor` su valori intermedi tra 3.9 e 18 mantenendo ankle
  morning best.
- Validare diagnosi di coupling knee-ankle isolando la dinamica knee dal
  feedback ankle.
- Notch a 28 Hz sul feedback knee (lato controllore) come opzione, separato dal
  filtro env appena introdotto.
- Cleanup modelli sperimentali (`slow_inner_pd_1405`, `pi_asym_knee1405`).
- Windows: build/copia DLL plugin PI completa e documentata.
- Secondo pass knee dello sweep locale.
- Confronto consolidato finale tra configurazioni storiche: PD, PI, cascade,
  retune PI, zeta07, pi-tuned, J_eff, Opzione D, slow inner, asym.
- Pulizia artefatti sweep `_cascade_local_gain_sweep_20260517_233607` e
  `_cascade_local_gain_sweep_20260517_234151`.
- LPF qdot: test asimmetrico solo ankle, cutoff 30/35 Hz e run lunga 30+ s se si
  riprende quella linea.
