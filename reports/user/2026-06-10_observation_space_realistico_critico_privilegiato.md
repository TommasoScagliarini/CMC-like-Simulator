# Observation space realistico + critico privilegiato (asymmetric actor-critic)

**Data**: 2026-06-10

## Problema

L'observation dell'env esponeva **l'intero stato del modello**: oltre allo stato
protesico e agli stati interni dei SEA, anche il pelvis 6-DOF assoluto, i giunti
controlaterali e i riferimenti IK (`target`/`target_vel`/`tracking_error`/
`kin_ref`). Misurato: **~54% delle feature era privilegiato**, cioè non
sensorizzabile su una protesi reale (con online-GRF obs: 45/73 feature; senza:
39/61). Una policy allenata su quei segnali non trasferisce al caso reale.

Set sensoriale realistico assunto (deciso con l'utente): encoder dei giunti
protesici (`q`, `qdot`), stato interno del motore (SEA `motor_angle`/`motor_speed`),
carico del piede protesico (lato sinistro). `qddot` **non** integrato. L'IMU→angolo
è un problema della sola implementazione reale e non si modella ora.

## Soluzione

L'observation è ora **partizionata** in un prefisso REALISTICO "attore"
(`obs[:n_actor]`) e un suffisso PRIVILEGIATO "critico" (`obs[n_actor:n_obs]`),
con architettura **asymmetric actor-critic**:

- **Attore (realistico)**: clock di episodio, gait clock (interim, vedi sotto),
  `q`/`qdot` dei giunti protesici, stati motore SEA, carico piede protesico
  (`online_left_*`), memoria comando SEA (`previous_endpoint`, `sea_u`, ...).
- **Critico (privilegiato)**: riferimento IK protesico (`*_target`,
  `*_target_vel`, `*_tracking_error`), contesto biologico completo (pelvis 6-DOF
  + controlaterale, `q`/`vel`/`kin_ref`), carico piede sano (`online_right_*`).

La realismo della policy è garantita per costruzione: l'informazione privilegiata
entra solo nel value head, mai nell'input dell'attore. Al deployment il critico si
scarta e la policy gira sul solo prefisso realistico.

Decisioni recepite:
- **Riferimento IK rimosso dall'attore** (privilegiato *e* anti-ex-novo,
  incoerente con l'output assoluto del 2026-06-09). Se debba uscire anche dal
  critico è **deferito** alla ridiscussione di reward/policy (TODO sotto): per ora
  resta nel bucket privilegiato.
- **Gait clock** del lato sano prescribed tenuto nell'attore come **unico input
  privilegiato accettato a interim**, marcato nel codice, da migrare a entrainment
  ipsilaterale/IMU quando la GRF online sarà validata. Dipendenza sottile (1
  scalare → sin/cos), facile da sostituire.
- La **reward è disaccoppiata** dall'observation (`_get_reward` ricalcola da
  stato/cinematica, non legge `obs_dict`): non toccata.

## Strategia

Implementazione **a fasi**:

### Fase 1 — Observation realistico symmetric (default, nessun RLModule)

`CMCEnvConfig.critic_privileged_observation` (default `False`):
- `False` → l'env emette SOLO il prefisso attore = `Box(n_actor)`, allenabile
  sulla pipeline `DefaultModelConfig` esistente (policy realistica "symmetric").
- `True` → l'env emette il superset completo `Box(n_full)` = `[attore |
  privilegiato]`, per il critico asimmetrico.

In entrambe le modalità il **dict completo** resta in `info["observation"]` (solo
l'array cambia), quindi diagnostica e terminazione vedono lo stato pieno.
`_unsafe_end_reason` reso **obs-independent**: legge `pelvis_ty` e le coord
protesiche dallo stato (`sv`), così "fall"/"joint_divergence" scattano identiche
anche quando `pelvis_ty` non è nell'array attore.

### Fase 2 — Critico privilegiato (custom RLModule asimmetrico)

Gated da `--critic-privileged-observation`. Nuovo
`AsymmetricActorCriticTorchRLModule` (Ray 2.55.1, new API stack), sottoclasse di
`DefaultPPOTorchRLModule` che **non usa il catalog encoder** (è dimensionato su un
solo obs space): costruisce MLP propri in `setup()` (`catalog_class=None` →
fallback `Box`→`TorchDiagGaussian`). La policy legge `obs[:n_actor]`, il value
head legge il vettore pieno; `_forward_train` emette `Columns.EMBEDDINGS` per il
riuso in `compute_values`; `get_non_inference_attributes` strippa `vf`/`vf_encoder`
all'export inference-only; `get_initial_state` → `{}`. `n_actor`/`n_full` arrivano
via `model_config` (passato con `RLModuleSpec`), serializzati col checkpoint →
`from_checkpoint` ricostruisce il modulo senza serializzazione custom. Il modulo
vive in un file importabile (non `__main__`) così worker e rollout lo
deserializzano.

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
  - CMCEnvConfig.critic_privileged_observation (nuovo campo)
  - proprietà actor_feature_names/privileged_feature_names/n_actor/n_obs
  - _get_observation: partizione attore/privilegiato + scelta array per modalità
  - _unsafe_end_reason: obs-independent (legge da sv)
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
  - flag --critic-privileged-observation, wiring env_config
  - build_config: rl_module condizionale (RLModuleSpec custom vs DefaultModelConfig)
Trajectory Generator/baseline_MLP/rollout_eval.py
  - flag --critic-privileged-observation, wiring env_config + summary
Trajectory Generator/baseline_MLP/commands.txt
  - regola d'oro #6 (obs split), blocco esempio TRAIN/ROLLOUT asimmetrico
```

```text
Trajectory Generator/baseline_MLP/asymmetric_rl_module.py   (NUOVO)
```

Nessuna modifica al plugin C++ SEA o alla semantica del comando SEA.
`env_factory.py` non richiede edit (il nuovo campo passa via `_CMC_ENV_FIELDS`).

**Rottura checkpoint**: cambia ordine e dimensione obs (attore in testa, IK fuori
dall'attore) + reti diverse in asimmetrico → checkpoint pre-2026-06-10
INCOMPATIBILI, riallenare (come la rottura 58→61 del 2026-06-09).

## Verifiche eseguite

### Statiche
- `py_compile` di env + train + rollout + `asymmetric_rl_module.py`: PASS.
- API Ray 2.55.1 verificate leggendo i sorgenti installati
  (`DefaultPPOTorchRLModule`, `TorchRLModule`, `RLModule`, `DefaultPPORLModule`).

### Fase 1 — smoke observation split (env reale, prescribed + online-GRF-obs)
- Dimensioni: prescribed `n_actor=22`, `n_full=61` (priv 39); online-GRF
  `n_actor=28`, `n_full=73` (priv 45).
- Prefix-invariance: `asym[:n_actor] == sym` al reset e su 6 step; nomi attore ==
  prefisso dei nomi pieni.
- Shape: symmetric `Box(n_actor)`, asymmetric `Box(n_full)`;
  `n_full == n_actor + n_priv`.
- Placement: `phase`/`gait_phase`/`pros_*`/`online_left_*` in attore;
  `pelvis_ty`/`*_target`/`*_tracking_error`/`online_right_*` in privilegiato; IK
  NON in attore.
- `info["observation"]` sempre dict completo (`pelvis_ty` e `online_right_*`
  presenti anche in symmetric).
- Reward identica tra modalità (seed + azioni fisse), schema stabile su più step.
- `_unsafe_end_reason({})` (dict vuoto) → nessun crash, legge da `sv`.
- Tiny train symmetric (2 worker, 2 iter, Box(n_actor=28)) `ok:true` → rollout
  `ok:true`, return 26.96, `pelvis_ty_min 0.95` (letto dal dict full in info).

### Fase 2 — asymmetric RLModule
- Unit smoke (no OpenSim): forward shapes (`(B,2*adim)`, embedding `(B,h)`, value
  `(B,)`); **policy ignora il suffisso privilegiato** e dipende dal prefisso
  attore; value dipende dal vettore pieno; dist `TorchDiagGaussian`;
  inference-only strippa `vf`/`vf_encoder` e mantiene `pi`; `forward_inference`
  su vettore pieno E su vettore largo `n_actor` (deploy reale futuro). ALL PASS.
- Tiny train asimmetrico (`--critic-privileged-observation`, 2 iter) `ok:true`.
- Rollout round-trip: `from_checkpoint` ricostruisce il custom module, `ok:true`,
  return 26.83.
- Resume da `checkpoint_last`: `restored_training_iteration 2`, riparte da iter 3,
  `ok:true` (`restore_from_path` ricostruisce il custom module).

Dir temporanee di verifica e smoke script (`_obs_split_smoke.py`,
`_asym_module_unit.py`) rimossi; run reali intatti.

## TODO

- [ ] **Decidere se il riferimento IK esce anche dal critico** (oggi privilegiato,
      non attore): da valutare nella ridiscussione di reward/policy. Se la reward
      task-based non premia la vicinanza alla IK, toglierlo anche dal critico;
      se si tiene un termine imitativo (curriculum), resta solo nel critico.
- [ ] **Migrare la sorgente del gait clock** dall'heel-strike del lato sano
      prescribed (privilegiato) a entrainment ipsilaterale/IMU, quando la GRF
      online sarà validata. Finché non fatto, è l'unico input privilegiato
      nell'attore (marcato nel codice).
- [ ] Valutare l'aggiunta di **memoria/ricorrenza** all'attore (storia di
      osservazioni o encoder ricorrente) per inferire lo stato nascosto dai soli
      segnali realistici — leva complementare al critico privilegiato.
- [ ] Allenare un run reale **asimmetrico** ([TRAIN-ASYM-WIN]) e confrontare
      apprendimento/explained-variance del critico vs il baseline symmetric.
- [ ] **macOS arm64**: il custom RLModule è puro Python/torch (nessun vincolo OS),
      ma va riverificato il round-trip `from_checkpoint`/`restore_from_path`
      sull'env macOS (questa sessione è Windows).
