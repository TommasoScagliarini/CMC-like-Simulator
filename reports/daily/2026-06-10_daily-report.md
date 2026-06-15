# Daily report - 2026-06-10

## Sintesi

Giornata su quattro blocchi:

1. **Robustezza del training** (mattina): il "skip in-place" delle iterazioni in
   stallo non funzionava; sostituito con il **restart del child da checkpoint**
   (Opzione B), poi training e inference **validati** end-to-end con run reali
   supervisionati (2 bug di path corretti), e **`commands.txt` ristrutturato**.
2. **Generatore ex-novo — observation space realistico + critico privilegiato**:
   split dell'osservazione in attore (realistico) / critico (privilegiato) con
   **asymmetric actor-critic**, in due fasi.
3. **Reward "imitation"**: nuova modalità reward in cui la protesi **imita la gamba
   sana in anti-fase**, come pre-training (con warm-start verso l'ex-novo).
4. **Esecuzione: full training imitativo notturno** — avviato, crash nativo di Ray
   a iter 13/40, **ripreso da checkpoint sotto wrapper auto-restart** (in corso).

Report utente prodotti oggi:

```text
reports/user/2026-06-10_skip_iterazione_restart_da_checkpoint.md
reports/user/2026-06-10_restart_checkpoint_supervisor_implementazione_validazione.md
reports/user/2026-06-10_validazione_training_restart_checkpoint_run_supervisionati.md
reports/user/2026-06-10_validazione_inference_rollout_fix_path_checkpoint.md
reports/user/2026-06-10_ristrutturazione_commands_txt_comandi_standard.md
reports/user/2026-06-10_observation_space_realistico_critico_privilegiato.md
reports/user/2026-06-10_reward_imitation_gamba_sana.md
```

## 1. Robustezza del training: restart da checkpoint

- **Skip in-place rotto**: la fault-tolerance di RLlib (`restart_failed_env_runners`)
  ricreava i worker uccisi, `algo.train()` non si sbloccava mai → lo "skip"
  diventava "abort". Scelta l'**Opzione B**: `algo.train()` torna chiamata
  bloccante lineare; su `iteration_timeout` il child scrive `summary.json`
  (`stop_reason="iteration_timeout"`, prossima iter + ultimo checkpoint) ed esce
  124; il **supervisor** termina il process tree e rilancia un interprete fresco
  che fa `restore_from_path(checkpoint_last)`. `--resume-from` manuale, metadata
  `checkpoint_*_meta.json`, abort dopo `--max-consecutive-skips`, `--checkpoint-every 1`.
- **Training VALIDATO**: 5 run reali supervisionati (config ibrida di produzione);
  nessun bug introdotto.
- **Inference VALIDATA** con **2 bug reali corretti**: `--checkpoint` e
  `--resume-from` con path relativo non seguivano la convenzione di `--output-dir`
  (risolti rispetto alla CWD invece che a `Trajectory Generator` dopo lo
  spostamento di `runs/`), rompendo i comandi rollout documentati.
- **`commands.txt` ristrutturato** in 8 sezioni ordinate per uso, con "REGOLE
  D'ORO" e tag grep-abili.

## 2. Observation space realistico + critico privilegiato

Problema: l'osservazione esponeva l'intero stato (pelvis 6-DOF assoluto,
controlaterale, riferimenti IK) → **~54% privilegiato**, non sensorizzabile sul
reale. Soluzione: partizione **attore** (prefisso realistico: encoder giunti
protesici `q`/`qdot`, stati motore SEA, carico piede protesico `online_left_*`,
clock + memoria comando) / **critico** (suffisso privilegiato: pelvis/controlaterale,
riferimento IK, `online_right_*`), con **asymmetric actor-critic** (policy realistica
deployabile, critico privilegiato → advantage a varianza più bassa).

- **Fase 1** (symmetric, default): l'env emette il solo prefisso attore
  `Box(n_actor)`, gira su `DefaultModelConfig`; `_unsafe_end_reason` reso
  obs-independent; dict completo sempre in `info["observation"]`. n_actor 22
  (prescribed) / 28 (online-GRF), full 61 / 73.
- **Fase 2** (asimmetrico): nuovo `asymmetric_rl_module.py`
  (`AsymmetricActorCriticTorchRLModule`, Ray 2.55.1) — policy legge `obs[:n_actor]`,
  value il vettore pieno; `RLModuleSpec` con `n_actor` in `model_config`;
  round-trip checkpoint/resume verificato.
- Riferimento IK tolto dall'attore (resta nel critico, **decisione deferita**);
  gait clock interim nell'attore (marcato).

## 3. Reward "imitation": la protesi imita la gamba sana (anti-fase)

Nuova modalità reward selezionabile (`reward_mode` in `RewardConfig`, flag
`--reward-mode {ex_novo,imitation}`), con la reward **ex-novo byte-identica**.

- `pros_knee_angle`→`knee_angle_r`, `pros_ankle_angle`→`ankle_angle_r`, target
  campionato **anti-fase** a `t − T/2` (mirroring corretto del cammino); posizione
  + velocità. Stessa convenzione di segno tra i lati (nessun flip).
- L'env emette sempre `sound_imitation_loss` (ignorato in ex-novo); proprietà
  `time_bounds` nell'interpolatore per il clamp del tempo shiftato.
- **Warm-start**: obs/action space identici tra le modalità → il checkpoint
  imitativo è seed diretto per l'ex-novo via `--resume-from --reward-mode ex_novo`.
- Verifiche: ex-novo invariato, anti-fase confermato (periodo 1.13 s), tiny
  train+rollout imitation `ok:true`, warm-start `ok:true`.

## 4. Esecuzione: full training imitativo notturno

- **Avvio** 2026-06-10 22:41 (`runs/baseline_mlp_imit_win`, 40 iter, 12 worker,
  batch 4096, episodi 2 s, `--reward-mode imitation`, oob allargata via
  `reward_imitation.json`).
- **Apprendimento sano**: iter 1 return 7.55 → **iter 13 return 15.51** (+105%),
  episodi che completano, terminazioni miste `episode_time_limit`/`grf_penetration`,
  nessun NaN. ~13.7 min/iter.
- **Crash** a iter 14 (02:07): `Windows fatal exception: access violation` in Ray
  (`metrics_logger.py`) durante il backward del learner — **crash nativo sporadico
  di Ray, non del nostro codice e non OOM** (32.9 GB liberi). Il supervisor interno
  NON rilancia sui crash nativi (solo su `iteration_timeout`) → run interrotto.
  **13 checkpoint validi** salvati (seed imitativo a return 15.51).
- **Ripresa** (2026-06-11 09:00): per evitare la dipendenza dalla latenza di
  notifica, creato un **wrapper auto-restart** (`run_imit_autorestart.ps1`) che
  riprende da `checkpoint_last` a ogni crash (cleanup orfani Ray, riduzione
  adattiva worker, fino a 15 attempt) finché non raggiunge le 40 iter. **In corso**
  dall'iter 13.
- **Deadline 09:00 (per le 40 iter) mancata**: il crash delle 02:07 + un gap di
  notifica (~6.7 h) hanno impedito il completamento entro l'orario. È comunque
  disponibile un **seed imitativo valido (iter 13)**; la run completa sta proseguendo.

## File principali modificati/aggiunti oggi

```text
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/commands.txt
Trajectory Generator/baseline_MLP/README.md
Trajectory Generator/osim_trj_cmc_like.py
kinematics_interpolator.py
Trajectory Generator/baseline_MLP/asymmetric_rl_module.py   (NUOVO)
Trajectory Generator/baseline_MLP/reward_imitation.json     (NUOVO)
run_imit_training.ps1 / run_imit_autorestart.ps1            (NUOVI, launcher)
```

Nessuna modifica al plugin C++ SEA o alla semantica del comando SEA.

## Verifiche eseguite (sintesi)

- Restart-checkpoint: 5 run training supervisionati + inference (2 bug path corretti).
- Observation split: smoke prefix-invariance/shape/reward-invariata/schema-stability;
  unit RLModule asimmetrico (forward/slice/inference-only); tiny train+rollout +
  resume sia symmetric sia asimmetrico.
- Reward imitation: ex-novo byte-identico, anti-fase, tiny train+rollout+warm-start.
- `py_compile` di tutti i file modificati; dir/script temporanei rimossi.

## TODO aperti e propagati

### Esecuzione training (immediati)
- **Verificare il completamento** del run imitativo sotto wrapper auto-restart
  (`runs/baseline_mlp_imit_win`, log `imit_autorestart.log`); analizzare curva
  `imitation_score`/return, terminazioni, stabilità; salvare il seed finale.
- **Supervisor: gestire i crash nativi del child** (oggi rilancia solo su
  `iteration_timeout`, non su access-violation/abort) — internalizzare la logica
  del wrapper auto-restart.
- Indagare l'**access violation di Ray** (`metrics_logger.py`, learner update):
  sporadico; valutare se ricorre, e se ridurre worker/`num-learners` lo mitiga.

### Generatore ex-novo (linea principale)
- **Reward ex-novo task-based** vera (auto-periodicità via gait clock,
  coordinazione/anti-fase, fattibilità GRF, stabilità, effort/energia SEA); a quel
  punto **decidere se il riferimento IK esce anche dal critico** (deferito).
- **Migrare la sorgente del gait clock** da heel-strike del lato sano prescribed
  (privilegiato) a entrainment ipsilaterale/IMU, quando la GRF online sarà validata.
- Valutare **memoria/ricorrenza** sull'attore (leva complementare al critico privilegiato).
- Allenare un run **asimmetrico** reale e confrontare explained-variance del critico
  vs symmetric; tarare `imitation_phase_shift`/`blend_*`.
- macOS arm64: riverificare round-trip RLModule, `--reward-mode`, warm-start,
  cleanup process group POSIX + resume RLlib; `setuptools<81`; ricompilare plugin
  onlineGRF `.dylib`.

### Throughput / dinamica / contatto (propagati)
- ~1.18 s/env-step (SO QP) collo di bottiglia; ridurre fallback bounded least-squares.
- Divergenze `joint_divergence_pros_knee_angle`, saturazione knee SEA, reserve
  biologiche/equilibrium; critic explained-variance negativa; `oob_weight`/filtro 6 Hz.
- Timing heel-strike **online** lato protesico e flag `in_contact` (prerequisito
  per varianti evento-discreto, non per il clock); rocker/COP push-off; contatto
  online lato sano per traiettorie ex-novo divergenti.

### Repository / housekeeping
- Pulire script/log temporanei (`results/_*`, `validation/_*`, launcher/log
  `run_imit_*.ps1`, `imit_*.log` una volta concluso il run).
- Valutare rimozione build artifact `build_online_grf/*.obj` da git.

### Validazione generatore / SNN / PPO / letteratura (propagati storici)
- F1-F4/F7 percorso SNN/skrl; cache `data_ptr` (F6/T2); reshape/BPTT (T3); spike
  gradient encoder (T4); reader `.sto` `inDegrees` (S1); `sea_stiffness` per-modello
  (S2/J2); riuso `InverseDynamicsSolver` (S3); SNN come RLModule.
- Recuperare il vero paper Wrapyfi (paper/7 è iCub); approfondire P17/P24/P14/P19/P23;
  collegare la letteratura alla roadmap ex-novo/GRF online.

### Propagati storici: controllo SEA
- Sweep `Kp_knee_motor` 3.9-18; coupling knee-ankle; notch 28 Hz knee; cleanup
  modelli sperimentali; build/copia DLL plugin PI Windows; secondo pass knee;
  confronto finale configurazioni storiche; LPF qdot asimmetrico ankle 30/35 Hz.
