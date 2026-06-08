# Validazione intensa del trajectory generator RL (creato con Codex)

## Problema

Il trajectory generator RL (PPO/SNN dentro `Trajectory Generator/`) è stato
prodotto con Codex e consegnato come "eseguibile end-to-end" ma non validato.
Serviva una validazione **approfondita e indipendente**: leggere tutto lo stack,
eseguire training/inference reali sul setup AB06 PI, verificare le invarianti
dichiarate (contratto azione, determinismo, checkpoint), stressare i punti
sospetti del code review (cache, BPTT, terminazioni) e caratterizzare la
stabilità su rollout lunghi a scala di passo.

## Soluzione

Validazione **read-only sul codice del generatore** (nessuna modifica al
generatore né al plugin C++): lettura completa dello stack, esecuzione di
training e inference reali, e probe mirati self-contained per isolare i
comportamenti dubbi. Esito: la **macchina è solida** (gira, è deterministica,
stabile in integrazione); i problemi trovati sono di **consistenza
policy↔simulatore** e robustezza, non crash.

## Strategia

1. Lettura statica dell'intero stack (env, training CLI, inference CLI,
   actor-critic, `PPO_SNN`, model SNN, generator, checkpoint, encoding,
   features, provider) + verifica che gli attributi `SimulationContext`/`config`
   usati dall'env esistano davvero.
2. Esecuzione reale in `envCMC-like`: smoke SNN, training tiny e training che
   **esercita il path di update multi-minibatch** (`mini_batches=2`,
   `learning_epochs=4`, multi-episodio), inference breve e **rollout lunghi**
   (2.0 s / 3.5 s).
3. Probe empirici decisivi e self-contained (senza OpenSim) per i sospetti:
   cache su `data_ptr`, valutazione ricorrente nell'update, clip dell'azione in
   inference, semantica `terminated`/`truncated`.
4. Re-test fedele quando un probe iniziale era costruito male (vedi
   Ritrattazione): non riportare un bug senza riprodurlo nel path reale.

## File modificati

Nessuna modifica al codice del generatore o al plugin. Creati solo script di
validazione (riutilizzabili) e artefatti di run:

```text
validation/_val_codex_diag.py          (determinismo, finestra IK, semantica terminazioni)
validation/_val_actor_critic_probe.py  (P1 cache, P2 naive [poi corretto], P3 clip)
validation/_val_update_path_probe.py   (P2 re-test fedele del path di update)
validation/_val_long_rollout.py        (rollout lungo con diagnostica per-step)
runs/_val_train_tiny, _val_train_tiny2, _val_train_update   (checkpoint+summary)
runs/_val_rollout_short                                     (inference summary)
```

(Gli artefatti in `runs/_val_*` sono cancellabili.)

## Verifiche eseguite

- Dipendenze `envCMC-like`: torch 2.7.0+cpu, snntorch 0.9.4, skrl 2.1.0,
  gymnasium 1.3.0, opensim 4.5.2 — import OK. `tests/smoke_test.py` → passed.
- Attributi `ctx`/`cfg` usati dall'env (`q_sv_idx`, `qdot_sv_idx`,
  `sea_motor_angle/speed_sv_idx`, `sea_knee/ankle_name`, `pros_coords`,
  `kinematics_lowpass_cutoff_hz`, `integration_scheme`, `sea_forward_mode`):
  tutti presenti e coerenti.
- Training tiny ed update-path: end-to-end OK, reward finiti, 8/8 episodi
  `terminated`, 0 troncati, checkpoint `best`/`last`/alias + `summary.json`.
- **Determinismo**: due run con lo stesso seed → identici su
  `reward_min/max/mean/last`, `best_score`, `episode_returns`.
- Checkpoint: export → reload (`ReferenceGenerator.from_checkpoint`) → `predict`
  verificato in-process ogni run; contract stretto (`output_contract=env_action`,
  `feature_names`, `action_shape=[3,2]`, obs 58-dim) in train e rollout.
- Inference breve (5 step): reward ~0.90, azione in range, termina pulito.
- **Rollout lunghi** (checkpoint `_val_train_update`):
  - 200 step / 2.0 s: nessuna divergenza, `pelvis_ty`min 0.96, finito,
    `u_abs_max`=1.0, **saturazione SEA 42/200 (21%)**, `action_abs_max`=**1.23**.
  - 351 step / 3.51 s: nessuna divergenza, `pelvis_ty`min 0.95, finito,
    `u_abs_max`=1.0, **saturazione 59/351 (17%)**, `action_abs_max`=**1.34**.
- Semantica terminazioni: episodio ad azione-zero fino all'orizzonte →
  `terminated=True, truncated=False` (il time-limit è etichettato terminale).
- Finestra IK AB06 PI: `t∈[11.99, 21.0]` → 9.01 s utili (rollout gait-scale
  possibili).

## Findings (severità, evidenza, fix)

- **F1 [Media] Inference ritorna la media NON clippata → supera i bound azione.**
  Training campiona clippato a `[-1,1]` (`clip_actions=True`), ma
  `predict_action` ritorna la media grezza. Probe: training `|max|=1.0` vs
  inference `|max|=3.28`. Reale: `action_abs_max` 1.23 (200) e 1.34 (351) > 1.0.
  *Fix:* clippare la media ai bound in `predict_flat`/`predict_action` (o
  `clip_mean_actions` esportato, o `output_transform="tanh"` con scala).
- **F2 [Media] Il SEA satura ~17–21% con una policy reale.** `u_abs_max=1.0`,
  ankle reference a transienti ~3 rad. Il "0% saturazione" del 2026-06-01 era una
  sinusoide 0.10; una policy reale ri-introduce saturazione **nonostante il
  filtro 6 Hz**. `reward_saturation_weight=0.1` troppo piccolo per mordere.
  *Fix:* limite di magnitudo azione + penalità saturazione efficace; rivalutare
  dopo un training vero.
- **F3 [Bassa/Media] Import torch/OpenSim/numpy fragile su Windows.** `import
  torch` nudo fallisce (OMP #15 / `fbgemm.dll` WinError 127) perché OpenSim è in
  PATH e numpy/MKL carica un `libiomp5md` in conflitto. Funziona solo perché
  `prosthesis_snn/__init__` toglie OpenSim dal PATH prima di torch, ma è
  sensibile all'ordine (numpy-prima-del-package rompe). Le CLI `-m` sono OK.
  *Fix:* shim importabile obbligatorio per primo + documentazione.
- **F4 [Bassa] Membrana generatore non resettata a fine episodio nel rollout.**
  `cmc_policy_rollout --reset-on-done` resetta l'env ma mai `generator.reset()`
  → membrana SNN sfora tra episodi (incoerente col training). Default
  mono-episodio OK. *Fix:* `generator.reset()` al reset env nel loop.
- **F5 [Bassa] `terminated`/`truncated` invertiti vs Gymnasium; GAE fa bootstrap
  solo su `terminated`.** L'env mette `terminated` all'orizzonte (time-limit) e
  `truncated` alla caduta. `_compute_gae` usa `non_terminal = not terminated`,
  azzerando il bootstrap all'orizzonte. Attenuante: `phase` è osservata; cadute
  attualmente 0. *Fix:* convenzione Gym + bootstrap su troncamento.
- **F6 [Bassa/latente] Cache forward keyed su `data_ptr()` (content-independent).**
  Onestamente **non riprodotto un risultato errato** (riuso puntatore non
  riprodotto in 20k tentativi; test in-place inconcludente per la rete piccola
  che satura). Mitigato dal `_clear_cache()` dopo ogni step di update. *Fix:*
  key su identità tensore + `_version`, o rimuovere la micro-ottimizzazione.
- **F7 [Bassa] Nessuna validazione `input_size == len(feature_names)` né
  architettura su resume/load.** Un `--load-reference-path`/`--resume-agent-path`
  con `--hidden-size` diverso dal checkpoint crasha a `load_state_dict`. *Fix:*
  validare/derivare l'architettura dal checkpoint.
- **Nota di design (non bug):** reward dominata da `reference_score` (peso 0.55
  = "resta vicino all'IK"); una policy non allenata segna già ~0.90 medio. I
  numeri "belli" non dimostrano apprendimento ma riflettono il baseline IK.

## Ritrattazione (trasparenza)

Sospetto iniziale: l'update PPO valuterebbe male la policy ricorrente
(minibatch trattato come sequenza temporale). Un **re-test fedele**
(`_val_update_path_probe.py`, membrane per-step memorizzate come nell'update
vero) ha mostrato valutazione per-campione **CORRETTA** (`max|diff|=6e-8` vs
collection-time). Il ramo "reshape-come-sequenza" esiste ma **non è esercitato**
nel wiring single-env (`num_envs` dedotto dalla batch-dim delle membrane =
dimensione minibatch). **Non è un bug.** Resta solo `_reshape_for_update` come
codice non testato per `num_envs>1` (coerente col TODO T3 del 2026-06-01).

## Risultato

Il generatore è **eseguibile, deterministico e stabile in integrazione** (351
step / 3.5 s senza divergenza, whole-body in piedi, tutto finito), con contratto
e checkpoint validati. I limiti reali sono: azione di inference oltre i bound
(F1), saturazione SEA con policy reale (F2), fragilità import Windows (F3), più
minori (F4–F7). Nessun bug bloccante; il sistema è pronto per iterare, ma i
numeri di reward attuali non vanno letti come "apprendimento" finché reward e
clip non sono sistemati.

## TODO aperti

- **F1**: clippare la media in inference ai bound dell'azione (consistenza
  train/deploy).
- **F2**: limitare la magnitudo azione e rendere efficace la penalità di
  saturazione; rivalidare il filtro 6 Hz con una **policy allenata** su rollout
  lungo.
- **F3**: rendere robusto/ordinato l'import torch/OpenSim/numpy su Windows
  (shim obbligatorio + doc).
- **F4**: `generator.reset()` al reset env nel rollout (`--reset-on-done`).
- **F5**: allineare `terminated`/`truncated` alla convenzione Gymnasium e fare
  bootstrap GAE sul troncamento.
- **F6**: irrobustire o rimuovere la cache su `data_ptr` in `actor_critic`
  (coerente col TODO T2 del 2026-06-01).
- **F7**: validare `input_size==len(feature_names)` e l'architettura su
  resume/load del checkpoint.
- **Reward**: ribilanciare i pesi così che "riprodurre l'IK" non sia già
  quasi-ottimo (gradiente utile per deviazioni), e alzare il peso saturazione.
- `_reshape_for_update` / BPTT: validare prima di passare a `num_envs>1`.
