# Reward "imitation": la protesi imita la gamba sana (anti-fase)

**Data**: 2026-06-10

## Problema

L'utente vuole lanciare un full training run per due scopi: (1) validare
end-to-end la pipeline nuova (obs realistica, action assoluta, GRF ibrida,
restart-da-checkpoint, gait clock); (2) fare un **pre-training imitativo** della
rete, così che il successivo training su reward ex-novo parta più facile.

La reward attuale è già imitativa, ma verso la **traiettoria IK protesica
registrata** (`reference_loss = (q_base[pros]-q)²`, peso blend 0.55). L'utente
vuole invece una reward il cui obiettivo è **imitare la gamba sana**:
`pros_knee_angle` insegue `knee_angle_r`, `pros_ankle_angle` insegue
`ankle_angle_r`. Richiesta: aggiungere a `reward_function.py` un flag che distingue
`imitation` da `ex_novo`, senza toccare la reward ex-novo.

## Soluzione

Nuova modalità reward **`imitation`** selezionabile via flag, con la reward
ex-novo lasciata **numericamente identica**.

- **Anti-fase T/2**: le due gambe camminano sfasate di mezzo ciclo, quindi il
  target è il giunto sano valutato a `t − T/2` (mirroring corretto del cammino),
  NON allo stesso istante (che darebbe gambe in fase = non-gait). Il periodo `T`
  viene dal gait clock (`mean_period`, ~1.13 s sui dati AB06).
- **Target = gamba sana, posizione + velocità**: l'env calcola
  `sound_imitation_loss = Σ (q_sano − q_pros)² + 0.02·(qd_sano − qd_pros)²` sui
  giunti protesici, con il sano campionato anti-fase.
- **Convenzioni**: dai dati AB06 i due lati hanno **la stessa convenzione di
  segno** (ginocchio flessione-negativo, caviglia dorsiflessione-positivo), quindi
  nessuna mappa affine/flip; l'interpolatore converte già gradi→radianti.
- **Architettura**: l'env espone sempre `sound_imitation_loss` in
  `info["reward_terms"]` (costo trascurabile, utile anche come diagnostica in
  ex-novo); lo shaping (blend/score) vive in `reward_function.py`, gated dal flag.
  In `ex_novo` il nuovo termine è ignorato → reward ex-novo invariata.

Reward in modalità imitation:
```
imitation_score = 1/(1 + imitation_weight · sound_imitation_loss)
base = clip(blend_imitation·imitation_score + blend_imitation_tracking·tracking_score − penalty)
reward = base − safety_term − oob_term
```
(niente reference/bio: reference è il vecchio target IK protesico, bio è
incontrollabile dalla policy). Default: `blend_imitation=0.8`,
`blend_imitation_tracking=0.2`, `imitation_weight=8.0`.

**Warm-start**: obs e action space sono identici tra le due modalità (cambia solo
la reward) → il checkpoint del pre-training imitativo è un seed diretto per
l'ex-novo via `--resume-from ... --reward-mode ex_novo`.

## Strategia

1. L'env calcola il target anti-fase interrogando `base_kin` a `t − T/2`; il tempo
   shiftato è clampato nel dominio dell'interpolatore (nuova proprietà
   `time_bounds`) per evitare extrapolazione ai bordi del dataset.
2. La mappa pros→sano è configurabile (`imitation_sound_coords`, default
   `pros_knee_angle→knee_angle_r`, `pros_ankle_angle→ankle_angle_r`); shift
   (`imitation_phase_shift=0.5`) e peso velocità (`imitation_vel_weight=0.02`)
   sono campi config.
3. Il flag `reward_mode` è un campo di `RewardConfig` (stringa), impostabile via
   `--reward-json` o via il nuovo flag CLI `--reward-mode {ex_novo,imitation}`
   (che ha precedenza). `from_mapping`/`to_dict` resi str-safe.

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
  - _get_reward: calcolo sound_imitation_loss anti-fase + chiave in reward_terms
  - CMCEnvConfig: imitation_phase_shift, imitation_vel_weight, imitation_sound_coords
kinematics_interpolator.py
  - proprietà time_bounds (dominio spline, per il clamp del tempo shiftato)
Trajectory Generator/baseline_MLP/reward_function.py
  - SOUND_IMITATION_LOSS; campi reward_mode/imitation_weight/blend_imitation/
    blend_imitation_tracking; from_mapping/to_dict str-safe; branch in compute_reward
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
  - flag --reward-mode + iniezione in reward_overrides
Trajectory Generator/baseline_MLP/rollout_eval.py
  - flag --reward-mode + iniezione in reward_overrides
Trajectory Generator/baseline_MLP/commands.txt
  - blocco B'') pre-training imitativo: TRAIN-IMIT-WIN, ROLLOUT-IMIT, WARM-START-EXNOVO
```

Nessuna modifica al plugin C++ SEA, alla dinamica o alla semantica del comando
SEA. `env_factory.py` non richiede edit (i campi passano via `_CMC_ENV_FIELDS`).
La reward ex-novo resta byte-identica.

## Verifiche eseguite

### Statiche
- `py_compile` di tutti i file modificati: PASS.

### Smoke reward (Part A, no OpenSim)
- Reward `ex_novo` byte-identica alla formula storica; **ignora**
  `sound_imitation_loss`.
- Branch `imitation`: usa `sound_imitation_loss` + tracking, **ignora**
  `reference_loss`; `imitation_score` loggato.
- `reward_mode` sopravvive al round-trip JSON (`from_mapping`/`to_dict` str-safe);
  bound oob ancora coerciti a float.

### Smoke env (Part B, env reale)
- `base_kin.time_bounds` = (11.99, 155.04) s; clamp del tempo shiftato corretto.
- `sound_imitation_loss` emesso dall'env == loss ricalcolato sul target anti-fase
  `base_kin.get(t − T/2)` (tol 1e-6); periodo gait 1.132 s.
- Verificato che è **davvero anti-fase**: il loss vs sano allo stesso istante
  differisce da quello a `t − T/2`.
- Segno/scala: target ginocchio sano in radianti, flessione-negativo.

### End-to-end (envCMC-rllib)
- Tiny train imitation (2 worker, 2 iter): `ok:true`; `reward_mode=imitation`
  propagato in `summary.json`.
- Rollout imitation: `ok:true`, return 15.47.
- **Warm-start**: `--resume-from` del checkpoint imitativo con `--reward-mode
  ex_novo` → ripristino RLlib iter 2, continua come iter 3, `ok:true`. Conferma il
  riuso del seed imitativo per l'ex-novo.

Dir temporanee di verifica e smoke script rimossi; run reali intatti.

## Come lanciare il full run

Comandi pronti in `commands.txt`:
- `[TRAIN-IMIT-WIN]` — pre-training imitativo 40 iter (12 worker), reward-json con
  banda oob allargata per coprire il ROM della gamba sana.
- `[ROLLOUT-IMIT]` — inference dal run imitativo (stessa modalità).
- `[WARM-START-EXNOVO]` — riprende il checkpoint imitativo come seed ex-novo.

## TODO

- [ ] Lanciare `[TRAIN-IMIT-WIN]` (full run): validare pipeline e ottenere il seed
      imitativo; analizzare `imitation_score`, terminazioni, stabilità.
- [ ] Tarare `imitation_phase_shift`/`blend_*` se l'anti-fase non è perfettamente
      allineato (es. offset toe-off vs heel-strike del gait clock).
- [ ] **Reward ex-novo task-based** vera (periodicità, coordinazione, fattibilità
      GRF, stabilità) — prerequisito per il run "vero" dopo il warm-start; a quel
      punto rivisitare anche **l'uscita del riferimento IK dal critico** (TODO
      aperto dal lavoro observation-space, non chiuso da questa modalità).
- [ ] **Gait clock**: migrare la sorgente a entrainment ipsilaterale/IMU quando la
      GRF online sarà validata (TODO aperto).
- [ ] macOS arm64: riverificare il flag `--reward-mode` e il warm-start.
