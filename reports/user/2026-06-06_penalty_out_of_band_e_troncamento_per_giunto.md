# Penalty out-of-band sul riferimento e troncamento per-giunto

## Problema

Due esigenze sulla baseline MLP, partendo dalla discussione sui limiti articolari:

1. **Troncamento episodio**: usava un'unica soglia simmetrica
   `max_abs_pros_q_rad = 4.0` (~229°), assurda come limite e identica per ginocchio
   e caviglia. Serviva una guardia **anti-divergenza per-giunto** (asimmetrica).
2. **Reward**: serviva una **penalità per uscita dalla banda fisiologica di gait**,
   da applicare al **riferimento generato dalla policy** (la traiettoria da
   inseguire, a monte di SEA + Static Optimization + contatto), non alla `q`
   simulata. Il razionale (chiarito dall'utente): la rete controlla direttamente il
   riferimento, quindi penalizzarlo dà un gradiente pulito.

## Soluzione

### Troncamento per-giunto (env)

In `osim_trj_cmc_like.py` rimosso lo scalare `max_abs_pros_q_rad` e introdotto
`truncation_bounds_rad` (Mapping `coord_name -> (low, high)`, `field(default_factory=...)`
in dataclass frozen). `_is_truncated` ora controlla per-giunto `q < low or q > high`
sulla **`q` simulata** (la divergenza è un fenomeno della dinamica simulata).

### Penalità out-of-band (reward_function.py)

`compute_reward` ha un nuovo kwarg `reference=`; la penalità è calcolata sul
**riferimento comandato** (`info["policy_segment_values"]`). `out_of_band_loss` =
media dell'escursione quadratica fuori da `[oob_q_min, oob_q_max]` per giunto.
`oob_term = oob_weight * oob_loss` è sottratto **dopo il clip** (come la safety):
resta attivo con gradiente anche quando la parte positiva della reward è già 0.
`RewardConfig` espone `oob_weight` (default 2.0, attivo), `oob_q_min`, `oob_q_max`
(per giunto, ordine `pros_coords = [knee, ankle]`). `from_mapping`/`to_dict` ora
gestiscono i campi tuple. Le componenti `oob_loss`/`oob_term` finiscono
automaticamente in TensorBoard come `reward/oob_loss` e `reward/oob_term`.

### Scoperta in verifica: convenzione di segno del ginocchio

I bound proposti inizialmente (troncamento knee `[0, 2]`, banda knee `[0, 1.35]`)
erano in **convenzione di segno opposta**. Misura empirica sui dati IK (~4 cicli di
gait, low-pass 6 Hz):

- `pros_knee_angle` ∈ **[-1.06, -0.13] rad** — flessione **negativa**, non raggiunge
  mai 0;
- `pros_ankle_angle` ∈ **[-0.13, 0.40] rad**.

Con i bound `[0, …]`, il ginocchio (sempre < 0) faceva **troncare ogni episodio allo
step 1** e penalizzava l'intera traiettoria. Su decisione dell'utente i segni sono
stati invertiti e la banda caviglia portata a `[-0.5, 0.5]`.

**Bound finali:**

| | Banda reward (oob, soft) | Troncamento (anti-divergenza) |
|---|---|---|
| knee  | `[-1.35, 0.0]` | `[-2.0, 0.0]` |
| ankle | `[-0.5, 0.5]`  | `[-0.9, 0.9]` |

Entrambi contengono il range IK misurato, quindi il tracking IK puro non viene né
penalizzato né troncato.

## Strategia

- **Confine netto**: l'env calcola i loss ed espone `q` simulata e riferimento;
  `reward_function.py` combina. La banda penalizza il **riferimento** (controllato
  dalla policy → gradiente pulito), il troncamento guarda la **`q` simulata**
  (divergenza).
- **Due ruoli distinti**: banda reward **stretta** (shaping morbido, gradiente
  persistente) ≠ bound di troncamento **largo** (termina solo una simulazione
  esplosa).
- **Verificare sui dati, non sulle assunzioni**: la convenzione di segno del
  ginocchio è stata scoperta misurando l'IK reale, non assunta dai numeri proposti.
- Default attivo (`oob_weight = 2.0`): la reward NON è più bit-identica a quella
  dell'env (lo è solo con `oob_weight = 0`); scelta esplicita dell'utente.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py` — rimosso `max_abs_pros_q_rad`,
  aggiunto `truncation_bounds_rad`; `_is_truncated` per-giunto.
- `Trajectory Generator/baseline_MLP/reward_function.py` — `out_of_band_loss`,
  kwarg `reference` in `compute_reward`, campi `oob_*` in `RewardConfig`,
  `from_mapping`/`to_dict` per i campi tuple, `RewardShapingWrapper` passa
  `info["policy_segment_values"]`.
- `Trajectory Generator/baseline_MLP/reward_overrides_example.json` — campi oob
  aggiornati (segni corretti).
- `Trajectory Generator/baseline_MLP/README.md` — sezione Reward (banda oob +
  convenzione segno), nota troncamento, esempi `--reward-json`.

## Verifiche eseguite

Da `envCMC-rllib`, CWD = repo root, setup AB06 PI:

- `py_compile` su env + reward_function → OK.
- **Unit**: `out_of_band_loss` (in-banda 0, escursione knee/ankle corrette, riga 1D);
  `compute_reward` (in-banda = nessuna penalità; fuori = penalizzato;
  `reward = base - safety - oob`; componenti con `oob_loss`/`oob_term`);
  `from_mapping`/`to_dict` con campi tuple + round-trip JSON.
- **Troncamento per-giunto** (`_is_truncated` diretto): knee 2.5 → tronca, knee 1.9
  → no (con i bound provvisori), ankle 1.5 → tronca; dopo correzione, bound
  knee `[-2,0]`/ankle `[-0.9,0.9]`.
- **Range IK reale** (probe): knee `[-1.06, -0.13]`, ankle `[-0.13, 0.40]`.
- **Verifica finale (bound corretti)**: episodio IK puro (azione nulla, 1.2 s) →
  **120 step, `terminated`, NON `truncated`** (regressione step-1 risolta);
  **`oob_term` max/mean = 0.0** (IK completamente in banda).
- **Tiny train end-to-end** con nuovo troncamento + nuova reward → EXIT 0;
  `reward/oob_loss` e `reward/oob_term` presenti in TensorBoard.

## TODO aperti

- **Tuning `oob_weight`**: tararlo guardando `reward/oob_term` in TensorBoard su un
  training reale (la penalità ora morde solo oltre la banda di gait).
- **Training reale** (50+ iterazioni) per validare le curve di apprendimento.
- (Carryover) **F2 reward rebalancing** (peso `reference_score` dominante),
  porting **gait metrics** (RMSE/Symmetry/Trend), **SNN come custom RLModule**.
