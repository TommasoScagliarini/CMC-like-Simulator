# Gait-phase clock del lato sano + output assoluto della rete (ex-novo)

Data: 2026-06-09

## Problema

Due interventi richiesti sulla pipeline RL del Trajectory Generator, conseguenti
all'analisi di design del giorno (vedi
[[2026-06-09_durata_episode_phase_clock_ex_novo]]):

1. **Clock di fase basato sul gait cycle del lato sano.** Mancava un orologio di
   fase agganciato alla falcata (pacemaker). L'unica "fase" in osservazione era
   il *progresso dell'episodio* (`(t - episode_start)/durata`), non la fase del
   cammino; e l'unica gait-phase disponibile veniva dal detector **online**
   (lato protesico), oggi inaffidabile.
2. **Output della rete ancora imitativo.** La rete era settata in
   `action_mode="delta"`: non emetteva una traiettoria, ma una **deviazione
   rispetto alla cinematica prescribed** (`q = q_base + action·max_delta`). Per
   generare traiettorie ex-novo serve un output **assoluto**.

## Strategia

- **Clock dal lato sano, deterministico e detector-free.** Il lato sano (destro)
  segue il GRF/cinematica prescribed: i suoi heel-strike sono deterministici e
  ricavabili offline. Si riusa la **stessa** detection a soglia che il simulatore
  già usa per `save_gait_events` (`output._cycles_from_vertical_grf` +
  `_read_storage_table`) sulla colonna verticale destra
  (`ctx.grf_vertical_force_columns["right"]`), così RL e simulatore concordano su
  cosa sia un heel-strike. La detection copre **l'intero file GRF** (non solo la
  finestra di setup), così `random_init`/episodi lunghi ovunque nel dataset hanno
  una fase valida. Il clock **non dipende** dal detector online rotto.
- **Fase come `(sin φ, cos φ)`** in osservazione (evita la discontinuità 1→0),
  con **offset del punto di reset tunabile** (default = heel-strike sano).
- **Output assoluto via la modalità già esistente** `action_mode="absolute"`
  (mappa l'azione `[-1,1]` su bound assoluti per coordinata), promossa a default,
  con bound che danno margine ex-novo oltre l'IK ma restano dentro le guardie
  anti-divergenza. Le modalità `delta`/`raw` restano disponibili per diagnostica.

Tutto il lavoro resta dentro l'adapter RL / `Trajectory Generator/`. Nessuna
modifica al simulatore CMC-like, alla Static Optimization, ai SEA o al plugin C++.

## Soluzione

### Clock di fase (`Trajectory Generator/osim_trj_cmc_like.py`)
- Nuova classe **`GaitPhaseClock`**: fase a dente di sega `φ ∈ [0,1)` agganciata
  agli heel-strike del lato sano, con extrapolazione robusta fuori range
  (periodo del ciclo più vicino) e `phase_offset` (frazione di ciclo) per
  spostare il punto di reset. Espone `phase(t)`, `phase_sin_cos(t)`, e i metadati
  `available`/`n_cycles`/`mean_period`.
- **`_load_sound_heel_strikes(side)`**: estrae i tempi di HS dal GRF prescribed
  riusando `output._cycles_from_vertical_grf`/`_read_storage_table`. Fallback
  sicuro (clock "non disponibile", fase costante 0) se dati/colonna mancano.
- **`_build_gait_clock`** chiamato in `_build_simulator`; `_gait_clock_summary`
  aggiunto all'info di `reset` per diagnostica.
- Osservazione: aggiunte le feature **`gait_phase`, `gait_phase_sin`,
  `gait_phase_cos`** (sempre presenti, mode-independent).
- Config `CMCEnvConfig`: `gait_clock_enable=True`, `gait_clock_side="right"`,
  `gait_clock_phase_offset=0.0`.

### Output assoluto (`osim_trj_cmc_like.py` + train/rollout)
- `CMCEnvConfig.action_mode` default **`"delta"` → `"absolute"`**.
- `CMCEnvConfig.absolute_bounds_rad` default: `pros_knee_angle (-1.5, 0.0)`,
  `pros_ankle_angle (-0.7, 0.7)` (contiene l'IK con margine, dentro
  `truncation_bounds_rad`).
- `train_ppo_mlp.py` e `rollout_eval.py`: nuovo flag **`--action-mode`**
  (`absolute`/`delta`/`raw`, default `absolute`); l'env_config ora passa
  `args.action_mode` invece di `"delta"` hardcoded.
- Aggiornata la docstring della classe env (la modalità di default ora è
  assoluta).

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
  - GaitPhaseClock (nuova classe)
  - _build_gait_clock / _load_sound_heel_strikes / _gait_clock_summary
  - CMCEnvConfig: action_mode default "absolute"; absolute_bounds_rad default;
    gait_clock_enable / gait_clock_side / gait_clock_phase_offset
  - _get_observation: feature gait_phase / gait_phase_sin / gait_phase_cos
  - reset(): info["gait_clock"]
  - docstring classe env (default "absolute")
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
  - --action-mode (default "absolute"); env_config usa args.action_mode
Trajectory Generator/baseline_MLP/rollout_eval.py
  - --action-mode (default "absolute"); env_config usa args.action_mode
```

## File aggiunti

```text
validation/_gait_clock_absaction_smoke.py   (smoke di validazione, temp _-prefix)
```

## Test e verifiche eseguite

`py_compile` PASS sui tre file modificati. Smoke completo
(`validation/_gait_clock_absaction_smoke.py`, eseguito in `envCMC-rllib`):
**tutti i check PASS, exit 0**.

- **A — clock sintetico**: `φ=0` all'HS, `0.5` a metà ciclo, wrap, extrapolazione
  pre/post-range, `φ` sempre in `[0,1)`, offset (0.25 → `φ=0` al 25% del ciclo),
  sin/cos, clock degenere (0/1 strike → non disponibile, fase 0).
- **B — env reale (prescribed, absolute)**: feature di fase presenti in
  osservazione; clock costruito dal GRF destro reale → **124 cicli del lato
  sano, periodo medio 1.132 s**; `φ=0` a un HS reale e `0.5` a mezzo periodo;
  mapping assoluto `0→[-0.75,0.0]`, `+1→[0.0,0.7]`, `−1→[-1.5,-0.7]`,
  **indipendente dall'IK**; step benigno (hold-pose) end-to-end pulito
  (`end_reason=None`, reward 0.974); `gait_phase` avanza 0.2737→0.3046.
- **C — regressione delta**: `+1 → q_base(t_k)+0.35 rad` intatto; feature di
  fase presenti anche in modalità delta.
- **D — stack reward (torch-free)**: `RewardShapingWrapper` + replica di
  `FlattenClipAction` in modalità assoluta → `reward_terms`/`reward_components`
  presenti, reward finito, clock disponibile, `action_mode` assoluto.

Nota: una verifica via `env_factory.make_cmc_env` (stack ibrido completo) è stata
sostituita dalla Part D torch-free perché l'import diretto di `env_factory`
falliva il caricamento di una DLL di torch (`fbgemm.dll`) fuori dal contesto di
lancio del training (è la ragione d'essere dello shim `win_runtime`); non è un
difetto delle modifiche.

## Implicazioni

- **Training fresco obbligatorio**: cambia la semantica dell'azione (assoluta) e
  la dimensione dell'osservazione (+3 feature di fase) → i checkpoint "delta"
  precedenti sono incompatibili.
- Il comando di training in `commands.txt` (che non passava `--action-mode`)
  userà ora **automaticamente** l'output assoluto.
- Il clock è agganciato al lato sano prescribed → **non** dipende dai bug HS
  online ancora aperti.

## TODO

### Prossimo passo diretto (questa linea di lavoro)
- **Costruzione della reward task-based che usa il clock**: forma dei termini di
  **auto-periodicità** (confronto col ciclo precedente alla stessa `φ`),
  **coordinazione/anti-fase** col lato sano, e **obiettivi di task**
  (stabilità, fattibilità GRF, effort/energia SEA, ROM); pesi/blend; gradiente
  residuo. Il clock è l'infrastruttura; la reward che lo sfrutta è il passo
  successivo (deferred per scelta in
  [[2026-06-09_durata_episode_phase_clock_ex_novo]]).
- **Approfondire la strategia di pre-training imitativo** (aggancio al template
  `φ`-indicizzato dalle colonne `pros_*` del `.mot`; criterio di passaggio a
  task-based).
- **Tarare `gait_clock_phase_offset`** (HS sano vs toe-off vs mid-stance) per il
  training più stabile.
- Valutare **episodi lunghi** (i ~143 s di dati lo consentono) e `random_init`
  con il clock attivo.

### TODO ereditati e propagati (ancora aperti)
- Far sì che il **timeout di iterazione/sampling** uccida davvero un'iterazione
  in stallo (oggi solo il run-timeout totale ha effetto); troncare gli episodi
  degeneri che bloccano il sampling.
- Ridurre alla radice le divergenze `joint_divergence_pros_knee_angle`, la
  saturazione del knee SEA e i fallback bounded least-squares della Static
  Optimization.
- Eseguire un rollout deterministico di `rl_module_best` del run ibrido notturno
  (2 s, record) per misurare cosa ha imparato in ~40 iter.
- Correggere i due bug HS **online** (timing heel-strike protesico; flag
  `in_contact` sempre attivo): prerequisito solo per varianti "evento discreto"
  / full task-based, NON per il clock qui implementato.
- Allineamento macOS: `setuptools<81` in `envCMC-rllib`; ricompilare/riconfermare
  il plugin onlineGRF `.dylib` su arm64.
- Migliorare il critic (explained variance negativa); tarare `oob_weight`;
  rivalidare il filtro 6 Hz su rollout lungo.
- Metriche di suitability ex-novo (velocità, stabilità, simmetria, effort,
  energia SEA, fattibilità GRF).
- Housekeeping repo (inclusi script temp `validation/_*.py`, tra cui
  `_gait_clock_absaction_smoke.py`), knowledge base letteratura, controllo SEA
  storici.
```
