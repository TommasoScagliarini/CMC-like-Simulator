# Validazione env RL: rollout lunghi, stato residuo, costo reset (Mac)

Data: 2026-05-31

## Problema

Il commit `9c52c5b` ("RL env created, to be validated") lasciava l'ambiente RL
di traiettoria `osim_trj_cmc_like.py` validato solo con uno smoke da 2 ms su
Windows. Restavano aperti i TODO del daily 2026-05-29:

- validare rollout piu' lunghi (20-50 ms, poi 0.5-1 s);
- verificare che il reset riusabile (`rebuild_model_on_reset=False`) non lasci
  stato residuo nel plugin SEA, nel controller o nella static optimization;
- misurare il costo di `reset()` con rebuild `True` vs `False`;
- monitorare il fallback dello static optimizer.

Inoltre, nel working tree il file dell'env era stato **spostato** dalla root al
nuovo contenitore `Trajectory Generator/` (in git: `D osim_trj_cmc_like.py` +
copia da 787 righe in `Trajectory Generator/osim_trj_cmc_like.py`), insieme al
sottoprogetto `Prosthesis_SNN` e a una cartella `build/` CMake untracked. Lo
spostamento aveva **rotto l'import** dello smoke esistente: `validation/
rl_env_smoke_ab06_pi.py` mette solo la repo root in `sys.path` e fa
`from osim_trj_cmc_like import ...`, ma il file non e' piu' in root.

La validazione del 29/05 era inoltre su Windows; questo Mac (macOS arm64) non
era mai stato verificato con lo stack validato (plugin SEA `.dylib`).

## Soluzione

1. **Import resiliente**: lo smoke ora aggiunge a `sys.path` anche
   `Trajectory Generator/`, mantenendo la repo root con precedenza (cosi'
   `config`, `simulation_runner`, ecc. risolvono sull'albero canonico).
   L'env si importa quindi dalla nuova posizione senza spostare nulla.

2. **Nuovo validatore** `validation/rl_env_rollout_ab06_pi.py` che copre i tre
   TODO aperti in modo deterministico (`random_init=False`, seed fissi, azioni
   scriptate):
   - **Test L** - rollout lungo (30 e 100 step), controllo finitezza,
     monotonicita' del tempo, terminazione pulita, reward/obs limitati,
     saturazione per coordinata protesica;
   - **Test R** - determinismo/stato residuo: due episodi identici sullo stesso
     env e confronto con un env ricostruito da zero;
   - **Test C** - costo `reset()` con riuso vs rebuild del modello.

Nessuna modifica al plugin C++ ne' alla semantica del comando SEA. Modifiche
solo a file di validazione.

## Strategia

- Validare **prima la piattaforma** (import env, smoke 2 ms) su Mac, poi
  estendere ai rollout lunghi.
- Rendere la prova di stato residuo **empirica e forte**: se il reset riusabile
  e' pulito, due episodi identici devono coincidere **bit a bit**, e un episodio
  riusato deve coincidere bit a bit con uno su modello appena costruito.
- Usare azioni scriptate piccole e lisce (sinusoidi seedate) per **eccitare** la
  memoria del controller (integrali cascade, LPF) senza uscire dal fisiologico.
- Limitare il numero di build del modello (ogni build carica modello + plugin +
  GRF da 143056 righe) riusando gli env tra i sotto-test.

## File modificati

```text
validation/rl_env_smoke_ab06_pi.py   (modificato: import resiliente alla
                                       nuova posizione dell'env)
validation/rl_env_rollout_ab06_pi.py (nuovo: rollout/residual/reset-cost)
```

Contesto ambiente:

```text
conda env : envCMC-like  (Python 3.10.20)
OpenSim   : 4.5.2-2026-02-28-e0471ab50
plugin    : plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dylib
modello   : models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim
setup     : models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
```

## Test e verifiche eseguite

### Smoke 2 ms (piattaforma Mac)

```bash
conda run --no-capture-output -n envCMC-like \
  python validation/rl_env_smoke_ab06_pi.py
```

Esito: passato (`rl_env_smoke_ab06_pi_ok`). Reset a 11.99, step a 11.992,
action sanity `0.0/+0.1/-0.1` tutte `terminated=True, truncated=False`,
osservazioni e reward finiti. Plugin caricato come `.dylib`, 21 coordinate
(19 bio, 2 pros), 76 muscoli, props SEA PI lette (`Ki`,
`integral_torque_limit` presenti).

### Rollout lunghi (Test L)

```bash
conda run --no-capture-output -n envCMC-like \
  python validation/rl_env_rollout_ab06_pi.py --reset-reps 2
conda run --no-capture-output -n envCMC-like \
  python validation/rl_env_rollout_ab06_pi.py --rollout-dur 1.0 \
  --residual-dur 0.02 --skip-cost
```

| durata | step | wall  | monotòno | terminated | truncated | nonfinite | reward[min,max] | obs_norm[min,max] | sat knee | sat ankle |
|--------|-----:|------:|----------|------------|-----------|----------:|-----------------|-------------------|---------:|----------:|
| 0.30 s |  30  | 31.8 s| True     | True       | False     | 0         | [0.37, 0.92]    | [9.2, 69.6]       | 21/30    | 0/30      |
| 1.00 s | 100  |104.4 s| True     | True       | False     | 0         | [0.33, 0.92]    | [5.55, 69.6]      | 74/100   | 0/100     |

Entrambi PASS. Nessun blow-up su 1 s nonostante il knee sia saturo nel 74% dei
passi.

### Determinismo / stato residuo (Test R)

```text
reuse  A1 vs A2  d_reward 0.000e+00  d_obs_norm 0.000e+00
fresh  A2 vs B   d_reward 0.000e+00  d_obs_norm 0.000e+00
```

Differenza **bit a bit nulla** sia tra due episodi riusati, sia tra episodio
riusato e modello ricostruito da zero - **anche attraverso uno step in cui la
SO fa backtracking**. Il reset riusabile non lascia stato residuo.

### Costo reset (Test C)

```text
reuse_reset_s    mean 0.00317  [0.00325, 0.00309]
rebuild_reset_s  mean 5.6950   [5.7011, 5.6889]
rebuild/reuse    ~1796x
```

Il reset riusabile (3.2 ms) e' circa **1800x** piu' economico del rebuild del
modello (5.7 s). Conferma `rebuild_model_on_reset=False` come default per PPO.

### Static optimizer

Warning osservati su tutta la run: 3 `feasibility backtracking did not meet
tolerance` (a `t=12.0300`, `scale=0.0039`, `|res|=254`, `rel=0.0237` -> ~2.4%
non soddisfatto) e 1 `QP did not converge` all'init del primo build. Caso
peggiore all'avvio AB06 (azione zero): `scale=1`, `rel=0.44`. Comportamento
**deterministico e non bloccante**: il rollout resta finito e i confronti
bit-a-bit tornano.

## Note operative e finding

- **Saturazione knee**: il comando normalizzato `u` del knee satura molto
  facilmente (21/30 a ampiezza azione 0.10, 74/100 su 1 s). Perturbazioni
  modeste del riferimento knee a 100 Hz (segment 10 ms) generano, via derivata
  dello spline e gain alti del cascade, velocity-ref grandi (`cascade_qdot_ref`
  ~25 rad/s) che mandano in saturazione il SEA. L'ankle resta comodo (picco
  0.21-0.84, mai saturo). Questo tocca le decisioni aperte in
  `Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md`: policy timing,
  scaling dell'action space, cap su `qddot_ref` nei truncation criteria.
- **`gymnasium` non installato** in `envCMC-like`: l'env usa il fallback shim
  interno e funziona per gli smoke, ma va installato (con `skrl`) prima del PPO
  reale.

## TODO

### Chiusi oggi

- Validazione piattaforma su macOS arm64 (plugin `.dylib`, smoke 2 ms).
- Rollout lunghi fino a 1 s: stabili, finiti, terminazione pulita.
- Stato residuo del reset riusabile: assente (determinismo bit-a-bit, riuso ==
  modello fresco).
- Costo `reset()` rebuild vs riuso: ~1800x, conferma default `False`.
- Import dello smoke riparato dopo lo spostamento dell'env in
  `Trajectory Generator/`.

### Nuovi / aperti

- **Layout git**: decidere se committare lo spostamento di
  `osim_trj_cmc_like.py` in `Trajectory Generator/` e aggiungere `build/` al
  `.gitignore`.
- **Ambiente PPO**: installare `gymnasium` (+ `skrl`) in `envCMC-like`.
- **Reward**: tarare i pesi dopo i primi rollout reali.
- **SO feasibility iniziale AB06**: valutare target/bounds biologici alla
  partenza (rel fino a 0.44 con azione zero) prima di training lunghi.
- **Saturazione knee / action design**: rivedere segment_duration, scaling
  azione e cap su accelerazione di riferimento (collegato a TODO_integration).

### Propagati (non affrontati oggi)

Dal 2026-05-29 e precedenti, ancora aperti:

- Sweep `Kp_knee_motor` (3.9-18) con ankle morning best; validazione coupling
  knee-ankle isolando la dinamica knee; notch 28 Hz opzionale sul feedback knee.
- LPF qdot: non promosso a default (knee RMSE +66% non accettato); restano test
  asimmetrico (solo ankle), cutoff 30/35 Hz, run lunga 30+ s.
- Windows: build/copia DLL plugin PI documentata.
- Confronto consolidato finale tra config storiche (PD, PI, cascade, zeta07,
  pi-tuned, J_eff, Opzione D).
- Cleanup artefatti sweep `_cascade_local_gain_sweep_20260517_*` e modelli
  sperimentali (`opt_b`, `opt_c`, `jeff_v3cap*`, `slow_inner_pd_1405`,
  `pi_asym_knee1405`).
- Decidere se promuovere `LLM_SIMULATOR_OVERVIEW.md` nel flusso `start_day`.
