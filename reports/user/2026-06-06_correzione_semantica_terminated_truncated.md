# Correzione semantica `terminated` / `truncated`

## Problema

L'ambiente RL condiviso classificava gli eventi di fine episodio con una
semantica invertita rispetto al contratto Gymnasium:

- il raggiungimento della durata episodio o della fine del dataset produceva
  `terminated=True`;
- caduta e divergenza articolare producevano `truncated=True`.

Questa classificazione era problematica per il calcolo dei target del value
function. Un limite temporale non indica uno stato terminale del task e deve
permettere il bootstrap; una caduta o divergenza, invece, rappresenta uno stato
terminale unsafe e non deve fare bootstrap.

La correzione doveva essere applicata sia alla baseline MLP RLlib sia al PPO_SNN
custom, senza effettuare uno scambio cieco dei due flag.

## Soluzione

### Ambiente condiviso

La semantica di fine episodio in `osim_trj_cmc_like.py` è ora:

| Evento | `terminated` | `truncated` | `info["end_reason"]` |
|---|---:|---:|---|
| Caduta | `True` | `False` | `fall` |
| Divergenza articolare | `True` | `False` | `joint_divergence:<coord>` |
| Limite temporale episodio | `False` | `True` | `episode_time_limit` |
| Fine dataset | `False` | `True` | `dataset_end` |
| Errore numerico catturato | `False` | `True` | `numerical_failure` |

Gli eventi unsafe hanno priorità se coincidono con la fine temporale. La safety
penalty è applicata soltanto agli stati unsafe, indipendentemente dal nome dei
flag.

La baseline MLP RLlib usa direttamente i flag corretti esposti dall'ambiente e
non richiede una modifica custom al calcolo GAE.

### PPO_SNN custom

Il PPO_SNN ora memorizza anche `next_values` per ogni transizione e usa due
maschere distinte nel GAE:

- `bootstrap_mask = not terminated`: permette il bootstrap sui troncamenti ma
  non sugli stati terminali;
- `continuation_mask = not (terminated or truncated)`: impedisce che il GAE
  propaghi vantaggi tra episodi diversi.

Gli stati ricorrenti vengono azzerati sia su `terminated` sia su `truncated`.
Durante l'update, il modello ricorrente riceve come `done` l'OR dei due flag.

## Strategia

- Verificata la semantica Gymnasium/RLlib prima di modificare il codice.
- Distinti gli eventi appartenenti al task dagli eventi esterni al task.
- Conservata la safety penalty per caduta e divergenza.
- Corretto il GAE custom PPO_SNN, dove il semplice scambio dei flag non sarebbe
  stato sufficiente.
- Esposta la causa precisa tramite `info["end_reason"]` per training,
  diagnostica e analisi post-run.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
  - classificazione corretta degli eventi di fine episodio;
  - aggiunta di `end_reason`;
  - safety loss legata agli stati unsafe.
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py`
  - memoria di `next_values`;
  - maschere GAE separate per bootstrap e continuazione;
  - reset ricorrente su entrambi i tipi di fine episodio.
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_train.py`
  - registrazione degli `end_reason` reali.
- `Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py`
  - test dedicati a terminale e troncamento nel GAE.
- `Trajectory Generator/baseline_MLP/reward_function.py`
  - documentazione coerente della safety penalty.
- `Trajectory Generator/baseline_MLP/README.md`
  - documentazione della semantica Gymnasium.
- `validation/rl_env_smoke_ab06_pi.py`
  - aspettativa corretta per la fine per time limit.
- `validation/rl_env_rollout_ab06_pi.py`
  - aspettativa corretta per la fine del rollout.
- `validation/_val_codex_diag.py`
  - diagnostica aggiornata.
- `validation/_val_long_rollout.py`
  - uso di `end_reason` al posto dell'errata equivalenza
    `truncated = divergenza`.

## Verifiche eseguite

- PPO_SNN smoke test: **PASS**, inclusi i nuovi test GAE:
  - un troncamento usa `next_value` per il bootstrap ma non propaga il vantaggio
    dell'episodio successivo;
  - una terminazione non usa `next_value`.
- `py_compile` sui file Python modificati: **PASS**.
- `py_compile` nell'ambiente `envCMC-rllib`: **PASS**.
- `git diff --check`: **PASS**; presenti soltanto warning di conversione
  LF/CRLF.
- Audit dei call-site `_compute_gae` e delle vecchie aspettative
  `terminated`/`truncated`: nessun riferimento incoerente rilevato.

Il rollout OpenSim completo non è stato rilanciato: una precedente esecuzione
diagnostica lunga aveva raggiunto il timeout. La classificazione e il calcolo
GAE sono stati verificati con test mirati.

## Correzione al report precedente

Nel report
`2026-06-06_penalty_out_of_band_e_troncamento_per_giunto.md`, la verifica finale
descriveva la normale fine dell'episodio come `terminated` e non `truncated`.
Dopo questa correzione, la normale fine temporale deve essere letta come
`truncated=True`, `terminated=False`; i bound anti-divergenza producono invece
una vera terminazione unsafe.

## TODO aperti

- Eseguire un rollout OpenSim completo e confermare nei risultati:
  `episode_time_limit`/`dataset_end` come troncamenti e
  `fall`/`joint_divergence` come terminazioni.
- Il training reale controllato da 50 iterazioni è stato completato il
  2026-06-07: `48` episodi sono terminati per `episode_time_limit` e `2` per
  `joint_divergence_pros_knee_angle`, confermando la distinzione tra
  troncamenti e terminazioni. Resta da migliorare il critic e ripetere la
  validazione su orizzonti full-gait.
- Tarare `oob_weight` usando `reward/oob_term` in TensorBoard.
- Completare il reward rebalancing F2, il porting delle gait metrics
  RMSE/Symmetry/Trend e l'integrazione SNN come custom RLModule.
