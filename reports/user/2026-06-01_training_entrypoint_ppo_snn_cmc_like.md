# Training entrypoint PPO/SNN CMC-like e smoke end-to-end

Data: 2026-06-01

## Problema

Dopo aver aggiunto il wrapper skrl actor-critic e il checkpoint inference-only,
mancava un entrypoint che collegasse davvero:

- environment CMC-like;
- skrl memory;
- `ProsthesisSNNActorCritic`;
- `PPO_SNN`;
- una micro-run PPO;
- checkpoint agent;
- export checkpoint inference;
- reload tramite `ReferenceGenerator`.

Lo script non deve essere specializzato su AB06: il setup del modello deve
arrivare da CLI o dal setup persistito del simulatore.

## Soluzione

E' stato aggiunto:

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_smoke.py
```

L'entrypoint e' generico rispetto al modello/setup:

```bash
python -m prosthesis_snn.training.cmc_ppo_smoke --setup-xml path/to/setup.xml
```

Se `--setup-xml` e' omesso, viene usato il setup persistito del simulatore,
come previsto da `CMCLikeProsthesisTrajectoryEnv`.

La pipeline eseguita e':

```text
CMCLikeProsthesisTrajectoryEnv
-> RandomMemory
-> ProsthesisSNNActorCritic
-> PPO_SNN
-> una update PPO
-> agent checkpoint
-> save_reference_checkpoint
-> ReferenceGenerator.from_checkpoint
-> predict finito
```

## Strategia

L'env RL attuale usa azioni come knot di traiettoria:

```text
action shape = (policy_knots, n_prosthetic_coords)
```

Lo script legge l'action space dall'env e costruisce il config SNN coerente
con quella forma. Per esempio, con il setup AB06 usato nei test:

```text
policy_knots = 3
action shape = (3, 2)
flattened action size = 6
output_coords = (knot_1, knot_2, knot_3)
output_channels = (pros_knee_angle, pros_ankle_angle)
```

Importante: in questo smoke i sei output sono semanticamente knot di
traiettoria, non ancora il contratto finale:

```text
(q, qdot, qddot) x (knee, ankle)
```

Il mismatch resta un TODO esplicito.

## File modificati

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_smoke.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
Trajectory Generator/Prosthesis_SNN/README.md
Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md
```

`ppo_snn.py` e' stato aggiornato per accettare `trainer_cfg` dict in modo
compatibile con `skrl 2.1.0`.

## Test e verifiche

Compilazione:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_smoke.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/actor_critic.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/checkpoint.py'
```

Smoke package:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Esito:

```text
smoke tests passed
```

Smoke end-to-end eseguito su setup AB06 tramite CLI:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m prosthesis_snn.training.cmc_ppo_smoke \
  --setup-xml models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --output-dir runs/prosthesis_snn_cmc_ppo_smoke_test \
  --timesteps 2 \
  --rollouts 2 \
  --episode-duration 0.004 \
  --segment-duration 0.002
```

Risultato:

```text
ok: true
timesteps: 2
rollouts: 2
times: [11.992, 11.994]
terminated_count: 1
truncated_count: 0
observation_size: 52
action_shape: [3, 2]
reload_output_finite: true
```

Verificato anche resume/load:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m prosthesis_snn.training.cmc_ppo_smoke \
  --setup-xml models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --output-dir runs/prosthesis_snn_cmc_ppo_smoke_resume_test \
  --timesteps 2 \
  --rollouts 2 \
  --episode-duration 0.004 \
  --segment-duration 0.002 \
  --resume-agent-path runs/prosthesis_snn_cmc_ppo_smoke_test/agent.pt \
  --load-reference-path runs/prosthesis_snn_cmc_ppo_smoke_test/reference.pt
```

Risultato:

```text
ok: true
terminated_count: 1
truncated_count: 0
reload_output_finite: true
```

Durante gli smoke compaiono warning SLSQP di clipping bounds nella static
optimization. La run resta valida: nessun truncation, reward finite,
osservazioni finite, checkpoint esportato e ricaricato.

## Stato finale

Sono chiusi a livello smoke:

- training entrypoint minimale CMC-like;
- collegamento env -> memory -> actor-critic -> PPO_SNN;
- una update PPO reale;
- checkpoint agent;
- checkpoint inference-only;
- reload via `ReferenceGenerator`;
- resume/load da checkpoint agent/reference.

## TODO aperti

- Risolvere il mismatch di contratto tra action space corrente
  `(policy_knots, n_prosthetic_coords)` e contratto SNN/inference desiderato
  `(q, qdot, qddot) x (knee, ankle)`.
- Promuovere lo smoke a training lungo con config dedicate, logging,
  checkpoint layout stabile e criteri di stop.
- Validare/tarare reward, normalizzazione input/output, observation space e
  penalita' sulla saturazione di `u` prima di training PPO lunghi.
