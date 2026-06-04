# Actor-Critic SNN skrl e chiarimento reward

Data: 2026-06-01

## Problema

Il package `Trajectory Generator/Prosthesis_SNN` aveva gia' il modello SNN
portabile (`ProsthesisReferenceSNN`) e l'adattamento `PPO_SNN`, ma mancava il
wrapper skrl actor-critic necessario per usare la rete in training PPO:

- policy gaussiana;
- value head;
- log-probability ed entropy;
- stato di membrana SNN esposto come `rnn`;
- compatibilita' con il caso policy/value condivisi.

Durante la discussione e' emerso anche un dubbio sulla voce "reward/truncation
minimale": nel codice dell'env RL una reward esiste gia', quindi quella voce
non indicava una reward assente.

## Chiarimento sulla reward

La reward e' gia' implementata in:

```text
Trajectory Generator/osim_trj_cmc_like.py
```

La configurazione contiene pesi per:

- tracking protesico;
- riferimento/base IK;
- tracking biologico;
- effort;
- smoothness.

La funzione `_get_reward()` calcola:

- `tracking_loss`;
- `reference_loss`;
- `bio_loss`;
- `effort_loss` basata su `u_sea`;
- `smoothness_loss` sulle policy knots;
- score normalizzati e reward finale clippata in `[0, 1]`.

Sono presenti anche criteri di truncation in `_is_truncated()`:

- altezza minima pelvis;
- limite assoluto su coordinate protesiche.

Quindi il punto "reward/truncation minimale" va interpretato meglio come:

```text
reward gia' presente, ma ancora da validare/tarare per training PPO SNN
```

In particolare restano da valutare:

- pesi effettivi della reward su rollout lunghi;
- eventuale penalita' esplicita sulla saturazione di `u`;
- coerenza soft tra `q_ref`, `qdot_ref`, `qddot_ref`;
- soglie di truncation adatte al training, non solo allo smoke dell'env.

## Soluzione implementata

E' stato aggiunto:

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/actor_critic.py
```

con `ProsthesisSNNActorCritic`, wrapper skrl condiviso per policy e value.

La struttura segue il riferimento:

```text
TommasoScagliarini/SNN-Colangelo-Mardaru-Scagliarini
```

in particolare il pattern di `src/model/snn_model.py`:

- un solo backbone SNN condiviso;
- due teste non-spiking LIF, una policy e una value;
- stato di membrana esposto tramite `get_specification()["rnn"]`;
- cache tra chiamata policy e chiamata value sullo stesso input;
- stesso oggetto usabile come `models["policy"]` e `models["value"]`.

## Strategia

La policy head usa lo stesso formato di `ProsthesisReferenceSNN`, cosi' i pesi
policy possono essere esportati verso `ReferenceGenerator` per inference.

La value head resta training-only e non entra nel contratto inferenziale del
simulatore.

Sono state aggiunte piccole compatibilita' per `skrl 2.1.0`, che differisce
dall'API usata nel codice upstream:

- config con metodo `.expand()`;
- metodo pubblico `update()`;
- firma `act(observations, states=None, *, timestep, timesteps)`;
- firma keyword-only per `record_transition()`;
- `Memory.create_tensor()` con argomenti keyword-only;
- gestione assente di `secondary_memories`.

## File modificati

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/actor_critic.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py
Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py
Trajectory Generator/Prosthesis_SNN/README.md
Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md
```

## Test e verifiche

Compilazione:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/actor_critic.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py' \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Smoke test:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Esito:

```text
smoke tests passed
```

Lo smoke verifica:

- forme output SNN;
- `ReferenceGenerator`;
- override protesico via provider ibrido;
- `ProsthesisSNNActorCritic.act()` per policy e value;
- presenza e forma degli stati `rnn`;
- entropy e log-probability;
- esportabilita' dello state dict verso `ProsthesisReferenceSNN`;
- inizializzazione `PPO_SNN`;
- `agent.act()`;
- `agent.record_transition()`.

## Passi successivi consigliati

Il prossimo blocco naturale e' il training entrypoint: collegare env AB06,
memory skrl, `ProsthesisSNNActorCritic`, `PPO_SNN`, trainer, checkpointing e
reload tramite `ReferenceGenerator`.

Prima di training lunghi conviene trattare la reward come "presente ma non
ancora validata": fare rollout diagnostici e controllare se i termini premiano
davvero cio' che vogliamo, soprattutto in presenza di saturazione del comando
SEA.
