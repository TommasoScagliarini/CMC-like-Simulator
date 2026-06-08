# Trajectory generator RL: training, inference e checkpoint best/last

## Problema

L'obiettivo della giornata era ottenere un trajectory generator RL eseguibile
end-to-end sul setup AB06 PI, utile per iniziare test reali anche senza tuning
definitivo. Serviva chiudere il contratto env/SNN, aggiungere una CLI di
training, una CLI di inference/rollout, checkpoint utilizzabili a runtime e
metriche minime per iniziare a capire reward, observation space e safety.

## Soluzione

E' stato implementato un percorso PPO/SNN funzionante dentro
`Trajectory Generator/`, senza modificare il plugin C++ SEA.

Il training ora produce:

- `agent.pt` e `reference.pt`: alias compatibili dell'ultimo modello salvato.
- `last_agent.pt` e `last_reference.pt`: ultimo modello salvato.
- `best_agent.pt` e `best_reference.pt`: miglior episodio secondo
  `episode_return`.
- `summary.json`: metriche di reward, termini diagnostici, action shape,
  feature names, conteggi termination/truncation e riferimenti ai checkpoint.

`best_reference.pt` e' il checkpoint preferito per inference, perche' contiene
solo la SNN esportata e i metadata necessari a verificare il contratto con
l'ambiente.

## Strategia

- Mantenere il contratto SNN -> env come `env_action`.
- Usare output SNN reshaped a `(policy_knots, n_prosthetic_coords)`.
- Usare feature privilegiate reali dell'ambiente, salvate per nome nel
  checkpoint.
- Rendere l'inference strict: fallisce se action shape o feature names non
  coincidono tra checkpoint ed env.
- Aggiungere reward terms configurabili per tracking, effort, smoothness,
  saturazione SEA e safety/truncation.
- Rendere `PPO_SNN` robusto rispetto alle firme skrl e al riuso accidentale
  del grafo autograd tra update.
- Documentare i comandi operativi in `Trajectory Generator/commands.txt`.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_train.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/cmc_policy_rollout.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/ppo_snn.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/__init__.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/config.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/generator.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/reference_provider.py`
- `Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/cmc_ppo_smoke.py`
- `Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py`
- `Trajectory Generator/Prosthesis_SNN/README.md`
- `Trajectory Generator/commands.txt`

## Verifiche eseguite

- Import check in `envCMC-like`: OpenSim, Torch, snntorch, skrl e gymnasium.
- `py_compile` sui moduli RL/env modificati.
- `Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py`.
- Tiny training AB06 PI con 2 timesteps e 2 rollouts.
- Tiny inference da `best_reference.pt`.
- Smoke training 50 ms.
- Verifica metadata di `best_reference.pt`:
  `checkpoint_role=best`, `selection_metric=episode_return`,
  `action_contract=env_action`, `action_shape=[3, 2]`.

## Risultato

Il generatore RL ora e' eseguibile per training e inference. Produce checkpoint
last/best e summary leggibili, quindi e' possibile iniziare test iterativi senza
dover ancora avere reward e observation space definitivi.

## TODO aperti

- Validare il trajectory generator RL: il generatore funziona tecnicamente, ma
  e' ancora da validare su rollout piu' lunghi, stabilita', tracking protesico,
  saturazioni SEA, comportamento biologico e qualita' della traiettoria.
- Tuning reward: pesare meglio tracking, riferimento IK, safety, effort,
  smoothness e saturazione.
- Definizione observation space: ridurre/strutturare le feature privilegiate e
  decidere cosa sara' disponibile in scenari meno privilegiati.
- Confronto con letteratura: usare la literature review per decidere reward,
  osservazioni, curriculum, vincoli di safety e metriche di validazione.
- Aggiungere checkpoint periodici per training lunghi e gestione piu' completa
  di interruzioni/crash.
