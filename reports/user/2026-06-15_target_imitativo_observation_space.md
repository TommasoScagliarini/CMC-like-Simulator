# Target imitativo nell'observation space

Data: 2026-06-15

## Problema

Nel training imitativo la policy riceveva il punteggio di reward legato al riferimento sano anti-fase, ma non osservava esplicitamente quel target. In pratica la rete doveva dedurre il target desiderato solo dal clock di gait e dal segnale di reward.

Questo rendeva piu difficile imparare curve cinematiche coerenti con il riferimento sano, soprattutto dopo aver verificato che il controller SEA segue correttamente il riferimento servito: il problema residuo era principalmente nella forma del riferimento generato dalla policy.

## Soluzione implementata

Sono state aggiunte quattro feature actor, attive solo quando `reward_mode: imitation`:

```text
healthy_knee_angle_imitation_target
healthy_knee_angle_imitation_target_vel
healthy_ankle_angle_imitation_target
healthy_ankle_angle_imitation_target_vel
```

Queste feature non sono coordinate sane raw allo stesso istante, ma il target sano anti-fase gia calcolato da `self.imitation_target(self.t)`, mappato sulle coordinate protesiche:

```text
healthy_knee_angle_imitation_target      = q_target["pros_knee_angle"]
healthy_knee_angle_imitation_target_vel  = qdot_target["pros_knee_angle"]
healthy_ankle_angle_imitation_target     = q_target["pros_ankle_angle"]
healthy_ankle_angle_imitation_target_vel = qdot_target["pros_ankle_angle"]
```

Non e stata modificata la reward.

## Strategia

La modifica e stata tenuta volutamente locale:

- nessun nuovo flag nello YAML;
- nessun refactor strutturale di `osim_trj_cmc_like.py`;
- gating automatico da `reward_mode`;
- `ex_novo` mantiene lo schema observation precedente;
- `imitation` riceve le quattro feature in piu nell'actor observation.

In `env_factory.make_cmc_env`, il flag interno viene impostato cosi:

```python
raw["include_imitation_target_observation"] = (
    reward_cfg.reward_mode == "imitation"
)
```

Quindi il comportamento dipende direttamente dalla sezione `reward:` della configurazione.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
  - aggiunto `CMCEnvConfig.include_imitation_target_observation`;
  - aggiunta la logica in `_get_observation` per inserire le quattro feature nell'`actor` dict.
- `Trajectory Generator/baseline_MLP/env_factory.py`
  - collegato il flag interno a `reward_cfg.reward_mode == "imitation"`.

## Verifiche eseguite

Compilazione Python:

```powershell
python -m py_compile "Trajectory Generator/osim_trj_cmc_like.py" "Trajectory Generator/baseline_MLP/env_factory.py"
```

Esito: passato.

Smoke test environment:

```text
reward_mode: ex_novo
  obs_shape = 31
  n_actor = 31
  new_fields = []

reward_mode: imitation
  obs_shape = 35
  n_actor = 35
  new_fields =
    healthy_knee_angle_imitation_target
    healthy_knee_angle_imitation_target_vel
    healthy_ankle_angle_imitation_target
    healthy_ankle_angle_imitation_target_vel
```

Esito: `ex_novo` resta invariato; `imitation` aumenta correttamente di 4 feature actor.

## Implicazioni operative

La dimensione dell'observation space cambia in modalita imitativa, quindi i checkpoint del training precedente non sono compatibili con questa nuova configurazione.

Il prossimo training con `reward_mode: imitation` deve partire da zero, senza `--resume-from`.

Comando consigliato:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --config "Trajectory Generator\baseline_MLP\training_cfg.v4_imitation.yaml" --output-dir "runs\baseline_mlp_imit_v4_c2_4hz_obs_target"
```

## Verifica post-training consigliata

Dopo il nuovo training conviene eseguire un rollout breve e confrontare:

- `served-target`;
- `actual-target`;
- andamento delle coordinate cinematiche rispetto al target sano anti-fase;
- eventuale saturazione SEA;
- eventuale clipping/aggressivita del comando policy.
