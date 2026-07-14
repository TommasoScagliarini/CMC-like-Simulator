# Target slew limiter per reward ex-novo

Data report: 2026-07-07

## Problema

Dopo la correzione della FSM/reward anti-fake-cycle, il training diagnostico
H2 non falliva piu' per conteggio di cicli fasulli, ma per terminazione precoce
`grf_penetration`.

La diagnostica ha mostrato che la policy, in `action_mode: absolute`, poteva
chiedere target assoluti troppo lontani dalla posa corrente. Nel rollout H2, al
primo step la policy chiedeva:

```text
raw_action = [0.4718, -0.1893]
target knee  = -0.396 rad
target ankle = -0.133 rad
```

mentre lo stato al reset era circa:

```text
knee  = -0.174 rad
ankle =  0.052 rad
```

Quindi il primo segmento da 10 ms chiedeva uno spostamento di circa:

```text
delta knee  = -0.222 rad
delta ankle = -0.185 rad
```

Il reference governor limitava velocita' e accelerazione, ma iniziava comunque
a inseguire un target lontano. Questo caricava il SEA e il contatto online
sinistro, portando la penetrazione GRF oltre la soglia hard:

```text
step 10: grf_penetration = 17.923 mm -> terminated
```

## Soluzione

E' stato aggiunto un limite generale target-to-target sulla generazione degli
step. Il limite non e' una eccezione sul primo frame: vale in ogni istante
dell'episodio e per ogni knot della traiettoria generata.

Logica:

```text
q_policy_raw = target assoluto chiesto dalla rete
q_prev       = target/reference corrente o knot precedente
q_served     = q_prev + clamp(q_policy_raw - q_prev, +/- limit * dt)
```

La policy continua a emettere un target assoluto, ma il simulatore riceve solo
un target cinematicamente raggiungibile rispetto al target precedente.

La configurazione ex-novo ora abilita:

```yaml
pros_knee_target_slew_rate_limit_rad_s: 2.5
pros_ankle_target_slew_rate_limit_rad_s: 2.0
```

A 100 Hz questi valori corrispondono a:

```text
knee  max 0.025 rad/step
ankle max 0.020 rad/step
```

## Strategia

La scelta e' stata limitare il target servito prima della simulazione dinamica,
non solo penalizzare a posteriori il target lontano. Una penalita' da sola
arriverebbe dopo lo step; se lo step produce subito `grf_penetration`, PPO
raccoglie episodi corti e poco informativi.

Il limiter:

- preserva `action_mode: absolute`;
- evita salti di target in reset e mid-episode;
- lascia visibile quanto la policy viene corretta;
- non modifica il plugin SEA ne' la semantica del comando SEA;
- non alza la soglia di penetrazione GRF.

Sono state aggiunte diagnostiche reward:

```text
target_slew_limit_enabled
target_slew_limited_fraction
target_slew_abs_delta_raw_max_rad
target_slew_abs_delta_served_max_rad
target_slew_excess_abs_max_rad
target_slew_excess_loss
```

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
```

Dettagli:

- `osim_trj_cmc_like.py`: aggiunto `target_slew_rate_limit_rad_s` a
  `CMCEnvConfig`, risoluzione scalar/mapping/list, clamp target-to-target in
  `_action_to_segment()`, diagnostiche `target_slew_*`.
- `training_config.py`: aggiunte le nuove chiavi YAML/CLI alla mappa di config.
- `train_ppo_mlp.py`: passaggio dei limiti knee/ankle dentro `env_config`.
- `rollout_eval.py`: stesso wiring del training, per coerenza tra training e
  rollout.
- `training_exnovo_cfg.yaml`: abilitati i limiti ex-novo iniziali.

## Verifiche

### Test mirato in memoria

E' stata rieseguita la stessa sequenza di azioni del rollout H2, con stesso
start e stesso ambiente OpenSim, ma con limiter attivo.

Prima:

```text
step 10: grf_penetration = 17.923 mm
terminated = true
```

Dopo:

```text
step 10: grf_penetration = 0.114 mm
terminated = false
target_slew_abs_delta_raw_max_rad = 0.5727
target_slew_abs_delta_served_max_rad = 0.0250
```

### Smoke CLI rollout

Comando eseguito con il vecchio checkpoint diagnostico H2, nuova config e
20 step metric-only:

```text
Trajectory Generator/runs/rollout/2026-07-07_diag10_reward_validation_limiter_smoke20/
```

Risultato:

```text
ok = true
steps = 20
episode_return = 2.4887987376508005
reward_mean = 0.12443993688254003
reward_min = 0.04280467113647023
reward_max = 0.1935700761290142
terminated = false
truncated = false
action_clipped_fraction = 0.0
record_outputs = false
```

### Test automatici

```text
py_compile: PASS
validation/validate_training_config.py: PASS
validation/test_reward_function.py: PASS (27)
git diff --check: PASS
```

## Stato e prossimi passi

Il failure mode immediato `grf_penetration` causato da target assoluti troppo
lontani e' stato mitigato nel test mirato e nello smoke rollout CLI.

TODO:

1. Eseguire un rollout registrato con `--record-outputs` usando la nuova config.
2. Generare plot dedicati per verificare GRF, penetrazione, target raw/served,
   coordinate protesiche e termini `target_slew_*`.
3. Ripetere il gate H2 completo: il rollout deve mostrare almeno una sequenza
   `HS -> TO` senza terminazione precoce.
4. Solo dopo H2 PASS riprendere training diagnostico 10 iterazioni e poi il
   gate 20-50 iterazioni.

