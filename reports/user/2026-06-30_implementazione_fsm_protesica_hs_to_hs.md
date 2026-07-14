# 2026-06-30 - Implementazione FSM protesica HS-TO-HS

## Problema

La reward ex-novo aveva bisogno di una macchina di fase protesica runtime che rendesse esplicita la sequenza:

```text
HS -> stance caricata -> TO -> swing scarico -> nuovo HS
```

La FSM doveva vivere nell'env, non nella reward wrapper, ed essere esposta nell'observation space actor per rispettare il vincolo di deployability: usare solo angoli protesici di ginocchio/caviglia e online GRF in modalita detector.

## Soluzione

E stata aggiunta una FSM runtime `ProstheticPhaseFSM` dentro `Trajectory Generator/`, istanziata dall'env e resettata a ogni `env.reset()`.

Stati implementati:

```text
WAIT_HS = 0
STANCE_AFTER_HS = 1
SWING_AFTER_TO = 2
VALID_CYCLE_COMPLETED = 3
TIMEOUT = 4
INVALID_EVENT = 5
```

La FSM produce osservazioni actor, diagnostica in `info["phase_fsm"]` e termini reward consumati dalla reward wrapper. Gli eventi invalidi generano penalita/diagnostica ma non terminano l'episodio di default. I timeout hard stance/swing restano invece terminali tramite la logica reward wrapper gia prevista.

## Strategia

La reward non mantiene piu una seconda macchina stati. L'env calcola la FSM usando il detector online GRF lato left; la reward consuma solo il payload `info["phase_fsm"]`.

La progressione evento e codificata come:

```text
WAIT_HS: 0.00
dopo valid_HS: 0.25
dopo valid_TO: 0.50
sullo step che chiude HS -> TO -> HS: 1.00
```

La landing window premia il ritorno progressivo della GRF durante lo swing, solo nella finestra configurata:

```yaml
phase_landing_window_start_s: 0.55
phase_landing_window_end_s: 1.10
```

I valori di bootstrap sono conservativi:

- `phase_min_stance_duration_s = 0.05`: filtro anti-chattering per TO troppo precoce.
- `phase_min_swing_duration_s = 0.20`: filtro anti-HS spurio dopo TO.
- `phase_landing_window_start_s = 0.55`: vicino allo swing nominale di circa 0.60 s.
- `phase_landing_window_end_s = 1.10`: finestra larga, dopo soft timeout e prima dell'hard timeout.

## File modificati

- `Trajectory Generator/prosthetic_phase_fsm.py`
  - Nuova FSM runtime HS-TO-HS.
- `Trajectory Generator/osim_trj_cmc_like.py`
  - Integrazione FSM nell'env.
  - Reset per episodio.
  - Esposizione observation actor e `info["phase_fsm"]`.
  - Uso detector GRF come sorgente per la FSM.
- `Trajectory Generator/baseline_MLP/reward_function.py`
  - Reward progressione evento.
  - Reward landing window.
  - Penalita invalid event/contact validity.
  - Reward wrapper aggiornata per consumare `info["phase_fsm"]`.
- `Trajectory Generator/baseline_MLP/env_factory.py`
  - Propagazione campi FSM dalla reward config all'env config.
- `Trajectory Generator/baseline_MLP/training_config.py`
  - Parsing config per `deployable_minimal_observation`.
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
  - CLI/config per `deployable_minimal_observation`.
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - CLI/config per `deployable_minimal_observation`.
- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - Logging dei nuovi termini FSM/reward.
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - Sezione `4.1) Prosthetic phase FSM / landing window`.
  - Modalita observation deployable minimale abilitata.
- `validation/test_reward_function.py`
  - Unit test FSM pura.
  - Reward tests per progress, landing window e invalid event.
- `validation/validate_training_config.py`
  - Smoke/config test dei nuovi campi.
- `validation/test_phase_fsm_prescribed_env.py`
  - Test CMC-like con prescribed GRF come dinamica e detector online come sensore.

## Test e verifiche

Compilazione Python:

```bash
python3 -m py_compile \
  "Trajectory Generator/prosthetic_phase_fsm.py" \
  "Trajectory Generator/osim_trj_cmc_like.py" \
  "Trajectory Generator/baseline_MLP/reward_function.py" \
  "Trajectory Generator/baseline_MLP/env_factory.py" \
  "Trajectory Generator/baseline_MLP/training_config.py" \
  "Trajectory Generator/baseline_MLP/train_ppo_mlp.py" \
  "Trajectory Generator/baseline_MLP/rollout_eval.py" \
  "Trajectory Generator/baseline_MLP/tb_logging.py" \
  validation/test_reward_function.py \
  validation/validate_training_config.py \
  validation/test_phase_fsm_prescribed_env.py
```

Reward/unit/config:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
```

Esito:

```text
ALL REWARD TESTS PASSED (18)
```

Config smoke:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/validate_training_config.py
```

Esito:

```text
ALL SMOKE CHECKS PASSED
```

CMC-like prescribed fast smoke:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
```

Esito:

```text
[PASS] prescribed CMC-like FSM smoke
```

CMC-like prescribed full-step:

```bash
CMC_FULL_FSM_PRESCRIBED_TEST=1 \
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_phase_fsm_prescribed_env.py
```

Esito:

```text
[PASS] prescribed CMC-like FSM smoke
```

Durata circa 2 minuti e 20 secondi. Durante la run e comparso un warning di static optimization:

```text
QP did not converge. Using bounded least-squares fallback.
```

Il warning non ha fatto fallire il test. La FSM ha chiuso almeno un ciclo left `HS -> TO -> HS` tramite `env.step()` reale, con prescribed GRF per la dinamica e detector online come sensore.

## Stato

La FSM e validata per il perimetro env/reward/config/prescribed CMC-like.

Resta da validare il comportamento come scaffold di apprendimento RL, cioe se una policy ex-novo usa il nuovo segnale per produrre cicli `HS -> TO -> HS` affidabili invece di collassare ancora su `phase_timeout:swing`.
