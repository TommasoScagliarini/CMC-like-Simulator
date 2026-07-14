# Timeout GRF/fase: penalita post-clip e terminazione hard

## Problema

Nel rollout post-training ex-novo:

```text
Trajectory Generator/runs/rollout/MLP_ExNovo_rollout_06-29-2026_post_training_best
```

sono emerse due criticita collegate:

1. il ginocchio protesico resta bloccato vicino al limite di flessione del
   comando assoluto, circa `-1.5 rad` nei dati (`~1.5 rad` se letto come
   flessione positiva);
2. la caviglia protesica resta vicino a `0.1 rad`;
3. il lato protesico perde quasi completamente il contatto, ma l'episodio non
   termina e arriva al limite temporale.

Il sommario rollout indicava:

```text
terminated = False
truncated = True
left contact fraction = 0.018196
right contact fraction = 0.759448
```

Gli eventi online del lato protesico erano solo:

```text
left heel_strike = 1
left toe_off = 1
```

quindi non veniva chiuso un secondo ciclo `HS -> TO -> HS` e la fase protesica
rimaneva bloccata.

## Diagnosi

La cinematica salvata nel rollout mostra che il SEA sta seguendo il riferimento
servito dalla policy: il problema non e' principalmente nel low-level SEA, ma
nel riferimento/action prodotto dalla rete.

Statistiche principali osservate:

```text
pros_knee_angle:
  min  -1.517374
  mean -1.452603
  max  -0.225770
  last -1.499583

pros_ankle_angle:
  min  -0.064172
  mean  0.104903
  max   0.407290
  last  0.082154
```

Nel trace policy:

```text
knee raw action mean ~= -1.79
knee action clipped to -1 in ~= 96% of steps
ankle raw action converges near 0.12
```

Con la mappatura assoluta corrente:

```text
pros_knee_angle  action -1 -> -1.5 rad
pros_ankle_angle action  0.12 -> ~0.08 rad
```

il ginocchio resta quindi al bound per saturazione dell'azione, mentre la
caviglia resta vicino al valore prodotto dalla policy. Inoltre
`policy_action_clip_weight` e `morphology_weight` erano a zero, quindi non
esisteva una penalita efficace per scoraggiare questa soluzione.

## Timeout gia presenti

I valori temporali erano gia stati inseriti in configurazione:

```yaml
phase_period_nominal_s: 1.58
phase_period_soft_margin_s: 0.25
phase_period_hard_min_s: 0.90
phase_period_hard_max_s: 2.20
phase_stance_timeout_s: 1.45
phase_swing_timeout_s: 0.90
phase_timeout_scale_s: 0.20
```

La stima proveniva dai report del 2026-06-26:

- `phase_period_nominal_s = 1.58` fissato offline/model-based per AB06
  treadmill;
- hard range ciclo `[0.90, 2.20]`;
- detector `grf_detector_HS-TO` validato con errore circa `13 ms` su HS e
  `26-27 ms` su TO;
- scelta esplicita di non stimare il periodo nominale dalla policy corrente, per
  evitare una metrica autoreferenziale durante il training ex-novo.

Il problema era che questi timeout contribuivano solo alla
`phase_regularity_loss`: riducevano la reward interna, ma non erano una penalita
post-clip e non potevano terminare l'episodio.

## Soluzione

E' stato introdotto il meccanismo standard:

```text
vicino al bound:
  penalita morbida post-clip

troppo fuori dal bound:
  terminazione esplicita
```

Nuovi parametri:

```yaml
phase_timeout_penalty_weight: 0.10
phase_stance_hard_timeout_s: 2.20
phase_swing_hard_timeout_s: 1.30
```

La `phase_timeout_loss` viene ora sottratta dopo il clipping principale della
reward, insieme agli altri guardrail hard/near-hard. Se il tempo in stance o
swing supera il relativo hard timeout, il wrapper forza la terminazione e
scrive:

```text
end_reason = phase_timeout:stance
end_reason = phase_timeout:swing
```

Nel caso osservato, dopo il `left toe_off` a circa `13.10 s`:

```text
soft swing penalty starts near 14.00 s
hard swing termination fires near 14.40 s
```

quindi il rollout patologico non arriverebbe piu al limite di episodio senza
contatti protesici.

## Strategia

La modifica e' stata mantenuta dentro `Trajectory Generator/`, per rispettare il
perimetro della rete/policy/training.

Non e' stata modificata la semantica del plugin C++ SEA, ne' il comando
high-level del SEA. La diagnosi ha separato:

- problema action/reference: knee saturo al bound e ankle vicino al comando
  prodotto dalla policy;
- problema reward/guardrail: assenza di penalita post-clip e terminazione per
  missing contact protesico.

## File modificati

File aggiornati:

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `validation/test_reward_function.py`
- `validation/validate_training_config.py`

Nota: il worktree conteneva gia molte modifiche non correlate. Questo intervento
ha riguardato solo timeout GRF/fase, penalita post-clip, terminazione hard e
relative verifiche.

## Test e verifiche

Verifiche eseguite:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/test_reward_function.py
```

Risultato:

```text
11 tests passed
ALL REWARD TESTS PASSED (11)
```

Validazione config:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/validate_training_config.py
```

Risultato:

```text
All smoke checks passed
```

Compilazione Python:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python -m py_compile \
  "Trajectory Generator/baseline_MLP/reward_function.py" \
  "Trajectory Generator/baseline_MLP/tb_logging.py" \
  validation/test_reward_function.py \
  validation/validate_training_config.py
```

Risultato: nessun errore.

## Conclusione

La causa del ginocchio bloccato e' la saturazione dell'azione della policy sul
canale knee, non un errore evidente del SEA. La caviglia resta vicino a `0.1 rad`
perche' quello e' il riferimento assoluto prodotto dalla policy dopo la
mappatura action-to-reference.

Il problema piu urgente sulla GRF era che il timeout gia progettato non aveva
effetto hard. Ora il missing contact protesico genera una penalita post-clip e,
se persiste oltre il bound hard, termina l'episodio.

## TODO

- Rilanciare un rollout con la nuova guardia per confermare che il caso senza
  secondo HS protesico termini con `phase_timeout:swing`.
- Valutare se riattivare un peso positivo su `policy_action_clip_weight`, dato
  che il canale knee resta clippato per circa il 96% del rollout analizzato.
- Solo dopo la verifica della fase/contatto, rivalutare `morphology_weight > 0`
  come guardrail cinematico phase-dependent.
