# Daily Report - 2026-07-07

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-07-07_target_slew_limiter_reward_exnovo.md`

## Problema

La failure analysis del rollout H2 ha mostrato che `action_mode=absolute`
poteva chiedere al primo step variazioni di circa `-0.22/-0.19 rad` su
knee/ankle. Il reference governor partiva comunque verso un target lontano e
la penetrazione raggiungeva `17.923 mm` in dieci step.

## Soluzione

E' stato introdotto un limiter target-to-target applicato a ogni knot:

```text
knee  = 2.5 rad/s -> 0.025 rad per step a 100 Hz
ankle = 2.0 rad/s -> 0.020 rad per step a 100 Hz
```

Il limiter preserva le azioni assolute, agisce prima della dinamica e registra
delta raw/served, frazione limitata ed excess loss.

## Risultati

Con la stessa sequenza H2:

```text
prima: step 10, penetration 17.923 mm, terminated
dopo : step 10, penetration 0.114 mm, non terminated
```

Lo smoke CLI da 20 step ha prodotto return `+2.489`, nessun terminale e nessun
clipping. Il failure immediato e' quindi mitigato, ma non e' ancora dimostrato
un TO protesico.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`

## Test e verifiche

- test mirato in memoria: PASS;
- smoke rollout CLI 20 step: PASS;
- `py_compile`: PASS;
- config smoke: PASS;
- reward tests: `27` PASS;
- `git diff --check`: PASS.

## TODO aperti e propagati

- [x] Isolare una causa del terminale precoce: salto del target assoluto.
- [ ] Eseguire H1 10 iterazioni con limiter.
- [ ] Eseguire H2 registrato e richiedere almeno `HS -> TO` sinistro.
- [ ] Verificare graficamente target raw/served, GRF e penetrazione.
- [ ] Mantenere bloccati i training lunghi finche' H2 non passa.

