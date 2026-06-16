# 2026-06-16 - Baseline MLP: layout run e comandi snelli

## Problema

La baseline MLP aveva ancora una gestione dei risultati troppo manuale:

- training e rollout potevano finire in cartelle diverse o legacy sotto `runs`;
- il training richiedeva spesso `--config` e `--output-dir`;
- il rollout richiedeva sempre `--checkpoint`, `--output-dir` e `--record-outputs`;
- la cartella `Trajectory Generator/runs` conteneva risultati storici non piu'
  utili accanto all'ultimo run V4 con observation space allargato.

L'obiettivo era rendere l'uso quotidiano simile al simulatore CMC-like root:
un comando essenziale, con default sensati e naming automatico.

## Soluzione

E' stata introdotta una struttura stabile dei risultati:

- training in `Trajectory Generator/runs/training`;
- rollout e oracle diagnostici in `Trajectory Generator/runs/rollout`;
- vecchi risultati diretti sotto `Trajectory Generator/runs` rimossi, mantenendo
  l'ultimo training/rollout V4 obs-target.

Il training ora parte con:

```powershell
python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py"
```

Default principali:

- config: `Trajectory Generator/baseline_MLP/training_cfg.yaml`;
- contenuto di `training_cfg.yaml` allineato a `training_cfg.v4_imitation.yaml`;
- output root: `Trajectory Generator/runs/training`;
- nome run: `MLP_[strategy]_training_[MM-DD-YYYY]`;
- `strategy` ricavata da `reward.reward_mode` (`imitation` oppure `ExNovo`);
- `--name _suffix` aggiunge un suffisso custom;
- collisioni gestite con `_02`, `_03`, ecc.

Il rollout ora parte con:

```powershell
python "Trajectory Generator\baseline_MLP\rollout_eval.py"
```

Default principali:

- se manca `--checkpoint`, viene scelto l'ultimo training valido in
  `Trajectory Generator/runs/training`;
- checkpoint default: `rl_module_best`;
- output root: `Trajectory Generator/runs/rollout`;
- nome rollout: nome del training con `training` sostituito da `rollout`;
- per run legacy senza `training` nel nome, fallback `<training_run>_rollout`;
- `--name _suffix` e collisioni `_02`, `_03` supportati anche nel rollout;
- `record_outputs` ora e' attivo di default;
- `--no-record-outputs` permette rollout leggeri.

## Strategia

La modifica e' stata fatta mantenendo compatibilita' con i flussi esistenti:

- `--config` nel training continua a sovrascrivere il default;
- `--output-dir` nel training e nel rollout mantiene precedenza totale sul
  naming automatico;
- `--checkpoint` nel rollout mantiene precedenza sul checkpoint automatico;
- i path legacy `runs\nome_run\...` vengono cercati anche sotto
  `runs/training`;
- il rollout continua ad auto-caricare `training_cfg.resolved.yaml` dal run del
  checkpoint, cosi env, rete, reward e action mode restano allineati al training.

Non e' stata modificata la semantica del plugin C++ SEA.

## File modificati

- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
  - default config/output;
  - naming automatico training;
  - `--name`;
  - collision handling.
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - allineata alla configurazione V4 imitation obs-target.
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - checkpoint default dall'ultimo training valido;
  - naming automatico rollout;
  - `--name`;
  - `--record-outputs` default on e `--no-record-outputs`.
- `Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py`
  - output sotto `runs/rollout`.
- `Trajectory Generator/baseline_MLP/commands.txt`
  - comandi raccomandati aggiornati per training e rollout no-flag.
- `Trajectory Generator/baseline_MLP/README.md`
  - documentazione aggiornata su layout, naming e comandi rapidi.
- `Trajectory Generator/baseline_MLP/env_factory.py`
  - gia' coinvolto nel lavoro V4 obs-target.
- `Trajectory Generator/osim_trj_cmc_like.py`
  - gia' coinvolto nel lavoro V4 obs-target.

## Risultati e stato delle cartelle

Stato previsto dopo la riorganizzazione:

- training mantenuto:
  `Trajectory Generator/runs/training/baseline_mlp_imit_v4_c2_4hz_obs_target`;
- rollout mantenuto:
  `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_rollout`;
- `Trajectory Generator/runs` contiene solo le sottocartelle `training` e
  `rollout`.

E' stata anche ispezionata la cartella accidentale
`Trajectory Generator/Trajectory Generator`, che contieneva vecchi artifact di
rollout/oracle generati da path legacy. Non e' stata rimossa in questa fase.

## Test e verifiche

Verifiche eseguite:

- `python -m py_compile` su:
  - `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`;
  - `Trajectory Generator/baseline_MLP/rollout_eval.py`;
  - `Trajectory Generator/baseline_MLP/training_config.py`.
- Test dei resolver training senza lanciare training:
  - default imitation;
  - default ex_novo;
  - collisioni `_02`;
  - `--name`;
  - `--output-dir` manuale.
- Test dei resolver rollout senza lanciare OpenSim/RLlib:
  - no args -> ultimo training valido e `rl_module_best`;
  - nome rollout derivato dal training;
  - suffisso `--name`;
  - `--no-record-outputs`;
  - checkpoint/output manuali;
  - fallback legacy `<training_run>_rollout`.
- `git diff --check` passato.

Non sono stati lanciati un training completo o un rollout OpenSim completo dopo
queste modifiche; la validazione e' stata limitata a compile e resolver.
