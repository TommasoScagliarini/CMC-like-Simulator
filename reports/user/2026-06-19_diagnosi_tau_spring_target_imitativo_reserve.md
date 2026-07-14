# Diagnosi tau_spring, target imitativo e reserve - 2026-06-19

## Problema

Dopo il confronto tra `asym100`, `asym200` e il riferimento del 2026-05-23,
la rete MLP sembrava ottenere un buon tracking cinematico, ma la coppia SEA
dell'ankle (`tau_spring`) aveva una forma non biomeccanicamente convincente.

Il caso piu' evidente era l'ankle:

- nei rollout MLP `asym100/asym200`, `SEA_Ankle_tau_spring` resta quasi vicino
  a zero con spike locali;
- nel run `05_23`, l'ankle mostra una grande coppia negativa ciclica, coerente
  con il comportamento protesico desiderato.

Questo ha sollevato il dubbio che la rete stesse "barando": buon tracking
cinematico, ma strategia dinamica non plausibile.

## Correzione eseguita

E' stato corretto il rollout evaluator affinche' il `RewardShapingWrapper`
veda l'azione grezza della policy prima del clipping.

Prima il rollout faceva:

```python
obs, reward, terminated, truncated, info = env.step(action)
```

dove `action` era gia' stata clippata per la traccia/summary. In questo modo
`policy_action_clip_loss` risultava cieco.

Ora il rollout fa:

```python
obs, reward, terminated, truncated, info = env.step(raw_action)
```

Il wrapper interno `FlattenClipAction` continua a proteggere il simulatore, ma
la reward puo' penalizzare correttamente la differenza raw-vs-applied.

## Strategia di analisi

La diagnosi e' stata separata in tre livelli:

1. Verificare che il bug del `policy_action_clip_loss` fosse reale e correggerlo.
2. Rigenerare rollout e plot per `asym200` con reward corretta.
3. Capire se la brutta forma di `tau_spring` fosse causata da:
   - errore SEA / outer controller;
   - traiettoria MLP sbagliata rispetto al proprio target;
   - target cinematico imitativo non equivalente al riferimento protesico usato
     nel run del 2026-05-23;
   - reserve actuator che falsano direttamente la coppia protesica.

## Risultati principali

### Clip policy

Il rollout `asym200` rigenerato ha confermato che la penalita' e' ora attiva:

```text
policy_action_clip_loss mean    0.00231764
policy_action_clip_loss max     0.0429788
policy_action_clip nonzero      75 / 501 step
action_clipped_fraction         0.07485
episode_return                  361.0965
```

La valutazione precedente era quindi leggermente ottimistica, ma il giudizio
qualitativo non cambia: `asym200` non supera `asym100` come baseline operativa.

### Tau spring e tau_ff_cmd

Il confronto tra `tau_spring` e `tau_ff_cmd` mostra che il SEA segue il momento
richiesto dalla dinamica corrente. Il problema non sembra essere un tracking
interno SEA sbagliato.

Per `asym100`:

```text
SEA_Ankle_tau_ff_cmd       RMS ~= 5.72 Nm
SEA_Ankle_tau_spring       RMS ~= 5.74 Nm
SEA_Ankle_tau_error        RMS ~= 0.44 Nm
```

Per il run `05_23`:

```text
SEA_Ankle_tau_ff_cmd       RMS ~= 48.06 Nm
SEA_Ankle_tau_spring       RMS ~= 48.07 Nm
SEA_Ankle_tau_error        RMS ~= 0.89 Nm
```

Conclusione: la coppia "brutta" del rollout MLP non nasce dal fatto che il SEA
non riesce a generare la coppia richiesta. Il momento richiesto dalla dinamica
per quella traiettoria e quel contesto e' gia' piccolo.

### Target cinematico imitativo

La differenza di range ankle esiste gia' nel riferimento cinematico, prima
della rete.

Nel file `models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`,
finestra `12.99-17.99 s`:

```text
ankle_angle_r:
  [4.005, 27.549] deg  ~= [0.070, 0.481] rad

pros_ankle_angle:
  [-7.949, 23.037] deg ~= [-0.139, 0.402] rad
```

Il target imitativo corrente usa il mapping:

```python
"pros_ankle_angle": "ankle_angle_r"
```

Quindi il target della rete per la protesi ankle deriva dalla caviglia destra
sana (`ankle_angle_r`), non dalla coordinata protesica/sinistra
`pros_ankle_angle` usata nel run `05_23`.

Il target ankle effettivamente visto nel rollout `asym100` e':

```text
target ankle asym100:
  [0.087, 0.441] rad
```

mentre il run `05_23` aveva:

```text
pros_ankle_angle 05_23:
  [-0.177, 0.431] rad
```

Conclusione: la rete non sta necessariamente sbagliando la generazione della
traiettoria rispetto al target attuale. Il target attuale e' pero'
sostanzialmente diverso dalla cinematica protesica che produce la coppia ankle
desiderata.

### Reserve

Le reserve locali sui due giunti protesici sono zero:

```text
pros_knee_angle_reserve_torque  = 0
pros_ankle_angle_reserve_torque = 0
```

Quindi si puo' escludere che le reserve locali di ginocchio/caviglia stiano
falsando direttamente la coppia erogata dai SEA.

Le reserve globali pero' sono molto alte in `asym100`, soprattutto sul bacino:

```text
tau_reserve_norm asym100:
  mean 333.2
  RMS  402.2
  max  843.2

pelvis_ty_reserve_torque:
  RMS     382.6
  max_abs 836.8
```

Per confronto, nel run corrente `results/sim_output_*` associato al riferimento
del 2026-05-23:

```text
tau_reserve_norm:
  mean 104.1
  RMS  114.4
  max  530.7
```

Conclusione: le reserve non falsano direttamente `tau_spring` dei SEA, ma
rendono la dinamica globale molto assistita. Penalizzarle puo' essere utile,
ma solo dopo aver corretto/verificato il target cinematico, altrimenti si rischia
di penalizzare un sintomo di un target incompatibile.

## Decisioni operative

- Da ora in poi `Trajectory Generator/baseline_MLP/training_cfg.yaml` e' la
  configurazione di riferimento. La `v4_imitation` non va piu' considerata per
  le decisioni operative.
- Non conviene aumentare ulteriormente i pesi imitation prima di risolvere il
  problema del target ankle.
- Non conviene introdurre un profilo di coppia `tau_spring` di riferimento:
  sarebbe troppo specifico e rischierebbe di "far tornare il risultato".
- La reward sui reserve va trattata con cautela: utile come vincolo di
  fattibilita' globale, ma non come prima correzione se il target cinematico e'
  incoerente.
- La prossima diagnosi deve confrontare direttamente:
  `ankle_angle_r`, `pros_ankle_angle`, target phase-based imitation e
  trajectory served dalla policy.

## File modificati

- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - fix del passaggio da azione clippata ad azione grezza in `env.step()`;
  - il clipping manuale resta usato solo per trace/summary.
- `reports/user/2026-06-19_diagnosi_tau_spring_target_imitativo_reserve.md`
  - questo report.

## Output rigenerati / consultati

- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-18-2026_asym_actor_critic_resume_200/`
- `plot/06_19_2026_1_imitTraining_asymActCrit_200/`
- `plot/06_18_2026_2_imitTraining_asymActCrit_100/`
- `plot/05_23_2026_2/`
- `results/sim_output_kinematics.sto`
- `results/sim_output_sea_diagnostics.sto`
- `results/sim_output_recruitment.sto`
- `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100/sim_outputs/`

## Verifiche eseguite

- `py_compile` su:
  - `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - `Trajectory Generator/baseline_MLP/reward_function.py`
  - `Trajectory Generator/baseline_MLP/env_factory.py`
- Test sintetico con dummy env:
  - base env riceve azione clippata;
  - `RewardShapingWrapper` vede azione raw;
  - `policy_action_clip_loss`, `policy_action_clip_fraction` e
    `policy_action_clip_abs_max` diventano non nulli quando l'azione esce dai
    bound.
- `git diff --check` sul fix.
- Rollout `asym200` rigenerato con `--record-outputs`.
- Plot MLP rigenerati: nessun missing channel.
- Confronto numerico:
  - `tau_spring` vs `tau_ff_cmd`;
  - range cinematici `ankle_angle_r` vs `pros_ankle_angle`;
  - reserve norm e componenti principali.

## TODO

- Verificare se il target imitation deve usare `pros_ankle_angle` / lato
  protesico/sinistro invece di `ankle_angle_r`, oppure se serve una conversione
  di offset/range tra coordinate.
- Generare un plot diagnostico dedicato con:
  - `ankle_angle_r`;
  - `pros_ankle_angle`;
  - target phase-based imitation;
  - served reference MLP;
  - actual prosthetic ankle.
- Valutare una metrica agnostica di fattibilita' globale basata su reserve norm,
  ma solo dopo la verifica del target cinematico.
- Se si introduce una reserve penalty, iniziare con peso basso e monitorare se
  entra in conflitto con l'imitation target.
