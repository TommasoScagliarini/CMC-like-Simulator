# Confronto asym100 vs config attuale con contact-validity GRF

## Problema

Prima di lanciare un nuovo training e' fondamentale stabilire se la configurazione
attuale sia confrontabile con `asym100`. Il rischio era attribuire eventuali
differenze di comportamento alla nuova penalita' sul momento GRF alla caviglia,
quando in realta' potevano esserci altre modifiche nascoste nel training setup.

Il riferimento usato per il confronto e':

```text
Trajectory Generator/runs/training/MLP_imitation_training_06-17-2026_asym_actor_critic_100/training_cfg.resolved.yaml
```

## Verifica Configurazione

Il confronto ha mostrato che le sezioni principali coincidono semanticamente:

- `model`;
- `ppo`;
- `parallelism`;
- `simulation`;
- `grf`;
- `supervision`;
- `logging`.

Anche le chiavi non scritte esplicitamente nel `training_cfg.yaml` attuale ma
presenti nel resolved storico coincidono via default:

```text
action_mode = absolute
max_delta_rad = 0.35
```

I pesi reward gia' presenti in `asym100` coincidono con quelli attuali.

## Differenze Effettive

La differenza intenzionale non e' una singola penalty isolata, ma un pacchetto di
validita' del contatto GRF online:

```text
asym100
+
GRF contact-validity / feasibility package
```

Il pacchetto comprende:

```yaml
grf_penetration_penalty_threshold_m: 0.012
grf_penetration_termination_m: 0.017
grf_ankle_moment_flip_weight: 0.25
grf_ankle_moment_flip_tau_tol_nm: 8.0
grf_ankle_moment_flip_force_threshold_n: 50.0
```

La prima parte restringe la guardia sulla penetrazione del contatto online:

```text
soft threshold: 12 mm
termination:    17 mm
```

La seconda parte penalizza il momento GRF alla caviglia quando la contact patch
online genera un momento con segno fisicamente sospetto:

```text
tau_GRF_about_ankle = ankle_axis · (M_ground - ankle_center x F_GRF)
excess = max(0, -tau_GRF_about_ankle - 8 Nm)
loss = excess^2 / 8^2
```

Il termine si attiva solo con contatto sufficientemente caricato:

```text
normal_force >= 50 N
```

## Differenze Non Operative

Nel config attuale compaiono anche tre chiavi non presenti nel resolved storico
di `asym100`, ma sono tutte a peso zero:

```yaml
sea_tau_spring_effort_weight: 0.0
sea_tau_spring_rate_weight: 0.0
policy_action_clip_weight: 0.0
```

Queste non cambiano la reward effettiva.

## Interpretazione Corretta Del Confronto

Il confronto corretto non va formulato cosi':

```text
asym100 vs asym100 + solo tau_GRF penalty
```

ma cosi':

```text
asym100 vs asym100 + GRF contact-validity package
```

Questa formulazione e' piu' coerente con la diagnosi fisica fatta finora: il burst
positivo di `tau_spring` non nasce da un problema isolato di reward torque-aware,
ma da una contact patch online che puo' diventare non valida attraverso
penetrazione, ribaltamento del COP e momento GRF alla caviglia con segno sospetto.

## File Coinvolti

- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - mantiene il setup `asym100`;
  - aggiunge i parametri del pacchetto GRF contact-validity.

- `Trajectory Generator/osim_trj_cmc_like.py`
  - espone i threshold di penetrazione aggiornati;
  - calcola e logga il momento GRF alla caviglia;
  - produce `grf_ankle_moment_flip_loss`.

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - pesa il nuovo termine nella reward centrale.

- `Trajectory Generator/baseline_MLP/env_factory.py`
  - passa `tau_tol` e `force_threshold` dal config reward all'env.

- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - include i diagnostici `grf_ankle_moment_flip_*` nel logging.

## Verifiche Eseguite

Sono state eseguite verifiche locali:

- confronto del `training_cfg.yaml` attuale con il resolved storico di `asym100`;
- verifica delle differenze reward operative e non operative;
- verifica che `training_cfg.yaml` risolva:

```text
imitation_ankle_velocity_weight = 0.04
grf_ankle_moment_flip_weight = 0.25
grf_ankle_moment_flip_tau_tol_nm = 8.0
grf_ankle_moment_flip_force_threshold_n = 50.0
```

- `py_compile` dei file Python toccati;
- test numerico della reward:

```text
grf_ankle_moment_flip_loss = 1.0
grf_ankle_moment_flip_weight = 0.25
reward base = 1.0
reward finale = 0.75
```

## Raccomandazione

Usare il nuovo training come test A/B del pacchetto:

```text
asym100
vs
asym100 + GRF contact-validity package
```

Nel reporting dei risultati bisogna evitare di dire che l'unica differenza e' la
penalita' `tau_GRF_about_ankle`. La frase corretta e':

```text
The learning/control setup is identical to asym100; the only intentional changes
are contact-validity terms associated with the online GRF model.
```

Metriche da controllare nel rollout:

- `tau_spring` alla caviglia protesica;
- `grf_ankle_moment_flip_loss`;
- `grf_ankle_moment_flip_cop_z_minus_ankle_z_m`;
- `grf_penetration_m`;
- tracking served-reference;
- reserve root, soprattutto `pelvis_ty`;
- eventuale aumento di fallimenti per `grf_penetration`.
