# Reward guard sul momento GRF alla caviglia

## Problema

Nei rollout con online GRF applicata sul lato protesico e' emerso un burst positivo
di `tau_spring` alla caviglia in `asym100`. L'analisi precedente ha mostrato che il
meccanismo piu' coerente non e' una semplice magnitudine verticale della GRF, ma il
segno del momento generato dalla contact patch online rispetto all'asse della
caviglia protesica.

Il caso patologico e':

```text
tau_GRF_about_ankle < 0
```

con modulo abbastanza grande da indicare che la GRF online sta generando un momento
alla caviglia con segno opposto rispetto al comportamento osservato nei casi sani o
CMC-like. Questo e' compatibile con il ribaltamento del braccio COP-caviglia
osservato durante il burst.

## Soluzione Implementata

E' stato aggiunto un nuovo termine diagnostico/reward:

```text
tau_GRF_about_ankle = ankle_axis · (M_ground - ankle_center x F_GRF)
excess = max(0, -tau_GRF_about_ankle - tau_tol)
raw_loss_nm2 = excess^2
loss = raw_loss_nm2 / tau_tol^2
```

con:

```yaml
tau_tol = 8.0 Nm
force_threshold = 50.0 N
weight = 0.25
```

Il termine si attiva solo quando:

- il lato `left` e' tra gli `online_grf_applied_sides`;
- la GRF online e' disponibile;
- il carico verticale/normal force supera `50 N`;
- `tau_GRF_about_ankle` e' piu' negativo di `-8 Nm`.

Il raw loss in `Nm^2` viene loggato per diagnostica, mentre la reward usa la
versione normalizzata. Questo rende il peso leggibile e stabile: con `tau_tol=8`,
un eccesso di `8 Nm` produce `loss = 1`, quindi una penalita' pari a `0.25`.

## Strategia

La penalita' non usa direttamente una morfologia di coppia prescritta e non impone
un target di torque. E' una guardia di validita' fisica sul contatto online: se la
contact patch produce un momento alla caviglia con segno sospetto e abbastanza
grande, la policy viene penalizzata.

Questo mantiene separati:

- trajectory generator;
- cascade/outer controller;
- motor driver SEA;
- reward biomeccanica.

Il termine non cerca di rendere la reward torque-aware in senso stretto. Cerca
invece di impedire che una soluzione apparentemente buona per tracking sfrutti un
contatto online fisicamente invalido.

## File Modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
  - aggiunti `grf_ankle_moment_flip_tau_tol_nm` e
    `grf_ankle_moment_flip_force_threshold_n` in `CMCEnvConfig`;
  - calcolo di centro e asse della caviglia protesica in ground;
  - calcolo di `tau_GRF_about_ankle`;
  - esposizione nei `reward_terms` di:
    - `grf_ankle_moment_flip_available`;
    - `grf_ankle_moment_flip_active`;
    - `grf_ankle_moment_flip_tau_nm`;
    - `grf_ankle_moment_flip_tau_tol_nm`;
    - `grf_ankle_moment_flip_force_n`;
    - `grf_ankle_moment_flip_force_threshold_n`;
    - `grf_ankle_moment_flip_cop_z_minus_ankle_z_m`;
    - `grf_ankle_moment_flip_excess_nm`;
    - `grf_ankle_moment_flip_raw_loss_nm2`;
    - `grf_ankle_moment_flip_loss`.

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - aggiunto `GRF_ANKLE_MOMENT_FLIP_LOSS`;
  - aggiunti in `RewardConfig`:
    - `grf_ankle_moment_flip_weight`;
    - `grf_ankle_moment_flip_tau_tol_nm`;
    - `grf_ankle_moment_flip_force_threshold_n`;
  - sottrazione del termine dopo il clip, come safety/contact feasibility.

- `Trajectory Generator/baseline_MLP/env_factory.py`
  - pass-through di `tau_tol` e `force_threshold` dal blocco reward all'env.

- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - logging TensorBoard dei diagnostici `grf_ankle_moment_flip_*`.

- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
  - setup allineato ad `asym100` per la reward imitation;
  - `imitation_ankle_velocity_weight: 0.04`;
  - aggiunto:

```yaml
grf_ankle_moment_flip_weight: 0.25
grf_ankle_moment_flip_tau_tol_nm: 8.0
grf_ankle_moment_flip_force_threshold_n: 50.0
```

## Peso Proposto

Il peso iniziale scelto e':

```yaml
grf_ankle_moment_flip_weight: 0.25
```

Motivo:

- e' abbastanza forte da colpire il burst positivo di `asym100`;
- non domina completamente tracking e imitation;
- resta nullo per i transitori prescritti inferiori a `8 Nm`;
- e' interpretabile perche' lavora su una loss normalizzata da `tau_tol^2`.

Un eventuale sweep ragionevole e':

```text
0.10 -> debole, diagnostico-regolarizzante
0.25 -> default consigliato
0.50 -> forte, da usare solo se il burst persiste
```

## Test e Verifiche

Verifiche eseguite:

- compilazione Python dei file modificati:

```text
py_compile osim_trj_cmc_like.py reward_function.py env_factory.py tb_logging.py
```

- test numerico della reward:

```text
grf_ankle_moment_flip_loss = 1.0
grf_ankle_moment_flip_weight = 0.25
reward base = 1.0
reward finale = 0.75
```

- verifica caricamento YAML:

```text
imitation_ankle_velocity_weight = 0.04
grf_ankle_moment_flip_weight = 0.25
grf_ankle_moment_flip_tau_tol_nm = 8.0
```

- pulizia dei `.pyc` generati dai check.

## Limiti

Il termine e' implementato sul lato protesico sinistro (`left`) per coerenza con
il setup attuale AB06/SEASEA. Se in futuro la protesi venisse spostata a destra o
si generalizzasse il modello, andra' generalizzata anche la scelta di ankle center
e ankle axis.

Il termine non sostituisce la diagnosi su penetrazione/COP. Va letto insieme a:

- `grf_penetration_m`;
- `grf_ankle_moment_flip_cop_z_minus_ankle_z_m`;
- `tau_spring` SEA;
- tracking `served-reference`;
- eventuali reserve root.

## Raccomandazione

Lanciare un training breve con il nuovo `training_cfg.yaml` e controllare nei log:

- frequenza di attivazione `grf_ankle_moment_flip_active`;
- valore medio/picco di `grf_ankle_moment_flip_loss`;
- scomparsa o riduzione del burst positivo di `tau_spring`;
- mantenimento del tracking served-reference;
- assenza di aumento anomalo di penetrazione o reserve.

Se la policy evita il burst senza degradare tracking e contatto, il peso `0.25`
puo' diventare il default operativo. Se il burst rimane, provare `0.50`; se invece
la policy diventa troppo conservativa o perde tracking, scendere a `0.10`.
