# Validazione inerzia effettiva knee/ankle - 2026-05-19

## Problema

Il redesign cascade zeta07 del 2026-05-18 aveva usato una stima di
`J_joint` ricavata dai gain del velocity loop, assumendo un plant scalare:

```text
qdot_dot = tau / J_joint
```

Le stime usate nel back-calc erano:

```text
J_knee  = 0.316 kg*m^2
J_ankle = 0.0193 kg*m^2
```

Dopo il fallimento operativo dello zeta07 sull'ankle, era rimasto aperto il
TODO di validare l'inerzia effettiva vista dal modello OpenSim.

## Strategia

E' stato aggiunto un validatore locale basato sulla matrice di massa OpenSim,
campionata lungo la cinematica IK del setup AB06 PI:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
time window: 11.99 -> 21.0 s
samples: 181
```

Per ogni campione viene costruita la matrice di massa `M` in ordine mobility
Simbody e vengono calcolate tre stime:

- `J_free_all = 1 / (M^-1)_ii`: risposta a una coppia unitaria con tutti i DOF
  del modello liberi;
- `J_locked = M_ii`: inerzia con gli altri DOF bloccati a accelerazione zero;
- `J_free_pros_pair`: risposta con liberi solo i due DOF protesici.

## Soluzione

Nuovo script:

```text
validation/effective_joint_inertia.py
```

Comando eseguito:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python validation/effective_joint_inertia.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --samples 181 \
  --out-dir results/_effective_joint_inertia_20260519
```

Artefatti:

```text
results/_effective_joint_inertia_20260519/summary.md
results/_effective_joint_inertia_20260519/summary.json
results/_effective_joint_inertia_20260519/samples.csv
```

## Risultati

| coord | backcalc | J_free_all median | ratio | J_locked median | ratio | J_free_pros_pair median | ratio |
|---|---:|---:|---:|---:|---:|---:|---:|
| pros_knee_angle | 0.316 | 0.159738 | 0.505 | 0.493791 | 1.56 | 0.444954 | 1.41 |
| pros_ankle_angle | 0.0193 | 0.00989248 | 0.513 | 0.0110981 | 0.575 | 0.0100005 | 0.518 |

Percentili principali:

```text
knee J_free_all  p05=0.1557, median=0.1597, p95=0.2173 kg*m^2
knee J_locked    p05=0.4812, median=0.4938, p95=0.5042 kg*m^2
knee J_pair      p05=0.4310, median=0.4450, p95=0.4552 kg*m^2

ankle J_free_all p05=0.00934, median=0.00989, p95=0.01037 kg*m^2
ankle J_locked   median=0.01110 kg*m^2
ankle J_pair     p05=0.00949, median=0.01000, p95=0.01050 kg*m^2
```

## Interpretazione

Il back-calc precedente sovrastima circa 2x l'inerzia libera per entrambi i
DOF:

```text
knee  J_free_all / J_backcalc  = 0.505
ankle J_free_all / J_backcalc  = 0.513
```

Per l'ankle le tre stime sono vicine: `~0.010-0.011 kg*m^2`. Quindi il valore
`0.0193` usato nel redesign zeta07 era troppo alto per la risposta libera e
anche per il caso locked.

Per il knee la risposta dipende molto dal modello di vincolo:

- `J_free_all` e' circa `0.160 kg*m^2`;
- `J_free_pros_pair` e' circa `0.445 kg*m^2`;
- `J_locked` e' circa `0.494 kg*m^2`.

Questo significa che il knee e' molto piu' sensibile all'accoppiamento con gli
altri DOF. Usare una sola inerzia scalare nel cascade e' una forte
approssimazione; per il knee va scelto esplicitamente se progettare sul plant
libero o sul plant quasi-bloccato dal controllo degli altri DOF.

## File modificati

```text
validation/effective_joint_inertia.py
reports/user/2026-05-19_validazione_inerzia_effettiva_knee_ankle.md
```

## Test e verifiche

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile validation/effective_joint_inertia.py
```

Run completa validazione:

```text
status: complete
samples: 181
output: results/_effective_joint_inertia_20260519
```

Smoke run post-refactor:

```text
samples: 5
status: complete
cartella temporanea rimossa
```

## TODO

- Aggiornare il design cascade usando una scelta esplicita di `J_eff`:
  - ankle: usare `J_eff ~= 0.010 kg*m^2`;
  - knee: decidere tra plant libero (`~0.160`) e plant quasi-bloccato
    (`~0.445-0.494`) in base alla dinamica realmente vista dal loop.
- Se si riprende zeta07, rifare il back-calc dei gain velocity PI con le nuove
  inerzie e poi validare su run breve prima del full run.
