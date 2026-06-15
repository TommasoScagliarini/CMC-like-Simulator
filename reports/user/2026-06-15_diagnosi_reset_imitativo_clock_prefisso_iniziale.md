# Diagnosi reset imitativo: clock retro-estrapolato e prefisso iniziale infeasible

**Data:** 2026-06-15

## Problema

Nel rollout del training imitativo V4 la gamba protesica mostrava un grande
swing del ginocchio all'inizio dell'episodio. Era necessario determinare perche
il reset risultasse dinamicamente incoerente partendo dal tempo di setup
`11.99 s`, mentre appariva molto piu regolare iniziando da `12.99 s`.

L'analisi ha distinto tre possibili cause:

- inizializzazione incoerente degli stati SEA;
- errore nella costruzione del target imitativo phase-based;
- incompatibilita tra il target protesico e il resto della posa prescritta.

## Causa individuata

Il primo heel strike destro valido rilevato nel file GRF avviene a:

```text
13.16376284 s
```

Il tempo iniziale originale `11.99 s` precede quindi il primo heel strike destro
di circa `1.174 s`.

In questa regione `GaitPhaseClock` non possiede un ciclo precedente reale e
retro-estrapola la fase usando il primo ciclo futuro:

```text
13.16376284 -> 14.77986941 s
periodo = 1.61610657 s
```

Il clock rimane matematicamente continuo, ma la fase retro-estrapolata non e
necessariamente compatibile con il prefisso iniziale del dataset.

Il target imitativo non usa direttamente la posa destra dello stesso istante.
Viene costruito mediando cinque cicli completi della gamba sana e valutando il
template periodico con shift fissi:

```text
knee shift  = 0.465 cicli
ankle shift = 0.452 cicli
```

Con `imitation_initialize_to_target=true`, il reset sostituisce solamente
angolo e velocita di ginocchio e caviglia protesici. Bacino, anca sinistra,
gamba destra e GRF rimangono associati alla posa prescribed dello stesso tempo.
Non viene eseguito un assembly o una proiezione su vincoli di contatto.

## Confronto quantitativo

### Reset a `11.99 s`

Il dato prescribed rappresenta ancora un doppio appoggio:

```text
GRF sinistra prescribed = 407.5 N
GRF destra prescribed   = 354.3 N
```

Il target imitativo del ginocchio e incompatibile con questa configurazione:

| Grandezza ginocchio | Prescribed | Target imitativo | Differenza |
|---|---:|---:|---:|
| posizione | `-0.1388 rad` | `-0.9335 rad` | `-0.7947 rad` (`-45.5 deg`) |
| velocita | `-0.0068 rad/s` | `3.2787 rad/s` | `3.2856 rad/s` |

Dopo il target-init:

```text
GRF online sinistra ~= 0 N
tau reserve norm     = 428.9 Nm
muscle share         = 0.067
```

Il ginocchio viene quindi posto in una configurazione da swing mentre il resto
dello stato richiede che il piede sinistro sia in appoggio. Lo stato e
numericamente accettato, ma dinamicamente infeasible.

Con reset prescribed allo stesso istante:

```text
GRF online sinistra = 444.4 N
tau reserve norm    = 71.3 Nm
muscle share        = 0.341
```

### Reset a `12.99 s`

A `12.99 s` il lato sinistro e in appoggio e il destro e in swing:

```text
GRF sinistra prescribed = 743.2 N
GRF destra prescribed   = 0 N
```

Il target periodico risulta casualmente vicino alla posa prescribed:

| Coordinata | Differenza target-prescribed |
|---|---:|
| ginocchio | `0.0240 rad` (`1.37 deg`) |
| caviglia | `0.0384 rad` (`2.20 deg`) |

Anche forzando `imitation_initialize_to_target=true`, il contatto viene
conservato:

```text
GRF online sinistra = 770.8 N
tau reserve norm    = 122.8 Nm
muscle share        = 0.390
```

Questo conferma che la differenza fra i due istanti non dipende da un errore di
inizializzazione dei SEA. Gli stati articolari, `motor_angle` e `motor_speed`
risultano coerentemente allineati al reset.

## Effetto sul training

Il prefisso precedente al primo heel strike puo influenzare anche il training,
non soltanto l'inizializzazione:

- la policy osserva una gait phase retro-estrapolata;
- la reward imitativa usa un target periodico non compatibile con il prefisso;
- la reward spinge la protesi ad abbandonare l'appoggio iniziale;
- con `random_init=false`, ogni episodio presenta lo stesso transitorio;
- la policy puo specializzarsi nella risposta al prefisso invece di imparare
  una strategia ciclica robusta.

## Soluzione adottata

Per il prossimo training breve e stata scelta una partenza deterministica
verificata a `12.99 s`, mantenendo il reset sulla posa prescribed completa:

```yaml
simulation:
  episode_start_offset_s: 1.0
  imitation_initialize_to_target: false
  random_init: false
```

Questa configurazione evita sia l'innesto protesico parziale sia la prima parte
del prefisso precedente al heel strike destro. Non viene considerata la
soluzione robusta definitiva: `12.99 s` e un punto iniziale verificato per il
prossimo esperimento.

## Strategia diagnostica

1. Aggiunta diagnostica di reset con confronto fra target imitativo, prescribed,
   riferimento servito, stato fisico e stati SEA.
2. Eseguiti rollout brevi con:
   - target-init a `11.99 s`;
   - prescribed-init a `11.99 s`;
   - prescribed-init a `12.99 s`;
   - target-init a `12.99 s`.
3. Ricostruiti heel strike, fase del clock, cicli usati dal template e GRF
   prescribed ai due istanti.
4. Confrontati GRF online, recruitment biologico e reserve richieste subito dopo
   il reset.

## File modificati

```text
Trajectory Generator/osim_trj_cmc_like.py
  - supporto diagnostico episode_start_offset_s;
  - payload reset_diagnostics con target/prescribed/served/actual/SEA.

Trajectory Generator/baseline_MLP/rollout_eval.py
  - propagazione dell'offset e della modalita di reset;
  - salvataggio rollout_reset_diagnostics.json e trace della policy.

Trajectory Generator/baseline_MLP/training_cfg.v4_imitation.yaml
  - episode_start_offset_s: 1.0;
  - imitation_initialize_to_target: false;
  - random_init: false.
```

Nessuna modifica e stata apportata al plugin C++ SEA o alla dinamica del
simulatore.

## Verifiche eseguite

Artefatti diagnostici:

```text
Trajectory Generator/runs/baseline_mlp_imit_v4_c2_4hz_finetune_reset_diagnostic
Trajectory Generator/runs/baseline_mlp_imit_v4_c2_4hz_finetune_reset_prescribed_diagnostic
Trajectory Generator/runs/baseline_mlp_imit_v4_c2_4hz_finetune_reset_aligned_diagnostic
Trajectory Generator/runs/baseline_mlp_imit_v4_c2_4hz_finetune_reset_target_12_99_diagnostic
```

Verifiche principali:

- confermata la coerenza fra giunti protesici e stati SEA al reset;
- confermata la perdita del contatto sinistro con target-init a `11.99 s`;
- confermata la conservazione del contatto con target-init a `12.99 s`;
- confermata la forte riduzione delle reserve usando la posa prescribed a
  `11.99 s`;
- rollout target-init dedicato a `12.99 s`: completato senza terminazione;
- configurazione YAML finale riletta e verificata.

## TODO

- [ ] Implementare una selezione robusta degli stati iniziali che verifichi
      compatibilita fra posa, velocita, target imitativo e pattern di contatto.
- [ ] Escludere o gestire esplicitamente il tratto precedente al primo heel
      strike invece di affidarsi implicitamente alla retro-estrapolazione.
- [ ] Prima di abilitare `random_init=true`, validare automaticamente molte gait
      phase e rifiutare gli stati iniziali dinamicamente incoerenti.
- [ ] Valutare se mantenere il target periodico medio oppure costruire target
      condizionati anche sullo stato di contatto.
- [ ] Dopo il prossimo training, confrontare il rollout da `12.99 s` con quello
      precedente verificando swing iniziale, GRF, reserve e generalizzazione di
      fase.
