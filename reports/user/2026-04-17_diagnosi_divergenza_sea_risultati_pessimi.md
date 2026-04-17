# Diagnosi divergenza SEA e risultati pessimi - 2026-04-17

## Contesto

L'analisi nasce dai risultati pessimi ottenuti dopo la modifica dei parametri
`K`, `Kp`, `Kd` per knee e ankle nel modello:

```text
models/Adjusted_SEASEA - Copia_tuned.osim
```

Obiettivo: capire se la divergenza e i risultati degradati dipendono da:

- tuning dell'outer loop;
- tuning dell'inner loop / low-level controller SEA;
- modellazione o logica del SEA;
- altro.

Vincoli rispettati durante la diagnosi:

- nessuna modifica a codice, modello tracked o cartella `results/`;
- run diagnostiche salvate solo in `/tmp`;
- nessuna reserve protesica reintrodotta nel QP;
- nessun forcing cinematico dei DOF protesici;
- nessuna modifica alla semantica del plugin C++ SEA.

## Artefatti prodotti

Directory diagnostica:

```text
/tmp/cmc_sea_diagnostics_20260417
```

File principali:

```text
/tmp/cmc_sea_diagnostics_20260417/diagnosis.md
/tmp/cmc_sea_diagnostics_20260417/summary.md
/tmp/cmc_sea_diagnostics_20260417/summary.csv
/tmp/cmc_sea_diagnostics_20260417/reports/
/tmp/cmc_sea_diagnostics_20260417/models/
/tmp/cmc_sea_diagnostics_20260417/runs/
```

Le copie `.osim` usate per i test sono state create sotto `/tmp` e non hanno
toccato il modello del repository.

## Parametri testati

### Modello tuned corrente

Parametri letti da `Adjusted_SEASEA - Copia_tuned.osim`:

```text
SEA_Knee:
  K  = 1400
  Kp = 0.786
  Kd = 6.9

SEA_Ankle:
  K  = 900
  Kp = 1.778
  Kd = 6.9
```

### Baseline storica stabile

Parametri indicati dai report del 16/04/2026 come ultimo inner loop validato:

```text
SEA_Knee:
  K  = 250
  Kp = 18.5
  Kd = 10

SEA_Ankle:
  K  = 500
  Kp = 35
  Kd = 20
```

## Matrice diagnostica

Sono state eseguite run brevi su `t = 4.26 -> 7.20 s`, per includere i picchi
osservati nella run pessima:

- picco knee intorno a `5.001 s`;
- picco ankle intorno a `7.032 s`.

Varianti inner loop:

- `T0_current_short`: parametri correnti del modello tuned;
- `T1_last_good_inner_short`: parametri inner storici validati;
- `T2_current_K_old_gains_short`: stiffness correnti, ma `Kp/Kd` storici;
- `T3_old_K_formula_gains_short`: stiffness storiche con gain da formula sweep.

Varianti outer loop testate dopo aver isolato l'inner stabile:

- `T1_last_good_inner_outer_mid_short`: `sea_kp=10`, `sea_kd=1.0`;
- `T1_last_good_inner_outer_high_short`: `sea_kp=20`, `sea_kd=2.0`.

E stata inoltre eseguita una full run:

- `T1_last_good_inner_full`: `t = 4.26 -> 11.06 s`.

## Risultati principali

| Caso | Stato | Knee max/RMS [deg] | Ankle max/RMS [deg] | Knee tau error RMS [Nm] | Ankle tau error RMS [Nm] | Knee max \|u\| | Ankle max \|u\| |
|---|---:|---:|---:|---:|---:|---:|---:|
| `T0_current_short` | complete | `55.464 / 25.106` | `484.530 / 214.357` | `2.907` | `20.205` | `0.337` | `0.980` |
| `T1_last_good_inner_short` | complete | `15.758 / 7.631` | `18.712 / 8.807` | `1.308` | `0.867` | `0.279` | `0.359` |
| `T2_current_K_old_gains_short` | complete | `16.520 / 7.577` | `18.827 / 8.810` | `1.319` | `0.864` | `0.277` | `0.374` |
| `T3_old_K_formula_gains_short` | complete | `20.923 / 10.599` | `164.368 / 81.589` | `1.567` | `7.511` | `0.295` | `0.531` |
| `T1_last_good_inner_outer_mid_short` | complete | `9.497 / 4.600` | `8.037 / 3.864` | `1.404` | `0.813` | `0.302` | `0.290` |
| `T1_last_good_inner_outer_high_short` | complete | `5.418 / 2.571` | `4.091 / 1.971` | `1.517` | `0.885` | `0.308` | `0.270` |
| `T1_last_good_inner_full` | complete | `16.978 / 7.957` | `18.712 / 8.088` | `1.144` | `0.774` | `0.279` | `0.359` |

La full run `T1_last_good_inner_full` ha completato:

```text
6800 step
t = 11.06 s
left gait cycles  = 5
right gait cycles = 5
```

Validator:

```text
PASS = 44
WARN = 2
FAIL = 0
```

## Diagnosi

### Causa primaria: tuning inner loop SEA

La causa primaria dei risultati pessimi e:

```text
b) tuning dell'inner loop / low-level controller del SEA
```

Evidenze:

- il modello corrente fallisce pesantemente soprattutto sull'ankle:
  - `pros_ankle_angle` RMS circa `214 deg`;
  - max error circa `484 deg`;
- sostituendo solo i parametri low-level SEA con la baseline storica, l'ankle
  scende a circa `8.8 deg` RMS nella stessa finestra;
- mantenendo le stiffness correnti ma ripristinando `Kp/Kd` storici si ottiene
  quasi lo stesso miglioramento;
- quindi il problema piu dannoso non e `K` da solo, ma i gain inner correnti
  troppo bassi:
  - `SEA_Knee Kp=0.786`;
  - `SEA_Ankle Kp=1.778`;
  - `Kd=6.9` per entrambi.

### Outer loop: fattore secondario

L'outer loop contribuisce al tracking, ma non spiega il collasso originale.

Evidenze:

- con inner loop gia stabile, aumentare l'outer migliora molto il tracking:
  - `outer_mid`: ankle RMS circa `3.9 deg`;
  - `outer_high`: ankle RMS circa `2.0 deg`;
- questo effetto appare solo dopo aver ripristinato un inner loop stabile;
- l'outer loop non e cambiato tra il caso corrente pessimo e i test con
  inner storico, quindi non puo essere la causa primaria del collasso.

### Logica/modellazione SEA: non supportata come causa primaria

La diagnosi non supporta al momento:

```text
c) errore di modellazione o logica SEA
```

Evidenze:

- accordo plugin/Python su `tau_input`: PASS;
- derivate SEA finite: PASS;
- stato motore non algebricamente forzato: PASS;
- nessuna tautologia `motor_angle = q + tau_ref/K`;
- nel caso corrente pessimo il clamp `tau_input = +/-500 Nm` non viene
  raggiunto.

### Altro: non primario

La categoria:

```text
d) altro
```

non e la causa primaria in questa analisi.

Evidenze:

- pelvis e giunti biologici passano il validator;
- la failure e concentrata sui due DOF protesici;
- la GRF produce gait events completi nella full run validata;
- non sono emersi segnali di path/reference/unit conversion come causa
  dominante.

## Raccomandazione

Non mantenere nel modello tuned corrente i gain SEA:

```text
SEA_Knee:
  K  = 1400
  Kp = 0.786
  Kd = 6.9

SEA_Ankle:
  K  = 900
  Kp = 1.778
  Kd = 6.9
```

Ripartire dalla baseline inner stabile:

```text
SEA_Knee:
  K  = 250
  Kp = 18.5
  Kd = 10

SEA_Ankle:
  K  = 500
  Kp = 35
  Kd = 20
```

Solo dopo aver fissato l'inner loop, procedere con tuning outer separato.

La variante:

```text
sea_kp = 20
sea_kd = 2.0
```

migliora molto nella finestra breve, ma deve essere validata su full run prima
di diventare default, perche aumenta il carattere controller-following del
risultato.

## Stato finale

- Tutte le run diagnostiche sono state salvate in `/tmp`.
- Nessun processo diagnostico e rimasto in esecuzione.
- Nessun file tracked del progetto e stato modificato dalla diagnosi, a parte
  questo report utente creato con `create_report`.
