# Daily Report - 2026-05-20

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-05-20_diagnosi_hpf50_motor_driver_coupling_knee_ankle.md`

## Problema

Il chattering osservato dopo l'introduzione del cascade poteva dipendere dal
cascade outer, dal motor driver o dal coupling meccanico knee-ankle. Il filtro
LPF su `qdot` del 19 maggio riduceva il rumore, ma peggiorava il knee tracking.

## Soluzione e strategia

E' stata eseguita una decomposizione HPF50 su cinque configurazioni: PID
single-loop, cascade morning-best, motor driver lento su entrambi i SEA e
variante asimmetrica. Due nuove run complete da `9010` step hanno isolato
l'effetto del motor driver mantenendo invariato il tracking outer.

## Risultati e decisione

- L'ipotesi che il cascade amplifichi il rumore e' falsificata: rispetto al
  PID single-loop il cascade e' circa `9-10%` piu' pulito su `tau_input`.
- Sul knee il motor driver veloce eccita la risonanza spring-rotor a `28 Hz`.
- Il driver lento riduce `tau_input` HPF50 knee del `70%` senza peggiorare il
  tracking in modo significativo.
- Sul lato ankle il driver lento peggiora il chattering del `53%`; la variante
  asimmetrica arriva a `+71%`, evidenziando coupling knee-ankle.
- La baseline operativa resta il morning-best del 18 maggio; nessun nuovo
  modello e' stato promosso.

## File e artefatti

- modelli sperimentali slow-inner e asimmetrico;
- script diagnostici HPF50;
- `results/_slow_inner_pd1405_cascade_morning_20260520/`;
- `results/_asym_knee1405_ankle_morning_20260520/`.

## Test e verifiche

- entrambe le run: complete, `9010` step;
- equivalenza Ki=0 con PD puro verificata nel plugin;
- identita' algebriche PID/cascade HPF50 verificate entro circa `1%`;
- tracking knee e ankle invariato fra i confronti controllati.

## TODO aperti e propagati

- [ ] Valutare valori intermedi di `Kp_knee_motor` fra `3.9` e `18`.
- [ ] Validare direttamente il coupling knee-ankle isolando la dinamica knee.
- [ ] Valutare un notch a `28 Hz` se si vuole evitare il retuning del driver.
- [ ] Decidere se archiviare i modelli `slow_inner_pd_1405` e
      `pi_asym_knee1405`.
- [ ] Completare build e smoke Windows del plugin PI.
- [ ] Produrre un confronto consolidato delle configurazioni storiche.

Questi TODO sono stati ripresi esplicitamente nel daily del 29 maggio.

