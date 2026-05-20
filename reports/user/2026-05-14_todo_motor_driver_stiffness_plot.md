# TODO motor driver con stiffness paper-equivalent - 2026-05-14

## Problema

Le run AB06 con stiffness SEA paper-equivalent (`Knee=321 Nm/rad`,
`Ankle=500 Nm/rad`) mostrano un comportamento fisicamente piu interessante
rispetto alla baseline rigida `1000/700`: la molla lavora di piu, la
separazione tra `motor_angle` e `joint_angle` e piu visibile e, in alcune fasi,
la potenza erogata dal motore resta inferiore alla potenza espressa al giunto.

La controparte e un aumento della dinamica rapida del motor driver, visibile
come chattering/rumore nei segnali interni, soprattutto quando si ritunano i
gain per mantenere la stessa pulsazione naturale e lo stesso damping del caso
`1000/700`.

## Osservazioni dai plot

Run di riferimento:

- baseline filtrata `K=1000/700`: `results/_grf_filtered_ab06_pd_full`
- stiffness-only `K=321/500`: `results/_stiff321_500_ab06_pd_full`
- stiffness matched dynamics `K=321/500`: `results/_stiff321_500_matched_dynamics_ab06_pd_full`

Plot generati:

- `plot/14_05_2026 - 4`: stiffness-only `321/500`
- `plot/14_05_2026 - 5`: `321/500` con gain interni ritunati per matchare
  `omega_n` e `zeta` del caso `1000/700`

Metriche principali:

```text
metric                     baseline 1000/700   321/500 no retune   321/500 matched
knee tracking RMS [deg]       3.134              3.125               3.089
ankle tracking RMS [deg]      5.439              5.416               5.433
knee tau_error RMS [Nm]       3.188              3.109               1.202
ankle tau_error RMS [Nm]      1.043              0.976               0.704
reserve norm RMS            114.736            114.313             114.654
knee speed_dot RMS [rad/s2]  59.51              92.25              175.67
ankle speed_dot RMS         112.45             128.03             171.05
tau_input saturations          0                  0                   0
```

Interpretazione operativa:

- abbassare la stiffness migliora leggermente o comunque non peggiora il
  tracking cinematico;
- la molla lavora di piu, specialmente al knee, con maggiore differenza tra
  `motor_angle` e `joint_angle`;
- la potenza del motore puo essere inferiore alla potenza al giunto perche la
  molla restituisce energia elastica;
- il matching aggressivo della dinamica interna riduce molto `tau_ref -
  tau_spring`, ma aumenta accelerazioni del motore e chattering;
- le riserve alte intorno a circa `13.4 s` e `17.7 s` restano quasi invariate,
  quindi non sembrano causate principalmente dal tuning del motor driver.

## TODO futuro

Capire come migliorare il motor driver mantenendo il vantaggio fisico della
stiffness piu bassa, con tre obiettivi simultanei:

1. abbassare l'errore di tracking di `tau_ref` senza rendere il motore troppo
   aggressivo;
2. abbassare la potenza erogata dal motore rispetto alla potenza al joint,
   favorendo il contributo elastico della molla quando possibile;
3. diminuire o eliminare il chattering nei segnali `tau_input`,
   `motor_speed_dot` e `tau_ref - tau_input`.
4. quantificare il margine di banda tra `tau_ref` generato dall'outer command
   e la dinamica del motor driver: il driver deve essere piu veloce del
   contenuto in frequenza del riferimento effettivo, non necessariamente avere
   un callback digitale piu rapido dell'outer loop. Questo serve a garantire il
   disaccoppiamento frequenziale tra i due controllori.

Possibili direzioni da testare:

- rituning meno aggressivo di `Kp` rispetto al matching esatto di `omega_n`;
- damping aggiuntivo o filtro sul riferimento `tau_ref`/`u` del driver;
- feedforward energetico o spring-aware che eviti di far inseguire al motore
  componenti ad alta frequenza gia assorbibili dalla molla;
- penalizzazione esplicita di `motor_power`, `motor_speed_dot` o jerk motore
  nella scelta dei gain;
- analisi frequenziale di `tau_ref`, `tau_error` e `motor_speed_dot`,
  confrontando `f95/f99` dei segnali con `fn = omega_n / (2*pi)` del driver e
  con il campionamento `T_control = 1 ms`;
- confronto sistematico su una griglia `Kp`, `Kd`, `K` usando come metriche
  `tau_error`, `motor_power/joint_power`, `speed_dot RMS` e saturazione.

## File coinvolti

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500.osim`
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_matched_dynamics.osim`
- `results/_stiff321_500_ab06_pd_full`
- `results/_stiff321_500_matched_dynamics_ab06_pd_full`
- `plot/14_05_2026 - 4`
- `plot/14_05_2026 - 5`

## Verifiche

- Entrambe le run sono complete fino a `t=21 s`.
- Il plotter non segnala canali mancanti.
- La validation resta con `PASS=40`, `WARN=5`, `FAIL=1`; il FAIL e il solito
  `mtp_angle_r`, gia presente nelle baseline e non specifico del motor driver.
