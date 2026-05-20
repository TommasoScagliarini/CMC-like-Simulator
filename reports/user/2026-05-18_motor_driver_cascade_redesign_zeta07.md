# Redesign motor driver e cascade outer con ζ=0.7 - 2026-05-18

## Problema

Dopo il sweep cascade del mattino (run di best ankle in
`results/_cascade_full_ankle5_20260518_105345`), il tracking cinematico
risultava buono ma il chattering su `tau_input` e su `motor_velocity`
restava elevato, sia sul knee sia (soprattutto) sull'ankle.

Era necessario capire:

1. quali sono le cause strutturali del chattering,
2. quali poli del motor driver lo generano,
3. qual e' la banda corretta del motor driver (e del cascade) data
   l'asimmetria del plant fisico (Jm, Bm, Ks, F_opt),
4. quali gain produrrebbero ζ=0.7 a tutti gli stadi.

## Strategia

Analisi in cascata strutturata su tre livelli, partendo dai parametri
fisici del plant e propagando in alto verso i loop esterni.

### Livello 0 — caratterizzazione del plant

Modello SEA `AB06_SEASEA_stiff321_500_pi.osim`:

```text
SEA_Knee:  Jm=0.01, Bm=0.10, Ks=321, F_opt=100
SEA_Ankle: Jm=0.01, Bm=0.10, Ks=500, F_opt=250
```

Risonanza meccanica rotore+molla (output bloccato):

```text
ω_mech_knee  = sqrt(Ks/Jm) = 179.2 rad/s  ( 28.5 Hz)
ω_mech_ankle = sqrt(Ks/Jm) = 223.6 rad/s  ( 35.6 Hz)
```

Smorzamento passivo intrinseco:

```text
ζ_open_knee  = Bm/(2 sqrt(Ks*Jm)) ≈ 0.028
ζ_open_ankle ≈ 0.022
```

Plant praticamente non smorzato: tutto lo smorzamento deve venire dal
controllore.

Vincolo di saturazione (`tau_input` plugin a ±500 Nm) per step a piena
scala `u=1`:

```text
ω_n_max = sqrt(500 * Ks / (Jm * F_opt))
knee:  ω_n_max = 400.6 rad/s
ankle: ω_n_max = 316.2 rad/s   <-- piu' stringente
```

L'ankle e' fisicamente piu' vincolato del knee.

### Livello 1 — diagnosi del chattering attuale

Costruito il modello state-space del motor driver linearizzato
(state `[θ_m, ω_m, ξ]`, θ_j tenuto fisso). Polinomio caratteristico:

```text
s^3 + (Kd+Bm)/Jm * s^2 + (1+Kp)*Ks/Jm * s + Ki*Ks/Jm = 0
```

Eigenvalues attuali (Ki=190/123, Kp=18/11.3, Kd=11):

- knee:  coppia complessa -549.9 ± 544.3j  (ω_n=774, ζ=0.71) + polo reale -10.19
- ankle: coppia complessa -549.9 ± 549.0j  (ω_n=777, ζ=0.71) + polo reale -10.19

PI zero a `-Ki/(1+Kp)`:

- knee:  -10.0   (cancellazione del polo lento al 98 %)
- ankle: -10.0   (cancellazione del polo lento al 98 %)

Verdetto:

- coppia complessa a 124 Hz con ζ=0.71: ottima dinamica, ma **3-4× sopra
  la risonanza meccanica** (28-36 Hz). Banda eccessiva.
- polo reale a 1.6 Hz: bottleneck dell'integratore. Il velocity-loop
  cascade (7.5-12 Hz attuale) opera **sopra** la banda integrale del
  driver, quindi il driver insegue tau_ref solo via PD (Kp+Kd) ad alta
  frequenza, e l'errore residuo persiste finche' xi non si carica
  lentamente. E' il meccanismo principale del chattering.

### Livello 2 — design del motor driver con ζ=0.7

Regola standard SEA: closed-loop `ω_n ≈ 2 · ω_mech`, vincolato dal
budget di saturazione e dal sampling Nyquist (T_control=1ms).

Per ζ=0.7 e polo reale a `p=0.2·ω_n` (compromesso classico):

```text
ω_n_knee_target  = 360 rad/s  ( 57 Hz)  ≈ 2× ω_mech_knee
ω_n_ankle_target = 280 rad/s  ( 45 Hz)  ≈ 1.25× ω_mech_ankle, sotto saturation cap
```

Differenziati: il knee puo' spingere di piu', l'ankle e' limitato dal
saturation cap di tau_input.

Formule di placement (derivate matching coefficients del polinomio
caratteristico target `(s+p)(s^2 + 2ζω_n s + ω_n^2)`):

```text
Ki = Jm * p * ω_n^2 / Ks
Kp = Jm * (ω_n^2 + 2ζ*ω_n*p) / Ks - 1
Kd = Jm * (p + 2ζ*ω_n) - Bm
```

Verificate analiticamente sui coefficienti del char poly.

Risultati:

| sea  | ω_n | p   | Kp    | Kd   | Ki    |
|------|----:|----:|------:|-----:|------:|
| knee | 360 |  72 | 4.17  | 5.66 | 290.7 |
| ankle| 280 |  56 | 1.01  | 4.38 |  87.81|

Confronto con valori correnti (knee 18/11/190, ankle 11.3/11/123):

- knee: Kp ridotto 4.3×, Kd 1.9×, Ki +53 %
- ankle: Kp ridotto 11.2×, Kd 2.5×, Ki -29 %

Poli risultanti:

- knee:  -252 ± 257j (ω_n=360, ζ=0.700)  + polo reale -72  (11.5 Hz)
- ankle: -196 ± 200j (ω_n=280, ζ=0.700)  + polo reale -56  ( 8.9 Hz)

PI zero (formula corretta `-Ki/(1+Kp)`):

- knee:  -56.25   vs polo -72   → cancellazione 78 % (miss 22 %)
- ankle: -43.75   vs polo -56   → cancellazione 78 % (miss 22 %)

Miss intrinseco per p/ω_n=0.2, ζ=0.7. Non eliminabile con la sola
PI struttura del plugin: la cancellazione esatta richiederebbe
Ki=(1+Kp)(Kd+Bm)/Jm=899 (ankle), che porterebbe la coppia complessa
sull'asse immaginario (limite Routh, ζ→0). Aperto come TODO da
valutare in simulazione.

### Livello 3 — design del cascade outer

Filosofia: separazione 5× per loop annidato.

```text
ω_motor      → ω_velocity      → ω_position
knee:  360   → 72  rad/s (11.5 Hz) → 14.4 rad/s ( 2.3 Hz)
ankle: 280   → 56  rad/s ( 8.9 Hz) → 11.2 rad/s ( 1.8 Hz)
```

Velocity loop = PI su qdot_error, plant `1/(J_joint * s)`.
Back-calc di J_joint dai gain correnti (ipotizzando ζ=0.7):

```text
J_knee  ≈ 0.316 kg*m^2
J_ankle ≈ 0.0193 kg*m^2
```

Formule:

```text
Kp_v = 2 * ζ * J * ω_v
Ki_v = J * ω_v^2
```

Position loop = P puro, sistema del 1° ordine `Kp_p = ω_position`
[1/s].

Risultati cascade:

| | Kp_p [1/s] | Kp_v [N·m·s/rad] | Ki_v [N·m/rad] |
|---|---:|---:|---:|
| knee  | 14.4 | 31.85 | 1638.0 |
| ankle | 11.2 |  1.51 |   60.5 |

Confronto con cascade corrente:

| | Δ Kp_p | Δ Kp_v | Δ Ki_v |
|---|---:|---:|---:|
| knee  | -24 % |   +9 % | +19 % |
| ankle | -76 % |  -47 % | -72 % |

L'ankle subisce la riduzione piu' drastica, coerente con la riduzione
di ω_motor dal 774 a 280 rad/s (-64 %).

I_limit cascade ankle: scalato da 200 a 80 Nm (40 % di F_opt),
coerente con Ki_v ridotto.

## Soluzione

Creato nuovo modello PI e nuovo setup XML, e aggiornato `config.py`
con i nuovi gain cascade.

### File creati/modificati

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07.osim       (NUOVO)
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07_setup.xml  (NUOVO)
config.py                                                                    (MODIFICATO)
tools/motor_driver_pole_map.py                                              (NUOVO)
tools/motor_driver_pole_locus.py                                            (NUOVO)
plot/05_18_2026_motor_driver_poles/motor_driver_poles.png                   (NUOVO)
plot/05_18_2026_motor_driver_locus/motor_driver_pole_locus.png              (NUOVO)
reports/user/2026-05-18_sweep_cascade_ankle_best_config.md                  (MODIFICATO: TODO)
```

Il modello PI originale `AB06_SEASEA_stiff321_500_pi.osim` e' stato
preservato per consentire confronti con i run precedenti.

### Nuova configurazione cascade (config.py)

```python
sea_cascade_kp_outer = {
    "pros_knee_angle":  14.4,
    "pros_ankle_angle": 11.2,
}

sea_cascade_kp_inner = {
    "pros_knee_angle":  31.85,
    "pros_ankle_angle":  1.51,
}

sea_cascade_ki_inner = {
    "pros_knee_angle":  1638.0,
    "pros_ankle_angle":   60.5,
}

sea_cascade_inner_i_torque_limit = {
    "pros_knee_angle":  50.0,
    "pros_ankle_angle": 80.0,
}
```

### Nuovi gain plugin (modello zeta07)

```text
SEA_Knee:  Kp=4.17, Kd=5.66, Ki=290.7
SEA_Ankle: Kp=1.01, Kd=4.38, Ki= 87.81
integral_torque_limit: 100 (entrambi, invariato)
stiffness, motor_inertia, motor_damping, F_opt: invariati
```

## Verifiche

### Polinomio caratteristico target vs calcolato

Per knee (ω_n=360, p=72, ζ=0.7), target coefficienti:

```text
2ζω_n + p   = 576       <-> p2 = (Kd+Bm)/Jm = (5.66+0.10)/0.01 = 576    PASS
ω_n^2 + 2ζω_n*p = 165 888 <-> p1 = (1+Kp)*Ks/Jm = 5.17*321/0.01 = 165 957 PASS
p * ω_n^2   = 9 331 200 <-> p0 = Ki*Ks/Jm = 290.7*321/0.01 = 9 331 470 PASS
```

Per ankle (ω_n=280, p=56, ζ=0.7):

```text
2ζω_n + p   = 448       <-> p2 = (4.38+0.10)/0.01 = 448         PASS
ω_n^2 + 2ζω_n*p = 100 352 <-> p1 = (1+1.01)*500/0.01 = 100 350    PASS
p * ω_n^2   = 4 390 400 <-> p0 = 87.81*500/0.01 = 4 390 500     PASS
```

### Compilazione

```bash
python -m py_compile config.py
```

Esito: **PASS**.

### Verifica gain plugin nel nuovo .osim

```bash
grep "<Kp>\|<Kd>\|<Ki>\|<stiffness>" models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_zeta07.osim
```

Output:

```text
<stiffness>321</stiffness>
<Kp>4.17</Kp>
<Kd>5.66</Kd>
<Ki>290.7</Ki>
<stiffness>500</stiffness>
<Kp>1.01</Kp>
<Kd>4.38</Kd>
<Ki>87.81</Ki>
```

Coerente con il design.

### Plot generati

- `plot/05_18_2026_motor_driver_poles/motor_driver_poles.png`:
  mappa dei poli del motor driver corrente (PI). Mostra coppia
  complessa a 124 Hz, polo reale a 1.6 Hz, e zero PI quasi
  cancellante.
- `plot/05_18_2026_motor_driver_locus/motor_driver_pole_locus.png`:
  luogo delle radici al variare di ω_n con ζ=0.7 e p=0.2·ω_n.
  Mostra il punto operativo corrente (×) e il nuovo design (★) per
  entrambi i SEA.

## Tabella gerarchica finale (ζ=0.7 a tutti i livelli)

| Loop          | knee [Hz] | ankle [Hz] | Rapporto vs interno |
|---------------|----------:|-----------:|---------------------|
| Position      |  2.3      |  1.8       | —                   |
| Velocity      | 11.5      |  8.9       | 5×                  |
| Motor driver  | 57.3      | 44.6       | 5×                  |

Tutti i rapporti sono 5×, gerarchia cascade rispettata.

## TODO

- **Smoke test e full run** con il nuovo setup
  `AB06_SEASEA_stiff321_500_pi_zeta07_setup.xml`. Validare:
  - assenza di saturazioni `tau_input`,
  - assenza di saturazioni `u`,
  - tracking cinematico vs run precedenti,
  - HPF >50 Hz su `tau_input`, motor_speed, motor_speed_dot,
  - motor power.

- **Confronto PD / PI / cascade aggressive / cascade zeta07** sui
  plot consolidati per chiudere il loop sulla strategia di chattering.

- **Valutare l'impatto della mancata cancellazione zero/polo nell'ankle**:
  miss del 22% intrinseco al rapporto p/ω_n=0.2 con ζ=0.7. Verificare
  in simulazione se la coda lenta a 8.9 Hz produce artefatti su
  `tau_spring`, motor power o chattering residuo. Se rilevante,
  considerare riduzione di p/ω_n a 0.1 (integratore piu' lento ma
  cancellazione migliore al 12 % di miss).

- **Aggiornare il default `config.py`** `model_bundle_dir` e
  `model_file` per puntare al nuovo PI zeta07, oppure passare il
  setup XML esplicitamente nei script di esecuzione.

- **Validare J_joint usato nel back-calc**: confrontare i nuovi
  Kp_v/Ki_v con run reali per verificare l'assunzione di J=0.316
  (knee) e J=0.0193 (ankle).

- **Windows pendente**: build/copia DLL plugin PI sulla macchina
  Windows + smoke load/run e verifica del modello PI con
  `Ki`, `integral_torque_limit`, `torque_error_integral`.

- **Pulizia artefatti**: valutare se conservare o cancellare cartelle
  di sweep parziali (`_cascade_local_gain_sweep_20260517_233607`,
  `_cascade_local_gain_sweep_20260517_234151`).
