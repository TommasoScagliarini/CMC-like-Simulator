# Analisi cascade SEA con J_eff corretta — 2026-05-19

Documento di analisi teorica, nessuna modifica al codice o ai modelli.
Scopo: rifare il design cascade del 2026-05-18 (zeta07) sostituendo le
inerzie di back-calc con le inerzie effettive validate il 2026-05-19.

## 1. Dati di ingresso

### 1.1 Plant SEA (modello `AB06_SEASEA_stiff321_500_pi.osim`)

```text
SEA_Knee:  Jm=0.01, Bm=0.10, Ks=321, F_opt=100
SEA_Ankle: Jm=0.01, Bm=0.10, Ks=500, F_opt=250
```

Risonanza meccanica rotore+molla (output bloccato):

```text
omega_mech_knee  = sqrt(Ks/Jm) = sqrt(321/0.01)  = 179.2 rad/s  (28.5 Hz)
omega_mech_ankle = sqrt(Ks/Jm) = sqrt(500/0.01)  = 223.6 rad/s  (35.6 Hz)
```

Smorzamento passivo:

```text
zeta_open_knee  = Bm/(2 sqrt(Ks*Jm))  ~ 0.028
zeta_open_ankle ~ 0.022
```

Saturazione plugin (tau_input clamp +/-500 N*m) per step u=1:

```text
omega_n_max = sqrt(500 * Ks / (Jm * F_opt))
knee:  omega_n_max = 400.6 rad/s
ankle: omega_n_max = 316.2 rad/s     <-- vincolo piu' stringente
```

### 1.2 Inerzia effettiva validata (report `2026-05-19_validazione_inerzia_effettiva_knee_ankle.md`)

Mediane su 181 sample lungo l'IK del setup PI:

| coord | J_back (zeta07) | J_free_all | J_locked | J_free_pros_pair |
|---|---:|---:|---:|---:|
| pros_knee_angle  | 0.316  | 0.159738 | 0.493791 | 0.444954 |
| pros_ankle_angle | 0.0193 | 0.009892 | 0.011098 | 0.010000 |

Rapporti J_free_all / J_back:

```text
knee:  0.505
ankle: 0.513
```

Il back-calc del 2026-05-18 sovrastimava ~2x l'inerzia libera in entrambi
i DOF.

Per l'ankle le tre stime convergono entro l'11%, quindi
`J_eff_ankle ~ 0.010 kg*m^2` e' un valore robusto.

Per il knee la stima dipende dal vincolo:

- plant libero (impulsivo):           `J_free_all = 0.160`
- coppia protesica libera:            `J_pair     = 0.445`
- altri DOF accelerazione zero:       `J_locked   = 0.494`

### 1.3 Configurazioni di riferimento

Motor driver PI plugin (best mattino 18/05, ripristinato a fine giornata):

```text
SEA_Knee:  Kp=18,   Kd=11, Ki=190
SEA_Ankle: Kp=11.3, Kd=11, Ki=123
integral_torque_limit = 100 (entrambi)
```

Cascade outer (best mattino 18/05, configurazione attiva):

```text
knee:  Kp_outer=18.85,  Kp_inner=29.2,    Ki_inner=1377.0, I_limit=50
ankle: Kp_outer=47.125, Kp_inner=2.8275,  Ki_inner=213.0,  I_limit=200
```

Cascade outer (design zeta07 18/05, fallito su ankle):

```text
knee:  Kp_outer=14.4, Kp_inner=31.85, Ki_inner=1638.0, I_limit=50
ankle: Kp_outer=11.2, Kp_inner=1.51,  Ki_inner=60.5,   I_limit=80
```

## 2. Modello del motor driver e formule

State linearizzato `[theta_m, omega_m, xi]` con `theta_j` fisso.
Polinomio caratteristico:

```text
s^3 + (Kd + Bm)/Jm * s^2 + (1 + Kp)*Ks/Jm * s + Ki*Ks/Jm = 0
```

Target ζ=0.7 con polo reale `p = 0.2 * omega_n`:

```text
Ki = Jm * p * omega_n^2 / Ks
Kp = Jm * (omega_n^2 + 2*zeta*omega_n*p) / Ks - 1
Kd = Jm * (p + 2*zeta*omega_n) - Bm
```

Zero PI (importante per chattering):

```text
zero_PI = -Ki / (1 + Kp)
```

Con `p/omega_n = 0.2` e `zeta=0.7` la cancellazione zero-polo e' ~78%,
miss intrinseco 22%.

## 3. Caratterizzazione del motor driver corrente

Eigenvalues calcolati con Ki=190/123, Kp=18/11.3, Kd=11:

```text
knee:  coppia complessa -549.9 +/- 544.3j  (omega_n=774, zeta=0.71)
       polo reale       -10.19
ankle: coppia complessa -549.9 +/- 549.0j  (omega_n=777, zeta=0.71)
       polo reale       -10.19
```

Quindi il PI plugin attuale e' **gia' un design a zeta=0.71** ma con
banda alta (774-777 rad/s, 123 Hz), ben sopra la risonanza meccanica
(28-36 Hz).

Zero PI corrente:

```text
knee:  -Ki/(1+Kp) = -190/19   = -10.0
ankle: -Ki/(1+Kp) = -123/12.3 = -10.0
```

Cancellazione del polo reale al 98% (-10 vs -10.19), molto buona.

## 4. Motor driver zeta07 (design 18/05)

Target piu' conservativo per ridurre chattering oltre risonanza:

```text
omega_n_knee  = 360 rad/s (57 Hz)  ~ 2x omega_mech_knee
omega_n_ankle = 280 rad/s (45 Hz)  ~ 1.25x omega_mech_ankle, sotto cap 316
```

Applicando le formule del §2:

| sea  | omega_n | p   | Kp    | Kd   | Ki    |
|------|--------:|----:|------:|-----:|------:|
| knee |  360    |  72 | 4.17  | 5.66 | 290.7 |
| ankle|  280    |  56 | 1.01  | 4.38 |  87.81|

**Il motor driver zeta07 dipende solo dal plant fisico (Jm, Bm, Ks),
non da J_eff, quindi resta invariato anche dopo la validazione J_eff.**

Verifica analitica per ankle (omega_n=280, p=56, zeta=0.7):

```text
2*zeta*omega_n + p   = 448         (Kd+Bm)/Jm = (4.38+0.10)/0.01 = 448
omega_n^2 + 2*z*on*p = 100 352     (1+Kp)*Ks/Jm = 2.01*500/0.01 = 100 500
p * omega_n^2        = 4 390 400   Ki*Ks/Jm = 87.81*500/0.01 = 4 390 500
```

Errori entro 0.15%.

## 5. Re-interpretazione del morning best con J_eff corretta

Conoscendo `Kp_v` e `Ki_v` del morning best, si possono ricavare la
bandwidth reale e lo smorzamento reale del velocity loop sotto le
diverse ipotesi di J:

```text
omega_v_actual = sqrt(Ki_v / J)
zeta_actual    = Kp_v / (2 * J * omega_v_actual)
```

### 5.1 Ankle (Kp_v=2.8275, Ki_v=213, J_eff=0.010)

```text
omega_v_actual = sqrt(213/0.010)        = 146 rad/s   (23.2 Hz)
zeta_actual    = 2.8275/(2*0.010*146)   = 0.97        (sovrasmorzato)
omega_p_actual = 47.125                 1/s           (7.5 Hz)

ratio omega_motor/omega_v = 777/146 = 5.3x
ratio omega_v/omega_p     = 146/47  = 3.1x
```

Quindi il morning best ankle implementa una gerarchia **5x/3x**, non
5x/5x, e opera a banda velocity ~+40% rispetto al design back-calc del
2026-05-18.

### 5.2 Knee con J_free=0.160 (Kp_v=29.2, Ki_v=1377)

```text
omega_v_actual = sqrt(1377/0.160)       = 92.8 rad/s
zeta_actual    = 29.2/(2*0.160*92.8)    = 0.98
```

### 5.3 Knee con J_pair=0.445

```text
omega_v_actual = sqrt(1377/0.445)       = 55.6 rad/s
zeta_actual    = 29.2/(2*0.445*55.6)    = 0.59         (sotto-smorzato)
```

Letture:

- se la dinamica knee reale e' vicina a J_free, il morning best era
  fortemente sovrasmorzato (analogo all'ankle);
- se invece e' vicina a J_pair, il morning best era sotto-smorzato
  (zeta=0.59) e questo potrebbe spiegare il chattering residuo sul
  knee anche nella configurazione mattutina.

## 6. Cascade outer con J_eff corretta

Formule (velocity loop PI su plant scalare `1/(J*s)`, position loop P
puro):

```text
Kp_v = 2 * zeta * J * omega_v
Ki_v = J * omega_v^2
Kp_p = omega_p           [1/s]
```

Con `zeta=0.7` in tutti i casi.

### 6.1 Variante V1 — 5x/5x stretto (zeta07 originale + J_eff)

Motor driver zeta07 (knee 360, ankle 280).

```text
knee  omega_v = 72 rad/s   omega_p = 14.4 rad/s
ankle omega_v = 56 rad/s   omega_p = 11.2 rad/s
```

Ankle (J=0.010):

```text
Kp_v = 2*0.7*0.010*56 = 0.784
Ki_v = 0.010*56^2     = 31.36
Kp_p = 11.2
```

Knee, tre opzioni:

| | A: J=0.160 | B: J=0.445 | C: J=0.494 |
|---|---:|---:|---:|
| Kp_v | 16.128 | 44.856 | 49.795 |
| Ki_v | 829.44 | 2307.0 | 2561.0 |
| Kp_p | 14.4   | 14.4   | 14.4   |

Limite operativo: la position loop ankle a 11.2 1/s e' lentissima.
Il fallimento zeta07 del 18/05 e' stato proprio su questo punto
(integratore al clamp 14.7% del tempo, ankle RMS 27.5 deg).
Mantenere 5x/5x peggiora ulteriormente la situazione perche' i
gain ankle scendono ancora del 48% rispetto a zeta07 originale.

### 6.2 Variante V2 — Hybrid 5x/3x (mimica morning best)

Motor driver zeta07 invariato. Posizione presa a `omega_v / 3`.

```text
knee  omega_v = 72   omega_p = 24    rad/s (3.82 Hz)
ankle omega_v = 56   omega_p = 18.7  rad/s (2.97 Hz)
```

Ankle (J=0.010):

```text
Kp_v = 2*0.7*0.010*56 = 0.784
Ki_v = 0.010*56^2     = 31.36
Kp_p = 18.7
```

Knee, due opzioni rilevanti:

| | A: J=0.160 | B: J=0.445 |
|---|---:|---:|
| Kp_v | 16.128 | 44.856 |
| Ki_v | 829.44 | 2307.0 |
| Kp_p | 24     | 24     |

Limite: ω_p_ankle = 18.7 1/s, ancora ~2.5x sotto al morning best (47).
Migliora rispetto a V1 ma non recupera il regime in cui il morning best
funziona.

### 6.3 Variante V3 — 3x/3x con motor zeta07

Motor driver invariato. Separazione 3x a tutti i livelli.

```text
knee  omega_v = 120  omega_p = 40    rad/s (6.37 Hz)
ankle omega_v = 93.3 omega_p = 31.1  rad/s (4.95 Hz)
```

Ankle (J=0.010):

```text
Kp_v = 2*0.7*0.010*93.3 = 1.3062
Ki_v = 0.010*93.3^2     = 87.05
Kp_p = 31.1
```

Knee A (J=0.160):

```text
Kp_v = 2*0.7*0.160*120 = 26.88
Ki_v = 0.160*120^2     = 2304.0
Kp_p = 40
```

Knee B (J=0.445):

```text
Kp_v = 2*0.7*0.445*120 = 74.76
Ki_v = 0.445*120^2     = 6408.0
Kp_p = 40
```

Il caso B porta Ki_v=6408, 4.7x il morning best: probabilmente troppo
aggressivo, rischio chattering knee. Il caso A resta nel range
fisicamente sensato.

Coerente con il TODO 18/05 per ankle:

```text
TODO 18/05:    Kp_outer_ankle ~30-40 1/s, omega_v_ankle ~90-100 rad/s
V3 J_eff:      Kp_outer_ankle = 31.1,     omega_v_ankle = 93.3 rad/s
```

I gain TODO 18/05 (`Kp_v=2.5, Ki_v=175`) erano calcolati con
`J_back=0.0193`. Con `J_eff=0.010` i gain analitici scendono a circa
meta' (Kp_v=1.31, Ki_v=87). La banda resta identica.

### 6.4 Variante V3' — V3 con motor ankle al cap saturazione

Per recuperare banda cascade ankle pur mantenendo separazione 3x,
si puo' spingere il motor driver ankle fino al cap fisico
`omega_n_max = 316 rad/s`. Motor driver ankle ricomputato:

```text
omega_n = 316, p = 0.2*316 = 63.2, zeta = 0.7

Ki_a = 0.01 * 63.2 * 316^2 / 500 = 126.2
Kp_a = 0.01 * (316^2 + 2*0.7*316*63.2) / 500 - 1
     = 0.01 * (99 856 + 27 954) / 500 - 1
     = 2.556 - 1 = 1.556
Kd_a = 0.01 * (63.2 + 2*0.7*316) - 0.10
     = 0.01 * 505.6 - 0.10
     = 4.956
```

Zero PI ankle:

```text
zero_PI = -126.2 / (1 + 1.556) = -49.4   vs polo reale -63.2
miss = 1 - 49.4/63.2 = 22%   (identico a zeta07, intrinseco a p/on=0.2)
```

Cascade ankle:

```text
omega_v_ankle = 316/3 = 105.3 rad/s (16.8 Hz)
omega_p_ankle = 105.3/3 = 35.1 rad/s (5.58 Hz)

Kp_v = 2*0.7*0.010*105.3 = 1.4742
Ki_v = 0.010*105.3^2     = 110.88
Kp_p = 35.1
```

Knee invariato rispetto a V3.

Caveat operativo: il motor driver e' al cap di saturazione teorico.
Va validato che il sat_count resti zero durante il push-off.

## 7. Tabella sintetica delle varianti

Confronto con morning best e zeta07 originale.

| variante | knee Kp_p / Kp_v / Ki_v | ankle Kp_p / Kp_v / Ki_v | omega_p ankle [1/s] | motor ankle |
|---|---|---|---:|---|
| morning best                | 18.85 / 29.2  / 1377  | 47.125 / 2.83  / 213    | 47.13 | 11.3/11/123 (omega_n=777) |
| zeta07 originale (J_back)   | 14.4  / 31.85 / 1638  | 11.2   / 1.51  / 60.5   | 11.20 | 1.01/4.38/87.81 (omega_n=280) |
| V1 5x/5x + J_eff (knee A)   | 14.4  / 16.13 / 829.4 | 11.2   / 0.784 / 31.36  | 11.20 | invariato zeta07 |
| V1 5x/5x + J_eff (knee B)   | 14.4  / 44.86 / 2307  | 11.2   / 0.784 / 31.36  | 11.20 | invariato zeta07 |
| V2 5x/3x + J_eff (knee A)   | 24    / 16.13 / 829.4 | 18.7   / 0.784 / 31.36  | 18.70 | invariato zeta07 |
| V2 5x/3x + J_eff (knee B)   | 24    / 44.86 / 2307  | 18.7   / 0.784 / 31.36  | 18.70 | invariato zeta07 |
| V3 3x/3x + J_eff (knee A)   | 40    / 26.88 / 2304  | 31.1   / 1.306 / 87.05  | 31.10 | invariato zeta07 |
| V3 3x/3x + J_eff (knee B)   | 40    / 74.76 / 6408  | 31.1   / 1.306 / 87.05  | 31.10 | invariato zeta07 |
| V3' 3x/3x + motor cap       | 40    / 26.88 / 2304  | 35.1   / 1.474 / 110.9  | 35.10 | 1.56/4.96/126.2 (omega_n=316) |

Caso knee B = J_pair=0.445.
Caso knee A = J_free=0.160.

## 8. Discussione

### 8.1 Banda ankle disponibile vs banda richiesta

Il morning best ankle a omega_p=47 1/s (7.5 Hz) traccia bene il
push-off. Le varianti che restano sotto questo limite (V1, V2) sono
attese sotto-performanti sul tracking ankle, indipendentemente dalla J
usata. Solo V3 e V3' raggiungono omega_p > 30 1/s, ma restano sotto il
morning best.

Tradeoff:

- la banda alta del morning best (motor driver omega_n=777) produce
  chattering oltre 50 Hz;
- la banda ridotta zeta07 elimina il chattering ma rende l'ankle
  insufficiente.

V3' tenta il compromesso: motor driver al cap fisico, gerarchia 3x,
J_eff corretta. Resta da capire (con simulazione) se la banda
intermedia (omega_n=316) e' sufficiente a ridurre il chattering rispetto
al morning best.

### 8.2 Scelta J per il knee

- J_free=0.160 produce gain bassi vicini ad A; coerente con risposta a
  disturbi impulsivi;
- J_pair=0.445 produce gain alti vicini al morning best; coerente con
  steady-state walking;
- J_locked=0.494 e' un limite superiore non realistico (gli altri DOF
  non sono mai realmente bloccati).

Senza un test diretto (step in qdot_ref) la scelta sicura per il design
e' una media `J ~ 0.27 kg*m^2`, molto vicina al back-calc originale
0.316. Con questa J:

```text
Kp_v = 2*0.7*0.27*120 = 45.36   (V3)
Ki_v = 0.27*120^2     = 3888    (V3)
Kp_p = 40
```

Resta da decidere come misurare J_knee operativo durante walking.

### 8.3 Miss zero/polo nel PI

Con `p/omega_n = 0.2` e `zeta = 0.7`, il miss del zero PI vs polo reale
e' 22%. Vale per knee 360, ankle 280, ankle 316. La cancellazione
esatta richiederebbe `Ki = (1+Kp)*(Kd+Bm)/Jm`, che porta la coppia
complessa sull'asse immaginario (Routh, zeta -> 0). E' un vincolo
strutturale della topologia PI plugin attuale e non si elimina con
ritarature.

## 9. Configurazione teorica raccomandata

Combina i punti del §8: V3' per ankle (motor cap + 3x/3x), V3 per knee
con J=0.27 come compromesso prudente.

```text
Motor driver:
  SEA_Knee:  Kp=4.17,  Kd=5.66, Ki=290.7   (zeta07, invariato)
  SEA_Ankle: Kp=1.56,  Kd=4.96, Ki=126.2   (NUOVO, omega_n=316 rad/s)

Cascade outer:
  knee  (J=0.27):  Kp_outer=40,   Kp_inner=45.36, Ki_inner=3888,   I_limit=50
  ankle (J=0.010): Kp_outer=35.1, Kp_inner=1.474, Ki_inner=110.88, I_limit=80
```

In alternativa, design 'conservativo' senza spingere il motor ankle al
cap, accettando ankle leggermente piu' lento del morning best:

```text
Motor driver: invariato zeta07.

Cascade outer:
  knee  (J=0.27):  Kp_outer=40,   Kp_inner=45.36, Ki_inner=3888,  I_limit=50
  ankle (J=0.010): Kp_outer=31.1, Kp_inner=1.306, Ki_inner=87.05, I_limit=80
```

## 10. TODO da chiudere prima di promuovere il design

Esito 2026-05-19: TODO chiusi su finestra critica di push-off
`13.1638 -> 14.7799 s`. La proposta teorica del §9 **non viene promossa**:
riduce il chattering ankle, ma perde troppo tracking cinematico.

Artefatti prodotti:

- `validation/cascade_qdot_step_inertia.py`
- `validation/cascade_jeff_todo_runner.py`
- `results/_cascade_qdot_step_20260519/`
- `results/_cascade_jeff_todo_20260519_screen_morning_pushoff1/`
- `results/_cascade_jeff_todo_20260519_screen_pushoff1/`
- `results/_cascade_jeff_todo_20260519_screen_p01_pushoff1/`

### 10.1 Scelta J_knee con step in qdot_ref

Test: `delta_qdot_ref = 1 rad/s`, errore posizione nullo, integratore
cascade nullo. Quindi il comando immediato e':

```text
tau_cmd = Kp_inner * delta_qdot_ref
```

Risposta mediana lungo 181 sample IK:

| design | Kp_inner | Ki_inner | tau step | J_free | qddot free | J_pair | qddot pair | J_locked | qddot locked |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| morning_best | 29.2 | 1377 | 29.2 | 0.1597 | 182.8 | 0.4450 | 65.62 | 0.4938 | 59.13 |
| V3a J_free | 26.88 | 2304 | 26.88 | 0.1597 | 168.3 | 0.4450 | 60.41 | 0.4938 | 54.44 |
| V3m J_mid | 45.36 | 3888 | 45.36 | 0.1597 | 284.0 | 0.4450 | 101.9 | 0.4938 | 91.86 |

Conclusione: `J_free_all` predice ~2.79x piu' accelerazione del bracket
`J_free_pros_pair`; `J_free_pros_pair` e `J_locked` sono invece vicini
(~1.11x). Quindi il knee non ha un unico J scalare robusto: il valore
free spiega bene la risposta impulsiva, mentre pair/locked e' il bracket
prudente quando il resto del modello e' cinematicamente vincolato. Per il
design cascade non basta scegliere `J=0.27`: va verificato in simulazione.

### 10.2 Screen comparativo push-off

Finestra: `13.1638 -> 14.7799 s`. Tutte le righe sotto sono simulate sulla
stessa finestra, quindi sono confrontabili fra loro.

| run | knee RMSE deg | ankle RMSE deg | knee max u | ankle max u | sat knee/ankle | esito |
|---|---:|---:|---:|---:|---:|---|
| morning_best | 0.207 | 2.410 | 0.654 | 0.636 | 0 / 0 | OK |
| V3a J_free | 0.180 | 15.614 | 1.000 | 0.549 | 0 / 0 | FAIL: knee `u` a 1, ankle tracking perso |
| V3m J_mid | 0.463 | 15.118 | 1.000 | 0.549 | 0 / 0 | FAIL: knee aggressivo, ankle tracking perso |
| V3' motor cap | 0.404 | 11.318 | 1.000 | 0.536 | 0 / 0 | FAIL: ankle migliora ma resta fuori scala |
| V3' p/on=0.1 | 0.291 | 11.246 | 1.000 | 0.521 | 0 / 0 | FAIL: zero/polo non e' la causa dominante |

Nessuna variante satura `tau_input` nel plugin (`sat_count=0`), incluso
V3' con motor ankle al cap. Il limite osservato e' il tracking: l'ankle
non segue anche se `ankle_max_u` resta ~0.52-0.55.

### 10.3 Chattering HPF50

| run | knee tau HPF50 | ankle tau HPF50 | knee motor_speed_dot HPF50 | ankle motor_speed_dot HPF50 |
|---|---:|---:|---:|---:|
| morning_best | 15.69 | 19.07 | 1688 | 2102 |
| V3a J_free | 8.22 | 1.28 | 968 | 338 |
| V3m J_mid | 81.17 | 1.37 | 10671 | 479 |
| V3' motor cap | 74.08 | 2.30 | 9784 | 566 |
| V3' p/on=0.1 | 37.80 | 1.82 | 5149 | 422 |

Conclusione: il redesign riduce molto il chattering ankle, ma il prezzo e'
un RMSE ankle 4.7x-6.5x peggiore del morning-best nella finestra di push-off.
Le varianti con `J_mid` rendono inoltre il knee molto piu' rumoroso del
morning-best. La variante `p/on=0.1` dimezza circa il chattering knee rispetto
a V3'/V3m, ma non recupera il tracking ankle: il miss zero/polo PI non domina
il fallimento.

### 10.4 TODO residui

- `V3 knee B (Ki_v=6408)` non e' stato testato: V3m e V3' sono gia'
  troppo aggressive sul knee e falliscono comunque per ankle tracking.
  Testarlo ora non cambierebbe la decisione di non promuovere il design.
- La validazione `J_eff` non e' stata ripetuta sui modelli V3/V3' perche'
  questi cambiano solo gain SEA/cascade, non geometria, masse o inerzie del
  modello OpenSim. Va ripetuta solo dopo un aggiornamento del modello fisico.

## 11. Riferimenti

- `reports/user/2026-05-18_motor_driver_cascade_redesign_zeta07.md`
- `reports/user/2026-05-18_run_zeta07_analisi_fallimento_tracking_ankle.md`
- `reports/user/2026-05-19_validazione_inerzia_effettiva_knee_ankle.md`
- `reports/daily/2026-05-18_daily-report.md`
- `validation/effective_joint_inertia.py`
- `validation/cascade_qdot_step_inertia.py`
- `validation/cascade_jeff_todo_runner.py`
- `tools/motor_driver_pole_map.py`
- `tools/motor_driver_pole_locus.py`
