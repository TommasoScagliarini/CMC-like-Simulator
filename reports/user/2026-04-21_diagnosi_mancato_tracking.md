# Diagnosi delle cause del mancato tracking cinematico a gain 100/20

**Data**: 2026-04-21
**Run analizzata**: `results/_codex_full_rk4_backtracking_100_20/` (stabile, gain outer `Kp=100, Kd=20`, RK4 bypass, `T_control=3 ms`)
**Modello**: `models/Adjusted_SEASEA - Copia_tuned.osim`

## Riepilogo numerico

| Metrica | Knee | Ankle |
|---|---:|---:|
| `tau_input_saturated` | 429 / 6801 (6.31%) | **4314 / 6801 (63.43%)** |
| max `|tau_ff_cmd|` | 1328 Nm | 554 Nm |
| max `|outer_pd_cmd|` | **3303 Nm** | **8332 Nm** |
| `|outer_pd|>400 Nm` (80% limite) | 1448 campioni | **5850 campioni** |
| `|tau_ff|>400 Nm` | 183 campioni | 42 campioni |

La richiesta dell'outer PD eccede ripetutamente il limite fisico del motore
(`±500 Nm`). Il feed-forward invece resta quasi sempre dentro il budget.

## C1 — Decomposizione di `tau_input_raw` in finestra satura

Nei campioni in cui `tau_input_saturated = 1`:

### Ankle (4314 campioni saturi)

| Componente | Mediana | p95 |
|---|---:|---:|
| `|tau_ff_cmd|` | **6.8 Nm** | 183 Nm |
| `|outer_pd_cmd|` | **2069 Nm** | 4411 Nm |
| `|inner_prop|` | 2216 Nm | 4948 Nm |
| `|inner_damp|` | 831 Nm | 1847 Nm |
| `|tau_input_raw|` | 1323 Nm | 4583 Nm |

Il feed-forward e' **trascurabile** rispetto all'outer PD (mediana 6.8 Nm
contro 2069 Nm). La saturazione ankle e' dominata quasi interamente dalla
componente outer PD e dai termini proporzionale/damping inner che la
inseguono. **C2 e' scartata come causa primaria sull'ankle: il problema
non e' il feed-forward, e' il PD esterno.**

### Knee (429 campioni saturi)

| Componente | Mediana | p95 |
|---|---:|---:|
| `|tau_ff_cmd|` | 73 Nm | 334 Nm |
| `|outer_pd_cmd|` | 84 Nm | 267 Nm |
| `|inner_prop|` | **568 Nm** | 827 Nm |
| `|inner_damp|` | 160 Nm | 397 Nm |
| `|tau_input_raw|` | 676 Nm | 851 Nm |

Sul knee la situazione e' diversa: ff e outer_pd sono comparabili e
moderati, ma il termine `inner_prop = Kp_inner*(theta_m_ref - theta_m)`
sale a 568 Nm mediano. Questo indica che **il motore sta inseguendo un
riferimento che non riesce a raggiungere** (errore tra angolo motore
desiderato e reale grande) — sintomo di un motore che slippa rispetto al
comando di velocita', cioe' di dinamica SEA non piu' valida come loop
elastico ideale.

## C2 — Feed-forward vs budget motore

Mediana `|tau_ff|` su finestra satura ankle: **6.8 Nm**, max 554 Nm in
tutta la run. Non e' il feed-forward a saturare il motore.

**Conclusione C2**: il feed-forward non richiede azioni di scaling. Lo
scaling del PD protesico (oggi disabilitato,
`enable_sea_feasibility_scaling=False`) sarebbe il meccanismo corretto,
ma attivarlo da solo riduce il PD e **degrada il tracking attivo**: e'
un palliativo, non un fix.

## C3 — Drift stato motore vs giunto (RK4 bypass)

Integrando `motor_speed` per ricostruire `motor_angle`:

| Coord | max |drift| | drift finale | joint range |
|---|---:|---:|---|
| Ankle | 1.13 rad (**64.7 deg**) | -0.07 rad | [-19.96, 13.93] rad |
| Knee | 0.28 rad (16 deg) | 0.04 rad | [-12.68, 13.41] rad |

Il joint range ankle [-20, +14] rad e' **fisicamente assurdo** (la
caviglia fisiologica sta in [-0.5, +0.3] rad): il giunto protesico ha
girato **multiple volte su se stesso**. Il motore e il giunto si
accoppiano tramite una molla torsionale; quando `tau_input` satura
costantemente, il motore non riesce a correggere la posizione del
giunto e la molla non ha piu' un punto di riferimento stabile.

Il drift assoluto motore-giunto e' modesto (<1.2 rad), **ma il problema
vero e' che entrambi divergono insieme** perche' il giunto fisico non
e' piu' vincolato da nessun vincolo cinematico (non c'e' contatto col
terreno nel simulatore corrente). **C3 non e' la causa primaria: e' un
sintomo dell'assenza di contatto/tracking attivo efficace.**

## C4 — Accoppiamento pelvis -> feed-forward protesico

Correlazioni lineari tra `tau_bio` su pelvis e `tau_ff_cmd` protesico:

| Pelvis DOF | corr con Ankle ff | corr con Knee ff |
|---|---:|---:|
| `pelvis_tilt` | -0.235 | **+0.545** |
| `pelvis_list` | +0.074 | +0.063 |
| `pelvis_tx` | -0.178 | +0.050 |
| `pelvis_ty` | +0.024 | +0.016 |

Accoppiamento **moderato sul knee** (r=0.55 con `pelvis_tilt`), basso
sull'ankle. Dato che il feed-forward ankle e' gia' piccolo (mediana 7
Nm nelle finestre sature), **C4 non e' un driver quantitativo
significativo**. La SO ha `feasibility_scale=1.0` ovunque: la
ripartizione bio-protesico via matrice di massa non sta producendo
residui.

## C5 — Banda passante inner vs outer

Formule (con `Jm=0.01 kg·m^2`, `Bm=0.1 N·m·s/rad`):

- `omega_n_inner = sqrt(K*(1+Kp_inner)/Jm)`
- `zeta_inner = (Bm + Kd_inner) / (2*sqrt(Jm*K*(1+Kp_inner)))`

| Sistema | K | Kp | Kd | omega_n | freq | zeta |
|---|---:|---:|---:|---:|---:|---:|
| Knee inner | 1000 | 3.9 | 9.7 | 700 rad/s | 111 Hz | 0.70 |
| Ankle inner | 500 | 8.8 | 9.7 | 700 rad/s | 111 Hz | 0.70 |
| Outer PD | — | 100 | 20 | 10 rad/s | 1.59 Hz | ~1.0 |

**Rapporto omega_n_inner / omega_n_outer = 70** su entrambi i giunti.

La separazione di scala e' abbondante: l'inner loop e' 70x piu' veloce
dell'outer e su questo piano **la cascata e' formalmente corretta**.
**C5 scartata** — il problema non e' la banda passante del PD inner in
se'.

Quindi se l'inner e' veloce e stabile, perche' satura? Perche' **il
target `tau_ref` che l'outer chiede al motore e' fisicamente
irragionevole** (vedi C1): `outer_pd` arriva a 8331 Nm sull'ankle, con
un motore da 500 Nm. L'inner loop non ha nulla da inseguire di
raggiungibile.

## Ranking delle cause

| # | Causa | Peso stimato | Evidenza |
|---|---|---|---|
| 1 | **Outer PD chiede coppie oltre il budget motore (ankle)** | **ALTO — primario** | outer_pd mediana 2069 Nm, max 8331 Nm vs limite 500 Nm; 86% dei campioni ha outer_pd > 400 Nm |
| 2 | **Motore knee insegue un riferimento irraggiungibile (inner_prop saturo)** | ALTO — secondario | inner_prop mediana 568 Nm su finestra satura knee |
| 3 | Assenza di vincolo cinematico efficace sul giunto protesico | ALTO — concorrente | `pros_ankle_angle` arriva a ±20 rad — il simulatore non ha contatto con ground |
| 4 | Feed-forward non vincolato (C2) | BASSO | ff mediana 7 Nm in finestra satura ankle |
| 5 | Accoppiamento M pelvis-protesi (C4) | BASSO | r=0.55 solo su knee-pelvis_tilt; ff knee piccolo |
| 6 | Drift RK4 (C3) | BASSO | drift <1.2 rad ma giunto stesso diverge per altri motivi |
| 7 | Banda passante inner (C5) | NULLO | ratio 70x gia' adeguato |

## Causa primaria identificata

**L'outer PD con `Kp=100, Kd=20` produce richieste di coppia ben oltre
la capacita' fisica del motore SEA**, in particolare sull'ankle dove
`|outer_pd_cmd|` supera 400 Nm nell'86% dei campioni con un motore da
500 Nm (saturazione a `tau_input`). La causa e' la combinazione di:

1. errore cinematico istantaneo amplificato da `Kp=100`,
2. derivata di errore amplificata da `Kd=20`,
3. assenza di contatto col terreno che renda fisicamente stabile la
   posizione del giunto protesico (anche questo non e' un bug del
   controllore SEA, e' una proprieta' del setup di simulazione),
4. mancanza di qualunque meccanismo che limiti la richiesta *prima* del
   clipping (ne' rate limiter sull'outer, ne' feasibility scaling
   attivo, ne' saturation anti-windup).

## Raccomandazione per la fase di fix

Interventi da valutare in ordine di priorita'.

### P1 — Introdurre feasibility scaling del PD outer saturation-aware

Attivare `enable_sea_feasibility_scaling=True` in
[config.py](../../config.py) e **estenderne la logica**: oggi in
[prosthesis_controller.py](../../prosthesis_controller.py) scala solo
la componente PD e solo se riesce a predire la saturazione. La
modifica minima:

- prima del clipping, calcolare `tau_headroom = tau_input_max -
  |tau_ff + inner_terms_previsti|`;
- se `|Kp*e_q + Kd*e_qdot| > tau_headroom`, scalare la componente PD
  in modo che `|tau_input_raw| <= tau_input_max * safety` (safety ~
  0.9).

Questo sopprime la saturazione cronica senza ridurre la coppia
feed-forward. Non e' un palliativo: e' la forma di anti-windup standard
per un attuatore con limite di coppia.

### P2 — Verificare la presenza/assenza di contatto col terreno

Il range `pros_ankle_angle` [-20, +14] rad dice che **nulla limita la
rotazione del piede protesico**. Se il modello non ha
`ContactGeometry`/`HuntCrossleyForce` attivi sotto la protesi, nessun
tuning del PD potra' produrre un tracking sensato: il giunto e' libero
di ruotare senza resistenza fisica. Da controllare
nell'`.osim` prima di qualsiasi tuning.

### P3 — Ridurre `Kp` outer per la sola ankle protesica

Se il vincolo del progetto e' mantenere `sea_kp/sea_kd = 100/20` solo
sui DOF biologici, considerare gain piu' bassi sui DOF protesici (es:
`pros_ankle: Kp=40, Kd=8`). Anche con inner SEA a 111 Hz, `Kp=100`
chiede errori zero in tempo 0.1 s su una protesi libera: la richiesta
e' fisicamente inoperabile.

### P4 — Non intervenire su C5 / banda inner

Gli sweep di omega_n/zeta riportati in
[tracking_cmc_like_100_20.md](2026-04-20_tracking_cmc_like_100_20.md)
non troveranno soluzioni: il collo di bottiglia non e' la banda inner
ma la richiesta dell'outer.

## Verification

La diagnosi e' coerente perche':

1. La decomposizione di `tau_input_raw` mostra che il feed-forward e'
   trascurabile nelle finestre sature (mediana 6.8 Nm ankle): questo
   esclude C2 come primaria.
2. Il rapporto `omega_n_inner/omega_n_outer = 70` smentisce C5.
3. La correlazione pelvis-ff knee 0.55, ma con ff knee piccolo in
   saturazione, ridimensiona C4.
4. Il drift motore-giunto <1.2 rad non spiega `pros_ankle_angle` a 20
   rad: C3 e' sintomo, non causa.
5. L'unica componente che supera sistematicamente il budget motore e'
   `outer_pd_cmd` (mediana 2069 Nm ankle, 86% dei campioni > 400 Nm).

Il fix P1 (feasibility scaling saturation-aware) e' il prossimo passo
concreto. P2 (contatto col terreno) deve essere verificato prima di
qualunque tuning, altrimenti tutti gli interventi sono a vuoto.
