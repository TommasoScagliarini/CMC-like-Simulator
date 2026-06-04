# Saturazione del knee nell'env RL: limit-cycle in cascade e rimedi

Data: 2026-06-01

## Problema

Nei rollout dell'ambiente RL di traiettoria (`Trajectory Generator/osim_trj_cmc_like.py`)
sul setup AB06 PI, il comando normalizzato `u` del SEA **knee** satura
(`|u| >= 0.999`) nella grande maggioranza degli step (21/30 nel rollout 0.30 s
documentato il 2026-05-31), mentre l'**ankle** non satura mai. Con i dati
cinematici sperimentali (IK reale) la saturazione non si verifica.

Domanda di partenza: perche' il knee satura cosi' tanto solo con i riferimenti
generati dalla policy e non con i dati sperimentali?

## Analisi della causa

Il setup PI gira in modalita' **cascade** (`sea_outer_controller_mode = "cascade"`):

    qdot_cas = qdot_ref + Kp_outer * e_q
    e_v      = qdot_cas - qdot_cur
    tau_cmd  = Kp_inner * e_v + Ki_inner * integral(e_v)
    u        = clip(tau_cmd / F_opt, -1, +1)

Guadagni e `F_opt` (da `config.py` e dal modello `AB06_SEASEA_stiff321_500_pi.osim`):

| | F_opt | Kp_inner | sensibilita' u per rad/s di e_v |
|---|---|---|---|
| knee  | 100 | 29.20  | 29.2/100 = **0.292** |
| ankle | 250 | 2.8275 | 2.83/250 = **0.011** |

Il comando knee e' **~26x piu' sensibile** all'errore di velocita' dell'ankle:
bastano ~3.4 rad/s di `e_v` per saturare il knee, all'ankle ne servirebbero ~88.

Verifica: il riferimento **non** e' a gradini. In
`ProstheticSegmentKinematics`, ogni nuovo segmento e' ancorato con valore e
derivata del segmento precedente (`anchor` + `anchor_derivative`), quindi la
spline Hermite e' C1-continua tra i segmenti. La discontinuita' c'e' solo sulla
derivata seconda (`qddot_ref`), che pero' il controllore cascade non usa.

La causa reale e' un **limit-cycle in saturazione** del knee, confermato a 1 ms:

- l'oscillazione del comando knee e' a **~35 Hz** (picco FFT) / ~40 Hz
  (sign-flip rate), in piena banda del problema storico "notch a 28 Hz sul
  feedback knee";
- la velocita' di giunto reale `qdot_cur` oscilla fino a **~26 rad/s** cambiando
  segno al rate di controllo;
- il termine che satura e' il **proporzionale interno** `Kp_inner * e_v`
  (contributo a `u` fino a ~9.9, cioe' 10x oltre il clamp), **non** il windup
  integrale (contributo limitato a ~0.31 dal `sea_cascade_inner_i_torque_limit`);
- il termine dominante di `e_v` e' `qdot_ref - qdot_cur`, ed e' grande
  soprattutto perche' `qdot_cur` stesso oscilla: il fenomeno e' in larga parte
  auto-sostenuto dal loop.

Con i dati sperimentali `qdot_cur ~= qdot_ref` (i guadagni cascade sono tarati su
quella traiettoria, morning-best 18/05), `e_v ~= 0`, e il loop resta nel regime
stabile. La policy RL, perturbando il riferimento fuori da quella traiettoria,
eccita la risonanza ~35 Hz del knee.

Il picco di `qdot_ref` iniettato scala come `amplitude * max_delta_rad / segment_duration`.

## Soluzione / strategia testata

Sono stati testati due rimedi che **non** modificano la semantica del comando
SEA ne' il plugin C++.

### Rimedio 2 - ridurre l'eccitazione del riferimento (env-side)

Sweep su `segment_duration` e `max_delta_rad` del knee (episodio 0.20 s,
azione sinusoidale scriptata, ampiezza 0.10, 1 ms):

    segment_duration (delta_knee=0.35):  sat      max|u|   max|qdot_cur|   R
      seg=0.01                           83.0%    1.000      26.22        0.71
      seg=0.02                           51.5%    1.000      10.45        0.86
      seg=0.05                            0.0%    0.797       2.83        0.96

    max_delta_knee (seg=0.01):
      0.35                               83.0%    1.000      26.22        0.71
      0.15                               66.5%    1.000      12.78        0.75
      0.05                               21.5%    1.000       4.85        0.88
      0.02                                0.0%    0.806       2.15        0.94

Cura il problema **alla radice**: spariscono sia la saturazione del comando sia
l'oscillazione meccanica (`qdot_cur` torna a 2-3 rad/s, fisiologico), e il reward
sale a 0.94-0.96.

### Rimedio 3 - filtro sul comando knee (controller-side, knob esistente)

Sweep sul LPF del comando `u` knee (`sea_u_lpf_cutoff_hz`, di default 0/spento),
nel caso peggiore seg=0.01, delta=0.35:

    LPF knee:        sat      max|u|   max|qdot_cur|   R
      off            83.0%    1.000      26.22        0.71
      80 Hz           0.5%    0.999      26.57        0.73
      50 Hz           0.0%    0.989      26.47        0.73
      30 Hz           0.0%    0.936      26.25        0.74
      20 Hz           0.0%    0.842      25.86        0.75
      10 Hz           0.0%    0.632      24.94        0.75

Azzera la **saturazione del comando** in modo molto efficace (basta 80 Hz), ma
`qdot_cur` resta **~26 rad/s** a ogni cutoff: il filtro maschera il sintomo nel
comando senza stabilizzare il giunto. Tratta il sintomo, non l'instabilita'
meccanica, e aggiunge ritardo di fase.

### Confronto a parita' di condizioni (seg=0.01, 20 step)

| | sat | max\|qdot_cur\| | R |
|---|---|---|---|
| Rimedio 2 (delta_knee=0.02) | 0% | 2.15  | 0.94 |
| Rimedio 3 (LPF 10 Hz)       | 0% | 24.94 | 0.75 |

Il rimedio 2 domina: stessa eliminazione della saturazione, giunto stabile e
reward molto migliore. E' anche gratis per il training, perche' sono parametri
dell'env e non del controllore validato.

## Approfondimento (b): l'oscillazione residua sotto LPF e' instabilita' whole-body

Domanda: perche' `qdot_cur` del knee resta ~26 rad/s anche col LPF, quando `u`
non satura piu'? Serie temporali allineate a 1 ms (`qdot_knee`, `u_knee`,
`|u_res|` max sui reserve, `pelvis_ty`):

| config | \|u_res\| picco | pelvis_ty range | qd_knee picco | qd sign-flip >30ms | u_knee picco |
|---|---|---|---|---|---|
| LPF 50 Hz, seg=0.01 | 46.7 | 47 mm | 26.5 @195 ms | 11 | 0.99 |
| LPF 10 Hz, seg=0.01 | 46.8 | 41 mm | 24.9 @196 ms | 15 | 0.63 |
| seg=0.05 (rimedio 2) | 2.0 | 1 mm | 2.8 @139 ms | 3 | 0.80 |

Tre conclusioni:

1. **Non e' un transitorio di init**: il picco di `qdot_knee` e' a ~195 ms (fine
   episodio) e i cambi di segno sono distribuiti su tutto l'episodio (oscillazione
   sostenuta).
2. **Non e' guidata dal comando SEA**: passando il LPF da 50 a 10 Hz il picco di
   `u` scende da 0.99 a 0.63, ma `qdot_knee` resta ~25 rad/s. Il knee oscilla
   anche quando il motore non lo sta spingendo.
3. **E' instabilita' di tutto il modello**: con seg=0.01 i reserve actuator
   saturano a `|u_res| ~= 46` (46x la forza nominale), il bacino rimbalza di
   ~47 mm e lo scheletro intero vibra; il knee protesico viene trascinato. Con
   seg=0.05 i reserve restano a ~2, il bacino e' fermo (~1 mm) e il knee e'
   liscio.

Origine confermata gia' al primo step (t=11.990, stesso init biologico per tutti):

    seg=0.01: muscle_share=0.10  capable_share=0.19  |u_res|=7.1
    seg=0.05: muscle_share=0.36  capable_share=1.00  |u_res|=0.8

A parita' di stato e riferimento biologico, cambia solo il segmento protesico:
con seg corto il riferimento protesico ha `qddot_ref` enorme (scala come
`1/seg^2`, ~350 rad/s^2 a seg=0.01 contro ~14 a seg=0.05). Questa accelerazione
di riferimento si accoppia, tramite la matrice di massa, nell'inverse dynamics
biologico: il recruitment muscolare diventa infeasible (`capable_share`
1.00 -> 0.19), i reserve esplodono, lo scheletro vibra e il knee protesico
oscilla di conseguenza.

Implicazione: il LPF sul comando (rimedio 3) e' cosmetico perche' il knee non e'
mosso dal comando ma dal thrashing whole-body; il rimedio 2 e' risolutivo perche'
abbatte la `qddot_ref` protesica che innesca il blow-up dei reserve. Questo e' lo
stesso fenomeno del warning static optimization ("QP did not converge") segnalato
nel daily 2026-05-29 e del coupling knee-ankle dei TODO storici.

## Raccomandazione operativa

Adottata la soluzione filtro 6 Hz lato simulatore (sezione "Soluzione finale"):
la policy genera un riferimento smooth con full authority e il sim lo band-limita
come l'IK. Per il training PPO non serve piu' vincolare `segment_duration` o
`max_delta_rad` per la stabilita': il filtro garantisce trackability a qualunque
combinazione. `segment_duration`/`policy_knots` restano liberi per la cadenza e
l'espressivita' della policy.

Nota: la modifica e' tutta lato env (`ProstheticSegmentKinematics`); il
controllore SEA e la config cascade validata (morning-best 18/05) restano
invariati. Il LPF sul *comando* (`sea_u_lpf_cutoff_hz`, rimedio 3) resta spento:
non promosso.

## Applicazione di remedy 2 (eseguita)

Verifica al **policy_knots=4** reale (il default di `CMCEnvConfig`; lo sweep
sopra usava policy_knots=1, che sottostimava il problema):

    policy_knots=4, episodio 0.20 s:
      seg=0.05 delta_knee=0.35 (default precedente)  sat=48.0%  max|qd|=16.60  |u_res|=22  R=0.87
      seg=0.05 delta_knee=0.05 (applicato)           sat= 1.0%  max|qd|= 3.04  |u_res|= 4  R=0.98
      seg=0.05 delta_knee=0.10                        sat=12.0%  max|qd|= 5.58  |u_res|= 6  R=0.96
      seg=0.01 delta_knee=0.05                        sat=69.5%  max|qd|=17.21  |u_res|=127 R=0.71
      seg=0.01 delta_knee=0.35                        DIVERGE (qd~1e7, u_res->1000, truncated)

Con 4 knot il default precedente (delta 0.35) saturava il 48%, e a seg corto
divergeva: il problema era reale anche al default. Un cap asimmetrico sul knee
(delta 0.05) lo riduceva a ~1%, ma a costo di authority e senza azzerarlo del
tutto. Questo cap e' stato un passo interim, poi **superato** dalla soluzione
finale qui sotto.

## Soluzione finale: filtro di riferimento 6 Hz lato simulatore

Separazione dei ruoli (su indicazione esplicita): la **rete** genera solo un
riferimento smooth (i knot); il **simulatore** lo gestisce come un qualsiasi
segnale cinematico e lo band-limita allo stesso cutoff dell'IK sperimentale
(`kinematics_lowpass_cutoff_hz`, 6 Hz) prima che il controllore protesico lo
insegua. Cosi' il riferimento generato ha lo stesso contenuto spettrale dell'IK
reale e non eccita la risonanza knee a ~35 Hz.

Implementato in `ProstheticSegmentKinematics` come reference-model del 2 ordine:

    qf_ddot = wn^2 (q_target - qf) - 2*zeta*wn*qf_dot,   wn = 2*pi*6 Hz, zeta = 1

integrato lungo il segmento a partire dallo stato del filtro (continuo tra
segmenti); la spline servita usa (qf, qf_dot) filtrati, quindi q/qdot/qddot
restano mutuamente consistenti e band-limited. La rete mantiene **full
authority** (`max_delta_rad` ripristinato a 0.35); e' il sim a garantire la
trackability.

Risultati (episodio 0.30 s, azione sinusoidale 0.10):

    config                          sat   max|u|  max|qd|    |u_res|   R
    k4 seg0.05 d0.35  filtro OFF     59%   1.000    17.68     22.79    0.84
    k4 seg0.05 d0.35  filtro ON       0%   0.460     1.34      1.82    0.99   <- default
    k4 seg0.01 d0.35  filtro OFF      81%   1.000   1.6e7      1000     diverge
    k4 seg0.01 d0.35  filtro ON        0%   0.325     1.21      2.46    0.99
    k1 seg0.05 d0.35  filtro ON        0%   0.437     1.38      1.91    0.99

Il filtro porta la saturazione a **0% a full authority**, tiene il giunto a
~1.3 rad/s e i reserve a ~2x, e **salva persino il caso seg=0.01** che prima
divergeva. L'env e' ora robusto a qualsiasi `policy_knots`/`segment_duration`.

Nuovi parametri in `CMCEnvConfig` (default sicuro, retro-compatibile):

    enable_pros_ref_lpf: bool = True
    pros_ref_lpf_cutoff_hz: Optional[float] = None   # None -> cutoff IK (6 Hz)
    pros_ref_lpf_zeta: float = 1.0

## File modificati / creati

- `validation/rl_env_knee_sat_diag.py` (nuovo): decomposizione per-step del
  comando cascade knee/ankle (`qdot_ref`, `qdot_cur`, `e_q`, `e_v`, contributi P
  e I, `u`).
- `validation/rl_env_knee_fix_sweep.py` (nuovo): caratterizzazione frequenza a
  1 ms + sweep rimedio 2 (`segment_duration`, `max_delta_rad` knee) e rimedio 3
  (LPF comando knee).
- `Trajectory Generator/osim_trj_cmc_like.py` (modificato): filtro di
  riferimento 6 Hz lato simulatore in `ProstheticSegmentKinematics`
  (reference-model 2 ordine, stato continuo tra segmenti); nuovi campi
  `enable_pros_ref_lpf` / `pros_ref_lpf_cutoff_hz` / `pros_ref_lpf_zeta` in
  `CMCEnvConfig`; wiring in `_build_simulator` (cutoff = quello dell'IK).
  `max_delta_rad` ripristinato a 0.35 (full authority). Vedi "Soluzione finale".

Nessuna modifica a `prosthesis_controller.py`, `config.py`, al plugin C++ o ai
modelli: la semantica del comando SEA e la config cascade validata restano
invariate.

## Test / verifiche eseguite

Ambiente `envCMC-like` su macOS arm64, setup
`models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`.

    conda run --no-capture-output -n envCMC-like python validation/rl_env_knee_sat_diag.py
    conda run --no-capture-output -n envCMC-like python validation/rl_env_knee_fix_sweep.py

Esiti: il diagnostico riproduce 21/30 step saturati (coerente col daily
2026-05-31); lo sweep conferma i numeri delle tabelle sopra. Entrambi
deterministici (random_init=False, seed fissi, azioni scriptate).

Regression dopo l'introduzione del filtro 6 Hz (default ON):

- `validation/rl_env_smoke_ab06_pi.py` -> `rl_env_smoke_ab06_pi_ok`.
- `validation/rl_env_rollout_ab06_pi.py --skip-cost`:
  - Test L (rollout 0.30 s, seg=0.01, knots=1): terminated, 0 nonfinite,
    `saturated steps pros_knee_angle = 0/30` (prima 21/30), `max|u| knee = 0.194`,
    reward [0.91, 0.999]. PASS.
  - Test R (determinismo): reuse e fresh bitwise identici (d_reward=0,
    d_obs=0) -> il filtro non rompe la riproducibilita' (stato azzerato al
    reset). PASS.
- Default puro `CMCEnvConfig()`: `max_delta_rad=0.35`, `enable_pros_ref_lpf=True`,
  `pros_ref_lpf_cutoff_hz=None` (-> 6 Hz IK), `pros_ref_lpf_zeta=1.0`.

## TODO chiusi oggi

- [b] CHIUSO. L'oscillazione residua del knee sotto LPF e' instabilita'
  whole-body sostenuta (non transitorio, non comando-driven): con seg corto la
  `qddot_ref` protesica (~1/seg^2) rende infeasible il recruitment biologico, i
  reserve saturano (~46x), lo scheletro vibra e il knee viene trascinato. Vedi
  sezione "Approfondimento (b)".

## TODO aperti

- Validare il filtro 6 Hz con una policy reale (azioni piu' ampie/varie della
  sinusoide 0.10) e su rollout lunghi (gait completo), non solo nei test brevi.
- Eventuale taratura di `pros_ref_lpf_cutoff_hz`/`pros_ref_lpf_zeta` se la policy
  necessita di piu' banda: trade-off banda-riferimento vs trackability/lag.
- Verificare l'effetto del lag causale del filtro sul tracking dell'IK (l'IK
  sperimentale e' filtrato a fase-zero, il riferimento online no): valutare se
  serve un piccolo lead/anticipo lato policy o reward.
- Valutare i target/bounds biologici della static optimization alla partenza
  AB06: anche con riferimento protesico dolce restano `capable_share < 1` e
  warning QP nei primi step (linea gia' aperta dal daily 2026-05-29).
