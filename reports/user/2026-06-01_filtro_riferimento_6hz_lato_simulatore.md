# Filtro di riferimento 6 Hz lato simulatore per l'env RL di traiettoria

Data: 2026-06-01

> Report di implementazione. La diagnosi completa del limit-cycle knee e' in
> `reports/user/2026-06-01_knee_saturazione_env_rl_limit_cycle.md`; qui si
> documenta solo la soluzione adottata e l'architettura dei filtri.

## Problema

Nell'env RL di traiettoria il riferimento protesico generato dalla policy
**bypassa** il filtro low-pass che il simulatore applica all'IK sperimentale,
perche' le coordinate protesiche vengono sovrascritte in
`ProstheticSegmentKinematics.get()`. Con un riferimento a banda larga
(es. `policy_knots=4`/`segment_duration=0.05` ~ 40 Hz) il knee SEA in cascade
entra in un limit-cycle a ~35 Hz: comando `u` in saturazione bang-bang,
`qddot_ref` enorme che accoppia nell'inverse dynamics biologico, reserve a ~46x
e modello che vibra. I dati sperimentali non saturano perche' l'IK e' gia'
filtrata a 6 Hz.

## Soluzione e strategia

Separazione dei ruoli (indicazione esplicita dell'utente):

- la **rete** genera solo un riferimento smooth (i knot di traiettoria);
- il **simulatore** lo gestisce come un qualsiasi segnale cinematico e lo
  **band-limita allo stesso cutoff dell'IK (6 Hz)** prima che il controllore
  protesico lo insegua.

Cosi' il riferimento generato ha lo stesso contenuto spettrale dell'IK reale e
non eccita la risonanza knee a ~35 Hz, indipendentemente da quanto e'
"aggressiva" la policy.

## Architettura: due filtri distinti, stesso cutoff

Importante: **non** e' stato riusato il filtro IK esistente; ne e' stato aggiunto
uno **nuovo e separato**, tarato sullo stesso cutoff.

### Filtro #1 - IK sperimentale (preesistente, invariato)

- Dove: `KinematicsInterpolator._lowpass_and_resample()` in
  `kinematics_interpolator.py`.
- Segnale: IK sperimentale dal `.mot` (tutte le coordinate).
- Tipo: Butterworth ordine 4, **zero-phase** (`sosfiltfilt`), **offline** al
  caricamento.
- Config: `kinematics_lowpass_cutoff_hz=6.0`, `kinematics_lowpass_order=4`.

### Filtro #2 - riferimento protesico della policy (nuovo)

- Dove: `ProstheticSegmentKinematics` in
  `Trajectory Generator/osim_trj_cmc_like.py`.
- Segnale: riferimento generato dalla policy, **solo** coordinate protesiche
  (`pros_knee_angle`, `pros_ankle_angle`).
- Tipo: reference-model del 2 ordine, **causale**, **online** (ad ogni segmento),
  stato continuo tra segmenti:

      qf_ddot = wn^2 (q_target - qf) - 2*zeta*wn*qf_dot,   wn = 2*pi*6 Hz, zeta = 1

  integrato a partire dallo stato del filtro; la spline servita usa i campioni
  filtrati `(qf, qf_dot)`, quindi `q/qdot/qddot` restano consistenti e
  band-limited.
- Config: `pros_ref_lpf_cutoff_hz=None` -> riusa `kinematics_lowpass_cutoff_hz`
  (6 Hz).

### Flusso dei dati

    coord biologiche  -> KinInterp.get()  -> [filtro #1: IK offline 6 Hz]  -> q_ref
    coord protesiche  -> ProstheticSegmentKinematics.get():
        se segmento policy attivo -> [filtro #2: nuovo, online 6 Hz] -> q_ref protesico
        altrimenti (pre-azione)   -> delega a KinInterp (IK, filtro #1)

Il filtro #1 non poteva essere riusato: e' offline/zero-phase (filtra tutto il
segnale in un colpo) e il riferimento policy non transita da
`KinematicsInterpolator`. Serviva un filtro causale online separato.

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`:
  - `ProstheticSegmentKinematics`: aggiunto il reference-model 2 ordine (stato
    `qf/qf_dot` continuo tra segmenti, reset in `clear_segment`), `set_segment`
    integra il filtro e serve la spline filtrata, `get()` invariato.
  - `CMCEnvConfig`: nuovi campi `enable_pros_ref_lpf=True`,
    `pros_ref_lpf_cutoff_hz=None`, `pros_ref_lpf_zeta=1.0`; `max_delta_rad`
    ripristinato a `0.35` (full authority).
  - `_build_simulator`: wiring del filtro, cutoff di default = quello dell'IK.

Nessuna modifica a `prosthesis_controller.py`, `config.py`, al plugin C++ o ai
modelli. Il controllore SEA e la config cascade validata (morning-best 18/05)
restano invariati.

## Test / verifiche

Ambiente `envCMC-like` su macOS arm64, setup
`models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`.

Effetto del filtro (episodio 0.30 s, azione sinusoidale 0.10):

    config                          sat   max|u|  max|qd|    |u_res|   R
    k4 seg0.05 d0.35  filtro OFF     59%   1.000    17.68     22.79    0.84
    k4 seg0.05 d0.35  filtro ON       0%   0.460     1.34      1.82    0.99   <- default
    k4 seg0.01 d0.35  filtro OFF      81%   1.000    1.6e7     1000     diverge
    k4 seg0.01 d0.35  filtro ON        0%   0.325     1.21      2.46    0.99
    k1 seg0.05 d0.35  filtro ON        0%   0.437     1.38      1.91    0.99

Regression (default filtro ON):

- `validation/rl_env_smoke_ab06_pi.py` -> `rl_env_smoke_ab06_pi_ok`.
- `validation/rl_env_rollout_ab06_pi.py --skip-cost`:
  - Test L (seg=0.01, knots=1): terminated, 0 nonfinite,
    `saturated steps pros_knee_angle = 0/30` (prima 21/30),
    `max|u| knee = 0.194`, reward [0.91, 0.999]. PASS.
  - Test R (determinismo): reuse e fresh bitwise identici (d_reward=0, d_obs=0)
    -> il filtro non rompe la riproducibilita' (stato azzerato al reset). PASS.
- `py_compile` su `osim_trj_cmc_like.py` OK.

Esito: 0% saturazione a full authority, robusto a qualsiasi
`policy_knots`/`segment_duration` (salva anche il caso seg=0.01 che prima
divergeva), giunto ~1.3 rad/s, reserve ~2x, reward ~0.99, determinismo
preservato.

## TODO aperti

- Validare il filtro con una policy reale (azioni piu' ampie/varie della
  sinusoide 0.10) e su rollout lunghi (gait completo), non solo nei test brevi.
- Verificare l'effetto del **lag causale** del filtro #2 sul tracking dell'IK:
  l'IK sperimentale e' filtrata a fase-zero, il riferimento online no. Valutare
  se serve un piccolo lead/anticipo lato policy o un termine di reward.
- Eventuale taratura di `pros_ref_lpf_cutoff_hz`/`pros_ref_lpf_zeta` se la policy
  necessita di piu' banda: trade-off banda vs trackability.
- Static optimization all'init AB06: anche con riferimento dolce restano
  `capable_share < 1` e warning QP nei primi step (linea aperta dal 2026-05-29).
