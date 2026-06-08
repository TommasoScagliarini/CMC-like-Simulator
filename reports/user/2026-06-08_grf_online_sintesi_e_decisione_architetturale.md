# GRF online: sintesi della sessione e decisione architetturale (pura → ibrida)

Data: 2026-06-08

Questo report consolida la sessione di lavoro sulla GRF online per il generatore
di traiettorie protesiche, dalla validazione gait-scale alla scelta
dell'architettura ibrida. Dettagli tecnici nei due report collegati:
[validazione gait-scale](2026-06-08_validazione_gait_scale_grf_online.md) e
[GRF ibrida protesica](2026-06-08_grf_ibrida_protesica.md).

## Problema

Per generare traiettorie protesiche ex-novo, la dinamica non può essere guidata
dalle GRF `prescribed` (time-driven dalla force-plate): diventano fisicamente
sbagliate appena la policy devia dalla IK. Serviva una GRF `online`
(contatto piede-terreno) capace di guidare la dinamica.

## Cosa è stato dimostrato (diagnosi)

La modalità `online` pura, validata su finestra gait-scale (2 s, non più 500 ms),
**non può** riprodurre il timing/forma del wrench prescribed in dinamica forward:

- correggendo l'impulso verticale da 0.83 a 0.94 (magnitudo quasi perfetta), le
  reserve del bacino sono scese solo da 11.1x a 10.3x;
- le reserve seguono il **deficit istantaneo** di forza (corr 0.999);
- un contatto memoryless state-based, anche con residuale full-wrench, può
  scalare la forza ma non rifasarla.

Conclusione: limite **strutturale** del modello di contatto, non di calibrazione.
Inseguire il prescribed è infeasible.

## Decisione architetturale

Concetti chiave emersi nella discussione:

- **Wrench** = forza 3D + momento 3D (equivalente: forza + COP + coppia libera).
  Eguagliare il wrench significa azzeccare tutte e sei le componenti nel tempo.
- **Reserve**: attuatori virtuali sui DOF non azionati del bacino (floating
  base). Sono alte quando manca il sostegno GRF; **non sono penalizzabili** nella
  reward, perché non controllabili dal policy.
- **Il timing del wrench è bloccante solo se la GRF viene APPLICATA per
  sostituire il prescribed.** Per i gait-event (primo contatto, geometrico) è
  irrilevante.
- **L'ankle senza GRF è un grado di libertà libero**: senza reazione al suolo
  l'angolo di caviglia non ha conseguenza dinamica; per ex-novo degenera.

Da qui la scelta: **architettura ibrida**

```
lato sano (destra)     -> prescribed GRF (insegue la IK, valido)
lato protesico (sin.)  -> GRF online APPLICATA (terreno reale per ankle/knee)
```

Non si insegue più il prescribed: serve solo un sostegno fisico plausibile.

## Soluzione implementata (solo Python; nessuna modifica a SEA o plugin C++)

Applicazione della GRF online **per-lato** (il plugin supportava già
`appliesForce` per istanza):

- `config.py`: `online_grf_applied_sides`;
- `online_grf.py`: `add_online_grf_forces(apply_sides=...)` per-sfera;
- `model_loader.py`: validazione, auto-disabilitazione prescribed sui lati
  applicati, propagazione nel context;
- `simulation_runner.py`: log in `run_status`;
- `main.py`: `--online-grf-applied-side`, `--disable-prescribed-grf-side`.

Generatore di contatto **fisico** (non fit al prescribed) con suola a fila e
**rocker**:

- `validation/generate_online_grf_row_profile.py` (marker tallone/punta →
  fila/rocker, materiale fisico, auto-fit ground);
- `validation/check_online_grf_cop_rollover.py` (verifica read-only del COP).

Lato protesico AB06 = sinistra (`foot_l`).

## Risultati

### Track A — ibrido col contatto esistente (v2): VALIDATO

Rollout 1 s, ibrido vs baseline scarico (lato protesico):

| Metrica | Scarico | Ibrido |
|---|---:|---:|
| Caviglia SEA tau media/max | 1.1 / 2.4 Nm | **30.6 / 53.9 Nm** |
| Ginocchio SEA tau media/max | 2.3 / 4.3 Nm | **38.4 / 111.9 Nm** |
| reserve `pelvis_ty` media | 512 N | **64 N** |

La caviglia passa da scarica a caricata (≈ valore di riferimento bilaterale
30.8 Nm); la reserve del bacino crolla 8x. La caviglia non è più un DOF libero.

### Track B — fila fisica e rocker per il COP

| Profilo (forward 1 s) | contact | picco Fy | picco pen | COP range | ankle max | reserve |
|---|---:|---:|---:|---:|---:|---:|
| v2 calibrato | 1.00 | 709 N | 17.5 mm | 6.6 cm | 53.9 Nm | **64 N** |
| flat row | 0.35 | 539 N | 4.9 mm | 1.1 cm | 39.3 Nm | 443 N |
| rocker k=1e6 | 0.50 | 2412 N | 8.5 mm | 2.4 cm | 267 Nm | 387 N |
| rocker soft k=2e5 | 0.79 | 1928 N | 15.8 mm | 1.6 cm | 111 Nm | 195 N |

La capacità rocker è implementata e parametrica, ma un rocker fisico che regga
con continuità, rotoli il COP e non produca spike richiede un tuning
multi-parametro non convergente in poche iterazioni. In più, il rollover non
emerge in regime CMC-tracking (la caviglia insegue la IK e beccheggia poco):
dovrebbe emergere con la policy RL che controlla attivamente la caviglia. Per
carico e reserve **il profilo v2 nell'ibrido resta oggi la scelta migliore**.

## File modificati / aggiunti

```text
config.py
online_grf.py
model_loader.py
simulation_runner.py
main.py
validation/generate_online_grf_row_profile.py            (nuovo)
validation/check_online_grf_cop_rollover.py              (nuovo)
online_grf_profiles/AB06_..._online_row.json             (nuovo)
online_grf_profiles/AB06_..._online_rocker.json          (nuovo)
online_grf_profiles/AB06_..._online_rocker_soft.json     (nuovo)
online_grf_profiles/AB06_..._online_gait2s_basis.json    (nuovo, scartato)
online_grf_profiles/AB06_..._online_gait2s_residual.json (nuovo)
```

## Test e verifiche

- `py_compile` di tutti i file modificati: PASS;
- CLI ibrida presente; smoke ibrido 0.1 s: right applied / left skipped / 8 online
  applied left;
- diagnosi gait-scale: trio forward 2 s `status=complete`, residuale gait-scale
  impulso 0.83→0.94 ma reserve 11.1x→10.3x;
- Track A 1 s: caviglia 1.1→30.6 Nm, reserve 512→64 N, `status=complete`;
- Track B: fila e rocker generati (auto-fit ground), forward 1 s tutti
  `status=complete`; confronto carico/COP/penetrazione/reserve;
- retrocompatibilità modi prescribed/online_sensor/online: preservata.

## Integrazione hybrid nel training RL (IMPLEMENTATA e validata)

Decisioni utente recepite: profilo **v2**; **niente penalty reserve**; penalty
leggera + terminazione su penetrazione del piede protesico; reserve del lato sano
ignorate; osservazione GRF attiva.

Implementazione (dentro `Trajectory Generator/`):

- `osim_trj_cmc_like.py`: `EnvConfig.online_grf_applied_sides` + soglie
  penetrazione (`grf_penetration_penalty_threshold_m=0.020`,
  `grf_penetration_termination_m=0.028`, `reward_grf_penetration_weight=5.0`);
  propagazione a `SimulatorConfig`; penalty graduata oltre 20 mm + terminazione
  `grf_penetration` oltre 28 mm (sotto il guard runner a 30 mm); reserve non
  penalizzate; `online_grf_applied_sides` nei payload info.
- `train_ppo_mlp.py`, `rollout_eval.py`: flag `--online-grf-applied-side`,
  wiring env_config + summary, **default profilo = v2**.
- `env_factory.py`: default profilo = v2.
- `commands.txt`: comando training/rollout ibrido.

Verifiche: `py_compile` PASS; smoke env ibrido (piede protesico caricato ~420 N,
penetrazione 9.5 mm → `pen_loss=0`, nessuna terminazione spuria, 20 contatti);
tiny training RLlib end-to-end exit 0, summary con `grf_mode=online_sensor`,
profilo v2, `online_grf_applied_sides=['left']`, osservazione attiva.

Comando hybrid (50 iter):

```text
train_ppo_mlp.py --grf-mode online_sensor --online-grf-applied-side left
  --episode-duration 2.0 --segment-duration 0.01 ...
```

## TODO

- [ ] Lanciare un training ibrido reale (50+ iter, episodi gait-scale) e
  analizzare apprendimento, terminazioni `grf_penetration`, carico caviglia.
- [ ] Tarare le soglie di penetrazione se la penalty/terminazione scatta troppo
  o troppo poco con la policy attiva.
- [ ] Tarare il rocker (stiffness/altezza/curvatura) con la policy che controlla
  la caviglia, per ottenere rollover COP senza spike.
- [ ] Contatto online anche sul lato sano quando le traiettorie ex-novo faranno
  divergere troppo il prescribed dal moto reale.
- [ ] Ricompilare/riconfermare il plugin onlineGRF su macOS arm64.
- [ ] Pulire gli script/log temporanei `results/_*.py`, `results/_*.log`.
