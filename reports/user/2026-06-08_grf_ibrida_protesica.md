# GRF ibrida: prescribed sul lato sano + online applicata sul lato protesico

Data: 2026-06-08

## Problema e decisione di direzione

La validazione gait-scale aveva dimostrato che la modalità `online` pura **non
può** riprodurre il timing/forma del wrench prescribed in dinamica forward: la
magnitudo (impulso) è correggibile ma le reserve del bacino restano `~10x`
(limite strutturale del contatto memoryless, non di calibrazione). Vedi
[2026-06-08_validazione_gait_scale_grf_online.md](2026-06-08_validazione_gait_scale_grf_online.md).

Anziché continuare a inseguire il prescribed, si è scelta un'architettura
diversa, motivata da un requisito di controllo:

- **prescribed sul lato sano** (insegue la IK, approssimazione valida);
- **GRF online APPLICATA sul lato protesico** per dare un terreno reale contro
  cui caviglia e ginocchio possano lavorare.

Motivazione (corretta): senza GRF applicata, l'angolo di caviglia non ha
conseguenza dinamica (non produce forza, non sposta il COP, non genera push-off)
→ la caviglia degenera in un **grado di libertà libero**, pinnato solo
dall'imitazione, inutilizzabile per traiettorie ex-novo. Il timing imperfetto
del wrench online **non** è più bloccante, perché non si insegue il prescribed:
serve solo un sostegno fisico plausibile, che il contatto Hunt-Crossley dà per
costruzione.

## Soluzione implementata (solo Python, nessuna modifica al C++ o al SEA)

Applicazione della GRF online **per-lato**. In modalità `online_sensor`, le sfere
del lato selezionato vengono applicate (non solo sensate), mentre il prescribed
viene auto-disabilitato su quei lati per non raddoppiare il carico.

- `config.py`: nuovo campo `online_grf_applied_sides`.
- `online_grf.py`: `add_online_grf_forces(..., apply_sides=...)` imposta
  `appliesForce` per-sfera (`sphere.side in apply_sides`); il default globale
  resta per retrocompatibilità.
- `model_loader.py`: validazione, auto-disabilitazione del prescribed sui lati
  applicati, log diagnostico, propagazione nel `SimulationContext`.
- `simulation_runner.py`: `online_grf_applied_sides` nel `run_status`.
- `main.py`: flag `--online-grf-applied-side` e `--disable-prescribed-grf-side`.

Il plugin C++ supportava già `appliesForce` per istanza: nessuna modifica nativa.

Lato protesico AB06 = **sinistra** (`foot_l`); lato sano = destra (`calcn_r`).

Smoke di verifica: `right_ground_force2 -> addForce OK`,
`left_ground_force1 -> skipped`, `Online GRF: 20 contacts (hybrid: 8 applied
(left), 12 sensor-only)`.

## Track A — l'ibrido col contatto esistente: VALIDATO

Rollout 1 s `[11.99, 12.99]`, profilo esistente `..._residual_tangent_v2`,
ibrido vs baseline scarico (prescribed sx disabilitata, online solo sensore):

| Metrica (lato protesico) | Scarico (sensor) | Ibrido (online applicata) |
|---|---:|---:|
| Caviglia SEA tau media/max | 1.1 / 2.4 Nm | **30.6 / 53.9 Nm** |
| Ginocchio SEA tau media/max | 2.3 / 4.3 Nm | **38.4 / 111.9 Nm** |
| reserve `pelvis_ty` media | 512 N | **63.8 N** (−87%) |
| Forza verticale piede protesico | 536 N (solo sensata) | 517 N (applicata) |
| Stato run | complete | complete (stabile) |

La caviglia passa da **scarica** (1.1 Nm) a **caricata** (30.6 Nm, quasi
identica al valore "loaded" del controllo bilaterale, 30.8 Nm). La reserve del
bacino crolla 8x. **La caviglia non è più un DOF libero: lavora contro una
reazione al suolo reale, e il bacino non è più tenuto su artificialmente.**

## Track B — contatto fisico a fila per il COP

L'analisi del COP (formula: `COP = Σ Fᵢ pᵢ / Σ Fᵢ`) ha mostrato che il
rollover tallone→punta richiede una distribuzione del contatto lungo la suola,
non poche sfere calibrate. È stato aggiunto un generatore di contatto **fisico**
(non fit al prescribed):

- `validation/generate_online_grf_row_profile.py`: fila di N×C sfere interpolate
  tra i marker tallone e punta, materiale fisico (k=1e6, μ=0.8), suola piatta al
  livello del marker più basso, **auto-fit dell'altezza del ground** per una
  penetrazione di picco target;
- `validation/check_online_grf_cop_rollover.py`: verifica read-only del
  rollover.

Profilo generato: `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_row.json`
(6 long × 2 lat per piede, raggio 2 cm). Test forward ibrido 1 s vs v2:

| Metrica (piede protesico) | v2 calibrato | Fila fisica |
|---|---:|---:|
| Penetrazione picco | 17.5 mm | **4.9 mm** |
| Frazione di contatto | 1.00 (sempre nel suolo) | **0.35** (stance/swing) |
| Picco Fy | 709 N | 539 N |
| Range COP (rollover) | 6.6 cm | 1.1 cm |

La fila è **più fisica** (penetrazione 5 mm, fasatura stance/swing corretta;
v2 resta incollato al suolo al 100% perché affonda 17 mm), **ma** la suola piatta
rigida resiste al beccheggio → il COP quasi non rotola. Il rollover ampio di v2
è in parte artefatto della penetrazione profonda.

### Geometria a rocker

È stata aggiunta al generatore una suola a **rocker** (sfere su arco convesso,
parametri `--rocker-radius`, `--apex-fraction`, `--stiffness`, `--dissipation`):
quando il piede beccheggia, il punto di contatto migra lungo l'arco. Test forward
ibrido 1 s (piede protesico):

| Profilo | contact | picco Fy | picco pen | COP range | ankle max | reserve `pelvis_ty` |
|---|---:|---:|---:|---:|---:|---:|
| v2 calibrato | 1.00 | 709 N | 17.5 mm | 6.6 cm | 53.9 Nm | **64 N** |
| flat row | 0.35 | 539 N | 4.9 mm | 1.1 cm | 39.3 Nm | 443 N |
| rocker rigido (k=1e6) | 0.50 | 2412 N | 8.5 mm | 2.4 cm | 267 Nm | 387 N |
| rocker soft (k=2e5) | 0.79 | 1928 N | 15.8 mm | 1.6 cm | 111 Nm | 195 N |

Due ostacoli reali:

1. **Spike d'impatto**: un rocker concentra il carico su 1–2 sfere; con
   stiffness alta produce picchi di forza (1928–2412 N ≫ peso) e di coppia di
   caviglia (111–267 Nm). Ammorbidire la molla li attenua ma non li elimina.
2. **Niente rollover in regime CMC-tracking**: in questo run la caviglia
   *insegue la IK*, quindi il piede beccheggia poco → il punto di contatto del
   rocker non migra (COP range 1–2 cm). Il rollover dovrebbe emergere quando la
   **policy RL controlla attivamente** la caviglia con beccheggio ampio.

Inoltre l'auto-fit del ground (tarato sugli stati `online_sensor`) non combacia
con la traiettoria forward del piede guidato dal contatto → le suole fisiche
sotto-supportano (reserve 195–443 N) rispetto a v2 (64 N), che però "imbroglia"
restando sempre penetrato.

**Conclusione Track B:** la capacità rocker è implementata e parametrica, ma una
suola fisica che regga con continuità, rotoli il COP e non produca spike richiede
un tuning multi-parametro (stiffness, altezza ground sulla traiettoria forward,
curvatura) non convergente in poche iterazioni. Per il carico di caviglia e le
reserve — ciò che conta per sbloccare il training — **il profilo v2 nell'ibrido
resta oggi la scelta migliore**; il rocker è uno strumento di raffinamento della
fedeltà del push-off, più utile quando la policy controllerà la caviglia.

## Conclusione

L'architettura ibrida **sblocca il generatore**: il piede protesico porta carico
reale, caviglia/ginocchio diventano variabili di controllo dotate di significato,
la dinamica è stabile e le reserve del bacino tornano piccole. Il timing del
wrench, irrisolvibile in pura online, qui è irrilevante.

Resta aperto il **rollover del COP** per un push-off fedele: una suola piatta
regge il peso ma non fa migrare il COP; serve una geometria a **rocker** (sfere
su un arco) o una suola più cedevole. È tracciabile e non blocca il training.

## File modificati / aggiunti

```text
config.py
online_grf.py
model_loader.py
simulation_runner.py
main.py
validation/generate_online_grf_row_profile.py   (nuovo)
validation/check_online_grf_cop_rollover.py      (nuovo)
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_row.json (nuovo)
```

## Test e verifiche

- `py_compile` di tutti i file modificati: PASS;
- CLI: `--online-grf-applied-side` / `--disable-prescribed-grf-side` presenti;
- smoke ibrido 0.1 s: setup corretto (right applied, left skipped, 8 online
  applied left);
- Track A 1 s ibrido vs scarico: caviglia 1.1→30.6 Nm, reserve 512→64 N,
  entrambi `status=complete`;
- generazione fila + auto-fit ground (penetrazione picco 6 mm su IK-replay);
- forward ibrido con fila 1 s: `status=complete`, penetrazione 4.9 mm, stance 35%;
- retrocompatibilità modi prescribed/online_sensor/online: preservata
  (ramo per-lato attivo solo con `online_grf_applied_sides` non vuoto).

## TODO

- [ ] Suola a **rocker** (sfere su arco antero-posteriore) o suola cedevole per
  ottenere un rollover COP tallone→punta in dinamica forward; ri-testare il COP
  forward nell'ibrido.
- [ ] Integrare l'ibrido nel training RL: `online_grf_applied_sides` nell'env
  `osim_trj_cmc_like.py` e negli entrypoint baseline_MLP (oggi c'è solo
  `disable_prescribed_grf_side`).
- [ ] Aggiungere penalty/terminazione su penetrazione del piede protesico come
  rete di sicurezza (la GRF la limita già fisicamente).
- [ ] NON penalizzare le reserve nella reward (sono artefatto del supporto
  mancante sul lato sano in single-support, non controllabili dal policy).
- [ ] Valutare contatto online anche sul lato sano quando le traiettorie ex-novo
  faranno divergere troppo il prescribed dal moto reale.
- [ ] Ricompilare/riconfermare il plugin onlineGRF su macOS arm64.
- [ ] Pulire gli script temporanei `results/_*.py` e i log `results/_*.log`.
