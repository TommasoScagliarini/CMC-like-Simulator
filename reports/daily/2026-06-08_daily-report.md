# Daily report - 2026-06-08

## Sintesi

Giornata lunga e densa, articolata in due grandi blocchi convergenti:

1. **GRF online**: dalla validazione gait-scale della modalità `online` pura
   (che ha dimostrato un limite **strutturale**) fino alla decisione e
   implementazione di un'architettura **ibrida** (prescribed sul lato sano +
   GRF online applicata sul lato protesico), poi integrata nel training RL.
2. **Training MLP/RLlib full-gait**: primo training diagnostico su episodi
   lunghi e un test diagnostico con GRF prescribed disabilitata su un lato.

Report utente prodotti oggi:

```text
reports/user/2026-06-08_training_full_gait_diagnostico_mlp.md
reports/user/2026-06-08_training_senza_grf_prescribed_sinistra.md
reports/user/2026-06-08_creazione_validazione_grf_online.md
reports/user/2026-06-08_validazione_online_grf_training_inference.md
reports/user/2026-06-08_analisi_cause_grf_online_non_utilizzabile.md
reports/user/2026-06-08_correzione_full_wrench_grf_online.md
reports/user/2026-06-08_validazione_gait_scale_grf_online.md
reports/user/2026-06-08_grf_ibrida_protesica.md
reports/user/2026-06-08_grf_online_sintesi_e_decisione_architetturale.md
```

## 1. Training full-gait diagnostico MLP/RLlib

Chiuso a livello diagnostico il primo TODO del 2026-06-07: la pipeline allena e
valuta episodi full-gait (`4.0 s`). Training 3 iter (2115 s, 0 timeout); il
checkpoint migliore completa un episodio da `4.0 s` (`truncated=True`, return
189.2) con almeno un ciclo online completo per lato. Limiti: 5/7 episodi
terminano per `joint_divergence_pros_knee_angle`, critic debole (explained var
negativa), saturazione knee SEA, reserve biologiche alte, fallback bounded
least-squares della Static Optimization, flag onlineGRF `in_contact` sempre
attivo, timing heel-strike sinistro sfasato nei rollout della policy.

## 2. Test diagnostico senza GRF prescribed su un lato

Introdotta l'opzione `--disable-prescribed-grf-side` (`prescribed_grf_disabled_
sides`): mantiene il prescribed come oracle ma ne omette la `ExternalForce` dalla
dinamica. Il test (left disabilitata) ha mostrato che reward e cinematica
restano quasi identici **perché le reserve del bacino mascherano la GRF
mancante** (reserve `pelvis_ty` 6.6 N → 513.6 N) e che la **caviglia protesica
si scarica** (3.0 vs 30.8 Nm). Conclusione chiave: la reward non distingue la
compensazione non fisica.

## 3. GRF online: creazione, validazione, analisi delle cause

- Calibrazione full-wrench (patch 2D, forza+momento), validazione fisica con
  attenzione alla penetrazione; promosso `online_sensor_basis` poi corretto per
  penetrazione eccessiva (~102 mm); watchdog esterno per `rollout_eval`.
- **Analisi delle cause**: il problema dominante non è la magnitudo verticale ma
  il **wrench esterno completo** (profilo temporale, COP, momenti). Le reserve
  del bacino seguono il deficit istantaneo di forza con correlazione ~`0.999`.
- **Correzione full-wrench + residuale state-based** (nel plugin C++, solo-stato,
  zero di default): tangenziale destro ricostruito, ma il gate dinamico restava
  `FAIL` (reserve active/sensor `1.82x` su 500 ms, soglia `1.5x`).

## 4. Validazione gait-scale: il limite è strutturale

Rifatta la validazione su finestra **gait-scale** (2 s) con cablaggio acceptance
corretto (il sensor-report e gli eventi erano artefatti di finestre da 19-500 ms).
Esito decisivo: correggendo l'impulso verticale da `0.83` a `0.94` (magnitudo
quasi perfetta) le reserve scendono solo da `11.1x` a `10.3x`. **Il limite è il
timing/forma del wrench, non la quantità di forza**: un contatto memoryless
state-based non può rifasare la forza in dinamica forward. Inseguire il prescribed
è infeasible.

## 5. Decisione e implementazione dell'architettura ibrida

Concetti chiave: il **wrench** è forza+momento (6D); le **reserve** del floating
base sono alte per GRF mancante e **non sono penalizzabili** (non controllabili
dal policy); il timing del wrench è bloccante **solo** se la GRF viene applicata
per sostituire il prescribed; **senza GRF la caviglia è un DOF libero**.

Architettura scelta: **prescribed sul lato sano + GRF online APPLICATA sul lato
protesico** (terreno reale per caviglia/ginocchio). Implementazione (solo Python,
nessuna modifica a SEA o plugin C++): applicazione GRF online **per-lato**
(`online_grf_applied_sides`, `appliesForce` per-sfera, prescribed auto-disabilitato
sui lati applicati).

**Track A validato**: rollout 1 s ibrido vs scarico → caviglia SEA `1.1 → 30.6 Nm`
(≈ valore caricato di riferimento), reserve `pelvis_ty` `512 → 64 N` (−87%),
stabile. La caviglia non è più un DOF libero.

**Track B (contatto fisico per il COP)**: generatore di suola a fila e **rocker**
(`generate_online_grf_row_profile.py`) + verifica COP read-only. La fila/rocker
sono più fisici (penetrazione ~5 mm, fasatura stance/swing) ma un rocker naïf
rigido produce spike (Fy 2412 N, ankle 267 Nm) e non rotola il COP in regime
CMC-tracking (la caviglia insegue la IK). Per carico/reserve **il profilo v2
resta oggi la scelta migliore**; il rocker è raffinamento del push-off, utile
con la policy che controlla la caviglia.

## 6. Integrazione hybrid nel training RL (implementata e validata)

Decisioni utente recepite: profilo **v2**; **niente penalty reserve**; penalty
leggera (>20 mm) + terminazione `grf_penetration` (>28 mm) sulla penetrazione del
piede protesico; reserve del lato sano ignorate; osservazione GRF attiva.

- `osim_trj_cmc_like.py`: `online_grf_applied_sides` + soglie penetrazione,
  penalty graduata + terminazione, propagazione, info.
- `train_ppo_mlp.py`, `rollout_eval.py`, `env_factory.py`: flag
  `--online-grf-applied-side`, wiring, **default profilo = v2**.
- `commands.txt`: comandi training/rollout ibrido + overnight (Windows 12 worker,
  macOS 5 worker, iperparametri scalati) + setup env macOS.

Verifiche: `py_compile` PASS; smoke env ibrido (piede protesico caricato ~420 N,
penetrazione 9.5 mm, `pen_loss=0`, nessuna terminazione spuria); tiny training
RLlib end-to-end exit 0 con config v2/left/osservazione propagata.

## 7. Git e ambienti

- Tutto il lavoro GRF di oggi committato e **unificato su `main`** (`3e22ff3`);
  branch di lavoro eliminato. Risolto il problema di pull sul Mac (era fermo a
  `9754577` perché i commit erano su un branch separato).
- Documentata la ricetta per creare `envCMC-rllib` su macOS (clone di
  `envCMC-like` + `ray[rllib]==2.55.1`, stessa versione di Windows).

## File principali modificati oggi

```text
config.py
main.py
model_loader.py
simulation_runner.py
online_grf.py
tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.{h,cpp}
tools/online_grf_contact/README.md
online_grf_profiles/README.md
validation/calibrate_online_grf_basis.py
validation/validate_online_grf.py
validation/verify_online_grf_plugin.py
validation/test_online_grf_core.py
Trajectory Generator/osim_trj_cmc_like.py
Trajectory Generator/baseline_MLP/{env_factory,train_ppo_mlp,rollout_eval}.py
Trajectory Generator/baseline_MLP/{README.md,commands.txt}
```

## File principali aggiunti oggi

```text
validation/analyze_online_grf_active_failure.py
validation/calibrate_online_grf_residual.py
validation/validate_online_grf_forward_drift.py
validation/online_grf_acceptance.py
validation/online_grf_acceptance_thresholds.json
validation/tune_online_grf_support.py
validation/generate_online_grf_row_profile.py
validation/check_online_grf_cop_rollover.py
Trajectory Generator/baseline_MLP/process_watchdog.py
Trajectory Generator/baseline_MLP/validate_online_grf_train_inference.py
online_grf_profiles/AB06_..._online_full_wrench_*.json (smoke/iter/residual/dense)
online_grf_profiles/AB06_..._online_gait2s_{basis,residual}.json
online_grf_profiles/AB06_..._online_{row,rocker,rocker_soft,physical_basis_10mm_balanced}.json
reports/user/2026-06-08_*.md (9 report)
```

## Verifiche eseguite

- training full-gait 3 iter + rollout deterministico 4 s: completati;
- test senza GRF prescribed sx + controllo bilaterale: completati;
- calibrazione/validazione full-wrench + residuale; acceptance gate;
- trio forward gait-scale (online/sensor/prescribed) 2 s: `status=complete`;
- forward-drift + acceptance gait-scale con input corretti;
- Track A ibrido (caricato vs scarico): caviglia 1.1→30.6 Nm, reserve 512→64 N;
- generatore fila/rocker + COP read-only + forward 1 s;
- `py_compile` di tutti i file Python modificati;
- smoke env ibrido + tiny training RLlib end-to-end;
- `git diff --check`; push e unificazione su `main`.

## TODO chiusi oggi

- eseguire training full-gait diagnostico (TODO #1 del 2026-06-07);
- introdurre disabilitazione per-lato del prescribed;
- analizzare a fondo perché la GRF online pura non è utilizzabile;
- correggere tangenziale/momento con residuale state-based;
- validare la modalità `online` su finestra gait-scale e diagnosticarne il limite
  strutturale (timing del wrench);
- decidere l'architettura della GRF per il generatore (ibrida);
- implementare l'applicazione GRF online per-lato (no C++);
- validare Track A (carico caviglia/ginocchio, reserve);
- implementare il generatore di contatto fisico fila/rocker;
- integrare l'ibrido nel training RL (env + train + rollout + penalty/terminazione);
- unificare i rami git su `main` e documentare l'env macOS.

## TODO aperti e propagati

### Priorità immediate: training ibrido
- Lanciare un training ibrido reale (50-80 iter, episodi 2 s) e analizzare
  apprendimento, terminazioni `grf_penetration`, carico caviglia, stabilità.
  Comandi pronti (Windows 12 worker ~70 iter; macOS 5 worker ~60 iter); una notte
  ≈ ~100-290k env-step (budget modesto: stadio, non convergenza).
- Tarare le soglie di penetrazione (penalty/terminazione) con la policy attiva.
- Prima del run macOS: **ricompilare/riconfermare il plugin onlineGRF `.dylib`**
  su arm64 (necessario per i contatti applicati).

### Contatto online / COP
- Tarare il rocker (stiffness, altezza ground sulla traiettoria forward,
  curvatura) con la policy che controlla la caviglia, per un rollover COP liscio
  senza spike.
- Valutare contatto online anche sul lato sano quando le traiettorie ex-novo
  faranno divergere troppo il prescribed dal moto reale.
- NOTA: l'obiettivo "reserve pure-online ≤ 1.5x" è **superato** dalla scelta
  ibrida (limite strutturale); non promuovere la modalità `online` pura a
  `validated`.

### Training MLP / dinamica (dal full-gait diagnostico)
- Ridurre le terminazioni `joint_divergence_pros_knee_angle`.
- Ridurre la saturazione del knee SEA.
- Analizzare/ridurre reserve biologiche ed equilibrium failures.
- Diagnosticare i fallback bounded least-squares della Static Optimization.
- Correggere/ricalibrare il timing heel-strike online del lato sinistro nei
  rollout della policy.
- Correggere il flag onlineGRF `in_contact` (resta sempre attivo nel rollout).
- Migliorare il critic (explained variance negativa).
- Rendere il launcher di monitoraggio capace di restituire subito il controllo e
  pubblicare aggiornamenti durante il training principale.
- Tarare `oob_weight` con `reward/oob_term`; rivalidare il filtro 6 Hz su rollout
  lungo con policy allenata.

### Repository / housekeeping
- Pulire gli script/log temporanei `results/_*.py`, `results/_*.log`,
  `validation/_hybrid_env_smoke.py`.
- Valutare se togliere da git i build artifact `build_online_grf/*.obj`
  (rigenerati a ogni compilazione).

### Traiettorie ex-novo / GLiDE-like
- Metriche di suitability (velocità, stabilità, simmetria, effort, energia SEA);
  action space assoluto/parametrico; livello QP vincolato; curriculum imitativa→
  task-based; separare fattibilità da imitazione; modulo metriche
  RMSE/NRMSE/Symmetry; metriche funzionali basate su GRF/impulsi.

### Validazione generatore / SNN / PPO (propagati)
- F1-F4, F7 percorso SNN/skrl; cache `data_ptr` (F6/T2); reshape/BPTT (T3); spike
  gradient encoder (T4); reader `.sto` `inDegrees` (S1); default `sea_stiffness`
  per-modello (S2/J2); riuso `InverseDynamicsSolver` (S3); SNN come RLModule.

### Knowledge base letteratura
- Recuperare il vero paper Wrapyfi (paper/7 è iCub); approfondire P17/P24/P14/
  P19/P23; collegare la letteratura alla roadmap ex-novo/GRF online.

### Propagati storici: controllo SEA
- Sweep `Kp_knee_motor` 3.9-18; coupling knee-ankle; notch 28 Hz knee; cleanup
  modelli sperimentali; build/copia DLL plugin PI Windows; secondo pass knee;
  confronto finale configurazioni storiche; cleanup artefatti sweep; LPF qdot
  asimmetrico ankle 30/35 Hz su run lunga.
