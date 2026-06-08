# Piano — Calcolo PURO delle GRF online per guidare la dinamica RL (AB06)

## Context

Durante il training della policy le **prescribed GRF non possono più guidare la dinamica**: sono time-driven (forze da force-plate applicate in funzione del tempo) e quindi fisicamente sbagliate appena la policy produce una traiettoria diversa dalla IK. Serve la modalità `online`, in cui il contatto piede-terreno calcolato online (`OnlineGRFSphereHalfSpaceForce`) guida *puramente* la dinamica.

Stato attuale: la modalità `online` è marcata `online_mode_status: "smoke_only"` nel profilo validato. Esiste solo un rollout pure-online di **0.5 s** (`results/online_basis_pure_rollout/`, `status=complete`, nessun guard trip) — troppo corto per osservare deriva/sink o un ciclo di gait. L'accuratezza in magnitudo su IK-replay è ~5-8% NRMSE-peak (floor strutturale documentato della legge state-based; griglie più dense non lo abbassano). Il rischio reale e **non misurato** è la dinamica forward: un deficit di supporto verticale può far affondare `pelvis_ty`, gonfiare `tau_bio`, far crescere `reserve_pelvis_ty` e far scattare la soglia di caduta (0.55 m) producendo terminazioni spurie. NB: solo una guard *superiore* a runtime (`online_grf_max_force_bw=5 BW`); **nessuna guard inferiore** → un deficit affonda in silenzio.

Decisioni utente per questo passaggio:
- **Bar di promozione = ENTRAMBI**: stabilità dinamica come gate obbligatorio **e** spingere la magnitudo il più vicino possibile al floor (~5%). → Stage 3 è core.
- **Perimetro = SOLO AB06** ora; pipeline per-modello generale rinviata (tooling resta riusabile).
- **Se instabile**: patch+ricalibrazione e, se non basta, **correttore residuale state-based** (pre-approvato).

Outcome atteso: profilo AB06 con `online_mode_status` promosso a "validated", supportato da un report di accettazione quantitativo (accuratezza IK-holdout + stabilità forward-dynamics), con magnitudo spinta verso il floor.

Vincolo di calcolo: ~**105 s wall per secondo simulato** (0.5 s → 52 s) a dt=1 ms con SO+SEA. Rollout lunghi → in **background**, con probe breve prima di estendere. Finestra IK AB06: t 11.99→21 (~9 s).

Caveat onesto: l'NRMSE-peak su IK-holdout molto probabilmente **non scenderà sotto ~5%**. Il gate di promozione resta la **stabilità dinamica**; la parità di magnitudo è un target verso il floor, non una condizione infeasibile.

---

## Stage 0 — Baseline riproducibile + parità plugin (no codice nuovo)

**Obiettivo:** punto di partenza noto; confermare parità C++/Python e load plugin sul toolchain attuale prima di fidarsi di qualsiasi numero forward.

**Riuso:** `validation/verify_online_grf_plugin.py`, `validation/test_online_grf_core.py`, `validation/validate_online_grf.py --no-calibrate`.

```
conda run -n envCMC-like python validation/test_online_grf_core.py
conda run -n envCMC-like python validation/verify_online_grf_plugin.py `
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml `
  --profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json `
  --report results/online_grf_plugin_audit_stage0.json
conda run -n envCMC-like python validation/validate_online_grf.py `
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml `
  --profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json `
  --no-calibrate --report results/online_grf_validation_stage0.json
```

**Gate:** audit `equivalent_within_1e-8_n: true`; test 5/5; holdout NRMSE-peak entro ~0.5pp del baseline (L 0.077 / R 0.042). Se la parità fallisce → ricompilare plugin (Stage 5) prima di procedere.

---

## Stage 1 — Rollout lungo pure-online su IK-replay (DECISIVO) — NEW validator

**Obiettivo:** stabilire se il profilo attuale, guidando *puramente* la dinamica, dà dinamica forward stabile su orizzonte gait-scale. È l'esperimento più decisivo ed economico: determina quanto serve davvero la correzione di magnitudo.

**1.1 — Tre rollout head-to-head sulla STESSA finestra IK AB06** (riuso `main.py`, no codice). Probe a ~2 s per misurare il wall-clock, poi estendere fino a ~9 s (t 11.99→~20.99), dt 0.001, in **background**.
```
# (a) pure online (guida la dinamica, no prescribed)
conda run -n envCMC-like python main.py `
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml `
  --grf-mode online --no-external-loads `
  --online-grf-profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json `
  --t-start 11.99 --t-end 20.99 --dt 0.001 --output-dir results/online_grf_longroll_online
# (b) online_sensor: come (a) ma --grf-mode online_sensor (senza --no-external-loads), output ..._sensor
# (c) prescribed: --grf-mode prescribed (no profilo), output ..._prescribed
```

**1.2 — NEW `validation/validate_online_grf_forward_drift.py`** (gap reale; `_val_long_rollout.py` fa diagnostica RL-env, non drift GRF). READ-ONLY: consuma gli output `.sto` già scritti, non calcola dinamica. Calcola, head-to-head online vs sensor vs prescribed:
- **Drift `pelvis_ty`** vs IK ref (campionata via `kinematics_interpolator.KinematicsInterpolator`): max sink, RMS, end-of-window, **pendenza di sink** (fit lineare → distingue offset bounded da runaway).
- **Reserve**: da `_reserve_torques.sto`/`_reserve_controls.sto` (reserve `pelvis_ty`): max |torque|, sustained (95° pct), growth slope; da `_recruitment.sto`: `unactuated_reserve_norm`/`tau_reserve_norm`.
- **Caduta**: frazione di tempo `pelvis_ty_q < 0.55`, durata fall-free; classifica `_run_status.txt` (`complete` vs `failed`+`error_type`).
- **Plausibilità GRF**: da `_online_grf.sto` picco verticale/lato (BW), **impulso verticale online/prescribed sulla stessa finestra** (ratio = misura di supporto dinamicamente rilevante), duty `in_contact`, max `penetration`, max `slip_speed`.
- Output: 1 JSON + PNG impilato (drift pelvis_ty / reserve pelvis_ty / GRF verticale BW) per i 3 run.
- Riusa: `output._read_storage_table`, `kinematics_interpolator.KinematicsInterpolator`, `validate_online_grf._external_grf` (impulso oracolo), `setup_io.read_setup_xml`, `path_resolver.resolve_repo_path`.
```
conda run -n envCMC-like python validation/validate_online_grf_forward_drift.py `
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml `
  --online-dir results/online_grf_longroll_online --sensor-dir results/online_grf_longroll_sensor `
  --prescribed-dir results/online_grf_longroll_prescribed --fall-threshold 0.55 `
  --report results/online_grf_forward_drift_stage1.json --plot results/online_grf_forward_drift_stage1.png
```

**Gate (DECISION POINT):** pure-online completa l'intera finestra (`status=complete`, no guard trip), drift `pelvis_ty` bounded, reserve non esplose vs sensor/prescribed.
- **PASS** → magnitudo già sufficiente per la stabilità; Stage 3 si limita allo *spingere verso il floor* (patch+ricalibrazione), correttore non necessario.
- **PARTIAL/FAIL** → il report individua il modo di fallimento (sink / reserve / caduta / guard) e scope direttamente Stage 3 incluso il correttore residuale.

---

## Stage 2 — Criteri di accettazione quantitativi (NEW harness)

**Obiettivo:** un unico gate riproducibile PASS/FAIL su ENTRAMBI gli assi (oggi esiste solo l'exit-code del plugin audit).

**NEW `validation/online_grf_acceptance.py`:** aggregatore sottile che ingerisce i JSON già prodotti (`validate_online_grf`, `analyze_online_grf_heel_strikes`, `validate_online_grf_forward_drift`, `verify_online_grf_plugin`), confronta ogni metrica con una soglia, scrive summary PASS/FAIL per-criterio (exit 0/1). Soglie in **NEW `validation/online_grf_acceptance_thresholds.json`** (default AB06).

**Soglie candidate (da ratificare dopo i numeri reali di Stage 1):**

ACCURATEZZA (IK holdout — bounded dal floor, "Entrambi" ⇒ gate + target):
- Vertical NRMSE-peak: **gate ≤ 0.10**/lato (no regressione); **target → ~0.05** (floor). Non impostare gate < 0.05.
- Vertical correlation ≥ **0.95**/lato. Contact F1 ≥ **0.90**/lato.
- Heel-strike F1 = **1.0**, timing MAE ≤ **20 ms** (criterio che conta per la scansione gait).
- Tangenziale NRMSE-peak ≤ **0.40**/asse orizzontale — **WARN-only** inizialmente.

DINAMICA FORWARD (rollout lungo online IK-replay):
- Max sink `pelvis_ty` ≤ **0.03 m**; pendenza di sink ≤ **0.005 m/s**.
- Sustained (95° pct) |reserve `pelvis_ty`| ≤ **1.5×** il run prescribed sulla stessa finestra.
- Fall-free = intera finestra; tempo sotto 0.55 m = 0; `status=complete`, no guard trip.
- Rapporto impulso verticale online/prescribed ∈ **[0.90, 1.15]**.

PARITÀ (sempre): plugin audit `max_abs_error_n ≤ 1e-8`.

**Gate:** l'harness riflette correttamente i numeri di Stage 0/1. Definisce il gate per Stage 3-4.

---

## Stage 3 — Migliorare il calcolo puro (CORE: bar = "Entrambi") — riuso + NEW correttore se serve

**Obiettivo:** alzare il supporto verticale verso il floor e chiudere il deficit emerso in Stage 1. I profili sono JSON → **nessuna modifica C++**.

- **3a — Patch plantare 3D più densa + ricalibrazione intensive** (riuso `validation/calibrate_online_grf_intensive.py`, N-sfere via `--spheres-per-side`, stage tangenziale dedicato): re-fit per-contatto radius/stiffness/dissipation/exponent sullo split di calibrazione; report holdout.
```
conda run -n envCMC-like python validation/calibrate_online_grf_intensive.py `
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml `
  --profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json `
  --out-profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_dense.json `
  --spheres-per-side 6 --report results/online_grf_calibration_dense_stage3.json
```
  Attesa: l'holdout probabilmente NON batte sostanzialmente il basis → il non-miglioramento è **conferma del floor** e indica che il deficit va chiuso da 3c, non da più sfere.
- **3b — Re-run basis NNLS** con più candidati (riuso `validation/calibrate_online_grf_basis.py`, `--location-count`/`--radius-count`/`--max-contacts-per-side`). Economico; può ribilanciare il supporto. Stesso caveat floor.
- **3c — Correttore residuale state-based (APPROVATO; eseguire se 3a/3b non chiudono il deficit)** — **NEW `validation/calibrate_online_grf_residual.py`**. Vincolo non negoziabile: funzione **solo dello stato** (penetrazione, velocità normale, indicatore regione plantare). **Vietato** qualsiasi termine su tempo, fase da clock, o segnale prescribed a runtime (invaliderebbe la premessa ex-novo). Opzioni ammesse: (i) guadagno/offset verticale per-lato sulla forza normale in funzione della penetrazione + indicatore regione; (ii) piccolo regressore su feature di stato fit sulla verticale prescribed in IK-replay, applicato dentro la legge di contatto. Validare su **IK HOLDOUT + nuovo rollout forward** (Stage 1 ripetuto).

**Gate per profilo candidato:** ri-eseguire Stage-1 forward-drift + Stage-2 acceptance; accettare solo se passa i criteri dinamici (impulso, drift, reserve) SENZA regredire accuratezza/eventi. Scegliere il profilo **più semplice** che passa.

---

## Stage 4 — Promozione a "validated" + evidenza di accettazione

**Obiettivo:** portare `online_mode_status` da `smoke_only` a validato per il profilo AB06 accettato.

**Riuso:** harness Stage-2 come gate; blocco `metadata` del profilo.

**Edit previsti (in implementazione, non in questo pass read-only):**
- `metadata` del profilo JSON: `online_mode_status` → es. `"validated_ik_replay_forward_dynamics"`; registrare summary Stage-1 (max sink, ratio reserve, ratio impulso, durata fall-free) + verdetto Stage-2 + hash set-soglie.
- Documentare che l'unica guard runtime è `online_grf_max_force_bw` e che il regime validato si appoggia ai criteri di accettazione (una guard inferiore di supporto è hardening futuro, non richiesto per passare).
- Aggiungere un report `reports/user/AAAA-MM-GG_*.md` con l'evidenza di accettazione e il caveat onesto sul floor.

**Gate finale (AB06):** verdetto Stage-2 = PASS sul profilo accettato; parità = 0; rollout lungo `status=complete`.

---

## Stage 5 — Ri-conferma cross-platform (macOS arm64)

**Obiettivo:** assicurare che il plugin che guida la dinamica sia valido sul secondo toolchain. Build macOS arm64 **già fatta/verificata il 2026-06-08** (`tools/online_grf_contact/README.md`) → solo ri-conferma per ambiente.

**Riuso (comandi già nel README):** build CMake → `plugins/libOnlineGRFContact.dylib`; `file`/`lipo -archs`/`otool -L`/`codesign --verify`; load tipo via `opensim.LoadOpenSimLibrary('plugins/OnlineGRFContact')`; `validation/verify_online_grf_plugin.py` (exit 0, ≤1e-8); smoke pure-online 2-3 s (`status=complete`).

**Gate:** parità exit 0 su macOS arm64; smoke pure-online completa.

---

## Rinviato (fuori perimetro: "Solo AB06 ora")

Generalizzazione per-modello (altri modelli di training, soglie/profili per-modello, secondo candidato `Adjusted_newmarkers`): tooling resta riusabile (`infer_profile_from_markers` → calibrazione → validazione → drift → acceptance), ma **non eseguito** in questo passaggio.

---

## Verifica end-to-end

1. Stage 0 verde (parità ≤1e-8, test 5/5, holdout baseline).
2. Stage 1: i 3 rollout completano; `validate_online_grf_forward_drift.py` produce JSON+PNG con drift/reserve/impulso head-to-head.
3. Stage 2: `online_grf_acceptance.py` riflette correttamente Stage 0/1 e dà un verdetto PASS/FAIL per-criterio.
4. Stage 3: ogni profilo candidato ri-passa Stage-1+Stage-2; scelto il più semplice che passa entrambi gli assi.
5. Stage 4: profilo promosso, report di accettazione scritto, gate finale PASS.
6. Stage 5: parità + smoke pure-online verdi su macOS arm64.

## File critici

- `validation/validate_online_grf.py` — helper condivisi (`_calculate_grf`/`_external_grf`/`_metrics`/`_sample_spheres`) riusati dai nuovi validator.
- `output.py` — `_read_storage_table` + schema colonne `_states.sto`/`_reserve_*.sto`/`_recruitment.sto`/`_online_grf.sto` che il drift validator parse-a.
- `simulation_runner.py` — `_sample_online_grf` (guard superiore, ~righe 843-862; rischio sink silenzioso) e `_write_run_status`.
- `online_grf.py` — core profilo + `online_grf_column_names`.
- `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json` — profilo da testare (Stage 1) e promuovere (Stage 4).
- `main.py` — run path `--grf-mode online --no-external-loads`.

## NEW codice (solo dove c'è gap reale)

- `validation/validate_online_grf_forward_drift.py` — validator drift/reserve forward (READ-ONLY su output esistenti).
- `validation/online_grf_acceptance.py` + `validation/online_grf_acceptance_thresholds.json` — gate PASS/FAIL aggregato.
- `validation/calibrate_online_grf_residual.py` — correttore residuale **solo-stato** (eseguito se 3a/3b non bastano).

Tutto il resto RIUSA: `validate_online_grf.py`, `calibrate_online_grf_intensive.py`, `calibrate_online_grf_basis.py`, `analyze_online_grf_heel_strikes.py`, `compare_online_grf_output.py`, `verify_online_grf_plugin.py`, `test_online_grf_core.py`, `main.py`, core `online_grf.py`.
