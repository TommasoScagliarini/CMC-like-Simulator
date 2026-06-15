# Fix cosmetica CLI (mojibake) e timeout/throughput del training

Data: 2026-06-09

## Problema

Lanciando il training ibrido con il nuovo output assoluto (vedi
[[2026-06-09_gait_phase_clock_e_output_assoluto_rete]]) sono emersi due problemi:

1. **Cosmetica CLI illeggibile (mojibake).** La barra di avanzamento, lo spinner
   e l'ETA apparivano come caratteri corrotti (`â–ˆ`, `â ‹`) invece dei glifi
   Unicode (`█`, `⠋`).
2. **Il training non arrivava alla fine.** Il run (60 iterazioni, 12 worker,
   `--iteration-timeout-s 1800`, `--run-timeout-s 36000`) è stato abortito a
   iterazione 12.

## Strategia di diagnosi

- **Mojibake**: ispezione di `progress_display.py`. C'era già un fallback ASCII,
  ma il rilevamento si basava solo su `stream.encoding`.
- **Crash**: ispezione read-only degli artefatti del run
  (`runs/baseline_mlp_hybrid_win/summary.json`, `watchdog_state.json`) e
  **misura diretta del costo per-step/per-reset** dell'env in single-process
  (niente Ray) per isolare env vs overhead Ray.

## Cause individuate

### 1. Mojibake: codepage reale della console non verificata

Sotto `conda run` Python riporta `sys.stderr.encoding = utf-8` e scrive byte
UTF-8, ma la **console Windows è in CP1252**: il check
`_stream_supports_unicode` passava (l'encoding "supporta" i glifi) e quindi
venivano emessi byte UTF-8 che la console rendeva come mojibake. Il fallback
ASCII non scattava mai.

### 2. "Non arriva alla fine": timeout di iterazione + throughput inerente

`summary.json` (autorevole):

```json
{ "ok": false, "timed_out": true,
  "error": "algo.train iteration 12 exceeded the 1800 s wall-clock timeout.",
  "iterations_run": 11 }
```

Misura diretta dell'env (single-process, hybrid, `segment 0.01`):

```text
env build : 5.61 s
reset     : 0.01 s   (il "reset thrashing" NON era il problema)
step      : ~1183 ms/step  (mean su 20 step benigni)
```

Quindi il costo dominante è **~1.18 s per env-step** (pipeline CMC: Static
Optimization QP + fallback). Con `action_mode=absolute` (nuovo default) le pose
ex-novo fanno **fallire più spesso la QP** (warning "QP did not converge →
bounded least-squares fallback"), portando le iterazioni a **~13-15 min** (più
lente del delta). Conseguenze:

- `--iteration-timeout-s 1800` (30 min) è solo ~2× il tempo iter normale → una
  singola iterazione degenere/lenta lo fa scattare e **aborta l'intero run**;
- `--run-timeout-s 36000` (10 h) < 60-80 iter × ~14 min (~14-19 h) → il run non
  potrebbe comunque completare nel budget.

Non è un bug del codice: è **config incoerente** + lentezza inerente del
simulatore (che AGENTS.md vieta di modificare senza richiesta esplicita).

## Soluzione

### Fix cosmetica (`Trajectory Generator/baseline_MLP/progress_display.py`)
- `_enable_windows_utf8_console()`: all'init di `LiveProgress` allinea la console
  Windows a UTF-8 (`SetConsoleOutputCP(65001)`), best-effort, no-op fuori Windows
  o senza console.
- `_windows_console_is_utf8()` + `_stream_supports_unicode` aggiornato: usa i
  glifi Unicode **solo** se la codepage reale della console è UTF-8; altrimenti
  **ASCII pulito**. Mai mojibake, in nessuna combinazione stream/console.

### Fix config (`Trajectory Generator/baseline_MLP/commands.txt`)
- Comando overnight Windows e macOS: `--iteration-timeout-s 1800 → 3600`,
  `--sample-timeout-s 1500 → 3000`, `--iterations 80 → 40` (coerente col budget
  10 h a ~13-15 min/iter).
- Aggiunta nota su: nuovo default `action_mode=absolute` (checkpoint delta
  incompatibili, training fresco), throughput ~1.15-1.2 s/step, e la regola
  `run-timeout ≈ iterations × ~800 s` + `iteration-timeout` ben sopra il tempo
  iter normale.

## File modificati

```text
Trajectory Generator/baseline_MLP/progress_display.py
  - _enable_windows_utf8_console(), _windows_console_is_utf8()
  - _stream_supports_unicode(): richiede console UTF-8 reale su Windows
  - LiveProgress.__init__: abilita UTF-8 prima del rilevamento
Trajectory Generator/baseline_MLP/commands.txt
  - overnight Win+macOS: iteration-timeout 3600, sample-timeout 3000, iterations 40
  - note su action_mode=absolute, throughput e regole timeout
```

## File aggiunti (diagnostica temporanea, `_`-prefix)

```text
validation/_env_timing.py                  (misura step/reset cost single-process)
validation/_gait_clock_absaction_smoke.py  (gia' presente dal report precedente)
```

## Test e verifiche eseguite

- `py_compile` di `progress_display.py` PASS.
- **Unit test cosmetica** (stream non-tty StringIO): output ASCII pulito, niente
  mojibake — `/ | train | [#######-----] | 30.0% | iter 3/10 | ... | ret 135.7`.
- **Console Windows**: `_enable_windows_utf8_console()` porta la CP a 65001; con
  stream cp1252 il fallback ASCII scatta correttamente. Nessun crash.
- **Training reale breve** (envCMC-rllib, online_sensor + applied left, absolute +
  clock, 2 worker, batch 128, 2 iter): **completato** —
  `Training complete: 2/2 iterations, best return 27.01`, `summary ok:true`,
  cosmetica Unicode pulita (`⠴ | train | [████████████░░░░] | 50.0% | ... | ETA`).
- **Inference reale breve** (`rollout_eval` sul checkpoint prodotto, absolute +
  clock + hybrid): **completata** — `Rollout done: 60 steps, return 36.65,
  truncated=True (episode_time_limit)`, `summary ok:true`, cosmetica pulita.
- Artefatti temporanei delle verifiche rimossi; i run reali
  (`baseline_mlp_hybrid_win`, `baseline_mlp_hybrid_win_rollout_best`) non toccati.

## Limiti / note oneste

- **Non è stato eseguito il run completo da 40 iter (~9 h)** in sessione: ho
  verificato che la pipeline completa su run breve e reso i timeout coerenti col
  budget; la modalità di fallimento (timeout) è risolta alla radice.
- **`action_mode=absolute` è più lento del delta** (~13-15 vs ~6-8 min/iter) per
  i fallback QP della SO su pose ex-novo. È inerente al simulatore; la
  mitigazione naturale è il **pre-training imitativo** (pose fisiologiche → meno
  fallback). Vedi TODO.

## TODO

### Prossimo passo diretto (linea di lavoro)
- **Reward task-based che usa il clock** (auto-periodicità + coordinazione +
  obiettivi di task) — deferred in
  [[2026-06-09_durata_episode_phase_clock_ex_novo]].
- **Pre-training imitativo** (template `φ`-indicizzato dalle colonne `pros_*`):
  oltre a inizializzare bene la policy, riduce i fallback QP della SO →
  iterazioni più veloci.
- **Tarare `gait_clock_phase_offset`** e valutare episodi lunghi / `random_init`.

### Throughput / robustezza training (quantificato oggi)
- ~1.18 s/env-step è il collo di bottiglia (SO QP + fallback bounded
  least-squares): diagnosticare/ridurre i fallback (già propagato) ridurrebbe
  direttamente il tempo per iterazione.
- L'**iteration-timeout aborta l'intero run** invece della sola iterazione
  degenere: valutare sampling asincrono o meno worker, così un singolo episodio
  patologico non gata l'iterazione sincrona (già propagato dal 2026-06-08/09).
- Lanciare il run completo (40 iter) con la config corretta e analizzare
  apprendimento/terminazioni/carico caviglia.

### TODO ereditati e propagati (ancora aperti)
- Ridurre alla radice divergenze `joint_divergence_pros_knee_angle`, saturazione
  knee SEA, fallback Static Optimization.
- Correggere i due bug HS **online** (timing protesico; flag `in_contact` sempre
  attivo) — prerequisito solo per varianti "evento discreto" / full task-based.
- Allineamento macOS: `setuptools<81` in `envCMC-rllib`; ricompilare/riconfermare
  il plugin onlineGRF `.dylib` su arm64.
- Migliorare il critic (explained variance negativa); tarare `oob_weight`;
  rivalidare il filtro 6 Hz su rollout lungo.
- Metriche di suitability ex-novo (velocità, stabilità, simmetria, effort,
  energia SEA, fattibilità GRF).
- Housekeeping repo (script temp `validation/_*.py`, inclusi `_env_timing.py` e
  `_gait_clock_absaction_smoke.py`), knowledge base letteratura, controllo SEA
  storici.
```
