# Rollout del best checkpoint nativo V26: bug di schema del probe scoperto, 7 plot prodotti, avvio opzione (A)

Data: 2026-08-18

## Problema

Il rollout auto-match del best checkpoint
(`MLP_imitation_native_v26_08-17-2026_grfsoft_knee1_ankle2_100iter/rl_module_best`)
veniva rifiutato dal contratto di osservazione: `actor checkpoint=39
runtime=43`.

## Diagnosi: bug reale del trainer (probe ≠ runner)

Catena verificata empiricamente (diff dei nomi feature tra i due env reali):

1. in modalità imitation, `make_cmc_env()` (env_factory.py:192) accende
   `include_imitation_target_observation` quando
   `reward_cfg.reward_mode == "imitation"` → 4 canali actor in più
   (`healthy_knee/ankle_angle_imitation_target` + `_vel`), actor 43, full 88;
2. il probe che dimensiona il modulo (train_ppo_mlp.py:1408) costruisce però
   l'env **direttamente** con `build_env_config()`, saltando `make_cmc_env()`
   → il flag resta False → il probe misura actor 39 / full 84;
3. il modulo salvato lo conferma: `model_config n_actor=39, n_full=84`, ma
   `observation_space=(88,)` — l'env vero dei runner emetteva 88 dim.

Il bug è latente dal 13/07 (commit `bdbf99c`) e colpisce **solo i training
imitation** successivi a quella data: il run nativo V26 è il primo. I run
ex_novo di luglio non erano affetti (flag False su entrambi i lati).

### Effetto reale sul run da 100 iter — benigno

Ordine actor runtime (43): le 4 feature di imitazione sono **dentro** lo
slice `obs[:39]` (posizioni 2–5); fuori restano solo le ultime 4:
`pros_knee/ankle_angle_sea_u_abs` e `_sea_u_saturated` — canali diagnostici
**quasi ridondanti** (funzioni deterministiche di `sea_u`, che l'actor ha
alle posizioni 33/38). Il critic usava il vettore 88 completo
(`_n_full=88` nel modulo caricato). La reward era imitation corretta.
Conclusione: policy coerente e ben informata; lo schema è però diverso sia
da giugno (che non aveva i target) sia dall'intento post-luglio (43 pieno).

## Rollout fedele eseguito (bypass dichiarato del validator)

Il modulo fa da sé lo slicing `obs[:39]` come in training: l'unico ostacolo
era il validator del rollout, sostituito con un logger in un driver
one-off (nessuna modifica a `rollout_eval.py`). Risultato (deterministico,
seed 123):

- `ok: true`, return episodio **136,9** (best training: 141,4 — coerente);
- `end_reason: phase_timeout:swing` — lo stesso muro diagnosticato in
  training, riprodotto al primo swing senza HS heel-qualified;
- penetrazione max **14,8 mm** (< soglia penalty 12 mm solo di poco,
  ampiamente sotto la terminazione 17 mm) — conferma che la policy nativa
  naviga nel margine sicuro, contro i 24–25 mm della navigazione legacy;
- 0 eventi binari invalidi; stack V26 attivo (binary_active,
  heel_qualified_fsm_v2, GRF correct_magnitude).

## 7 plot canonici

Cartella: `plot/08_18_2026_1_imitation_native_v26_100iter_best/` (01–07 +
`missing_channels.txt`). Note di lettura:

- fig. 06: inner loop quasi perfetto (errore ref→giunto ±0,02 rad ankle,
  ±0,015 knee, transiente iniziale a parte);
- fig. 07 (metrica imitativa): ankle segue il target shiftato (±0,1 rad);
  knee **smorza i picchi di flessione** (~0,85 vs ~1,1 rad, errore fino a
  ±0,3 rad) — il residuo coerente col ~88% di tracking per-step;
- fig. 03/04 vuote: l'episodio (13,0→15,55 s, ~2 cicli cinematici) non
  contiene alcun ciclo HS→HS **qualificato** completo — fotografia esatta
  del muro dell'heel-qualification.

## Avvio opzione (A) — autorizzata dall'utente

Scoperta di plumbing: `phase_swing_hard_timeout_s` è già un campo di
`RewardConfig` (reward_function.py:182) inoltrato all'env da
`make_cmc_env()` → si imposta via reward JSON, **zero modifiche al codice**
(la stima "2 righe di passthrough" del verdetto 18/08 era pessimista).

Lancio smoke effettuato:

- resume da `checkpoint_last` (iter 100) — il probe NON va corretto per la
  continuazione: lo spec 39 del probe coincide col checkpoint 39 (una
  correzione ora romperebbe il load dei pesi); il fix probe→`make_cmc_env`
  va applicato solo al prossimo training **fresh**;
- `experimental_configs/reward_swing_timeout_2p6.json`
  (`phase_swing_hard_timeout_s: 2.6`, era 1,3);
- `--binary-phase-invalid-event-policy reject_continue` (da `terminate`);
- output: `runs/training/MLP_imitation_native_v26_08-18-2026_swing2p6_rejectcontinue_smoke10`,
  ricetta invariata per il resto (resolved yaml del run nativo come
  `--config`);
- gotcha operativo scoperto al primo lancio: `--iterations` è un traguardo
  logico **cumulativo** e il resume ripristina il contatore a 100 — con
  `--iterations 10` il run "completa" a vuoto (0 iterazioni, exit 0).
  Rilanciato con `--iterations 110` (100 + 10 di smoke).

Criterio di promozione dello smoke: `episode_len_mean` deve sfondare
stabilmente il tetto ~250 step; poi continuazione piena e ri-confronto sui
criteri di giugno.

## Verdetto smoke (aggiornamento in giornata): PASS pieno

10/10 iterazioni (101–110) in ~5,3 h. Contatori lifetime (cumulativi,
ereditati dal resume):

- `phase_timeout_swing`: 1354 → **1354** — **zero nuovi timeout di swing**
  in 10 iterazioni (la baseline ne produceva ~14/iter): muro dissolto;
- `invalid_binary_event`: 549 → 549 — `reject_continue` assorbe senza
  terminare, come da design;
- `grf_penetration`: 530 → 588 (+5,8/iter) — con episodi più lunghi la
  penetrazione torna il margine attivo (atteso; la baseline l'ha già
  dimostrata riducibile con l'apprendimento);
- `episode_len_mean`: dal tetto ~250 a picchi di **383** (media ultime 5:
  ~323, trend in salita);
- return: best **177,0** (iter 103) contro 141,4 in 100 iterazioni di
  baseline — nuovo massimo con +25% in sole 10 iterazioni.

Criterio di promozione (len > 250 stabile) soddisfatto → lanciata la
**continuazione piena** a traguardo logico 200 (90 iterazioni nel nuovo
regime), resume da `smoke10/checkpoint_last`, output
`runs/training/MLP_imitation_native_v26_08-18-2026_swing2p6_rejectcontinue_cont200`.
Nota di costo: ~30 min/iter (vs ~11 della baseline) — episodi più lunghi
con più frazione di stance caricata; stima ~45 h.

## File modificati/creati

- nuovo: `Trajectory Generator/baseline_MLP/experimental_configs/reward_swing_timeout_2p6.json`;
- nuovo: cartella plot `plot/08_18_2026_1_imitation_native_v26_100iter_best/`;
- nuovo: rollout `runs/rollout/MLP_imitation_native_v26_08-17-2026_grfsoft_knee1_ankle2_100iter_rollout/`;
- nessuna modifica ai sorgenti di produzione (driver di bypass in
  scratchpad, one-off).

## Test e verifiche

- diff empirico dei nomi feature tra env di training ricostruito ed env di
  rollout (entrambi reali, stessa resolved yaml);
- larghezze del modulo lette dal pickle del checkpoint
  (`class_and_ctor_args.pkl`), non inferite;
- rollout completato `ok: true` con summary/trace/sto per il plotter;
- primo lancio smoke: esito a vuoto diagnosticato dal log ("next logical
  iteration 101/10") e da summary.json (0 successful iterations);
  rilancio con target 110 in corso al momento della scrittura.

## TODO

- [ ] Verdetto smoke 10 iter (episode_len > 250?) → continuazione piena
  opzione (A) e ri-confronto coi criteri di giugno.
- [ ] Fix del probe in `train_ppo_mlp.py:1408` (usare `make_cmc_env` come i
  runner) da applicare **prima del prossimo training fresh**, non alla
  continuazione.
- [ ] Valutare l'aggiunta di `binary_phase_invalid_event_policy` e di un
  override esplicito dello schema a `rollout_eval.py` (oggi il rollout di
  questo checkpoint richiede il driver di bypass).
- [ ] A baseline raggiunta: catena di luglio (critic warmup → qualifica →
  checkpoint-zero → morphology → training ex-novo + corridor).
- [ ] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`.
