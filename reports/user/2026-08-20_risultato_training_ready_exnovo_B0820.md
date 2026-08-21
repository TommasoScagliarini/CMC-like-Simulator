# Training-ready ex-novo con warm start imitativo nativo V26 — lineage B0820

Data: 2026-08-20

## Problema

Portare la nuova baseline imitativa nativa V26
(`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`, equivalente a
giugno al 96%) fino allo stato training-ready per l'ex-novo con warm start
— l'obiettivo del 15/07 — con detector binario V26 attivo e morphology
corridor attivo e validato. Decisioni utente incassate: comportamento
caviglia accettato come baseline; involucro 20/28 mm; guardie di processo
promosse a terminazioni pulite.

## Soluzione e strategia

Catena a 6 stadi (runbook dedicato:
`2026-08-20_runbook_catena_warm_start_imitativo_exnovo.md`), eseguita e
collaudata in giornata:

1. **Fix guardie** (pre-stadio): 30 mm → `grf_penetration_hard`
   (terminazione MDP, mai worker-fatal); incoerenze latch/fase V26 →
   `BinaryPhaseTransferError` terminale. Suite 9/9 PASS.
2. **Manifest actor** della baseline generato (39 nomi, convenzione
   slice della lineage giugno, digest SHA dei pesi).
3. **Critic warmup** (5 iter, actor+logstd congelati, audit fail-closed):
   PASS — `vf_loss` 8,86 → 0,07, actor byte-identico, transplant report
   completo. Il suo `checkpoint_last` è il **checkpoint-zero**.
4. **Qualifica fisica** dei 3 start esatti (rollout deterministici puri,
   semantica reject_continue come il training): **3/3 PASS fisico** —
   penetrazione max 11,6 / 17,1 / 22,1 mm (< 28), 1 evento invalido
   assorbito per start, zero crash. Nota: cicli del passo ancora
   degeneri agli start ex-novo (TO precoce → stance timeout a ~2,5–3 s) —
   è il lavoro del training. Confronto storico: l'H0 di luglio moriva a
   71 step per penetrazione sullo start −0,20.
5. **Corridor**: readiness di macchina 8/8 PASS.
6. **Preflight smoke** (3 iterazioni ex-novo vere dal checkpoint-zero,
   actor sbloccato, exact-start, corridor a 0,0025): PASS — 0 errori
   off-grid, 0 worker morti, interleaving 3/3 start in ogni batch, KL
   quieto, **14 episodi già a orizzonte pieno**; muri policy-dipendenti
   attesi in avvio: 36 stance-timeout e 34 `morphology_causal_contract_failure`
   (cicli non qualificati che rompono l'ancoraggio causale del corridoio —
   destinati a recedere col training, da monitorare).

## Stato: TRAINING READY

Comando del training ex-novo (lineage B0820):

```
train_ppo_mlp.py \
  --config "Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml" \
  --resume-from "validation/critic_warmup/2026-08-20_B0820_native_v26_frozen_actor_iter5/checkpoint_last" \
  --iterations <N logico cumulativo: 8 gia' consumati> \
  --output-dir "Trajectory Generator/runs/training/MLP_ExNovo_B0820_<descrizione>"
```

(`training_exnovo_cfg.yaml` = canonico editabile, promosso al contenuto
B0820 il 20/08; lo snapshot congelato della lineage resta
`experimental_configs/exnovo_v26_B0820_chain.yaml`.)

Le tre condizioni dell'obiettivo sono soddisfatte: (1) warm start
imitativo — actor della baseline nativa trapiantato e auditato; (2)
detector binario V26 attivo e funzionante (heel-qualified, drop-mode,
zero incidenti su tutta la catena); (3) morphology corridor attivo e
validato (readiness 8/8 + telemetria viva nel preflight).

## File modificati/creati oggi (catena)

- `simulation_runner.py` (+`OnlineGRFPenetrationLimitExceeded`),
  `Trajectory Generator/osim_trj_cmc_like.py` (catch tipizzato +
  `grf_penetration_hard` + catch terminale BinaryPhaseTransferError),
  `Trajectory Generator/binary_phase_adapter_v26.py` (3 raise tipizzati);
- `Trajectory Generator/baseline_MLP/rollout_eval.py` (passthrough
  `--binary-phase-invalid-event-policy`);
- `validation/test_process_guards_termination.py` (nuovo, 3 test);
- `experimental_configs/exnovo_v26_B0820_chain.yaml` (config di catena);
- manifest actor nei moduli della baseline;
- artefatti: `validation/critic_warmup/2026-08-20_B0820_native_v26_frozen_actor_iter5/`
  (checkpoint-zero), rollout di qualifica `qual_B0820_*`,
  `runs/training/MLP_ExNovo_B0820_preflight_smoke3/`.

## Test e verifiche

- suite guardie+policy 9/9; corridor readiness 8/8; ruff pulito sui file
  toccati;
- ogni stadio con esito letto da artefatti reali (transplant report,
  rollout summary, jsonl, log Ray);
- 0 errori off-grid e 0 worker morti su warmup, qualifica e preflight.

## TODO

- [ ] Lancio del training ex-novo B0820 (durata/N a scelta utente).
- [ ] Durante il run: monitorare la discesa di `phase_timeout_stance` e
  `morphology_causal_contract_failure` (i due muri policy-dipendenti).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato
  alla prossima esecuzione della catena.
- [ ] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`.
