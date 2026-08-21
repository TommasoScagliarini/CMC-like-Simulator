# Runbook: dalla baseline imitativa al training ex-novo con warm start (catena standard, riproducibile)

Data: 2026-08-20 — prima esecuzione: lineage **B0820** (baseline
`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`).

Questo documento è la procedura standard per trasformare QUALSIASI baseline
imitativa (nuovo training, fine-tune, reward modificata) in uno stato
training-ready per l'ex-novo con warm start. La proprietà chiave: la catena
ha **un solo input** (il checkpoint della baseline + il suo resolved yaml);
tutto il resto è derivato o pinnato nel config di catena.

## Prerequisiti (una tantum, già in essere dal 20/08)

1. **Baseline nativa**: la baseline DEVE essere addestrata nello stesso
   ambiente del runtime ex-novo (stack detector V26, stesso schema
   osservazioni). Una baseline di dominio diverso riapre il problema di
   adattamento di luglio (markov35/DAgger) — evitato per costruzione.
2. **Guardie di processo come terminazioni** (commit del 20/08): guardia
   30 mm → `end_reason=grf_penetration_hard`; incoerenze latch/fase V26 →
   `BinaryPhaseTransferError` terminale. Suite:
   `validation/test_process_guards_termination.py` (3 test) +
   `validation/test_binary_invalid_event_policy.py` (6 test).
3. **Passthrough del policy-flag nel rollout**: `rollout_eval.py` espone
   `--binary-phase-invalid-event-policy` e lo eredita dal yaml via layered
   defaults (la qualifica gira con la stessa semantica del training).

## Input della catena

- `BASELINE_MODULE = <run>/rl_module_best` (actor 2×256 tanh, convenzione
  slice-39 della lineage giugno);
- `CHAIN_YAML`: il **canonico editabile** è
  `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml` (promosso
  al contenuto B0820 il 20/08 su decisione utente); ogni lineage ne
  congela una copia in `experimental_configs/` al freeze della catena
  (B0820: `exnovo_v26_B0820_chain.yaml`). La ricetta ex-novo completa con
  i delta dichiarati della lineage:
  penetrazione 20/28 mm, `phase_swing_hard_timeout_s: 2.6`,
  `binary_phase_invalid_event_policy: reject_continue`, profilo GRF
  applicato = file di giugno (SHA `09e04ab9…`), exact-start sui tre start
  (1.7568709838 / 1.9568709838 / 2.1568709838), batch 4608, minibatch 512,
  1 epoca, lr 5e-7, KL guard 0.01, logstd frozen, corridor
  `morphology_weight 0.0025` con profilo event-warped.
  Per una nuova lineage: copiare il yaml, aggiornare l'header di
  provenienza e gli eventuali delta dichiarati.

## Stadio 0 — Manifest delle feature actor (se assente)

Il trapianto richiede `actor_feature_manifest.json` accanto al modulo
sorgente (39 nomi nell'ordine runtime + `actor_digest`). Se il run di
baseline non lo ha prodotto, generarlo: nomi = slice `[0:39]` dell'ordine
actor runtime (verificabile con un probe env → `actor_feature_names`),
digest = `warm_start.actor_state_digest(module_state)`. Senza manifest il
resolver ripiega sul manifest builtin di giugno (31 nomi) e fallisce con
"31 names vs shape (256, 39)".

## Stadio 1 — Critic warmup (actor congelato)

```
train_ppo_mlp.py \
  --config <CHAIN_YAML> \
  --warm-start-raw \
  --warm-start-raw-source <BASELINE_MODULE> \
  --freeze-actor --freeze-logstd \
  --iterations 5 \
  --output-dir validation/critic_warmup/<data>_<lineage>_native_v26_frozen_actor_iter5
```

Cosa fa: trapianta l'actor in un Algorithm ex-novo fresco (critic,
optimizer e contatori nuovi), congela actor+logstd e allena il critic
sulla reward ex-novo per 5 iterazioni (luglio ne usava 1; 5 è il valore
dichiarato della catena standard, ~50 min).

**Gate**: run 5/5 senza crash; `actor_transplant_report.json` presente;
`vf_loss` in discesa netta (B0820: 8,86 → 0,07); l'audit di freeze è
fail-closed (se l'actor deriva di un byte il run abortisce da solo).
Return negativi sono ATTESI (reward ex-novo su policy imitativa).

## Stadio 2 — Qualifica fisica sui tre start esatti

Tre rollout deterministici puri del modulo di warmup sull'env di catena
(driver: `scratchpad/run_qualification_B0820.py`, parametrico su
checkpoint/config/offset — da promuovere a script versionato alla prossima
esecuzione):

```
rollout_eval.py --checkpoint <warmup>/rl_module_last --config <CHAIN_YAML> \
  --episode-start-offset-s {1.7568709838 | 1.9568709838 | 2.1568709838} \
  --name qual_<lineage>_<start>
```

**Gate dichiarati**: per ogni start — `ok: true`, nessun crash worker,
penetrazione max < 28 mm, eventi invalidi assorbiti (non fatali).
Il completamento dell'orizzonte NON è un gate del warm start: è
l'obiettivo del training ex-novo.

Esito B0820: 3/3 PASS fisico (penetr. 11,6/17,1/22,1 mm; 1 evento
assorbito per start). Nota strutturale osservata: dopo il primo HS la
policy imitativa stacca presto → `to_too_early_after_hs` (droppato) → la
FSM resta in stance → `phase_timeout:stance` a ~2,5–3 s. È il punto in cui
le blend di fase della reward ex-novo devono lavorare.

## Stadio 3 — Checkpoint-zero

`<warmup>/checkpoint_last` **È** il checkpoint-zero (actor baseline
byte-identico + critic caldo + stato Algorithm completo). Stessa
convenzione del canonico H0 di luglio (`validation/critic_warmup/...`).
Nessun passo aggiuntivo; la provenienza è nel transplant report.

## Stadio 4 — Morphology corridor

- **4a, di macchina** (per env-change, non per baseline):
  `pytest "Trajectory Generator/baseline_MLP/validation/test_morphology_corridor_v26_readiness.py"`
  → B0820: 8/8 PASS.
- **4b, comportamentale**: coperto dal preflight (stadio 5) con corridor
  attivo a peso 0,0025 e telemetria corridoio nel jsonl.

## Stadio 5 — Preflight smoke e dichiarazione training-ready

```
train_ppo_mlp.py --config <CHAIN_YAML> \
  --resume-from <warmup>/checkpoint_last \
  --iterations 8 \
  --output-dir runs/training/MLP_ExNovo_<lineage>_preflight_smoke3
```

(3 iterazioni ex-novo vere: actor sbloccato dal yaml, logstd frozen,
exact-start attivo, corridor attivo. `--iterations` è un traguardo logico
CUMULATIVO: il warmup ha consumato 1–5, quindi 8 = 3 iterazioni nuove.)

**Gate**: 3/3 iterazioni senza crash né errori nei log Ray della sessione;
batch exact-start bilanciato (i learner-check di interleaving sono
fail-closed nel trainer); metriche corridoio presenti; nessuna esplosione
KL (guard 0.01).

Esito B0820: vedi report di risultato
(`2026-08-20_risultato_training_ready_exnovo_B0820.md`).

## Comando finale (il "training-ready")

```
train_ppo_mlp.py --config <CHAIN_YAML> \
  --resume-from <warmup>/checkpoint_last \
  --iterations <N> \
  --output-dir runs/training/MLP_ExNovo_<lineage>_<descrizione>
```

## Modi di guasto incontrati alla prima esecuzione (e loro fix, già a repo)

1. Manifest actor mancante → generato (stadio 0) — errore chiaro
   "31 names vs shape (256, 39)".
2. `rollout_eval` senza passthrough del policy-flag → primo evento
   invalido organico fatale in qualifica → aggiunto flag + layered default.
3. (Ereditati dalla settimana, già chiusi: drop-mode del cursore,
   microstep all'orizzonte, guardie di processo.)

## Cosa si ripaga cambiando baseline, cosa no

- **Si riesegue** (policy-dipendente): stadi 0–3 e 5 (~mezza giornata di
  macchina).
- **Non si tocca**: chain yaml (salvo delta dichiarati), fix runtime,
  validazione detector, corridor di macchina (4a), scripts e gate.
