# Training esplorativo ex-novo con V26 + corridor e warm-start H0 — ABORT a 31/50

Data: 2026-08-16

## Problema

Misurare empiricamente cosa produce il regime di training del 15/07 con le
nuove feature montate (detector binario V26 attivo, Morphology Corridor
causale a peso 0,0025) usando come warm-start l'H0 canonico **non adattato**
a V26 — dato che il candidato adattato non esiste ancora (V12R12 terminale al
fit P0). Run dichiaratamente **esplorativa e fuori dal protocollo
fail-closed**: nessuna one-shot consumata, nessun namespace protetto toccato,
risultati non qualificabili.

## Setup

- comando: `train_ppo_mlp.py` diretto (il launcher morphology validato è
  fail-closed sull'handoff), config = resolved YAML del pilot 15/07
  (12 EnvRunner, 3 exact-start, batch 4.608, minibatch 512, lr 5e-7,
  γ 0,99, λ 0,9, clip 0,05), `--warm-start` (full checkpoint H0 canonico
  `checkpoint_last` del 13/07), 50 iterazioni richieste;
- detector: `binary_active`, profilo V25, contratto
  `binary_point_v25+heel_qualified_fsm_v2` (V26), sampling 1 ms,
  debounce 5 ms;
- corridor: `event_anchored_causal_delayed_experimental`, peso 0,0025,
  delay 40 ms, effetti causali abilitati, hard termination spenta — **senza**
  il fix strict-delay (iniettabile solo dal launcher gated);
- nota tecnica: primo lancio fallito per bug del trainer (`--reward-json`
  con JSON inline lungo → `Path.exists()` → `OSError Errno 63` su macOS);
  risolto passando il JSON come file
  (`runs/training/2026-08-16_exploratory_v26_corridor.reward.json`).

## Esito operativo

- warm-start ripristinato correttamente (ripartenza dalla logical iteration
  2/50, come il pilot 15/07);
- **31/50 milestone completate in ~6 h**, poi
  `Training ABORTED — exceeded 1 consecutive child crash restart(s) without
  checkpoint progress`;
- **7 worker uccisi** dall'adapter V26 fail-closed, tutti con la stessa firma:
  `ValueError: Actor FSM rejected a V20 active event —
  invalid_event_type=to_too_early_after_hs, state_name=STANCE_AFTER_HS`
  (identica al salvage P1 del 10/08 e alla deriva R6/R10/R11);
- **5 crash del child**: i frammenti persi dai worker morti sbilanciano il
  batch e la guardia dell'exact-interleaving (protocollo 14/07) ferma
  l'iterazione (es. `got [3072, 3072, 2304]`); il supervisore ha riavviato
  dal checkpoint finché due crash consecutivi senza progresso (iterazione 32)
  non hanno chiuso il run.

## Risultati (30 iterazioni utili, ~142k step campionati)

| Metrica | Inizio (it. 2) | Fine (it. 31) | Trend |
|---|---:|---:|---|
| Return medio episodio | −3,24 | −3,06 | piatto (medie 1ª/2ª metà: −3,39 / −3,41) |
| Lunghezza media episodio | 85,8 step | 84,5 step | piatto |
| Terminazioni per penetrazione GRF | — | **2.138 cumulative** (~70/iterazione) | costante |
| Terminazioni per divergenza ginocchio | — | 180 cumulative | costante |

Per-start (stabili per tutta la run):

| Start | Lunghezza media | Return medio |
|---|---:|---:|
| −0,20 (1,757 s) | 55 step (~0,6 s) | −2,66 |
| nominale (1,957 s) | **37 step (~0,4 s)** | −2,32 |
| +0,20 (2,157 s) | 160 step (~1,6 s) | −4,06 |

Corridor: configurato e attivo (chiavi presenti nella config risolta), ma
nessuna metrica morphology separata nel log di training; con episodi da
0,4–1,6 s il corridor event-anchored ha avuto pochissimi segmenti HS→TO
completi su cui ancorarsi. Il suo contributo non è distinguibile in questa
run.

## Diagnosi

1. **H0 non adattato collassa su V26 su tutti e tre gli start** — non solo
   sul `+0,20`: il nominale è il peggiore (0,4 s). I rollout storici che
   completavano 500 step su nominale/seed usavano il candidato R6
   (V26-adattato); H0 puro su V26 non era mai stato misurato fuori dal caso
   `+0,20`. Ora sappiamo che l'incompatibilità è globale.
2. **PPO a lr 5e-7 × 30 update non muove nulla**: KL per update ~3–5e-4,
   nessun trend su return o lunghezza. Come previsto: è una lucidatura, non
   un salvataggio.
3. **Il regime si rompe anche operativamente**: la catena
   evento-invalido → worker morto → batch sbilanciato → guardia
   interleaving → crash child → restart supervisore rende il training
   instabile per costruzione, indipendentemente dall'apprendimento.
4. Conclusione: **la condizione del 15/07 + nuove feature NON è
   raggiungibile senza il warm-start adattato a V26.** La run è la
   controprova empirica, lato training, di ciò che V12R11 aveva dimostrato
   lato rollout. Il percorso resta: successore imitativo V26-only
   (V12R13) → P3 fisico → Q3 → checkpoint-zero → morphology A/B → questo
   stesso comando di training, con `--resume-from checkpoint_zero` al posto
   di `--warm-start`.

## File prodotti

- run root:
  `Trajectory Generator/runs/training/2026-08-16_exploratory_exnovo_v26_corridor_h0ws_50iter/`
  (31 milestone, `checkpoint_best/last`, `train_iterations.jsonl`,
  `summary.json`, `supervisor_state.json`, `faulthandler.log`);
- reward JSON:
  `Trajectory Generator/runs/training/2026-08-16_exploratory_v26_corridor.reward.json`;
- questo report.

Nessun file di protocollo, namespace `v12r*`, plugin o sorgente di
produzione è stato modificato. Il bug `Errno 63` di `--reward-json` in
`train_ppo_mlp.py` resta da correggere (workaround documentato).

## Test e verifiche

- warm-start, detector V26 e corridor verificati attivi dalla config risolta
  e dal comportamento runtime (eventi V26 nei log worker);
- crash forensi: traceback dei worker letti dai log di sessione Ray,
  firma unica `to_too_early_after_hs`;
- metriche estratte da `train_iterations.jsonl` (30 righe valide).

## TODO

- [ ] Correggere in `train_ppo_mlp.py` la gestione di `--reward-json` inline
  lungo (catch di `OSError` in `_load_reward_json_for_run_name`).
- [ ] Riprendere il percorso qualificato: autorizzare V12R13 (design V12R12,
  gate offline ricalibrati sui valori dimostrati, metriche persistite
  pre-gate) e portarla fino al P3 fisico.
- [ ] Valutare, nel design del successore, il dato nuovo di questa run: la
  fragilità dipende dalla fase di partenza (nominale peggiore del `+0,20`
  in condizioni di training) — i casi di development/Q3 coprono già start
  diversi, ma il P1 candidate-exposed potrebbe pesare anche il nominale.
- [ ] Trattare i risultati della run come diagnostici: checkpoint e modulo
  `rl_module_best/last` non sono candidati e non vanno promossi né riusati
  come warm-start.
