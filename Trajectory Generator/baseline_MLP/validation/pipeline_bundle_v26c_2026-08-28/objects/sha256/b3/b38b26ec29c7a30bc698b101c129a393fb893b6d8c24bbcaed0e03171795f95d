# Report approfondito: dal training imitativo nativo V26 alla FSM attore v3 (17–21 agosto 2026)

Data: 2026-08-21

Questo report ricostruisce, in ordine causale, tutto il lavoro svolto dal
lancio del primo training imitativo con il detector binario V26 (17/08)
fino alla validazione e promozione della FSM attore v3 (21/08): cosa è
stato fatto, cosa è stato scoperto, cosa è cambiato nel codice, quali
numeri lo dimostrano e cosa resta aperto. I report giornalieri citati
contengono i dettagli; qui si dà il filo conduttore.

## 0. Punto di partenza e obiettivo

Obiettivo dichiarato dall'utente: tornare alla condizione *training-ready*
del 15/07 — training ex-novo con warm start imitativo — aggiungendo
(1) detector binario GRF V26 attivo e funzionante e (2) morphology corridor
attivo e validato. La via dell'adattamento della vecchia baseline (lineage
H0/V12) si era rivelata un problema di recovery fuori supporto senza
teacher; l'intuizione dell'utente fu di **rifare il training imitativo
nativamente sotto V26**, aspettandosi lo stesso risultato di giugno.

## 1. Primo training imitativo nativo V26 (17–18/08)

- Modifica runtime abilitante: policy eventi invalidi
  (`raise`/`terminate`/`reject_continue`); smoke 5 iter, poi run 100 iter,
  13 worker, ricetta di giugno + stack V26, policy `terminate`.
- Esito: 100/100 senza crash; tracking per step ≈ 88 % di giugno ma
  episodi dimezzati (~250 step) da un muro unico: `phase_timeout_swing`
  (1,3 s senza HS heel-qualified). Penetrazione appresa via (248 → 58 per
  blocco).
- Rollout del best e 7 plot (`plot/08_18_2026_1_...`): fig. 06 inner loop
  quasi perfetto; fig. 07 picchi del ginocchio smorzati; fig. 3/4 vuote
  (nessun ciclo HS→HS qualificato completo).
- **Bug scoperto**: il probe di `train_ppo_mlp` dimensiona l'actor con
  `build_env_config()` saltando `make_cmc_env()` → modulo a 39 input su env
  a 43 (i 4 target di imitazione). Effetto benigno (perde solo `u_abs`/
  `u_saturated`, ridondanti), ma il rollout richiede un bypass del validator.
  Scoperto poi che **lo stesso bug esisteva a giugno** (modulo 31 su 35):
  la convenzione di slicing è parte della "stessa cosa" di giugno, quindi
  il probe NON va corretto per i training imitativi comparabili.

## 2. Opzione (A): timeout swing 2,6 s + reject_continue (18–19/08)

- `phase_swing_hard_timeout_s` passa già via reward JSON (zero codice).
- Smoke 10 iter: "PASS" apparente (len fino a 383, best 177) →
  continuazione a 200 (iter 111→128 poi fermata dall'utente).
- **Il verdetto dello smoke era parzialmente sbagliato**: i contatori
  "0 timeout / 0 invalidi" erano congelati perché quegli episodi
  **uccidevano il worker** invece di terminare.

## 3. Indagine iterazioni lente: la crash-storm (19/08)

Requisito utente: 8–15 min/iterazione come a giugno (la continuazione
andava a 36). Evidenze: costo per step invariato (1,18 vs 1,35 s), reset
in ms, learner in ms, utilizzo 25 % vs 98 %; watch dal vivo: 1–2 processi
al 99 % (rebuild di env) e il resto fermo; **log Ray: 1.044 errori
off-grid, 639 file worker** (~30 morti/iterazione, silenziose per
`ignore_env_runner_failures`).

Due bug, entrambi corretti con test (9/9) e certificati (7,7 min/iter,
0 off-grid su 72 iterazioni successive):

1. **Mio**: l'assorbimento `reject_continue` avanzava il cursore sensori
   senza committare le FSM candidate → `previous_time_s off-grid` allo
   step successivo → morte garantita del worker. Fix: `invalid_event_mode=
   "drop"` nell'adapter (commit dei candidati window-consistent, evento
   non pubblicato) e un unico percorso di commit nell'env.
2. **Latente V26**: microstep degenere all'orizzonte (accumulo float su
   500 step contro tolleranza 1e-12) rifiutato dal trasporto binario. Fix:
   test dell'orizzonte a mezza finestra di policy.

Spiegazione dell'overshoot di campionamento (7562 = 2×3781): un worker
morto a metà round fa rilanciare un round intero.

## 4. Continuazioni 132→200 e verdetto (19/08)

Pulite (0 incidenti), best 196,4 a iter 162, poi plateau; equivalenza con
giugno NON raggiunta (65 % return, 84 % len): storia sporca della lineage
(100 iter strozzate + 28 di crash-storm). Rollout del best: orizzonte pieno,
return 210,7, ma fig. 07 con ginocchio smorzato e fig. 03 con caviglia
scarica (niente push-off).

## 5. Audit di equivalenza giugno ↔ fresh run (20/08)

Diff chiave-per-chiave dei resolved yaml + default dell'epoca di giugno
(commit `407398a9`) + larghezze reali dei moduli. Esiti: reward effettiva
identica (0,8 served + 0,2 imitation; i gruppi aggiunti dopo sono a blend
zero), PPO identico, convenzione osservazioni identica. **Due derive
scovate e decise dall'utente**: soglie di penetrazione **20/28 mm**
(giugno) contro 12/17 mai decise; profilo GRF applicato: il file di giugno
(`tangent_v2`, ripristinato da git) — poi scoperto **byte-identico** a
`grf_correct_magnitude` dell'area ex-novo (stesso SHA-256: era un rename).
Smoke di decollo sovrapponibile a giugno.

## 6. Fresh run june-equivalent: equivalenza raggiunta (20/08)

100/100 in 14,4 h, 0 off-grid: ultime 20 iter return 257,4 vs 268,6 (96 %),
len 494 vs 476 (104 %), best 271,0 vs 293,6; curve incrociate; metà delle
penetrazioni di giugno. Rollout del best (iter 87): return 363,8,
orizzonte pieno, tutte le 7 figure piene (primi cicli qualificati):
smorzamento del ginocchio sparito. Spiegazione dei tempi (14,4 vs 11,8 h):
43 morti worker nel transitorio iniziale (guardia 30 mm a 2 mm dalla
terminazione 28 mm) + 5 % per-step V26; a regime 7,7 min/iter.
Coppia SEA positiva alla caviglia in loading response: adattamento indotto
dalla heel-qualification (100 % entro 0,5 s da un HS), accettato come
baseline dall'utente.

## 7. Catena B0820 → training-ready (20/08)

Guardie di processo promosse a terminazioni (30 mm → `grf_penetration_hard`;
incoerenze latch → `BinaryPhaseTransferError` terminale; 9/9 test). Poi:
manifest actor (39 nomi + digest) → critic warmup 5 iter (actor congelato,
`vf_loss` 8,86 → 0,07) → qualifica 3 start (3/3 fisica sicura, ma
stance-lock a ~2,5–3 s) → checkpoint-zero (= warmup) → corridor readiness
8/8 → preflight smoke 3 iter (0 crash, interleaving 3/3, 14 episodi a
orizzonte). **Training-ready dichiarato**; runbook riproducibile scritto;
canonico `training_exnovo_cfg.yaml` promosso al contenuto B0820 (scelta
utente b).

## 8. Training ex-novo 50 iter e il limite vero (21/08)

Pulito (7,5 h, 0 incidenti) ma **piatto**: return −18 → −35, muri
costanti (~5,4 stance-timeout e ~5,6 contract-failure/iter); rollout del
best allo start nominale identico al pre-training; figura corridoio tutta
tratteggiata. Riformulazione: il pilot di luglio del 15/07 ebbe lo stesso
esito tecnico (50 update, milestone respinte) ma il suo H0 *già camminava*
agli start esatti grazie all'adattamento supervisionato — passaggio che
oggi abbiamo saltato a ragione (non serve più per lo schema) ma che
insegnava anche gli start.

## 9. FSM attore v3: la soluzione strutturale (21/08)

L'utente rifiuta il tampone (fine-tune della policy sugli start) e chiede
robustezza architetturale a start casuale e eventi non nominali. Design
approvato (dwell 80 ms, rimbalzo penalizzato, ciclo con resync senza
bonus): resync su contraddizione col **latch funzionale del detector**,
cancellazione HS rimbalzato, bootstrap verificato. Validazione a 4 livelli:

- L0 23/23 (golden v2 byte-identico); L1 replay fedele 7/7 con desync
  1,9–2,1 s → ≤ 0,08 s; L2 23 start × 2 versioni senza un errore, v3 ≡ v2.
- L3 ha smascherato **due strati sotto la FSM**: il ledger causale del
  corridoio (a) non accettava le transizioni di riparazione → **ri-armo**
  implementato; (b) **bug preesistente**: tolleranza temporale 1e-12 su due
  orologi float indipendenti (Δ misurato 1,1e-12) → ogni candidato
  pendente coincidente con un confine era un contract failure spurio —
  causa verosimile del 43 % di terminazioni per contract failure anche in
  v2 e di gran parte delle 298 del training da 50 iter. Fix: 1e-9.
- Risultato finale: L3a 3/3 start a orizzonte pieno; L3b v3: contract
  failure +0, stance timeout +0, +20 episodi a orizzonte (v2 sullo stesso
  corridoio corretto: +5 / +12 / +7 — serve la combinazione).

Decisioni applicate: v3 default (EnvCfg, CLI, canonico), candidato
corridoio V26 **promosso post-Q2** con test di governance riscritto (guard
runtime preservato), nessun training lanciato. Repo a zero test rossi.

## 10. Stato del codice (file chiave toccati nell'arco)

`osim_trj_cmc_like.py` (invalid-event policy, guardie, orizzonte,
versione FSM), `binary_phase_adapter*.py` (drop-mode, latch passthrough,
raise tipizzati), `prosthetic_phase_fsm.py` (v3), `simulation_runner.py`
(eccezione penetrazione tipizzata), `experimental_morphology_corridor.py`
(repair whitelist, ri-armo, tolleranza), `train_ppo_mlp.py` /
`rollout_eval.py` / `training_config.py` (flag e propagazioni, telemetria
causale nei trace), config canonico e snapshot B0820, candidato V26
promosso; nuove suite in `validation/` (policy eventi invalidi, guardie,
FSM v3, ledger causale) e strumenti (replay, sweep, plot corridoio).

## 11. Cosa resta aperto

- Training ex-novo B0820 con v3 + corridoio corretto: la prima misura
  onesta della ricetta (i 50 update precedenti erano troncati dal bug di
  tolleranza). Da lanciare su tua decisione.
- Ricetta ex-novo: lr 5e-7 × 1 epoca era tarata sulla fragilità di H0; la
  baseline B0820 ha margini ampi — candidata a un regime più deciso sotto
  KL-guard (da decidere coi numeri del primo run pulito).
- Follow-up v3: riportare lo swing timeout da 2,6 a 1,3 s.
- Probe `train_ppo` (39/43): correggere solo con migrazione di schema
  dichiarata (lineage futura), non per i training comparabili con giugno.
