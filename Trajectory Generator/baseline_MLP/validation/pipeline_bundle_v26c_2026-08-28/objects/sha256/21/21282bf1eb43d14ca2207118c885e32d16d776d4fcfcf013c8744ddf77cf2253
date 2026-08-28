# Audit di equivalenza statistica giugno ↔ fresh run nativo V26

Data: 2026-08-20

## Obiettivo (richiesta utente)

Prima di lanciare il nuovo training da 100 iterazioni: garantire che sia
«statisticamente la stessa cosa» del training di giugno
(`MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2_100iter`) — stessa
reward e stesse caratteristiche — con l'unica variabile sperimentale
dichiarata: lo stack detector binario V26. Non bit-a-bit.

## Metodo

Diff completo chiave-per-chiave dei `training_cfg.resolved.yaml` (giugno vs
lineage V26); per le 114 chiavi presenti solo nel yaml V26 (schema cresciuto
dopo giugno), confronto col **default runtime dell'epoca di giugno** letto
dal commit attivo il 23/06 (`407398a9`, 2026-06-19); verifica delle
larghezze reali dei moduli dai pickle dei checkpoint.

## Esiti — identico a giugno (verificato)

- **Reward effettiva identica**: composizione 0,8·served_imitation +
  0,2·imitation, pesi interni byte-uguali; TUTTI i gruppi aggiunti dopo
  giugno (phase_regularity, contact_load, morphology, pelvis, slip, …)
  sono gati a blend/peso **zero** → contributo nullo. I campi non
  esistevano nella RewardConfig di giugno.
- **PPO identico**: batch 4096, minibatch 512, 10 epoch, lr, clip, γ/λ
  uguali; `kl_coeff 0,2 / kl_target 0,01` espliciti oggi = default RLlib
  che giugno usava implicitamente (il train_ppo di giugno non aveva i flag).
- **Architettura e convenzione osservazioni identiche**: scoperto che il
  bug del probe (dimensionamento actor via `build_env_config` che salta
  `make_cmc_env`) **esisteva già a giugno**: modulo giugno `n_actor=31,
  n_full=76` su obs reali 80 — stessa convenzione di slicing che perde i 4
  canali `u_abs/u_saturated` in coda al blocco actor. Il 39 della lineage
  V26 = 31 di giugno + gli 8 canali FSM del detector (variabile
  dichiarata). **Decisione: il probe NON va corretto per questo run** —
  correggerlo introdurrebbe canali che giugno non aveva.
- 13 env runner, start episodi fisso a offset 1,0 (niente exact-start),
  stesso seed, stesse osservazioni controller/riferimento/GRF, stesso
  `pros_ref_model` e governor, orizzonte 5 s.
- Campi solo-V26 immateriali (bookkeeping/feature spente): exact_start off,
  rl_module standard, freeze off, morphology off, slew limit 0, ecc.

## Derive NON dichiarate scovate dall'audit (decise dall'utente)

1. **Soglie di penetrazione GRF**: giugno usava **20 mm penalty / 28 mm
   terminazione** (default dell'epoca, letti da `407398a9`); il codice
   attuale usa 12/17 — tutti i run nativi V26 hanno combattuto un vincolo
   ~40% più stretto senza che fosse mai stato deciso (probabile
   contributore primario del gap len/return e delle ~6 penetrazioni/iter).
   → **Decisione utente: 20/28 come giugno.**
2. **Profilo GRF online applicato**: giugno usava
   `online_full_wrench_residual_tangent_v2.json`, cancellato dal commit
   `bdbf99c` (13/07); i run nativi usavano `grf_correct_magnitude.json`.
   → **Decisione utente: struttura a due livelli** — profilo APPLICATO
   (sostegno, gamba sinistra) = quello di giugno, **ripristinato da git
   byte-identico** (SHA-256
   `09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0`);
   profilo di DETECTION = il binario validato attuale
   (`grf_detector_HS-TO.json` + profilo geometrico V25), intoccato.
   Nota di rischio residuo: il detector binario è geometry-only
   (force-free), ma la heel-qualification legge le forze applicate → tarata
   nell'era correct_magnitude; verifica dedicata nello smoke.

   **Verifica aggiuntiva su domanda utente (esito notevole)**: quale
   profilo applicato usa oggi l'area ex-novo? TUTTI i run ex-novo (dal
   primo, 29/06, fino all'esplorativo del 16/08) e l'intera filiera del
   detector usano `grf_correct_magnitude.json` — che risulta
   **byte-identico** al `tangent_v2` di giugno (stesso SHA-256
   `09e04ab9…`): il commit `bdbf99c` (13/07) lo aveva semplicemente
   rinominato. Conseguenze: (i) nessuno shift di dominio GRF applicato in
   NESSUN punto della catena giugno → imitazione nativa → ex-novo; (ii) il
   rischio di taratura della heel-qualification decade (stesse forze per
   costruzione); (iii) la "recipe delta" sul profilo dichiarata nei run
   nativi era in realtà un no-op. Il fresh run può usare indifferentemente
   i due nomi: si usa `tangent_v2` (nome di giugno) per fedeltà di
   provenienza.

## Compensazioni V26 confermate (già concordate, reward-neutrali vs giugno)

- `phase_swing_hard_timeout_s` 1,3 → 2,6 (giugno: 0 timeout mai scattati →
  la soglia non incideva sul suo training);
- `binary_phase_invalid_event_policy = reject_continue` col drop-mode
  corretto (concetto inesistente a giugno: gli eventi legacy non potevano
  essere invalidi).

## Esecuzione

- Smoke di equivalenza da 5 iterazioni lanciato
  (`MLP_imitation_native_v26_08-20-2026_june_equiv_smoke5`): config = yaml
  di giugno + stack V26 + le due decisioni. Criteri: heel-qualification
  viva sotto le forze di giugno (HS qualificati, drop di eventi invalidi
  non patologici), 0 off-grid, ~8 min/iter.
- A smoke OK: fresh run 100 iterazioni, stessa config, `--iterations 100`.

## TODO

- [ ] Verdetto smoke equivalenza (in corso).
- [ ] Fresh run 100 iter + confronto formale finale coi criteri di giugno.
- [ ] Fix probe `train_ppo_mlp.py:1408`: rinviato deliberatamente (rompe
  l'equivalenza di schema con giugno); da riconsiderare per le lineage
  future con migrazione di schema dichiarata.
- [ ] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`.
