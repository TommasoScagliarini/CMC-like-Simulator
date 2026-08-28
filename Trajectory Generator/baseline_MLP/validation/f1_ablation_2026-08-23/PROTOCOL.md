# F1 — Protocollo preregistrato: ablation causale della regressione 39→35 e della scala stocastica

**Protocol id:** `F1-S1-ablation-39to35-sigma-v1` (sorgente machine-readable: `f1_protocol.json`, unica fonte delle soglie; il suo SHA-256 è registrato nel manifest di dry-run e in ogni receipt)
**Data di preregistrazione:** 2026-08-23, prima di qualunque rollout, simulazione o fit F1
**Piano:** `reports/plans/2026-08-22_piano_operativo_recupero_pipeline_exnovo_v26_fsmv3_morphology.md`, Fase 1
**Gate di ingresso:** F0 PASS tecnico (`reports/user/2026-08-23_fase0_gate_finale_recupero_ab06.md`)
**Ruoli:** esecutore operativo Claude Fable 5 (max); architetto e gate owner Codex

## 1. Domanda e disegno

La regressione della lineage B0820 nasce, secondo la diagnosi F0, dal trapianto 39D→35D senza refit target-domain (4 target privilegiati eliminati, 2 colonne clock azzerate) e dalla scala stocastica ereditata (sigma ≈ 0,5 state-dependent contro 0,005 costante di luglio). F1 isola i due meccanismi senza PPO con cinque candidati sotto lo **stesso runtime di B** (`v3_canonical`: resolved yaml della run `MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter`, SHA `a870cc38…`):

| Candidato | Actor | Media | Sigma | Driver | Classe |
|---|---|---|---|---|---|
| **A_iso39_v3** | V26 nativo 39D (`rl_module_best`, frozen) | originale | nativa (solo det nei gate) | `f1_rollout_aiso` + adapter `aiso4` | isometrico privilegiato |
| A_iso39_v3_s005 | V26 con testa log-std costante (derivato stage 1) | = V26 | 0,005 | idem | isometrico privilegiato |
| **B** | B0820_H0 `rl_module_last` (actor = trapianto hard-drop, bit-exact) | hard-drop | nativa state-dependent | `rollout_eval.py` | isometrico |
| **C** | B con testa log-std costante (derivato stage 1) | = B | 0,005 | `rollout_eval.py` | isometrico |
| **D** | refit supervisionato minimo della media di B (stage 2) | refit | 0,005 | `rollout_eval.py` | isometrico |
| A_native | V26 39D sotto il proprio runtime imitativo | originale | det | **riuso F0 ctrl39** | controllo non isometrico |
| E | JUL_H0 (sigma 0,005 nativa) sotto v3 | originale | 0,005 | **riuso F0 det/stoch** | controllo storico non isometrico |
| A_iso39clk_v3 | V26 + clock prescritto ripristinato (adapter `aiso6clk`) | originale | det | `f1_rollout_aiso` | **diagnostico opzionale, non gate** |

Contrasti dichiarati: **drop** = A_iso vs B (isometrico); **sigma** = B-stoch-nativa vs C-stoch-0,005 appaiati per (start, seed) con common random numbers; **recuperabilità** = D vs C offline (fedeltà al teacher) e closed-loop (D vs B/C e frazione di A_iso). A_native ed E non entrano in alcun contrasto isometrico.

Start esatti: −0,20 s (1.756870983805102), nominale (1.956870983805102), +0,20 s (2.156870983805102). Seed di sviluppo 123–125; **126–128 sigillati e mai usati** (refuso fail-closed nel codice).

## 2. A_iso39_v3: adapter validation-only

* Il runtime v3 emette un actor block 35D con `gait_phase_sin/cos = (0, 1)` costanti (clock disabilitato) e nessun target imitativo. L'adapter (`f1_obs_adapter.py`) deriva dai due manifest content-addressed (35D `c6f86028…`, 39D `2837779c…`) il blocco mancante: esattamente i 4 nomi `healthy_{knee,ankle}_angle_imitation_target[_vel]`, contigui all'indice 2; qualunque altra configurazione è rifiutata.
* A ogni step: `obs39 = insert(obs35, targets(t_pre))` e **assert esatto** `project39to35(obs39) == obs35` (stesso dtype, confronto bitwise), contato nel side-car (`projection_assert_count == steps` è requisito G1). Il vettore completo passa da 84 a 88 colonne con lo stesso layout dell'osservazione di training V26.
* **Revisione r3 (S2a):** il primo cross-check (r1/r2) confrontava la ricostruzione float64 con l'osservazione registrata in float32 e non poteva essere esatto per costruzione (scarto 2,37e-7 = half-ulp float32 a |v| ∈ [4,8)); la prova causale ha mostrato 6000/6000 celle bit-exact dopo il cast al dtype dell'env e 6000/6000 celle float64 post-step bit-exact. Il check è ridefinito come sopra (§5, G1) senza tolleranze; il projection contract è invariato.
* I 4 target sono ricostruiti **in sola lettura** con le classi dell'env (`GaitPhaseClock`, `PhaseBasedImitationTarget`) dai dati prescritti e dai parametri dell'env (heel strike dal metodo puro `_load_sound_heel_strikes`, `base_kin`, `imitation_sound_coords/phase_shifts/phase_shift/phase_samples`, finestra `t_start/t_end`): il clock e il target propri dell'env restano disabilitati, nessun attributo di env/reward/reset/fisica/slew/OOB/terminazioni è modificato. Le chiavi di configurazione da cui dipende la ricostruzione sono uguali fra config V26 nativa e v3 (verifica nel manifest, `reconstruction_config_equality.equal = true`).
* Innesto: `f1_rollout_aiso.py` esegue il loop di produzione `rollout_eval.run()` invariato, sostituendo nel namespace del modulo (processo di validazione soltanto) il loader `RLModule.from_checkpoint` (wrapper `AIsoAdapterModule`, `_n_actor=35`, `_n_full=84`, delega al V26 reale) e `env_factory.make_cmc_env` (proxy che consegna l'env all'adapter). Il modo `passthrough` (modulo B, nessun inserimento) deve riprodurre bit-exact il job B-det di `rollout_eval.py` (G1).
* **Limite dichiarato:** A_iso ripristina 4 dei 6 segnali persi; il clock prescritto resta `(0, 1)` come per B. A_iso differisce da A_native anche per slew 2,5/2,0 vs 0, reward ex_novo vs imitation, start, terminazioni: A_native resta un controllo non isometrico. Se A_iso fallisce il predicato di successo (§5), l'esito **non** è attribuito al drop e F2 va ridisegnata (deadlock del clock online); il diagnostico `aiso6clk` serve a questa discriminazione.

## 3. Sigma costante (C, A_iso_s005, D)

Regola strutturale (non `detach()`): righe log-std di `pi.1.weight` = 0, `pi.1.bias[2:4] = float32(ln 0,005) = −5.2983174324035645`; entropia attesa −7,758758 nat (tolleranza 1e-5) verificata dopo save/reload. La media e ogni altro tensore restano bit-identici (`configure_actor_exploration.configure_constant_std`, ricetta di luglio riusata per import): perciò **C-det deve essere bit-identica a B-det** (G1).

## 4. Dataset e refit di D

* **Dati:** trace B-stoch-nativa della matrice F1, seed 123–124 (train) e 125 (validation), tre start ciascuno; split **per traiettoria** (6 train / 3 validation), deterministiche escluse; nessuna nuova raccolta dopo il fit, zero round DAgger.
* **Righe:** `obs35` = `actor_observation_vector_before` reale; `t_pre = [reset_time] + time[:-1]` (float esatti dell'env); `obs39` = `obs35` con i 4 target a 2:6 presi dal side-car A_iso dello stesso start (griglia `t_pre` identica per costruzione, uguaglianza esatta richiesta) o dal ricostruttore env-side; **teacher** = media deterministica V26 su `obs39` (forward numpy float64 su input cast float32, scarto misurato dal percorso torch ≤ 2e-7 in F0); registrata anche la media di B per la baseline.
* **Refit (budget fisso):** init = actor B; allenabile l'intera rete della media (encoder + righe media di `pi.1`); righe log-std come buffer costanti; colonne `gait_phase_sin/cos` del primo strato hard-zero; loss `MSE(mean, teacher) + 1.0·mean(relu(|mean|−1)²)`, anchor 0; Adam lr 1e-4, batch 256, **300 epoche**, seed 2026, deterministico; selezione **fixed_final_epoch** (nessun early stopping, nessuna selezione su validation o closed-loop). Motivazione: luglio ha usato ~77k passi su 24.712 righe; D usa ~3.600 passi su ~3.000 righe, deliberatamente minimale (prova di recuperabilità, non bridge).

## 5. Gate preregistrati (soglie in `f1_protocol.json`)

**G1 – integrità (hard).** Receipt ok/rc 0/summary ok/trace/SHA e chiusura pre = post = manifest per ogni job; seed ⊆ {123,124,125}; C-det ≡ B-det (trace SHA, 3 start); B-det F1 ≡ B0820_H0 det F0 (riproducibilità inter-fase); passthrough ≡ B-det nominale; A_iso: assert a ogni step, `t_pre` del side-car = regola di shift, nomi env = manifest35, nomi inseriti = manifest39; chiavi di ricostruzione uguali V26/v3; ricostruzione dei target vs trace ctrl39 F0, due confronti **esatti senza tolleranza** (revisione di strumentazione r3): (a) ricostruzione castata al dtype dell'osservazione **letto dall'env** (`observation_space.dtype`, float32) == `obs39[:,2:6]` bit-per-bit a `t_pre` identici su tutte le righe dei 3 start; (b) ricostruzione float64 ai tempi post-step == `imitation_target_q/qdot` registrati dall'env nella stessa trace bit-per-bit; entrambe le differenze massime == 0,0; valori registrati rappresentabili nel dtype dell'osservazione; lo scarto grezzo float64-vs-registrato (quantizzazione float32 della registrazione) è solo informativo; teacher numpy vs azioni A_iso registrate ≤ 1e-5; moduli derivati C/A_iso_s005/D con testa costante ed entropia attesa dopo reload, media bit-exact alla sorgente, colonne clock zero in D; CRN: z di step 1 identico B-stoch/C-stoch (≤ 1e-5) e rep2 di B-stoch nominale seed 123 bit-exact; dataset con split per traiettoria e digest; refit con `epochs_run = 300`, `fixed_final_epoch`, `dagger_rounds = 0`.

**G2 – fedeltà offline (hard).** Sulle traiettorie di validation (seed 125, 3 start): `rmse_D_val ≤ 0,5 · rmse_C_val` (C ha la media di B = errore del hard-drop verso il teacher) e `rmse_D_val ≤ 1,5 · rmse_D_train`; informativo, non gate: `rmse_D_val ≤ 0,02` (luglio in-distribution 0,0081).

**G3 – closed-loop (hard).**
*Predicato di successo A_iso (det):* somma cicli validi v3 sui 3 start ≥ 4; ≥ 2 start con ≥ 1 ciclo; ≥ 2 start a orizzonte; ≥ 2 start con ROM knee ≥ 0,6 rad. Motivazione F0: A_native 3/2/2 cicli, orizzonte 3/3, ROM 1,00/1,00/0,88; JUL_H0 det sotto v3 3/2/0, orizzonte 2/3, ROM 0,93/0,92/0,87; B 0/0/0, ROM 0,18/0,34/0,28. Se falso → `NOT_EVALUABLE_AISO_FAILED`, ridisegno F2, nessuna attribuzione al drop.
*D vs C det (frazione preregistrata 0,5):* `Σcicli(D) ≥ max(1, ⌈0,5·Σcicli(A_iso)⌉)` e `Σcicli(D) > Σcicli(C)`; ROM knee `D ≥ C + 0,5·(A_iso − C)` in ≥ 2/3 start; orizzonte in ≥ 2/3 start; sicurezza: `reserve_max(D) ≤ 1,15·max(reserve_max(C), reserve_max(A_iso))` per start e fine ≠ `grf_penetration` in ≥ 2 start. Motivazione: recuperare metà del divario B→A_iso con un refit minimo off-policy è l'evidenza minima che il hard-drop (non sigma, non il runtime) sia la causa dominante e recuperabile per supervisione; meno di ciò riapre la diagnosi (Gate F1 del piano).
*D vs C stoch (sigma 0,005, 9 coppie CRN):* frazione media di orizzonte e somma cicli di D ≥ C (non regressione).

**G4 – contrasto sigma (diagnostico).** Delta appaiati B-stoch-nativa vs C-stoch-0,005: orizzonte, cicli, return, step clippati, penetrazione, reserve.
**G5 – controlli (diagnostico).** A_native (F0) vs A_iso: effetto del runtime sull'actor privilegiato; E vs B/C/D: storico non isometrico; `aiso6clk` se eseguito.

**Regola decisionale:** F1 PASS ⇔ G1 ∧ G2 ∧ G3 (con predicato A_iso vero). La fase negativa della caviglia è riportata, **non** è gate F1 (requisito F2).

## 6. Stage di esecuzione (S2, dopo autorizzazione) e comandi

Stage 0 (S1, questo documento): protocollo, tooling, test sintetici, dry-run. Stage 1: cross-check adapter vs ctrl39 F0 (`f1_adapter_crosscheck.py`, env costruito con `--max-steps 0`, nessuno step), derivazione C e A_iso_s005 (`f1_sigma_variant.py --materialize`), 41 rollout (A_iso det 3, A_iso_s005 stoch 9, B det 3, B stoch 10, C det 3, C stoch 9, passthrough 1, diagnostico clock 3). Stage 2: dataset D (`f1_dataset.py`), refit (`f1_refit.py --fit`). Stage 3: 12 rollout D. Stage 4: `f1_analysis.py` → gate. I comandi esatti, senza placeholder, sono nel manifest di dry-run (`stage_commands`) e nel report S1.

## 7. Artefatti e provenienza

Tutto sotto `Trajectory Generator/` (`baseline_MLP/validation/f1_ablation_2026-08-23/` per il codice; `runs/rollout/validation/f1_ablation_runs/2026-08-23_F1_ablation_39to35_sigma_r1/` per gli output). Content-addressed, no-clobber, SHA-256 per ogni file consumato/prodotto; receipt schema `f1.1` con chiusura contemporanea (classe B+, validatore F0 `verify_closure_fields` invariato); F0 e i tre file user-owned dirty immutabili; nessuna modifica a produzione, C++, SEA, modello, reward; path POSIX e interprete selezionato senza path hard-coded (Windows x86 / macOS arm64).
