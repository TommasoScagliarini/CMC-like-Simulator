# V26B — Supplemento al memo rev3g (audit obbligatorio pre-GO; additivo, memo precedente immutato)

**Data:** 2026-08-24 · Nessun fit/export/rollout; nessun codice rev3g; frozen intoccati.

## (E) Parent effettivo di JUL_H0 — identificato e pinnato
Run: `Trajectory Generator/runs/training/target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/`
- `adaptation_report.json` → `deb900a6be1107c0fa917cd6adf6aa69e9e214f1f9f60f9926f84e24b7e1bdeb` (hyperparameters verbatim: seed 123, batch 128, lr 5e-5, val 0.2, patience 60, clip 1.0, **logstd_weight 0.0 + freeze_logstd_head TRUE**, **anchor_weight 0.01**, trainable_first_layer_features null = full actor, `first_layer_feature_scales` sulle 10 Markov; epochs 400/400, best 392; train/val 19 770/4 942)
- `run_summary.json` → `15daef3e7395839a1f6e78d08e8c769b5597e6a27abbe12b8056c9ac4601a6b7` — **`ok: false`** (gate storico `nominal_mean_shift` FALLITO; selezione avvenne via gate closed-loop): parent NON-gate-passing, fatto storico registrato.
- **Prova di parentela BYTE-IDENTICA**: `rl_module_target_adapted/module_state.pkl` → `44457ca5df7fa0e0e1f1d361d940136917fe8f71e984a1b0afaefb8ca3ced33b` = **il pin JUL_H0 `44457ca5…`** (il warmup critic non toccò i byte dell'attore); catena digest `42837d48…` (source) → `a0801a9e…` (adapted = JUL_H0 actor digest).
- `markov_dataset_report.json` → `2e07c80e2c29336146928d159346f7ee875c9dfd6feceb98418a09d9533cdb1e`.

## (A) Confronto dimensionale ESATTO luglio-13/07 vs rev3d/f
| aspetto | luglio (parent JUL_H0) | catena rev3d/f | rapporto/nota |
|---|---|---|---|
| anchor formula | `0.01 · Σ_t mean_t((p−p0)²)` sui 6 tensori pi (`target_domain_imitation.py:1142-1148`, file `442be2222c93…`) | `1e-3 · Σ_t sum((p−p0)²)` sui 6 tensori mean | per Δ² uniforme D: luglio **0.060·D**, nostro **75.522·D** → **≈1259× più forte** |
| anchor per-parametro | W2 1.526e-7, W1 1.116e-6, b1/b2 3.906e-5, head 9.766e-6, b3 2.5e-3 | 1e-3 uniforme | W2 **6554×**, W1 896×, b 25.6×, head 102×, b3 0.4× |
| clip | `relu(|m|−1)²·mean`, peso 1.0 | identico | = |
| aux phase head | ASSENTE | λφ=0.1 (training-only) | invenzione di catena |
| logstd | congelata (weight 0, freeze TRUE) | congelata (placeholder) | = (la distillazione era solo 11/07) |
| stopping | early stopping val 0.2 / patience 60 (best 392/400) | fixed 300, nessuna validazione | deviazione preregistrata (rev3b) |
| input scaling | scale fisiche fisse sulle 10 Markov, **assorbite all'export** | input raw | vedi (B) |
| seed/batch/lr | 123 / 128 / 5e-5 | 2026 / 256 / 1e-4 | diversi |

## (B) Semantica scaling luglio — VERIFICATA nel codice, nessun mismatch
Training: `tensor_obs = raw_tensor_obs / input_scales` (`target_domain_imitation.py:1049`). Export: `first_layer_weight[:, index].div_(scale)` per ciascuna feature scalata (`:1203-1207`) → il modulo esportato consuma osservazioni FISICHE raw: `raw·(W/s) ≡ (raw/s)·W`. **Assorbimento intenzionale e corretto; nessun deployment mismatch.** Le 10 feature/scale: knee/ankle `previous_endpoint` 1.0, `served_ref` 1.0, `served_ref_vel` 4.0/3.5, `served_ref_accel` **60.0/55.0**, `sea_u` 1.0. Effetto reale: SOLO geometria di ottimizzazione (lr efficace per-feature), contratto runtime invariato.

## (D) Correzione della motivazione δ=0.30 + gate ricalibrato
**Correzione**: l'affermazione «δ=0.30 rende RMSE 0.15 matematicamente irraggiungibile» è INVALIDA per una RMSE aggregata: una coppia conflittuale sparsa forza errore ≥|Δu|/2 su UNA riga; l'aggregato su N righe si sposta di ~|Δu|/(2√N) (qui ≈0.001) — trascurabile. I conflitti di coppia vincolano il **worst-case per-riga**, non l'aggregato.
**Dinamica locale naturale (adjacent, 13 936 coppie)**: slope |Δu|/d — knee p50 0.0076 / p90 0.0563 / **p99 0.0890** / max 0.238; ankle p99 0.0909. Envelope a-priori `E_g(d) = SF·Q99_g·d`, SF=2 dichiarato, D_max=2r*=1.429. **Tail rate della popolazione definente sotto lo stesso envelope: 4/13 936 = 0.0287%.**
**Applicazione alle coppie non-adjacent/cross-trajectory (59 350 entro D_max)**: violazioni 3 428 (5.78%; same-traj-far 269, same-role 1 216, cross-role 1 943); worst excess ×5.79 (d 0.206, |Δu| 0.212); sensibilità SF 1/1.5/2/3 → 6 943/4 481/3 428/1 879 (riportata per trasparenza, non per scelta).
**Controfattuale decisivo**: le 144 righe post-mismatch del naïf-R2 hanno NN p50 **9.08 ≫ D_max** → il gate di coppia NON le vede; le boccia il **G3-blocked** (kNN-vs-label 0.442/0.248 ≫ 0.15). Quindi l'identificabilità va custodita da G3-blocked, non dal gate locale.
**Gate preregistrabile proposto (nessun numero scelto sul max osservato)**:
- **Vincolante 1 — G3-blocked/group-held-out** (definito nel memo precedente): kNN k=5 ≤ 0.15/giunto per ruolo e aggregato. Stato attuale: PASS (0.021–0.070); boccia il naïf-R2 (0.442).
- **Vincolante 2 — magnitudine catastrofica**: zero coppie non-adjacent entro D_max con excess |Δu|/E(d) > κ_m = **10** (costante d'ordine dichiarata a priori). Stato attuale: worst ×5.79 → PASS.
- **Diagnostico (riportato, non vincolante)**: densità di violazioni dell'envelope vs tail rate adjacent (attuale: 5.78% vs 0.0287%, ratio 201×) — rumore di label di piccola magnitudine (~0.2 a piccola d), assorbito dal target RMSE 0.15; renderlo vincolante richiederebbe un κ arbitrario che i dati attuali non passerebbero (κ=10 → limite 0.287%). Decisione a te.

## (C) Deviazioni: indispensabili vs eliminabili (con la questione R2-da-R1)
Indispensabili per lo student causale V26B 35D: contratto 35D/clock privilegiato zero (ovvio); niente early stopping su validazione (principio di catena rev3b, il fixed-final-epoch è storicamente legittimo); seed/batch della catena SE si vuole comparabilità con R0a/R1.
Eliminabili per fedeltà a luglio (raccomandate): **(1) normalizzazione per varianza → MSE piatta** (memo precedente, confermata: luglio era piatta); **(2) anchor → formula luglio `0.01·Σ_t mean_t`** ancorata a θ_R1 (la nostra è ~1259× più forte: differenza materiale nascosta); **(3) aux phase head → rimuovere** (assente a luglio, training-only, nessun impatto sul contratto); **(4) adottare le scale fisiche luglio sulle 10 Markov con assorbimento all'export** (fedeltà di ottimizzazione, zero rischio contratto, codice storico verificato).
**R2-da-R1 con July numerics integrale (400+ES/b128/lr5e-5/seed123)?** Architetturalmente INCOERENTE: R1 è nato sotto rev3d; adottare ora l'intero pacchetto numerico luglio romperebbe la comparabilità R0a→R1→R2 e reintrodurrebbe l'early stopping vietato dalla catena, senza restituire davvero «luglio» (il parent luglio ha un'altra storia di init). **Raccomandazione: coerenza di catena sui numerici (300/b256/lr1e-4/seed2026, fixed-final), fedeltà a luglio su loss/anchor/aux/scaling (punti 1–4).**

## Raccomandazione finale per il GO rev3g
Loss piatta a masse luglio (λ esatte) + anchor formula luglio 0.01·per-tensor-mean su θ_R1 + niente aux + scale fisiche assorbite; gate: G1 invariato, **G3-blocked vincolante**, **magnitudine κ_m=10 vincolante**, densità envelope diagnostica; numerici di catena invariati. Nessun codice scritto e nessun training fino al tuo GO.
