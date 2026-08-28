# V26B — V1: transplant init 39D → 35D (report immutabile)

**Data:** 2026-08-24 · **Protocollo:** `V26B-bridge-rev3` (`5e0b6a5c…`) · **Stage:** `V1_init_transplant_39_to_35`, GO dell'architetto dopo verifica indipendente della copertura anchor (coverage JSON `5c8123607bba3bf6339ad220163f95966a0f9cf4497854ed32aac863e1e9218e`, report `4348aded…`) · **Esito: EXPORT COMPLETATO, batteria fail-closed tutta PASS.** Report immutabile; rettifiche solo in nuovi file.

## 1. Sorgente (unica): V26 agosto 39D
`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best` — actor digest `5bbc6cbd3c7e…`, `module_state.pkl` `0ba56eb7…`. Immutabilità verificata: i 4 file del modulo sorgente ri-hashati dopo la build sono invariati (registrato nel receipt). JUL_H0/T1/T1R/B0820 mai usati come init (JUL consultato SOLO come riferimento informativo di equivalenza dei ctor args del formato 35D).

## 2. Ricetta eseguita (preregistrata rev3, lignaggio luglio 25/06+09/07+13/07)
1. **Trasferimento per NOME** via manifest pinnati (35D `c6f86028…`, 39D `2837779c…`): biiezione order-preserving verificata (35 nomi condivisi = 39 meno i 4 target a 2:6); ogni colonna mantenuta è bit-identica alla sorgente.
2. **Rimozione dei 4 target con compensazione mean-bias** `b1 += W1_39[:,2:6] @ m`, `m` = media float64 dei target float32 consumati dall'attore sulle **3 trace anchor det pinnate** (1500 righe, pin verificati):
   - `m = [-0.417821058, +0.013012919, +0.235458239, +0.006400180]` (per-trace nel receipt)
   - delta bias: ‖δ‖₂ = 0.771961, range [-0.127501, +0.114380], SHA float64 `2ba368ab3df7fc51…`; bias risultante SHA `82f15783cc1cb77c…`; max |Δb1| = 0.127501
3. **Colonne clock prescritte (0:2) azzerate** — nessuna compensazione clock (preregistrato: solo azzeramento).
4. **Hidden layer e mean head copiati bit-identici** (verificato con `array_equal`).
5. **Log-std sostituita**: righe sorgente state-dependent (‖righe‖₂ = 0.9454, bias ≈ [-0.0383, -0.0329]) → righe zero + bias esattamente `float32(ln 0.005)`. **σ=0.005 è un placeholder di serializzazione richiesto dal formato del modulo: NON è la selezione del σ operativo, nessun gate lo legge, il σ operativo sarà deciso esclusivamente dallo sweep V4.**

## 3. Batteria fail-closed (tutta PASS)
Esattamente le 10 chiavi `pi*` originali nell'ordine originale; alias encoder bit-identici; tutti float32; larghezza input 35; hidden 256; `W1[:,0:2] == 0`; invarianza clock **bit-identica** (forward numpy con clock perturbato ±10σ); log-std righe zero e bias esatto; tutto finito; **save/reload bit-exact** (pickle → `warm_start.load_module_state` → confronto esatto); ctor args riscritti dal solo V26 (obs space 88→84 = n_actor+49, `n_actor` 39→35, `n_full` 84 invariato) e verificati equivalenti alla forma 35D deployable comprovata; `f2r_refit.validate_init_state` (batteria F2R storica) PASS sul modulo finale ricaricato.

## 4. Fidelity vs V26 sulle 3 trace anchor det (INFORMATIVA, non gate V1)
| start | knee RMSE | knee MAE | knee max | ankle RMSE | ankle MAE | ankle max |
|---|---|---|---|---|---|---|
| minus020 | 0.9390 | 0.7176 | 2.4463 | 0.6567 | 0.5105 | 2.0030 |
| nominal | 0.9319 | 0.7085 | 2.3731 | 0.6348 | 0.4993 | 2.0292 |
| plus020 | 0.9234 | 0.7204 | 2.3210 | 0.6097 | 0.4938 | 1.8823 |
| **overall** | **0.9315** | — | — | **0.6340** | — | — |
RMSE pooled 0.7967 (unità azione grezze pre-clip). Degrado atteso e coerente col punto-baseline tipo B0820: lo student ha perso target privilegiati e clock prescritto; V2 (adattamento al dominio target) è lo stadio correttivo preregistrato.

## 5. Export (transazionale, no-clobber, content-addressed)
- Percorso: `runs/rollout/validation/v26b_bridge_runs/2026-08-24_V26B_anchors_r1/student/V1_35D_transplant/`
- Promozione: **`darwin_renameatx_np_RENAME_EXCL`, single-step atomica**; lock rilasciato; staging assente.
- `rl_module/`: `module_state.pkl` `16c2d1ae9fb4e77fffa092d74d37e78f54ba24d990774e91bf1d412c551bb031`, `class_and_ctor_args.pkl` `c9a6722f…`, `metadata.json` `3a032ba5…` (bytes V26), `actor_feature_manifest.json` `db779f41…` (35 nomi + nota placeholder σ).
- **Nuovo actor digest: `ae846220a6f7f1ac1289ccc9636e3ad2e5bc7842ba7ece0b62bb9d7590e7f587`**
- Receipt canonico: `v26b_v1_receipt.json` → SHA `65b447ea331d58b457fe6df48e67697dbd6693986552411c0f9780c75ffb995c` (lega sorgente, manifest, protocollo, coverage JSON, file di output, digest del codice: `v26b_student.py` 3cc2bff5afc0f700…, `warm_start.py`, `f2r_refit.py`, ecc.).

## 6. Test e vincoli
`test_v26b_student.py`: **SELFTEST PASS 83 check** (transplant sintetico con matematica della compensazione verificata indipendentemente, determinismo, 7 negativi fail-closed, build reale + batteria F2R, guardia stage token, export in tempdir con verifica receipt/no-clobber/lock, immutabilità sorgente). Log export no-clobber `v1_export_*.log` (`REAL_EXIT_CODE=0`). Nessun rollout, fit, dataset, env stepping; F0/F1/F2R e file dirty user-owned intoccati.

## 7. Stato
**STOP.** In attesa dell'audit dell'architetto. Nessun lavoro V2, nessun rollout del nuovo student.
