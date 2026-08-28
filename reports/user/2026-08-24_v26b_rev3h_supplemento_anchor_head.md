# V26B — Supplemento additivo al memo rev3h: correzione della frase «esattamente 6×» (rev3h immutato)

**Data:** 2026-08-24. Il memo rev3h (`e2f31d1eab306625456195d9e71348457d66e7d527060e012c9a06a58dc85081`) afferma che l'anchor rev3g è «esattamente 6.0× più forte di luglio». La frase è imprecisa sui due termini head.

**Semantica storica esatta** (file pinnato `442be2…:1141-1146` ≡ blob `bdbf99c1:789-794`): l'anchor itera su `module.pi.named_parameters()` — i 6 tensori includono `output_layer.weight` **[4,256]** e `bias` **[4]** FULL, non le sole righe mean [2,256]/[2]. `freeze_logstd_head` avviene via restore post-step (non `requires_grad=False`), quindi al momento di CIASCUNA valutazione della loss le due righe logstd sono al valore anchor (delta zero):
- `mean(Δ_W3_full²)` su 4×256 = **0.5 · mean(Δ_W3_meanrows²)** (512 entry non nulle su 1024);
- `mean(Δ_b3_full²)` su 4 = **0.5 · mean(Δ_b3_meanentries²)** (2 su 4).
**Anchor storico esatto: `0.01 · mean([MSE_W1, MSE_b1, MSE_W2, MSE_b2, MSE_W3full, MSE_b3full])`** = `0.01 · mean([MSE_W1, MSE_b1, MSE_W2, MSE_b2, 0.5·MSE_W3m, 0.5·MSE_b3m])`.

**Rapporto rev3g/storico corretto**: `sum`-vs-`mean` dà **6×** sui primi 4 tensori e **12×** (6×/0.5) sui due termini head mean-only. La conclusione operativa del rev3h (anchor rev3g più forte, artefatto `65276df6…` AUDIT-REJECTED) resta valida; cambia solo la caratterizzazione quantitativa per-tensore, che rev3i implementa esattamente.
