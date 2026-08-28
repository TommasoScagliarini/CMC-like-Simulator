# V26B — R2I (rev3i): stage correttivo PASS ed export (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3i_r2_corrected` → SHA `7cde9329afd90aadd30206e8c060ef341f34b67049867cf3f441881ec78b73d6` (creato PRIMA di codice e fit; pinna rev3g `bb4401fc…`, memo rev3h `e2f31d1e…`, **supplemento rev3h** `b7fe2b9a…`, stato rejected di R2G, init R1, tooling condiviso) · **Esito: test 19/19 (+ regressione r2g 20/20) → fit deterministico singolo autorizzato → postfit PASS → export atomico.**

## 0. Dichiarazioni ordinate
- **Imprecisione rev3h corretta** dal supplemento additivo `2026-08-24_v26b_rev3h_supplemento_anchor_head.md` (`b7fe2b9a…`): l'anchor storico itera sui 6 tensori `pi.named_parameters()` con head FULL [4,256]/[4] e righe logstd a delta zero al momento della loss ⇒ rapporto rev3g/storico = **6× sui primi 4 tensori, 12× sui due termini head** (non «esattamente 6×» uniforme). rev3h non mutato.
- **`V2_R2G` (digest `65276df6…`) resta AUDIT-REJECTED**: preservato su disco, mai init, mai rollout — registrato anche nel receipt R2I (`audit_rejected_predecessor`).
- **Nessun rollout, PPO, critic warmup o sigma sweep eseguito**; produzione/FSM/reward/morphology/SEA/C++ intoccati; σ resta placeholder NON deciso; init = R1 `c7bcee1c…` (non R2G).

## 1. Correzioni implementate e testate PRIMA del fit
- **Anchor storico esatto**: `0.01 · mean([MSE_W1, MSE_b1, MSE_W2, MSE_b2, 0.5·MSE_W3m, 0.5·MSE_b3m])` verso θ_R1_scaled — fattori 0.5 espliciti ≡ delta full zero-padded; **test VALORE e GRADIENTE** contro riferimento sintetico con tensori full [4,h]/[4]: uguaglianza ≤1e-10 su valore e su ogni gradiente; sanity `mean([1,1,1,1,.5,.5]) = 5/6` (fp32).
- **Loss dati (a′)**: `l_data(B) = K·Σ_B λ_i e_i`, **K = ceil(10727/256) = 42 costante** (verificato a runtime); test: somma per-epoca dei termini batch == K·obiettivo globale (esatto, due partizioni diverse) e coefficiente per-riga == K·λ_i indipendente dalla composizione (perturbazione su due partizioni). **Dichiarazione Adam nel receipt**: traiettoria NON equivalente al full-batch, nessuna equivalenza rivendicata.
- Invariati da rev3g: dataset/split/G1/G3-blocked/envelope κ_m/T1-T2/scaling luglio/numerici 300-b256-lr1e-4-seed2026/clip-azione 1.0 (storica)/aux ASSENTE/provenance costruita pre-fit (ora 10 digest incl. rev3h/supplemento/rev3i).

## 2. Esecuzione (log no-clobber `r2i_*.log`, REAL_EXIT_CODE=0)
Prefit: G1 PASS; G3-blocked 0.0358/0.0239 aggregato (ruoli tutti ≤0.15); envelope κ_m=10: 0 violazioni (worst ×5.79); controfattuale 144 righe respinte (0.452/0.248). T1 = **7.60e-08**; fit su train 10 727; T2 = **5.37e-08**. Weighted objective 0.01032 → 0.00168.

## 3. Gate postfit (holdout mai visto dal fit) — TUTTI PASS
| insieme | righe | RMSE knee/ankle |
|---|---|---|
| aggregato | 3 188 | **0.0679 / 0.0439** |
| base | 200 | 0.1081 / 0.0812 |
| r1_prefix | 20 | 0.0284 / 0.0278 |
| alt_minus020 | 1 489 | 0.0585 / 0.0380 |
| alt_plus020 | 1 479 | 0.0699 / 0.0424 |
Fit-corpus (train, distinto): aggregato 0.0609/0.0389, base 0.0406/0.0257. Q3 PASS; save/reload bit-exact; smoke export PASS. Shift parametri vs R1: **19.59** (R2G rejected: 18.6 — l'anchor storico più debole ha permesso ~5% di movimento in più, con holdout invariato al terzo decimale).

## 4. Artefatti e SHA
- Modulo: `student/V2_R2I/rl_module/` — **actor digest `f6579a7fdf27dc6af32d4cc6a2b9e2dbb182fc3c7cb888d83ed1a50de4490ae2`**, `module_state.pkl` `61bfb835e63629bc…`; promozione atomica nativa.
- **Receipt canonico** `v26b_r2i_receipt.json` → **`07fdce69fd85b21519c71c73e4ec79347f0f06cfc162c19591310936eb33a2bd`** (amendments 10-digest, semantica anchor esplicita, dichiarazione Adam, predecessor rejected, T1/T2, gate completi, history, code digests).
- Tooling: `v26b_r2i.py` → `44bc8b3a7c030291…`, `test_v26b_r2i.py` → `b2073b0a85e3194a…`; supplemento rev3h `b7fe2b9a…`; rev3i `7cde9329…`.

## 5. Stato
**STOP per l'audit dell'architetto.** Nessuna promozione implicita: V2_R2I attende il tuo gate.
