# V26B — Stage R2 offline (rev3f): STOP fail-closed al gate G2, nessun fit eseguito (report immutabile)

**Data:** 2026-08-24 · **Emendamento:** `rev3f_r2_offline_roleweighted` → SHA `011eff9b5da77dd437fb1e9f046988929dea3b196a66c8e4a0adb1e993ef3652` (creato PRIMA di ogni fit; pinna rev3e, receipt/addendum/trace R1, trace R0a, dataset base `ce309b40…`+receipt, alt-start `9862d5f8…`+receipt `febf65b5…`, init R1 `c7bcee1c…`, codice/test) · **Esito: G1 PASS, G2 FAIL (23 gruppi ambigui) → STOP con diagnostica come ordinato («NON rilassarlo arbitrariamente»), NESSUN fit, NESSUN export.** G3+coverage eseguiti come SOLA diagnostica.

## 1. Cosa è stato costruito (tutto additivo; frozen intoccati)
- Tooling `v26b_r2_offline.py` → `7c310fc3cc3bb1ea…`, `test_v26b_r2_offline.py` → `16fafee4e268dc80…` — SELFTEST **16/16** (lineage con 2 negativi, conteggi/dedup/G1, G2 known-answer, G3 sintetico, pesi luglio, guard token; il test ha scoperto e fatto correggere un crash del coverage con ruolo assente → ora rifiuto pulito).
- **Dataset R2** (in memoria, con provenance per-riga; il salvataggio npz avviene solo dopo i gate → NON salvato): base 992 + r1_prefix **97** (98 − 1 collisione riga-di-reset, label identiche) + alt_minus020 6 447 + alt_plus020 6 429 = **13 965 righe uniche**, 1 collisione, zero conflitti.
- **G1 PASS**: le 98 righe del prefisso coincidono TUTTE col riferimento R0a sulle 8 feature discrete; esclusi 144/242 (frazione 0.595 < 0.80), contabilizzati.
- **Pesi luglio** (registrati): w = {base 0.6474749919, r1_prefix 0.0288119133, alt_minus020 0.1618565474, alt_plus020 0.1618565474}; λ per-riga = w_r/n_r con n_r POST-dedup {992, 97, 6447, 6429}; Σλ = 1 (verificato nel selftest); formula batch = Σλe/Σλ, e_i = mean_j (m−y)²/Var_{r,j}.

## 2. G2 FAIL — diagnostica completa (artefatto registrato `v26b_r2_g2_stop_20260824_160456.json` → `c5653ec83c2960a827e8de2c891488f1cecabdbeb1700c1c97c2c841c959e8c5`)
Firma dichiarata: 8 feature discrete July-rule (valori esatti) + stance/swing elapsed quantizzati a 0.05. Soglia: zero gruppi con spread label > 0.2/giunto.
- **23 gruppi violanti; 12 988/13 965 righe (93%) coinvolte.**
- **Causa dominante: DEGENERAZIONE CICLICA della firma** — la coppia (stato FSM, elapsed) si ripete a ogni ciclo del cammino e nelle zone pre-gait, mentre u_IK è indicizzata al **tempo assoluto** di un riferimento IK **non esattamente periodico**: la stessa firma raccoglie ricorrenze a t, t+T, t+2T con label diverse. Evidenze: gruppi da 868–896 righe con t_span 3.25 s (≈ 2 cicli interi) e composizione multi-ruolo; il peggiore è **mono-ruolo**: 1 218 righe alt_minus020 in UNA firma, spread knee 1.093, t_span 1.86 s.
- L'ambiguità è in larga parte **within-role** (within-role spread fino a 1.09): non è un artefatto della composizione B+C né del cross-start; **anche il solo dataset base la contiene** (gruppi 'base' con spread fino a 0.996). Sarebbe scattata identica su qualunque dataset della catena.

## 3. G3 + coverage (eseguiti come DIAGNOSTICA, non per aggirare G2)
- **G3 PASS netto**: kNN k=5, split role-stratified deterministico (rng 2026, 20%/ruolo, scaling mean/std SOLO sul train, feature costanti escluse), holdout 2 792 righe → **RMSE 0.0356 / 0.0214** ≤ 0.15.
- **Coverage**: tutti i ruoli presenti; r1_prefix il più isolato (NN verso gli altri ruoli p50 2.55 unità std; base 1.51; alt 0.57–0.73).
- **Lettura congiunta**: lo stato continuo COMPLETO identifica le label (una lookup le riproduce a ~0.036) — l'ambiguità vive solo nella proiezione a 10 feature di G2. È esattamente il caso «gate troppo grossolano per le feature continue» previsto dall'ordine: quindi STOP senza rilassamento, decisione a te.

## 4. Punti di decisione per l'architetto (presentati, NON implementati)
1. Ridefinire la firma G2 spezzando la degenerazione ciclica (es. aggiungere l'indice di ciclo `valid_hs_count`/`valid_to_count` — già feature discrete July-rule ma ASSENTI dal vettore 35D deployable — oppure `phase_cycle_progress_credit` e/o memoria controller quantizzata), con soglia invariata.
2. Sostituire G2 con un gate di ambiguità nello spazio continuo (es. coppie a distanza < ε con |Δlabel| > δ — la scansione quasi-collisioni già validata), che misura la stessa proprietà senza la proiezione degenere.
3. Accettare G3 come gate di identificabilità unico per questo stage.
Ognuna richiede un tuo emendamento additivo; il fit role-weighted resta pronto e non eseguito.

## 5. Stato
Nessun fit, nessun export, nessun rollout/PPO/critic/σ-sweep; dataset R2 non persistito (per progetto: salvataggio solo post-gate). σ = placeholder NON deciso. Frozen e produzione intoccati, catena rev3…rev3f verificata. **STOP per audit.**
