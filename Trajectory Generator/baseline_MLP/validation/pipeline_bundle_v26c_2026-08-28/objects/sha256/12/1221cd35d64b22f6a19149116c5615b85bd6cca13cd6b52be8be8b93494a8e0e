# V26B — Memo diagnostico rev3g (audit/proposta; additivo, nessun fit/export/rollout; rev3f e frozen intoccati)

**Data:** 2026-08-24 · Dataset ricostruito identico in memoria (13 965 righe, stessi conteggi/collisione del run rev3f). Tutte le metriche in 35D deployable, scaling mean/std SOLO sul train dello split dichiarato, feature costanti escluse.

## (1) Unità indipendenti per ruolo
| ruolo | righe | traiettorie | seed | intervallo t |
|---|---|---|---|---|
| base | 992 | **2** (`anchor_det_nominal` 500; `R0A…det` 492) | 123 | 13.95–18.94 |
| r1_prefix | 97 | **1** | 123 | 13.96–14.92 |
| alt_minus020 | 6 447 | **13** (12 stoch σ{0.0025,0.005,0.01}×seed{1000-1003} + det 123) | 123,1000-1003 | 13.75–18.74 |
| alt_plus020 | 6 429 | **13** | 123,1000-1003 | 14.15–19.14 |
**29 traiettorie reali.** Base e prefix NON hanno abbastanza gruppi per un group-holdout interno → dichiarato e usato blocked contiguous window con embargo esplicito.

## (2a) Curve NN/quasi-collisioni (kNN k=10 per riga, coppie non ordinate; scala locale a-priori r* = p50 delle distanze fra step CONSECUTIVI della stessa traiettoria = **0.718** unità std; p25/p75 = 0.611/0.876)
| categoria | coppie | d p50 | |Δu|knee p50/p95/max | conflitti ≤r*, δ=0.2 | conflitti ≤r*, δ=0.3 |
|---|---|---|---|---|---|
| same-traj adjacent (Δstep≤2) | 8 756 | 0.660 | 0.006/0.045/0.103 | **0**/5 584 | 0 |
| same-traj far (ricorrenze cicliche) | 2 234 | 1.449 | 0.030/0.210/0.536 | 34/332 (max 0.235) | **0** |
| same-role altra traiettoria | 37 742 | 0.609 | 0.003/0.074/0.299 | 247/26 830 (max 0.299) | **0** |
| cross-role | 21 026 | 0.625 | 0.006/0.154/1.064* | 252/14 432 (max 0.261) | **0** |
*max 1.064 a distanza > 2r* (non locale). A δ=0.2: 533 coppie conflittuali totali entro r* (0.9–1.7% di densità, magnitudine ≤ 0.299); a δ=0.3: ZERO. Trasparenza: il max osservato (0.299) è a ridosso di 0.30 — dichiarato, non nascosto.

## (2b) G3 leakage-controlled (group-holdout: 3/13 traiettorie intere per start alt, rng 2026; base/prefix: blocked window [201-300]/[41-60] con embargo 10/5 step esclusi dal train)
| insieme | righe holdout | kNN k=5 RMSE knee/ankle |
|---|---|---|
| aggregato | 3 188 | **0.0358 / 0.0239** |
| base | 200 | 0.0699 / 0.0586 |
| r1_prefix | 20 | 0.0262 / 0.0707 |
| alt_minus020 | 1 489 | 0.0206 / 0.0143 |
| alt_plus020 | 1 479 | 0.0408 / 0.0222 |
**Tutti ≤ 0.15 con ampio margine anche senza leakage temporale**: l'identificabilità nello spazio continuo 33D regge sotto split per-gruppo/bloccato. (Il G3 row-wise del rev3f dava 0.0356/0.0214: il leakage c'era ma non era il fattore decisivo.)

## (3) Proposta G2-continuous (deterministico, preregistrabile)
Gate: **zero coppie fra TRAIETTORIE DIVERSE con d(z_i,z_j) ≤ r\* e |Δu|_g > δ per un giunto g**, con:
- r\* := p50 delle distanze fra step consecutivi della stessa traiettoria (definizione a-priori: la risoluzione temporale naturale dei dati; qui 0.718);
- coppie same-trajectory adjacent escluse (portano la variazione fisiologica della label, p95 0.045); le ricorrenze same-traj-far incluse (sono conflitti reali);
- **δ: due opzioni** — (i) δ=0.2 come da ordine: il dataset attuale FALLIREBBE (533 coppie, max 0.299, densità ≤1.7%); (ii) motivazione quantitativa: **δ = 2×RMSE_gate = 0.30** (una coppia con |Δu|>0.30 rende matematicamente irraggiungibile 0.15 su almeno una delle due righe: |e_i|+|e_j| ≥ |Δu|): il dataset passa con zero coppie. In alternativa (iii) δ=0.2 con soglia di densità (< 2% delle coppie entro r\* e magnitudine max < 0.30) invece di zero-tolleranza. **La scelta è tua; i numeri sopra valgono per tutte e tre.**

## (4) Clock privilegiato nelle metriche
Escluse per varianza nulla ESATTAMENTE e SOLO `gait_phase_sin` (≡0) e `gait_phase_cos` (≡1): il clock privilegiato è morto e fuori da tutte le metriche; 33 feature effettive (elenco = 35D meno le due). Il clock online è vivo nei ruoli base/alt e morto nelle righe r1_prefix, ma pooled ha varianza > 0 → incluso correttamente.

## (5) Pesi: discrepanza documentale (rev3f resta immutato)
Il CODICE calcola i pesi a runtime ed è ESATTO: base 0.6474587245063127, r1_prefix 0.028811913240530916, alt 0.16186468112657817 ciascuno (= valori dell'architetto). Il TESTO dell'amendment rev3f riporta cifre errate (0.6474749919/0.0288119133/0.1618565474 — errore di trascrizione manuale). rev3g registrerà i valori esatti; rev3f non viene toccato.

## (6) Loss: fedeltà a luglio
Loss storica 13/07 (`target_domain_imitation.py:1136-1148`): `mse_loss(means, targets)` **PIATTA, non normalizzata per varianza**, su righe fisicamente tiled → equivale ESATTAMENTE a Σλ_i·mean_j(m−y)²/Σλ con λ_i = mass_r/(24712·n_r) — cioè la nostra struttura di pesi ma **senza 1/Var**. La normalizzazione per-role per-joint del rev3f è un'invenzione della catena (lineage F2R/T1R), NON storica: amplifica gli errori knee ~8× rispetto ad ankle e ripesa i ruoli per spread delle label. **Proposta per fedeltà all'obiettivo «riportarsi alla condizione finale di luglio»: rev3g adotti e_i = mean_j(m−y)² (MSE piatta) con le stesse λ di massa**, regolarizzatori invariati come già preregistrati (deviazioni note: λ_a 1e-3 su Σq² vs July 1e-5 su media per-tensore + distillazione logstd assente perché congelata — già documentate).

## (7) Proposta precisa per il GO rev3g
1. G2-continuous come al punto (3) — δ a tua scelta fra (i)/(ii)/(iii), r\* a-priori, coppie same-traj-adjacent escluse.
2. G3 sostituito dalla versione leakage-controlled qui definita (group-holdout 3/13 per start alt, blocked+embargo per base/prefix; RMSE ≤ 0.15/giunto per ruolo e aggregato) — già PASS sui dati attuali.
3. Loss July-faithful (MSE piatta pesata per massa) al posto della variante normalizzata; pesi con le cifre esatte.
4. Tutto il resto invariato da rev3f (dataset, G1, budget, gate post-fit, export condizionale, divieti).
**Nessun training fino al GO rev3g.**
