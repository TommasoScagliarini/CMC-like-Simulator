# V26B — Audit memo rev3h (additivo): due discrepanze vincolanti in R2G; artefatto 65276df6… AUDIT-REJECTED

**Data:** 2026-08-24 · Nessun fit/rollout eseguito; rev3g immutato; nessun artefatto cancellato.

## 0. Dichiarazione di stato
**L'artefatto `student/V2_R2G` (actor digest `65276df60b20b0180cfea6695bbbf4fb82e685b0dd0ea52578297e0f147c891a`, receipt `5374f1af…`) è AUDIT-REJECTED / NON-PROMOSSO.** Resta su disco come evidenza (mai cancellato, mai sostituito, mai usato come init o per rollout) finché l'architetto non decide.

## 1. Discrepanza anchor — evidenza byte/linee (nessuna assunzione)
- File pinnato dall'emendamento rev3g: `Trajectory Generator/baseline_MLP/target_domain_imitation.py` (SHA `442be2222c935e18…`), righe 1141-1146:
  `anchor_terms = [(parameter - anchor[name]).square().mean() … if parameter.requires_grad]` → `anchor_loss = torch.stack(anchor_terms).mean()` → `+ anchor_weight * anchor_loss`.
- Blob dell'epoca del run 13/07 (`git show bdbf99c1:…target_domain_imitation.py`, righe 789-799): **identico** — `torch.stack(anchor_terms).mean()`.
- Artefatti del run: `adaptation_history.json` registra solo train_loss/validation_mse per epoca; **nessuna prova contraria** trovata in codice o artefatti.
**Verdetto**: la semantica storica è `0.01 × MEAN sui tensori trainabili delle mean-squared delta` (6 tensori nel run 13/07: head incluso perché il freeze avveniva via `restore_logstd_head()` post-step, non via requires_grad). rev3g ha usato `sum()` → **anchor esattamente 6.0× più forte di luglio**; l'etichetta “July-faithful” dell'anchor in rev3g/receipt/report era quindi impropria. Correzione proposta: `0.01 · mean_t` (coefficiente effettivo per-tensore 0.01/6 ≈ 1.667e-3 · mean_t).

## 2. Discrepanza masse — quantificazione dello schedule ESEGUITO
Il fit eseguito usa `l_data(B) = (Σ_B λe)/(Σ_B λ)` (rinormalizzazione locale). Replay esatto delle permutazioni (rng 2026, 300 epoche, batch 256, train 10 727):
| ruolo | massa target | massa effettiva | errore rel. |
|---|---|---|---|
| base | 0.647459 | 0.639721 | **−1.195%** |
| r1_prefix | 0.028812 | 0.028987 | +0.609% |
| alt_minus020 | 0.161865 | 0.165668 | **+2.350%** |
| alt_plus020 | 0.161865 | 0.165624 | +2.322% |
Dispersione del peso per-riga entro ruolo (dipendenza dalla composizione del batch): cv ≈ 0.87%. La violazione del preregistrato è modesta ma reale e strutturale (l'obiettivo globale dichiarato non è quello ottimizzato).

## 3. Implementazione deterministica proposta (con dimostrazione numerica)
**Proposta (a′) — somma pesata a costante globale**: `l_data(B) = K · Σ_{i∈B} λ_i e_i` con `K = n_batches = 42` costante dichiarata (batch 256 invariato).
- **Esattezza**: ogni riga contribuisce con peso FISSO `K·λ_i`, indipendente dalla composizione del batch; la somma per-epoca dei termini batch = `K · Σ λ_i e_i` **ESATTAMENTE** (dimostrato full-corpus vs accumulo batch: delta 0.00e+00, solo ordine di somma).
- **Scala**: `K · E[Σ_B λ] = 1.0023 ≈ 1` → stessa scala media del ratio-form: disturbo minimo su lr/regolarizzatori.
- **Interpretazione Adam dichiarata (nessuna falsa equivalenza)**: ogni step Adam minimizza `l(B) = K·Σ_B λe + clip_mean(B) + w_a·anchor`; il peso per-riga è costante ma la MAGNITUDINE del termine dati varia con Σ_B λ (composizione), quindi la traiettoria Adam NON è identica a un full-batch: ciò che si garantisce è l'obiettivo per-riga esatto e il gradiente-somma per epoca esatto, non l'equivalenza dell'ottimizzatore.
- Alternative valutate: **(b) batching stratificato** per quote di ruolo — quote frazionarie per prefix (77 righe/42 batch ≈ 1.8), non integrali-stabili; **(c) accumulo full-batch, 1 step Adam/epoca** — obiettivo esatto ma schedule Adam radicalmente diverso (300 step invece di 12 600 → richiederebbe ricalibrare epoche/lr = modifica di iperparametri). **Raccomandata (a′)**.

## 4. Distinzioni richieste
- **Action-clip penalty** `relu(|m|−1)²·mean`, peso 1.0: **RESTA** — storica (identica in luglio e nella catena, riga 1137 del pinnato).
- **Aux phase loss**: **ASSENTE** in rev3g come a luglio (rimossa correttamente; il rigetto non la riguarda).

## 5. Impatto numerico atteso della correzione (per la decisione)
- Anchor 6× più debole → maggiore libertà di allontanarsi da θ_R1_scaled; nel run rejected lo shift era 18.6 con l'anchor forte; con 0.01·mean ci si attende shift maggiore e (probabilmente) RMSE fit più basso; i gate holdout restano gli arbitri.
- Correzione masse: spostamento piccolo (≤2.35% rel.) del bilanciamento verso base.
- Nessun altro iperparametro cambia (300/b256/lr1e-4/seed2026/clip 1.0; logstd placeholder; no critic; scaling luglio con T1/T2 invariati).

## 6. Opzioni per la decisione dell'architetto
1. **rev3i correttivo (raccomandato)**: anchor `0.01·mean_t` + loss (a′) a costante globale; tutto il resto identico a rev3g; refit deterministico completo; nuovo export (V2_R2I) con V2_R2G lasciato come rejected-evidence; gate identici (prefit già indipendenti dal fit; postfit su holdout).
2. Come 1 ma con batching stratificato (b) — sconsigliato (quote frazionarie).
3. Tenere V2_R2G dichiarandone le semantiche reali (anchor 6×, masse ±2.35%) come deviazioni documentate — sconsigliato: contraddice il preregistrato rev3g.
**Nessun fit/rollout sarà eseguito fino al GO.**
