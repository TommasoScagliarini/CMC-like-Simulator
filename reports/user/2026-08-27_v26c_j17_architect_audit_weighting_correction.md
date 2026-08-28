# V26C J17 — Correzione dell'effective weighting

**Data**: 2026-08-27
**Stadio**: `V26C_J17_CAUSAL_DIAGNOSIS` — correzione additiva puntuale
**Natura**: **overlay**. Nessun artefatto preesistente è stato modificato.
**Nessun fit, rollout, environment, critic, PPO, collection. J18 non iniziato.**
**Report obbligatorio di fine fase.**

---

## 1. Il rilievo

L'audit ha bloccato la chiusura per un'incoerenza formale nell'effective weighting. Il rilievo è
corretto. I difetti sono **tre**, tutti miei: **due errori numerici** (D1, D2) e **un errore di
categoria** (D3).

### D1 — incoerenza interna al JSON

Nell'addendum, `cell_B_unique` e `cell_C_unique` dichiaravano `effective_rows = 4000` e
`share_of_aggregate_pct = 1.9556`. **I due campi si contraddicevano dentro lo stesso oggetto**: la
percentuale era calcolata sulla fetta unica da 500 righe, non sulle 4000 effettive.

Causa: l'espressione generatrice usava `len(slice)` — la vista da 500 righe — invece di
`unique_rows × repeat_factor`.

Valore corretto: **15.6452%**.

### D2 — errore di arrotondamento nel Markdown

La tabella del report riportava **15.6455%**. Il valore esatto è

```
4000 / 25567 × 100 = 15.64516759885790276528337310 %
```

che a quattro decimali è **15.6452**, non 15.6455. Causa: aritmetica fatta a mano, non il codice.

### D3 — errore di categoria

`nominal_anchor_A_D_region`, `j14_post_mismatch_671` e `j14_pre_mismatch_183` comparivano nella
stessa lista dei blocchi disgiunti, come se fossero membri additivi dell'aggregato. **Non lo sono**,
e così presentati rendevano possibile un doppio conteggio.

---

## 2. Effective weighting corretto — solo blocchi disgiunti

Denominatore: **25567** righe.

**Nota di precisione, prima della tabella.** `25567 = 37 × 691`: nessuno dei due fattori è 2 o 5,
quindi **ogni quota è un decimale periodico** e **nessuna** può essere scritta esattamente in forma
decimale, a nessuna precisione finita. Per questo ogni blocco porta **tre** rappresentazioni: la
**frazione razionale**, che è l'unica esatta; l'**approssimazione Decimal a 28 cifre
significative**, correttamente arrotondata ma *non* esatta; e l'arrotondamento a quattro decimali
per la lettura.

| blocco | uniche | repeat | righe | frazione (esatta) | Decimal, 28 cifre sign. (approssimazione) | 4 dec |
|---|---|---|---|---|---|---|
| `j7_nominal` | 500 | 32 | 16000 | `1600000/25567` | 62.58067039543161106113349239 | **62.5807** |
| `j7_recovery` | 713 | 1 | 713 | `71300/25567` | 2.788751124496421167911761255 | **2.7888** |
| `cell_B` | 500 | 8 | 4000 | `400000/25567` | 15.64516759885790276528337310 | **15.6452** |
| `cell_C` | 500 | 8 | 4000 | `400000/25567` | 15.64516759885790276528337310 | **15.6452** |
| `j14_recovery_854` | 854 | 1 | 854 | `85400/25567` | 3.340243282356162240388000156 | **3.3402** |
| **somma** | 3067 | — | **25567** | `2556700/25567` = **100** | — | vedi sotto |

### L'esattezza viene dagli interi, non dai decimali

```
righe:    500×32 + 713×1 + 500×8 + 500×8 + 854×1
        = 16000 + 713 + 4000 + 4000 + 854 = 25567          ← esatto

frazioni: 1600000/25567 + 71300/25567 + 400000/25567
        + 400000/25567 + 85400/25567 = 2556700/25567 = 100  ← esatto
```

Le due identità sopra sono la prova. **Nessuna somma di decimali è usata come evidenza.**

### La somma dei valori visualizzati è 100.0001, e non è un difetto

```
62.5807 + 2.7888 + 15.6452 + 15.6452 + 3.3402 = 100.0001
```

Cinque valori arrotondati **indipendentemente** a quattro decimali non devono sommare
all'arrotondamento del loro totale: qui quattro dei cinque arrotondano per eccesso. La quota
aggregata **esatta è 100**, e l'arrotondamento a quattro decimali di quel totale esatto è
**100.0000**. Lo dichiaro esplicitamente perché non venga scambiato per una nuova incoerenza.

---

## 3. Viste e sottopartizioni — non additive

| elemento | righe | tipo | di | perché non si somma |
|---|---|---|---|---|
| `nominal_anchor_A_D_region` | 500 | **vista analitica** | `j7_nominal` | sono le 500 righe nominali uniche; `j7_nominal` le porta già 32 volte, per 16000 righe e il 62.5807% dell'aggregato. Contarla di nuovo le duplicherebbe. |
| `j14_post_mismatch_671` | 671 | **sottopartizione** | `j14_recovery_854` | 671 + 183 = 854, esattamente il blocco padre |
| `j14_pre_mismatch_183` | 183 | **sottopartizione** | `j14_recovery_854` | idem |

**Le statistiche per riga su queste viste restano valide** e sono corrette come riportate: la
ripetizione non altera una statistica per riga. Era rappresentato male soltanto il loro **peso
aggregato**.

---

## 4. Cosa questa correzione NON tocca

- Ogni RMSE, MAE, massimo assoluto e signed mean shift delle tabelle T3, in unità normalizzate e
  fisiche — sono statistiche **per riga**, indipendenti da come si riporta la quota aggregata.
- La misura della massa d'errore quadratico iniziale, calcolata direttamente sugli array da 25567
  righe: **non ha mai usato** `share_of_aggregate_pct`.
- Le distanze T5 e l'ordinamento temporale.
- Ogni verdetto da H1 a H6.
- La decisione architetturale registrata: J8 5/6 miglior attore closed-loop, J11 4/6, J15R1 0/6,
  J8 futuro parent operativo con update incrementale B-only vincolato, J11 e J14 evidenza
  diagnostica, nessun artefatto July in lineage.
- La riformulazione DAgger.

È una correzione di **numeri e inquadramento**, non di misure né di conclusioni.

---

## 5. Artefatti

**Prodotti da questa correzione, senza sovrascrivere nulla:**

| artefatto | SHA-256 |
|---|---|
| `v26c_j17_architect_audit_addendum_weighting_correction_2026-08-27.json` | `f3ba6f042e1a8457710c9996f89acf61391d45c341ec27c7484181080f56d52c` |
| questo report | riportato nel riepilogo di consegna |

**Pinnati e verificati byte-identici — i due addendum originari:**

| artefatto | SHA-256 |
|---|---|
| `v26c_j17_architect_audit_addendum_2026-08-27.json` | `9f9dde817e4ad169ea05ecffb071f23af90d2570bfbb5c130b2777bedf9cea79` |
| `reports/user/2026-08-27_v26c_j17_architect_audit_addendum.md` | `fa53184fca4a75b2ef369b0f800f690fe742544810a6bef70f7921335973f46e` |

**Pinnati e verificati byte-identici — i cinque artefatti J17:**

| artefatto | SHA-256 |
|---|---|
| `v26c_j17_prereg_causal_diagnosis.json` | `1e765477660e0f0c7d80b91cd677889b974394b1c4dd12b85cfa24bcb991fba4` |
| `v26c_j17_architect_go_2026-08-27.json` | `ac5e063750f8284824acebd6815def09561a940200b3e30e3b9ae441155478ca` |
| `v26c_j17_diagnostic_measurements.json` | `5c5f309a5cde39a6233d52afe65d90be17a999879ea8acbecb64f53429434e5c` |
| `v26c_j17_reconciliation_and_corrections.json` | `d165b86732ce78000355f64b220382e6723edccbfc0a65f0e4032189b9ad2d56` |
| `reports/user/2026-08-27_v26c_j17_diagnosi_causale_regressione_j16.md` | `80562064f2b274014bf42794824b67b04e0c34f3367b2a63f59385cc0495df8a` |

Entrambi i file sono stati scritti con strumenti di editing basati su patch. Nessun subagente usato.

---

## 6. Invarianti verificate

- `j18_runs`, `j17_runs`, `g_i_runs`, `ppo_runs`, `critic_runs`: **assenti**.
- Nessun lock, nessuno staging.
- **Nessun artefatto preesistente modificato**: i due addendum e i cinque J17 sono byte-identici ai
  pin sopra.
- **Zero** fit, rollout, environment, critic, PPO, collection.
- **Semi 126, 127, 128**: non letti, non generati, non usati. Fase G–I non aperta.
- Nessuna modifica a FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione.
- **Worktree sporco dell'utente preservato**, invariato.

---

## 7. TODO propagati

- **La correzione proposta non è eseguita**: parent J8, update B-only vincolato. Richiede un tuo GO.
- **`nominal_mean_shift` dichiarato e non misurato** nel runner J15R1.
- **`policy_std` sempre `null`** — difetto cosmetico ereditato da J12.
- **Nessuna leaf pinna il runner che l'ha scritta**; per J15R1 due runner puntano alla stessa leaf.
- **Il fit J11 non è bit-riproducibile** dai propri artefatti; J15R1 lo è.
- **La leaf J2 non ha `commit_verification.json`** né `actor_digest`.
- **`best_validation_mse` contaminato** in entrambi i fit dalla ripetizione dei blocchi.
- **La deviazione pre-breccia non è rilevabile su singola run**: servirà un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 8. STOP

Correzione additiva completata. **J18 non preparato e non iniziato.**

**Fermo in attesa del tuo audit.**
