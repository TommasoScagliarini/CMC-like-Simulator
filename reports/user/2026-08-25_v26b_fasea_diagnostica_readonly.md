# Fase A — diagnostica read-only del DAgger V26B

**Data**: 2026-08-25 · **Stadio**: `V26B-FASEA-DIAGNOSTICS`
**Natura**: sola lettura. Nessun fit, training, rollout o candidato. Nessun file di produzione modificato.

---

## 1. File creati (solo additivi)

| file | SHA-256 |
|---|---|
| `Trajectory Generator/baseline_MLP/validation/v26b_bridge_2026-08-24/v26b_fasea_diagnostics.py` | `a43b70bcf7badba924efdfb5b2866f144144a130b82337a8e2b84984f9843aad` |
| `.../test_v26b_fasea_diagnostics.py` | `2edba41af3eb489553c3ba6cee861b0ea206a75fbe0d54402395ff46b6cfe039` |
| receipt `runs/.../2026-08-24_V26B_anchors_r1/diagnostics/faseA/v26b_fasea_receipt.json` | `716a779da817d6e85fb595357caa63badc024e92d2a7693e9f2d4b5394784363` |
| receipt supplementare (varianti di troncamento) `.../v26b_fasea_receipt_supplement.json` | `2328126860e6760ba13cdd00ed8034712da62f89e7571b193ec5d7a92e0c0086` |

`git status` conferma zero modifiche a `osim_trj_cmc_like.py`, `target_domain_imitation.py`, `target_domain_noise_adaptation.py`, `prosthetic_phase_fsm.py`, `online_grf.py`. Portabilità: solo `pathlib`, nessuna chiamata di shell, nessun separatore specifico di OS — verificato da test su macOS arm64, valido per Windows x86.

**Comandi eseguiti**
```
python test_v26b_fasea_diagnostics.py      -> {"selftest": "PASS", "checks": 827}
python v26b_fasea_diagnostics.py --run     -> receipt + supplemento
```

## 2. Provenance del parent — fail-closed

Spina dorsale verificata digest per digest:
`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best` **`5bbc6cbd…`** → `V1_35D_transplant` **`ae846220…`** → `S0D_35D_DISTILLED` **`481dd0d2…`**.

Regola di contaminazione: un artefatto è **DAgger-contaminato** se il proprio receipt porta un blocco dataset on-policy (`trace_repeat`, `prefix_rows`, `raw_visited_rows`, `on_policy_share`) **oppure se lo è un qualunque antenato**. La contaminazione è ereditata lungo la catena, non dedotta dal nome.

- **Contaminati, esclusi come parent**: `REV4B`, `REV4C`, `REV4D`, `REV4E`.
- **Puliti** (figli diretti di S0D, nessun round DAgger nella catena), dal più recente: `S1C2Z_L20` (19:31:23), `S1C2Z_L10`, `S1C2Z_L05`, `S1C1_W8`, `S1C1_W4`, `S1C1_W2`, `S1B_A6…A1`, `S1A` (17:34:50).

Il guard rifiuta qualunque `derived_from` che contenga un marcatore di percorso luglio.

**AMBIGUITÀ CHE NON RISOLVO** — "l'ultimo artefatto 35D pulito" non è univoco:
- **cronologicamente ultimo** è `S1C2Z_L20_35D_NONDEPLOYABLE` (`71d21f309ccf1d…`), ma su L20 esiste una tua decisione permanente: *«L20 NON è promosso né diventa baseline/init/source»*;
- **l'unico pulito mai autorizzato come init/anchor** è `S1A_IK_AB06_35D_NONDEPLOYABLE` (`8f3e0ce17eff7c…`), padre diretto di tutto il ramo DAgger.

Il tool li elenca entrambi con digest e timestamp e non sceglie. **La scelta è tua.**

## 3. Mismatch discreto — la regola "phase-aligned" di luglio

Usa **verbatim** `target_domain_noise_adaptation.truncate_before_discrete_mismatch` (importata, non ricopiata; il test lo verifica). Nominal = stati del corpus teacher allo **stesso indice assoluto**. Indici discreti derivati dalla regola di luglio: `[11,12,13,17,18,19,20,21]`.

| traccia | primo mismatch | righe trattenute | STANCE | SWING | eventi |
|---|---|---|---|---|---|
| S1A (392) | **step 14** | 13 | 13 | 0 | 0 |
| REV4C (116) | **step 62** | 61 | 61 | 0 | 0 |
| REV4E (372) | **step 62** | 61 | 61 | 0 | 0 |

Natura dei mismatch:
- **S1A step 14**: unica feature `online_left_in_contact` (teacher 0, visitata 1) — un **flicker istantaneo del flag di contatto**, non una divergenza di fase.
- **REV4C/REV4E step 62**: `online_left_toe_off` più l'intero blocco FSM — il teacher fa toe-off, lo studente resta in stance. **Divergenza di fase genuina.**

Varianti riportate nel supplemento (il tool non ne sceglie nessuna):

| variante | S1A | REV4C | REV4E |
|---|---|---|---|
| tutte le 8 discrete (produzione) | 14 | 62 | 62 |
| solo blocco FSM/fase `[17..21]` | 62 | 62 | 62 |
| solo one-hot FSM `[17,18,19]` | 62 | 62 | 62 |

**Conseguenza strutturale**: sotto **ogni** variante il prefisso trattenuto è **≤ 61 righe, 100% STANCE, zero SWING, zero eventi**. La regola di luglio, applicata fedelmente su questo lineage, è *più* restrittiva del cap a 68 che produsse REV4C e **non può correggere lo swing**. Il motivo è che studente e teacher divergono in fase esattamente allo step 62.

## 4. Eseguibilità delle label — criterio preregistrato: FAIL sopra il 10%

Catena di riferimento congelata dal config v3 risolto: limiter target-to-target, **knee 2,5 rad/s**, **ankle 2,0 rad/s**, `policy_knots = 1`, `segment_duration = 0.01 s`, tolleranza di raggiungimento 0,02 rad (un passo di slew della caviglia).

L'aritmetica locale è verificata **differenzialmente** contro una trascrizione letterale di `osim_trj_cmc_like._limit_target_slew`: 200 casi casuali, uguaglianza **bit-identica**, nella struttura `policy_knots=1` (una chiamata per step). Il conteggio in forma chiusa è verificato sufficiente e minimale su altri 300 casi. Il limiter è lo stadio vincolante; il modello `butterworth3_jerk_limited` a 4,0 Hz aggiunge ritardo ulteriore, quindi le cifre sono un **limite inferiore**.

Soglie di ammissione per raggiungibilità, derivate dal **solo corpus teacher congelato** (p90): knee **0,19179938416928055** rad, ankle **0,22940547205507755** rad.

| traccia | righe ammesse | step necessari (med / p90 / max) | step residui nel segmento (med / min) | non eseguibili | verdetto |
|---|---|---|---|---|---|
| S1A | 157 | 5 / 9 / 19 | 126 / 22 | 0 (0,00%) | **PASS** |
| REV4C | 98 | 2 / 5,3 / 10 | 67,5 / 19 | 0 (0,00%) | **PASS** |
| REV4E | 260 | 1 / 6 / 11 | 57 / 1 | 9 (**3,46%**) | **PASS** |

Con ammissione per troncamento discreto: S1A 13 righe, REV4C 61, REV4E 61 — **0 non eseguibili** in tutti e tre.

**Stratificazione che segnalo esplicitamente**: in REV4E il sottoinsieme **SWING fallisce da solo** — 7 non eseguibili su 52 (**13,5%**, sopra la soglia del 10%) contro 2 su 208 in STANCE (1,0%). Le righe di swing hanno mediana 5 step necessari contro 24,5 residui, con code strette. L'aggregato passa (3,46%) solo perché lo stance domina numericamente. **Se il round successivo deve correggere lo swing, questo è il vincolo che si incontra per primo.**

## 5. Copertura e divieto di interpolazione cross-phase

| dataset | righe | STANCE | SWING | HS | TO | feature costanti |
|---|---|---|---|---|---|---|
| corpus teacher (500) | 500 | 219 | 281 | 3 | 3 | 3 |
| S1A | 392 | 132 | 260 | 0 | 1 | 7 |
| REV4C | 116 | **116** | **0** | 0 | 0 | **14** |
| REV4E | 372 | 263 | 109 | 2 | 2 | 3 |

Coppie cross-phase (teacher-parent in stato FSM diverso dalla riga visitata) fra le righe ammesse:

| traccia | ammissione per raggiungibilità | ammissione per troncamento discreto |
|---|---|---|
| S1A | **68 su 157** — interpolazione da restringere | 0 su 13 — CLEAN |
| REV4C | **37 su 98** — da restringere | 0 su 61 — CLEAN |
| REV4E | **78 su 260** — da restringere | 0 su 61 — CLEAN |

Il divieto è fail-closed: l'interpolazione è ammessa **solo** dove teacher-parent e riga visitata condividono lo stato FSM; ogni altra riga ammessa entra con `k = 0`. Il troncamento discreto rende la coerenza cross-phase automatica (0 coppie), perché tronca prima del confine di segmento.

## 6. Ambiguità e limiti dichiarati

1. **Quale parent pulito** — cronologico contro autorizzato (§2). Non risolta dal tool.
2. **Quale insieme discreto** per il troncamento: la regola di produzione include `online_left_in_contact`, e un flicker di un campione tronca S1A a 13 righe come farebbe una divergenza di fase. Le tre varianti sono riportate; la scelta è tua.
3. **Il limiter è un limite inferiore**: il filtro butterworth3 a 4 Hz e il governor aggiungono ritardo non contabilizzato. Un test che li includa richiederebbe di istanziare il modello di riferimento, cosa che ho evitato per restare in sola lettura.
4. **La tolleranza di raggiungimento** (0,02 rad = un passo di slew della caviglia) è una mia scelta di misura, non un parametro di produzione. Cambiandola cambiano i conteggi di step, non i verdetti (i margini sono ampi: mediana 1–5 step necessari contro 57–126 residui).
5. **Nessuna delle due ammissioni produce copertura di swing sotto la regola di luglio** (§3): il conflitto fra fedeltà al metodo di luglio e necessità di correggere lo swing è reale e non risolvibile dentro i vincoli attuali.

## 7. Stato

Nessun fit, training, rollout o candidato. Nessuna modifica a FSM v3, corridoio morfologico, contatto, reward, limiter/governor o guardie di sicurezza. Nessun artefatto di luglio usato operativamente: solo la funzione di troncamento importata come codice di produzione.

## 8. TODO propagati

- **TODO-2** — σ = 0.005 placeholder non risolto. *(aperto, ereditato)*
- **TODO-3** — B3 indeterminato: campo di fase identicamente zero su ogni traccia v3. *(aperto, ereditato)*
- **TODO-4** — Conflitto cammino/plantarflessione: REV4E resta l'unico con un ciclo valido plantarflettendo. *(aperto)*
- **TODO-5** — Tensione aritmetica interpolazione↔quota a copertura piena. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura come terminatore di REV4E. *(aperto)*
- **TODO-8** — Nuovo: la regola di troncamento discreto di luglio non produce copertura di swing su questo lineage (≤61 righe, 100% stance). *(nuovo)*
- **TODO-9** — Nuovo: le righe di swing ammesse per raggiungibilità falliscono il criterio del 10% da sole (13,5%). *(nuovo)*
