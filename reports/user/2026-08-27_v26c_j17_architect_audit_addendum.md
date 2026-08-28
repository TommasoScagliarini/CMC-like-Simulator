# V26C J17 — Addendum di chiusura dopo l'audit dell'architetto

**Data**: 2026-08-27
**Stadio**: `V26C_J17_CAUSAL_DIAGNOSIS` — chiusura additiva
**Natura**: colmatura di lacune, **additiva**. Nessun artefatto J17 è stato modificato.
**Nessun fit, rollout, environment, critic, PPO, collection. J18 non iniziato.**
**Report obbligatorio di fine fase.**

---

## 1. Decisione architetturale, registrata

Decisione già presa da Codex, architetto e gate owner. La registro; non è mia da riaprire.

- **Miglior attore closed-loop attuale: J8, 5/6.**
- J11 = 4/6. J15R1 = 0/6.
- **Il futuro parent operativo sarà J8**, con update incrementale **B-only vincolato**.
- **J11 e J14 restano evidenza diagnostica**, non parent operativo.
- **Nessun checkpoint o dataset di luglio entra nella lineage.**

**Corroborazione dal record** (verifica, non ridiscussione): il receipt committato
`j9r1_runs/j9r1_closed_loop_v26c_2026-08-26_r1/v26c_j9r1_closed_loop_receipt.json`
(`c201b666f68451e4646fbaae3f149e9afaf873b5c94f99c95fc4286faaeda4f4`) registra
`cells_behavioural_pass: 5`, `cells_telemetry_valid: 6`, e per cella
**A=PASS B=FAIL C=PASS D=PASS E=PASS F=PASS**.

Il 5/6 verifica, e **l'unica cella fallita è proprio B** — esattamente ciò che un update B-only
prende di mira.

---

## 2. Lacuna 1 — T3 completo

La preregistrazione prometteva RMSE, MAE, massimo assoluto e signed mean shift, **per ciascuna
azione**, in unità **normalizzate e fisiche**, con sample count ed effective weighting, su **tutti**
i blocchi. L'artefatto J17 ne riportava un sottoinsieme. Ecco il tavolo completo.

Quantità: delta offline **J15R1 − J11**, entrambi gli attori sugli **stessi** input congelati
(aggregato committato J15R1, 25567 righe). Fisica = normalizzata × `max_delta_rad` = **0.35 rad**
(`action_mode: absolute`, dal config runtime pinnato). Il forward usato riproduce il `policy_mean`
del runtime a **1.58e-7**.

### Ginocchio

| blocco | n | RMSE | MAE | max\|·\| | shift | RMSE rad | MAE rad | max rad | shift rad |
|---|---|---|---|---|---|---|---|---|---|
| ancora nominale (A–D) | 500 | 0.017395 | 0.012705 | 0.124469 | +0.001979 | 0.006088 | 0.004447 | 0.043564 | +0.000693 |
| J7 nominale (tiled) | 16000 | 0.017395 | 0.012705 | 0.124469 | +0.001979 | 0.006088 | 0.004447 | 0.043564 | +0.000693 |
| J7 recovery | 713 | 0.018311 | 0.013927 | 0.144190 | +0.003757 | 0.006409 | 0.004875 | 0.050467 | +0.001315 |
| cella B unique | 500 | 0.018122 | 0.012918 | 0.089094 | −0.003804 | 0.006343 | 0.004521 | 0.031183 | −0.001331 |
| cella C unique | 500 | 0.020428 | 0.015191 | 0.089072 | −0.003938 | 0.007150 | 0.005317 | 0.031175 | −0.001378 |
| **J14 correttive** | **854** | **0.536005** | **0.355822** | **1.499600** | **+0.290697** | **0.187602** | **0.124538** | **0.524860** | **+0.101744** |
| J14 post-mismatch | 671 | 0.602793 | 0.433148 | 1.499600 | +0.350626 | 0.210977 | 0.151602 | 0.524860 | +0.122719 |
| J14 pre-mismatch | 183 | 0.091792 | 0.072292 | 0.228840 | +0.070960 | 0.032127 | 0.025302 | 0.080094 | +0.024836 |

### Caviglia

| blocco | n | RMSE | MAE | max\|·\| | shift | RMSE rad | MAE rad | max rad | shift rad |
|---|---|---|---|---|---|---|---|---|---|
| ancora nominale (A–D) | 500 | 0.010140 | 0.006647 | 0.101730 | +0.001345 | 0.003549 | 0.002327 | 0.035606 | +0.000471 |
| J7 nominale (tiled) | 16000 | 0.010140 | 0.006647 | 0.101730 | +0.001345 | 0.003549 | 0.002327 | 0.035606 | +0.000471 |
| J7 recovery | 713 | 0.010367 | 0.007147 | 0.115694 | +0.001878 | 0.003628 | 0.002501 | 0.040493 | +0.000657 |
| cella B unique | 500 | 0.009378 | 0.006371 | 0.047955 | −0.000605 | 0.003282 | 0.002230 | 0.016784 | −0.000212 |
| cella C unique | 500 | 0.012632 | 0.008465 | 0.062806 | −0.001043 | 0.004421 | 0.002963 | 0.021982 | −0.000365 |
| **J14 correttive** | **854** | **0.167995** | **0.111492** | **0.590176** | **−0.022924** | **0.058798** | **0.039022** | **0.206562** | **−0.008023** |
| J14 post-mismatch | 671 | 0.188577 | 0.135377 | 0.590176 | −0.034357 | 0.066002 | 0.047382 | 0.206562 | −0.012025 |
| J14 pre-mismatch | 183 | 0.036229 | 0.023916 | 0.156330 | +0.018997 | 0.012680 | 0.008371 | 0.054716 | +0.006649 |

### Effective weighting

| blocco | uniche | repeat | righe | % aggregato |
|---|---|---|---|---|
| J7 nominale | 500 | 32 | 16000 | 62.5807% |
| J7 recovery | 713 | 1 | 713 | 2.7888% |
| cella B | 500 | 8 | 4000 | 15.6455% |
| cella C | 500 | 8 | 4000 | 15.6455% |
| **J14 correttive** | **854** | **1** | **854** | **3.3402%** |
| totale | 3067 | — | 25567 | 100% |

Le righe con `n = 500` nelle tabelle sopra sono i **blocchi unici**; i tiled sono riportati a parte
per rendere esplicito che la ripetizione non cambia la statistica per riga ma cambia il peso.

**Lettura.** Sul ginocchio il segno dello shift **si inverte** fra le ancore nominali (+) e le celle
B e C (−); sulle 854 righe correttive è **due ordini di grandezza più grande** (+0.1017 rad contro
+0.0007), e concentrato nella metà post-mismatch (+0.1227 rad contro +0.0248 della pre-mismatch).

---

## 3. Lacuna 2 — T5, distanza dall'intero aggregato

L'artefatto J17 riportava la distanza degli stati J16 dalle **sole ancore**. Mancava quella
dall'**intero** aggregato di training.

### Definizione, dichiarata

- **Sottospazio**: le 33 colonne non-clock, indici 2..34. Le colonne 0 e 1 sono escluse perché
  hanno peso **esattamente zero** nel primo layer di entrambi gli attori e sono registrate grezze
  nelle trace mentre i dataset le portano proiettate a zero: includerle aggiungerebbe uno scarto
  costante di 1.0 privo di significato comportamentale.
- **Scaling**: deviazione standard per-feature calcolata sull'**intero** aggregato J15R1 (25567
  righe); le feature a std nulla mappate a 1.0.
- **Metrica**: L2 euclidea nello spazio standardizzato.
- **Riduzione**: per ogni stato visitato, il **minimo** sull'insieme di riferimento.
- **Riferimenti**: (a) ancore = 1500 righe; (b) **intero aggregato** = 3060 righe distinte su 25567.

### Distanza mediana dagli stati visitati, per cella

| cella | J11 in J12 → aggregato | J15R1 in J16 → aggregato | J11 → ancore | J15R1 → ancore |
|---|---|---|---|---|
| A | 0.429 | **2.493** | 0.475 | 2.516 |
| B | 0.466 | **2.839** | 0.473 | 2.907 |
| C | 0.419 | **3.084** | 0.427 | 3.116 |
| D | 0.450 | **1.101** | 0.456 | 1.119 |
| E | **0.000** | 2.615 | 1.947 | 2.729 |
| F | **0.000** | 2.513 | 4.474 | 2.691 |

Le celle E ed F di J12 hanno distanza **esattamente 0.000** dall'aggregato perché **quegli stati
sono il blocco J14 dell'aggregato**. Il rapporto è una divisione per zero: lo riporto come
indefinito, non come un numero grande.

### Frazione di stati oltre 3σ dall'intero training set

| cella | J11 in J12 | J15R1 in J16 |
|---|---|---|
| A | **0.0%** | **40.8%** |
| B | **0.0%** | **47.2%** |
| C | **0.0%** | **54.1%** |
| D | **0.0%** | **16.6%** |
| E | **0.0%** | **40.2%** |
| F | **0.0%** | **39.7%** |

**EVIDENZA**: J11 non esce **mai** oltre 3σ dal proprio training set, in nessuna delle sei celle.
J15R1 vi passa fra il **16.6% e il 54.1%** di ogni episodio.

**INFERENZA**: la deriva di supporto è massiccia, ma **non è l'origine** del divario — che nasce al
passo 1 su un'osservazione bit-identica. È una conseguenza che poi amplifica.

### Ordinamento temporale, con l'offset corretto

Semantica di registrazione verificata: `knee_rad[i]` e `ankle_rad[i]` sono lo stato **pre-step**
all'istante `time_before[i]`, bit-identici a `observation[i][2]` e `[4]`; `raw_action[i]` agisce
sull'intervallo `[time_before[i], time_after[i]]`; `penetration_m[i]` è il valore **post-step**
all'istante `time_after[i] = time_before[i+1]`. Passo dell'ambiente 0.01 s, nessuna serie sub-step.

1. **Azione**: differisce al passo 1 su un'osservazione **bit-identica**. Causalmente prima **per
   costruzione**: non può derivare da una differenza di stato, perché lo stato è lo stesso.
2. **Cinematica**: differisce ai passi 2–3 (>1e-6 rad), il primo istante che la registrazione può
   esprimere.
3. **Penetrazione** oltre 0.020: passi 11, 27, 28 o 47 secondo la cella.

**Limite dichiarato**: fra 2 e 3 il divario è **parzialmente mascherato**. In A, B, D, E, F la
penetrazione è identicamente 0.0 in **entrambe** le run per i primi 10–14 passi, col piede in aria:
una differenza in una grandezza nulla in entrambe non è misurabile. **Nella cella C**, dove il piede
è già caricato al reset, differenza cinematica e differenza di penetrazione compaiono nello **stesso
istante fisico** e **non sono ordinabili**.

---

## 4. Lacuna 3 — la formulazione DAgger, corretta

**Cosa avevo scritto.** Il report J17 descriveva il refit fresco come «uno scostamento dalla
premessa di DAgger, che vuole gli stati raccolti dalla policy che si sta migliorando».

**Perché era troppo ampia.** Rifittare da zero su un aggregato accumulato **non viola
universalmente DAgger**. DAgger aggrega dataset fra iterazioni, e rifittare l'aggregato da
un'inizializzazione fissa è una variante legittima dell'algoritmo. La mia formulazione generalizzava
oltre l'evidenza.

**La conclusione ammessa**, specifica a **questa** implementazione, è la congiunzione di quattro
fatti misurati:

1. gli stati correttivi furono raccolti da **J11**;
2. il fit fu inizializzato da **J2**, un attore diverso;
3. il **3.34%** delle righe portava l'**84.81%** della massa d'errore iniziale, e la massa totale
   dell'aggregato era **6.58×** quella di J11;
4. le ancore erano **funzionalmente non protette** — auto-distillate dall'inizializzazione, quindi
   con 5.6e-10 di gradiente al passo 0, e con l'unico vincolo un penalty L2 di peso 0.01 nello
   **spazio dei parametri**, senza alcun vincolo nello spazio delle **uscite**.

**Non affermo**: che il refit fresco su aggregato sia sbagliato in generale; che DAgger richieda un
update incrementale; che uno solo fra (1)–(4) avrebbe prodotto da solo questo esito.

Questa formulazione **supersede** il testo delle sezioni 3.9 e 7 del report J17. Quel report resta
**byte-identico**: la correzione vive qui.

---

## 5. Riconciliazione con gli artefatti immutabili

Ogni numero di questo addendum è ricalcolato dagli stessi artefatti pinnati e **concorda** con
quanto già registrato in J17. I cinque artefatti sono pinnati e **non modificati**:

| artefatto | SHA-256 |
|---|---|
| `v26c_j17_prereg_causal_diagnosis.json` | `1e765477660e0f0c7d80b91cd677889b974394b1c4dd12b85cfa24bcb991fba4` |
| `v26c_j17_architect_go_2026-08-27.json` | `ac5e063750f8284824acebd6815def09561a940200b3e30e3b9ae441155478ca` |
| `v26c_j17_diagnostic_measurements.json` | `5c5f309a5cde39a6233d52afe65d90be17a999879ea8acbecb64f53429434e5c` |
| `v26c_j17_reconciliation_and_corrections.json` | `d165b86732ce78000355f64b220382e6723edccbfc0a65f0e4032189b9ad2d56` |
| report J17 | `80562064f2b274014bf42794824b67b04e0c34f3367b2a63f59385cc0495df8a` |

**Prodotti da questo addendum:**

| artefatto | SHA-256 |
|---|---|
| `v26c_j17_architect_audit_addendum_2026-08-27.json` | `9f9dde817e4ad169ea05ecffb071f23af90d2570bfbb5c130b2777bedf9cea79` |
| questo report | (in calce) |

Nessun subagente è stato usato per questo addendum. I due usati in J17 erano read-only e sono già
riconciliati nel report J17.

---

## 6. Invarianti verificate

- `j18_runs`, `j17_runs`, `g_i_runs`, `ppo_runs`, `critic_runs`: **assenti**.
- Nessun lock, nessuno staging.
- **Nessun attore, dataset, configurazione, leaf o report preesistente modificato.**
- **Zero** fit, rollout, environment, critic, PPO, collection.
- **Semi 126, 127, 128**: non letti, non generati, non usati. Fase G–I non aperta.
- **Worktree sporco preservato**: i tre file tracciati modificati dall'utente sono invariati.
- Nessuna modifica a FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione.
- July resta solo informativa.

---

## 7. TODO propagati

- **La correzione proposta non è eseguita**: parent J8, update B-only vincolato. Richiede un tuo GO.
- **`nominal_mean_shift` dichiarato e non misurato** nel runner J15R1 — aperto.
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

Chiusura additiva completata. **J18 non preparato e non iniziato.**

**Fermo in attesa del tuo audit.**
