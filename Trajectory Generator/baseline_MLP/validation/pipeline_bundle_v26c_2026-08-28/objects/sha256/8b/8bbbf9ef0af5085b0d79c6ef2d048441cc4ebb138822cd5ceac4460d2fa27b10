# V26C J18 — Esecuzione dell'update B-only vincolato su J8

**Data**: 2026-08-27
**Stadio**: `V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE` — **esecuzione**
**Esecutore**: Opus 5, effort xhigh
**Esecuzione**: **una sola volta**, sotto GO architetto `fb12d3ec…`, nessun retry.

## VERDETTO OFFLINE: `FAIL_CLOSED_NO_SURVIVOR`

**Zero candidati su sedici** hanno superato tutti gli hard gate. **Nessun actor è stato scritto.**
Nessuna soglia rilassata, nessun valore aggiunto agli assi, nessun candidato rilanciato.

---

## 1. Cosa è stato eseguito

GO verificato prima del lancio: SHA-256 `fb12d3ec89183e9211a3b240d1177f23386d70802b793d9da02144c840e9a2e4`,
`validate_go` → `valid=true`, `problems=[]`, **sette pin** tutti allineati ai file su disco.
`j18_runs` **assente** prima del lancio.

Comando eseguito esattamente una volta, exit code **0**.

| protocollo | valore registrato |
|---|---|
| candidati | **16** |
| seed | 123 |
| epoche | 200 |
| batch | 128 |
| batch per epoca | **33** |
| step optimizer per candidato | **6600** — identico per tutti e 16, verificato |
| early stopping | nessuno |
| retry | nessuno |
| chunk finale corto | tenuto |
| dataset | **4221** righe, `{A:500, B:14, C:2497, D:1210}`, disgiunti, priorità A>B>C>D |
| stimatore | `L_M = (N/|M|) · Σ w_block(i)/N_block(i) · e_i` |
| RNG | `default_rng(123)`, ricostruito identico per candidato |

**Equivalenza raw/scaled prima dell'optimizer**: numpy `3.205e-10`, torch `1.192e-07`, tolleranza
`1e-06`, round-trip pesi del parent **bit-identico** → `ok=true`.

**Assorbimento delle scale, sui 16 candidati**: numpy max `5.821e-09`, torch max `1.788e-07`, ben
dentro `1e-06`.

**Il round-trip dei pesi post-training NON è bit-identico**, come previsto dall'addendum v3: fra
**43 e 66 entry su 8960** differiscono per candidato. Se avessi tenuto la pretesa di bit-identità,
**tutti e sedici** i candidati sarebbero stati abortiti per una ragione che non ha nulla a che vedere
con la loro qualità. La correzione v3 era necessaria, non cosmetica.

---

## 2. Esito dei gate

| gate | falliti su 16 |
|---|---|
| **G1** `preservation_max_drift_C` | **16** |
| **G2** `preservation_max_drift_D` | **16** |
| **G3** `preservation_bias_C` | 10 |
| **G4** `preservation_bias_D` | 5 |
| G5 `on_policy_improvement_B` | 0 |
| G6 `teacher_non_regression_A` | 0 |
| G7 `logstd_byte_identical` | 0 |
| G8 `no_critic` | 0 |
| G9 `clock_columns_zero` | 0 |
| G10 `key_set_identical` | 0 |
| G11 `finiteness` | 0 |

**Il fit ha imparato benissimo i target e ha rotto gli anchor.**

- **G5 passa 16/16, con enorme margine**: `MSE_B` scende a **0.1%–2.9%** del baseline
  `0.1909311190718203`. Il gate chiedeva ≤50%. Il minimo è `0.000113` (candidato 1), cioè **0.1%**.
- **G6 passa 16/16**: `MSE_A` fra `0.000409` e `0.001803`, tutti **sotto** il tetto di non-regressione
  `0.004108`. Nessun candidato peggiora sul teacher; tutti migliorano.
- **G1 e G2 falliscono 16/16**: il drift massimo sugli anchor va da **0.04697 a 0.22930** su C e da
  **0.01982 a 0.26322** su D, contro un budget di **0.005**. Il candidato meno lontano è il **13**
  (λ=30, lr=5e-05) con **0.04697 = 9.4× il budget**; il peggiore è il 2 a **45.9×**.

### Per candidato

| c | λ | β | lr | G1 drift C | G2 drift D | G3 bias C | G4 bias D | MSE_B | % baseline | MSE_A | epoca* |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 1 | 1 | 1e-5 | 0.17491 | 0.20684 | 0.004499 | 0.002380 | 0.000560 | 0.3% | 0.001135 | 199 |
| 1 | 1 | 1 | 5e-5 | 0.12247 | 0.06951 | 0.006389 | 0.005602 | **0.000113** | **0.1%** | 0.000522 | 184 |
| 2 | 1 | 5 | 1e-5 | **0.22930** | **0.26322** | 0.002738 | 0.001383 | 0.000344 | 0.2% | 0.001803 | 199 |
| 3 | 1 | 5 | 5e-5 | 0.15145 | 0.18793 | 0.002686 | 0.000592 | 0.000119 | 0.1% | 0.000913 | 179 |
| 4 | 3 | 1 | 1e-5 | 0.15569 | 0.15337 | 0.002065 | 0.000954 | 0.000868 | 0.5% | 0.001028 | 199 |
| 5 | 3 | 1 | 5e-5 | 0.09562 | 0.03499 | 0.002613 | 0.002672 | 0.000181 | 0.1% | **0.000409** | 196 |
| 6 | 3 | 5 | 1e-5 | 0.18648 | 0.23625 | 0.001683 | 0.000253 | 0.000351 | 0.2% | 0.001551 | 199 |
| 7 | 3 | 5 | 5e-5 | 0.11835 | 0.10594 | 0.002490 | 0.000928 | 0.000156 | 0.1% | 0.000704 | 198 |
| 8 | 10 | 1 | 1e-5 | 0.12741 | 0.09123 | 0.000817 | 0.000229 | 0.002095 | 1.1% | 0.000945 | 200 |
| 9 | 10 | 1 | 5e-5 | 0.05179 | 0.02269 | 0.002436 | 0.001783 | 0.000202 | 0.1% | 0.000486 | 199 |
| 10 | 10 | 5 | 1e-5 | 0.16705 | 0.18257 | 0.000916 | 0.000104 | 0.000586 | 0.3% | 0.001308 | 199 |
| 11 | 10 | 5 | 5e-5 | 0.10089 | 0.05223 | 0.004420 | 0.003610 | 0.000146 | 0.1% | 0.000723 | 179 |
| 12 | 30 | 1 | 1e-5 | 0.09264 | 0.05496 | 0.000674 | 0.000319 | 0.005533 | 2.9% | 0.000895 | 200 |
| **13** | **30** | **1** | **5e-5** | **0.04697** | **0.01982** | 0.000389 | 0.000400 | 0.000307 | 0.2% | 0.000560 | 191 |
| 14 | 30 | 5 | 1e-5 | 0.14423 | 0.12358 | 0.000570 | 0.000935 | 0.001481 | 0.8% | 0.001074 | 198 |
| 15 | 30 | 5 | 5e-5 | 0.06090 | 0.03031 | 0.001385 | 0.000921 | 0.000184 | 0.1% | 0.000648 | 192 |

*epoca del checkpoint selezionato. Per **14 candidati su 16 non è la 200ª** — epoche selezionate:
179, 179, 184, 191, 192, 196, 198, 198, 199×6, 200×2. La regola preregistrata (minimo composito,
pareggi all'epoca più precoce) ha quindi scartato una coda peggiorativa nella grande maggioranza dei
casi, non in una minoranza.

---

## 3. Il drift è concentrato, e il colpevole è il ginocchio

Questo è il fatto più informativo dell'intera esecuzione, ed è **misura**, non interpretazione.

**Rapporto massimo/RMSE sugli anchor: da 5.4 a 15.5** su tutti e sedici i candidati. Il drift non è
diffuso: la stragrande maggioranza degli stati anchor è preservata, e pochi stati sono spostati
molto.

Per il candidato 13, il migliore su questo asse:

| blocco | n | RMSE | massimo | knee max | ankle max |
|---|---|---|---|---|---|
| C anchors | 2497 | **0.003038** | 0.046971 | **0.046971** | 0.008081 |
| D supporto | 1210 | **0.002644** | 0.019820 | **0.019820** | 0.005031 |

**L'RMSE sugli anchor del candidato 13 è 0.003038, cioè già dentro il budget di 0.005.** È solo il
peggiore stato singolo a sforare, di 15.5× rispetto all'RMSE.

**In tutti e sedici i candidati il massimo del blocco C coincide con il massimo del ginocchio.** La
caviglia resta molto più vicina: 0.008081 contro 0.046971 nel candidato 13.

**Inferenza, non causa dimostrata**: la loss minimizza un **errore quadratico medio** su C e D,
mentre i gate G1/G2 vincolano un **massimo**. Nulla nell'obiettivo penalizza la coda.

A parità di β e lr, alzare λ da 1 a 30 riduce il massimo su C:

| β, lr | λ=1 | λ=30 | fattore |
|---|---|---|---|
| β=1, lr=1e-5 | 0.17491 (c0) | 0.09264 (c12) | 1.9× |
| β=1, lr=5e-5 | 0.12247 (c1) | **0.04697** (c13) | 2.6× |
| β=5, lr=1e-5 | 0.22930 (c2) | 0.14423 (c14) | 1.6× |
| β=5, lr=5e-5 | 0.15145 (c3) | 0.06090 (c15) | 2.5× |

Trenta volte il peso di preservation compra meno di tre volte di riduzione del massimo, e resta a
9.4× dal budget. È **coerente** con la lettura sopra — un peso maggiore su una media non controlla
una coda — ma **non la dimostra**: un λ ancora maggiore non è stato provato, e non lo sarà senza una
nuova preregistrazione.

Osservazione secondaria, sempre misurata: **G3 e G4 passano quasi sempre ad alto λ** — il bias medio
del candidato 13 è `0.000389` e `0.000400`, ben dentro `0.0014557`. Gli anchor sono preservati **in
media** e violati **in punti specifici**.

---

## 4. Integrità del leaf

Leaf: `j18_runs/j18_b_only_update_v26c_2026-08-27_r1/` — **cinque file, nessuna `rl_module`**.

| file | SHA-256 |
|---|---|
| `v26c_j18_b_only_update_receipt.json` | `600c3c45a49d39da93e4e6e890a479e923e51d456cf92c4faa245031929452db` |
| `v26c_j18_selection.json` | `8ef47ef6343399dfded8ffe85ec109a6b420d0c804c32ba19d7b8906348aac30` |
| `v26c_j18_candidate_metrics.json` | `f4b8688dd8d4d0c4f330064dce4ac9ccd021e9529669a04e97ea615f226f36f3` |
| `history.json` | `3eaba5128c1d0734a60127cddeb5f383d794fa59951a3c7cf2eb9e8dd4e46eee` |
| `commit_verification.json` | `15bd6e5d3510ce7974d5df0a4669077a3d5995332e8d56cfd22447a3d44d3247` |

- **Verifica post-commit**: `ok=true`, `problems=[]`, 3 file ri-risolti dal receipt committato,
  ri-hashati e confrontati sia col receipt sia coi byte in staging.
  `aggregate_digest = e6921b116dbfba7b19aaf60853d128dd237b3c1693a835f23862fa54d7501e1b`.
- **Marcatore born-invalid rimosso** solo dopo la verifica; **nessuna directory di staging residua**.
- Il receipt **non contiene il proprio hash**; ogni hash che dichiara è stato ricalcolato e
  corrisponde.
- **Tutti e sedici i candidati sono registrati** in `v26c_j18_candidate_metrics.json`, con metriche e
  esito di ogni gate G1–G11, benché nessuno sia sopravvissuto.
- **Zero findings non finiti** su tutti e sedici: la sanitizzazione non ha dovuto intervenire, e il
  percorso finito è rimasto numericamente intatto.
- Il receipt pinna parent (`9c5b1571…` byte reali, `actor_digest 6a879714…`), i cinque artefatti di
  preregistrazione, il runner e i **sette** pin del GO.

---

## 5. Limiti — cosa questo risultato NON dice

- **Nessuna qualifica closed-loop è stata eseguita.** Nessun rollout, nessun environment, nessun
  PPO, nessuna nuova collection. Il verdetto è **esclusivamente offline**.
- Il `FAIL_CLOSED` **non dice che l'approccio B-only sia sbagliato**. Dice che, con questa griglia e
  queste soglie, nessun candidato ha tenuto il drift massimo sugli anchor entro 0.005.
- **Non dice che un candidato bocciato fallirebbe in closed loop.** J17 ha già mostrato che
  l'offline non certifica il closed loop; vale anche al contrario — un gate offline superato non
  garantisce il passaggio, e uno fallito non garantisce il fallimento. Sono filtri, non oracoli.
- La soglia 0.005 era derivata da una perturbazione **a media nulla** tollerata da J8, usata
  dichiaratamente solo per fissare un ordine di grandezza. Un drift sistematico non è la stessa cosa,
  e il report di readiness lo dichiarava già.
- **J8 resta l'attore operativo migliore, 5/6**, invariato e non toccato: `9c5b1571…`.

---

## 6. Invarianti verificate

- Eseguito **una sola volta**, exit code 0, **nessun retry**.
- **Nessun actor scritto**; nessuna `rl_module` nel leaf.
- Nessun rollout, environment, critic, PPO, nuova collection, cambio di soglia o griglia.
- **J8 invariato**: `9c5b157156e6b9c2…`. J2, J7, J9R1, J10R1 non toccati.
- I **sette file pinnati non sono stati modificati**: runner `d3949631ddd135c5…`,
  test `6050f4c062f64c41…`, e i cinque JSON ai loro hash di GO.
- Semi 126, 127, 128: non letti, non generati, non usati.
- FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione: non modificati.
- Worktree sporco preservato. Nessun subagente usato.

---

## 7. TODO propagati

- **Decisione architetturale aperta**: come procedere dopo un fail-closed 16/16 su G1/G2. Non la
  prendo io e non apro la fase successiva.
- Se si vorrà indagare la coda del drift servirà una **nuova preregistrazione**: la griglia e le
  soglie attuali sono esaurite e non vanno rilassate a posteriori.
- I gate offline filtrano, non certificano: la riqualifica closed-loop A–F resta il verdetto
  vincolante, non eseguita.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- `policy_std` sempre `null`, ereditato da J12.
- Nessuna leaf pinna il runner che l'ha scritta — **J18 lo fa**: il receipt pinna
  `runner_sha256`.
- Il fit J11 non è bit-riproducibile dai propri artefatti; J15R1 lo è.
- `best_validation_mse` contaminato in entrambi i fit dalla ripetizione dei blocchi.
- La deviazione pre-breccia non è rilevabile su singola run: serve un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 8. STOP

Esecuzione singola completata, audit read-only del leaf completato, verdetto offline
**`FAIL_CLOSED_NO_SURVIVOR`**, nessun actor prodotto.

**Fase successiva non aperta. Fermo in attesa del tuo audit.**
