# V26C J19A — Esecuzione della riproduzione singola

**Data**: 2026-08-27
**Stadio**: `V26C_J19A_SINGLE_REPRODUCTION` — **esecuzione**
**Esecutore**: Opus 5, effort xhigh
**Esecuzione**: **una sola volta**, sotto GO architetto `d8b03042…`, exit code **0**, nessun retry.

## VERDETTO: `PASS` — un actor promosso

La riproduzione è **bit-identica**, non semplicemente entro tolleranza. Tutti e undici i criteri
vincolanti di eligibilità offline passano. **Nessun rollout è stato eseguito e J19B non è stata
avviata né preparata.**

---

## 1. Cosa è stato eseguito

GO verificato prima del lancio: SHA-256
`d8b0304247cb520595de69f4a4e1e7dd2037fe3607c331752e5d16e218a1bbf7`, `valid=true`, `problems=[]`,
**quattro pin** tutti allineati ai file su disco. `j19a_runs` **assente** prima del lancio.

Su tua istruzione **non ho eseguito la suite readiness** né prima né dopo: la sua `S47` («nessun file
GO su disco») è ora storicamente falsa per la presenza legittima del GO, esattamente come `E54` per
J18. Il runner ha eseguito il **proprio preflight vincolante**, 57 check, prima del fit.

Configurazione: candidato **13** — λ = 30.0, β = 1.0, lr = 5e-05, seed 123, 200 epoche, batch 128,
33 batch/epoca, **6600 step**, nessuna griglia, nessun early stopping, nessun retry.

**La traiettoria di training è coincisa con quella congelata di J18 fin dai valori stampati**:
epoca 1 `0.11884679439336537`, epoca 50 `0.013546582022616795`, epoca 100 `0.003533092456140303`,
epoca 150 `0.0018923459303896214`, epoca 200 `0.001473467772658344` — identici a J18.

---

## 2. Riproducibilità — `abs_delta = 0` su tutte e sei le metriche

Confronto `math.isclose(rel_tol=1e-06, abs_tol=1e-09)`, **tolleranza numerica e non gate
prestazionale**, come registrato.

| campo | fresh | frozen | abs delta | esito |
|---|---|---|---|---|
| `MSE_A` | 0.00055970096664322976 | 0.00055970096664322976 | **0.000e+00** | PASS |
| `MSE_B` | 0.00030691261600864986 | 0.00030691261600864986 | **0.000e+00** | PASS |
| `rmse_C` | 0.0030383822645046045 | 0.0030383822645046045 | **0.000e+00** | PASS |
| `rmse_D` | 0.0026441111267731902 | 0.0026441111267731902 | **0.000e+00** | PASS |
| `max_abs_C` | 0.046971050069934295 | 0.046971050069934295 | **0.000e+00** | PASS |
| `max_abs_D` | 0.019819501669915418 | 0.019819501669915418 | **0.000e+00** | PASS |

Campi esatti: `best_epoch = 191` PASS, `optimizer_steps = 6600` PASS, `batches_per_epoch = 33` PASS.

**Il margine di tolleranza non è stato usato.** Le sei metriche coincidono al bit, non entro 1e-06.
È il risultato più forte che questo controllo potesse dare: il fit J18 è esattamente riproducibile
sulla stessa piattaforma, dal suo runner pinnato.

### Comparazione esatta campo per campo

**36 campi confrontati, 0 falliti.** Coprono seed e protocollo, gli otto campi candidate/config, i
quattro conteggi di riga, le quattro coppie di hash observations/labels, byte e digest del parent J8
e del parent di J8, e sette campi di provenance.

La **distinzione storica** è presente sul campo `labels_sha256.D`: le sue etichette sono calcolate, e
il contratto **cross-platform** di J18 le vincolava con tolleranza 1e-06 anziché per hash; questa
riproduzione è **same-platform** e ha richiesto — e ottenuto — l'hash esatto.

---

## 3. Eligibilità offline — 11/11

| id | criterio | misurato | soglia | origine | esito |
|---|---|---|---|---|---|
| G5 | `on_policy_improvement_B` | 0.000306912616 | ≤ 0.09546555954 | preregistrato J18 v1 | PASS |
| G6 | `teacher_non_regression_A` | 0.0005597009666 | ≤ 0.004108013186 | preregistrato J18 v1 | PASS |
| **G3** | `preservation_bias_C` | 0.000388983597 | ≤ **0.0008992143952** | **bias empirico J8** | PASS (0.433×) |
| **G4** | `preservation_bias_D` | 0.000399679421 | ≤ **0.0007264156156** | **bias empirico J8** | PASS (0.550×) |
| **E-C** | `preservation_rmse_C` | 0.003038382265 | ≤ **0.004473968748** | **RMSE J8** | PASS (0.679×) |
| **E-D** | `preservation_rmse_D` | 0.002644111127 | ≤ **0.005262698259** | **RMSE J8** | PASS (0.502×) |
| G7 | log-std byte-identica e state-independent | esatto | — | strutturale | PASS |
| G8 | nessun critic | 0 | — | strutturale | PASS |
| G9 | colonne clock zero | 0 | — | strutturale | PASS |
| G10 | key set identico | esatto | — | strutturale | PASS |
| G11 | finitezza ricorsiva | esatto | — | strutturale | PASS |

**Zero findings non finiti**, quindi la promozione non è stata bloccata da quella via.

### Diagnostici, non vincolanti

| | valore | riferimento registrato |
|---|---|---|
| G1 max drift C | **0.046971** | 0.005, **non binding** |
| G2 max drift D | **0.019820** | 0.005, **non binding** |
| POST-485 | max **0.957933**, rmse 0.363305, n=485 | **non binding** |

Il candidato **eccede di 9.4×** il vecchio riferimento max-drift e ciò **non lo blocca più**: è
esattamente l'effetto della tua decisione architetturale, ed è qui visibile in atto.

**Il POST-485 merita la tua attenzione, con la sua avvertenza.** Sullo stesso insieme di stati:

| attore | closed-loop | max POST-485 |
|---|---|---|
| J8 | 5/6 | 0.01726 |
| J11 | 4/6 | 0.37754 |
| **J19A c13** | **non testato** | **0.957933** |
| J15R1 | 0/6 | 1.51946 |

Il candidato si colloca **fra J11 e J15R1**, più vicino al secondo. *Avvertenza dichiarata e non
attenuata*: quelle 485 righe sono la **regione bersaglio della correzione**, dove un attore corretto
**deve** cambiare. Un valore alto è atteso per costruzione e l'associazione con il fallimento
closed-loop è **confusa dal disegno**. Non è un criterio, non lo propongo come tale, e non ne traggo
alcuna previsione. Lo segnalo perché è l'unico numero della fase che si muove nella direzione degli
attori falliti.

---

## 4. L'actor committato

| | valore |
|---|---|
| `module_state.pkl` SHA-256 | `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530` |
| `actor_digest` | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| `source_actor_digest` (J8) | `6a879714044ba8321fedf8e554d0f2ec448c1f1177e1648e4e4aa72195031207` |
| label | `J19A_SINGLE_REPRODUCTION_OF_J18_C13` |
| status | `reproduced and offline-eligible; closed-loop A-F NOT run` |

- Il manifest dichiara il **proprio** hash e il **proprio** digest — verificato ricalcolandoli.
  Nessuna eredità del difetto del sidecar stantio di J8.
- Il digest **differisce** dal parent: è un actor nuovo, non una copia.
- Key set identico a J8, **10 chiavi**; **35D**; nessuna chiave critic.
- **Log-std byte-identica a J8** e state-independent, dopo 6600 step.
- Colonne clock **esattamente zero**; alias encoder **bit-identici**.

---

## 5. Integrità del leaf

Leaf: `j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/` — **sette file**.

| file | SHA-256 |
|---|---|
| `v26c_j19a_single_reproduction_receipt.json` | `235a117fc849bbe137dfd7ea29621390a6d1aa71aa9f9b4d95ca0e7a5dd50dad` |
| `v26c_j19a_result.json` | `8982b2fc40e514bed903af0c33e0a4ab3737ebf849a44a232912a816002dc254` |
| `history.json` | `0a00cb6be58945d525201f76aa425ad713addbd4946d108e02cf4bc2c4111af0` |
| `commit_verification.json` | `4d526dc119a10983e0186257132fbff58569d90ab0d1a793e0afae5ba2f32c61` |
| `rl_module/module_state.pkl` | `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530` |
| `rl_module/actor_feature_manifest.json` | `2c01067e9a569354cc4099537a3a556ab50d55aa8baa1d2127324dafeee27c54` |
| `rl_module/class_and_ctor_args.pkl` | `897e2f13695c52a411d49f957bdaf99ab864411334538703844f1b063857cd02` |

- **Verifica post-commit**: `ok=true`, `problems=[]`, **5 file** ri-risolti dal receipt committato,
  ri-hashati e confrontati sia col receipt sia coi byte in staging.
  `aggregate_digest = e019c144b9eadc33f68e8a007b3ade0612f8681b4f68214363f2a3c8a7284932`.
- **Marcatore born-invalid rimosso** solo dopo la verifica; **nessuna staging residua**.
- Il receipt **non contiene il proprio hash**; pinna i quattro pin del GO, il runner J18
  `d3949631…`, il parent J8 e il parent di J8, e registra `rollout_performed = false`.
- `class_and_ctor_args.pkl` è **byte-identico** a quello di J8 (`897e2f13…`), come deve essere: la
  classe e gli argomenti del costruttore non cambiano.

---

## 6. Limiti — cosa questo risultato NON dice

- **Nessuna qualifica closed-loop è stata eseguita.** Nessun rollout, nessun environment, nessun
  PPO. Il verdetto è **esclusivamente offline**.
- L'eligibilità offline dice che l'actor **non è peggiore dell'aggiornamento che ha prodotto J8** su
  bias e RMSE agli anchor, e che corregge il blocco on-policy. **Non dice che passerà A–F.**
- J17 ha misurato che l'offline non certifica il closed-loop; la diagnosi J18 ha misurato il verso
  opposto — J4, con il drift più basso di tutti, incorse comunque in una violazione diagnostica a
  25 mm. **I gate offline filtrano, non certificano.**
- Il POST-485 a 0.957933 è il dato che più invita alla cautela, con la sua confusione di disegno
  dichiarata in §3.
- **J8 resta l'attore operativo, 5/6**, invariato e non toccato.

---

## 7. Invarianti verificate

- Eseguito **una sola volta**, exit code 0, **nessun retry**.
- **Un solo actor** persistito, in **un solo** leaf, **solo dopo** riproducibilità ed eligibilità.
- **Nessun rollout, environment, critic, PPO, collection.** `j19b_runs`: assente. **J19B non avviata
  e nemmeno preparata.**
- **Non modificati**: `v26c_j16_closed_loop.py` (`6ac45854…`), `v26c_j18_b_only_update.py`
  (`d3949631…`), `test_v26c_j18_b_only_update.py` (`6050f4c0…`), il leaf J18, J8, J2, J4, J7, J9R1,
  J10R1, produzione, FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura.
- Semi 126, 127, 128: non letti, non generati, non usati.
- Worktree sporco preservato. Nessun subagente usato in questa fase.

---

## 8. TODO propagati

- **J19B non preregistrata, non implementata, non avviata**: attende il tuo audit di questo leaf.
- Il POST-485 del candidato (0.957933) va valutato da te prima di J19B, con la confusione di disegno
  dichiarata.
- La suite readiness J19A resterà a 126/127 per `S47`, invariante scaduta dopo l'emissione del GO,
  come `E54` per J18: **non modificata**.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- `policy_std` sempre `null`, ereditato da J12.
- `best_validation_mse` contaminato in entrambi i fit dalla ripetizione dei blocchi.
- La deviazione pre-breccia non è rilevabile su singola run: serve un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 9. STOP

Esecuzione singola completata, riproduzione bit-identica, eligibilità 11/11, un actor promosso,
verifica post-commit superata.

**J19B non avviata e non preparata. Fermo in attesa del tuo audit.**
