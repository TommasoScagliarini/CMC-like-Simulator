# V26C J18 — Addendum v2: preparazione dell'esecuzione

**Data**: 2026-08-27
**Stadio**: `V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE` — **preparazione esecuzione**
**Versione**: 2, **additiva**. La preregistrazione v1 non è riscritta né modificata.
**Esecutore**: Opus 5, effort xhigh

**Nessun fit, nessun rollout, nessun environment, nessun critic, nessun PPO, nessuna collection,
nessun candidato actor, nessuna `j18_runs`, nessun seme sigillato, nessun GO valido creato.**

**STOP prima del fit. Fermo per audit.**

Nota di ambiente: **`apply_patch` non esiste in questo ambiente**. Ho usato gli strumenti di editing
patch-based disponibili, che rispettano lo stesso vincolo: nessun `cat`, nessuna redirezione,
nessun trucco di shell.

---

## 1. Problema

La v1 registrava l'obiettivo `sum_b w_b · MSE_b` e il protocollo, ma lasciava non dette **due
semantiche** che il percorso di esecuzione non può evitare di decidere. Deciderle dentro il codice,
in silenzio, le avrebbe rese non auditabili. Sono chiuse qui, additivamente e prima dell'esecuzione.

---

## 2. Lacuna 1 — semantica dei minibatch

L'obiettivo registrato è **per blocco**: ogni blocco è mediato dentro sé stesso e poi pesato. Un
minibatch però non vede blocchi, vede righe. Una media per batch sulle righe ripesa ogni blocco in
base a **quante sue righe sono capitate nel batch**: l'obiettivo ottimizzato non sarebbe quello
registrato. Con il blocco B a 14 righe su 4221 questo non è un dettaglio.

### Stimatore registrato

```
L_M = (N / |M|) · Σ_{i ∈ M} [ w_block(i) / N_block(i) · e_i ]
```

con `N = 4221`, `e_i` errore quadratico alla riga *i* mediato sulle **due** azioni, e
`coef_i = w_block(i)/N_block(i)` ricostruito per candidato (perché `w_B = β` e `w_C = w_D = λ`
vengono dalla griglia).

**Identità su un'epoca**: sommando le perdite dei chunk pesate per `|M|/N` si ottiene
`Σ_i coef_i·e_i = Σ_b w_b·MSE_b`, cioè **esattamente** l'obiettivo v1.
**Non distorto**: per `M` uniforme, `E[L_M] = (N/|M|)·(|M|/N)·Σ_i coef_i·e_i = Σ_b w_b·MSE_b`. È uno
stimatore corretto, non un'approssimazione.

Verificato su un dataset sintetico **deliberatamente sbilanciato** (3/1/5/2): la somma pesata dei
chunk coincide con l'obiettivo registrato entro 1e-12, mentre **la media ingenua per batch dà un
risultato diverso** — il test morde.

### RNG e ordine

- `numpy.random.default_rng(123)`, **ricostruito da zero per ciascun candidato**: tutti e 16
  percorrono la stessa sequenza di permutazioni; le differenze vengono solo dalla griglia.
- una `rng.permutation(4221)` per epoca; **una visita per riga per epoca**.
- chunk consecutivi di 128, **incluso il chunk finale corto da 125**, che è tenuto e non scartato:
  `32·128 + 125 = 4221`.
- **33 batch/epoca × 200 epoche = 6600 step per candidato**; 16 candidati → 105 600 step.
- nessun early stopping, nessun retry.
- fresco per candidato: byte del parent ricaricati, seed torch, seed numpy legacy, Adam e i suoi
  buffer di momento, generatore di permutazioni.

Selezione del checkpoint invariata: obiettivo composito sui blocchi **pieni** dopo ogni epoca,
pareggi all'epoca più precoce. Il dominio di valutazione (quello scalato, lo stesso della loss) è
scritto nero su bianco; metriche finali e gate sono valutati sull'**actor raw**.

---

## 3. Lacuna 2 — scaling con un parent a controller vivo

La convenzione July addestra su `obs/scales` e assorbe le scale in `pi.0.0.weight` al salvataggio.
J11 e J15R1 inizializzavano dai pesi **raw** del parent e alimentavano input scalati. **Questo
preserva la funzione solo se i pesi del parent sulle colonne scalate sono zero** — vero per il loro
parent J2, **falso per J8**.

| colonna | feature | scala | max\|W\| nel parent J8 |
|---|---|---|---|
| 27 | `pros_knee_angle_served_ref_vel` | 4.0 | 2.065141e-03 |
| 28 | `pros_knee_angle_served_ref_accel` | 60.0 | 3.697728e-04 |
| 32 | `pros_ankle_angle_served_ref_vel` | 3.5 | 3.183345e-03 |
| 33 | `pros_ankle_angle_served_ref_accel` | 55.0 | 3.810721e-04 |

| approccio | spostamento dell'uscita prima di qualunque gradiente |
|---|---|
| ingenuo (J11/J15R1) | **1.077083e-02** — circa **74× il budget G1** di 0.005 |
| corretto (`W_scaled = W_raw · s`) | 5.960464e-08 numpy, 1.192093e-07 torch (= 2⁻²³) |

Il candidato sarebbe partito da un actor **diverso** da J8, silenziosamente, allo step 0.

**Contratto vincolante**: prima dell'optimizer, i parametri in dominio scalato devono riprodurre
l'uscita raw del parent sulle 4221 righe entro **1e-06**, in **entrambi** i kernel numpy e torch;
altrimenti il run aborta. Il round-trip dei pesi `(W·s)/s == W` è **bit-identico** (misurato 0.0) ed
è asserito a run time, non assunto: è una proprietà di questi valori, non un teorema.

**La bit-identità dell'uscita non è dichiarata**, perché sarebbe falsa: `(W·s)@(o/s)` e `W@o` sono
sequenze diverse di operazioni float32. J11 e J15R1 potevano dichiararla solo perché la loro
moltiplicazione e divisione erano no-op su pesi nulli.

---

## 4. Un difetto trovato dal test double, non da lettura

Avevo asserito che l'assorbimento delle scale fosse un **round-trip bit-identico dei pesi**. È
sbagliato in due modi, e il test double lo ha fatto emergere subito.

1. **Direzione sbagliata.** Ciò che avevo misurato esatto su J8 è `(W·s)/s == W`. Dopo
   l'addestramento serve `(W/s)·s == W`, che per pesi float32 arbitrari non vale. L'assert avrebbe
   **abortito un run sano**.
2. **Confronto fra kernel diversi.** Il secondo tentativo confrontava il forward torch float32 con
   il forward numpy float64 e riportava **9.259e-04**, tre ordini di grandezza sopra la tolleranza.
   Decomposto:

   | confronto | max\|Δ\| |
   |---|---|
   | assorbimento vero, float64 contro float64 | **3.205e-10** |
   | gap kernel float64/float32, **raw contro raw** | 7.828e-07 |
   | float32 numpy contro torch | 8.941e-08 |

   Il gap fra kernel è presente **anche senza alcuna scala** e non ha nulla a che vedere con
   l'assorbimento.

Corretto: il contratto è misurato **like-for-like**, entrambi i lati nel kernel float64, ed è
l'assorbimento a essere sotto gate; il gap fra kernel è registrato come diagnostica e **non
governa nulla**. Tre test bloccano la regressione, incluso uno che verifica che il gap fra kernel
sia almeno 100× l'errore di assorbimento — cioè che i due non tornino a essere confusi.

---

## 5. Architettura dell'esecuzione

- **16 candidati** esattamente quelli preregistrati, ciascuno inizializzato **indipendentemente dai
  byte incontaminati di J8**, con Adam fresco e RNG fresco. Solo seed 123.
- **Log-std** = righe **2:4** di `pi.1.weight`/`pi.1.bias`, proiettate dopo **ogni** step e al
  ripristino e al salvataggio; asserite byte-identiche a J8 **e** state-independent.
- **Colonne clock 0:2** esattamente zero dopo ogni step e al salvataggio.
- **Alias encoder** ricostruiti bit-identici; stesso key set e stesse shape di J8; 35D; nessun critic.
- **Persistenza**: metriche ed esiti di **tutti** i gate G1–G11 per **tutti e 16** i candidati; **al
  più UN actor**, il primo in classifica fra i sopravvissuti. **Sedici actor non vengono mai
  scritti.** Con zero sopravvissuti si fallisce chiuso, **senza alcun actor**.
- **Leaf** content-addressed, no-clobber, **born-invalid**: il marcatore `TECHNICAL_INVALID` è la
  **prima** scrittura in staging e la sua rimozione è l'**ultima**, dopo la verifica post-commit che
  ri-risolve ogni path dal receipt **committato**, ri-calcola gli hash e li confronta sia col receipt
  sia coi byte in staging. Su fallimento il marcatore **resta**.
- **GO architetto**: deve nominare lo stadio, autorizzare esplicitamente l'esecuzione e pinnare
  l'hash esatto di **tutti e sei** gli artefatti. Pin mancante, hash stantio, scope allargato o
  stadio sbagliato sono **rifiuti**, mai avvisi.

**Non ho creato alcun GO valido.** La validazione è testata con payload **in memoria**, quindi
nessun file GO valido esiste su disco — verificato da un test.

Il digest dell'actor riusa la funzione canonica del progetto (`actor_state_digest`, trascrizione di
`warm_start`), che riproduce J8 = `6a879714…` e J2 = `59d54240…`. Non ne ho inventata una nuova.

---

## 6. Test

**139/139 PASS.** Preflight invariato a **42/42**.

| gruppo | cosa morde |
|---|---|
| E01–E05 | formula esatta su dataset sbilanciato; la media ingenua dà un valore diverso; chunk singolo scalato per `N/\|M\|`; coefficienti `w_b/N_b` |
| E06–E10 | due stream `default_rng(123)` identici; init deterministico; il parent non viene mutato |
| E11–E17 | 33 batch/epoca, 6600 step, 16×6600 = 105 600; copertura esatta di ogni riga; chunk finale corto tenuto; guardia AST sul contratto di step |
| E18–E24 | test double a 2 epoche sul percorso reale: conteggio step, key set, log-std byte-identica, state-independence, clock a zero, alias, e che abbia davvero addestrato |
| E25–E30c | quattro colonne scalate; parent a controller vivo; round-trip bit-identico; equivalenza entro tolleranza; la via ingenua supera G1; assorbimento like-for-like distinto dal gap fra kernel |
| E31–E38 | tutti e 16 registrati; un solo actor; classifica corretta; fail-closed senza actor; tally dei gate; serializzazione senza tensori |
| E39–E45 | leaf in directory temporanea: verifica ok, marcatore rimosso solo dopo, no-clobber, mismatch catturato, marcatore **mantenuto** sul fallimento |
| E46–E54 | GO: sei pin obbligatori, stadio errato, autorizzazione assente, pin mancante, hash stantio, scope allargato, file assente, nessun GO su disco |
| E55–E58 | nessun import di environment/RL/simulatore; import esattamente l'insieme atteso; torch solo lazy |
| T24e–T24g | G7 fallisce su log-std perturbata **e** su log-std resa state-dependent; G9 fallisce su clock non nullo **a qualunque magnitudine** |

---

## 7. Artefatti

| artefatto | SHA-256 |
|---|---|
| `v26c_j18_prereg_b_only_constrained_update.json` (v1, **invariato**) | `f19de7b5c4fa1c4c6b2101e34013576d88a838032c5c490a788e784504a96720` |
| `v26c_j18_prereg_addendum_v2_minibatch_and_scaling_2026-08-27.json` (**nuovo**) | `bc50072835929961f85aa5ce955124491ca3c9e16d87ec2691b29ab7b996a71d` |
| `v26c_j18_dataset_manifest.json` (**invariato**) | `ba9ebf9c77550595562d3047368483f8fab7d4c46a0032c177ffd278a07910d2` |
| `v26c_j18_provenance_overlay_j8_2026-08-27.json` (**invariato**) | `dc4fd169ec95e4bcec1d472ab94f176b980f059ffdd98e28d009e83db0db9128` |
| `v26c_j18_b_only_update.py` (**esteso**) | `50480b24b32ecedd95d9c138f6b72d994a17e2a6a976054045a7126612d42b73` |
| `test_v26c_j18_b_only_update.py` (**esteso**) | `109c4b3a7ab08c5705d49c21eb39b36df9e24b9e2be6da5cc0a59cb31badcfea` |

Questi sei sono esattamente i pin che un GO valido dovrà portare. Nessun JSON contiene il proprio
hash.

---

## 8. Invarianti verificate

- `j18_runs`: **assente**. Nessuna directory di staging. Nessun file GO.
- Nessun candidato actor creato. **Zero** fit, rollout, environment, critic, PPO, collection.
- **Nessuna mutazione** di J8, J2, J7, J9R1, J10R1: hash degli input invariati.
- Semi 126, 127, 128: non letti, non generati, non usati; verificato via AST con allowlist di due
  sole costanti, essa stessa asserita.
- Non modificati: FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione.
- Worktree sporco dell'utente **preservato**.
- **Nessun subagente usato.**

---

## 9. TODO propagati

- **J18 non eseguito**: serve un GO che pinni i sei hash sopra.
- I gate offline **filtrano, non certificano**: il verdetto vincolante resta la riqualifica
  closed-loop A–F, che questa fase non esegue né pre-autorizza.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- `policy_std` sempre `null`, ereditato da J12.
- Nessuna leaf pinna il runner che l'ha scritta.
- Il fit J11 non è bit-riproducibile dai propri artefatti; J15R1 lo è.
- `best_validation_mse` contaminato in entrambi i fit dalla ripetizione dei blocchi.
- La deviazione pre-breccia non è rilevabile su singola run: serve un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 10. STOP

Percorso di esecuzione implementato e testato. **Nessun fit avviato, nessun GO valido creato.**

**Fermo in attesa del tuo audit.**
