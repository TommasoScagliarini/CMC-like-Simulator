# V26C J18 — Preregistrazione dell'update B-only vincolato su J8

**Data**: 2026-08-27
**Stadio**: `V26C_J18_J8_B_ONLY_CONSTRAINED_UPDATE` — **readiness / preregistrazione**
**Esecutore**: Opus 5, effort xhigh
**Stato**: nessun fit, nessun rollout, nessun environment, nessun critic, nessun PPO, nessuna
collection, nessun candidato actor, **nessuna `j18_runs`**, nessun seme sigillato.

**Fermo per audit Codex. Non chiedo un GO utente.**

---

## 1. Perimetro

Preparazione soltanto. Ho misurato, preregistrato e testato; non ho addestrato nulla.

- Lineage operativa: **August V26 imitation → J2 35D → J8 → questo update incrementale B-only**.
  Luglio resta fonte metodologica e non entra in lineage.
- **J8 è parent immutabile.** Nessun artefatto J8 è stato modificato.
- Un solo actor 35D. Nessun 25D separato, nessun widening, nessuna feature controlaterale.
- Intera mean-network addestrabile; log-std congelata byte-identica; critic escluso.
- Semi 126–128 e fase G–I: non letti, non generati, non usati.
- Non toccati: FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione.
- Worktree sporco preservato. Editing patch-based; nessun `cat`, nessuna redirezione.

---

## 2. Il blocker di provenienza J8 — risolto

### Il rilievo

`j8_runs/.../rl_module/actor_feature_manifest.json` è **byte-identico a quello di J2**
(`0c88018d…`). Descrive quindi J2: dichiara `module_state_sha256 = 0f182ea9…` (J2), porta
`actor_label = "J2_BASE35_JULY_FAITHFUL"`, uno `status` di fase base e un `derived_from` che punta
oltre il parent di J8; non ha alcuna chiave `actor_digest`. La leaf J8 inoltre non ha
`commit_verification.json` — stessa convenzione della leaf J2, non una novità di J8.

Per direttiva: **nessun artefatto J8 modificato**, e il rilievo non è trattato come blocker
scientifico. È un difetto di *reporting* di provenienza in un solo sidecar.

### La domanda che doveva essere sciolta prima di chiudere

I rollout A–F che J8 supera 5/6 sono stati guidati dal modulo J8 reale `9c5b1571…` o da J2
`0f182ea9…`?

### Risposta: dal modulo J8 reale. Due evidenze indipendenti, concordi.

**Documentale.** Il receipt J9R1 (`c201b666…`) **distingue già** i due concetti in campi separati:

| campo | valore |
|---|---|
| `actor_before.artefacts_sha256["rl_module/module_state.pkl"]` | `9c5b1571…` ← **actor usato** |
| `amendment.pinned_artefacts_sha256[…/module_state.pkl]` | `9c5b1571…` ← stesso pin, indipendente |
| `actor_before.lineage.parent_module_state_sha256` | `0f182ea9…` ← **parent**, campo distinto |

Il receipt non ha mai confuso actor e parent. Solo il sidecar lo fa. Corrobora il campo
`mask_contract.note`: *"the mask FLIP relative to J1/J2/J3, where 25:35 were zero"*, con dieci
`controller_norms` non nulli sulle colonne 25–34 — descrizione vera di J8 e falsa di J2.

**Empirica, e discriminante.** Ogni step di trace registra `actor_observation_vector_before` e
`policy_mean`. Ho rigiocato le osservazioni registrate attraverso tre moduli candidati.

Discriminante strutturale: `max|W[:,25:35]|` del primo layer vale **0.0 in J2**, `1.2939e-02` in J8,
`4.58e-04` in J4. J2 ha il blocco controller *esattamente morto*: è distinguibile dal solo
comportamento.

| cella | n | `max|J8 − registrato|` | `max|J2 − registrato|` | `max|J4 − registrato|` |
|---|---|---|---|---|
| A | 500 | **1.435e-07** | 1.533e-02 | 1.483e-02 |
| B | 500 | **1.470e-07** | 1.957e-02 | 1.920e-02 |
| C | 500 | **1.240e-07** | 1.735e-02 | 1.685e-02 |
| D | 500 | **1.337e-07** | 1.494e-02 | 1.466e-02 |
| E | 500 | **1.363e-07** | 1.599e-02 | 1.553e-02 |
| F | 500 | **1.140e-07** | 1.553e-02 | 1.554e-02 |

J8 riproduce tutte e 3000 le coppie entro **1.470e-07**, cioè il rumore di round-trip float32. J2
sbaglia fino a 1.957e-02, **133 119×** il residuo di J8 e quattro ordini di grandezza sopra il
rumore.

**Verdetto: DIMOSTRATO.** L'ipotesi «i rollout A–F usarono J2» è **falsificata su tutte e sei le
celle**. È una misura che discrimina fra alternative nominate, non una correlazione. Nessun blocker.

### Conseguenze operative registrate

J18 vincola il parent ai **byte reali** di `module_state.pkl`, mai al sidecar; ricalcola il digest
(`6a879714…`); e qualunque manifest scritto per un candidato J18 dovrà essere veritiero sul proprio
modulo, come già fanno J11 e J15R1.

---

## 3. Composizione dei blocchi — decisione dell'architetto

Il requisito iniziale era **contabilizzare** gli overlap. Sulla base delle misure l'architetto ha
deciso la **disgiunzione globale con priorità semantica A > B > C > D**. La mia proposta precedente
(B > A, per scarsità) è **superata**; implemento l'ordine deciso.

### Matrice di overlap, sui blocchi grezzi

Base: vettori osservazione da 35 float32, identici bit a bit.

```
          A      B      C      D        righe  distinte  dup.interni
   A    500      1      0      0          500       500            0
   B      1     15      0      0           15        15            0
   C      0      0   2497      0         2500      2497            3
   D      0      0      0   1210         1210      1210            0
```
diagonale = osservazioni distinte nel blocco; fuori diagonale = intersezione.

Due sospetti verificati, **uno confermato e uno smentito**:

- **confermato** — `A ∩ B = 1`: al passo 0 lo stato è il reset, e teacher e on-policy partono dallo
  stesso reset.
- **smentito** — il supporto nominale J7 unico **non** coincide con gli stati della cella A:
  intersezione **0**. C e D sono genuinamente disgiunti.
- i 3 duplicati interni a C sono il passo 0 delle celle A, D, E, F, che condividono
  `offset_s = 1.956870983805102`. La cella C ha offset +0.20 s e non collide.

### Conflitti di target: misurati, e sono ZERO

| sovrapposizione | righe | target | `max|Δ|` | righe in conflitto |
|---|---|---|---|---|
| `A ∩ B` | 1 | `[0.20619573, −0.00133646]` in entrambi | **0.000e+00** | 0 |
| duplicati interni a C | 4 | `[0.74620132, −0.00803327]` in tutti | **0.000e+00** | 0 |

In `A ∩ B` entrambi i blocchi etichettano l'indice 0 con l'azione J10 all'indice 0: è **lo stesso
esempio**, non uno stato con due target. Nei duplicati di C il target è l'output deterministico di
J8 sullo stesso stato.

**Conseguenza**: la regola «prevale la priorità più alta» **non scarta alcuna informazione di
target**. Cambia solo il *peso*: lo stato di reset condiviso passa da 2× (A e B) a 1× in A, e il
reset nominale da 4× a 1× dentro C.

### Conteggi prima e dopo

| blocco | ruolo | prima | dopo | rimosse |
|---|---|---|---|---|
| A | target principale — teacher J10-B completo | 500 | **500** | 0 |
| B | target on-policy — prefisso J8 pre-mismatch | 15 | **14** | 1 |
| C | preservation anchors — A,C,D,E,F | 2500 | **2497** | 3 |
| D | preservation support — J7 unique | 1210 | **1210** | 0 |
| | **totale** | 4225 | **4221** | 4 |

C per cella dopo: A 500, C 500, D 499, E 499, F 499.
Verificato: 4221 righe, **4221 osservazioni distinte** nell'unione → disgiunzione dimostrata, non
assunta.

### Prefisso on-policy, derivato e non hardcodato

Regola: il prefisso è la massima corsa contigua iniziale in cui le osservazioni J9-B e J10-B
concordano su **ogni** feature discreta. Operatore riusato immutato da J14.

- indici discreti `[11,12,13,17,18,19,20,21]`
- **primo mismatch all'indice 15** (step 16), colonna **11 = `online_left_in_contact`**, J9 = 0.0
  contro J10 = 1.0
- prefisso 15, contiguo; **485 righe post-mismatch escluse**
- tempi J9-B e J10-B **bit-identici**, `max|Δt| = 0.000e+00`

Onestà sul blocco: dentro il prefisso le feature **continue già divergono**
(`pros_knee_angle_served_ref_accel` 50.35, 3.638 σ). È una regione di accordo di *fase discreta*,
**non** di identità di stato.

---

## 4. Misure baseline — da cui derivano tutte le soglie

Con candidato ≡ J8, prima di qualunque update:

| blocco | n | MSE | RMSE | `max|drift|` |
|---|---|---|---|---|
| A | 500 | 0.004108013186414973 | 0.064094 | — |
| B | 14 | **0.1909311190718203** | 0.436957 | — |
| C | 2497 | 1.29e-15 | ~0 | 1.435e-07 |
| D | 1210 | 1.32e-16 | ~0 | 2.971e-08 |

C e D non sono esattamente 0 perché le etichette di C sono i `policy_mean` **registrati in float32**
mentre la valutazione gira in float64: è rumore di quantizzazione, non drift.

**Il blocco on-policy parte 46.48× peggio del blocco teacher.** L'errore di J8 sugli stati che
visita davvero è molto maggiore dell'errore sugli stati visitati dal teacher: è esattamente la
ragione per cui questa fase esiste.

Per azione, normalizzato e fisico (scala 0.35 rad):

| blocco | azione | RMSE | MAE | max | shift | RMSE rad |
|---|---|---|---|---|---|---|
| A | knee | 0.084307 | 0.030861 | 0.575914 | −0.020973 | 0.029507 |
| A | ankle | 0.033292 | 0.018273 | 0.221774 | +0.004254 | 0.011652 |
| B | knee | 0.601829 | 0.489729 | 1.006425 | **−0.483844** | 0.210640 |
| B | ankle | 0.140229 | 0.134169 | 0.221883 | +0.134169 | 0.049080 |

### Le due scale misurate che ancorano le soglie

**Perturbazione tollerata.** Massimo disturbo additivo realmente applicato nelle celle stocastiche
che J8 **ha superato**: D 0.016491, E 0.014557, F 0.016350. Il minimo è **0.014557** (cella E),
0.005095 rad. J8 ha completato e superato quelle celle con quel disturbo attivo a ogni passo.

*Limite dichiarato di questa evidenza*: quel disturbo era a media nulla e variabile passo per
passo, mentre un drift di pesi è sistematico e persistente. Non sono equivalenti, quindi lo uso solo
per fissare un **ordine di grandezza**.

**Variazione naturale del comando.** Variazione per passo dell'azione di J8 su tutte e sei le celle:
knee p50 0.011918, p95 0.043163; ankle p50 0.007493, p95 0.038970.

---

## 5. Griglia finita preregistrata

`L(θ) = w_A·MSE_A + w_B·MSE_B + w_C·MSE_C + w_D·MSE_D`, ogni MSE **mediato dentro il proprio
blocco** e poi pesato esplicitamente: nessun conteggio righe di un blocco filtra nell'influenza di
un altro.

La correzione è in **output/action space** con preservation funzionale. C e D sono anchor
*di uscita*, non una penalità nei parametri: è la risposta strutturale alla misura J17 secondo cui
J15R1 aveva solo un anchor in spazio parametri a peso 0.01 e il suo blocco nominale era
auto-distillato dal parent (errore quadratico all'inizializzazione 5.63e-10, quindi **gradiente
nullo al passo 0**).

| asse | valori | applica a |
|---|---|---|
| `preservation_weight_lambda` | 1.0, 3.0, 10.0, 30.0 | `w_C`, `w_D` |
| `on_policy_weight_beta` | 1.0, 5.0 | `w_B` |
| `learning_rate` | 1e-05, 5e-05 | — |

`w_A` fisso a 1.0. Enumerazione: cicli annidati nell'ordine dichiarato, esterno per primo →
`candidate_index` 0…15. **16 candidati, indice unico.**

Fisso fuori griglia: seed **123**, 200 epoche, batch 128, Adam, 6600 update massimi, shuffling
deterministico dal solo seed, log-std congelata per proiezione, colonne clock 0–1 azzerate dopo ogni
step e al salvataggio, alias `pi_encoder.*` ricostruiti bit-identici, scale fisiche assorbite nel
layer 1 al salvataggio.

**Nessun retry adattivo**: nessun candidato viene mai rilanciato, riseminato o ritoccato in risposta
al proprio risultato.

**Nessuno split di validazione**, e lo dichiaro invece di nasconderlo: C e D sono anchor, non un
target di generalizzazione, e B ha 14 righe e non è divisibile. L'obiettivo composito è valutato sui
blocchi **pieni** a ogni epoca e si tiene l'epoca a obiettivo minimo, pareggi risolti sull'epoca più
precoce. Significa che la scelta del checkpoint **non è una stima out-of-sample**.

---

## 6. Soglie derivate — hard gate contro ranking

Ogni soglia numerica è derivata da una quantità **misurata**. Nel runner le due principali sono
**espressioni**, non letterali, così una costante arrotondata non può allontanarsi dalla misura che
cita.

### Hard gate (devono passare tutti)

| id | cosa | soglia | derivazione | valore al parent |
|---|---|---|---|---|
| G1 | `max|drift|` su C | 0.005 | 2.91× sotto 0.014557, la più piccola perturbazione superata; e sotto la variazione mediana per passo di entrambe le azioni | 1.435e-07 |
| G2 | `max|drift|` su D | 0.005 | come G1 | 2.971e-08 |
| G3 | bias (shift medio con segno) per azione su C | 0.0014557 | **un decimo** di 0.014557: la perturbazione superata era a **media nulla**, quindi non copre un bias della stessa taglia, che va tenuto un ordine di grandezza più stretto | <1.5e-07 |
| G4 | bias su D | 0.0014557 | come G3 | ~0 |
| G5 | `MSE_B` | **0.09546555953591015** | esattamente metà della baseline misurata | 0.1909311190718203 |
| G6 | `MSE_A` | **0.004108013186414973** | la baseline stessa: A non deve migliorare, ma non deve peggiorare | idem |
| G7 | log-std congelata e state-independent | uguaglianza esatta | invariante strutturale | — |
| G8 | nessuna chiave critic | 0 | invariante | 0 |
| G9 | colonne clock esattamente zero | 0 | invariante | 0 |
| G10 | key set identico a J8, shape incluse | uguaglianza | invariante: 35D, un solo actor | — |
| G11 | finitezza | né NaN né Inf | invariante | — |

Il gate di bias separato non è decorativo: J17 ha misurato che il fallimento J16 comportava un
ginocchio **sistematicamente sotto-comandato** prima della breccia. Un limite sul caso peggiore non
esclude un bias piccolo e persistente, che è il modo di fallire già osservato.

### Ranking, solo fra i candidati che passano tutti i gate

Ordine **totale e deterministico**: 1) `MSE_B` minimo — 2) `MSE_A` minimo — 3) `max|drift|` su C∪D
minimo — 4) `candidate_index` minimo. L'ultima chiave è unica per costruzione, quindi nessun
pareggio resta irrisolto. Viene selezionato **esattamente un** candidato.

### Fail-closed

Se **nessun** candidato passa tutti gli hard gate, la fase **fallisce**: nessuna promozione, nessuna
soglia rilassata, nessun valore aggiunto agli assi, nessun rilancio. Si riporta il fallimento con le
metriche misurate di tutti e sedici i candidati e si torna all'architetto.

---

## 7. Runner, preflight, test

`v26c_j18_b_only_update.py` — `--preflight-only` e `--dry-run` ricostruiscono il dataset, lo
verificano contro il manifest pinnato e **non addestrano, non scrivono, non creano directory**.
`torch` è importato pigramente dentro il fit, quindi preflight e test non lo caricano mai.
`--execute` richiede `--go-file` e `run_fit` **rifiuta di partire**: l'esecuzione non è autorizzata
in readiness.

**Preflight: 42/42 PASS.** Verifica gli hash di tutti e nove gli input, i byte reali del parent, che
il sidecar sia stantio *come documentato*, che il prefisso sia derivato, i conteggi per blocco, la
disgiunzione, i conflitti di target a zero, gli hash di contenuto di ogni blocco, la baseline e la
finitezza della griglia.

**Suite di test: 76/76 PASS.**

---

## 8. Tre difetti che i test hanno morso prima dell'esecuzione

Li dichiaro perché sono miei e perché due erano invisibili a lettura.

**D1 — il gate G7 sarebbe stato vacuo.** Avevo scritto `logstd_keys()` cercando chiavi contenenti
`logstd`/`log_std`. **Quelle chiavi non esistono.** L'actor ha 10 chiavi e la testa emette **quattro**
valori: `pi.1.weight` ha shape (4, 256) e `pi.1.bias` shape (4). Le righe 0:2 sono le medie, le
righe **2:4 sono la log-std**. La ricerca per nome restituiva l'insieme vuoto, e un'uguaglianza su
insieme vuoto è **vera per vacuità**: il gate «log-std byte-identica» sarebbe passato sempre, anche
su un candidato che l'avesse stravolta. Corretto: il congelamento e il gate sono ora una **slice di
righe**; ho aggiunto il requisito che le righe log-std del peso restino esattamente zero (log-std
indipendente dallo stato), e un test che **perturba la slice e verifica che G7 fallisca** — un gate
che non può fallire non è un gate.

Questo cambia anche la descrizione di cosa si addestra: la mean-network è
`pi.0.0.*`, `pi.0.2.*` e le **righe 0:2** della testa.

**D2 — soglia arrotondata che divergeva dalla misura.** Avevo scritto `0.0954655596` dichiarandola
«metà della baseline», ma metà di `0.1909311190718203` è `0.09546555953591015`. Ora il runner
**deriva** la soglia (`BASELINE_MSE_B / 2.0`) e un test AST verifica che sia un'espressione e non un
letterale.

**D3 — il mio test sui semi sigillati era impreciso.** Segnalava `128` ovunque, colpendo
`FIT_BATCH_SIZE = 128`, che è una batch size e non un seme. Sostituito con una allowlist di due sole
dichiarazioni nominate, essa stessa asserita, così una nuova occorrenza altrove morde comunque.

---

## 9. Cosa questa fase può e non può stabilire

**Può**: che un candidato migliori il blocco on-policy almeno del 50% mentre la sua uscita resta
dentro un budget di drift misurato su 3707 stati di preservation.

**Non può**: che il candidato passi il closed loop. J17 lo ha misurato direttamente — J15R1
migliorava offline sulle righe correttive e poi ha fatto **0/6**, perché un'osservazione
bit-identica al passo 1 divergeva in una traiettoria diversa. **I gate offline fanno da filtro, non
da certificato.**

Il verdetto vincolante su qualunque candidato prodotto qui sarà una riqualifica closed-loop A–F
successiva, che questa fase non esegue e non pre-autorizza.

---

## 10. Artefatti prodotti

| artefatto | SHA-256 |
|---|---|
| `v26c_j18_prereg_b_only_constrained_update.json` | `f19de7b5c4fa1c4c6b2101e34013576d88a838032c5c490a788e784504a96720` |
| `v26c_j18_dataset_manifest.json` | `ba9ebf9c77550595562d3047368483f8fab7d4c46a0032c177ffd278a07910d2` |
| `v26c_j18_provenance_overlay_j8_2026-08-27.json` | `dc4fd169ec95e4bcec1d472ab94f176b980f059ffdd98e28d009e83db0db9128` |
| `v26c_j18_b_only_update.py` | `77523bec5bc0a2a60cf82c25aa7c9cffe2ed376b65080d3775f53e5d9b8eb3c2` |
| `test_v26c_j18_b_only_update.py` | `61bf15668325cf6a743d5bb726e4b6015b94bda17e3590ed4efe127ef2f47bf6` |

Nessun JSON contiene il proprio hash; è verificato da test.

Input pinnati e verificati byte-identici: le sei trace J9R1, il receipt J9R1, il teacher J10R1-B, il
dataset J7, e i sei file della leaf J8.

---

## 11. Invarianti verificate

- `j18_runs`, `g_i_runs`, `ppo_runs`, `critic_runs`: **assenti**.
- Nessun candidato actor creato. Nessun lock, nessuno staging.
- **Nessun artefatto J8 modificato**, né alcun artefatto J2/J4/J7/J9R1/J10R1.
- **Zero** fit, rollout, environment, critic, PPO, collection.
- Semi 126, 127, 128: non letti, non generati, non usati; verificato via AST sul runner.
- Non modificati: FSM v3, detector/morphology, reward, sigma, SEA/C++, architettura, produzione.
- Worktree sporco dell'utente **preservato**.
- Editing patch-based; nessun subagente usato.

---

## 12. TODO propagati

- **J18 non eseguito**: la griglia richiede un GO esplicito dell'architetto.
- **Deviazione dichiarata**: la disgiunzione rimuove 4 righe su 4225 (0.095%). B tiene 14 righe e C
  ne tiene 2497. È la conseguenza diretta della priorità decisa; la segnalo perché sia sull'atto.
- Il sidecar `actor_feature_manifest.json` di J8 **resta stantio per decisione architetturale**:
  chi lo consuma deve leggere l'overlay di provenienza.
- La leaf J8 **non ha `commit_verification.json`**, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- `policy_std` sempre `null` — difetto cosmetico ereditato da J12.
- Nessuna leaf pinna il runner che l'ha scritta.
- Il fit J11 non è bit-riproducibile dai propri artefatti; J15R1 lo è.
- `best_validation_mse` contaminato in entrambi i fit dalla ripetizione dei blocchi.
- La deviazione pre-breccia non è rilevabile su singola run: serve un confronto appaiato.
- **LOTO / LOCO / B1R1 / B1R2** e generalizzazione/Epic restano TODO futuri.
- **Semi 126–128 e fase G–I** restano sigillati.

---

## 13. STOP

Readiness completata. **Nessun fit, nessun candidato, nessuna `j18_runs`.**

**Fermo in attesa dell'audit Codex.**
