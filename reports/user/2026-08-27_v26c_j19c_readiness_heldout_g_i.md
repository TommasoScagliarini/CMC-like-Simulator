# V26C J19C — Readiness della fase held-out G/H/I

**Data**: 2026-08-27
**Stadio**: `V26C_J19C_J19A_HELDOUT_G_I` — **readiness**
**Esecutore**: Opus 5, effort xhigh — braccio operativo. Codex resta architetto e gate owner.

**Esito: GO tecnico di readiness. NON eseguito. Preregistrazione SIGILLATA.**

**Nessun rollout, nessun fit, nessun critic, nessun PPO, nessuna collection, nessuna `j19c_runs`,
nessun GO.**
Eseguiti solo compile, test statici, derivazione in memoria e il **preflight reale end-to-end**, che
è inerte. **Nessun `--run`.**

**Nessuna autorizzazione implicita al rollout.**

---

## 1. Problema

J19A ha superato J19B con **6/6** sulle celle A–F, seed 123–125. Quelle sei celle sono però le
stesse su cui l'intera catena è stata costruita e diagnosticata. Le tre celle stocastiche D, E ed F
hanno usato i semi 123, 124 e 125, già impiegati altrove nella catena. Resta la domanda che nessuna
di esse può sciogliere: **l'attore regge sotto realizzazioni del rumore che non ha mai incontrato?**

I semi 126, 127 e 128 sono stati tenuti **sigillati** per tutta la catena proprio per questo. Questa
fase li apre, e li apre una volta sola.

**Precisazione scientifica, dopo un tuo rilievo.** J19C **non testa partenze nuove**. Tutte e tre le
celle partono dallo **stesso offset nominale** 1.956870983805102 s, esattamente come le celle D, E ed
F di J19B. Ciò che è nuovo è il **seme**, e quindi la **realizzazione del rumore di esplorazione
tenuto**. Una partenza nuova sarebbe un `episode_start_offset_s` diverso: J19B ha già coperto
−0.20 s e +0.20 s nelle celle B e C, e J19C non copre nessuna delle due. La mia formulazione
precedente — «partenze che non ha mai visto» — era **errata**, ed è corretta qui, nella
preregistrazione e nel runner, con due test (`C68`, `C69`) che vietano il ritorno di quella prosa.

## 2. Strategia

**Stesso protocollo, tre celle nuove.** J19C è una **copia meccanica auditabile** di J19B, derivata
programmaticamente; J19B e J16 restano byte-invariati.

Il vincolo è espresso come **`core_scientific_equivalence` fail-closed contro J19B**, con una
**whitelist chiusa** di differenze autorizzate. Qualunque differenza fuori dalla whitelist **blocca
il preflight**.

**La preregistrazione precede qualunque uso dei semi.** Il documento
`v26c_j19c_prereg_heldout_g_i.json` è stato scritto **prima** del derivatore e del runner, ed è il
primo artefatto in cui 126/127/128 compaiono. Nulla è stato campionato o simulato con essi.

---

## 3. Le tre celle

| cella | modo | seed | offset_s | etichetta |
|---|---|---|---|---|
| **G** | `stochastic_held` | **126** | 1.956870983805102 | heldout nominal sigma 0.005 |
| **H** | `stochastic_held` | **127** | 1.956870983805102 | heldout nominal sigma 0.005 |
| **I** | `stochastic_held` | **128** | 1.956870983805102 | heldout nominal sigma 0.005 |

Tutte e tre alla **partenza nominale**, tutte `stochastic_held`, **sigma 0.005**,
**noise_hold_steps 1**, **500 step**, ordine G→I sequenziale, senza fail-fast comportamentale.
Sono il protocollo delle celle D/E/F di J19B con il solo seme sostituito.

Aggregato: **PASS se e solo se 3/3 comportamentali E 3/3 telemetry-valid.**

---

## 3-bis. Sigillatura e ENV_MUTATION_POLICY

**Sigillo.** `PIN_PREREG` non e' piu' `PENDING`: e' l'hash reale della preregistrazione, e il
controllo e' **incondizionato** - nessun ramo accetta piu' `PENDING`, verificato via AST
(`C45e`, `C45f`). Un test **muta il file della prereg** e verifica che il runner lo **rifiuti**,
ripristinandolo byte-identico (`C45g`, `C45h`).

**Un conflitto che ti espongo invece di risolverlo in silenzio.** Avevi indicato di sigillare
all'hash `739cadc0…`, quello allora corrente. Ma la prereg conteneva essa stessa una delle
affermazioni errate del punto 2 - «whether the J19A actor holds on **three starts it has never
seen**». Correggerla ne cambia l'hash. Sigillare `739cadc0…` avrebbe certificato una frase
scientificamente sbagliata, quindi ho corretto e sigillato al **nuovo** hash:

| | SHA-256 |
|---|---|
| prereg prima della correzione (indicato nella direttiva) | `739cadc09041eccef57a66273dd25e1f0ca16b6b99f73ee43f329b536071faee` |
| **prereg corretta e SIGILLATA** | **`54a1e6fc1f469ba169c80cc851be3fb1ab551e399553ed26c58aa1f101349e2d`** |

**ENV_MUTATION_POLICY.** La costante e' **pinnata e byte-identica a J19B**, e la core equivalence
esiste per provarlo: il suo campo `why` dice «the matrix is a sweep over the three preregistered
start offsets», che descrive la matrice A-F e **non** questa. **Non l'ho toccata.** Riformularla
avrebbe rotto proprio la byte-identita' che deve dimostrare.

Quello che ho fatto invece:

- e' una **superset conservativa ereditata**: la policy *autorizza* a variare
  `episode_start_offset_s`, e J19C semplicemente **non esercita mai** quell'autorizzazione;
- **l'unico campo che varia davvero fra G, H e I e' `output_dir`**;
- la prereg documenta la lacuna in `env_mutation_policy_note`;
- sei test la congelano: `C62` (un solo offset per tutte e tre le celle), `C63` (e' il nominale
  congelato, nessuna partenza perturbata), `C64` (la policy resta una superset), `C65`
  (byte-identica a J19B), `C66` (documentata), `C67` (il commento non parla piu' di sweep).

### 3-ter. La presentazione della policy, dopo il tuo ultimo rilievo

Avevi ragione: correggere prosa e commenti **non bastava**, perche' l'output JSON del preflight
serializzava `dict(ENV_MUTATION_POLICY)` e presentava quindi come **proprie di J19C** due
affermazioni false — il campo `why` con lo sweep, e la voce `FORBIDDEN_HERE` «episode_start_offset_s
is the ONLY one the matrix varies». Il receipt futuro avrebbe conservato la prima **senza contesto
correttivo**.

Ho anche scoperto che il mio innesto precedente sul commento era **atterrato nel posto sbagliato**:
aveva colpito un commento dentro `FORBIDDEN_HERE`, non accanto alla costante, generando un testo
confuso che diceva perfino «byte-identical to **J19C's**». Rimosso e rifatto correttamente.

**Correzione minima, senza toccare la costante ne' le otto funzioni bytecode-identiche.** Un solo
helper, `env_mutation_report()`, inserito dopo la costante, che rappresenta la policy in **due
sezioni esplicite**:

| sezione | contenuto |
|---|---|
| `inherited_conservative_superset` | copia **byte-identica** della costante J19B, con dichiarazione che il suo `why` descrive J19B e **non** J19C, che *autorizza* a variare l'offset e che questo stadio **non esercita mai** quell'autorizzazione |
| `effective_j19c` | `scientific_or_runtime_fields_that_actually_vary: []`, `episode_start_offset_s_varies: false`, offset **1.956870983805102 in tutte e tre**, `matrix_is_a_sweep: false`, unica differenza fra celle `output_dir` |

**La frase storica sopravvive solo dentro la sezione etichettata `inherited`**, verificato: presente
in `inherited`, assente in `effective`.

`FORBIDDEN_HERE` ora dice: *«mutating any scientific or runtime env field; in J19C NONE varies,
episode_start_offset_s included: the only per-cell difference is output_dir»*.

L'helper e' **fail-closed**: se la matrice variasse l'offset, **solleva** (`C78` lo dimostra).
Entrambi i siti — preflight **e** receipt futuro — passano dall'helper; `C79`/`C79b` verificano via
AST che la costante nuda sia serializzata **solo** dentro l'helper e dentro il confronto di
equivalenza, e **mai** in `preflight` o `run_matrix`.

---

## 4. Il difetto che hai rilevato, e gli altri due

Avevi trovato che `matrix_policy.aggregate_pass_iff` conservava ancora **"6/6"**. Confermato: J19B
scrive la regola aggregata in **due punti con formulazioni diverse**, e la mia mappa ne sostituiva
solo una — quella con `AND` maiuscolo. Il preflight di J19C dichiarava quindi 6/6 nel proprio output.

Cercando gli altri residui ne ho trovati **due ulteriori**, entrambi reali:

| riga | testo | verdetto |
|---|---|---|
| 6 | docstring: «run sequentially in the order **A-F**» | **errato** → G-I |
| 1050 | `aggregate_pass_iff: "6/6 …"` | **errato** → 3/3 |
| 1550 | `run_matrix`: «**All six cells**, in the frozen order» | **errato** → tre celle held-out |

E **quattro riferimenti che ho lasciato intatti**, perché sono citazioni storiche esplicite a J19B,
J9R1 o J18 e il tuo vincolo era di non alterarle:

- «the J19B receipt does not record a full 6/6» — controllo sull'evidenza di ingresso;
- «a cell-by-cell comparison against J9R1's A-F matrix» — spiega la decisione architetturale;
- «which PASSED J19B 6/6» — provenienza dell'attore;
- «cells A, C, D, E and F (2497)» — descrive i blocchi di training J18, non la matrice.

Un test (`C60`) verifica che **nessun "6/6" sopravviva su una riga che non nomini J19B**, e `C61` che
non resti prosa A-F sulla matrice propria.

Terzo difetto, mio: il blocco di equivalenza leggeva `ref.AGGREGATE_RULE`, ma **J19B non ha quella
costante** — lì la regola è un letterale inline. Corretto trascrivendola in `J19B_AGGREGATE_RULE` e
**verificando a run time che quella stringa occorra davvero nel sorgente J19B pinnato**, invece di
fidarmi della trascrizione.

---

## 5. Core scientific equivalence

Verificata a ogni preflight, **fail-closed**.

**Identico a J19B** — 8 funzioni **bytecode-identiche** (`base_env_config`, `cell_env_config`,
`expected_reset_time`, `unit_correction`, `evaluate_cell_gate`, `cell_verdict`,
`penetration_report`, `production_stack`) e **17 costanti core** confrontate per valore: gate table,
`SIGMA` 0.005, `SIGMA_TOLERANCE`, `NOISE_HOLD_STEPS` 1, `EXPECTED_STEPS` 500, `OFFSET_NOMINAL`,
`FROZEN_OFFSETS`, `ACTOR_WIDTH`, `CLOCK_COLUMNS`, `CONTROLLER_COLUMNS`, `RESET_TIME_TOLERANCE_S`,
`ENV_MUTATION_POLICY`, i tre pin dell'attore J19A e le fonti dei nomi feature.

**14 gate per cella**, telemetry-integrity, action path, env config, contratto di penetrazione,
recording, commit e fail-fast: tutti invariati. **Zero soglie inventate.**

**Whitelist chiusa — sei sole differenze autorizzate:**

| id | differenza |
|---|---|
| W1 | matrice **6 → 3** |
| W2 | id **A–F → G–I** |
| W3 | ogni cella `stochastic_held` |
| W4 | seed **123/124/125 → 126/127/128** |
| W5 | stringhe e path di stadio |
| W6 | aggregato **6/6 → 3/3** |

**Qualunque altra differenza solleva e blocca il preflight.**

---

## 6. File

| artefatto | SHA-256 |
|---|---|
| `v26c_j19c_prereg_heldout_g_i.json` — **SIGILLATA** | `54a1e6fc1f469ba169c80cc851be3fb1ab551e399553ed26c58aa1f101349e2d` |
| `v26c_j19c_derive_from_j19b.py` | `db63979fadaea96d7dc41427e40308b4a6ac4dce9c1371ad47ffdfefa3e7435e` |
| `v26c_j19c_heldout_g_i.py` | `b63a108259eac856ce9890bfa2a49437ed05489dc957ae5ad03b81c7abea2d8b` |
| `test_v26c_j19c_heldout_g_i.py` | `d27a1e7b0ce876bd4818a1edcd36978b95ec4a52290aabd5862b6950b5b478cc` |

Mappa di derivazione: **14 voci testuali / 166 hit**, **6 innesti strutturali / 6 hit**,
**totale 172**. Ogni voce colpisce almeno una volta. **Residui J19B fuori dai blocchi: NESSUNO.**

Un GO J19C dovrà pinnare questi quattro più `v26c_j19b_closed_loop.py` (`327b8fe3…`),
`v26c_j16_closed_loop.py` (`6ac45854…`), `v26c_penetration_contract.py` (`9257e9b8…`) e il
`module_state.pkl` di J19A (`8153dc97…`).

---

## 7. Test — 120/120

| gruppo | cosa morde |
|---|---|
| **C01–C08** | J19B e J16 al pin; il runner su disco è **esattamente** l'output dello strumento; ogni voce della mappa colpisce; zero residui; lo strumento **rifiuta** un J19B alterato |
| **C09–C22** | equivalenza core fail-closed; **8/8 funzioni bytecode-identiche**; 17 costanti identiche; sigma 0.005, noise hold 1, 500 step, start nominale; gate table e pin attore identici; contratto penetrazione pinnato; whitelist con **esattamente sei** voci |
| **C23–C29** | **NEGATIVI**: rifiutati seed 123, seed 125, seed 999, modo `deterministic`, start −0.20 s, start +0.20 s, cella rinominata, matrice a 2 celle, matrice a 6 celle; rifiutati sigma, noise hold, steps, offset nominale e actor width alterati; rifiutato il ritorno alla regola 6/6; **rifiutata una funzione scientifica derivata** |
| **C30–C33** | la CLI espone **esattamente** `--preflight --run --authorized-stage --out --no-progress`; **nessuna opzione** per seed, start, mode, sigma, steps o noise hold; `MATRIX`, `SIGMA`, `NOISE_HOLD_STEPS`, `EXPECTED_STEPS`, `OFFSET_NOMINAL` sono costanti di modulo assegnate **una sola volta** e mai dentro una funzione; nessuna variabile d'ambiente governa il protocollo |
| **C34–C45i** | prereg senza self-hash; stage coincidente; i semi dichiarati; le tre celle coincidono col runner; 14 gate e aggregato 3/3; bande esatte con 28 che passa; whitelist a sei voci; **evidenza d'ingresso J19B verificata PASS su 6 celle**; **il pin è l'hash reale**, il guard è **una sola comparazione incondizionata**, **nessun ramo accetta `PENDING`**, e **una prereg MUTATA sul file è RIFIUTATA** e poi ripristinata byte-identica |
| **C70–C83** | il report della policy ha **due sole sezioni etichettate**; l'inherited riproduce la costante byte-identica e dichiara che il suo `why` descrive J19B; l'effective dichiara **nessun campo scientifico variabile**, offset nominale in tutte e tre, unica differenza `output_dir`; **la frase storica sta solo in inherited**; l'helper **rifiuta** una matrice che varii l'offset; preflight e receipt passano entrambi dall'helper e la costante nuda non compare mai in `preflight`/`run_matrix`; `FORBIDDEN_HERE` non rivendica piu' un offset variabile |
| **C46–C69** | **preflight reale end-to-end**: `verdict GO`, `blockers []`; **nessun ray, torch, OpenSim o gym importato**; sentinella mai creata; albero **byte per byte invariato**; `aggregate_pass_iff` dichiara **3/3**; scriverebbe solo il leaf J19C; non promuove nulla; **`j19c_runs` assente**, nessun lock, nessuna staging, nessun GO; **J19A e J19B byte-invariati**; chiusura del namespace; nessun residuo 6/6 o A-F sulla matrice propria; **offset congelato** su tutte e tre le celle, policy superset byte-identica e documentata, **nessuna prosa propria che rivendichi partenze nuove** |

**Preflight reale**: `verdict GO`, `blockers []`, exit 0. **Nessun `--run` eseguito.**

---

## 8. Verifiche di invarianza

- `j19c_runs`: **assente**. Nessun lock, nessuna staging, nessun file GO.
- `PIN_PREREG` = **`54a1e6fc1f469ba169c80cc851be3fb1ab551e399553ed26c58aa1f101349e2d`**: la
  preregistrazione è **SIGILLATA**, e la guardia è **incondizionata** — nessun ramo accetta
  `PENDING`. Una prereg che non corrisponda al pin fa **rifiutare** l'avvio.
- **Byte-invariati**: `v26c_j19b_closed_loop.py` `327b8fe3…`, `v26c_j16_closed_loop.py`
  `6ac45854…`, `v26c_penetration_contract.py` `9257e9b8…`, J19A `module_state.pkl` `8153dc97…`,
  receipt J19B `789c57ca…`.
- Nessun fit, critic, PPO, collection, modifica scientifica.

---

## 9. Limiti

- Questa fase **non è stata eseguita**. Il `GO` riportato è l'esito del **preflight**, non un
  verdetto sull'attore.
- Tre celle non sono una misura di generalizzazione: sono **tre realizzazioni nuove del rumore alla
  stessa partenza nominale**. Non dicono nulla su partenze diverse, morfologie diverse o soggetti
  diversi. Le partenze perturbate ±0.20 s sono già state coperte da J19B, non da qui.
- LOTO, LOCO, B1R1, B1R2 e l'Epic multi-modello **restano fuori perimetro**.
- Nulla è promosso e nulla è definito deployable.
- Il margine più stretto osservato in J19B era **2.38 mm** sulla barra binding (cella F, 25.6151 mm).
  Le celle held-out sono stocastiche alla partenza nominale, quindi confrontabili con D/E/F, che in
  J19B stavano fra 24.46 e 25.62 mm. **Non è una previsione**, è il contesto entro cui leggere
  l'esito.

---

## 10. Esito

**GO tecnico di readiness: BLOCKED su nulla.** 120/120 test, preflight reale `GO` con
`blockers []`, preregistrazione sigillata.

**Fermo prima dell'esecuzione. Nessuna autorizzazione al rollout è implicata.**

---

## 11. TODO propagati

- **J19C non eseguita**: `PIN_PREREG` è sigillato; serve un GO che pinni gli otto artefatti.
- **LOTO / LOCO / B1R1 / B1R2** e l'Epic di generalizzazione restano fuori perimetro.
- `policy_std` sempre `null`, difetto cosmetico ereditato da J12.
- Il sidecar `actor_feature_manifest.json` di J8 resta stantio per decisione architetturale.
- La leaf J8 non ha `commit_verification.json`, come la leaf J2.
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1.
- Le suite readiness J18, J19A e J19B restano ciascuna con un check storicamente falso dopo
  l'emissione dei rispettivi GO: **non modificate**, per decisione architetturale.

---

## 12. Comandi eseguiti

```
python v26c_j19c_derive_from_j19b.py --check
python v26c_j19c_derive_from_j19b.py --write
python test_v26c_j19c_heldout_g_i.py                    # 120/120
python v26c_j19c_heldout_g_i.py --preflight             # verdict GO, blockers []
```

**Non eseguito**, e non autorizzato da questa fase:

```
python v26c_j19c_heldout_g_i.py --run --authorized-stage V26C_J19C_J19A_HELDOUT_G_I \
    --out j19c_runs/j19c_heldout_g_i_v26c_2026-08-27_r1
```

**STOP. In attesa del tuo audit e dell'eventuale GO.**
