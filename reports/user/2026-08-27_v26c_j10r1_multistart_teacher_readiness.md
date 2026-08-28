# V26C J10R1 — Multistart prescribed-teacher collection: readiness

**Data**: 2026-08-27
**Stadio proposto**: `V26C_J10R1_MULTISTART_TEACHER`
**Stato**: readiness pronta per la seconda review dell'architetto. **Nessun rollout eseguito.**
**Esito conferito**: nessuno. Questa fase non autorizza fit, critic, PPO, promozione o deployment.

---

## 1. Problema

La readiness **J10 è stata rifiutata dall'architetto prima dell'esecuzione**. J10 non è mai
stato eseguito: nessun rollout, nessun leaf, nessun lock, nessuno staging, nessun output.

Il rigetto non tocca matrice, parent, teacher, gate o scienza dell'esperimento. Riguarda
**come l'evidenza viene risolta, contata, hashata e committata**.

### Motivazioni del rigetto (registrate nel record additivo)

1. **Coverage risolta dal manifest sbagliato.** La feature `phase_fsm_wait_hs` doveva essere
   risolta dal manifest delle feature del parent. Quel manifest dichiara la slice June-lineage
   a **39** colonne, mentre il blocco attore runtime è a **35**: un indice risolto dal manifest
   non è l'indice che l'ambiente produce davvero. Verificato su disco: nel manifest
   `phase_fsm_wait_hs` è all'indice **21**, nel runtime è all'indice **17**. Un indice da
   manifest avrebbe contato `phase_expected_to`.
2. **Conteggio output troppo permissivo.** Il controllo su `sim_outputs` accettava una directory
   **non vuota**. Accetta 18 file da uno scrittore fallito in silenzio, e accetta 20 da qualcosa
   che ha scritto lì dentro.
3. **Nessun hash degli artefatti.** Il receipt registrava i *path* ma non gli *hash*: nulla a
   valle poteva provare che il file letto è il file misurato.
4. **Nessuna verifica post-commit.** Non veniva verificato che i path leaf-relative risolvano e
   che gli hash si riproducano dopo il rename dello staging sul leaf. Un commit non verificato
   può lasciare un leaf che *sembra* valido e non lo è.
5. **Nessun test dedicato** né sulla byte-identità del bundle J10 rifiutato, né sul rifiuto
   effettivo di un conteggio output sbagliato, di un path di staging o di un hash manomesso.

### Pin del bundle J10 rifiutato (immutabile)

| File | SHA-256 |
|---|---|
| `v26c_j10_prereg_multistart_teacher.json` | `79b1b573eb45831e7333c3dbc539f76a7e9555986b38e73ea83a91590f8241d7` |
| `v26c_j10_multistart_teacher.py` | `4d436aceaf0d78aee269f24cf90d47ca81877eecd49aeb155768b0613e35fa18` |
| `test_v26c_j10_multistart_teacher.py` | `fc1cf58721f84302b5a8f3665133cefde8d27ff130108189be7b39bec1ae1ba5` |
| `v26c_j10_multistart_teacher_authorization.json` | `d6a3fddd6fb1681fd65a49e0e1bdc5f7880c441d6ca059c941fc573f27a5346f` |
| `reports/user/2026-08-27_v26c_j10_multistart_teacher_readiness.md` | `a1f75107673a58deb798424c33cf1e142523c9573d9b812d31ce3a88287c80f9` |

Questi file sono **preservati byte-identici** come record di ciò che è stato proposto e rifiutato.
Non vengono mai modificati, riparati o eseguiti. Il runner J10R1 ne verifica gli hash e **si
rifiuta di partire** se uno di essi è cambiato (verificato: la manomissione di
`v26c_j10_multistart_teacher.py` blocca `verify_prereg`, e il file viene ripristinato identico).

---

## 2. Soluzione: J10R1, successore additivo

J10R1 è un **successore**, non una modifica. Ha preregistrazione, runner, test, autorizzazione,
readiness e leaf propri. Matrice, parent, teacher, gate e scienza sono **invariati**.

- **Leaf**: `j10r1_runs/j10r1_multistart_teacher_v26c_2026-08-27_r1`
- **Matrice**: 2 celle, B a `-0.20 s` (offset `1.756870983805102`) e C a `+0.20 s`
  (offset `2.156870983805102`), 500 step ciascuna, seed 123, ordine congelato B poi C.
- **Parent**: `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best`,
  `module_state.pkl` = `0ba56eb7…`. Fissa la lineage, **non è mai la sorgente d'azione** e non
  viene mai caricato. Lo studente J8 non è mai parent, policy o sorgente di label.
- **Label**: `target_domain_imitation.prescribed_teacher_action`, riuso read-only. σ = (0, 0),
  lookahead 0. Non è una policy.
- **Coverage gate**: vincolante, sull'**intera collezione**, criterio "almeno una riga con
  `phase_fsm_wait_hs == 1`". Nessuna soglia inventata, nessun minimo, nessun requisito per cella.

### Le cinque correzioni obbligatorie

**1 — Coverage risolta dal runtime vivo.** `runtime_feature_names(base)` legge
`base.actor_feature_names` dall'ambiente costruito, esattamente come `v26c_j1_collect.py:621`, e
richiede larghezza 35. Il receipt dichiara `resolved_by` = *LIVE RUNTIME ENVIRONMENT*, con
`not_resolved_from` che nomina esplicitamente il manifest a 39. Il manifest è letto in un solo
punto (`parent_manifest_width()`), il cui unico consumatore è un campo di provenance a sola
registrazione.

**2 — Esattamente 19 file regolari.** Dopo `env.close()` la directory `sim_outputs` di ogni cella
viene elencata una volta sola; entry non regolari (directory, symlink) sono rifiutate; il conteggio
deve essere **esattamente 19**. Il receipt registra conteggio e nomi. Il test asserisce il
*messaggio* della guardia, non solo il tipo di eccezione: 18 e 20 devono fallire **su quel
controllo**, non su uno dei ~15 altri raggiungibili prima.

**3 — Hash di tutto, prima del receipt.** Per ogni cella: SHA-256 dei quattro artefatti
(`teacher_dataset.npz`, `trace.json`, `kinematics.npz`, `penetration.npz`) e di tutti i 19
`sim_outputs` — 23 hash per cella, 46 in totale — con chiave **leaf-relative**, calcolati dopo che
tutti gli artefatti sono scritti e chiusi e **prima** che il receipt venga scritto. L'hash di
`teacher_dataset.npz` è dichiarato **vincolante per il futuro J11**, che dovrà verificarlo prima di
consumare il dataset. Registrare quell'hash **non autorizza alcun fit**.

**4 — Verifica post-commit, con leaf che nasce invalido.**

Il punto delicato è che `os.rename` è atomico, ma l'atomicità non dice nulla su *se i byte
committati siano i byte misurati*. La semantica adottata:

- Un marker `TECHNICAL_INVALID` viene scritto **nello staging, prima del rename**, quindi viene
  committato *dal rename stesso*. **Non esiste alcun istante in cui un leaf esista senza dichiararsi
  non verificato** — nemmeno se il processo viene ucciso tra rename e verifica.
- Dopo il rename si ri-risolve ogni path leaf-relative registrato **nel receipt committato** (non
  nella copia in memoria) e si ricalcola ogni hash.
- Il receipt stesso è confrontato con lo SHA misurato sul receipt staged: un receipt corrotto non
  può autovalidarsi. Il receipt viene letto **una volta sola**: gli stessi byte sono quelli hashati,
  confrontati e parsati.
- Un receipt che abbia perso o riordinato una cella è **rifiutato**, non "verificato" su ciò che
  resta.
- Se la verifica passa: `commit_verification.json` con `pass: true`, e la rimozione del marker è
  **l'ultima scrittura** del commit.
- Se fallisce: `commit_verification.json` con `pass: false`, il marker viene riscritto per dire che
  la verifica è fallita, e il runner solleva. Il leaf è **preservato** come evidenza sul commit: mai
  cancellato, mai riparato, mai ritentato.
- La verifica cattura `Exception`, **non** `BaseException`: un Ctrl-C propaga invece di essere
  registrato come verdetto, e non invalida in modo permanente una raccolta byte-perfetta.
- Se la scrittura di `commit_verification.json` stessa fallisce (ENOSPC, EROFS, EIO — proprio la
  classe di guasto che questa verifica esiste per rilevare), il marker è già lì e il leaf resta
  invalido.

**Regola di validità**: un leaf J10R1 è evidenza valida **se e solo se** `commit_verification.json`
esiste al suo interno e dichiara `pass: true`.

**5 — Test dedicati.** Byte-identità del bundle J10 (per nome, non solo per conteggio); rifiuto di
18 e di 20 output con asserzione sul messaggio; rifiuto di una entry non regolare; nessun path di
staging nel receipt; manomissione di un `sim_output`, di `teacher_dataset.npz`, artefatto
cancellato, ventesimo file intruso post-commit, path che evade il leaf; e il percorso "verifica
fallita → marker + `pass: false` + leaf preservato + lock e staging rilasciati".

---

## 3. Strategia di verifica

Oltre ai test, sono stati eseguiti **tre reviewer indipendenti read-only in parallelo** su commit
atomico, hashing/provenance e rigore dei test. Hanno trovato **19 difetti reali, tutti corretti**.
I più significativi:

| # | Difetto | Correzione |
|---|---|---|
| 1 | La scrittura di `commit_verification.json` non era protetta: un `OSError` lasciava un leaf popolato **senza** marker né file di verifica | Scritture protette; il marker pre-commit rende comunque il leaf invalido |
| 2 | Finestra tra `rename` e la scrittura della verifica in cui il leaf sembrava valido | **Leaf nato invalido**: marker scritto nello staging prima del rename |
| 3 | `except BaseException` attorno alla verifica: un Ctrl-C invalidava per sempre una raccolta integra | `except Exception`; `BaseException` propaga |
| 4 | Un receipt con `cells: []` dava `pass: true` con 0 file controllati | La lista celle deve corrispondere alla matrice congelata, in ordine |
| 5 | Il receipt — l'unico file di cui il verificatore si fida — non era controllato | Confronto con lo SHA dello staging; lettura unica dei byte |
| 6 | Receipt hashato da letture indipendenti: poteva certificare byte diversi da quelli riportati | Una sola lettura; stessi byte hashati, confrontati e parsati |
| 7 | `sim_outputs` rielencata due volte nel verificatore | Un solo elenco, classificato due volte |
| 8 | `_leaf_rel` poteva emettere un path con `..` (relative_to è lessicale) e sollevava `ValueError` nudo | Rifiuto esplicito in `J10R1Error`, prima che spenda due rollout |
| 9 | Il digest `env_config_sha256` includeva il path di staging → **irriproducibile per costruzione** | Digest che esclude `output_dir`, con la directory registrata a parte in forma leaf-relative |
| 10 | Il digest runtime top-level descriveva la config **inerte** del preflight | Stesso metodo di hashing per base e celle, quindi confrontabili |
| 11 | `paths_are_leaf_relative: True` era **falso** per 5 campi repository-relative | Claim ristretto agli artefatti prodotti; i campi repo-relative sono elencati |
| 12 | Due root di path fusi in un dict senza marcatore di root | Due dict separati e nominati |
| 13 | `PIN_REJECTED_J10_REPORT_REL` dichiarato e mai usato | Ora vincola il nome del report nel prereg |
| 14 | Set di pin duplicato nel prereg, uno solo applicato | Il runner rifiuta se le due copie divergono |
| 15 | Costanti di reset preregistrate con l'ultima cifra sbagliata | `13.746870983805103` / `14.146870983805101`, i valori che l'aritmetica IEEE-754 produce |
| 16 | Il preflight calcolava un reset time da `t_end = 30.0` **hardcoded**, mentre il vivo è 21.0 | Rimossa la falsa precisione: `t_start`/`t_end` vengono dal setup XML e non esistono inertemente, quindi il preflight non pubblica alcun numero e il controllo autoritativo resta a run time a 1e-9 s |
| 17 | Il test 18/20 poteva passare per una qualsiasi delle ~15 altre eccezioni | Asserzione sul messaggio della guardia |
| 18 | Gate cinematico confrontato **con se stesso** (`== dict(J3...)`), non falsificabile | Ancorato ai valori letterali |
| 19 | Correzione 1 senza test comportamentale (`FakeEnv.names` mai variato); clipping mai esercitato; nessuna cella che fallisse davvero | Feature spostata in colonna 0 → stesse 24 righe trovate; larghezza ≠ 35 rifiutata; clipping end-to-end con cella B FAIL e cella C che gira comunque |

Difetto residuo **noto e dichiarato**: `rename(2)` POSIX sostituisce in silenzio una directory di
destinazione **vuota**, quindi il controllo di non-esistenza immediatamente prima del rename è
check-then-act. Il lock esclusivo esclude ogni altra run di questo stadio; il caso residuo è un
processo estraneo che crei una directory vuota sul path del leaf dentro quella finestra, che non
distruggerebbe evidenza perché una directory vuota non ne contiene. Windows fallisce chiuso
(`FileExistsError`). Dopo il rename il leaf deve inoltre portare il marker pre-commit, quindi un
leaf che non sia quello staged viene rifiutato.

---

## 4. File creati

| File | SHA-256 |
|---|---|
| `v26c_j10r1_prereg_multistart_teacher.json` | `72fcd6b9971ac2d3a2d8dda68af8d1aec9aba366c0a2a9ca679317d26fa12874` |
| `v26c_j10r1_multistart_teacher.py` | `b0e13e599e91aeeafb5ae178d8b164aa670645a7c97c4584af5344cc97c700e6` |
| `test_v26c_j10r1_multistart_teacher.py` | `6debac1897cc61ceaf3f9f7b411136ebfb92ccb07e4276961f1cceac94b73ee2` |
| `v26c_j10r1_multistart_teacher_authorization.json` | `36bb09a14d76dee90e93b5efb1f9e18db5b8da08b5a3a7cee831c06f02d23006` |

Tutti additivi. **Nessun file J0–J9R1, nessun artefatto July, nessun report, nessuna
configurazione di produzione è stato modificato.** FSM, corridoio morfologico, reward, SEA e plugin
C++ non sono toccati.

## 5. Test e verifiche eseguite

- `py_compile` su runner e test: OK.
- Suite sintetica: **357 check, PASS**. Nessun residuo sotto la root di validazione
  (`snapshot(HERE) == before_all`).
- Preflight inerte: **GO**, nessun blocker. Nessun torch/RLlib/OpenSim importato, nessun ambiente
  costruito o resettato, nessun output scritto, sentinella mai creata.
- Rifiuti di argomento verificati: `--collect` senza stage token, con il token J10, e `--out` senza
  `--collect` falliscono tutti fail-closed.
- Assenza confermata su disco: nessun `j10r1_runs`, nessun `j10_runs`, nessun lock, nessuno
  staging, nessuna sentinella.
- Bundle J10 verificato byte-identico ai cinque pin.
- `git status`: nessun file tracciato modificato oltre ai tre già modificati a inizio sessione.

## 6. Cosa **non** è stato fatto

- **Nessun rollout reale.** I due rollout della matrice non sono stati lanciati.
- Nessun fit, critic, PPO, cluster Ray, env runner o rollout dello studente.
- L'autorizzazione è scritta ma dichiara `execution_permitted_now: false`.

---

## 7. TODO propagati

- **LOTO** — non integrato. TODO futuro, non J10R1.
- **LOCO** — non integrato. TODO futuro, non J10R1.
- **B1R1** — non integrato. TODO futuro, non J10R1.
- **B1R2** — non integrato. TODO futuro, non J10R1.
- **Epic generalizzazione multi-modello** — APERTO. Questa fase copre ancora AB06 e tre start
  dello stesso trial; non dimostra alcuna generalizzazione ad altri soggetti, trial o velocità.
- **Il fit che userà questi dati (J11)** — NON fa parte di questa fase. Combinare queste righe con
  J7 e rifittare è uno stadio separato, con propria preregistrazione e propria autorizzazione.
  Vincolo già registrato: J11 dovrà verificare lo SHA-256 di `teacher_dataset.npz` prima di
  consumarlo.
- **J9R1 FAIL su cella B** — resta la ragione per cui questa raccolta esiste. Il checkpoint non è
  training-ready finché B fallisce, e il contratto canonico `training_exnovo_cfg` usa
  `exact_start_sampling: true` con tutti e tre gli offset.
- **Bundle J10 rifiutato** — resta byte-identico come evidenza. Mai modificato, mai eseguito. TODO
  solo per l'architetto: decidere se archiviarlo.

---

## 8. Attesa

**Fermo per la seconda review dell'architetto.** Nessuna esecuzione senza un GO esplicito.
