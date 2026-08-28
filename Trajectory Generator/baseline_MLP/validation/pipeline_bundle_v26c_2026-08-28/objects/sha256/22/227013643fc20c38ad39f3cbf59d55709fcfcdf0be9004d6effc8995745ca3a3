# V26C J15R1 — Readiness correttiva: la correzione del runner

**Data**: 2026-08-27
**Stadio**: `V26C_J15_FRESH_35D_POST_MISMATCH_REFIT` — nome operativo **J15R1**
**Esito**: bundle correttivo completo e verificato. **Nessun fit eseguito.**
**Autorizzazione**: `execution_permitted_now: false`.
**Report obbligatorio di fine fase.**

---

## 1. Problema

L'unica esecuzione autorizzata di J15 è fallita in 0.062 s con
`NameError: name 'verify_warm_start_source' is not defined`, sollevato dentro `run_fit` prima dei
dati, di torch, dell'ottimizzatore e di qualunque percorso di scrittura.

`run_fit` è **spliciata verbatim** da J11 — scelta fatta per non alterare la matematica
ricopiandola a mano. Ma una funzione copiata porta con sé le **dipendenze di namespace della
propria origine**: chiama `verify_warm_start_source()` come nome nudo. Avevo aliasato ogni altro
helper necessario, non quello.

**Il difetto vero non era la riga mancante: era che nulla nella readiness eseguiva `run_fit`.** La
suite usa un doppio iniettato per non richiedere un training da 400 epoche; il preflight non
raggiunge mai quella funzione. Così 208 check e un preflight GO sono passati mentre il percorso
principale di produzione non era eseguibile.

---

## 2. La correzione, e solo quella

Una binding a livello di modulo:

```python
verify_warm_start_source = J11.verify_warm_start_source
```

### Diff esatto contro il runner fallito

```
--- v26c_j15_fresh_refit.py (FAILED)
+++ v26c_j15_fresh_refit_r1.py
@@ -1,4 +1,9 @@
 #!/usr/bin/env python
-"""V26C J15 - fresh 35D supervised refit from the J2 parent, post-mismatch corrected.
+"""V26C J15R1 - fresh 35D supervised refit from the J2 parent, post-mismatch corrected.
+
+ADDITIVE CORRECTED SUCCESSOR of v26c_j15_fresh_refit.py (sha256 7b0073c8...), which is
+PRESERVED BYTE-IDENTICAL as the record of a failed execution. The only difference is the
+module-level binding of verify_warm_start_source; every scientific and commit semantic is
+unchanged, and a test proves the bytecode of every shared function is identical.
 
 WHAT THIS STAGE IS
@@ -208,4 +213,10 @@
 _resolve_inside = J11._resolve_inside
 actor_state_digest = J11.actor_state_digest
+# THE J15R1 CORRECTION, and the whole of it. run_fit is spliced verbatim from J11 and calls
+# verify_warm_start_source() as a BARE NAME, resolved in its origin's module namespace. J15
+# aliased every other helper it needed but not this one, so the production fit entry path
+# raised NameError before touching data, torch or any write path. Binding it here is the
+# minimal semantic fix; a namespace-closure test now makes the class of defect unrepeatable.
+verify_warm_start_source = J11.verify_warm_start_source
 
 
```

**13 righe**: una riga di docstring sostituita da cinque che identificano il file come successore
additivo, cinque righe di commento e **una** binding. Nessun'altra riga differisce.

### Prova meccanica dell'identità scientifica

Non basta guardare il diff, quindi il test lo dimostra:

- **ogni funzione a livello di modulo condivisa dai due runner compila a bytecode identico**
  (`co_code` confrontato funzione per funzione);
- **21 costanti scientifiche** confrontate uguali: `STAGE`, `TOTAL_ROWS`, `EXPECTED_N_VAL`,
  `EXPECTED_N_TRAIN`, `NEW_ROWS`, `CELL_REPEAT`, `CLOCK_COLUMNS`, `ACTOR_WIDTH`, `ACTION_DIM`,
  `DIRECT_KEYS`, `ALIAS_PAIRS`, `J15_BINDING_NAMES`, `J11_BINDING_SUBSET`,
  `J15_ADDITIONAL_BINDING`, `RELATIVE_LEAF`, `PIN_PREREG`, `PIN_PARENT_STATE`, `JULY_HP`,
  `BEST_EPSILON`, `NEW_CELL_SPANS`, `BLOCKS`;
- `pinned_sources()` restituisce un insieme identico nei due;
- `gate_matrix()` restituisce una struttura identica — **17 regole binding, invariate**;
- il modulo corretto **aggiunge esattamente un nome e non ne rimuove nessuno**.

Per questo riuso lo **stesso STAGE**, la **stessa preregistrazione byte-identica**
(`49c748b3…`) e la **stessa leaf autoritativa** ancora assente: ogni semantica scientifica e di
commit è provata invariata.

---

## 3. Il nuovo controllo: chiusura di namespace

Basato sulla **semantica del codice Python**, non su una ricerca testuale fragile.

Per ogni funzione a livello di modulo il test disassembla il code object **e ogni code object
annidato** in `co_consts` — così copre anche `forward`, `project` e le closure interne — e
raccoglie l'`argval` di ogni `LOAD_GLOBAL`, `STORE_GLOBAL`, `DELETE_GLOBAL`. È esattamente ciò
che CPython cercherà nel namespace del modulo a run time.

**Asserzione: zero global non risolti**, su tutte le funzioni di modulo e individualmente su 22
funzioni critiche per l'esecuzione, `run_fit` inclusa.

**Regressione esplicita**: `run_fit` legge `verify_warm_start_source` come global, il runner
corretto lo lega, ed è legato all'implementazione congelata di J11 — non a una reimplementazione.

**Il controllo è provato prima di essere creduto.** La stessa analisi applicata al runner **fallito**
riporta esattamente:

```
{'run_fit': ['verify_warm_start_source']}
```

Se questo controllo fosse esistito nella readiness di J15, l'esecuzione fallita non sarebbe mai
stata autorizzata. Il runner fallito è preservato e **mostra ancora il difetto**: un test verifica
che non contenga la binding, così la riparazione non è stata fatta di nascosto su di lui.

---

## 4. Analisi di namespace, misurata

| funzione | global non risolti |
|---|---|
| `run_fit` nel runner **fallito** | **1 — `verify_warm_start_source`** |
| `run_fit` nel runner **corretto** | **0** |
| tutte le altre funzioni di modulo, in entrambi | 0 |

`run_fit` ha 17 nomi liberi: nel runner fallito 16 risolvevano e uno no. L'estensione del difetto
era esattamente quella — un nome, in una funzione.

---

## 5. Preservazione della catena fallita

Nessuno di questi file è stato modificato, riparato o cancellato. Un test ne verifica gli hash a
ogni esecuzione.

| artefatto | SHA-256 |
|---|---|
| `v26c_j15_fresh_refit.py` (runner **fallito**) | `7b0073c8832e1af64a243cf9bff70662fb786f84691d3a5eab451e230192507a` |
| `test_v26c_j15_fresh_refit.py` | `5bdeaf8d72b3619c5bca0b99b9bfa1da1644804fe043ba32936e95d920fd1176` |
| `v26c_j15_fresh_refit_authorization.json` (rev3) | `61b6fa4fd45fcf00a418efdcb3a450b7eeaddd56b486cc481ef9de44f127a3c4` |
| `v26c_j15_architect_go_2026-08-27.json` (rev1) | `d6c661aa2dd826c73d022420414783810d964e29680be5e8b6468cbca6ff093c` |
| `v26c_j15_architect_go_2026-08-27_rev2.json` (**consumato**) | `05d58a33fe1a8f5dbaeec156e588ea4cb4b4c0ad0fa0c9638da51a80c841a3a2` |
| `j15_runs/j15_execution_exit_2026-08-27.json` | `9c27a8b8556e4046cfebf08b40925a0e97eeac9991b6026c5bc6001e5bcb70c5` |
| `j15_runs/j15_execution_stderr_2026-08-27.txt` | `310cba674c013003a9ab36f78dd0e7d6690e375107b20c531af098edb27342b5` |
| `j15_runs/j15_execution_stdout_2026-08-27.txt` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| report di esecuzione fallita | `92eaa78787c0f9cfb934cfeb9c879844bda3982ef36fffb80cc5976d49187ee5` |

`j15_runs/` contiene **esattamente** i tre log e nulla più: nessuna leaf, nessun lock, nessuno
staging. Verificato anche dalla suite.

---

## 6. Artefatti nuovi

| file | SHA-256 |
|---|---|
| `v26c_j15_fresh_refit_r1.py` | `2972e4b4dbfe6e247b48b94cb6c4c8606cd8c893b59f0cbdc30c72220129efb1` |
| `test_v26c_j15_fresh_refit_r1.py` | `8a7f00775cb4b848f80bceebe6066ce7a521b1642c9044d2e5724a2bd56a4c5a` |
| `v26c_j15r1_fresh_refit_authorization.json` | `45dd836eecdbb20dfe3e804b5111803dad51729be7e18a6dfbe78ca2aefad216` |
| **riusata invariata**: `v26c_j15_prereg_fresh_refit.json` | `49c748b3a20925a0c270768aa6fddcd3adee474eb1de1cfaa885f849719d1ad2` |

---

## 7. Contratto scientifico, invariato

Lineage **V26 agosto imitation → J2 35D**. Aggregato **25567**. Split **5113 / 20454**.
Iperparametri **esattamente quelli di J11**, dalla fonte J8 congelata. **Un solo attore 35D**;
nessun widening, nessun 25D, nessuna feature controlaterale. Mean network **interamente
addestrabile**. Clock **hard-zero**. Log-std **proiettata e congelata**. Critic **assente**.
**17 gate binding**, incluso quello sull'intero blocco J14 da 854 righe. **Nessuna soglia e nessun
iperparametro cambiati.**

---

## 8. Verifiche eseguite

Tutte read-only, statiche, selftest e preflight, con `PYTHONDONTWRITEBYTECODE=1` come richiesto.

| controllo | esito |
|---|---|
| `py_compile` runner r1 + test r1 | **OK** |
| suite r1 | **300 check PASS** |
| preflight r1 | **GO**, 0 blocker, 23 sorgenti pinnate, non scrive nulla |
| stage nel preflight | `V26C_J15_FRESH_35D_POST_MISMATCH_REFIT` — lo stesso |
| aggregato / split | 25567 · 5113/20454 |
| gate | **17**, `identical_to_j11: false` |
| parent | J2 · `is_j11: false` |
| leaf attesa | `j15_runs/j15_fresh_refit_v26c_2026-08-27_r1`, 8 file |
| torch importato dal preflight | **no** |
| semi sigillati letti | **0** |
| chiusura di namespace | **zero global non risolti**, ovunque |
| il controllo morde sul runner fallito | **sì**, e riporta esattamente il nome atteso |
| catena fallita | **9 artefatti byte-identici** |
| leaf autoritativa / lock / staging / sentinella | **assenti** |
| `--fit` invocato in questa fase | **mai**, né il vecchio né il corretto |
| worktree | preservato: solo i tre file tracciati già dirty a inizio sessione |

---

## 9. Stato dell'autorizzazione

Il GO rev2 autorizzava **una** esecuzione ed è **consumato**: `executions_performed: 1`, esito
fallito. Il runner fallito **non va più invocato**.

Il record `v26c_j15r1_fresh_refit_authorization.json` dichiara `execution_permitted_now: false` e
registra esplicitamente che una futura esecuzione corretta **richiede un tuo nuovo GO** e sarà
permessa **una sola volta**. Non mi autoautorizzo nulla.

---

## 10. Limitazioni

Non esiste alcun attore J15: nessun peso è stato prodotto, **nessuno dei 17 gate è mai stato
valutato**. Lo stadio resta **solo supervisionato** e non avanzerà alcuna pretesa closed-loop
neppure a fit riuscito. Il problema diagnosticato in J13 — lo swing che non si chiude — resta
**intatto e non affrontato**.

---

## 11. TODO propagati

- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Semi 126, 127, 128** — riserva held-out finale, mai letti.
- **Il gate A–F di regressione closed-loop** dopo un refit riuscito.
- **Regola di governance**: ogni fase si chiude con uno user report dedicato, auditato prima della
  fase successiva. Questo report chiude la fase J15R1-readiness.

---

## 12. Prossimo passo

Il tuo audit. Se approvi, serve un **nuovo record GO additivo** che pinni i tre hash di input di
J15R1 e autorizzi **una sola** esecuzione. Fino ad allora non parte nulla.

**Fermo in attesa del tuo audit.**
