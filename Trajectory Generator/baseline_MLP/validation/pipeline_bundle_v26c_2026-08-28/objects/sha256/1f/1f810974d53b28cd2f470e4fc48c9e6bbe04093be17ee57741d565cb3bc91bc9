# V26C J14 — Correzione rev3: bytecode all'import e `Path.open` non intercettata

**Data**: 2026-08-27
**Stadio**: `V26C_J14_POST_MISMATCH_DAGGER_DATASET`
**Revisione**: **rev3**, superseding rev2
**Stato**: bundle corretto, in attesa di un nuovo audit. **Leaf non materializzato.**
**Autorizzazione**: `execution_permitted_now: false`.
**Nessuna materializzazione, fit, rollout, seed 126–128, critic o PPO.**

---

## 1. I due rilievi

Entrambi reali, entrambi accettati. Non erano dettagli formali: colpivano l'unica cosa che rev2
affermava, cioè che la validation root non cambia di un byte.

### 1.1 — Il bytecode scritto prima che la guardia esistesse

Il test importava `v26c_j14_dagger_dataset` **prima** di installare `HermeticGuard`, e `snapshot()`
escludeva `__pycache__`. Un `.pyc` scritto all'import era quindi invisibile a **entrambi** i
controlli: la guardia non c'era ancora, e la fotografia iniziale non guardava lì.

Non è un'ipotesi. Il file è su disco:

```
__pycache__/v26c_j14_dagger_dataset.cpython-310.pyc   45917 byte   27 ago 07:19
```

scritto dall'import di questa stessa suite. Controprova diretta:

```
import senza dont_write_bytecode  -> pyc creato: __pycache__/demo_mod.cpython-310.pyc
dont_write_bytecode PRIMA import  -> pyc creato: NESSUNO
```

La conclusione dell'audit è esatta: **«not one byte» non era self-contained**, dipendeva dal fatto
che chi lancia la suite esportasse `PYTHONDONTWRITEBYTECODE`.

### 1.2 — `Path.open` in scrittura non intercettata

`pathlib.Path.open` non passa da `builtins.open`. La guardia non la vedeva, e la frase
«write-mode open» era più larga di ciò che veniva davvero coperto.

**Il buco è più profondo di come è formulato nel rilievo.** Su questo interprete (CPython 3.10)
`pathlib` cattura la funzione **originale** al momento della definizione della classe:

```
io.open is builtins.open                     -> True
_NormalAccessor.open is io.open originale    -> True
Path.open intercettata patchando io.open     -> False
```

Quindi **patchare `io.open` da solo non avrebbe comunque raggiunto `Path.open`**: `_NormalAccessor`
tiene un riferimento diretto all'oggetto funzione, non fa un lookup sul modulo. Ho verificato
empiricamente prima di scrivere la correzione. Per questo la guardia patcha **`io.open` e
`Path.open`**, con una probe distinta per ciascuna: la probe su `io.open` da sola non proverebbe
nulla su `Path.open`, e viceversa.

---

## 2. La correzione

**Ambito: solo il test.** Runner, preregistrazione rev1, scienza, matrice, celle, semi, regola di
etichettatura, politica di troncamento, soglie e conteggi **invariati**. Il runner è byte-identico:
`fe7c4951c195224b…`.

| | rev2 | rev3 |
|---|---|---|
| `sys.dont_write_bytecode` | mai impostato | **True, riga 23, prima dell'import a riga 24** |
| `snapshot()` | escludeva `__pycache__` | **nessuna esclusione** |
| primitive `open` guardate | `builtins.open`, `os.open` | **+ `io.open`, + `Path.open`** |
| probe | 10 | **12** |
| check | 216 | **230** |

**Nuove asserzioni:**

- `sys.dont_write_bytecode is True` (MEASURED) e la riga che lo imposta precede l'import (STATIC,
  per numero di riga).
- Il **codice** di `snapshot()` non contiene alcuna esclusione: scan AST del corpo con la docstring
  esclusa — nessun literal `__pycache__`, e un solo filtro, `is_file()`.
- Un file dentro `__pycache__` **è davvero catturato** da `snapshot()` (MEASURED, nel temp root).
- Il `.pyc` del runner è **dentro** la fotografia iniziale, quindi deve sopravvivere alla suite
  byte-identico.
- `builtins.open`, `io.open` e `Path.open` sono **tutti e tre** gli oggetti guardati mentre la
  suite gira, e `Path.open` è realmente sostituita, non aliasata all'originale.
- `Path.open(..., "w")` in scrittura **dentro** il temp root continua a funzionare.

**Due nuove probe**, entrambe bloccate: `Path.open("w")` puntata al record rev0 congelato — con
ri-hash del record subito dopo, per provare che non è stato toccato — e `io.open("w")` sotto la
validation root.

---

## 3. Un errore mio durante la correzione

Il primo check che ho scritto per l'esclusione era
`"__pycache__" not in inspect.getsource(snapshot)`. È fallito, e giustamente: la **docstring** di
`snapshot()` nomina `__pycache__` per spiegare che non lo esclude. Un match su testo grezzo non
distingue codice da prosa.

Sostituito con uno scan AST del corpo della funzione, con la docstring rimossa: nessuna costante
stringa `__pycache__` nel codice, e un unico `if`, che è `p.is_file()`. La docstring può nominarlo;
il codice no.

---

## 4. Portata dell'esenzione `dir_fd`, dichiarata stretta

Il commento di rev2 lasciava intendere una copertura più ampia di quella provata. Ora dice
esattamente cosa è e cosa non è:

- le chiamate con `dir_fd=` sono **delegate senza controllo**;
- questa suite **non effettua alcuna chiamata fd-relative diretta** — asserito con scan AST sul
  proprio sorgente (`kw.arg == "dir_fd"`, zero occorrenze);
- l'unico codice che raggiunge quel ramo è la discesa interna di `shutil.rmtree`, il cui punto
  d'ingresso è a sua volta guardato;
- **non è una garanzia universale**: una scrittura fd-relative emessa da codice esterno a questa
  suite passerebbe.

Lo stesso perimetro è scritto in `scope_of_the_hermeticity_claim` dentro il record rev3, diviso in
`proved`, `delegated_unchecked` e `not_claimed`.

---

## 5. Verifiche

Tutte eseguite **senza** `PYTHONDONTWRITEBYTECODE` nell'ambiente, che è il punto.

| controllo | esito |
|---|---|
| `py_compile` runner + test, `PYTHONPYCACHEPREFIX` fuori dalla validation root | **OK** |
| suite completa, `PYTHONDONTWRITEBYTECODE` **assente** | **230 PASS** |
| preflight read-only | **GO**, 0 blocker, 21 sorgenti, leaf J12 `FAIL` / J10R1 `PASS` / J11 `PASS` |
| snapshot esterno completo prima/dopo, **`__pycache__` incluso** | **631 file** (di cui **50** in `__pycache__`) — **0 aggiunti, 0 rimossi, 0 cambiati** |
| digest root, **tre invocazioni consecutive** | **invariante** prima/dopo ogni run |
| digest root riproducibile, **escluso il record rev3** | `8863d50af54fbbded2f3c2c91992dee4a3139ab0e0f9986d77d356057c8431d0` su **630 file** |
| 21 fonti congelate | **21/21 verificate** |
| pin authorization rev3 | **24/24 verificati** |
| record superseduti rev0/rev1/rev2 | **4/4 byte-identici** |
| `j14_runs` / lock / staging / sentinella | **assenti** |

Prima della creazione di rev3 il conteggio esterno era **630 file**, che coincide con il digest
indipendente riportato nel tuo audit; il 631° è il record rev3 stesso.

### Un limite che ho scoperto verificando, e che dichiaro invece di nasconderlo

Una **mia** verifica finale ha importato il test **come modulo** (`import test_v26c_j14_…`) per
controllare che la guardia ripristinasse `io.open` e `Path.open`. L'interprete ha compilato e
riscritto `__pycache__/test_v26c_j14_dagger_dataset.cpython-310.pyc` **prima** che la prima riga del
corpo potesse disattivare il bytecode. Il digest root è cambiato di conseguenza.

Ho misurato l'impatto esatto: **28/28 pin re-verificati** (24 dell'authorization + 4 record
superseduti), tutti esatti. L'unico file cambiato è quel `.pyc` generato — nessuna sorgente pinnata,
nessuna receipt, nessuna leaf, nessun record congelato.

**Il limite è strutturale e va detto**: nessuna guardia in-process può impedirlo, perché il `.pyc`
del modulo viene scritto **prima** che la guardia esista. L'invocazione supportata è come script
(`python test_….py`), per cui CPython non scrive alcun `.pyc`; se la suite viene invece importata
come modulo, l'unica difesa è `PYTHONDONTWRITEBYTECODE` nell'ambiente. È scritto in
`limits_of_the_claim` dentro rev3, accanto a `scope_of_the_hermeticity_claim`.

Dopo la ri-baseline il digest è **invariante su tre invocazioni consecutive come script**, con
`+0 −0 ~0` ogni volta.

### Il digest è auto-riferito, quindi lo pubblico escludendo sé stesso

Un secondo dettaglio emerso verificando: rev3 **vive sotto la root che impronta**, quindi qualunque
digest scritto dentro rev3 si invalida nell'atto stesso di scriverlo. Il numero pubblicato è perciò
calcolato **escludendo il record rev3**, ed è riproducibile:

```
sha256 sulle righe "relpath:sha256\n", ordinate per relpath,
per ogni file regolare sotto la validation root, __pycache__ INCLUSO,
escluso v26c_j14_dagger_dataset_authorization_rev3.json

630 file -> 8863d50af54fbbded2f3c2c91992dee4a3139ab0e0f9986d77d356057c8431d0
```

E vale la pena essere precisi su cosa l'ermeticità afferma davvero: **non un valore assoluto di
digest, ma l'invarianza** — il digest prima è uguale al digest dopo, con zero file aggiunti, rimossi
o cambiati. Quella proprietà è verificata su tre run consecutivi ed è indipendente da quale sia il
valore.

### Rigenerazione di rev3 prima della consegna

Il blocco `verification` della prima stesura di rev3 citava un digest a 630 file misurato **prima**
che rev3 stesso esistesse, quindi non riproducibile dal tuo audit — ed è esattamente il tipo di cosa
che farebbe scattare un NO-GO. Ho **rigenerato rev3** prima di consegnarlo, con il digest corrente,
l'invarianza su tre run, l'auto-esclusione e la cronologia completa dichiarata in
`digest_history_disclosed`. Il record non era ancora uscito dalle mie mani e non è mai stato oggetto
di audit; nessun record già consegnato è stato toccato.

---

## 6. Catena delle revisioni

Nessun record precedente modificato. La preregistrazione in uso resta **rev1**.

| record | ruolo | SHA-256 |
|---|---|---|
| `v26c_j14_prereg_dagger_dataset.json` | rev0, preservato, INERTE | `4c0720bad952f97c9cb5d68f3bd567f6cf9d9bb9b02508b42d8522268cba8717` |
| `v26c_j14_dagger_dataset_authorization.json` | rev0, preservato, INERTE | `22d374e549e736b91f4b332042c53fc05ba6be20d64022634ec2d4de77fcbd25` |
| `v26c_j14_prereg_dagger_dataset_rev1.json` | **preregistrazione in uso** | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` |
| `v26c_j14_dagger_dataset_authorization_rev1.json` | rev1, preservato, INERTE | `4341e58e32e2d43d779ae8ae1dbe562511c4b4459a94330ff76c1519c9b1f1b6` |
| `v26c_j14_dagger_dataset_authorization_rev2.json` | rev2, preservato, INERTE | `5ae9dc08b1a5dfee66db0305ce5d56d5d142ed51ac5482f2fee5efe583a9a2be` |
| **`v26c_j14_dagger_dataset_authorization_rev3.json`** | **in uso**, `execution_permitted_now: false` | `46bbaa262fa0aeea9b3b09608196556f68ac602bdc9b265df4e344d41159dc2f` |

rev3 pinna byte-identico rev2 che supersede e registra tutti e quattro i record superseduti.
**rev2 è ora inutilizzabile**: pinna il test a `4833a559765c6197…`, hash che non esiste più — chi
agisse su rev2 fallirebbe chiuso al primo controllo.

### Bundle in uso

| File | SHA-256 | variato in rev3 |
|---|---|---|
| `v26c_j14_prereg_dagger_dataset_rev1.json` | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` | no |
| `v26c_j14_dagger_dataset.py` | `fe7c4951c195224b8006655b831549b4b613cfd80a346b7174c58962263e2e0c` | **no** |
| `test_v26c_j14_dagger_dataset.py` | `6245a9ccee244a4d24deb02117a168a57fd45999485ba496b6d1daca15e5e52a` | **sì** |
| `v26c_j14_dagger_dataset_authorization_rev3.json` | `46bbaa262fa0aeea9b3b09608196556f68ac602bdc9b265df4e344d41159dc2f` | nuovo |

---

## 7. Una nota di trasparenza sul `__pycache__` esistente

I 50 `.pyc` già presenti sotto la validation root sono **preesistenti**, scritti da esecuzioni
anteriori sotto la vecchia esclusione. Non li ho rimossi: cancellarli sarebbe stata una write nella
validation root, cioè esattamente ciò che questa correzione esiste per impedire, e avrebbe alterato
il conteggio che il tuo audit ha verificato indipendentemente. Ora sono **dentro** l'insieme
sorvegliato e devono restare invariati — e lo sono.

Conseguenza da dichiarare: `test_v26c_j14_dagger_dataset.cpython-310.pyc` su disco è ora **stantìo**
rispetto al nuovo sorgente del test. È innocuo — il test è eseguito come `__main__` e mai importato
come modulo, quindi quel `.pyc` non viene né letto né riscritto — ma lo segnalo invece di lasciarlo
implicito. È stato rinfrescato dalla mia verifica descritta in §5, non dalla suite.

---

## 8. Invariato rispetto a rev2

854 righe (E 500 + F 354), repeat 1, nessun troncamento, primo mismatch E=95 / F=90, post 406+265,
671 righe che un troncamento avrebbe scartato, etichette teacher same-step, clock a zero esatto,
provenienza per riga, semi 126–128 sigillati e mai letti, il flag come diagnostico con il
controesempio della cella D, l'assunzione dell'etichetta open-loop dichiarata, l'aritmetica futura
`16713 + 8×(500+500) + 854 = 25567` verificata dai file, iperparametri identici a J11, e tutti i
TODO propagati. I test di materializzazione restano non-authoritative sotto `tempfile` con
`OUTPUT_ROOT_OVERRIDE`.

---

## 9. STOP

**Nessuna materializzazione. Nessun fit, rollout, critic o PPO. Seed 126–128 non aperti.**
L'authorization rev3 dichiara `execution_permitted_now: false`.

**Fermo in attesa del tuo audit.**
