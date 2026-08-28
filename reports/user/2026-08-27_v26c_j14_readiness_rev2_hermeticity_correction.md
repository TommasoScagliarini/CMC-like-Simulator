# V26C J14 — Correzione di ermeticità rev2 dopo audit Codex

**Data**: 2026-08-27
**Stadio**: `V26C_J14_POST_MISMATCH_DAGGER_DATASET`
**Revisione**: **rev2**, superseding rev1
**Stato**: bundle corretto, in attesa di un nuovo audit. **Leaf non materializzato.**
**Autorizzazione**: `execution_permitted_now: false`.
**Nessuna materializzazione, fit, rollout, seed 126–128, critic o PPO.**

---

## 1. Il rilievo

Il NO-GO è rimasto attivo, e il motivo era giusto: **il test suite rev1 non era ermetico**. Tre
test negativi scrivevano e ripristinavano **artefatti congelati reali** sotto la validation root:

| righe rev1 | cosa faceva |
|---|---|
| ~95–103 | scriveva byte nel **vero** `SUPERSEDED_PREREG` (la preregistrazione rev0 congelata) |
| ~427–433 | creava e rimuoveva `TECHNICAL_INVALID` dentro la **vera** leaf J11 |
| ~449–464 | creava e rimuoveva un marker e un lock accanto alla **vera** leaf J12 |

Il fatto che un `finally` ripristinasse non basta, ed è il punto centrale del rilievo: **una suite
interrotta** — Ctrl-C, OOM, kill — lascerebbe la preregistrazione congelata corrotta, o una leaf
sorgente marcata invalida. Un artefatto immutabile che passa anche solo per un istante da uno stato
alterato non è più un artefatto immutabile.

Rilievo accettato senza riserve.

---

## 2. La correzione

**Ambito: ermeticità del test soltanto.** Il **runner non è cambiato**: è byte-identico a rev1
(`fe7c4951c195224b…`). Il difetto era interamente nel suite.

### Ogni test negativo ora opera su un mirror temporaneo

- `build_mirror()` copia **ogni sorgente pinnata** e **entrambe le preregistrazioni** dentro la
  temp root.
- `use_mirror()` fa monkeypatch delle costanti di path del modulo — `HERE`, `PREREG`,
  `SUPERSEDED_PREREG`, i cinque leaf, `J11_AGGREGATE`, `J2_MODULE` — e al termine ripristina
  **solo i riferimenti Python**, senza toccare alcun file.
- Le tre trace da 51 MB sono **symlink**, perché nessun test le manomette; tutto ciò che un test
  negativo può toccare è una **copia reale e indipendente**.
- `reset_mirror()` ricostruisce il mirror fra un test e l'altro, e scrive esclusivamente nella
  temp root.

Dopo ogni test negativo il suite **verifica sull'artefatto reale** che non sia stato toccato: la
rev0 è ancora al suo hash, la leaf J11 non ha marker, la leaf J12 non ha né marker né lock.

### Il guard fail-closed

`HermeticGuard` avvolge **l'intera suite** e intercetta ogni primitiva di scrittura diretta sotto
la validation root, sollevando invece di toccare il file:

- `Path.write_text`, `write_bytes`, `mkdir`, `unlink`, `rmdir`, `touch`, `rename`, `replace`,
  `symlink_to`, `hardlink_to`, `chmod`;
- `os.rename`, `replace`, `remove`, `unlink`, `mkdir`, `makedirs`, `rmdir`, `truncate`;
- `os.open` con flag di scrittura, e `open()` in modo `w/a/x/+`;
- `shutil.rmtree`, `copyfile`, `copy2`, `move`;
- `np.savez`, `np.savez_compressed`.

Solo la temp root resta scrivibile. Ogni tentativo bloccato è registrato in `ATTEMPTS`.

### Il guard è provato prima di essere creduto

Dieci probe, una per primitiva, **ciascuna riproduce una scrittura che il suite rev1 eseguiva
davvero**: byte nella rev0 congelata, marker nella leaf J11, lock accanto alla J12, unlink di una
sorgente pinnata, `open(..., 'w')`, `os.rename`, `rmtree` di una leaf, `mkdir`, `savez`, `os.open`.
Tutte e dieci vengono bloccate e registrate; il suite verifica poi che **nessuna abbia lasciato
traccia su disco** e che la temp root sia rimasta pienamente scrivibile.

### Un falso positivo reale trovato dal guard

Alla prima esecuzione il guard ha bloccato `shutil.rmtree` sul mirror — cioè dentro la temp root,
dove è lecito. Causa: `rmtree` scende con `os.unlink(nome, dir_fd=...)`, passando un **nome nudo**
relativo a un descrittore di directory; risolverlo contro la cwd lo faceva apparire sotto la
validation root.

Corretto: una chiamata con `dir_fd` viene delegata senza controllo, **perché il punto d'ingresso
`shutil.rmtree` è già guardato** e la discesa fd-relative è quindi coperta lì. Il guard non è stato
indebolito: è stato reso corretto.

---

## 3. Verifica esterna dell'ermeticità

Non mi sono affidato all'asserzione interna del suite. Ho hashato la validation root file per file
**prima e dopo** un'esecuzione completa, dall'esterno:

```
file sotto la validation root: 579
file aggiunti : NESSUNO
file rimossi  : NESSUNO
file cambiati : NESSUNO
```

**La validation root è byte-identica prima e dopo la suite.**

---

## 4. Governance: catena delle revisioni

Nessun record precedente è stato modificato. La preregistrazione in uso resta **rev1**, che questa
revisione non tocca.

| record | ruolo | SHA-256 |
|---|---|---|
| `v26c_j14_prereg_dagger_dataset.json` | rev0, preservato, INERTE | `4c0720bad952f97c9cb5d68f3bd567f6cf9d9bb9b02508b42d8522268cba8717` |
| `v26c_j14_dagger_dataset_authorization.json` | rev0, preservato, INERTE | `22d374e549e736b91f4b332042c53fc05ba6be20d64022634ec2d4de77fcbd25` |
| `v26c_j14_prereg_dagger_dataset_rev1.json` | **preregistrazione in uso** | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` |
| `v26c_j14_dagger_dataset_authorization_rev1.json` | rev1, preservato, INERTE | `4341e58e32e2d43d779ae8ae1dbe562511c4b4459a94330ff76c1519c9b1f1b6` |
| **`v26c_j14_dagger_dataset_authorization_rev2.json`** | **in uso**, `execution_permitted_now: false` | `5ae9dc08b1a5dfee66db0305ce5d56d5d142ed51ac5482f2fee5efe583a9a2be` |

rev2 pinna byte-identico l'authorization rev1 che supersede, e registra tutti e tre i record
superseduti in `preserved_superseded_records` — verificati integri.

**L'authorization rev1 è ora inutilizzabile**: pinna un test il cui hash non esiste più
(`a2e8c84daa43930b…` → `4833a559765c6197…`).

---

## 5. Evidenze e hash

### Bundle in uso (rev2)

| File | SHA-256 | variato in rev2 |
|---|---|---|
| `v26c_j14_prereg_dagger_dataset_rev1.json` | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` | no |
| `v26c_j14_dagger_dataset.py` | `fe7c4951c195224b8006655b831549b4b613cfd80a346b7174c58962263e2e0c` | **no** |
| `test_v26c_j14_dagger_dataset.py` | `4833a559765c619773895d2aca2f2fde0e43a3d6082141fbe8e0c091d9802191` | **sì** |
| `v26c_j14_dagger_dataset_authorization_rev2.json` | `5ae9dc08b1a5dfee66db0305ce5d56d5d142ed51ac5482f2fee5efe583a9a2be` | nuovo |

### Verifiche eseguite

- `py_compile` su runner e test: **OK**.
- Suite completa: **216 check, PASS** (rev0 169 → rev1 190 → rev2 216).
- Preflight inerte: **GO**, nessun blocker, 21 sorgenti pinnate, tre leaf validati
  (J12 `FAIL`, J10R1 `PASS`, J11 `PASS`), torch e ray assenti.
- **21/21 fonti congelate verificate** per hash su disco.
- **24/24 pin dell'authorization rev2 verificati.**
- **3/3 record superseduti** verificati byte-identici.
- **Ermeticità verificata dall'esterno**: 579 file, zero aggiunti, rimossi o cambiati.
- Nessun `j14_runs`, **nessun lock**, **nessuno staging**, **nessuna sentinella**.
- `git status`: solo i tre file già dirty a inizio sessione.

---

## 6. Invariato rispetto a rev1

Il runner, la preregistrazione rev1, i 21 pin, le 854 righe (E 500 + F 354), repeat 1, nessun
troncamento, primo mismatch E=95 / F=90, le 671 righe che un troncamento avrebbe scartato, le
etichette teacher same-step, il clock a zero esatto, la provenienza per riga, i semi 126–128
sigillati, il flag come diagnostico con il controesempio della cella D, l'assunzione dell'etichetta
open-loop dichiarata, l'aritmetica 25567 verificata dai file, e tutti i TODO propagati.

I test di materializzazione restano **non-authoritative** sotto `tempfile` con
`OUTPUT_ROOT_OVERRIDE`, come prima.

---

## 7. STOP

**Nessuna materializzazione. Nessun fit, rollout, critic o PPO. Seed 126–128 non aperti.**
L'authorization rev2 dichiara `execution_permitted_now: false`.

**Fermo in attesa del tuo nuovo audit.**
