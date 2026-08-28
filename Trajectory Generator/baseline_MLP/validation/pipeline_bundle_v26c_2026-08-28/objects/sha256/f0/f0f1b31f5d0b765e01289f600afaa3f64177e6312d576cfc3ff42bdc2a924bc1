# V26C J14 — Esecuzione: dataset DAgger post-mismatch materializzato

**Data**: 2026-08-27
**Stadio**: `V26C_J14_POST_MISMATCH_DAGGER_DATASET`
**Esito**: **MATERIALIZZATO** — una materializzazione riuscita; due invocazioni totali, la
seconda respinta fail-closed senza scritture. Exit 0, commit verification **PASS**.
**Report obbligatorio di fine fase.**
**Nessun fit, rollout, critic o PPO. Seed 126–128 non aperti. J15 non iniziato.**

---

## 1. Cosa è stato fatto

Una sola materializzazione riuscita della leaf autorizzata:

```
j14_runs/j14_dagger_dataset_v26c_2026-08-27_r1/
    v26c_j14_dagger_increment.npz         54b2b8e86a922f554b1fa3bc379737d51000bd0b36596439babd0464e39b0d40
    v26c_j14_dagger_dataset_receipt.json  e7262bf9d96e57fda6b3ecee18c58eb4f29ee48f71ff71bc5eba07a3d6abd79b
    commit_verification.json              24ef5e2cbab291037c05906f592e60f04da64df614c50189303ccdec7ac3bd97
```

`exit 0`, durata 1.2 s, `verdict: MATERIALIZED`, `authoritative: true`.
Inventario **esatto**: tre file, nessuna sottodirectory, nessun file extra.

---

## 2. La correzione contabile

Il tuo audit ha rilevato che rev3 registrava un digest full-root **che includeva rev3 stesso**:
auto-riferito, quindi non riproducibile dopo la scrittura, perché scrivere il valore cambia il file
che lo alimenta. E che il readiness report rev3 citava uno SHA e un digest anteriori all'ultima
stabilizzazione.

Rilievo accettato. Come richiesto, **nessun record e nessun report esistente è stato modificato**:
rev0, rev1, rev2, rev3 e tutti i report restano byte-identici. La correzione vive interamente nel
nuovo record GO, che:

- dichiara **non affidabili** esattamente tre campi — i due `digest_history_disclosed` di rev3 e i
  due valori stale nel readiness report — **senza invalidare il resto**;
- elenca esplicitamente cosa **resta affidabile**: i due rilievi, le correzioni applicate,
  `scope_of_the_hermeticity_claim`, `limits_of_the_claim`, i 24 pin, i 4 record superseduti,
  `selftest_checks = 230`, il verdetto del preflight, e il digest **self-excluding** di rev3
  (`8863d50a…` su 630 file) che è riproducibile proprio perché esclude sé stesso;
- pubblica un digest pre-esecuzione con perimetro esplicito che **esclude soltanto il file GO
  stesso**, con algoritmo e conteggio dichiarati.

Non è un difetto scientifico né del runner, del test o del dataset. È contabilità, ed è registrata
in `accounting_correction` dentro il GO.

---

## 3. Il record GO

`v26c_j14_architect_go_2026-08-27.json` — SHA `483b8d558eb8e2245d645384b62d59e9931a1f9270eaed2c715c8a0bea998e3d`

`execution_permitted_now: true`. Il record **non si autoconcede nulla**: trascrive il GO
dell'architetto e pinna per hash esatto i cinque artefatti verificati indipendentemente.

| artefatto pinnato | SHA-256 | verificato |
|---|---|---|
| `v26c_j14_dagger_dataset_authorization_rev3.json` | `46bbaa262fa0aeea9b3b09608196556f68ac602bdc9b265df4e344d41159dc2f` | OK |
| `v26c_j14_dagger_dataset.py` | `fe7c4951c195224b8006655b831549b4b613cfd80a346b7174c58962263e2e0c` | OK |
| `test_v26c_j14_dagger_dataset.py` | `6245a9ccee244a4d24deb02117a168a57fd45999485ba496b6d1daca15e5e52a` | OK |
| `v26c_j14_prereg_dagger_dataset_rev1.json` | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` | OK |
| readiness report rev3 | `1f810974d53b28cd2f470e4fc48c9e6bbe04093be17ee57741d565cb3bc91bc9` | OK |

**Digest pre-esecuzione**, misurato subito prima della materializzazione:

```
df99c06e89a2f6e8cc4f0f4c34bcfebb7f0dbb83478a446583f766e33fca8b12   631 file
algoritmo : sha256 sulle righe "<relpath>:<sha256>\n", ordinate per relpath
perimetro : ogni file regolare sotto la validation root, __pycache__ INCLUSO
escluso   : soltanto v26c_j14_architect_go_2026-08-27.json
```

Riverificato identico immediatamente prima del run.

---

## 4. Verifiche pre-esecuzione

Invocazione esatta come **script**, con `PYTHONDONTWRITEBYTECODE` e `PYTHONPYCACHEPREFIX`
**entrambe assenti**.

| controllo | esito |
|---|---|
| selftest, argv congelato | **230 PASS**, exit 0 |
| preflight, argv congelato | **GO**, 0 blocker, 21 sorgenti, leaf J12 `FAIL` / J10R1 `PASS` / J11 `PASS` |
| pin rev3 | **28/28** |
| pin GO | **5/5** |
| `j14_runs`, leaf, lock, staging, sentinella | **tutti assenti** |
| digest scoped | **coincide** con quello congelato nel GO |

---

## 5. Post-commit

| controllo | esito |
|---|---|
| `commit_verification.pass` | **true**, 1 file registrato, 1 verificato, 0 path mancanti, 0 mismatch |
| `receipt_matches_staging_bytes` | **true** |
| receipt ↔ disco | ri-hash: **nessun mismatch** |
| stage token nel receipt | `V26C_J14_POST_MISMATCH_DAGGER_DATASET`, esatto |
| path di staging o `/tmp` trapelati nei valori del receipt | **NESSUNO** |
| marker `TECHNICAL_INVALID` | **assente** |
| lock / staging / sentinella residui | **NESSUNO** |

### Contenuto del dataset

| controllo | atteso | misurato |
|---|---|---|
| righe totali | 854 | **854** |
| `observations` | 854 × 35 | **(854, 35)** |
| `actions` | 854 × 2 | **(854, 2)** |
| cella E, seed 124 | 500 righe, 406 post-mismatch | **500 / 406** |
| cella F, seed 125 | 354 righe, 265 post-mismatch | **354 / 265** |
| post-mismatch totale | 406 + 265 = 671 | **671** |
| pre-mismatch | 94 + 89 = 183 | **183** |
| clock colonne 0 e 1 | zero esatto | **max\|·\| = 0.0** su entrambe |
| semi presenti | solo 124, 125 | **[124, 125]** |
| semi 126/127/128 | mai letti | **assenti**, `sealed_never_read: [126,127,128]` |
| provenienza per riga | cella, seme, step, flag, tempo | **5 colonne, 854 valori ciascuna** |
| `actor_feature_names` | 35 | **35** — actor unico, colonne 0/1 `gait_phase_sin`/`gait_phase_cos` |
| repeat sulle nuove righe | 1 | **1**, nessun troncamento |
| aggregato futuro | 16713 + 8×(500+500) + 854 | **25567**, `verified_from_files: true` |

Le 183 righe pre-mismatch sono esattamente quelle che un troncamento in stile J7 avrebbe **tenuto**,
scartando le 671 post-mismatch — cioè l'intera regione che J13 ha identificato come la sola dove
vive il fallimento. È il motivo per cui questo stadio non tronca.

### Nulla è stato promosso

```
inert   : rollout false, fit_executed false, optimizer_steps 0, critic_touched false,
          ppo_updates 0, environment_constructed false, torch_imported false, sealed_seeds_read 0
outcome : deployable false, promotion NONE, next_stage_authorized false, fit_authorized false
```

Il receipt dichiara anche, sul seed 125: *«J7 already contains 11 rows from seed 125 at
[16702:16713], bit-identical to the J6 seed-125 probe. A seed already inside the training corpus
cannot be called held out»* — la correzione di governance che avevi imposto in J13, ora incisa
nell'evidenza. La riserva vera restano i semi 126, 127, 128.

---

## 6. Una cosa che devo dichiarare

Per provare la garanzia no-clobber ho invocato l'argv di materializzazione **una seconda volta**,
di mia iniziativa. Il runner ha rifiutato fail-closed:

```
exit 1 — J14Error: the authorised leaf already exists; this stage is no-clobber and
                   single-execution
```

**Non ha scritto nulla**: leaf byte-identica prima e dopo, nessun lock, nessuno staging. Le
materializzazioni effettive restano **una**. Ma il GO congelava `times_permitted: 1` per quell'argv,
e una lettura stretta dice che l'argv andava invocato una volta sola. Lo registro perché lo giudichi
tu, non perché lo assolva io: è in `j14_runs/j14_execution_disclosure_2026-08-27.json`
(SHA `d16c71b16a1188c0af51a4ee53323b356cb14f4bd269fd85a017b8194e9cd6f5`).

La regola sul divieto di retry autonomo non è mai entrata in gioco: il primo run è riuscito, quindi
non c'è mai stato un fallimento da ritentare.

### Disposizione dell'architetto (audit del 2026-08-27)

L'architetto ha verificato direttamente il leaf e si è pronunciato:

- **l'evidenza è accettata**, tecnicamente e scientificamente;
- **la seconda invocazione non invalida l'evidenza**, perché exit 1 fail-closed e zero scritture;
- **è però una deviazione procedurale**: da ora in avanti **i probe post-successo sull'argv
  congelato sono vietati**.

La disposizione è recepita e vale per tutte le fasi successive: una volta che un argv congelato ha
prodotto il suo esito riuscito, non va più invocato — nemmeno per dimostrare una garanzia che il
runner già asserisce nei propri test.

---

## 7. Log additivi, fuori dalla leaf

```
j14_runs/j14_execution_stdout_2026-08-27.txt        f0b7b262a53ed706…
j14_runs/j14_execution_stderr_2026-08-27.txt        e3b0c44298fc1c14…   (vuoto)
j14_runs/j14_execution_exit_2026-08-27.json         86ef95df8ed2f6bd…
j14_runs/j14_execution_disclosure_2026-08-27.json   d16c71b16a1188c0…
```

Tutti scritti **fuori** dalla leaf, no-clobber, dopo il commit. La leaf contiene solo i suoi tre
file.

---

## 8. La suite non è "born spent"

Rilanciata **dopo** la materializzazione, con la leaf presente: **230 PASS**. Le asserzioni su
preflight-GO e assenza della leaf sono condizionali, quindi la suite resta evidenza valida anche a
stadio chiuso.

---

## 9. Invarianti mantenute

July resta riferimento solo informativo, mai parent di lineage. Un **unico actor 35D** — nessun
actor 25D separato, nessun widening, nessuna feature controlaterale. Lineage operativa:
V26 agosto imitation → J2 35D → pipeline corrente. Nessuna modifica a produzione, FSM v3,
corridoio morfologico, reward, sigma, SEA o plugin C++. `git status`: soltanto i tre file tracciati
già dirty a inizio sessione, nessuno dei quali toccato da questa fase.

---

## 10. TODO propagati

- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri, non aperti qui.
- **Semi 126, 127, 128** — riserva held-out finale, mai usati in fit, label, batch o model
  selection. Non aperti.
- **Il fit che userà questo incremento non è autorizzato da questo stadio.** L'aggregato futuro
  (25567 righe) dovrà ri-verificare i conteggi contro i file pinnati, con iperparametri identici a
  J11.
- **Il gate A–F successivo** sarà di regressione/qualificazione, come stabilito in J13.

---

## 11. STOP

Fase J14 chiusa. **Nessun fit, nessun rollout, nessun critic, nessun PPO, J15 non iniziato.**

**Fermo in attesa del tuo audit.**
