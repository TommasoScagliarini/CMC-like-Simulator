# V26C — J20 R3: materializzazione del GO operativo dell'architetto

**Data:** 2026-08-28
**Stadio:** `V26C_J20_RESTORE_AUDIT_R3`
**Fase:** **APPROVAZIONE DEL GO — NIENTE È STATO ESEGUITO**
**Foglia di destinazione:** `…/j20_runs/j20_restore_audit_v26c_2026-08-28_r3/` — **assente**
**G9:** **resta aperto**, `training_ready: false`, `promotion: NONE`

---

## 1. Il GO operativo

| | |
|---|---|
| **file** | `v26c_j20_restore_audit_r3_architect_go.json` |
| **sha256** | **`fe090682f046a1ba74a3cc4f7d8c6b319c243920a375b8c19265e107bb9ca583`** |
| `kind` | `ARCHITECT GO - OPERATIONAL` |
| `status` | **`APPROVED`** — stringa esatta |
| `stage` | **`V26C_J20_RESTORE_AUDIT_R3`** — coincide con `GO_REQUIRED_STAGE` |
| `authorises_execution` | **`true`** |
| `authorises_retry` | `false` |
| `authorises_ppo` | `false` |
| `authorises_ex_novo` | `false` |
| `authorises_promotion` | `false` |
| `authorises_training` | `false` |
| `authorises_rewriting_the_warmup_leaf` | `false` |
| **pin** | **92**, copiati **verbatim** dal DRAFT rev7 |

## 2. Risultato della validazione

Eseguita importando il runner e chiamando `validate_go`. **Nessun comando con
`--execute`. Nessun `ray`, nessun `Algorithm`, nessun environment, nessun child,
nessun training, sampling o rollout.**

```
validate_go        valid=True, problems=0
pin labels         required=92 supplied=92
pins identical to the rev7 DRAFT:  True
pins_copied_from file / sha match: True / True
destination leaf   j20_restore_audit_v26c_2026-08-28_r3  exists=False
```

Controlli read-only aggiuntivi, tutti superati **prima** che il file venisse
scritto: nessun pin stale, nessun artefatto mancante, nessun pin fuori dalla
mappa chiusa del runner, nessuna delle 92 etichette richieste lasciata senza pin.

Il pacchetto non è stato perturbato dalla creazione del GO:

| comando | risultato |
|---|---|
| `test_v26c_j20_restore_audit_r3.py` | **344/344** |
| `v26c_j20_restore_audit_r3.py --preflight-only` | **READY, 92/92 pin** |

## 3. Provenienza dei pin

`pins_copied_from`:

- **file:** `v26c_j20_restore_audit_r3_go_DRAFT_rev7.json`
- **sha256:** `07a5bc4302f7618857511bd0105771de5025df8d6909ac87e8d652ad91d07943`

La mappa `pinned_artefacts_sha256` è stata **copiata verbatim**, non ricalcolata,
così che il GO autorizzi esattamente l'insieme di artefatti che l'architetto ha
rivisto. Ogni pin copiato è stato poi ri-hashato contro il disco come verifica di
attualità; il generatore rifiutava di scrivere il file in caso contrario. Il
DRAFT rev7 è stato verificato all'hash citato dall'architetto **prima** di
leggerne i pin.

## 4. Approvazione indipendente dell'architetto, registrata

Campo `architect_verification`:

- `independent: true`, data **2026-08-28**
- **rev7: `ACCEPTED`**
- suite: `test_v26c_j20_restore_audit_r3.py`, **344/344 check passati**
- preflight: `--preflight-only`, **READY, 92/92 pin**
- gli hash registrati **coincidono**

## 5. Autorizzazione esplicita dell'utente, verbatim

Campo `user_authorisation`, data **2026-08-28**, due dichiarazioni registrate
alla lettera:

1. `"ok procedi a risolvere per essere training ready"`
2. `"fermati prima di lanciare un training"`

**Interpretazione: NARROW.** Le due dichiarazioni si leggono insieme e la seconda
limita la prima.

### Cosa autorizza

- **esattamente una** esecuzione di `v26c_j20_restore_audit_r3.py --execute`
- **un solo** processo child validation-only
- il percorso di produzione reale `train_ppo_mlp.run` → `algo.restore_from_path`
- **zero iterazioni**: `iteration_start` è derivato come
  `restored_logical_iteration + 1 = 2` contro un target di 1, quindi
  `range(2, 2)` è vuoto
- **nessun retry**, qualunque sia l'esito

### Cosa NON autorizza

- qualsiasi training, qualsiasi passo di gradiente, qualsiasi `algo.train`
- qualsiasi sampling e qualsiasi rollout
- PPO, run ex-novo, qualsiasi promozione
- la **training readiness** in sé
- l'attestazione di aggregazione delle evidenze — che **non esiste** e che questo
  GO non progetta, non preregistra e non approva
- che quell'attestazione, se mai verrà costruita, lanci un training
- una seconda esecuzione di questo stadio, o un R4

La prima dichiarazione chiede di lavorare **verso** la training readiness. Questo
GO è **un passo** su quel percorso: chiude il gate G9 e nient'altro. Un R3 che
passa registra comunque `promotion: NONE`, `training_ready: false`,
`next_stage_authorized: false`. La seconda dichiarazione è onorata esattamente:
la run si ferma prima di qualsiasi addestramento.

## 6. Entrambi i DRAFT preservati

| file | sha256 | stato |
|---|---|---|
| `v26c_j20_restore_audit_r3_go_DRAFT.json` (rev6) | `dce5a521fb21a5e192d341aff0fbc977da87cf71b8a3b0e81bc522399fe2d908` | storico, superato, inerte, **rifiutato** da `validate_go` |
| `v26c_j20_restore_audit_r3_go_DRAFT_rev7.json` | `07a5bc4302f7618857511bd0105771de5025df8d6909ac87e8d652ad91d07943` | preservato, inerte, **rifiutato** da `validate_go` |

Nessuno dei due è stato modificato o cancellato. Il GO operativo è un **file
separato** e non li sostituisce.

## 7. File

- **Creato:** `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j20_restore_audit_r3_architect_go.json`
- **Creato:** questo report.

Nessun altro file è stato creato o modificato. rev6, rev7, i due DRAFT, il
wrapper congelato, R1, R2, il checkpoint e ogni file di produzione restano
invariati.

## 8. Stato

Entrambe le condizioni richieste da rev5, portate avanti da rev6 e rev7, sono ora
soddisfatte: **autorizzazione esplicita dell'utente** e **GO dell'architetto con
`status` esattamente `APPROVED`**.

**Il GO è materializzato e valido, ma NON è stato usato.** `v26c_j20_restore_audit_r3.py --execute` **non è stato lanciato**, la foglia di
destinazione è assente, e nulla è stato addestrato, campionato o promosso.
**G9 resta aperto.**
