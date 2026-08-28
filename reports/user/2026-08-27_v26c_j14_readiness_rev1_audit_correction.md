# V26C J14 — Correzione di readiness rev1 dopo audit Codex

**Data**: 2026-08-27
**Stadio**: `V26C_J14_POST_MISMATCH_DAGGER_DATASET`
**Revisione**: **rev1**, superseding
**Stato**: bundle corretto, in attesa di un nuovo audit. **Leaf non materializzato.**
**Autorizzazione**: `execution_permitted_now: false`.
**Nessuna materializzazione, fit, rollout, seed 126–128, critic o PPO.**

---

## 1. Il rilievo dell'audit

L'audit Codex ha restituito **NO-GO** su una sola lacuna, ed era reale:

> `coverage_report()` legge `j11_runs/j11_multistart_fit_v26c_2026-08-27_r1/v26c_j11_aggregate_dataset.npz`,
> ma quel file (e il leaf J11 da cui proviene) non è incluso in `PIN_SOURCES` né verificato da
> `verify_sources`.

**Il file veniva letto e mai pinnato.** L'inviluppo di training contro cui l'intero report di
copertura è misurato — `phase_swing_elapsed_norm` max 0.188846, `pros_knee_angle` min −1.009047, e
quindi i conteggi 211 e 195 di righe oltre l'inviluppo — proveniva da un artefatto la cui integrità
non era verificata da nessuna parte. Nel bundle rev0 il conteggio pin era 18; l'aggregato J11 non
era fra questi.

Il rilievo è accettato senza riserve. Non era un dettaglio formale: era una sorgente di misura fuori
dalla catena di provenance.

---

## 2. Cosa è cambiato in rev1

**Ambito: solo provenance.** Nessuna scienza, matrice, cella, seme, regola di etichettatura,
politica di troncamento, soglia o conteggio è cambiata.

| | rev0 | rev1 |
|---|---|---|
| sorgenti pinnate | 18 | **21** |
| pin totali dell'authorization | 21 | **24** |
| leaf sorgente validati | J12, J10R1 | J12, J10R1, **J11** |
| lettura dell'aggregato J11 | path costruito inline, nessun controllo | costante pinnata `J11_AGGREGATE`, **ri-hash al momento della lettura** |
| check | 169 | **190** |

**I tre pin aggiunti:**

| file | SHA-256 |
|---|---|
| `j11_runs/…/v26c_j11_aggregate_dataset.npz` | `a936c580c4db19383255d3d5f7346560e0fd99dc4b139ac202858e82cca13f42` |
| `j11_runs/…/v26c_j11_multistart_fit_receipt.json` | `39228c5cf00a753f1d57f07d4794ac2996401e1b40587cf1ec1e5f5e2b0ae65f` |
| `j11_runs/…/commit_verification.json` | `1d24eed05dd04c187c014181c89a0611a648e5601cacff5e1ae0221adcfb8643` |

**Il leaf J11 è ora validato come gli altri**: `commit_verification.pass = true`, nessun marker
`TECHNICAL_INVALID`, nessun lock o staging stantìo accanto, e **verdict PASS** — con una ragione
esplicita nel codice: un inviluppo preso da un fit non qualificato non sarebbe l'inviluppo di un
attore qualificato.

**`coverage_report` ri-hasha al momento della lettura**, non solo all'ingresso: se il file cambiasse
fra `verify_sources` e la lettura, la funzione fallirebbe chiusa.

---

## 3. Supersessione, senza toccare i record immutabili

Come richiesto, i record rev0 dichiarati immutable/frozen **non sono stati modificati**. Sono stati
creati due record superseding:

| record | stato | SHA-256 |
|---|---|---|
| `v26c_j14_prereg_dagger_dataset.json` | **rev0, preservato byte-identico, INERTE** | `4c0720bad952f97c9cb5d68f3bd567f6cf9d9bb9b02508b42d8522268cba8717` |
| `v26c_j14_dagger_dataset_authorization.json` | **rev0, preservato byte-identico, INERTE** | `22d374e549e736b91f4b332042c53fc05ba6be20d64022634ec2d4de77fcbd25` |
| `v26c_j14_prereg_dagger_dataset_rev1.json` | **in uso** | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` |
| `v26c_j14_dagger_dataset_authorization_rev1.json` | **in uso**, `execution_permitted_now: false` | `4341e58e32e2d43d779ae8ae1dbe562511c4b4459a94330ff76c1519c9b1f1b6` |

### Il record rev0 è ora inutilizzabile, e lo è verificabilmente

Non per dichiarazione, ma per costruzione. La sua tabella `frozen_inputs_sha256` pinna il runner e
il test con gli hash rev0, che non esistono più:

```
OK     v26c_j14_prereg_dagger_dataset.json  -> 4c0720bad952f97c   (preservato)
STALE  v26c_j14_dagger_dataset.py           -> fe7c4951c195224b   (era 42629c18…)
STALE  test_v26c_j14_dagger_dataset.py      -> a2e8c84daa43930b   (era f3914398…)
```

Chiunque agisse su quel record fallirebbe chiuso al primo controllo di hash. In più il runner ora:

- **punta a rev1** e rifiuta di partire se `PREREG` fosse il record superseduto;
- **asserisce che rev0 sia ancora presente e byte-identico**, rifiutando se fosse stato editato;
- **verifica che rev1 registri l'hash corretto** del record che supersede.

Tutti e tre i comportamenti sono coperti da test negativi.

---

## 4. Il tuo primo test e il rerun

Riporto il tuo esito come me l'hai comunicato: **la tua prima esecuzione del suite è fallita per una
race concorrente con la creazione del record**, e il **rerun successivo ha dato 169/169 PASS**.

Il meccanismo è coerente con il modo in cui il suite è costruito, e lo segnalo perché si ripresenterà:
il test apre con `snapshot(HERE)` e chiude asserendo `snapshot(HERE) == before_all`, cioè che **non
un byte sotto la directory di validazione sia cambiato durante l'esecuzione**. Se un file viene
creato o modificato lì mentre il suite gira — per esempio la scrittura di un record — quell'ultima
asserzione fallisce. Il fallimento era quindi un artefatto di concorrenza, non un difetto del
bundle, ed è esattamente ciò che quel controllo esiste per rilevare.

**Nota operativa**: il suite va eseguito quando nessun altro processo scrive sotto
`v26c_july_replica_2026-08-26/`. Non ho indebolito il controllo per accomodare la race: è la
garanzia più forte del suite.

Nel bundle rev1 il conteggio è salito a **190** con l'aggiunta dei test di supersessione e di
tamper.

---

## 5. Test aggiunti

**Sull'aggregato J11** (il rilievo dell'audit):
- il file è pinnato e verifica su disco;
- check **AST** che `coverage_report` carichi **solo** attraverso la costante `J11_AGGREGATE`, e
  che ri-hashi al momento della lettura;
- **tamper negativo**: alterando il pin, sia `coverage_report` sia `verify_sources` rifiutano, con
  il messaggio che nomina il file;
- il leaf J11 è validato (pass, nessun marker, verdict PASS), e un marker `TECHNICAL_INVALID`
  sintetico sul leaf J11 lo fa rifiutare.

**Sulla supersessione:**
- rev1 è la revisione in uso e il runner non punta al record superseduto;
- rev0 è preservato **byte-identico** e conteneva davvero **18** pin senza l'aggregato J11 — la
  lacuna, misurata;
- editare rev0 blocca lo stadio, con il messaggio che dice «was MODIFIED»;
- **puntare il runner a rev0 è rifiutato**: il suo hash non è il pin.

**Conteggi aggiornati**: 21 sorgenti pinnate, 24 pin nell'authorization, 190 check.

---

## 6. Verifiche eseguite

- `py_compile` su runner e test: **OK**.
- Suite completa: **190 check, PASS**.
- Preflight inerte: **GO**, nessun blocker, 21 sorgenti pinnate, tre leaf validati
  (J12 `FAIL` — che è il motivo per cui questo stadio esiste — J10R1 `PASS`, J11 `PASS`),
  854 righe, nessun modulo pesante introdotto, torch e ray assenti.
- **24/24 pin dell'authorization rev1 verificati su disco.**
- Staleness del record rev0 verificata: due dei tre input pinnati non corrispondono più.
- Nessun `j14_runs`, lock, staging o sentinella.
- `git status`: solo i tre file già dirty a inizio sessione.

---

## 7. Evidenze e hash

### Bundle in uso (rev1)

| File | SHA-256 |
|---|---|
| `v26c_j14_prereg_dagger_dataset_rev1.json` | `877cccc1bb93868a5f1050a7cf2cfc1f5776d18437e099996f059373a6cb36d5` |
| `v26c_j14_dagger_dataset.py` | `fe7c4951c195224b8006655b831549b4b613cfd80a346b7174c58962263e2e0c` |
| `test_v26c_j14_dagger_dataset.py` | `a2e8c84daa43930b9d2fe6aecd7d2751f6313abfc9ef1a6f40aca635532ff180` |
| `v26c_j14_dagger_dataset_authorization_rev1.json` | `4341e58e32e2d43d779ae8ae1dbe562511c4b4459a94330ff76c1519c9b1f1b6` |

### Bundle superseduto (rev0), preservato e inerte

| File | SHA-256 | stato |
|---|---|---|
| `v26c_j14_prereg_dagger_dataset.json` | `4c0720bad952f97c9cb5d68f3bd567f6cf9d9bb9b02508b42d8522268cba8717` | INERTE, non referenziato, guardato per byte-identità |
| `v26c_j14_dagger_dataset_authorization.json` | `22d374e549e736b91f4b332042c53fc05ba6be20d64022634ec2d4de77fcbd25` | INERTE, non autorizzato, **inutilizzabile** |

**File modificati**: `v26c_j14_dagger_dataset.py` e `test_v26c_j14_dagger_dataset.py` (non erano
dichiarati immutabili). **File creati**: i due record rev1 e questo report.
**Nessun artefatto J0–J13, nessun report esistente, nessuna sorgente July, nessuna configurazione di
produzione toccati.** Il report J13 e la readiness J14 rev0 restano invariati.

### Il comando futuro, invariato salvo la revisione dei record

```
cwd:         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter: /opt/anaconda3/envs/envCMC-rllib/bin/python
argv:        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j14_dagger_dataset.py
             --materialize
             --authorized-stage V26C_J14_POST_MISMATCH_DAGGER_DATASET
             --out <repo>/…/j14_runs/j14_dagger_dataset_v26c_2026-08-27_r1
```

---

## 8. Invariato rispetto a rev0

Tutto il resto: le 854 righe (E 500 + F 354), repeat 1, nessun troncamento, primo mismatch E=95 /
F=90 su `online_left_in_contact`, 671 righe che un troncamento avrebbe scartato, etichette teacher
same-step con griglia temporale bit-identica, clock a zero esatto, provenienza per riga, semi
126–128 sigillati e mai letti, il flag come diagnostico con il controesempio della cella D,
l'assunzione dell'etichetta open-loop dichiarata, l'aritmetica futura 25567 verificata dai file, e
tutti i TODO propagati.

---

## 9. STOP

**Nessuna materializzazione. Nessun fit, rollout, critic o PPO. Seed 126–128 non aperti.**
L'authorization rev1 dichiara `execution_permitted_now: false`.

**Fermo in attesa del tuo nuovo audit.**
