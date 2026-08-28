# V26C J15 — Esecuzione: il fit è FALLITO, fail-closed

**Data**: 2026-08-27
**Stadio**: `V26C_J15_FRESH_35D_POST_MISMATCH_REFIT`
**Esito**: **FALLITO**, `exit 1` dopo **0.062 s**. Nessuna leaf creata, nessun peso prodotto.
**Esecuzioni**: una, e una sola. **Nessun retry, nessuna riparazione, nessun secondo fit.**
**Report obbligatorio di fine fase.**

---

## 1. Cosa è successo

Il fit autorizzato è stato lanciato una volta con l'argv congelato ed è morto quasi subito, prima di
toccare torch, il dataset o qualunque path di scrittura:

```
Traceback (most recent call last):
  File ".../v26c_j15_fresh_refit.py", line 1616, in main
    print(json.dumps(commit(args.out, args.authorized_stage, progress=args.progress),
  File ".../v26c_j15_fresh_refit.py", line 1476, in commit
    fit = run_fit(progress=progress)
  File ".../v26c_j15_fresh_refit.py", line 622, in run_fit
    modules["warm_start.py"] = verify_warm_start_source()
NameError: name 'verify_warm_start_source' is not defined
```

**È un difetto mio, nel bundle di readiness che ho consegnato.** Non è un problema dei dati, del
parent, del gate o della scienza: il runner non era eseguibile lungo il proprio percorso principale.

---

## 2. Comando e ambiente esatti

```
cwd         : /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter : /opt/anaconda3/envs/envCMC-rllib/bin/python
argv        : Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j15_fresh_refit.py
              --fit
              --authorized-stage V26C_J15_FRESH_35D_POST_MISMATCH_REFIT
              --out  <repo>/…/j15_runs/j15_fresh_refit_v26c_2026-08-27_r1
environment : PYTHONDONTWRITEBYTECODE  ASSENTE   (rimossa e verificata assente)
              PYTHONPYCACHEPREFIX      ASSENTE   (rimossa e verificata assente)
exit        : 1
durata      : 0.062 s
```

L'argv non è stato alterato: è stato **letto** dal record GO rev2 e passato invariato. Prima del
lancio il driver ha riverificato l'hash di rev2, `execution_permitted_now`, `times_permitted == 1`,
tutti e cinque i pin di readiness e l'assenza della leaf.

### Prova dell'esecuzione unica

`j15_runs/j15_execution_exit_2026-08-27.json` registra `executions_performed: 1`,
`times_permitted: 1`. L'argv congelato **non è stato più invocato** dopo il fallimento, in nessuna
forma: né per riprovare, né per diagnosticare. Tutta l'analisi della §4 è statica e read-only.

---

## 3. Stato fail-closed, verificato

| controllo | esito |
|---|---|
| leaf `j15_runs/j15_fresh_refit_v26c_2026-08-27_r1` | **ASSENTE** — nessun commit è avvenuto |
| lock `.lock_j15_*` | **NESSUNO** |
| staging `.staging_j15_*` | **NESSUNO** |
| sentinella `_j15_*` | **NESSUNA** |
| marker `TECHNICAL_INVALID` | non applicabile: non esiste alcuna leaf |
| 5 pin di readiness | **tutti intatti** |
| incremento J14, aggregato J11, parent J2 | **intatti per hash** |
| `git status` tracciati modificati | i soli tre già dirty a inizio sessione |

Il fallimento è avvenuto **prima** che `commit()` prendesse il lock o creasse lo staging: `run_fit`
è chiamata all'inizio, e l'eccezione è salita prima di qualunque scrittura. Non c'è nulla da
ripulire e nulla è stato ripulito.

`j15_runs/` esiste ma contiene **solo i tre log additivi** scritti dal driver dopo il fallimento,
fuori da qualunque leaf, no-clobber. Non è un artefatto del runner.

---

## 4. Causa, misurata

`run_fit` è **spliciata verbatim** da J11 — una scelta che avevo fatto proprio per evitare che una
ricopiatura a mano alterasse la matematica. Il diff atteso di 18 righe era corretto e il test lo
verificava a ogni run.

Ma una funzione copiata porta con sé le **dipendenze di namespace della propria origine**. `run_fit`
chiama `verify_warm_start_source()` come nome nudo, che in J11 esiste a livello di modulo. In J15
avevo aliasato molti helper — `_sha_file`, `_rmse`, `load_j7`, `load_cell`, `load_parent_state` e
altri — ma **non quello**: lo usavo solo come `J11.verify_warm_start_source()` dentro il mio
`verify_modules()`.

Analisi statica dei nomi liberi, eseguita ora:

| funzione | nomi non risolti in J15 |
|---|---|
| **`run_fit`** | **1 — `verify_warm_start_source`** |
| `audit`, `build_manifest`, `build_receipt`, `commit`, `preflight`, `build_aggregate`, `load_j14`, `verify_sources`, `verify_prereg`, `verify_committed_leaf`, `gate_matrix`, `validate_out` | nessuno |

`run_fit` ha 17 nomi liberi: **16 risolvono, 1 no**. L'estensione del difetto è esattamente quella —
un nome, in una funzione. Nessun'altra parte del runner è affetta.

### Perché la readiness non l'ha intercettato

Questo è il punto che conta, e va detto senza attenuazioni.

- La suite **non chiama mai `run_fit`**: zero call site. I test di materializzazione usano il doppio
  iniettato `fake_fit`, per non richiedere un training da 400 epoche a ogni esecuzione.
- Il preflight **non raggiunge mai `run_fit`**: è chiamata solo dentro `commit`, riga 1476.
- Nessun controllo verificava che i **nomi liberi** di `run_fit` risolvessero nel namespace di J15.

Risultato: una suite da 208 check e un preflight GO sono passati mentre il percorso principale del
runner non era eseguibile. Le 208 verifiche erano vere; semplicemente **nessuna toccava questo**.
La verifica basata sul diff mi ha dato fiducia mal riposta: il diff era corretto, ma un diff non
dice nulla sul namespace in cui il codice copiato dovrà girare.

Non ho applicato alcuna correzione. La correzione minima sarebbe di una riga — legare
`verify_warm_start_source = J11.verify_warm_start_source` accanto agli altri alias — accompagnata
da un controllo statico che asserisca che **ogni** nome libero di `run_fit`, e di ogni altra
funzione del runner, risolva nel modulo. Ma la decisione è tua: la regola dice preservare l'evidenza
e fermarsi, e mi fermo.

---

## 5. Cosa NON è stato fatto

- Nessun retry, nessuna riparazione, nessuna cancellazione, nessun secondo fit.
- L'argv congelato non è stato più invocato dopo il fallimento.
- Nessun rollout, environment, critic, PPO. Torch non è mai stato importato: l'eccezione è salita
  prima della sua riga di import.
- Semi 126, 127, 128 non aperti.
- Nessuna modifica a produzione, FSM v3, corridoio morfologico, reward, sigma, SEA o plugin C++.
- Nessuna modifica a prereg, authorization, record GO, leaf o report esistenti.
- J16 non iniziato.

---

## 6. Artefatti e hash

### Prodotti da questa esecuzione — solo log, fuori da qualunque leaf

| file | SHA-256 |
|---|---|
| `j15_runs/j15_execution_exit_2026-08-27.json` | `9c27a8b8556e4046cfebf08b40925a0e97eeac9991b6026c5bc6001e5bcb70c5` |
| `j15_runs/j15_execution_stderr_2026-08-27.txt` | `310cba674c013003a9ab36f78dd0e7d6690e375107b20c531af098edb27342b5` |
| `j15_runs/j15_execution_stdout_2026-08-27.txt` | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` (vuoto) |

### Invariati e riverificati dopo il fallimento

| artefatto | SHA-256 |
|---|---|
| `v26c_j15_prereg_fresh_refit.json` | `49c748b3a20925a0c270768aa6fddcd3adee474eb1de1cfaa885f849719d1ad2` |
| `v26c_j15_fresh_refit.py` | `7b0073c8832e1af64a243cf9bff70662fb786f84691d3a5eab451e230192507a` |
| `test_v26c_j15_fresh_refit.py` | `5bdeaf8d72b3619c5bca0b99b9bfa1da1644804fe043ba32936e95d920fd1176` |
| `v26c_j15_fresh_refit_authorization.json` (rev3) | `61b6fa4fd45fcf00a418efdcb3a450b7eeaddd56b486cc481ef9de44f127a3c4` |
| readiness report | `949cd6859d3a5b648bc549bde9e0431436a0d1e8529dd3d5a8f4eb6570e5ed40` |
| GO rev1 | `d6c661aa2dd826c73d022420414783810d964e29680be5e8b6468cbca6ff093c` |
| GO rev2 (consumato da questa esecuzione) | `05d58a33fe1a8f5dbaeec156e588ea4cb4b4c0ad0fa0c9638da51a80c841a3a2` |
| incremento J14 | `54b2b8e86a922f554b1fa3bc379737d51000bd0b36596439babd0464e39b0d40` |
| aggregato J11 | `a936c580c4db19383255d3d5f7346560e0fd99dc4b139ac202858e82cca13f42` |
| parent J2 `module_state.pkl` | `0f182ea9f8939e2b7824e85c12c57343309c444680682b9bce5858dd74f9d130` |

---

## 7. Metriche e gate

**Nessuna.** Il fit è morto prima di costruire l'aggregato, prima dello split, prima del primo step
di ottimizzazione. Non esistono RMSE prima/dopo, non esiste una history, non esiste uno stato
committato, e **nessuno dei 17 gate binding è stato valutato** — né i 16 preservati da J11, né il
17°, `j14_increment_rmse_decreases` sulle 854 righe.

Non c'è alcun risultato scientifico da riportare. Ciò che il bundle di readiness aveva verificato
staticamente — 25567 righe, split 5113/20454, clock a zero esatto, parent J2, semi sigillati —
resta vero come proprietà degli input, ma **non è stato esercitato** da questa esecuzione.

---

## 8. Limitazioni

Il vincolo dichiarato in readiness vale a maggior ragione ora: questo stadio è **solo
supervisionato** e non avanza alcuna pretesa sul closed-loop. Con un fit fallito non c'è nemmeno una
pretesa supervisionata: **non esiste alcun attore J15**, e nulla è stato promosso, qualificato o
reso deployabile.

Il problema diagnosticato in J13 — lo swing che non si chiude — resta **intatto e non affrontato**.

---

## 9. Stato della fase successiva

**J16 è BLOCCATO.** La catena richiede un attore J15 committato, che non esiste.

Il GO rev2 autorizzava una sola esecuzione ed è stato **consumato**. Non riprendo il fit senza un
tuo nuovo GO, e quel GO dovrà pinnare un runner corretto, quindi hash diversi da quelli attuali.

---

## 10. TODO propagati

- **La correzione del runner e il controllo statico che l'avrebbe prevenuta** — proposti in §4, non
  applicati. Decisione tua.
- **LOTO / LOCO / B1R1 / B1R2** — restano TODO futuri.
- **Semi 126, 127, 128** — riserva held-out finale, mai letti.
- **Il gate A–F di regressione closed-loop** dopo un refit riuscito.
- **Regola di governance**: ogni fase si chiude con uno user report dedicato, auditato prima della
  fase successiva. Questo report chiude la fase J15-fit, con esito negativo.

---

## 11. STOP

Fit fallito, evidenza preservata fail-closed, nessun retry. **Nessun rollout, critic, PPO. J16 non
iniziato.**

**Fermo in attesa del tuo audit.**
