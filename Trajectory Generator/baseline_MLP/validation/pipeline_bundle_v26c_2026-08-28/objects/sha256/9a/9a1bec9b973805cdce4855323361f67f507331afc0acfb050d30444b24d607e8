# V26B — esecuzione B0/B1 nel ramo 35D-masked: **NO-GO offline**

**Data**: 2026-08-25
**Esito**: B0 materializzato · B1 eseguito · **verdetto offline NO-GO su 5 gate di 8**
**Nessun rollout, nessuna collection, nessuna fase Markov, nessuna promozione. Nessun iperparametro corretto post-hoc.**
**Nessuna modifica a produzione, FSM v3, morfologia, contatto, reward, guardie, config v3, C++ o SEA.**

---

## 1. Protocollo di luglio ricostruito e pinnato

`v26b_july_imitation_protocol_pin.json` — SHA-256 `0198e10965000ed908194083e23a682fec72fe5fc9c2c22e0f022a4f30a049d7`

Estratto dal sorgente di produzione, non dalla memoria:
- `target_domain_imitation.py` SHA `442be2222c935e18607e944d946d23bc8151b32c5e57d64036b8e5a00c1e9ed3`, `adapt_actor` a riga 942;
- blocco loss + ordine dei passi, righe **1135–1159**, SHA `b75c3e2d7bc18255aff8a44be2b731d4a12edd7cd6dcbcfec7038761ba4acb52`;
- iperparametri dai default argparse di `target_domain_dagger.py`: 400 / 64 / 3e-4 / 0.20 / 60 / 1.0 / 0.1 / 1e-5 / 123.

Il test verifica che il blocco pinnato **coincida ancora** col sorgente su disco, e che la mia loss sia **bit-identica** a una trascrizione letterale della produzione su 50 casi casuali. L'ordine per batch è verificato sul corpo del ciclo — `zero_grad → backward → step → restore_logstd → project_columns → assert` — non su ricerche di sottostringa nel file, che avrebbero confrontato definizioni con call site.

**Rilievo utile**: la produzione supporta già nativamente tutto ciò che serve a questo ramo — `hard_zero_first_layer_features` (la maschera), `training_indices`/`validation_indices` (il LOTO), `selection_mode=fixed_final_epoch` (il fit finale) e `freeze_logstd_head`. Non uso `adapt_actor` solo perché carica con `RLModule.from_checkpoint`, formato diverso dal nostro export `rl_module/`; come negli otto fit precedenti implemento in sandbox e pinno la semantica per confronto differenziale.

**Deviazioni dichiarate**: split esplicito LOTO invece della permutazione casuale di luglio; colonne hard-zero (non usate nel path 11/07); `use_deterministic_algorithms(True)` sempre invece che solo in modalità fissa. Il peso logstd resta 0.1 per fedeltà, ed è **dimostrato inerte**: il test verifica che 0.1 e 0.0 producano risultati **bit-identici**, perché con testa a sigma costante l'output logstd non dipende dal tronco.

## 2. B0 — materializzato

Digest **`b2429a8217f99ed7ab4e61620480f797d4617c7713b857624e85c095bed57c55`**, identico a quello previsto dal preflight. Parent esclusivo V26 `5bbc6cbd…`.

| file | SHA-256 |
|---|---|
| `student/B0_35D_MASKED/v26b_b0_receipt.json` | `79a014b6794add8abfb3434c31e791b26687ed4aaa5bd25ceb2299622b988fce` |
| `rl_module/module_state.pkl` | `aa7ea0fa1bbef8bb6ef2a33ee8ebe5defeeb4959148a589b81ff994cf291171f` |
| `rl_module/actor_feature_manifest.json` | `447013433a0a349c6876acf4b9459c5203e48e7d76e405fed06264f9db84c0be` |

Manifest unico 35D con la dichiarazione `controller_state_mask` (attiva, colonne 25–34, valore 0.0). Nessuno scaling applicato: le sole scale ≠ 1.0 della tabella pinnata (indici 27, 28, 32, 33) cadono **tutte dentro il canale mascherato**.

## 3. B1 — le invarianti passano, la qualità no

Dataset: 1500 righe su 3 traiettorie di ancore V26 pinnate, label `u_IK` per lookup esatto. Split LOTO 3×(1000/500). Ogni fold riparte dallo **stesso B0**.

### Invarianti — tutte PASS

| invariante | esito |
|---|---|
| 10 chiavi, larghezza 35 | ✅ |
| colonne mascherate esattamente zero | ✅ |
| colonne clock esattamente zero | ✅ |
| logstd byte-identica a B0 | ✅ |
| nessun aggiornamento delle colonne mascherate | ✅ |
| critic mai caricato né modificato | ✅ |
| equivalenza funzionale 25D **dopo il fit** | ✅ bit-identica |
| ogni fold migliora rispetto a B0 | ✅ (0,62 → 0,046–0,176) |

**La macchina di mascheramento funziona esattamente come progettata.**

### Fold

| holdout | best epoch / eseguite | RMSE B0 → dopo | knee rmse / max | ankle rmse / max |
|---|---|---|---|---|
| minus020 | **14** / 74 | 0,61265 → **0,17640** | 0,23752 / 0,65100 | 0,07628 / 0,23802 |
| nominal | 385 / 400 | 0,62335 → 0,04637 | 0,04929 / 0,12710 | 0,04326 / 0,15611 |
| plus020 | 159 / 219 | 0,62306 → 0,05202 | 0,05483 / 0,16973 | 0,04905 / 0,23297 |

Mediana dei best epoch: `[14, 159, 385]` → **159** (regola fissata prima del fit: mediana ordinata, arrotondamento al più vicino con pareggi verso l'alto). Fit finale su tutte le 1500 righe, senza validazione, per 159 epoche.

### Gate vincolanti

| gate | soglia | osservato | esito |
|---|---|---|---|
| invarianti di integrità | — | tutte vere | **PASS** |
| equivalenza funzionale 25D | bit-exact | bit-identica | **PASS** |
| ogni fold migliora su B0 | — | 3/3 | **PASS** |
| RMSE held-out per fold | ≤ 0,05 | 0,17640 · 0,04637 · **0,05202** | **FAIL** |
| media pesata LOTO | ≤ 0,03 | **0,09160** | **FAIL** |
| RMSE finale aggregata | ≤ 0,02 | **0,04128** | **FAIL** |
| RMSE finale per giunto | ≤ 0,03 | knee **0,04910** · ankle **0,03159** | **FAIL** |
| max_abs finale per giunto | ≤ 0,15 | knee **0,16001** · ankle 0,13123 | **FAIL** |

**VERDETTO: NO-GO.** Nessun rollout eseguito, nessun iperparametro toccato.

## 4. Diagnosi, senza proposte

Il risultato è dominato da **un solo fold**: minus020 a 0,17640 contro 0,04637 e 0,05202, cioè 3,4–3,8×.

Le **label sono identiche** fra le tre traiettorie — stesso riferimento AB06, solo traslato: knee u ∈ [−0,3554, +0,7813] e ankle u ∈ [−0,2217, +0,5717] in tutte e tre. Quindi non è un problema di distribuzione del bersaglio.

A differire è il **supporto osservativo**, misurato nello spazio delle 25 feature vive:

| holdout | distanza mediana al training | p90 | frazione oltre il p99 interno del training |
|---|---|---|---|
| **minus020** | **0,645** | **5,043** | **47,4%** |
| nominal | 0,391 | 1,061 | 9,2% |
| plus020 | 0,398 | 0,900 | 8,8% |

Tenendo fuori minus020, quasi metà delle sue righe cade fuori dal supporto del training, mentre per le altre due la frazione è sotto il 10%. Nominal e plus020 sono mutuamente interpolabili; **minus020 è un'estrapolazione**. Il suo early stop a 14 epoche è coerente: il fit diverge da quella traiettoria quasi subito.

Riporto anche, come fatto e non come proposta: il fold nominal ha raggiunto 0,04637 alla **epoca 385** su 400, cioè non aveva ancora smesso di migliorare quando il budget è finito.

**Non correggo nulla.** La regola preregistrata è NO-GO, niente rollout, nessuna correzione post-hoc di iperparametri, e la rispetto.

## 5. File creati o modificati

**Nuovi tooling e pin**

| file | SHA-256 |
|---|---|
| `v26b_july_imitation_protocol_pin.json` | `0198e10965000ed908194083e23a682fec72fe5fc9c2c22e0f022a4f30a049d7` |
| `v26b_b_exec.py` | `6b21a287b65a7e0eef339dcc4da1402a86dd48c73758baba20d6cebaae9b8410` |
| `test_v26b_b_exec.py` | `0eb47e0ede32ff89af2c8ef80dbcae491237a218acf34bb389ef0892b2f8b8fc` |

**Artefatti prodotti**

| file | SHA-256 |
|---|---|
| `student/B0_35D_MASKED/…` | receipt `79a014b6…`, module_state `aa7ea0fa…`, manifest `44701343…` |
| `candidates/B1_BASE35_MASKED/v26b_b1_receipt.json` | `e7328ac2eb49787e905ef0919f06c540bdd2ce4ec30d290e711f8b0af42f53b1` |
| `candidates/B1_BASE35_MASKED/rl_module/module_state.pkl` | `778ff748bd33ca658b62aae1b091be7b702242e9faabb8e0192daf7453e80a5b` |
| `candidates/B1_BASE35_MASKED/rl_module/actor_feature_manifest.json` | `3351f9c34b3b87a96dbe1a6d7944b0f1e44e97e90e449e94f4ba3989a4199b2c` |
| log `b0_materialize_20260825_150405.log` | `86bfd2687da4decf30df0a03228b0b99d96581cc07758f8f7eadcc939ce57ef9` |
| log `b1_base_fit_20260825_150418.log` | `145f18f9b3bd78d76a3175088c30623de4660cc7c0ac04040b7bda7a53f3dcc4` |

**Nessun file esistente modificato.** I moduli B0/B1 preflight-only restano ai loro SHA pinnati nell'emendamento; l'esecuzione vive in un modulo separato e additivo.

**Comandi eseguiti**
```
python test_v26b_b0b1_masked35.py  -> PASS, 85 check
python test_v26b_b_exec.py         -> PASS, 100 check
python v26b_b1_base_fit.py --preflight -> GO
python v26b_b_exec.py --stage b0 --authorized-stage V26B-B0-MASKED35-TRANSPLANT
python v26b_b_exec.py --stage b1 --authorized-stage V26B-B1-BASE-FIT-MASKED35
```

`git status` conferma zero modifiche a `osim_trj_cmc_like.py`, `target_domain_imitation.py`, `prosthetic_phase_fsm.py`, `online_grf.py`, `reward_function.py`, al config v3 e a `tools/`. Nessuna directory di rollout B0/B1 esiste.

## 6. Decisioni che spettano a te

1. Se il NO-GO chiuda la fase base così com'è, o se rivedere le soglie preregistrate alla luce del fatto che **minus020 è fuori supporto al 47,4%**.
2. Se il dataset base debba restare a tre traiettorie o se il LOTO su tre start sia una richiesta di generalizzazione più severa di quella che la fase base deve soddisfare.
3. Se il budget di 400 epoche sia il vincolo operante, dato che il fold nominal migliorava ancora alla 385.

Non propongo nulla di operativo e non tocco iperparametri: attendo.

## 7. TODO propagati

- **TODO-2** — σ non assunto, da misurare prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo: **rimosso per costruzione** in B0/B1, invarianti verificate. *(indirizzato)*
- **TODO-12** — Nuovo: minus020 è fuori dal supporto delle altre due traiettorie al 47,4%; il LOTO su tre start è quindi un test di estrapolazione, non di interpolazione. *(nuovo)*
