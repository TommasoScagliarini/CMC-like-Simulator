# V26C — J21: correzione rev1 dell'attestazione di aggregazione (training-ready)

**Data:** 2026-08-28
**Stadio:** `V26C_J21_TRAINING_READY_ATTESTATION`
**Fase:** **CORREZIONE DELLA PREPARAZIONE — NIENTE È STATO ESEGUITO**
**Foglia di destinazione:** `…/j21_runs/…_r1/` — **assente**; `j21_runs/` non esiste
**GO:** **solo DRAFT, doppiamente inerte** (base e rev1, entrambi rifiutati)
**Preflight:** **READY — 18/18 gate, 139/139 pin**
**Suite:** **354/354 check**

---

## 1. Il rilievo più importante: T15 era falso

Il documento base diceva *«nessuno stadio a monte ha mai rivendicato una
promozione»*. **È falso.** J19A registra legittimamente `actor_promoted: true`:
ha promosso un **attore**, al proprio stadio, dopo undici criteri binding di
eleggibilità offline. Un gate costruito su quell'affermazione sarebbe stato
vacuo oppure avrebbe richiesto di mentirgli.

rev1 lo corregge apertamente. T15 ora vincola **soltanto**:

- nessuno stadio a monte ha promosso **QUESTO checkpoint J20** a
  `TRAINING_INPUT_ONLY`;
- nessuno ha rivendicato **training readiness del checkpoint**;
- nessuno ha autorizzato un lancio, uno stadio successivo, PPO, ex-novo o un
  deployment.

E registra la promozione J19A come **compatibile e distinta**, sotto
`compatible_and_distinct`: `scope: STAGE-LOCAL ACTOR PROMOTION`, oggetto diverso
(un attore, non un checkpoint), affermazione diversa (eleggibilità offline, non
training readiness). **È richiesta, non tollerata**: un J19A che *non* avesse
promosso il proprio attore farebbe **fallire** il gate, perché l'attore dentro
il checkpoint non avrebbe alcuna eleggibilità alle spalle.

La rivendicazione che J21 può fare è ora esattamente: **«la prima attestazione
`TRAINING_INPUT_ONLY` a livello di checkpoint della V26C corrente, dopo J20»**.
Non «il primo training_ready della storia del progetto» — indimostrabile qui, e
i test `B51`/`B52` verificano che **nessun documento del pacchetto lo asserisca**.

## 2. Contatori di training: scopati e tipizzati

`training_started: false` e `training_iterations_run: 0` si riferiscono **solo**
a (a) training pilota PPO o ex-novo a valle di J20 che aggiorni l'attore, e (b)
l'esecuzione di J21 stessa. **Non negano il warm-up del critic**, che ha davvero
completato un'iterazione.

Registrati separatamente e veridicamente, in `training_history_facts`:

| campo | valore |
|---|---|
| `critic_warmup_iterations_completed` | **1** |
| `checkpoint_logical_iteration` | **1** |
| `next_logical_iteration` | **2** |
| `j21_execution_training_iterations` | **0** |
| `downstream_actor_training_started` | **false** |

Il test `D16` verifica che questi tre numeri coincidano con l'evidenza committata
(warm-up `iterations_completed_this_process`, meta `logical_iteration`, R3
`next_iteration`). I test `D12`/`D13` verificano che ogni conteggio sia un
**intero e non un booleano** — la correzione di tipizzazione richiesta. Il set di
campi FAIL è ora scritto per esteso in rev1, non per riferimento.

`environment_constructed: false` è **scopato a J21 stessa**: gli stadi
precedenti hanno costruito environment (J19B, J19C e R3), e `field_scopes` lo
dice. Lo stesso per `ray_started`, `sampling`, `rollout`. `deployable` resta
falso senza scope, sempre.

## 3. Worker sync: prova vera, non un contatore

`weights_seq_no` conta quante volte il **learner** ha pubblicato i pesi. Non dice
nulla su cosa gli EnvRunner abbiano ricevuto. **Declassato a telemetria di
supporto**, non vincola più T8.

Il nuovo **T16** legge il `actor_transplant_report.json` pinnato del J20 e
vincola:

| campo | valore richiesto | misurato |
|---|---|---|
| `weights_synced_before_first_sample` | `true` | **true** |
| `env_runner_count_checked` | **14** | 14 |
| `env_runner_actors_exact` | `true` | true |
| `env_runner_actor_digests` | **14 voci, tutte `d4a13ff7…`** | 14/14 |
| `learner_actor` | `exact` true, `max_abs_diff` 0.0, expected = actual = `d4a13ff7…`, `missing_keys` [] | conforme |
| `saved_initial_actor` | idem | conforme |

Mutation test su **ogni** campo (`B23`–`B25`): 8 mutazioni sul blocco
`integration_validation` più 5 campi × 2 blocchi più i 2 blocchi mancanti.
`B26` verifica che T8 **non** fallisca più su un `weights_seq_no` alterato, e
`B27` che resti comunque registrato.

## 4. Lineage agosto: misurata, non dichiarata

Il base confrontava una **costante** e faceva uno scan a sottostringa per la
parola «July» — insensato qui, dato che la directory contenitrice si chiama
`v26c_july_replica_2026-08-26` e quindi *ogni* etichetta la contiene.

Il nuovo **T17** misura la catena **byte per byte**, ogni link letto dalla
receipt del figlio:

| link | genitore | figlio | fonte |
|---|---|---|---|
| Agosto V26 → J2 | `0ba56eb7…` (manifest39 `2837779c…`, width 39) | `0f182ea9…` (manifest35 `0c88018d…`) | J2 receipt `derivation.parent` + `output_files_sha256` |
| J2 → J8 | `0f182ea9…` | `9c5b1571…` | J8 receipt `inputs.parent` + `output.files` |
| J8 → J18 | `9c5b1571…`, actor `6a879714…` | — (J18 persiste **0** attori, selezione fail-closed) | J18 receipt `inputs.parent` |
| J18/J8 → J19A | `9c5b1571…`, actor sorgente `6a879714…` | modulo `8153dc97…`, actor `d4a13ff7…`, width **35** | J19A receipt + manifest |
| J19A → warm-up | actor `d4a13ff7…` | actor `d4a13ff7…`, invariato | transplant report |

**19 mutation test, uno per link e per campo** (`B28`). Nove artefatti nuovi
pinnati: modulo e manifest della sorgente agosto, modulo/manifest/receipt J2,
modulo/receipt J8, receipt J18 e overlay di provenienza J18.

### Semantica luglio, raffinata

**Vietato** come genitore operativo o input di training: checkpoint, pesi
attore, dataset, trace, label di luglio.
**Ammesso** come fonte informativa dichiarata: costanti metodologiche
July-derived (la provenienza degli iperparametri che la receipt J2 cita), codice
di scaling e protocollo di seeding/split, la banda diagnostica dei 25 mm, e le
**etichette** che contengono la parola July — inclusa la directory contenitrice e
l'etichetta `JULY_FAITHFUL` di J8.

T1 non legge più etichette. Richiede **evidenza positiva** dalla receipt di J8:
`july_faithfulness.does_not_replicate` deve elencare *July's dataset, July's
parent, July's actor, July's output*, e `operational_lineage` deve nominare il
genitore **August V26 J2**. Un'etichetta non è una lineage; decidono i byte del
genitore, che T17 misura. `B30` verifica esplicitamente che
`JULY_FAITHFUL` **non** produca un falso fallimento; `B04` che una nuova
etichetta contenente «july» non lo produca.

## 5. Integrità dei contenuti irrobustita (T18)

Il base ri-hashava *qualunque cosa* le mappe committed contenessero: una mappa
svuotata avrebbe verificato **a vuoto**, e un file aggiunto sarebbe passato
inosservato.

Ora richiesto:

- le mappe committed sono **dizionari presenti** di dimensione esatta:
  **J19B 132**, **J19C 66**; mappa assente, vuota, non-dict o di dimensione
  sbagliata → fallimento;
- il set di file **corrente** di ogni foglia J19 deve essere **esattamente** le
  chiavi della mappa **più** la propria receipt e `commit_verification.json`;
  qualsiasi extra è un fallimento;
- set di foglia esatti: **J19A 7**, **J19B 134**, **J19C 68**, **R3 11**;
- il gate dei **24** file di checkpoint resta invariato (T12).

**Tecnica di test ermetica**, come richiesto: `check_leaf_set` e
`check_committed_map` prendono gli input **come argomenti**, quindi i 10 test
`B31`–`B40` (extra, mancante, vuoto, non-dict, dimensione errata, receipt non
oggetto…) girano su **dati sintetici**. Nessun test crea, cancella o rinomina
alcunché sotto una foglia reale.

## 6. Valori J19C esatti

rev1 sostituisce le approssimazioni arrotondate del base:

| cella | base (arrotondato) | rev1 (esatto dalla receipt) |
|---|---|---|
| G | 0.024417393735478 | `0.024417393735478` |
| **H** | ~~0.025684~~ | **`0.025684338426320914`** |
| **I** | ~~0.024763~~ | **`0.024762677044730928`** |

I sei valori J19B erano già esatti e sono ri-verificati. `B54`–`B57` confrontano
ogni valore di rev1 con la receipt e verificano esplicitamente che H e I **non**
siano i valori arrotondati.

La dichiarazione resta invariata: **tutte e nove** le celle superano la banda
soft di 20 mm; **solo F e H** raggiungono la banda luglio di 25 mm, con **4
campioni su 500** ciascuna; **nessuna** supera i 28 mm, l'unica soglia binding.

## 7. File

| file | stato | sha256 |
|---|---|---|
| `v26c_j21_prereg_training_ready_attestation.json` | **base, invariato** | `df178ffae521aaf46837258ce2036789c6f8ae9f6b4cfb27b4cd11b85d06da79` |
| `v26c_j21_prereg_training_ready_attestation_rev1.json` | **creato** | `9fa87c120de56d4ef0ae063ce3a4126b94a37ca629fe30710387958d9bcdfdfa` |
| `v26c_j21_training_ready_attestation.py` | **modificato** | `e0741e5ea499986118bfc5ea7eacdae78b11f6d177c2c1cc9d3076a512f57b87` |
| `test_v26c_j21_training_ready_attestation.py` | **modificato** | `9a9b2a7b278867fdd669beb25144b457726575800cfae48bc5d61c3eac8bb2f4` |
| `v26c_j21_training_ready_go_DRAFT.json` | **preservato, superato** | `b2b4f6014e2fdd43dc9ca0714f24311ee0ba67d3d20ec79488531182cd10716c` |
| `v26c_j21_training_ready_go_DRAFT_rev1.json` | **creato** | `2286c51025508c38fe51735f4830a75c5eca5542fcf0e6bfcc8a78d7e1aca909` |
| `reports/user/2026-08-28_v26c_j21_correzione_rev1_attestazione_training_ready.md` | **creato** | questo report |

**Nessun file preesistente al pacchetto J21 è stato modificato.** R1, R2, R3, la
foglia warm-up, il checkpoint, le sette preregistrazioni J20, i DRAFT e il GO
operativo R3, e ogni file di produzione restano invariati e ri-hashati a ogni
preflight.

Il DRAFT base è **preservato e mai modificato**; `validate_go` ora lo rifiuta
anche per **staleness**, oltre che per status (`G09d`–`G09g`).

## 8. Conteggi

| grandezza | base | rev1 |
|---|---|---|
| gate | 15 | **18** (aggiunti T16, T17, T18; ridefiniti T1 e T15; ristretto T8) |
| pin verificati per hash | 129 | **139** (92 ereditati dal GO R3 + 45 aggiuntivi + 2 prereg) |
| pin richiesti al GO | 131 | **141** (139 + aggregatore + suite) |
| check della suite | 216 | **354** |

L'ereditarietà dei 92 pin del GO R3 tramite il suo hash pinnato è mantenuta come
progettata, come accettato dall'architetto.

## 9. Test e preflight — risultati esatti

| comando | risultato |
|---|---|
| `python test_v26c_j21_training_ready_attestation.py` | **354/354 check passati** |
| `python v26c_j21_training_ready_attestation.py --preflight-only` | **READY — 18/18 gate, 139/139 pin**, destinazione assente, **nessuna scrittura** |
| `validate_go` sul DRAFT rev1 | **`valid: false`**, rifiutato su entrambi i conteggi |
| `validate_go` sul DRAFT base | **`valid: false`**, rifiutato anche per staleness |

Eseguiti come:

```
cd "Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26"
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python <file>
```

Comando di esecuzione proposto, **non eseguito** e con GO operativo inesistente:

```
env PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
    v26c_j21_training_ready_attestation.py --execute \
    --go-file v26c_j21_training_ready_architect_go.json
```

**Nessun Ray, nessun environment, nessun rollout, nessun training, nessuna
foglia J21, nessun GO operativo.** L'aggregatore resta standard-library-only:
dopo un `evaluate()` completo i moduli pesanti caricati sono **NESSUNO**.

## 10. Preoccupazioni

1. **Il protocollo del pilota resta non sigillato.** Invariato e voluto: J21
   attesta la **readiness del checkpoint**, non del pilota. Se «training_ready»
   venisse letto come «pronto a lanciare», l'attestazione sarebbe fraintesa.
2. **Il gate `env_runner_count_checked == 14`** è una costante presa dalla
   configurazione del warm-up (13 EnvRunner più il locale). Un pilota futuro con
   parallelismo diverso avrebbe un numero diverso: il gate descrive **il
   warm-up**, non il pilota, ed è corretto solo per questo checkpoint.
3. **J18 ha persistito zero attori** (selezione fail-closed). Il link J8 → J18 è
   quindi un link di **protocollo e metriche**, non di pesi: J19A riproduce il
   candidato c13 dal genitore J8. rev1 lo dichiara apertamente; è una catena
   corretta ma non lineare, e vale la pena che l'architetto lo confermi.
4. **T18 confronta i set di file, non i byte dei file extra.** Un file aggiunto
   viene rifiutato; un file *sostituito* è colto dalla mappa committed. La
   copertura è completa solo perché le due verifiche coesistono.
5. **Il valore `next_logical_iteration = 2`** viene da R3, che lo derivò come
   `restored_logical_iteration + 1`. È corretto come «prossima iterazione se si
   riprendesse», non come previsione di ciò che un pilota farà.
6. Restano aperte le preoccupazioni 2–6 del report di preparazione: nove celle
   su nove sopra i 20 mm, critic addestrato per soli 4096 step con
   `vf_explained_var` 0.8018, qualificazione closed-loop a σ = 0.005 con 500
   step per cella.

## 11. TODO

- [ ] Revisione dell'architetto su rev1, aggregatore, suite e DRAFT rev1.
- [ ] Se approvato: GO operativo `APPROVED` **separato** + autorizzazione
      esplicita dell'utente, poi **una sola** esecuzione.
- [ ] **Dopo** un eventuale PASS: preregistrare il pilota PPO conservativo, che
      J21 deliberatamente **non** inventa.
- [ ] Le tre osservazioni «registrato ma non gateato» di R3 rev7
      §`correction_6`, ancora aperte.
- [ ] Decisione sul marker `RESTORE_AUDIT_PENDING` della foglia warm-up.
- [ ] Decidere sui due `gcs_server` orfani del 18/08, tuttora **non toccati**.
- [ ] I 2 check `B06`/`I07` della suite R1 e i 4 della suite R2 restano rossi per
      assenza-destinazione post-esecuzione: annotati, non corretti.
- [ ] TODO ereditati: generalizzazione multimodello (epic 22/08); gate finale di
      recupero AB06.

---

**Stato conclusivo:** correzione rev1 additiva applicata e verificata. Base
preregistrazione, DRAFT originale e tutti gli artefatti precedenti **preservati**.
Suite **354/354**, preflight **READY 18/18 gate, 139/139 pin**. **Nessuna
esecuzione, nessuna foglia J21, nessun GO operativo, nessun training.**
`training_ready` resta **falso** finché l'aggregazione non viene autorizzata ed
eseguita.
