# V26C J20-K1R1 — Preregistrazione dell'emendamento al gradiente non degenere

**Data**: 2026-08-27
**Stadio**: `V26C_J20_K1R1_NONDEGENERATE_GRADIENT_AMENDMENT`
**Emenda**: `V26C_J20_CRITIC_WARMUP_READINESS`
**Esecutore**: Opus 5, effort xhigh — braccio operativo. Codex architetto e gate owner.

## ESITO: **GO K1R1 EMESSO / K1R1 NON ESEGUITO**

*Revisione architetturale del 2026-08-27 recepita (sei correzioni, §0). Riverifica contro il
filesystem del 2026-08-27 superata, GO emesso in §0-bis. **La sonda non è stata eseguita.***

**La sonda non è stata eseguita.** Nessun leaf K1R1, nessuno staging, nessun lock, nessun
`TECHNICAL_INVALID`, nessun Ray, nessun ambiente, nessun rollout, nessun `algo.train`, nessun
optimizer, nessun PPO, nessun warm-up. Il GO esiste e autorizza **una sola** esecuzione futura;
questo report non la avvia.

**Il leaf K1 resta `FAIL_CLOSED` 28/29, intatto**, insieme al suo GO, ai byte del suo runner,
alla sua preregistrazione e al suo report. Nulla di tutto ciò è stato toccato.

---

## 0. Le sei correzioni della revisione

### 1 — Pin del GO completi, e risolti da una mappa chiusa

Il GO pinnava sei file. Ora ne pinna **quattordici**, che sono **tutti** gli input e gli
eseguibili che lo stadio usa semanticamente: i tre artefatti propri, **tutti e sei** i file
immutabili della leaf K1, il runner K1, la prereg K1 e il GO K1, il `module_state.pkl` di
J19A, e l'unico modulo di produzione importato, `baseline_MLP/asymmetric_rl_module.py`.

Il payload del GO porta **solo label e hash**. Il path lo risolve il runner da
`GO_REQUIRED_PIN_TARGETS`, mai il payload: un GO che potesse nominare i propri path potrebbe
puntare un pin a un file diverso da quello che lo stadio leggerà davvero, e il pin sarebbe
decorativo. La label di `asymmetric_rl_module.py` **non è nemmeno un path relativo a questa
directory** — è risolta da una costante interna, ed è il test che lo dimostra.
`validate_go` rifiuta pin mancanti, extra e stantii.

### 2 — Load esatto, non solo `strict=True`

`strict=True` prova che nessuna chiave mancava o era inattesa. **Non** prova che i valori
atterrati siano quelli su disco: una promozione silenziosa di dtype, o una copia che perde
uno zero con segno, passerebbero un load strict. Ora tutti e sedici i tensori sono
confrontati — prima l'insieme delle chiavi, poi dtype, shape e byte C-order — e `A04`
richiede: missing e unexpected vuoti, 16/16 su entrambi i lati, stesso keyset, e tutte e
sedici le identità loaded-vs-saved vere. Mappa per-chiave e conteggio finiscono nel result e
nel receipt.

### 3 — Contratto di freeze verificato sul modulo vivo

Chiedere un flag nel costruttore non è la stessa cosa che averlo vivo sull'oggetto che verrà
derivato. Dopo il load il modulo è interrogato: `_freeze_actor is True`,
`_freeze_logstd is True`, `inference_only is False`, `_n_actor == 35`, `_n_full == 84`,
`_action_dim == 2`, e la presenza di **entrambi** i figli della torre di valore. `A04` vincola
anche su questo, senza indebolire l'insieme dei check.

### 4 — Range dello stimolo vincolante a run time

`PIN_STIMULUS_MIN`, `MAX` e `ABS_MIN` erano asseriti solo dal test esterno. Ora sono **tre
check dentro `stimulus_report`**: `stimulus["ok"]` fallisce se uno solo differisce. Le
proprietà passano da otto a **undici**. Un test perturba ciascuno dei tre pin e verifica che
lo stadio si blocchi davvero.

### 5 — Immutabilità K1 verificata **tre** volte

Prima della misura, **dopo il backward e prima di qualunque scrittura**, e **dopo il commit**.
I due primi confronti mettono a confronto i sei hash fra loro *e* con i valori pinnati; `A01`
e il composito falliscono in chiuso su entrambi. La verifica post-commit ri-hasha i sei file
K1 una terza volta: un disallineamento aggiunge un problema, quindi **`TECHNICAL_INVALID`
resta al suo posto** e la leaf nasce invalida per sempre.

`k1_leaf_unchanged` non è più una costante `True`: è **derivato** da queste verifiche, e
l'evidenza before / after / post-commit è registrata negli artefatti.

Verificare solo all'inizio lascerebbe non osservato proprio l'intervallo in cui questo stadio
gira — l'unico in cui un incidente potrebbe toccare l'evidenza con cui si compone.

### 6 — Linguaggio scientificamente preciso

«Esattamente due cose differiscono / nient'altro si muove» era un'affermazione più forte del
vero. Ora: **W1 e W2 sono le sole differenze *scientifiche* nel calcolo del gradiente**;
il **percorso evidenziario differisce intenzionalmente**, perché K1R1 ricostruisce un modulo e
carica byte per byte lo stato completo post-K1 invece di rieseguire il trapianto — rieseguirlo
rimisurerebbe evidenza che K1 ha già prodotto sotto un GO. Lo stadio **non è quindi
operativamente identico a K1**, e non va presentato come tale. È identico in ciò che
**misura**, e ogni tensore da cui il gradiente è calcolato è provato byte-identico a quello
che K1 ha committato. Corretto in runner, prereg, test e report.

---

## 0-bis. Riverifica del 2026-08-27 ed emissione del GO

Tutti i controlli sono stati rieseguiti contro il filesystem attuale, non contro i valori del
turno precedente.

| verifica | esito |
|---|---|
| `py_compile` sugli 8 file Python del perimetro | **OK 8/8** |
| suite K1R1 | **213/213** |
| suite telemetria | **161/161** |
| suite readiness J20 | **187/191**, fail **esattamente** `B01 B03 B05 B07` |
| `derive --check` | **matches** |
| preflight K1R1 | **READY**, **12/12 pin**, stimolo **11/11 proprietà**, zero scritture |
| dry-run K1R1 | inerte, zero scritture |
| leaf K1: ri-hash indipendente | **6/6 byte-invariati** |
| leaf K1: verdetto su disco | `FAIL_CLOSED` **28/29**, unico fallito `P24…` |
| leaf K1: commit verification | `ok = True`, `problems = []`, nessun `TECHNICAL_INVALID` |
| destinazione K1R1 | **assente** |

### Il GO

`v26c_j20_k1r1_architect_go.json` — SHA-256
`5323b448c0ee37e7c15175745fd2306cd4beeccfbf7a4404cd1896018a4bf07d`, 80 righe.

Validato in sola lettura dal runner: **VALIDO**, 14 label richieste / 14 fornite, nessun pin
extra, nessuno mancante, **nessuno stantio**. `authorises_execution: true`; `authorises_warmup`,
`authorises_training`, `authorises_rollout`, `authorises_ppo`, `authorises_k1_retry` e
`authorises_rewriting_k1` **tutti `false`**. `execution_limit: 1`, `retry_authorised: false`.

I quattordici pin, per gruppo:

| gruppo | label |
|---|---|
| **1** — artefatti K1R1 | `v26c_j20_k1r1_prereg_nondegenerate_gradient_amendment.json`, `v26c_j20_k1r1_gradient_amendment.py`, `test_v26c_j20_k1r1_gradient_amendment.py` |
| **2** — leaf K1, tutti e sei | `commit_verification.json`, `probe_actor_transplant_report.json`, `probe_full_module_state.pkl`, `probe_module_manifest.json`, `v26c_j20_critic_warmup_readiness_receipt.json`, `v26c_j20_probe_result.json` |
| **3** — catena K1 | `v26c_j20_critic_warmup_readiness.py`, `v26c_j20_prereg_critic_warmup_readiness.json`, `v26c_j20_architect_go_k1.json` |
| **4** — input e codice eseguito | `j19a_runs/.../rl_module/module_state.pkl`, `baseline_MLP/asymmetric_rl_module.py` |

**Nessun report è fra i pin**: i report sono narrativa, non input del calcolo.

I pin sono **label**, non path: ciascuna è risolta dalla mappa chiusa `go_pin_targets()` del
runner, mai dal documento. `baseline_MLP/asymmetric_rl_module.py` non è nemmeno un path
relativo alla directory dello stadio, ed è risolto da una costante interna — è la
dimostrazione che il payload non può scegliersi i percorsi.

Il GO ribadisce esplicitamente che il record K1 è di sola lettura e che il suo verdetto
`FAIL_CLOSED` 28/29 **non deve essere riscritto a PASS**.

---

## 1. Problema

La sonda K1 è tornata `FAIL_CLOSED` 28/29. L'unico check fallito,
`P24_frozen_module_still_gives_the_critic_gradient`, chiedeva che tutti e sei i tensori critic
ricevessero gradiente: `vf_encoder.0.weight` non lo riceveva.

**La causa è lo stimolo della sonda, non la pipeline.** K1 alimentava il modulo con un batch
8×84 di **zeri**. Per uno strato lineare `y = Wx + b` vale `∂L/∂W = (∂L/∂y)·xᵀ`, che con
`x = 0` è **identicamente nullo** qualunque cosa faccia il freeze; il gradiente del bias non
ne risente perché `∂L/∂b = ∂L/∂y`.

La diagnosi non è una congettura: il record di K1 mostra il pattern previsto e **solo** quello.
In **entrambi** i bracci manca esattamente e soltanto il **peso** di primo strato; entrambi i
**bias** di primo strato lo ricevono; entrambi i pesi di **secondo** strato lo ricevono, perché
il loro ingresso è `tanh(W₀·0 + b₀) = tanh(b₀) ≠ 0`. Il braccio di controllo è **non
congelato** e mostra la stessa lacuna: nessuna spiegazione legata al freeze può renderne conto.

**Cosa K1 aveva stabilito**: il freeze funziona, e non vacuamente — cinque parametri attore
che ricevono gradiente da non congelati ne ricevono zero da congelati, e cinque dei sei
tensori critic lo ricevono mentre l'attore è tagliato fuori.
**Cosa K1 non aveva stabilito**: che `vf_encoder.0.weight` lo riceverebbe sotto uno stimolo
non degenere. Dalla forma chiusa della derivata **lo si inferisce**; misurarlo è ciò per cui
esiste questo emendamento.

## 2. Perimetro — cosa questo stadio deliberatamente NON fa

| | |
|---|---|
| **non ritenta K1** | K1 è girato una volta sotto un GO che vietava il retry |
| **non sovrascrive nulla** | leaf, receipt, commit verification, result, GO, prereg e byte del runner K1 restano invariati |
| **non riscrive il verdetto** | il risultato K1 resta `FAIL_CLOSED` 28/29 su disco, per sempre. È quello che è successo; un emendamento che ne modificasse il record sarebbe un falso |
| **non ritrapianta l'attore** | nessun nuovo trapianto |
| **non reinizializza il critic** | carica lo **stato a 16 tensori esatto** che K1 ha committato, quindi misura **lo stesso modulo** che K1 ha misurato, non un sosia |
| **non è operativamente identico a K1** | ricostruisce un modulo e carica lo stato post-K1 byte per byte invece di rieseguire il trapianto: percorso evidenziario diverso di proposito, oggetto misurato identico |
| **non indebolisce P24** | il check emendato è **strettamente più forte**: norma di gradiente finita **e strettamente positiva** su tutti e sei i tensori critic, più nessun gradiente all'attore |

## 3. Implementazione

### Le due sole differenze da K1 — whitelist chiusa

**W1 — lo stimolo.** Una matrice 8×84 float32 deterministica e algebrica al posto degli zeri:

```
value[p-1][q-1] = sign * ((p*37 + q*53 + p*q) mod 191 + 1) / 192.0
                  con p, q 1-based e sign = -1 quando (p+q) è dispari
```

**Nessun RNG, nessun seed**: aritmetica intera e una sola divisione esatta. La matrice è una
proprietà di questo file sorgente e di nient'altro — non di un seme, non dell'accumulo float
di una piattaforma.

| proprietà | misurato |
|---|---|
| shape / dtype | 8×84 / float32 |
| ogni elemento non nullo | **sì**, 672/672 |
| ogni elemento in [-1, 1] | **sì**, `[-0.9947916865348816, +0.9947916865348816]` |
| `|min|` | `0.0052083334885537624` = 1/192 |
| righe distinte | **8/8** |
| colonne costanti | **nessuna**; ogni colonna ha **8 valori distinti** |
| valori distinti per riga | **84** |
| segni | 336 positivi, 336 negativi |
| digest float32 C-order | `4f643e9cf7d46fb95044cf87ad09342ba56c2e22ca7cda4b72bd57e2a537fb35` |

Le **undici** proprietà — le otto strutturali più i tre confronti col range pinnato — sono
**ri-derivate a run time** e lo stadio rifiuta di procedere se una
sola cade. Una costruzione che smettesse di soddisfarne una verrebbe intercettata, invece di
produrre in silenzio un esperimento più debole di quello preregistrato.

**W2 — i pesi per riga.** `[1, 2, 3, 4, 5, 6, 7, 8]`, otto valori positivi distinti sul
termine di valore, così le otto righe non possono annullarsi simmetricamente proprio nel
gradiente del peso di primo strato — la quantità che K1 non poteva osservare.

**Sono le sole differenze *scientifiche*** nel calcolo del gradiente: stessa classe, stessa
geometria, stessi flag di freeze, stesso percorso forward (`_forward_train` logits più
`compute_values`), un solo backward, stessa ri-verifica byte dopo. Il **percorso
evidenziario** differisce di proposito, ed è un'affermazione diversa — vedi §0.6.

### Il modulo sotto test

Lo **stato a 16 tensori esatto** dal leaf K1
(`probe_full_module_state.pkl`, `b5aef9b2…`), caricato con `load_state_dict(strict=True)` in un
`AsymmetricActorCriticTorchRLModule` costruito con `n_actor=35`, `n_full=84`,
`inference_only=False`, `freeze_actor=True`, `freeze_logstd=True`, seguito dal confronto
byte-per-byte di tutti e sedici i tensori contro il pickle salvato. I pesi della costruzione
sono irrilevanti: tutti e sedici vengono sovrascritti dallo stato salvato.

### Condizioni d'ingresso — fail-closed

L'emendamento **rifiuta di partire** se l'evidenza K1 non è esattamente quella per cui è
scritto: verdetto `FAIL_CLOSED`, 28 su 29, `P24` unico fallito, gli altri ventotto tutti veri,
commit verification `ok`, marcatore `TECHNICAL_INVALID` assente, e gli otto flag di inerzia
tutti falsi. Comporre con un esperimento diverso non significherebbe nulla.

### Il gate composito

**PASS 29/29 se e solo se** il record K1 immutabile è esattamente `FAIL_CLOSED` 28/29 con
`P24` come unico fallimento, i suoi altri ventotto check sono intatti, **e** il P24 emendato
passa qui. I ventotto sono **letti, non ri-misurati**: sono evidenza prodotta una volta sotto
un GO, e rieseguirli **sostituirebbe** evidenza invece di aggiungerne.

`P24` emendato = `A06 ∧ A07 ∧ A08`: tutti e sei i tensori critic con gradiente finito e
**strettamente positivo**, `vf_encoder.0.weight` incluso, e **nessun** parametro attore con
gradiente. Un tensore di gradiente di zeri esatti **è presente**: il fallimento di K1 era
proprio un gradiente presente-ma-nullo, quindi richiedere la presenza riammetterebbe il
difetto che questo stadio esiste per chiudere.

**Soglie inventate da questo stadio: 0.**

## 4. Artefatti

| SHA-256 | righe | file |
|---|---|---|
| `d770477cf31988d0f464b157a258b42fa16a1bd8de03c90d24dc8767fb9fbac5` | 300 | **NUOVO** `v26c_j20_k1r1_prereg_nondegenerate_gradient_amendment.json` |
| `572e17e7c3e6599eb3c3a325b4f4b525ca71e8f13bcfc1c6a4d6dca22287adb9` | 1205 | **NUOVO** `v26c_j20_k1r1_gradient_amendment.py` |
| `882328aa0fac322935d0f79cfc374e86ce152e8840c7e88f32e890f6220aa9f0` | 751 | **NUOVO** `test_v26c_j20_k1r1_gradient_amendment.py` |

Tutti in `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`.
Preregistrazione **sigillata**; il runner la verifica **incondizionatamente**, senza alcun
ramo che accetti un placeholder.

**Destinazione preregistrata**, non ancora creata:
`j20_runs/j20_k1r1_nondegenerate_gradient_v26c_2026-08-27_r1`.

## 5. Risultati esatti delle verifiche

| verifica | esito |
|---|---|
| `py_compile` sugli 8 file Python toccati o dipendenti | **OK 8/8** |
| **suite K1R1** | **213/213** |
| suite readiness J20 (esistente, invariata) | **187/191** — vedi nota |
| suite telemetria | **161/161** |
| `v26c_j20_derive_warmup_config.py --check` | **matches** |
| K1R1 `--preflight-only` | **READY**, **12/12 pin**, zero scritture, né torch né ray |
| K1R1 `--dry-run` | piano stampato, zero scritture |
| `git diff --check` | **pulito** |
| leaf K1R1 | **assente** |

### Sulla suite J20 a 187/191

I quattro check che ora falliscono sono **`B01`, `B03`, `B05`, `B07`**, e sono **tutti e soli**
quelli che asseriscono che la destinazione K1 sia assente e che il suo preflight riporti
`READY`. Falliscono perché **il leaf K1 ora esiste**: è la garanzia no-clobber a esecuzione
singola che funziona esattamente come progettata — il runner K1 rifiuta di girare una seconda
volta. **Nessun check scientifico è regredito**: derivazione, overlay, comando sigillato, gate
preregistrati, semantica del GO e pin restano tutti verdi.

Il 191/191 era una misura **pre-esecuzione**. Non ho toccato quella suite: i suoi byte sono
pinnati nel GO K1 e modificarla invaliderebbe quel pin. Lo segnalo perché resti agli atti
invece di apparire come una regressione.

### Cosa mordono i 213 check K1R1

- **struttura**: 14 chiamate proibite assenti, 5 import proibiti assenti, **un solo**
  `backward` e solo in `measure_gradients`, nessun optimizer, `torch.optim` mai raggiunto
  neanche come attributo, e **nessuna funzione di RNG** — lo stimolo deve essere algebrico,
  non campionato;
- **isolamento**: il runner importa solo `asymmetric_rl_module` fra il codice di produzione;
  non importa `warm_start`, `env_factory`, `rollout_eval` né `train_ppo_mlp`;
- **stimolo**: tutte e sette le proprietà, il digest pinnato, il determinismo su tre
  ricomputazioni, e una **ri-derivazione indipendente della formula** scritta nel test che
  deve coincidere byte per byte;
- **evidenza originale**: `verify_original` rifiuta qualunque cosa non sia `FAIL_CLOSED` 28/29
  con `P24` unico fallito, e i sei hash del leaf K1 sono ri-verificati;
- **composito**: non può riportare 29/29 per nessuna via diversa dai ventotto immutabili più
  un P24 emendato genuinamente passato, e dichiara di non riscrivere K1;
- **strettezza**: il test asserisce che il gradiente debba essere `finito ∧ > 0`, non
  semplicemente presente;
- **GO**: la mappa chiusa ha esattamente **14 label**, ne copre ogni categoria richiesta,
  `validate_go` risolve i path dalla mappa e non dal payload, e nove forme di GO invalido sono
  rifiutate, incluse `authorises_k1_retry` e `authorises_rewriting_k1`;
- **load e freeze**: 14 check su identità byte per chiave, keyset, conteggio e contratto di
  freeze vivo;
- **immutabilità K1**: 11 check sulle tre verifiche e sul fatto che il flag sia *derivato*;
- **linguaggio**: 8 check che il runner e la prereg non rivendichino identità operativa;
- **byte**: `bytes_identical` rifiuta cambio di dtype, cambio di shape e **zero con segno** —
  quest'ultimo verificato mostrando che `np.array_equal` lo accetterebbe;
- **digest**: la trascrizione locale di `actor_state_digest` riproduce `d4a13ff7…` dai byte
  reali di J19A.

## 6. Invarianti — verificati

Non modificati: il leaf K1, il suo GO, il suo runner, la sua preregistrazione, la sua suite e
il suo report; J19A, J19B, J19C e ogni leaf e runner precedente; `training_exnovo_cfg.yaml` e
`training_cfg.yaml`; `tb_logging.py` e `train_ppo_mlp.py`; il plugin C++ SEA e il modello
OpenSim; FSM v3, detector, corridoio morphology; la reward; l'architettura di rete; sigma;
ogni directory sotto `Trajectory Generator/runs`.

Aggiunti, tutti additivi: la preregistrazione, il runner dell'emendamento, la suite di test e
questo report.

`git diff --check` pulito. L'unica riga sporca in `training_exnovo_cfg.yaml` era già presente
all'inizio della sessione e non è mia.

## 7. Limiti

- Questo stadio è **preregistrazione e readiness**. Non ha misurato nulla: la sonda non è
  stata eseguita.
- L'inferenza che `vf_encoder.0.weight` riceverà gradiente sotto questo stimolo resta
  un'**inferenza** finché la sonda non gira. È esattamente ciò che K1R1 esiste per convertire
  in misura.
- Se il P24 emendato fallisse, il composito sarebbe `FAIL_CLOSED`, l'evidenza sarebbe
  preservata e **non ci sarebbe retry**.
- Lo stadio non costruisce alcun ambiente: non dice nulla sul comportamento dell'attore sotto
  campionamento parallelo Ray.
- **Nulla è promosso, nulla è deployable.**

## 8. Fase successiva — esplicitamente NON autorizzata

- **La sonda K1R1 è ora autorizzata** dal GO `v26c_j20_k1r1_architect_go.json`
  (`5323b448…`, 14 pin), per **una sola** esecuzione e senza retry. Non è stata eseguita.
- **Il warm-up del solo critic, `V26C_J20_CRITIC_WARMUP`, resta non autorizzato** e non è
  raggiungibile da questo stadio.
- Nessun retry di K1 è stato tentato né è autorizzato.
- Nessuna promozione di checkpoint.

**TODO propagati**: l'audit di restore a zero iterazioni richiesto dal gate `G9` della
preregistrazione J20; `policy_std` sempre `null`, difetto cosmetico ereditato da J12; il
manifest della leaf J8 stantio, invariato per decisione architetturale; la leaf J8 senza
`commit_verification.json`; `nominal_mean_shift` dichiarato e non misurato in J15R1;
`ENV_MUTATION_POLICY` in J19C come superset conservativa ereditata; le suite readiness J18,
J19A, J19B, J19C e J20 non modificate dopo i rispettivi GO; **LOTO**, **LOCO**, **B1R1**,
**B1R2**; l'**Epic** di generalizzazione multi-modello.

## 9. STOP

Emendamento progettato, implementato, testato staticamente e **autorizzato**. **Non eseguito.**

Il GO consente **una sola** esecuzione, senza retry qualunque sia l'esito. Nessun leaf K1R1
esiste, e questo report non avvia nulla.

**GO K1R1 EMESSO / K1R1 NON ESEGUITO. Fermo.**
