# V26C J20 — Readiness del warm-up del solo critic (K0 audit + K1 probe)

**Data**: 2026-08-27
**Stadio**: `V26C_J20_CRITIC_WARMUP_READINESS` — **readiness, revisione architetturale recepita**
**Esecutore**: Opus 5, effort xhigh — braccio operativo. Codex architetto e gate owner.

## ESITO: **READY PER REVISIONE ARCHITETTURALE / K1 NON ESEGUITO**

Preflight **20/20 pin**, suite readiness J20 **191/191**, suite telemetria **161/161**,
`git diff --check` pulito.

**Nessun warm-up. Nessun PPO. Nessun rollout. Nessun ambiente. Nessun Ray. Nessun
autograd K1. Nessun GO creato. Nessun leaf `j20_runs`.**

Il verdetto `READY` del report precedente era **provvisorio** ed è stato sostituito da
questo. La tua revisione del 2026-08-27 ha trovato tre difetti reali; sono corretti qui, e
in due casi su tre il difetto era che il codice **dichiarava** una proprietà invece di
**averla**.

---

## 0. Le tre correzioni della revisione

### A — `observed_rows` non era una partizione

**Difetto.** `observed_rows` era inizializzato a `1.0` su **ogni** riga, quindi contava le
righe viste, non quelle leggibili. Un'iterazione in cui nessuna riga portava telemetria
avrebbe riportato `observed_rows = 4096`, `missing = 4096` e tutti i contatori assenti —
una lettura che si poteva scambiare per salute.

**Correzione.** I due campi ora **partizionano** le righe: riga leggibile →
`observed=1, missing=0`; riga assente o invalida → `observed=0, missing=1`. Per ogni riga
la somma è esattamente 1; sull'aggregato la somma è il numero di righe.
`observed_rows` **non è un contatore di righe**: conta le righe la cui telemetria si è
potuta davvero leggere, ed è l'unico denominatore sotto cui «zero incidenti» significhi
qualcosa.

**Prove.** Per ogni caso `observed + missing == 1` esatto e ciascuno ∈ {0.0, 1.0};
sull'aggregato di 7 righe miste, `3 osservate + 4 mancanti = 7`; e un'iterazione **cieca**
di 500 righe riporta `observed_rows = 0`, non un bollino di salute.

### B — «non può interrompere il sampling» era una dichiarazione, non un fatto

**Difetto.** La docstring prometteva che un bug di telemetria non potesse fermare il
campionamento, ma nessuna delle quattro superfici di guasto era protetta.

**Correzione.** Quattro confini, ciascuno protetto al proprio livello:

| confine | comportamento in guasto |
|---|---|
| **estrazione** | `training_health_row` che solleva → lo step non contribuisce, il sampling prosegue |
| **metrics logger** | ogni `log_value` protetto **singolarmente**: un campo ostile non ne sopprime altri sette |
| **TensorBoard** | **solo** i tag `training_health/` prendono il ramo protetto |
| **JSONL** | `_training_health_metrics` degrada a tutti-`None`, mai a zeri |

Diagnostica: **una riga su stderr per categoria per processo**, emessa da un percorso a sua
volta protetto — una diagnostica che potesse sollevare reintrodurrebbe, un livello più su,
esattamente il guasto che segnala.

**Il caso negativo conta quanto quelli positivi.** Un guasto su un tag TensorBoard **non**
`training_health` **continua a propagarsi**, ed è asserito da test: proteggere tutti i tag
avrebbe cambiato in silenzio il comportamento in errore di metriche preesistenti, cosa che
nessuno ha chiesto. La protezione è chirurgica — **una sola riga cancellata** in tutto il
diff di produzione, quella `writer.add_scalar` sostituita dal ramo.

### C — K1 non esercitava il percorso di produzione

**Difetto.** Il probe usava `load_state_dict(strict=False)`, un percorso che la produzione
**non prende**. Non provava nulla sull'algebra delle colonne, sull'azzeramento gait-clock,
né sul rifiuto del trapianto di toccare il critic.

**Correzione.** Il percorso autorevole è ora
**`warm_start.transplant_actor_state`** — la funzione che `train_ppo_mlp` chiama davvero —
pilotata da **`warm_start.resolve_source_actor_features`** contro l'overlay esplicito, con
`mode="drop"` e `zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES`.

Il contratto di feature del **target** è preso dal **record dell'ambiente vivo** (gli
`actor_feature_names` registrati in una cella J19C committata), **non** dal manifest che
descrive la sorgente: usare lo stesso manifest da entrambi i lati farebbe accordare il
contratto con sé stesso per costruzione, e non proverebbe nulla.

Il probe verifica ora, sul percorso reale: attore trapiantato **byte-exact**; critic **non
importato dalla sorgente** e lasciato all'inizializzazione **del target** (comparatore di
`warm_start` esatto, digest del critic invariato); σ = 0.005 esatti e log-std byte-identico;
attore congelato; gradienti **solo** al critic nel braccio congelato; **colonne 25:35 di
controller-memory vive** e byte-identiche alla sorgente. Il caricamento nudo resta, ma
**declassato a diagnostica secondaria** — è comunque utile sapere che RLlib lascia un critic
fresco in silenzio, senza eccezione né warning.

**Geometria invariata**: attore 35D, critic 84D. Nessun attore 25D, nessun widening, nessuna
feature controlaterale.

---

## 1. Cosa è stato costruito

Sette artefatti nuovi e due file di produzione modificati (+262 righe, **una sola
cancellazione**: la riga `writer.add_scalar` sostituita dal ramo protetto di B).

| SHA-256 | righe | file |
|---|---|---|
| `f4ec06d77641a460e26df554f4c5c0f10ebceacb4534bbd0368773294706130c` | 472 | **NUOVO** `v26c_j20_prereg_critic_warmup_readiness.json` |
| `d33e0709ca4682ef857e07a85c7015d2d857aba2b04f454afb3bc1aa0c57858b` | 1335 | **NUOVO** `v26c_j20_critic_warmup_readiness.py` |
| `34558c921f7d9d3b892aa69ba38e2efc7599dab5eb97619224be2a8435d14939` | 705 | **NUOVO** `test_v26c_j20_critic_warmup_readiness.py` |
| `f21654ce315827e11bf3a7a8c48331454b71a0c9f439c75454f9652ede81aaaf` | 417 | **NUOVO** `v26c_j20_derive_warmup_config.py` |
| `d8bcfd39f6015eaf7a41deda2edec201e9dbe89df6c425a67f3bf14bc5e4f525` | 251 | **NUOVO** `v26c_j20_warmup_cfg.yaml` |
| `b0f9035413be0a56216e391cd27c4282c3cb708b7733a67ad32e506d281d39ca` | 163 | **NUOVO** `v26c_j20_actor_feature_manifest_overlay.json` |
| `608cbe0ac927506553bee52b6a1eeb815d8d288f61a4a67f6410fbebc486206b` | 749 | **NUOVO** `validation/test_training_health_telemetry.py` |
| `b5543e286410aa9476b5e91614b3d1028516696f1a5c743ec858d1e2c609ce26` | 588 | **MODIF** `tb_logging.py` (+230, −1) |
| `cbd278c39c4fe9efcfb52b7a95eb3e85f24020eb6aedd1892f7832b1472be375` | 4103 | **MODIF** `train_ppo_mlp.py` (+32, −0) |

I primi sei vivono in `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`.

**Preregistrazione risigillata** a `f4ec06d7…`; il runner la verifica **incondizionatamente**,
senza alcun ramo che accetti un placeholder.

---

## 2. Lineage — decisione 1 dell'architetto

**J19A 35D qualificato, non B0820 39D.**

| | |
|---|---|
| fonte immutabile | `j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/rl_module` |
| `module_state.pkl` | `8153dc9765cb984ae05502b57283c00c09b12de2c4b9d5128a0de0fc12566530` |
| actor digest | `d4a13ff742266e9643012a27c57a6ea6b9205b030529d4c7a8af6d874ab26e96` |
| tensori | 10 chiavi, **6 indipendenti** + 4 alias encoder, 76 036 parametri |
| lineage operativa | August V26 imitation → J2 35D → J8 → J18 c13 → J19A |
| evidenza d'ingresso | J19B `PASS` 6/6 e J19C `PASS` 3/3, entrambi i receipt ri-hashati dal preflight |

**B0820 39D è degradato a precedente metodologico** e dichiarato tale nella prereg: attore a
39 feature, mai qualificato closed-loop. **Luglio resta informativo**: nessun checkpoint,
dataset o label di luglio è input operativo; luglio fornisce solo le **meccaniche**, e i
valori sono stati riletti dai due artefatti contemporanei del 13/07.

---

## 3. Il config del warm-up è **derivato**, non scritto — decisione 2

`training_exnovo_cfg.yaml` **non è letto e non è modificato**. Il config nasce da una
trasformazione meccanica del config runtime che l'intera catena V26C già pinna.

| | |
|---|---|
| sorgente | `MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml` |
| SHA sorgente | `a870cc38a77d853bbd5fba86b51cfcc3ef20a33a5823f4a42f1b968ba4a537db` (242 righe) |
| sostituzioni | **11**, che coprono **14** righe sorgente |
| righe ereditate verbatim | **228** |
| invarianti asseriti alla loro riga pinnata | **15** |

Ogni voce della mappa pinna **sia il numero di riga sia il testo esatto atteso**: pinnare
solo il testo sarebbe ambiguo dove due righe coincidono, pinnare solo il numero seguirebbe
in silenzio una deriva della sorgente.

**Le 11 sostituzioni:**

| id | campo | da → a |
|---|---|---|
| S1 | `model.freeze_actor` | `false` → **`true`** |
| S2 | `ppo.train_batch_size` | 4608 → **4096** |
| S3 | `ppo.num_epochs` | 1 → **10** |
| S4 | `ppo.lr` | 5.0e-07 → **1.0e-04** |
| S5 | `ppo.clip_param` | 0.05 → **0.2** |
| S6 | `ppo.kl_coeff` | 1.0 → **0.2** |
| S7 | `parallelism.num_env_runners` | 12 → **13** |
| S8 | `parallelism.ray_num_cpus` | 13 → **14** |
| S9 | `parallelism.exact_start_sampling` | `true` → **`false`** |
| S10 | `simulation.iterations` | 55 → **1** |
| S11 | `simulation.episode_start_offset_choices_s` | tre offset → **lista vuota** |

Invariati e asseriti: `minibatch_size 512`, `gamma 0.99`, `lam 0.9`, `kl_target 0.01`,
`vf_clip_param 10.0`, `vf_loss_coeff 1.0`, `freeze_logstd true`,
`asymmetric_actor_critic true`, `seed 123`, `episode_start_offset_s 1.956870983805102`,
`grf_penetration_termination_m 0.028`.

**Le quattro sezioni intoccabili sono byte-identiche alla sorgente**: `grf` (18 chiavi),
`supervision` (11), `logging` (3), `reward` (**125 chiavi**). Il tool **rifiuta** una
sostituzione che cada dentro una di esse, e il preflight ricompara tutte e quattro.

Perché S9 e S11 vanno insieme: con `exact_start_sampling: true` il trainer **pretende**
`num_epochs 1` ed esattamente tre offset (`_validate_start_sampling_args`). Luglio usava 10
epoche e una sola partenza, quindi il contratto exact-start deve essere spento perché le
meccaniche di luglio siano esprimibili. Verificato contro i validatori reali, non dedotto.

---

## 4. L'overlay del manifest — decisione 3

**Il manifest J19A resta byte-identico.** Nessun byte del leaf è toccato.

**Il blocco era reale e l'ho misurato**: `warm_start._load_actor_feature_manifest`
(`warm_start.py:163-168`) solleva `ValueError` perché il manifest J19A non ha
`actor_feature_names` — **l'unico su 85 manifest del repo**. Omettere il flag non aiuta:
l'auto-discovery trova lo stesso manifest e fallisce identicamente.

**Una prima stesura dell'overlay era ben documentata e inutilizzabile**: teneva la lista
sotto una chiave di documentazione, e il loader reale l'ha rifiutata con la stessa
`ValueError` che l'overlay esiste per evitare. Le tre chiavi consumabili sono state
**portate al livello superiore** per quel motivo misurato:

- `actor_feature_names` — 35 nomi, ordine significativo
- `actor_feature_count` — 35
- `actor_digest` — `d4a13ff7…`

Le ultime due sono **opzionali** per `warm_start` e **hard-checked quando presenti**:
includerle trasforma il trapianto in un controllo di provenienza fail-closed. Un manifest
appaiato al modulo sbagliato viene rifiutato invece che accettato in silenzio.

**Provenienza**: i 35 nomi non sono scritti da me. Sono trascritti da **13 artefatti
committati che concordano tutti** — 9 `.npz` di cinematica J19B/J19C, i manifest J2/J8/J15 e
il receipt J19C. Il preflight rilegge tutte e 13 le fonti e rifiuta qualunque disaccordo.
`sha256` della lista unita: `fb37bed7390b232b0874f5dd98aeb3e5448301ecbdd9d444101f69776f1d508f`.

**Un effetto collaterale che ho misurato invece di assumere.** Con `gait_clock_enable: false`
il trapianto azzera le colonne target di `gait_phase_sin` e `gait_phase_cos`
(`train_ppo_mlp.py:1784-1799`). Quelle colonne in J19A sono **già esattamente zero a livello
di bit**, in `pi.0.0.weight` e nell'alias `pi_encoder.0.weight`: l'azzeramento è quindi un
**no-op dimostrabile** e non può perturbare l'attore prima del primo gradiente. Se non lo
fosse stato, il gate di identità byte sarebbe fallito per un motivo che nessuno aveva
previsto. È il check **P17** del probe.

L'overlay dichiara inoltre **stantio** il campo `status` del manifest J19A
(«closed-loop A-F NOT run»), vero alla nascita di J19A e falso da J19B/J19C in poi.
**Dichiarato, non corretto** — il precedente è l'overlay J8 ordinato dall'architetto.

Il config sorgente non è fornito: la directory J19A non ha un `training_cfg.resolved.yaml`
perché J19A era un fit supervisionato offline, non un training. Il report del trapianto
registrerà `source_config_exists: false`, che è vero. Inventare un config per riempire il
campo non lo sarebbe.

---

## 5. Telemetria di sola osservazione — decisione 4

**Non cambia nulla dell'ambiente, della reward, della dinamica o del controllo di flusso.**
Legge il dict `info` che l'ambiente già pubblica e non ci scrive dentro.

Otto campi, per iterazione, in `train_iterations.jsonl` sotto la chiave `training_health` e
in TensorBoard come sezione di primo livello `training_health/`:

| campo | sorgente | riduzione |
|---|---|---|
| `observed_rows` | step la cui telemetria è **leggibile** | somma |
| `missing_telemetry_rows` | step la cui telemetria è assente o invalida | somma |
| `phase_timeout_stance_rows` | `reward_terms.phase_timeout_exceeded > 0` **e** `phase_timeout_side == 1.0` | somma |
| `morphology_causal_contract_failure_rows` | `reward_terms.morphology_causal_failed_closed > 0` | somma |
| `resync_event_rows` | `phase_fsm.resync_event_this_step > 0` | somma |
| `resync_count_max` | `phase_fsm.resync_count` | **max** |
| `hs_cancelled_count_max` | `phase_fsm.hs_cancelled_count` | **max** |
| `timeout_side_disagreement_rows` | disaccordo `reward_terms` ↔ `phase_fsm`, o side sconosciuto | somma |

Il codice `1.0 = stance`, `2.0 = swing` è quello di `prosthetic_phase_fsm.py:1685-1689` e
`:1707-1711`. **Max e non ultima riga** perché sotto il contratto di cancellazione v3
`valid_hs_count` può **diminuire** — è la regola di `v26c_j1_collect._summarise`, trascritta.

**Partizione (correzione A).** `observed_rows` e `missing_telemetry_rows` partizionano le
righe: riga leggibile → `1, 0`; riga assente o invalida → `0, 1`. Per ogni riga la somma è
esattamente 1, sull'aggregato è il numero di righe. `observed_rows` **non conta le righe
viste**, conta quelle leggibili — l'unico denominatore sotto cui «zero incidenti» significhi
qualcosa. Un'iterazione cieca riporta `observed_rows = 0`, non salute.

**Zero reale contro telemetria mancante.** Una riga senza telemetria non versa uno zero in
**nessun contatore di incidente**. Quindi un contatore che legge 0 significa «osservato e non
accaduto», mai «mai guardato». Nella riga JSONL un campo che non è mai arrivato è `null`, mai
`0.0`. Uno zero silenzioso trasformerebbe la strumentazione mancante in un certificato di
buona salute: è esattamente il fallimento che `v26c_j1_collect` rifiuta sollevando.

**Contenimento (correzione B).** Estrazione, metrics logger, TensorBoard e proiezione JSONL
sono protetti ciascuno al proprio confine; un guasto della telemetria degrada la telemetria e
nient'altro. Un guasto su un tag TensorBoard **non** `training_health` continua a propagarsi,
per non cambiare in silenzio il comportamento in errore di metriche preesistenti.

**Due campi oltre i cinque richiesti**, entrambi motivati:
- `observed_rows` — senza denominatore «zero incidenti» non è un'affermazione di salute.
- `timeout_side_disagreement_rows` — `v26c_j1_collect` **solleva** su questi due casi; una
  callback di sampling non può sollevare, quindi li conta.

**Equivalenza con l'aggregazione validata, contro evidenza committata.** La regola di
conteggio è verificata sommando le righe di trace committate e confrontando con i **receipt
committati**, non con una reimplementazione:

| cella | righe | `phase_timeout_stance` | `morph_causal_failure` | `resync_count` | `hs_cancelled_count` |
|---|---|---|---|---|---|
| J12 F | 354 | 0 = 0 | 0 = 0 | 0 = 0 | **1 = 1** |
| J19C G | 500 | 0 = 0 | 0 = 0 | 0 = 0 | 0 = 0 |
| J19C H | 500 | 0 = 0 | 0 = 0 | 0 = 0 | 0 = 0 |

J12 cella F è **l'unica trace della catena con eventi FSM non nulli** — un timeout in swing e
un heel-strike cancellato — e serve proprio a rendere il controllo non vacuo.

**Suite: 161/161 check**, inclusi ogni chiave richiesta resa singolarmente assente, `NaN`,
infinita, non numerica e `None`; la partizione verificata riga per riga, sull'aggregato e su
un'iterazione cieca di 500 righe; logger ostile su tutti i campi e su uno solo, estrazione
che solleva, writer TensorBoard ostile, `env_metrics` ostile, diagnostica una-tantum, e il
caso negativo che un guasto non-health continua a propagarsi; e una verifica che la regola di
conteggio non scriva mai nel dict che legge.

**Non è un gate.** Nessuna soglia introdotta. Niente termina, blocca o rollbacka su questi
valori.

---

## 6. Lo stadio readiness fail-closed — decisione 5

**K0 — audit.** Il preflight ri-hasha **20 artefatti** e passa 20/20: la prereg sigillata, i
7 file del leaf J19A, il config derivato, l'overlay, il config runtime sorgente, i 4 file di
produzione pinnati (`asymmetric_rl_module.py`, `warm_start.py`, `train_ppo_mlp.py`,
`tb_logging.py`), la suite di telemetria, e i receipt J19B/J19C.

Decodifica inoltre `class_and_ctor_args.pkl` **senza importare la classe che nomina**,
disassemblando gli opcode del pickle: `n_actor=35`, `n_full=84`, `inference_only=True`,
`vf_share_layers=False`. Un preflight non deve importare codice di produzione per decidere se
quel codice è quello pinnato.

`--preflight-only` **non scrive nulla** (verificato confrontando l'albero prima e dopo) e
**non importa né torch né ray** (verificato in sottoprocesso).

**K1 — probe, sul percorso reale di produzione.** Costruisce **un** modulo della classe
pinnata alla geometria pinnata con `inference_only=False`, poi trapianta i dieci tensori
attore chiamando **`warm_start.transplant_actor_state`** — la funzione che `train_ppo_mlp`
chiama davvero — pilotata da **`warm_start.resolve_source_actor_features`** contro l'overlay
esplicito, con `mode="drop"` e `zero_target_features=warm_start.DISABLED_GAIT_CLOCK_FEATURES`.
Misura **29 proprietà**, `P01`–`P29`:

- **P01–P04** — classe corretta, 16 chiavi, le 6 chiavi critic presenti con shape esatte
  `(256,84) (256,) (256,256) (256,) (1,256) (1,)`, **87 809 parametri di critic**;
- **P05–P07** — il **resolver reale** accetta l'overlay esplicito (`explicit_manifest`) e
  valida il digest dell'attore contro i byte della sorgente; il contratto di feature del
  **target** viene dal **record dell'ambiente vivo** in una cella J19C committata e coincide
  con quello della sorgente, 35 largo;
- **P08–P11** — il **trapianto reale** copia 33 feature e azzera **solo** le due gait-clock,
  senza feature target-only né source-only scartate; riporta che la sorgente è actor-only con
  zero chiavi non-attore; lascia il critic intatto (`critic_init_mode`
  `fresh_target_untouched`); e il critic risultante **è l'inizializzazione del target**, non
  qualcosa importato dalla sorgente — il comparatore di `warm_start` è esatto e il digest del
  critic è invariato;
- **P12–P16** — il caricamento di produzione dello stato trapiantato è **strict e completo**;
  tutti e dieci i tensori attore **byte-identici** (stesso dtype, stessa shape, stessi byte
  C-order — deliberatamente **non** `array_equal`, che è numerico e chiamerebbe uguali un
  float32 e un float64, e `-0.0` e `+0.0`); digest canonico riprodotto; **il comparatore di
  `warm_start` stesso** dichiara il trapianto esatto con `max_abs_diff` 0.0; alias encoder
  ancora byte-identici;
- **P17–P18** — le colonne clock sono già esattamente zero, quindi l'azzeramento è un no-op
  dimostrabile; e il blocco **controller-memory 25:35 sopravvive vivo**, byte-identico alla
  sorgente e non azzerato — se il trapianto lo avesse azzerato insieme alle colonne clock, il
  warm-up campionerebbe da una policy diversa da quella che J19B e J19C hanno qualificato;
- **P19–P22** — le righe log-std sono byte-identiche, il log-std è **stato-indipendente**
  (righe 2:4 di `pi.1.weight` esattamente zero), **σ = 0.005 esatti** entro 1e-09, e i flag di
  freeze sono vivi sul modulo;
- **P23–P26** — il modulo congelato **non dà gradiente all'attore** e **lo dà a tutti e sei** i
  tensori critic; un **braccio di controllo non congelato** riceve gradiente sull'attore, così
  P23 non è vacuo; e i tensori attore sopravvivono al probe byte-identici;
- **P27–P29** — l'hash del modulo completo **differisce** da quello di J19A, come deve; il
  leaf J19A è byte-invariato su tutti e sette i file; e la **diagnostica secondaria** del
  caricamento nudo mostra il critic fresco silenzioso, con chiavi mancanti esattamente le sei
  del critic.

Il caricamento nudo `load_state_dict(strict=False)` **non è più il percorso autorevole**: è
registrato come diagnostica perché il comportamento silenzioso di RLlib merita di stare agli
atti, ma non prova nulla sull'algebra delle colonne né sul rifiuto di toccare il critic.

**Il probe non chiama mai `algo.train()`, non costruisce optimizer, non fa step, non avvia
Ray, non costruisce né steppa un ambiente.** L'assenza è asserita **strutturalmente**
camminando l'AST del runner stesso: nessuna chiamata proibita, nessun import proibito,
**esattamente un** `backward` in tutto il file e solo dentro `gradient_reach`. La suite
asserisce inoltre in positivo che il runner **chiami davvero** `transplant_actor_state`,
`resolve_source_actor_features` e i due comparatori di `warm_start`.

**L'unico punto in cui tocca autograd** è quel singolo backward su un batch sintetico, senza
optimizer e senza step. Serve perché `freeze_actor` è implementato staccando i logit
(`asymmetric_rl_module.py:86-88, :154`): se il grafo sia davvero tagliato è una proprietà del
modulo costruito, e asserirla senza misurarla sarebbe esattamente il tipo di affermazione non
verificata che questa pipeline rifiuta. **Lo segnalo esplicitamente perché tu possa vietarlo**:
è l'unica libertà che mi sono preso rispetto alla lettera di «non eseguire training».

Il leaf, quando il probe verrà autorizzato, sarà content-addressed, no-clobber, atomico e
**born-invalid**: `TECHNICAL_INVALID` scritto per primo nello staging e rimosso per ultimo,
solo dopo che la verifica post-commit ha ri-risolto ogni path dal receipt **committato**, l'ha
ri-hashato e l'ha confrontato sia col receipt sia coi byte staged.

---

## 7. I gate preregistrati della futura esecuzione unica — decisione 6

Dodici gate, `G1`–`G12`. **Soglie inventate da questo stadio: 0.**

| id | criterio |
|---|---|
| **G1** | esattamente **una** iterazione logica: una sola riga in `train_iterations.jsonl` |
| **G2** | `num_env_steps_sampled_lifetime` esattamente **4096**, e delta per iterazione esattamente 4096 |
| **G3** | `vf_loss` e explained variance **presenti e finite** — **nessuna soglia** |
| **G4** | mean KL **esattamente 0.0** o numericamente nullo entro 1e-09 |
| **G5** | attore **bit-exact** a ogni audit, digest `d4a13ff7…` a ogni voce |
| **G6** | log-std **byte-identico** e σ ancora 0.005 |
| **G7** | il digest del critic **cambia** |
| **G8** | l'hash del file modulo completo **NON deve** uguagliare `8153dc97…`; il confronto vincolante è sui **tensori attore** |
| **G9** | checkpoint completo con `learner/state.pkl` e optimizer **ripristinabile**, verificato da un audit di restore a zero iterazioni |
| **G10** | exit 0, un solo processo, nessun retry, nessun crash, nessun timeout |
| **G11** | il blocco `training_health` è arrivato: `observed_rows` = 4096, `missing_telemetry_rows` = 0, e la somma dei due = 4096 |
| **G12** | i sette file del leaf J19A ri-hashano ai valori pinnati |

**Su G2** — 4096 sono i **timestep per iterazione**, non un numero di ambienti e non un
cumulativo. La crescita apparente vista nei run multi-iterazione è
`num_env_steps_sampled_lifetime` che accumula un batch per iterazione (4608 → 9216 → 13824 →
18432 → 23040 nel run di agosto). Pinnare **sia il totale sia il delta** rende i due
impossibili da confondere.

**Su G3** — luglio non fissò alcuna soglia: le sue due warm-up registrarono EV 0.598 e 0.238
ed **entrambe passarono**. Inventare una barra qui ripeterebbe l'errore della soglia max-drift
di J18: un numero che nessuno aveva calibrato e che rifiutava tutto.

**Su G8** — il modulo completo porta sei tensori che J19A non ha, quindi l'hash del **file**
non può coincidere e pretenderlo sarebbe un errore di categoria. Il check **P27** del probe
dimostra questa proprietà **prima** che il warm-up giri.

---

## 8. Il comando sigillato

**Verificato contro il parser reale**: `parse_args` più tutti e quattro i validatori
(`_validate_rl_module_args`, `_validate_warm_start_args`, `_validate_start_sampling_args`,
`_validate_kl_guard_args`) senza alcun `SystemExit`. Nessun training, nessun Ray e nessun
ambiente sono stati avviati per verificarlo.

```
cd "Trajectory Generator/baseline_MLP"

PYTHONDONTWRITEBYTECODE=1 /opt/anaconda3/envs/envCMC-rllib/bin/python \
  <BASELINE>/train_ppo_mlp.py \
  --worker-process \
  --config <J20_DIR>/v26c_j20_warmup_cfg.yaml \
  --output-dir <OUTPUT_DIR> \
  --warm-start-raw \
  --warm-start-raw-source <J20_DIR>/j19a_runs/j19a_single_reproduction_v26c_2026-08-27_r1/rl_module \
  --warm-start-raw-source-feature-manifest <J20_DIR>/v26c_j20_actor_feature_manifest_overlay.json \
  --asymmetric-actor-critic \
  --freeze-actor \
  --freeze-logstd \
  --iterations 1 \
  --checkpoint-every 1 \
  --retain-iteration-checkpoints \
  --tensorboard \
  --no-progress \
  --no-update-history
```

`<BASELINE>` = `Trajectory Generator/baseline_MLP`,
`<J20_DIR>` = `.../validation/v26c_july_replica_2026-08-26`,
`<OUTPUT_DIR>` = un path **assoluto** a una directory **fresca**.

- **`--worker-process` è ciò che rende il comando one-shot**: `main()` esegue
  `if not args.worker_process: raise SystemExit(run_supervised(args))`. Col flag,
  `run_supervised` non viene mai raggiunto — nessun loop di restart, nessun respawn di
  sottoprocesso, nessun crash-retry, nessun `supervisor_state.json`. **Un crash è un
  fallimento, non un retry.**
- **Nessun parametro scientifico è sulla riga di comando**: vivono tutti nel config derivato e
  hashato, quindi l'invocazione non può dissentire in silenzio dal documento preregistrato.
- I tre flag di modello sono ripetuti benché già nel YAML (`set_defaults` fa sopravvivere il
  valore YAML quando il flag è assente): il comando sigillato dev'essere autoevidente, un
  lettore non deve aprire un YAML per vedere che è un run ad attore congelato.
- **`--output-dir` deve essere una directory fresca**: `run()` la crea con `exist_ok=True` e
  **non rifiuta** una directory non vuota, quindi riusarla erediterebbe in silenzio la sua
  `train_iterations.jsonl`.

Il runner è la **sola** sorgente di questo comando (`sealed_command()`), e la suite asserisce
che runner e preregistrazione concordino **token per token**.

---

## 9. Test e verifiche eseguite

| verifica | esito |
|---|---|
| `py_compile` sui 6 file Python toccati | **OK 6/6** |
| `test_training_health_telemetry.py` | **161/161** |
| `test_v26c_j20_critic_warmup_readiness.py` | **191/191** |
| `v26c_j20_derive_warmup_config.py --check` | **matches**, 11 sostituzioni / 228 righe ereditate |
| parser reale dell'overlay (`warm_start._load_actor_feature_manifest`) | **accettato**, 35 nomi |
| `--preflight-only` | **READY**, 20/20 pin, **zero scritture**, né torch né ray importati |
| `--dry-run` | stampa piano e comando sigillato, **zero scritture** |
| `--probe` senza `--go-file` | **rifiutato**, exit 1 |
| `--probe` con GO inesistente | **rifiutato**, exit 1 |
| `git diff --check` | **pulito** |
| `j20_runs` dopo tutte le prove | **assente** |

Fra i 191 check J20, quelli che mordono davvero:

- il runner è **strutturalmente** incapace di addestrare — AST, non docstring: 12 chiamate
  proibite assenti, 4 import proibiti assenti, **un solo** `backward` e solo in
  `gradient_reach`, nessun optimizer costruito, `torch.optim` mai raggiunto neanche come
  attributo;
- **in positivo**, il runner **chiama davvero** `transplant_actor_state`,
  `resolve_source_actor_features`, `compare_non_actor_states` e `compare_actor_states`,
  importa `warm_start`, guida il trapianto con la politica di azzeramento di produzione e
  prende il contratto target dal record dell'ambiente vivo;
- la derivazione **rifiuta** una sorgente la cui riga pinnata è cambiata (`S2`), la cui riga
  **invariante** è cambiata (`gamma`), o il cui hash non è quello pinnato;
- l'overlay è accettato dal **loader reale**, e le 13 fonti committate concordano tutte;
- **cinque forme di GO invalido** sono rifiutate: stage sbagliato, `authorises_execution` non
  esattamente `true`, uno qualsiasi dei quattro flag `authorises_warmup/training/rollout/ppo`,
  pin mancante, pin stantio, pin fuori perimetro;
- il probe non parte senza GO valido **e** senza preflight passato;
- la suite di telemetria è rieseguita come sottoprocesso dentro la suite J20.

Fra i 161 check di telemetria, quelli nuovi di questa revisione:

- **partizione**: `observed + missing == 1` su ogni riga, `== 7` su un aggregato di 7 righe
  miste (3 osservate, 4 mancanti), e un'iterazione **cieca** di 500 righe che riporta
  `observed_rows = 0`;
- **contenimento**: logger che solleva su tutti i campi e su uno solo (gli altri sette
  passano comunque), estrazione che solleva, writer TensorBoard ostile, `env_metrics` ostile
  che degrada la riga a tutti-`None`, `on_episode_step` che sopravvive a un logger ostile al
  solo blocco health, la diagnostica emessa **una volta per categoria**, e il **caso
  negativo** che un guasto su un tag TensorBoard non-health **continua a propagarsi**.

### Correzioni documentali della seconda revisione

La tua revisione ha trovato tre disallineamenti fra preregistrazione e codice. Nessuno
toccava un gate; erano tutti documentali, ed erano tutti sbagliati nella direzione che conta
— il documento diceva **meno** di quello che il codice fa.

| # | dove | diceva | dice ora |
|---|---|---|---|
| 1 | `training_health_telemetry.tested` | «130 checks» | la suite deve passare **per intero**; 161 check a questa revisione, con lo **sha256 della suite pinnato dal runner**, così il numero non può divergere dal file senza che il preflight fallisca prima |
| 2 | `K1_probe.what_it_does` | «measures 18 properties» | «measures 29 properties, P01 to P29» |
| 3 | `the_gait_clock_zeroing.why_it_is_checked_anyway` | «Check P19» | «Check P17» — P19 è la byte-identity delle righe log-std, il no-op gait-clock è P17 |

La correzione 3 valeva anche per l'overlay (`the_readiness_probe_checks_it_again`) e per la
sezione 6 di questo report. Le due occorrenze di `P19` rimaste — nella lista dei check
preregistrati e nella descrizione `P19–P22` qui sotto — sono **corrette**: si riferiscono al
P19 della nuova numerazione, cioè le righe log-std.

Sul punto 1 ho scelto la formulazione non-stantia oltre al numero esatto: un conteggio nudo
in un documento sigillato è destinato a invecchiare, mentre legarlo all'hash pinnato della
suite rende impossibile che diverga in silenzio.

### Nota sui miei test

Nella prima stesura cinque check (`A06`, `D11`, `H20`, `I08`, `I09`) controllavano la
**formulazione** dei documenti invece della sostanza, e sono stati riscritti perché mordessero
sull'invariante. Gli artefatti non sono stati cambiati per far passare i test. Lo ripeto qui
perché resti agli atti anche dopo la revisione.

---

## 10. Invarianti

**Non modificati**: J0–J19 runner e leaf; il leaf J19A byte per byte; `training_exnovo_cfg.yaml`;
`training_cfg.yaml`; il plugin C++ SEA e il modello OpenSim; FSM v3, detector, corridoio
morphology; il comportamento della reward; l'architettura di rete; sigma.

**Modificati in modo additivo e dichiarato**: `tb_logging.py` (+166, −0) e `train_ppo_mlp.py`
(+22, −0), sola strumentazione in lettura.

**Worktree sporco preservato.** L'unica modifica a `training_exnovo_cfg.yaml` visibile in
`git status` (`progress: false → true`) **era già presente all'inizio della sessione** e non è
mia: `git diff` la mostra come singola riga preesistente.

---

## 11. Limiti

- **Questo stadio non stabilisce che il warm-up riuscirà.** Nessuna barra di explained variance
  esiste, quindi nessuna può essere fallita.
- **Il comportamento dell'attore sotto campionamento parallelo Ray non è mai stato misurato**
  in questa catena. J19B e J19C sono rollout sequenziali; questo stadio non costruisce alcun
  ambiente. Resta l'affermazione non testata più grande del percorso.
- Nessuna generalizzazione oltre le **nove celle** della qualificazione combinata J19B + J19C.
- **Nulla è promosso, nulla è deployable.**
- Il probe, quando autorizzato, esegue **un** backward. Non è training per nessuna definizione
  operativa — nessun optimizer, nessuno step, byte ri-verificati dopo — ma è l'unico punto in
  cui il perimetro tocca autograd, e lo dichiaro perché sia una tua decisione e non una mia.
- **Il probe K1 non è mai stato eseguito**, per tua istruzione esplicita. È verificato
  staticamente: sintassi, AST, e le firme di tutte le funzioni `warm_start` che chiama
  (`load_module_state`, `resolve_source_actor_features`, `transplant_actor_state`,
  `compare_non_actor_states`, `compare_actor_states`) controllate per introspezione contro i
  miei siti di chiamata. Non ho una prova di esecuzione end-to-end del probe, e non la
  rivendico.

---

## 12. TODO — propagati

**Prossimi passi, da preregistrare e autorizzare separatamente:**

- il **GO per il probe K1** di questo stadio;
- il **GO per l'esecuzione unica del warm-up** `V26C_J20_CRITIC_WARMUP`;
- l'**audit di restore a zero iterazioni** che il gate G9 richiede.

**TODO già aperti, propagati:**

- `policy_std` sempre `null`, difetto cosmetico ereditato da J12;
- il manifest della leaf J8 resta **stantio**, invariato per decisione architetturale;
- la leaf J8 non ha `commit_verification.json`, come la leaf J2;
- `nominal_mean_shift` dichiarato e non misurato nel runner J15R1;
- `ENV_MUTATION_POLICY` in J19C resta una superset conservativa ereditata;
- le suite readiness J18, J19A, J19B e J19C restano ciascuna con un check storicamente falso
  dopo l'emissione dei rispettivi GO, non modificate per decisione architetturale;
- **LOTO**, **LOCO**, **B1R1**, **B1R2**;
- l'**Epic** di generalizzazione multi-modello.

---

## 13. STOP

Le tre correzioni A, B e C sono recepite. **Nessun warm-up avviato, nessun PPO, nessun
rollout, nessun ambiente, nessun Ray, nessun autograd K1, nessun GO creato, nessun leaf.**

Il probe K1 richiede un GO dedicato; l'esecuzione del warm-up ne richiede un altro, separato.

**NO-GO. Fermo in attesa della tua revisione.**
