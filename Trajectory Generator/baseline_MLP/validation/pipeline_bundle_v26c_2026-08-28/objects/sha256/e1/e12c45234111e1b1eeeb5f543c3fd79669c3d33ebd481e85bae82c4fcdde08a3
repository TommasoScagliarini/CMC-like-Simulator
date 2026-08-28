# V26C J12 — Qualifica closed-loop post-J11: readiness

**Data**: 2026-08-27
**Stadio proposto**: `V26C_J12_CLOSED_LOOP_QUALIFICATION`
**Stato**: bundle **r2**, dopo la prima revisione architetturale. **Nessun rollout eseguito,
nessun leaf creato.**
**Autorizzazione**: `execution_permitted_now: false`. Il comando è congelato ma **non** autorizzato.

---

## 0. Revisione r1 → r2: correzione delle etichette residue

La readiness r1 **non è stata approvata**. L'architetto ha richiesto la correzione di otto etichette
copiate da J9R1 che rendevano ambiguo a quale stadio e a quale attore J12 si riferisse, **senza
cambiare scienza né avviare rollout**.

| # | Prima | Dopo |
|---|---|---|
| 1 | docstring `verify_actor`: "the exact six-file leaf" | "the exact **EIGHT-file J11 leaf**", con la ragione: J11 committa anche l'aggregato e il proprio `commit_verification.json` |
| 2 | blocker preflight e rifiuto al commit: "a J9 lock" / "the J9 lock" | **"a J12 lock"** / **"the J12 lock"** |
| 3 | `would_write.per_cell`: `j9_cell_<ID>_*` | **`j12_cell_<ID>_*`**, più la directory `sim_outputs/` con i suoi 19 file |
| 4 | errore sul conteggio feature: "the J8 manifest holds…" | **"the J11 manifest holds…"** |
| 5 | errore di drift post-matrice: "the J8 actor changed" | **"the J11 actor changed"** |
| 6 | commento sul close fallito: "chained J9Error" | **"chained J12Error"**, che è l'eccezione realmente sollevata |
| 7 | `argparse description`: "V26C J9 closed-loop qualification" | **"V26C J12 closed-loop qualification of the J11 actor"** |
| 8 | `FORBIDDEN_HERE`: "mutating any env field but episode_start_offset_s" | riformulato senza ambiguità (sotto) |

**Punto 8 in dettaglio.** La vecchia formulazione lasciava intendere che `output_dir` fosse
un'eccezione alla regola d'immutabilità. Non lo è: non è affatto un campo scientifico o di runtime.
Ora `FORBIDDEN_HERE` dice che **nessun campo scientifico o di runtime è mutabile** e che
`episode_start_offset_s` è **l'unico** che la matrice varia — perché la matrice *è* uno sweep sui
tre start preregistrati. `output_dir` è dichiarato separatamente come **strumentazione per-cella
autorizzata**: nomina dove finiscono i 19 `sim_outputs` di quella cella, non ha significato
scientifico, non cambia la dinamica e non entra in alcun gate. La policy è esposta in una
costante `ENV_MUTATION_POLICY` che compare sia nel preflight sia nel receipt, e resta comunque
applicata dalla guardia di uguaglianza sulle chiavi stabili.

### Un nono punto, non in lista: un'affermazione **falsa**, non un'etichetta

Applicando le otto correzioni ho trovato un blocco copiato che non era mal etichettato ma
**fattualmente sbagliato per questo stadio**. `multistart_disambiguation` dichiarava:

> "the two July teacher datasets of 8000 rows remain DEFERRED and absent from J7/J8"

Vero dell'attore J8 che J9R1 testava. **Falso dell'attore J11**, il cui aggregato da 24713 righe
*include* le due celle J10R1 replicate otto volte. Portarlo avanti invariato avrebbe messo a
receipt un'asserzione non vera.

Corretto: i dati multistart di training sono **PRESENTI**, e sono le celle **August J10R1**, non i
dataset July — che restano deferiti e non sono mai un input operativo. La distinzione fra dati
multistart di *training* e validazione multistart in *closed loop* resta, e il blocco registra
esplicitamente da cosa è stato corretto. È una correzione di correttezza a un'affermazione, non un
cambio di scienza.

### Cosa **non** è cambiato

Nessuna soglia, nessun gate, nessuna cella della matrice, nessun offset, nessun σ, nessuna
autorità. `verify_scientific_equivalence` continua a riportare `thresholds_invented_here: 0` e
l'uguaglianza campo per campo con J9R1 e con J1/J3 alla sorgente regge invariata.

I riferimenti a J8 e J9 che sono **davvero storici** sono stati mantenuti e ognuno è ora asserito
dal test: l'attore che **non** è J8/J4/July, i cinque artefatti J9 pinnati come evidenza
immutabile, la correzione unità-offset letta dalla preregistrazione J9 originale, il difetto del
manifest J8 che J11 ha corretto, e `actor_before` in `verify_scientific_equivalence`.

**Verifiche r2**: `py_compile` OK su runner e test; suite **410 check PASS** (23 nuovi, uno per
correzione più le prove sulla policy di mutazione e sul blocco multistart); preflight **GO**
inerte, torch e ray assenti; nessun `j12_runs`, lock, staging o sentinella; **22/22 pin
riverificati** su disco.

---

## 1. Problema

J11 ha prodotto un attore fittato su 24713 righe, con **PASS offline** su 16/16 gate vincolanti.
Ma un PASS offline non dice nulla sul comportamento in anello chiuso: il gap che ha originato
questa catena — J9R1 fallito sulla cella B con 0 cicli validi e l'FSM bloccato in WAIT_HS per
500/500 step — si manifesta **solo** rollout-ando.

J12 è la qualifica closed-loop di quell'attore, e **solo** di quell'attore.

---

## 2. Strategia

Adattare il runner **già hardened J9R1** all'attore J11, cambiando l'attore e **nient'altro** di
scientifico.

### Il fatto che ha guidato tutto il lavoro

La matrice A–F che hai contrattualizzato è **già esattamente** quella di J9R1: stessi id, modi,
seed e offset, cell per cell. Non c'era nulla da ridisegnare. Ne consegue che la prova più forte
che questo stadio possa dare non è "ho riusato i gate", ma **"ho dimostrato che non ho cambiato
nulla"**.

`verify_scientific_equivalence` importa il modulo **J9R1 congelato** (pinnato per hash; importarlo
non fa I/O e non tira torch) e confronta **campo per campo**, prima che esista un ambiente:
`COMMON_GATE`, `KINEMATIC_GATE`, `SIGMA`, `SIGMA_TOLERANCE`, `NOISE_HOLD_STEPS`, `EXPECTED_STEPS`,
`RESET_TIME_TOLERANCE_S`, `CLOCK_COLUMNS`, `CONTROLLER_COLUMNS`, `ACTOR_WIDTH`, `FROZEN_OFFSETS`,
e la matrice cella per cella su id/mode/seed/offset. Poi ri-deriva il gate comune da `J1_GATE` e
quello cinematico da `J3_KINEMATIC_GATE` **alla sorgente**, così l'uguaglianza non è con una copia
ma con l'originale. Il receipt registra `thresholds_invented_here: 0`.

Il test dimostra che il meccanismo morde: alterando una soglia di J9R1 in memoria, lo stadio si
rifiuta di partire.

### Lineage

August V26 imitation → J2 35D → **J11** → questa qualifica. July è metodologia ed evidenza
soltanto. Il parent J8 è escluso esplicitamente, così come J4 e ogni checkpoint July.

### Matrice (invariata, 500 step/cella, ordine congelato)

| cella | modo | seed | offset | start |
|---|---|---|---|---|
| A | deterministic | 123 | `1.956870983805102` | nominale |
| B | deterministic | 123 | `1.756870983805102` | −0.20 s |
| C | deterministic | 123 | `2.156870983805102` | +0.20 s |
| D | stochastic_held | 123 | `1.956870983805102` | nominale, σ 0.005 |
| E | stochastic_held | 124 | `1.956870983805102` | nominale, σ 0.005 |
| F | stochastic_held | 125 | `1.956870983805102` | nominale, σ 0.005 |

`noise_hold_steps = 1`. Un FAIL comportamentale **non** ferma le celle successive; un'eccezione
tecnica o d'integrità sì, e allora non viene committato nulla. Provato staticamente (nessun
`break`/`continue` nel loop) e dinamicamente (con un gate fallito, tutte e sei le celle girano).

### Semantica delle azioni

σ **non viene mai iniettata come costante**. Nel ramo stochastic, `rollout_eval._held_stochastic_action`
calcola `action = mean + std * unit`, dove `std = exp(logits)` è letta dalla testa **viva** e
`unit` viene da `HeldStandardNormal(default_rng(seed), shape, 1)`. Il runner verifica a **ogni
step** che `|std − 0.005| ≤ 1e-6` e che `applied_noise == std * unit` esattamente. Nel ramo
deterministico l'azione è confrontata bit a bit con l'helper di `rollout_eval`.

---

## 3. Gate — nessuna soglia inventata

**Comune** (da `J1_GATE`, meno la barra soft 0.020): `steps_required 500`,
`end_reason episode_time_limit`, `valid_cycles_min 2`, `phase_timeout_stance_max 0`,
`phase_timeout_swing_max 0`, `morphology_causal_contract_failure_max 0`,
`hs_cancelled_count_max 0`, `resync_count_max 1`.

**Cinematico** (da `J3_KINEMATIC_GATE`, invariato): `ankle_min ≤ −0.03` (esattamente −0.03 PASSA),
`ankle ROM ≥ 0.30`, `knee ROM ≥ 0.60`, ginocchio **strettamente** flesso (`< 0.0`, esattamente 0.0
FALLISCE), bounds inclusivi `knee ∈ [−1.5, 0.0]` e `ankle ∈ [−0.7, 0.7]`.

**Telemetria**: invariante tecnica separata, mai comportamentale — `hs ≥ cicli`, `to ≥ cicli`,
`|hs − to| ≤ 1`. Una violazione rende la cella **INVALID**: un'affermazione sull'evidenza, mai sul
cammino.

**Penetrazione**: `> 0.020` soft diagnostica, `≥ 0.025` diagnostica July, `> 0.028` **unico hard
binding** — esattamente 0.028 **passa**. Il test verifica i sei casi di bordo
(0.0199 / 0.020 / 0.0201 / 0.025 / 0.028 / 0.0281) contro il contratto, e un check **AST** prova
che nel runner non esiste alcun letterale numerico 0.020/0.025/0.028: le tre soglie compaiono solo
in prosa, e l'autorità è il contratto.

**Aggregato**: PASS se e solo se 6/6 comportamentali PASS **e** 6/6 telemetria valida.

---

## 4. Visibilità per cella

Ogni cella espone, in un blocco `telemetry` dedicato invece che sepolto nel summary:
`phase_timeout_stance`, `phase_timeout_swing`, `morphology_causal_contract_failure`,
`resync_count`, `hs_cancelled_count`, `valid_cycle_count`, `valid_hs_count`, `valid_to_count`,
`steps`, `end_reason`. Più: cinematica (min/max di ginocchio e caviglia), le **tre** bande di
penetrazione con conteggi e frazioni, il gate completo con l'elenco di ciò che è fallito,
l'integrità telemetrica, il verdetto, e il conteggio **WAIT_HS**.

### WAIT_HS: diagnostico, non un gate

J9R1 non contava WAIT_HS affatto. L'ho aggiunto perché è la lettura più rapida sul fatto che il
modo di fallimento della cella B si sia ripresentato: là l'FSM restò in WAIT_HS per 500/500 step
con zero cicli.

Ma è **esplicitamente non vincolante**, e questa è una scelta che va motivata: attaccargli una
soglia sarebbe un **gate nuovo**, e il tuo punto 4 dice di riusare esattamente i gate provati senza
inventarne. Un gate nuovo va preregistrato come tale, non introdotto di contrabbando dentro un
adattamento. Il test verifica staticamente che `evaluate_cell_gate` non menzioni mai WAIT_HS.

L'indice è risolto **per nome** dai nomi runtime vivi (in quest'ordine è 17), mai posizionalmente.

### Una correzione a J9R1

`DIAGNOSTIC_NOT_BINDING` di J9R1 elenca `valid_cycle_count` fra i diagnostici, mentre
`valid_cycles` è un check **vincolante** nello stesso file. È auto-contraddittorio. In J12 è
rimosso, **senza toccare alcuna soglia**. Il test misura entrambe le cose: che J12 non lo elenchi e
che J9R1 lo elencasse.

---

## 5. Findings dei tre audit paralleli

**A — struttura di J9R1.** La matrice A–F esiste già identica; il percorso `stochastic_held` è già
production-grade e non va riscritto. J9R1 **non ha alcuna verifica post-commit**: registra l'hash
del receipt dopo il rename e non lo confronta con nulla. Registra `_rel(sim_out)`, un path di
**staging** repo-relative che cessa di esistere al commit. Accetta qualsiasi directory
`sim_outputs` **non vuota**. Il suo test asserisce incondizionatamente il proprio preflight GO,
quindi è nato spento.

**B — caricabilità del leaf J11.** Un difetto reale e serio: `RLModule.from_checkpoint`
**fallisce in silenzio**. `class_and_ctor_args.pkl` nomina `asymmetric_rl_module`; se quel modulo
non è importabile, ray cattura *ogni* eccezione, ripiega su un `RLModule` base e lo restituisce
**senza sollevare**, con `get_state() == {}`. Uno stadio che non controllasse rollout-erebbe una
rete vuota senza accorgersene. Inoltre: il leaf J11 ha **8 file**, non i 6 di J8, e lo schema del
suo receipt è diverso (`receipt["parent"]`, `receipt["aggregate"]`, non `receipt["inputs"]`) —
una copia verbatim di J9R1 avrebbe sollevato `KeyError`.

**C — costanti dei gate.** Tutte trascritte e confermate ai valori sopra. Confermato che
`episode_start_offset_choices_s` nella config pinnata è esattamente
`[1.756870983805102, 1.956870983805102, 2.156870983805102]`, che l'offset è **relativo a
`cfg.t_start`**, e che σ = 0.005 viene da `pi.1.bias[2:] = -5.2983174324035645`.

### Cosa ho fatto con questi findings

| finding | risposta in J12 |
|---|---|
| `from_checkpoint` fallisce in silenzio | il modulo caricato deve essere `AsymmetricActorCriticTorchRLModule`, `get_state()` deve avere le dieci chiavi attore, e **ogni tensore deve essere bit-identico** al `module_state.pkl` pinnato. Tre modi di fallimento testati separatamente. |
| `asymmetric_rl_module.py` non era pinnato | pinnato: definisce la classe deserializzata |
| leaf a 8 file, schema receipt diverso | `PIN_J11` a 8 voci; lineage letta dallo schema J11 (parent, aggregate sources, verdict, 24713 righe) |
| nessuna verifica post-commit | adottata da J10R1/J11: leaf **nato invalido**, marker nello staging *prima* del rename, ri-risoluzione e ri-hash di ogni path contro il receipt **committato** |
| path di staging nel receipt | tutti leaf-relative; il test verifica che `STAGING_NAME` non compaia nel receipt. Ne ho trovato uno residuo (`base_output_root`) proprio con quel test e l'ho corretto |
| `sim_outputs` non vuota | **esattamente 19** file regolari, con 18 e 20 entrambi rifiutati e l'asserzione sul messaggio della guardia |
| test nato spento | le asserzioni GO/assenza sono **condizionali** sull'esistenza del leaf |

---

## 6. Verifica dell'attore prima di ogni rollout

Il leaf J11 è consumato **solo se** il suo `commit_verification.json` dichiara `pass: true`, non
porta marker `TECHNICAL_INVALID`, e non sopravvive alcun lock o staging accanto. Misurato: tutti e
tre soddisfatti.

Poi: 8 file pinnati; clock `[0, 1]` **esattamente zero** in entrambi i layer 35-wide; dieci colonne
controller **vive** (norma minima 0.0084); σ = 0.005 con deviazione **3.29e-10** contro una
tolleranza di 1e-6; il manifest nomina il modulo che gli sta accanto e porta l'`actor_digest`
corretto — il difetto di J8, il cui manifest nomina il proprio *parent*, non è ereditato.

---

## 7. File creati

Hash **r2**, rigenerati dopo le correzioni:

| File | SHA-256 | variato in r2 |
|---|---|---|
| `v26c_j12_prereg_closed_loop_qualification.json` | `5f0ed000b62c7c64ceaca8eca0d83d36e863619bf528a285aa7d596d63120bbe` | no |
| `v26c_j12_closed_loop.py` | `65d7b8619074c909753750f03aec751d38179c2226e50c87da62113ea5769fca` | **sì** |
| `test_v26c_j12_closed_loop.py` | `ba0a899b0b6c9835cefa27ea4fb0282f6237c0e30bc251825771eb02ca9209ce` | **sì** |
| `v26c_j12_closed_loop_authorization.json` | `0d83943e0e057cc925cfc395945d86ff53c756909010d4781423039b595ca3bf` | **sì** |

(r1, superati: runner `f33edb80…`, test `8e2a8817…`, authorization `d5ff28ad…`.)

Tutti additivi. **Nessun file J0–J11, nessun artefatto July, nessun report esistente e nessuna
configurazione di produzione è stato modificato.** FSM, corridoio morfologico, reward, SEA e plugin
C++ non toccati. `git status` mostra solo i tre file già dirty a inizio sessione.

**Pin**: 16 locali (bundle, moduli congelati, contratto, gli 8 file del leaf J11) + 6 repo
(config risolta, `osim_trj_cmc_like.py`, `rollout_eval.py`, `env_factory.py`,
`exploration_noise.py`, `asymmetric_rl_module.py`). Tutti e 22 verificati su disco.

---

## 8. Test e preflight

**Selftest: 410 check, PASS.** Copre: preregistrazione e pin; lineage e rifiuto di J8/J4/July;
gli 8 file del leaf J11 con la validità del commit; il contratto di maschera e σ; equivalenza
scientifica campo per campo con J9R1 e con J1/J3 alla sorgente, con la prova che una soglia
alterata **blocca** lo stadio; i sei casi di bordo della penetrazione; WAIT_HS diagnostico e
assente dal gate; i tre modi di fallimento silenzioso del caricamento del modulo; 18 e 20
`sim_outputs` rifiutati con asserzione sul messaggio; lock conteso; l'intera matrice a sei celle
con tutta la visibilità per cella; path leaf-relative; verifica post-commit e manomissioni; un FAIL
comportamentale che **non** ferma le celle successive; un fallimento tecnico che non committa nulla
e lascia sopravvivere una sentinella concorrente; il percorso "verifica fallita → marker + leaf
preservato"; e le garanzie statiche sul marker pre-rename, sullo scope `Exception`, sull'unico
`rmtree` e sull'assenza di break/continue.

**Aggiunti in r2** (23 check): una prova per ognuna delle otto etichette corrette, ciascuna in due
direzioni — la stringa vecchia **assente** e quella nuova **presente**; la descrizione argparse
letta via **AST** invece che per substring; la ritenzione asserita di ognuno dei cinque riferimenti
J8/J9 genuinamente storici; la policy di mutazione dell'ambiente (`episode_start_offset_s` unico
campo scientifico mutabile, `output_dir` strumentazione, vecchia formulazione ambigua rimossa) più
la verifica AST che `cell_env_config` nomini esattamente quelle due chiavi; e le quattro
asserzioni sul blocco multistart corretto.

**Preflight inerte: GO**, nessun blocker, **torch e ray assenti da `sys.modules` prima e dopo**,
nessuna primitiva di scrittura chiamata (verificato con monkeypatch), sentinella mai creata.

Rifiuti di argomento verificati: `--run` senza stage token e con il token J9R1 falliscono entrambi
fail-closed. Nessun `j12_runs`, lock, staging o sentinella.

---

## 9. Verdetto readiness

**PRONTO.** Nessun blocker.

Da tenere presente prima di un eventuale GO: l'inventario del leaf sarà definitivo al momento della
scrittura, e uno stadio successivo lo pinnerà esattamente.

---

## 10. Comando che aspetta il tuo GO

```
cwd:         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter: /opt/anaconda3/envs/envCMC-rllib/bin/python
argv:        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j12_closed_loop.py
             --run
             --authorized-stage V26C_J12_CLOSED_LOOP_QUALIFICATION
             --out <repo>/Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j12_runs/j12_closed_loop_v26c_2026-08-27_r1
```

`--no-progress` è l'unica variazione ammessa e cambia solo lo stdout. `OUTPUT_ROOT_OVERRIDE` deve
restare non impostata.

---

## 11. TODO propagati

- **LOTO** — non integrato. TODO futuro, non J12.
- **LOCO** — non integrato. TODO futuro, non J12.
- **B1R1** — non integrato. TODO futuro, non J12.
- **B1R2** — non integrato. TODO futuro, non J12.
- **Epic generalizzazione multi-modello** — APERTO. Questa fase copre ancora AB06 e tre start dello
  stesso trial; non dimostra alcuna generalizzazione.
- **Critic warm-up** — NON parte di questa fase e non autorizzato da essa. Dovrà costruire un
  modulo fresco `inference_only=False` e trapiantare l'attore: `class_and_ctor_args.pkl` fissa
  `inference_only=True`, quindi `from_checkpoint` restituirà sempre un modulo actor-only. Serviranno
  tutte e dieci le chiavi attore, alias inclusi.
- **PPO** — non autorizzato. Un PASS closed-loop è una precondizione, mai un permesso.
- **J9R1 FAIL sulla cella B** — è la domanda a cui questa fase risponde. Finché non è eseguita, non
  è risposta.
- **Trade-off recovery di J11** — J11 è dello 0.59% peggiore di J8 sul blocco recovery. Se il
  closed-loop dovesse fallire, quel dato è il primo posto dove guardare.
- **`nominal_mean_shift` 0.0264 di J11** contro la soglia offline July di 0.005 — segnalato nel
  report J11; questa fase non vi attacca alcun gate.
- **WAIT_HS senza soglia** — deliberato. Se vuoi che diventi vincolante, è una decisione
  architetturale che richiede la sua preregistrazione.
- **Difetti di J9R1 ereditati come evidenza** — la sua `DIAGNOSTIC_NOT_BINDING` auto-contraddittoria
  e l'assenza di verifica post-commit restano nel suo leaf immutabile. Corretti qui, non là.

---

## 12. Cosa questa fase **non** ha fatto

- **Nessun rollout.** Nessun ambiente costruito, resettato o steppato. Nessun leaf creato.
- Nessun critic, PPO, cluster Ray o env runner.
- L'autorizzazione è scritta ma dichiara `execution_permitted_now: false`.

**Fermo in attesa del tuo GO.**
