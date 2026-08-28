# V26C J10 — raccolta teacher multistart, readiness

**Token proposto:** `V26C_J10_MULTISTART_TEACHER` · **Data:** 2026-08-27
**Stato: PRONTO PER L'AUTORIZZAZIONE INTERNA DELL'ARCHITETTO. Nessun rollout eseguito.**
Solo tooling, test sintetici e preflight inerte. Nessun fit, critic, PPO, Ray cluster o rollout
dello student.

---

## 1. Problema

La qualifica closed-loop J9R1 ha dato **FAIL** sulla cella B, la partenza `−0.20 s`: 0 cicli
validi, 0 heel strike, FSM ferma in `WAIT_HS` per 500/500 step. La causa misurata non è la FSM né
un difetto di misura, ma un **support gap del dataset di training**:

- nel dataset J7 (16713 righe) il one-hot FSM assume **due soli valori** — `(0,1,0)`
  STANCE_AFTER_HS su 12486 righe e `(0,0,1)` SWING_AFTER_TO su 4227 — e **`(1,0,0)` WAIT_HS
  compare 0 volte**;
- la cella B vive per 500/500 step esattamente in `WAIT_HS`, cioè in uno stato su cui l'attore
  non è mai stato addestrato;
- fuori supporto: B 500/500 step (100%), contro A 1.4% e C 12.2%;
- il dataset J7 non contiene alcuna riga da uno start non nominale (tutte le tracce sorgente
  hanno `episode_start_offset_s` = 1.956870984).

Sotto il contratto canonico di training (`exact_start_sampling: true` con i tre offset) il
training campionerebbe anche `−0.20 s`: finché B non è coperta, il checkpoint **non è
training-ready**.

## 2. Soluzione — decisione dell'architetto

**Decisione dell'architetto, applicata senza alterazioni:** usare la **metodologia multistart di
luglio, esclusivamente sulla lineage August V26.** Nessuna architettura alternativa è stata
proposta o valutata.

J10 raccoglie **esattamente due rollout prescribed-teacher** da 500 step agli offset
`1.756870983805102` (−0.20 s) e `2.156870983805102` (+0.20 s), nell'ordine **B poi C**, seed 123,
σ = 0, lookahead 0. Il nominale **non** viene riraccolto: esiste già come leaf J1.

### Metodologia di luglio, ricostruita e riusata (solo come metodo)

Audit read-only dei due leaf di luglio (`validation/controller_memory_ablation/
2026-07-13_markov35_teacher_start_{minus,plus}020`) — nessun artefatto di luglio è parent, dato o
output di questo stadio:

- **azione teacher = riferimento prescritto**, `prescribed_teacher_action` su `base.base_kin`,
  cioè lo spline IK; **nessun checkpoint viene interrogato**;
- **σ esattamente 0**, `lookahead 0.0`, `hold_steps 1`; `actions == executed_actions` bit per bit;
- fra i due config risolti di luglio **differiva una sola chiave**, `episode_start_offset_s`
  (diff: un solo hunk);
- gate di raccolta a 5 campi (episodio pieno, non terminato, ≥1 ciclo valido, penetrazione sotto
  la guardia, dataset finito), **senza alcun criterio specifico per l'offset**;
- combinazione successiva con `--recovery-dataset` ×2 e `repeat 8` → 4000+4000 righe su 24712;
- la selezione di luglio giudicò l'attore sui **rollout closed-loop ai tre start**, non sulla
  metrica offline. Il refinement con shift nominale migliore fu **rifiutato** perché moriva allo
  start −0.20 s.

J10 riusa il metodo e **irrigidisce** dove serve: la raccolta di luglio lasciò
`teacher_sim_outputs/` **vuota**; J10 richiede 19 `sim_outputs` per cella, come il leaf J1.

## 3. Strategia e contratto

- **Parent operativo unico:** l'imitativo August V26,
  `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/rl_module_best`, `module_state`
  `0ba56eb7…` — **lo stesso parent che J1 pinna** (`v26c_j1_collect.py:66`). Fissa la lineage e
  **non è mai la sorgente delle azioni**; non viene mai caricato.
- **Lo student J8 è escluso** come parent, come policy e come sorgente di etichette, con rifiuto
  esplicito nel runner e nel receipt.
- **Config pinnata** `a870cc38…` attraverso il builder J1 riusato **immodificato**; FSM v3,
  corridoio, reward, SEA e produzione **intatti**.
- **Muta solo** `episode_start_offset_s` e `output_dir`: chiavi stabili confrontate una per una
  contro la base già verificata, con `stable_keys` che esclude **entrambe** le chiavi mutabili.
- **Nessun RLlib**: la sorgente è prescritta, quindi nessun modulo di policy viene caricato e
  nessun Ray cluster / worker / EnvRunner esiste.
- **Reset esatto**: offset **relativo a `cfg.t_start`**; atteso `11.99 + offset`, cioè
  `13.746870983805103` (B) e `14.146870983805101` (C), tolleranza 1e-9 s, verificato subito dopo
  il reset contro i valori **live** dell'ambiente.

### Artefatti salvati per cella

`teacher_dataset.npz` con **le sei chiavi J1** — `observations` (obs35 grezze), `actions`
(teacher prescritto), `executed_actions`, `action_noises`, `times`, `actor_feature_names` — più
`trace.json`, `kinematics.npz`, `penetration.npz` e la directory `sim_outputs` (19 file).

### Gate

**Per cella, vincolanti:** 500 step; `episode_time_limit`; ≥2 cicli validi; timeout stance e swing
= 0; morphology causal failure = 0; `hs_cancelled` = 0; `resync` ≤ 1; **action clipping = 0**;
gate cinematico **identico a J9R1** (`ankle_min ≤ −0.03`, ROM caviglia ≥ 0.30, ROM ginocchio
≥ 0.60, ginocchio strettamente flesso, bounds invariati); penetrazione sotto contratto unico —
**20 mm soft diagnostica, ≥25 mm July diagnostica, >28 mm unico hard binding, esattamente 28
passa**.

Il clipping è vincolante qui (in J1 era diagnostico): un'azione clippata significherebbe che
l'etichetta registrata non è quella eseguita.

**Coverage gate, vincolante e a livello di raccolta:** l'unione delle due celle deve contenere
**almeno una riga con `phase_fsm_wait_hs == 1`** — esattamente il gap misurato in J7. Nessuna
soglia ulteriore è inventata: nessun minimo oltre uno, nessuna frazione, nessun requisito per
cella. La feature è risolta **per nome**.

**Nessun fail-fast comportamentale:** un FAIL in una cella è preservato e l'altra cella gira
comunque. Solo un'eccezione tecnica ferma la raccolta, fail-closed.

## 4. Correzioni di provenance obbligatorie, applicate

1. **Nessun path di staging nel receipt.** Ogni percorso registrato è **leaf-relative**
   (`j10_cell_B_teacher_dataset.npz`, …) e quindi valido dopo il commit atomico. Il test asserisce
   che la stringa `.staging` non compare in nessun punto del receipt e che ogni percorso
   registrato **risolve** nel leaf committato. Corregge il difetto del receipt J9R1.
2. **`valid_cycle_count` non è più fra i non-binding.** `valid_cycles` è vincolante, quindi la
   sua misura non compare fra i diagnostici; lo stesso vale per `action_clipped_steps`, ora
   vincolante. La lista diagnostica è `episode_return`, `realized_noise_rms`,
   `max_reserve_norm_nm`, `mean_reserve_norm_nm`, `slew_limited_steps`. `verify_prereg` **rifiuta
   fail-closed** una preregistrazione che violasse questa regola.

## 5. Un finding bloccante trovato dall'audit parallelo

Il manifest del parent dichiara **39** feature (slice June-lineage, che include quattro
`healthy_*_imitation_target[_vel]`), mentre il blocco actor runtime è **35**. La mia prima stesura
leggeva i nomi dal manifest e **sarebbe fallita al preflight**. J1 li prende dall'**ambiente vivo**
(`v26c_j1_collect.py:621`) ed è ciò che J10 ora fa, con il fatto 39-vs-35 registrato esplicitamente
invece che subito. Il parent è pinnato **per sola provenance** e non viene mai caricato, quindi la
differenza di larghezza non può influenzare alcun valore registrato in questo stadio.

## 6. File creati

| file | sha256 |
|---|---|
| `v26c_j10_prereg_multistart_teacher.json` | `79b1b573eb45831e7333c3dbc539f76a7e9555986b38e73ea83a91590f8241d7` |
| `v26c_j10_multistart_teacher.py` | `4d436aceaf0d78aee269f24cf90d47ca81877eecd49aeb155768b0613e35fa18` |
| `test_v26c_j10_multistart_teacher.py` | `fc1cf58721f84302b5a8f3665133cefde8d27ff130108189be7b39bec1ae1ba5` |
| `v26c_j10_multistart_teacher_authorization.json` | `d6a3fddd6fb1681fd65a49e0e1bdc5f7880c441d6ca059c941fc573f27a5346f` |
| `reports/user/2026-08-27_v26c_j10_multistart_teacher_readiness.md` | questo report |

Tutti additivi. Nessun artefatto J0–J9R1, sorgente di luglio, report o configurazione di
produzione è stato modificato.

## 7. Test e verifiche

- `py_compile` exit 0 su runner e test J10 e su J1/J3.
- **Selftest J10 sintetico: PASS, 158/158.** Copre: pin e preregistrazione; parent e rifiuto
  dello student; il fatto 39-vs-35; mutazione solo offset+output_dir con `stable_keys`;
  aritmetica di reset e clamp; ogni criterio del gate incluso il clipping vincolante;
  `ankle_min` −0.0099 FAIL vs esattamente −0.03 PASS; penetrazione 0.028 PASS e 0.0280001 FAIL;
  il coverage gate con 0 righe (FAIL), 1 riga (PASS) e scope collection-wide; preflight inerte con
  ogni primitiva di scrittura e ogni import pesante armati; la raccolta completa a due celle su
  stack sintetico con verifica delle sei chiavi del dataset, `actions == executed_actions`,
  rumore esattamente zero, 19 sim_outputs, path leaf-relative risolvibili, **FAIL di coverage che
  committa comunque l'evidenza**, e fallimento tecnico che non committa nulla preservando il
  contenuto concorrente.
- Regressioni applicabili: J7 builder **PASS 190/190**, penetration contract **PASS 141/141**.
- **Preflight J10 in processo fresco: GO**, exit 0, nessun blocker, 13 pin verificati, sentinella
  **non creata**, `heavy_modules_introduced_by_preflight = []`.
- Snapshot 360 file identico prima e dopo il preflight; `j10_runs`, lock, staging e sentinella
  **assenti**.

**Nota, non regressione:** `test_v26c_j9r1_closed_loop.py` ora fallisce perché asserisce che il
proprio preflight sia GO, precondizione decaduta quando il leaf J9R1 è stato committato. È la
stessa condizione **spent-stage** già osservata per J7-materialize e J8; non l'ho toccata.

## 8. Comando congelato (candidato, NON eseguito)

```
cwd         /Users/tommy/Documents/CMC-like-Simulator - Claude
interpreter /opt/anaconda3/envs/envCMC-rllib/bin/python
argv        Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j10_multistart_teacher.py
            --collect
            --authorized-stage V26C_J10_MULTISTART_TEACHER
            --out /Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j10_runs/j10_multistart_teacher_v26c_2026-08-27_r1
```

Verificato: token == `STAGE` del runner, `--out` == `authorized_leaf()`, script risolvibile dalla
cwd congelata, `--no-progress` non passato, `OUTPUT_ROOT_OVERRIDE` deve restare non impostato.
`execution_permitted_now: false` finché l'architetto non autorizza internamente.

## 9. TODO propagati

- **LOTO** — non integrato, TODO futuro.
- **LOCO** — non integrato, TODO futuro.
- **B1R1** — non integrato, TODO futuro.
- **B1R2** — non integrato, TODO futuro.
- **Generalizzazione / Dataset Epic** — aperto: la raccolta copre AB06 e tre start dello stesso
  trial; non dimostra generalizzazione a soggetti, trial o velocità diverse.
- **Il fit che consumerà questi dati** — **non fa parte di questo stadio**. Combinare queste righe
  con J7 e rifare il fit è uno stadio separato, con preregistrazione e autorizzazione proprie. Le
  proporzioni di luglio (repeat 8 sul multistart) sono registrate come metodologia, non come
  decisione già presa.
- **Difetti documentali noti** — il manifest sidecar del leaf J8 descrive i pesi J2 (conseguenza
  della regola "sidecar byte-identici al parent"); il canale legacy a 20 N etichetta come
  "heel_strike" tocchi di solo avampiede; i path `sim_outputs` del receipt J9R1 puntano allo
  staging rinominato. Nessuno invalida un numero; J10 corregge l'ultimo per sé.
- **Correzione documentale unità** — vari record V26C congelati dicono "rad" dove gli artefatti
  primari dicono **secondi**.

## 10. Stato

Fase di tooling/readiness **conclusa**. Nessun rollout eseguito, nessun leaf creato, nessuna
autorizzazione di esecuzione attiva. **STOP per la revisione e l'autorizzazione interna
dell'architetto.**
