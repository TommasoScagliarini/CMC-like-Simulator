# Qualifica closed-loop J9R1 dell'attore J8

**Token:** `V26C_J9R1_CLOSED_LOOP` · **Data:** 2026-08-27 · **Esecuzione unica, nessun retry.**
**Esito aggregato: FAIL** — 5/6 celle comportamentali PASS, 6/6 telemetria valida.
Evidenza committata integralmente. Nessun critic, nessun PPO, nessuna promozione, nessuna scelta.

Il report distingue esplicitamente **FATTI** (letti dagli artefatti), **INFERENZE** (sostenute ma
un passo oltre il dato) e **RACCOMANDAZIONI** (opzioni, non decisioni).

---

## 1. Problema

La qualifica J9 del 2026-08-26 si era interrotta con un **fallimento tecnico**, non
comportamentale. Il runner J9 costruiva la configurazione dell'ambiente tramite il builder J1
irrigidito passando `output_dir=None`. In quel ramo il builder imposta `record_outputs=False` e
`save_outputs_on_close=False` e omette del tutto `output_dir` e `output_prefix`
(`v26c_j1_collect.py:325-330`). Ogni cella rimetteva soltanto `output_dir`
(`v26c_j9_closed_loop.py:369`), lasciando i due flag a False; `osim_trj_cmc_like.close()` non
chiamava quindi mai `runner.save_results()` (`osim_trj_cmc_like.py:2048-2053`) e il controllo
post-close che pretendeva i `sim_outputs` (`v26c_j9_closed_loop.py:916-917`) non poteva essere
soddisfatto.

La cella A aveva completato i 500 step e chiuso correttamente; l'eccezione ha impedito ritorno,
logging e commit. **Nessun verdetto comportamentale fu restituito, osservato, persistito o
validabile**, e nessuna inferenza su di esso è ammissibile. Il tentativo è conservato in
`v26c_j9_technical_failure_2026-08-26.json` con i tre log verbatim.

## 2. Correzione J9R1

`J9R1` è uno **stadio additivo separato**, non un retry: token, leaf e receipt propri. Gli
artefatti J9 originali non sono stati riscritti né modificati e restano pinnati come evidenza.

La correzione è **di sola strumentazione**: la base viene costruita con una radice di output
**reale**, come fanno J1, J3, J5 e J6, così che ogni cella erediti `record_outputs=True`,
`save_outputs_on_close=True`, `output_dir` e `output_prefix`. I due flag sono verificati
fail-closed **due volte** — dentro `base_env_config` e di nuovo nel punto di chiamata di
`run_matrix` — quindi una base con registrazione spenta non può più raggiungere un ambiente. La
registrazione è neutra rispetto al risultato simulato (`simulation_runner.py:486-487` è un append
su buffer) ma non è gratuita in RAM e I/O: il rischio residuo è registrato nell'amendment.

Corretta anche la guardia sulle chiavi stabili: `stable_keys = set(base) − {episode_start_offset_s,
output_dir}`, con uguaglianza esatta su tutte e sole quelle. La forma J9 escludeva solo l'offset e
avrebbe fallito ogni cella contro una base con registrazione attiva.

**Verifica dell'efficacia (FATTO):** ogni cella ha prodotto **19 file** `sim_outputs`, lo stesso
conteggio del leaf J3. Il difetto è chiuso.

## 3. Strategia

Sei celle dichiarate in anticipo, eseguite come **una sola matrice senza fail-fast
comportamentale**: un FAIL in una cella viene preservato e le altre proseguono. Solo un errore
tecnico o di integrità ferma la matrice, fail-closed e senza commit.

L'equivalenza scientifica con J9 è provata **campo per campo**: il runner J9R1 calcola l'hash del
runner J9 originale, lo rifiuta se difforme, lo importa e confronta 24 costanti — MATRIX,
EXPECTED_STEPS, SIGMA, SIGMA_TOLERANCE, NOISE_HOLD_STEPS, i due gate, i diagnostici, tolleranza e
semantica di reset, bande di penetrazione, pin. **Tutte uguali.** Differiscono solo STAGE, leaf e
nome del receipt, come devono.

## 4. Lineage — August V26

- **Attore:** unico, `j8_runs/j8_recovery_fit_v26c_2026-08-26_r1`, `module_state`
  `9c5b157156e6b9c2a69a16f14908d6750ac6acdad95516eba9ac9378912dbc82`, caricato in sola lettura e
  **verificato byte-identico prima e dopo** la matrice.
- **Provenienza:** parent J2 `0f182ea9…` + dataset J7 corrente `bb9b21f0…` (16713 righe).
- **Luglio:** solo metodologia ed evidenza. Nessun peso, dato o etichetta di luglio entra qui.
- **Attore 35D unico:** colonne clock 0:2 esattamente zero, colonne controller 25:35 vive.
  Nessun 25D autonomo, nessun widening, nessun input controlaterale.
- **σ = 0.004999999670722372** dalla testa log-std congelata dell'attore, mai modificata.

## 5. Risultati per cella

Offset in **secondi**, relativi a `cfg.t_start = 11.99`. Reset verificato subito dopo il reset:
errore **esattamente 0.0 s** in tutte e sei le celle, nessun clamp.

| Cella | modo | seed | offset s | reset assoluto | verdetto | return | cicli | HS | TO | pen max |
|---|---|---|---|---|---|---|---|---|---|---|
| A | deterministic | 123 | 1.956871 (nominale) | 13.946870984 | **PASS** | 42.500 | 2 | 3 | 3 | 24.249 mm |
| B | deterministic | 123 | 1.756871 (−0.20 s) | 13.746870984 | **FAIL** | −0.068 | 0 | 0 | 0 | 0.000 mm |
| C | deterministic | 123 | 2.156871 (+0.20 s) | 14.146870984 | **PASS** | 47.286 | 2 | 3 | 3 | 23.388 mm |
| D | stochastic_held | 123 | 1.956871 | 13.946870984 | **PASS** | 42.403 | 2 | 3 | 3 | 24.472 mm |
| E | stochastic_held | 124 | 1.956871 | 13.946870984 | **PASS** | 42.706 | 2 | 3 | 3 | 23.891 mm |
| F | stochastic_held | 125 | 1.956871 | 13.946870984 | **PASS** | 42.390 | 2 | 3 | 3 | 25.135 mm |

Tutte le celle: **500 step**, `end_reason = episode_time_limit`, telemetria valida.

### Cinematica (rad)

| Cella | knee min/max | knee ROM | ankle min/max | ankle ROM |
|---|---|---|---|---|
| A | −0.9943 / −0.1649 | 0.8294 | −0.1093 / +0.3991 | 0.5084 |
| B | −1.3259 / **−0.4517** | 0.8742 | −0.0298 / +0.1855 | **0.2153** |
| C | −0.9916 / −0.1369 | 0.8547 | −0.1330 / +0.4060 | 0.5389 |
| D | −0.9954 / −0.1645 | 0.8309 | −0.1106 / +0.4005 | 0.5111 |
| E | −0.9922 / −0.1676 | 0.8246 | −0.1090 / +0.4006 | 0.5095 |
| F | −0.9961 / −0.1616 | 0.8345 | −0.1074 / +0.4003 | 0.5077 |

Criteri vincolanti falliti dalla sola cella B: `valid_cycles` (0 < 2), `kinematic_ankle_min`
(−0.029781 > −0.03) e `kinematic_ankle_amplitude` (0.2153 < 0.30).

## 6. Soglie di penetrazione 20 / 25 / 28 mm

Autorità unica: `v26c_penetration_contract_2026-08-26.json` (`95a47d53…`). Nessuna soglia locale è
scritta nel runner.

| Cella | max | >20 mm (diagnostica) | ≥25 mm (July, diagnostica) | >28 mm (unico vincolante) | esito |
|---|---|---|---|---|---|
| A | 24.249 mm | 114 | 0 | 0 | PASS |
| B | 0.000 mm | 0 | 0 | 0 | PASS |
| C | 23.388 mm | 96 | 0 | 0 | PASS |
| D | 24.472 mm | 109 | 0 | 0 | PASS |
| E | 23.891 mm | 111 | 0 | 0 | PASS |
| F | 25.135 mm | 106 | 2 | 0 | PASS |

**Nessuna cella supera i 28 mm.** Solo F tocca la banda July (2 campioni ≥ 25 mm), diagnostica.
Le celle che camminano penetrano **meno del parent J2** in closed-loop (J3 27.05 mm, J5-r2
26.91 mm, J6 26.68–27.29 mm).

**Avvertenza (INFERENZA):** lo `0.000 mm` della cella B è un PASS **vacuo**, non un merito. Il
piede non ha mai raggiunto il piano di contatto del modello applicato; il numero è corretto ma in
quella cella non misura nulla.

## 7. Contatori FSM v3

`phase_timeout_stance`, `phase_timeout_swing`, `morphology_causal_contract_failure`,
`hs_cancelled_count`, `resync_count`, `action_clipped_steps`: **tutti 0 in tutte e sei le celle**,
B compresa.

**Interpretazione corretta (FATTO + INFERENZA):** per la cella B questi zeri sono **vacui**. La
FSM è rimasta in `WAIT_HS` per 500/500 step, con 0 transizioni. Un timeout di stance o di swing,
un heel-strike cancellato o un resync sono eventi che possono verificarsi **solo dopo** che una
stance è iniziata: in `WAIT_HS` non c'è nulla che possa andare in timeout o essere cancellato.
Gli zeri non attestano quindi salute della FSM in B — attestano che la macchina non è mai partita.
È esattamente `valid_cycles` (0 < 2) il criterio che impedisce a questa configurazione di
produrre un falso PASS: senza di esso una cella immobile e senza contatto passerebbe tutti gli
altri contatori.

## 8. Diagnosi della cella B — evidenze

### 8.1 Non è vero che "non c'è stato alcun contatto"

La formulazione precedente era **errata** e va corretta. I numeri primari, letti da
`j9_cell_B_trace.json`:

| grandezza | chiave | valore |
|---|---|---|
| GRF principale sinistra, max | `online_grf.left.normal_force` | **0.000376 N** |
| GRF principale sinistra, step in contatto | `online_grf.left.in_contact` | **0 / 500** |
| stesso file `.sto` (5000 righe) | `left_in_contact` | **0 / 5000** |
| detector **tallone**, max | `online_grf_detector.sensors.left_heel.normal_load_n` | **1.558e-09 N** |
| detector tallone, step in contatto | `...left_heel.in_contact` | **0 / 500** |
| detector **avampiede**, max | `...left_toe.normal_load_n` | **160.020 N** |
| detector avampiede, step in contatto | `...left_toe.in_contact` | **66 / 500** |
| campioni binari `true` (qualunque tipo) | `binary_phase_sensor_samples` | **89 / 5000**, tutti toe, **0 heel** |

**FATTO:** il piede sinistro **ha caricato il terreno**, ma **solo di avampiede**: quattro
tocchi, picco 160.020 N (≈0.21 BW), ~0.089 s di contatto geometrico dell'avampiede. Il **tallone
non è mai arrivato a distanza misurabile**: penetrazione esattamente 0 e forza ≤ 1.56e-09 N su
tutti i 500 step.

### 8.2 Perché la FSM resta in WAIT_HS

**FATTO:** `phase_fsm.state_name` = `WAIT_HS` per 500/500 step, `expected_next_event` =
`heel_strike`, **0 transizioni**. Il contratto della FSM
(`binary_phase_fsm_v26.py`, `binary_point_v25+heel_qualified_fsm_v2`) stabilisce che una stance
può iniziare solo da uno stato grezzo `CONTACT_HEEL` o `CONTACT_BOTH`: *toe-only resta una
diagnostica grezza e non può iniziare una stance*. La cella B ha prodotto **solo parole TOE**
(0 campioni heel-true su 5000), quindi nessun heel strike poteva essere qualificato. La FSM ha
funzionato **esattamente come specificato**.

### 8.3 Non è un difetto di misura né di reset

Ogni ipotesi di difetto è esclusa positivamente (FATTI):

1. **Reset corretto.** `reset_check`: atteso 13.746870983805103, osservato 13.746870983805103,
   errore **0.0 s**, nessun clamp, verificato subito dopo il reset contro `cfg.t_start + offset`.
2. **Sensori armati e vivi.** `binary_phase_fsm.samples_processed` arriva a **5000** come in A e
   C; `binary_phase_fsm_executed` True su 500/500; baseline catturata al reset. Il sensore del
   tallone e la GRF principale assumono **500 valori distinti** nei 500 step: si muovono con
   l'arto, non sono congelati, azzerati o NaN.
3. **I due modelli di contatto sono diversi per costruzione, con offset misurato di 24.3 mm.** Il
   piano della GRF applicata sta a y = −0.009503954760384832; il piano del detector a
   y = +0.0148208231, **0.0243248 m più in alto**. Un tocco superficiale carica il detector mentre
   il modello applicato legge zero: è progetto, non guasto.
4. **A e C riproducono lo stesso "zero" nella stessa geometria.** Filtrando A e C agli step con
   penetrazione tallone 0 e penetrazione avampiede ≤ 0.022881 m (il massimo di B): A ha 151 step,
   C ne ha 154, e in **tutti** la GRF applicata è ≤ 0.00115 N con penetrazione 0 e `in_contact`
   False. La forza applicata supera 1 N solo da penetrazione ≥ 0.023086 m (A) / 0.022980 m (C):
   **B si ferma 1.44 mm sotto**. Un sensore difettoso non può produrre, in due celle che passano,
   esattamente la lettura che produce in B per la stessa geometria.
5. **Telemetria auto-consistente.** `telemetry_integrity.pass = true`,
   `qualification_technically_valid = true`: il FAIL di B è registrato come **comportamentale**,
   non tecnico.

**INFERENZA (ben sostenuta):** lo zero di eventi e di penetrazione della cella B è
**comportamento reale**, non un difetto di misura o di reset. La catena causale misurata è:
politica che al reset −0.20 s converge su un ginocchio permanentemente flesso (mai oltre
−0.4517 rad) con caviglia quasi ferma (ampiezza 0.215 rad) → arto effettivo troppo corto e tibia
troppo inclinata perché il tallone raggiunga il suolo → contatto solo di avampiede e superficiale
(22.88 mm, 1.44 mm sotto la soglia del modello applicato) → GRF applicata 0 e `max_penetration_m`
esattamente 0 → la FSM heel-qualified non vede mai HEEL né BOTH → resta in `WAIT_HS` → 0 HS,
0 TO, 0 cicli. Ogni anello è misurato; solo "la politica converge su un attrattore a ginocchio
flesso" è interpretazione della cinematica.

### 8.4 Il support gap di J7, misurato

**FATTO.** Confrontando la prima osservazione di ogni cella con il range del dataset J7
(16713 righe, 1210 stati unici):

| feature | B iniziale | J7 min | J7 max | scarto |
|---|---|---|---|---|
| `phase_fsm_wait_hs` | **1.0** | **0.0** | **0.0** | categoricamente nuovo |
| `pros_knee_angle_vel` | 4.010993 | −2.706446 | 2.780337 | +1.2307 (z ≈ +2.71) |
| `SEA_Knee_motor_speed` | 4.010993 | −4.742242 | 2.844554 | +1.1664 (z ≈ +2.39) |
| `pros_knee_angle_served_ref_vel` | 4.010993 | −2.598257 | 2.764310 | +1.2467 (z ≈ +2.72) |

Il dato più forte è categorico: nel dataset J7 il one-hot FSM assume **due soli valori** —
`(0,1,0)` STANCE_AFTER_HS su 12486 righe (74.7%) e `(0,0,1)` SWING_AFTER_TO su 4227 (25.3%).
**`(1,0,0)` WAIT_HS compare 0 volte.** L'attore non è mai stato addestrato su un singolo campione
in stato WAIT_HS, cioè esattamente lo stato in cui la cella B vive per 500 step su 500.

**FATTO.** Estendendo a tutto l'episodio, step fuori supporto (escluse le colonne clock, che sono
bit-zero nei pesi e quindi inerti): A 7/500 (1.4%), C 61/500 (12.2%), D 8, E 8, F 9 — **B
500/500 (100%)**, con `phase_fsm_wait_hs` fuori range in tutti i 500. A e C sono **interamente
dentro** il supporto al reset; solo B è fuori.

**FATTO.** Il dataset J7 non contiene alcuna riga da uno start non nominale: tutte le tracce
sorgente (J1, J3, J6×3) hanno un unico `episode_start_offset_s` = 1.956870984. Il receipt lo
dichiara (`multistart: "OMITTED / DEFERRED"`) e la composizione chiude senza resto
(500×32 + 713 = 16713).

**INFERENZA:** la causa primaria del FAIL di B è l'**omissione dei dati multistart** dal dataset
di training. È la stessa classe di problema che a luglio fu affrontata aggiungendo 8000 righe
teacher agli offset ±0.20 s. **Luglio resta solo metodologia**: nessun peso, dato o etichetta di
luglio è o sarà usato come lineage operativa; il riferimento è al *metodo*, non all'artefatto.

## 9. Difetti documentali del receipt (non invalidanti)

Due incoerenze **di sola documentazione**, che non toccano numeri, gate né verdetti (FATTI):

1. **Percorsi `sim_outputs` che puntano allo staging rinominato.** Ogni `cells[i].sim_outputs`
   contiene una stringa che punta a
   `…/j9r1_runs/.staging_j9r1_closed_loop_v26c_2026-08-26_r1/j9_cell_<ID>_sim_outputs`. Quel
   percorso **non esiste più**: lo staging è stato rinominato atomicamente sul leaf finale. I file
   esistono, 19 per cella, nel leaf committato. La stringa è stata formata prima del rename.
2. **`valid_cycle_count` compare sia come vincolante sia fra i diagnostici.** È correttamente
   vincolante in `gate.criteria.common.valid_cycles_min` e in `gate.checks.valid_cycles` (dove per
   B vale `false`), **ed è anche elencato** in `DIAGNOSTIC_NOT_BINDING` e quindi in
   `diagnostics_not_binding`. La contraddizione è **apparente e nominale**: i due nomi sono
   diversi (`valid_cycles` è il check, `valid_cycle_count` è la misura ripetuta fra i diagnostici)
   e il gate ha correttamente fatto fallire B su `valid_cycles`. Resta però una formulazione
   ambigua da correggere in una futura revisione del runner.

Un terzo rilievo, **fuori dal receipt** (FATTO): il manifest
`j8_runs/…/rl_module/actor_feature_manifest.json` è **byte-identico a quello di J2** — conseguenza
diretta della regola J8 "sidecar byte-identici al parent". Dichiara quindi
`module_state_sha256: 0f182ea9…` (che è J2), `actor_label: J2_BASE35_JULY_FAITHFUL` e una maschera
controller `active: true` sulle colonne 25–34, mentre i pesi J8 accanto hanno hash
`9c5b1571…` e quelle colonne **vive**. J9R1 legge da quel file **solo** `actor_feature_names`
(identici fra J2 e J8), quindi nessun contratto è stato compromesso; ma il file descrive pesi
diversi da quelli che accompagna.

## 10. File e contenuto del leaf

Leaf: `j9r1_runs/j9r1_closed_loop_v26c_2026-08-26_r1`.

**Conteggio corretto (FATTO):** **25 entry di primo livello** = 19 file + 6 directory. In
ricorsivo il leaf contiene **133 file**: 1 receipt + 18 fra trace e npz (3 per cella) + **114 file
di simulazione** nelle sei directory `sim_outputs` (102 `.sto` + 12 `.csv`, 19 per cella).
La formulazione precedente "il leaf contiene 25 file" era errata.

| file | sha256 |
|---|---|
| `v26c_j9r1_closed_loop_authorization.json` | `fc51bc05cb38b7c75664cb439e857d59baff6a7e15ffecfe5f869ad99ba9b42f` |
| `…/v26c_j9r1_closed_loop_receipt.json` | `c201b666f68451e4646fbaae3f149e9afaf873b5c94f99c95fc4286faaeda4f4` |
| `reports/user/2026-08-27_qualifica_closed_loop_j9r1.md` | questo report |

Immutati e riusati: `v26c_j9r1_recording_amendment.json` (`cf84b474…`),
`v26c_j9r1_closed_loop.py` (`f58b0d14…`), `test_v26c_j9r1_closed_loop.py` (`6f538780…`),
`v26c_j9r1_readiness.json` (`e20b16c2…`), `v26c_j9_technical_failure_2026-08-26.json`
(`67c97df1…`) e i tre log in `j9_technical_failure_2026-08-26/`.

## 11. Test e verifiche

- `py_compile` exit 0 su runner e test J9R1 e sugli originali J9.
- **Selftest J9R1 sintetico: PASS, 360/360.** Regressione suite J9 originale: **PASS, 288/288**.
- Preflight inerte subito prima del lancio: **GO**, nessun blocker, 23 pin dell'amendment
  verificati, 24 costanti scientifiche uguali a J9, sentinella **non creata**,
  `heavy_modules_introduced_by_preflight = []`.
- Comando eseguito **una sola volta**, esattamente come congelato, senza aggiungere o togliere
  flag; progresso visibile cella per cella.
- Dopo la run: nessun lock, staging o sentinella residua; attore J8 byte-identico; runner,
  amendment, `osim_trj_cmc_like.py` e `rollout_eval.py` invariati.
- `stderr`: solo `RuntimeWarning` benigni di SLSQP e `DeprecationWarning` RLlib.
- Receipt: `fit_executed false`, `critic_touched false`, `ppo_updates 0`, `actor_edited false`,
  `ray_cluster_started false`, `ray_workers_started false`, `env_runners false`.

## 12. Implicazioni — il checkpoint non è training-ready

**FATTO.** Il contratto canonico di training, `training_exnovo_cfg.yaml`, usa
`exact_start_sampling: true` con

```yaml
episode_start_offset_choices_s:
  - 1.756870983805102   # -0.20 s
  - 1.956870983805102   # nominale
  - 2.156870983805102   # +0.20 s
```

Gli stessi tre valori sono nella configurazione runtime pinnata usata da J9R1
(`MLP_ExNovo_B0820_fsmv3_fixedcorridor_50iter/training_cfg.resolved.yaml`, `a870cc38…`), e il
diff non tracciato del file canonico non tocca queste chiavi.

**Conseguenza (INFERENZA diretta):** un training sotto quel contratto campiona **anche lo start
−0.20 s**, cioè esattamente la cella che fallisce. Finché B non è corretta, **il checkpoint non è
training-ready sotto il contratto canonico**, e **un waiver limitato al nominale non è
sufficiente**: non si tratta di una cella accessoria ma di uno dei tre start che il training
userebbe. L'aggregato preregistrato è 6/6; 5/6 non è un pass parziale.

**Cosa il risultato dice comunque (FATTO):** l'attore J8 qualifica in closed-loop sul nominale e
su `+0.20 s`, in deterministico e sui tre seed stocastici a σ = 0.005, con FSM v3 pulita,
penetrazione sotto la banda vincolante e migliore del parent J2.

## 13. Opzioni minime verso training-ready — RACCOMANDAZIONI, non scelte

Nessuna di queste è stata avviata, scelta o preferita. Nessuna inventa architetture nuove.

1. **Colmare il support gap con dati multistart** (metodo già validato a luglio, qui solo come
   metodologia): raccogliere righe teacher agli start ±0.20 s e rifare il fit. È l'opzione che
   attacca la causa primaria misurata — 0 righe WAIT_HS nel training set. Costo: una raccolta
   più un nuovo stadio di fit, entrambi da preregistrare e autorizzare.
2. **Estendere il dataset alle sole condizioni WAIT_HS** senza raccolta multistart completa, se
   esiste già materiale in quello stato. Da verificare prima: al momento non risulta.
3. **Rivedere il contratto di start del training** (ad es. curriculum sui tre start) — è una
   modifica di contratto, non di architettura, e spetta interamente all'architetto e all'utente.
4. **Accettare formalmente il FAIL su B come limite noto** documentando che il training userà
   comunque i tre start: opzione che **sconsiglio** senza ulteriori evidenze, perché farebbe
   partire PPO da uno stato in cui un terzo degli start non produce nemmeno un ciclo di cammino.

## 14. TODO propagati

- **LOTO** — non integrato, TODO futuro.
- **LOCO** — non integrato, TODO futuro.
- **B1R1** — non integrato, TODO futuro.
- **B1R2** — non integrato, TODO futuro.
- **Dati multistart di training** — i due dataset teacher di luglio (8000 righe agli offset
  −0.20 s e +0.20 s) restano **differiti e assenti** da J7/J8. Da non confondere con la
  validazione closed-loop multistart, eseguita qui come criterio vincolante (celle B e C). Il
  fallimento di B è evidenza diretta a favore di riconsiderare questo differimento.
- **Generalizzazione / Dataset Epic** — TODO aperto: i test coprono AB06 e tre start dello stesso
  trial; non dimostrano generalizzazione a soggetti, trial o velocità diverse.
- **Correzione documentale unità** — vari record V26C congelati dicono "rad" dove gli artefatti
  primari dicono **secondi**; nessun artefatto numerico è stato influenzato. Correzione additiva
  in `v26c_j9_prereg_closed_loop.json → offset_unit_correction`.
- **Difetti documentali da sanare in una futura revisione** — i percorsi `sim_outputs` allo
  staging e l'ambiguità `valid_cycle_count` nel receipt J9R1; il manifest sidecar J8 che descrive
  i pesi J2; il canale legacy a 20 N che in `gait_events_online.csv` etichetta come
  "heel_strike" quattro tocchi di solo avampiede della cella B (canale non vincolante, non ha
  influenzato FSM, reward né verdetto, ma può ingannare un lettore).

## 15. Stato

Fase J9R1 **conclusa con esito FAIL**, evidenza integralmente committata, **STOP per l'audit
dell'architetto**. Nessun retry, nessun critic, nessun PPO, nessuna promozione, nessuna scelta fra
le opzioni della sezione 13. La decisione su come trattare il FAIL della cella B — e se ciò
implichi riaprire i dati multistart di training — spetta all'architetto e all'utente.
