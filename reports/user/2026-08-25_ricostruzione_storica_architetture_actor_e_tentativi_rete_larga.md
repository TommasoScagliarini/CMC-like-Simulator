# Ricostruzione storica: architetture actor giugno–luglio, adapter, e ogni tentativo di rete più larga

**Data**: 2026-08-25
**Natura**: task **read-only**. Nessun training, nessun rollout, nessuna modifica, nessun file toccato.
**Metodo**: ricostruzione dagli artefatti (forme dei pesi lette dai `module_state.pkl`, config risolte, manifest, receipt) e dai report. **Fatti** e **inferenze** sono separati esplicitamente e marcati.

---

## 0. Il risultato in una riga

**FATTO.** In tutto l'arco giugno → agosto la topologia nascosta dell'actor non è mai cambiata: **2 layer da 256, tanh**, in ogni singolo run di training. È cambiata soltanto la **larghezza dell'ingresso** (31 → 17 → 39 → 35). Le uniche reti più larghe mai esistite (512 e 1024) sono di **agosto**, appartengono alla lineage H0/V12 e **nessuna delle due era un aumento di capacità per l'addestramento imitativo**.

---

## 1. Warm start imitativo del 24/06 — architettura e dimensione

**FATTO.** Run: `Trajectory Generator/runs/training/MLP_imitation_training_06-24-2026/`

Config risolta `training_cfg.resolved.yaml`, righe 4–8:
```
num_hidden_layers: 2
dim_hidden_layers: 256
fcnet_activation: tanh
asymmetric_actor_critic: true
seed: 123
```

Forme reali dei pesi, lette da `rl_module_best/module_state.pkl` (SHA-256 `fa703b2c058483a688a72c8cb0aab13ce541659c00d5611782f6d6ba54749ed8`):

| tensore | forma |
|---|---|
| `pi.0.0.weight` | (256, **31**) |
| `pi.0.2.weight` | (256, 256) |
| `pi.1.weight` | (**4**, 256) |
| chiavi totali | 10 (con alias `pi_encoder.*`) |

Topologia: **31 → 256 → 256 → 4**, dove 4 = 2 mean + 2 log-std.
`rl_module_last` SHA-256 `5e30757aa6bebf5f2fa27608983c73d36784088589e03fd8da27f6730570387a`, stesse forme.

**FATTO — precisazione sulla denominazione.** Il daily del 24/06 (`reports/daily/2026-06-24_daily-report.md`) descrive la giornata come chiusura dell'ablation GRF soft e preparazione del training imitativo successivo; la voce «Scegliere sorgente warm-start» compare come TODO ancora aperto il **26/06** (`reports/daily/2026-06-26_daily-report.md:349`). Il 24/06 è quindi un **run di training imitativo**, non ancora un warm start designato.

**FATTO — la sorgente warm-start effettivamente usata a luglio non è il 24/06.** I report di trapianto di luglio citano come `source_checkpoint`:
`runs/training/MLP_imitation_training_06-23-2026_grfsoft_knee1_ankle2`
(in `runs/training/_warm_start_smoke_20260709_2050/actor_transplant_report.json` e `_warm_start_smoke_20260709_smallbatch/actor_transplant_report.json`). La directory reale è `..._grfsoft_knee1_ankle2_100iter`, `rl_module_best/module_state.pkl` SHA-256 prefisso `d365d2f0e22adaca`, forme **identiche**: W1 (256, 31), W3 (4, 256).

**FATTO.** Tutti i run imitativi di giugno condividono la stessa topologia:

| run | W1 | W3 | sha prefix |
|---|---|---|---|
| 06-21 | (256, 31) | (4, 256) | `771ff7faedc828d5` |
| 06-22 | (256, 31) | (4, 256) | `28ab3ac320ee54d0` |
| 06-23 | (256, 31) | (4, 256) | `65f08369b3c06ea7` |
| 06-23 grfsoft 100iter | (256, 31) | (4, 256) | `d365d2f0e22adaca` |
| 06-23 resume knee2 | (256, 31) | (4, 256) | `f9bf0d7343e248e3` |
| 06-23 resume rebalance | (256, 31) | (4, 256) | `502e8dc3b3b3c431` |
| **06-24** | **(256, 31)** | (4, 256) | `fa703b2c058483a6` |
| 06-25 grfhalf 40iter | (256, 31) | (4, 256) | `8043adc9a154b320` |
| 06-25 grfhalf smoke | (256, 31) | (4, 256) | `66b357b53d1ee998` |

## 2. «Ex novo del 15/07» — architettura e dimensione

**FATTO — il 15/07 non esiste un run ex-novo.** In `runs/training/` non esiste alcuna directory con data 07-15 né 2026-07-15. Il daily `reports/daily/2026-07-15_daily-report.md` documenta un **pilot PPO da 50 update partito da H0** (230.400 transizioni, lr 5e-7, batch 4.608, 12 EnvRunner), non la creazione di un'architettura ex novo. Esito dichiarato: successo tecnico, **nessun checkpoint promuovibile**, otto milestone preregistrate tutte respinte dalla validazione closed-loop.

**FATTO — l'actor in uso il 15/07.** Il daily indica come checkpoint canonico
`validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last`.
Forme lette dal suo `module_state.pkl` (SHA prefisso `44457ca5df7fa0e0e1f1d361d9401369`, identico fra `rl_module_best` e `rl_module_last`):

| tensore | forma |
|---|---|
| `pi.0.0.weight` | (256, **35**) |
| `pi.0.2.weight` | (256, 256) |
| `pi.1.weight` | (4, 256) |

Topologia: **35 → 256 → 256 → 4**. Rispetto al 24/06 cambia **solo** la larghezza d'ingresso (31 → 35); i due layer nascosti da 256 e l'attivazione tanh sono identici.

**FATTO — evoluzione della lineage ex novo** (config risolta e forma W1 lette per ciascun run):

| run | hidden cfg | W1 |
|---|---|---|
| `MLP_ExNovo_training_06-29-2026` | 2 × 256 tanh | (256, **31**) |
| `MLP_ExNovo_training_06-30-2026_timeout_20iter` | 2 × 256 tanh | (256, 31) |
| `MLP_ExNovo_FSM_smoke_06-30-2026_10iter` | 2 × 256 tanh | (256, **17**) |
| `MLP_ExNovo_FSM_fullbatch_06-30-2026_100iter` | 2 × 256 tanh | (256, 17) |
| `MLP_ExNovo_training_07-01-2026_ledger_clawback_20iter` | 2 × 256 tanh | (256, 17) |
| `MLP_ExNovo_training_07-01-2026_ledger_clawback_richobs_100iter` | 2 × 256 tanh | (256, **39**) |
| `MLP_ExNovo_training_07-02-2026_startHS_corridor005_100iter` | 2 × 256 tanh | (256, 39) |
| `MLP_ExNovo_training_07-23-2026` | 2 × 256 tanh | (256, **35**) |
| `MLP_ExNovo_B0820_*` (agosto) | 2 × 256 tanh | (256, 35) |
| `2026-08-16_exploratory_exnovo_v26_corridor_h0ws_50iter` | 2 × 256 tanh | (256, 35) |

**INFERENZA (alta confidenza).** La domanda «architettura ex novo del 15/07» ha come risposta materiale **35 → 256 → 256 → 4**: è l'architettura in uso in quella data, ereditata dal lavoro Markov-35 del 13/07 e mantenuta fino a oggi. Lo dichiaro come inferenza perché la data 15/07 non ha un artefatto ex-novo proprio: la attribuisco per continuità del checkpoint canonico citato nel daily.

## 3. Adapter, teacher-student, compatibilità dei pesi

**FATTO — non è mai esistito un adapter.** In nessun artefatto compare un layer di adattamento, una proiezione appresa o un blocco di conversione fra spazi osservativi. La compatibilità è sempre ottenuta per **trasferimento colonna-per-colonna a larghezza nascosta costante**.

**FATTO — trapianti di luglio.** `actor_transplant_report.json` (run `_warm_start_smoke_20260709_2050` e `_smallbatch`):
```
removed_feature_mode      = drop
critic_init_mode          = fresh_target_untouched
source_only_features_dropped = []
```
Le feature rimosse vengono **eliminate**, il critic viene **reinizializzato fresco** e il target non viene toccato.

**FATTO — trapianto V26B 39D → 35D** (`reports/user/2026-08-24_v26b_v1_transplant_35d_student.md`, righe 9–11):
1. trasferimento **per nome** via manifest pinnati (35D `c6f86028…`, 39D `2837779c…`), biiezione order-preserving verificata: i 35 nomi condivisi sono i 39 meno i 4 target agli indici 2:6; ogni colonna mantenuta è **bit-identica** alla sorgente;
2. rimozione dei 4 target con **compensazione nel bias**: `b1 += W1_39[:, 2:6] @ m`, con
   `m = [-0.417821058, +0.013012919, +0.235458239, +0.006400180]` (media float64 dei target consumati sulle 3 trace anchor pinnate, 1500 righe);
3. hidden 256 invariato; `W1[:, 0:2] == 0`; invarianza clock bit-identica; save/reload bit-exact.

**FATTO — la larghezza nascosta non è mai cambiata in un trapianto.** Tutti i trapianti censiti (giugno→agosto) sono 256 → 256. Non esiste alcun artefatto in cui pesi siano stati trasferiti fra larghezze nascoste diverse.

**FATTO — teacher-student di luglio.** `runs/training/target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/markov_dataset_report.json`: `nominal_steps = 500`, `nominal_repeat = 32`, `nominal_training_samples = 16000`, `recovery_training_samples = 712`, `aggregate_samples = 24712`, `actor_feature_count = 35`. DAgger R1 (`target_domain_dagger_2026-07-11_r1/dagger_dataset_report.json`): `teacher_samples = 500`, `unique_dagger_samples = 68`, `dagger_training_samples = 272`, `aggregate_samples = 772`.

## 4. Ogni tentativo storico di rete più larga

**FATTO — censimento esaustivo.** Scansione di tutti i `hidden_dims` / `fcnet_hiddens` presenti nei JSON del progetto: esistono **esattamente tre larghezze**.

| larghezza | file | lineage | data |
|---|---|---|---|
| **[256, 256]** | 67 | tutte | giugno → agosto |
| **[512, 512]** | 14 | `v12r6` | **14/08/2026** |
| **[1024, 1024]** | 9 | `v12r10` | **15/08/2026** |

**Nessun tentativo di rete più larga esiste in giugno o luglio.** Entrambi sono di agosto e appartengono alla lineage H0/V12, non al ramo imitativo V26B.

### 4.1 W512 — `v12r6`, 14/08/2026 · terminal FAIL

**FATTO — non era un aumento di capacità.** Dal report `reports/user/2026-08-14_h0_v12r6_functional_composite_terminal_fail.md`: il candidato è un **composito funzionale** `35 → 512 → 512 → 2` che rappresenta *esattamente* `0.70 · P2 + 0.30 · R5`, costruito come primo layer = concatenazione delle due torri da 256, secondo layer = matrice **block-diagonal**, mean head = combinazione esatta. Motivazione testuale: *«una semplice interpolazione dei parametri non rappresentava però la stessa funzione e la distillazione in una rete larga 256 introduceva nuovo errore»*. La larghezza 512 serve a **rappresentare esattamente**, non ad apprendere di più.

**FATTO — gate offline tutti PASS**: global RMSE `0.0053635542`, max `0.05221927`, reset max `0.00092254`; `+0.20` RMSE `0.0059064599`; `-0.20` RMSE `0.0058482193`; seed 127 RMSE `0.0051694711`; finestra critica `+0.20` RMSE `0.0046853500` / max `0.02854511`, **migliore del baseline P2** (`0.0056222442 / 0.03706494`).
Equivalenza funzionale su 9.232 righe: errore massimo `1.1324882507324219e-06` contro limite `2e-06`.
Candidate ID `AB06_H0_V12R6_FUNCTIONAL_COMPOSITE_A030_W512:340c2c65c2300a90ce46c09837e679a99e5dea09ce3935574ef5345fafb709f3`.

**FATTO — causa del fallimento: fisica closed-loop, non l'architettura.** Primo rollout `deterministic_offset_plus_0p20` terminato al **passo 179**, `end_reason = grf_penetration`, penetrazione finale **`0.0257906750 m`** contro limite `< 0.025 m`; `terminated = true`, nessun clipping azione, nessun hard invalid, zero teacher query. Ledger terminale SHA-256 `ce67aea83b1f98aa251ad130af1da25982435381c0aeddeafa0a56bd3274e340`.
Interpretazione registrata: *«Il gate offline è necessario ma non sufficiente per la stabilità closed-loop»*; al passo 150 l'azione caviglia è ≈ `-0.1095` contro `+0.1177` nella replay teacher sicura; la penetrazione cresce da `0.01103 m` (passo 170) a `0.02579 m` (passo 179).

### 4.2 W512 in `v12r9` · FAIL

**FATTO** (dalla premessa di `reports/user/2026-08-15_h0_v12r10_protocollo_canonico_e_fit.md`): V12R9 ha prodotto un corpus valido di 11.875 righe con 2.431 label observer, ma **«il fit W512 ha terminato in FAIL e il relativo candidato non può essere promosso o ritentato»**. Non ho trovato un report dedicato con le metriche di quel fit: **lacuna dichiarata**.

### 4.3 W1024 — `v12r10`, 15/08/2026 · terminal FAIL

**FATTO — dry-fit.** `baseline_MLP/validation/v12r10/diagnostics/results/w1024_gate_aligned_dry_fit.json`: `hidden_dims = [1024, 1024]`, status **`PASS_H0_V12R10_STANDARD_W1024_UNCHANGED_NUMERICAL_GATES`** — i gate offline si possono chiudere **senza allentare le soglie**.
Una variante fallisce: `w1024_r6_residual_reset3_dry_fit.json`, status **`FAIL_H0_V12R10_STANDARD_W1024_UNCHANGED_NUMERICAL_GATES`**, con `determinism_replica.reason = "preregistered rule: stop after first failed fit"`.

**FATTO — esecuzione production.** Preflight PASS, singolo fit W1024 completato, candidato congelato:
`AB06_H0_V12R10_RECOVERY_W1024:dcae7d71fa4e246dba16b7078fd74a78a5df4e6330e4230225ffc23c767e1113`, freeze receipt SHA-256 `4c8750f124afa2c2e1305917d13bf98404c5b5bf06cc6cd81c9df59aa2b3ac0b`.

**FATTO — causa del fallimento: di nuovo fisica, più un gate indipendente.** Qualification FAIL al primo development `+0.20`: **212 step**, `end_reason = grf_penetration`. Forense:
- step 211: penetrazione `0.024668982443890087 m`;
- step 212: `0.026729949134248383 m`;
- incremento terminale **`2.061 mm` in un policy step**;
- rampa monotona step 202–212: `12.793 → 26.730 mm` in `0.10 s`;
- **quattro clipping dell'azione 0 agli step 207–210, massimo raw `1.06784`** → gate indipendente fallito.

Stato terminale `FAIL_H0_V12R10_RECOVERY_PIPELINE_TERMINAL`, next stage `STOP_TERMINAL_NO_RETRY`, `development_count = 0`. Ledger SHA-256 `cf50e9450e29abbb8ef9ce759b825a6d5e09905fb6b62d6f2161feed3f6f1cb1`. Detector/FSM V26, routing, finitezza, static optimization e SEA **regolari**. R10 arriva a `18.338 mm` allo step 179, dove R6 era già terminato: estende l'orizzonte di 33 step ma non chiude il rischio fisico.

**FATTO — il verdetto architetturale esplicito.** Dalla sezione «Forense post-terminale e decisione V12R11»:

> *«La proiezione stateless V26-pura è l'identità `P(x)=x`, quindi il source H0 W256 implementa già esattamente `H0(P(x))`. Un nuovo fit W1024 introdurrebbe soltanto errore di approssimazione del nuovo target.»*

È la ragione documentata per cui allargare **non** è stato considerato la leva: per **quel** target la rete W256 lo realizzava già esattamente.

### 4.4 Il reperto semantico collaterale del v12r10

**FATTO** (premessa dello stesso report): nei picchi peggiori il teacher legacy cambia stato per **timeout** mentre il detector/FSM V26 riporta correttamente `STANCE`. L'audit conta **232 righe teacher in `TIMEOUT` contro zero righe V26** nello stesso stato, e **107 cliff di label** senza transizione o pulse V26 adiacente. E, testualmente:

> *«Nel corpus non esistono label discordanti per osservazioni byte-identiche, quindi non è dimostrata un'impossibilità matematica sulle 35 feature; resta però un alias path-dependent dovuto a history nascosta del `LegacyGaitShadow`.»*

Il successore vincolante indicato è *«un relabel stateless e byte-deterministico V26-puro, non l'introduzione di uno shadow legacy online»*.

## 5. Confronto con il V26 B0 attuale

**FATTO — forme lette dagli artefatti:**

| attore | W1 | W2 | W3 | colonne W1 non-zero | SHA-256 |
|---|---|---|---|---|---|
| V26 agosto 39D (parent imitation) | (256, **39**) | (256, 256) | (4, 256) | 39 / 39 | `0ba56eb703a238de41afd10d079c1cd59903ba20189e24d43b5c3a363cde15bd` |
| V1 35D transplant | (256, **35**) | (256, 256) | (4, 256) | **33** (zero: 0, 1) | `16c2d1ae9fb4e77fffa092d74d37e78f54ba24d990774e91bf1d412c551bb031` |
| **B0 35D masked (parent attuale)** | (256, **35**) | (256, 256) | (4, 256) | **23** (zero: 0, 1, 25–34) | `aa7ea0fa1bbef8bb6ef2a33ee8ebe5defeeb4959148a589b81ff994cf291171f` |
| 24/06 imitativo | (256, **31**) | (256, 256) | (4, 256) | 31 / 31 | `fa703b2c058483a688a72c8cb0aab13ce541659c00d5611782f6d6ba54749ed8` |
| 15/07 (markov35 canonico) | (256, **35**) | (256, 256) | (4, 256) | — | prefisso `44457ca5df7fa0e0e1f1d361d9401369` |

**FATTO — sintesi del confronto.** Il B0 attuale ha **esattamente la stessa topologia nascosta del 24/06 e del 15/07**: 2 layer da 256, tanh, testa a 4 uscite, 10 chiavi `pi*`. Le differenze sono tutte in ingresso:

- 31 (24/06) → 35 (15/07 e oggi): +4 colonne;
- delle 35 colonne di B0, **12 sono azzerate a mano** (le 2 di clock 0–1 e le 10 controller/Markov 25–34), quindi il supporto effettivo è **23D**, contro 31 utili nel 24/06 e 35 nel 15/07.

**INFERENZA (media confidenza).** Rispetto al 15/07, B0 lavora oggi con **12 colonne in meno** a parità di capacità nascosta. È una riduzione di informazione in ingresso di circa il 34% del vettore, deliberata (rimozione del proxy autoregressivo e del clock) e non compensata da alcun aumento di capacità. Lo qualifico come inferenza perché il confronto «colonne utili» non tiene conto del contenuto informativo effettivo di ciascuna colonna, che non ho misurato in questo task.

## 6. Fatti contro inferenze — riepilogo

**Fatti verificati su artefatti** (path e hash sopra):
1. topologia 2 × 256 tanh in **ogni** run giugno→agosto;
2. 24/06 = 31 → 256 → 256 → 4; il warm-start di luglio usa però il 23/06 grfsoft, non il 24/06;
3. nessun run ex-novo datato 15/07; actor canonico in quella data 35 → 256 → 256 → 4;
4. nessun adapter mai esistito; compatibilità per trasferimento nominale con compensazione nel bias, hidden costante a 256;
5. esattamente tre larghezze mai esistite: 256, 512 (v12r6, 14/08), 1024 (v12r10, 15/08);
6. entrambe le reti larghe hanno **superato i gate offline** e sono fallite in **closed-loop per penetrazione GRF** (0.02579 m allo step 179; 0.026730 m allo step 212 con 4 clipping);
7. verdetto architetturale registrato: per quel target W1024 aggiunge solo errore di approssimazione;
8. B0 attuale: 35D con 12 colonne azzerate → supporto effettivo 23D, hidden invariato.

**Inferenze, marcate come tali:**
- l'attribuzione dell'architettura «ex novo del 15/07» avviene per continuità del checkpoint canonico, non per un artefatto datato 15/07;
- il calo di colonne utili 35 → 23 di B0 rispetto al 15/07 è una riduzione informativa non compensata; l'entità reale non è misurata qui;
- il reperto «alias path-dependent» del v12r10 ha la **stessa forma** del conflitto testa↔ciclo1 misurato in B1R2 (conflitto di label a osservazioni vicine, **nessuna collisione esatta**, sospetta dipendenza da storia nascosta). **Non affermo che sia la stessa causa**: le lineage differiscono (teacher legacy shadow in V12/H0 contro label da cache IK in V26B) e la verifica non è stata fatta.

**Lacune dichiarate:**
- non ho trovato un report dedicato con le metriche del fit W512 fallito in `v12r9`;
- non esiste un artefatto ex-novo datato 15/07;
- nessun tentativo storico di rete più larga **addestrata da zero sul compito imitativo**: entrambi i casi 512 e 1024 nascono come composizione o recupero nella lineage H0/V12, quindi il verdetto «allargare aggiunge solo errore» **non è trasferibile** al fit base V26B senza una verifica dedicata.

## 7. Cosa non ho fatto

Nessun training, rollout, DAgger, fit o modifica di alcun file. Nessuna proposta di nuovo arm, come richiesto. Nessun candidato toccato. Consegno solo la ricostruzione.
