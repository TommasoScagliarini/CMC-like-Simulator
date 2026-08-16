# Daily Report - 2026-08-09

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- [H0 V11 weighted full-mean — fit PASS, rollout autonomo FAIL](../user/2026-08-09_h0_v11_weighted_full_mean_fail_closed_loop.md)

## Sintesi

La giornata ha implementato, verificato, congelato ed eseguito il protocollo
H0 V11 `weighted full-mean`, autorizzato per superare il precedente blocco di
imitazione offline e avvicinare H0 alla semantica causale del detector binario
V26.

Il design audit, i fit P0-P3 e i tre round safe DAgger hanno chiuso PASS. Il
primo rollout finale autonomo di P3 ha pero' terminato fail-closed al passo
259/500: la penetrazione della GRF ha raggiunto `25,5699117 mm`, oltre il gate
rigido `<25 mm`. V11 e' pertanto terminale, il candidato P3 non e'
promuovibile e `H0_TRAINING_READY` resta falso.

Il detector V26 e la FSM non hanno causato il failure. Gli eventi erano
ordinati e validi; non sono comparsi NaN, action clipping, timeout, fallback
SEA o soluzioni SO non accettate. L'evidenza indica invece un divario
closed-loop: durante DAgger il safety latch sostituiva il candidato con il
teacher in oltre meta' degli step, mentre P3 non era mai stato testato puro
prima del finale.

Non sono stati aperti trial protetti o reserve, non e' stato eseguito PPO e
non e' stato pubblicato alcun comando di warm-start.

## 1. Protocollo V11

Contratti:

```text
AB06_H0_PRIMARY_SPLIT_V11_V26_WEIGHTED_FULL_MEAN_SAFE_DAGGER
h0_primary_split_v11_weighted_full_mean_v1
```

Il fit congelato usa:

- H0 fresco a ogni stage P0-P3;
- rete delle medie completa `35 -> 256 -> 256 -> 2` con `tanh`;
- `logstd` congelata;
- sorgente actor-only, critic assente e zero update critic/PPO;
- normalizzazione calcolata sui 3.000 campioni base e incorporata nel primo
  layer;
- colonne clock 0 e 1 bit-zero;
- peso 100 per le righe reset e 1 per le altre;
- AdamW full-batch per 3.000 epoche, seguito da LBFGS;
- nessun anchor, polish, fallback o sweep.

Il design audit non ha scritto checkpoint e ha chiuso PASS con:

| Metrica | Osservata | Gate |
|---|---:|---:|
| RMSE | `0,0049338517` | `<=0,006` |
| Max abs | `0,0558278039` | `<=0,060` |
| Max reset | `0,0003790185` | `<=0,003` |
| Fold max diff | `4,1723e-7` | `<=1e-6` |

Receipt principali:

- design audit:
  `96f848d72e2aa72bec4bb108416a84c38730a7b9ccc2d1f36bd172f5a897bacf`;
- preflight:
  `e5cc499eac563516054fe10497bd54ae60790750cc4a8cc92c14f876cd5864a5`;
- execution lock:
  `deba2d6e69e97f3b6814e8a7dcbd79cbef6f1ec7f938c94b6d6604c87e4404f1`.

## 2. Fit P0-P3

| Stage | Righe | RMSE | Max abs | Max reset | Stato |
|---|---:|---:|---:|---:|---|
| P0 | 3.000 | `0,004933852` | `0,055827804` | `0,000379018` | PASS |
| P1 | 4.000 | `0,004968897` | `0,056236781` | `0,000573277` | PASS |
| P2 | 5.000 | `0,004881451` | `0,056750298` | `0,000357516` | PASS |
| P3 | 6.000 | `0,004962145` | `0,057175428` | `0,000352509` | PASS |

I quattro fit hanno prodotto quattro update actor supervisionati, zero update
critic e zero update PPO. Le metriche offline sono entro gate, ma non
migliorano monotonicamente: l'errore massimo cresce fino al `95,3%` del limite
ammesso in P3.

Il candidato congelato pre-finale e':

```text
H0_PRIMARY_SPLIT_V11_P3_b9581a50130ed6fe
```

Il freeze SHA e'
`f2b496d18e26eb06e73672cb32b197ab54fe56e1cf1edbee4bd0a58fa6c72735`.
Il freeze attesta contenuto e provenienza, ma non costituisce promozione.

## 3. Safe DAgger

Ogni round ha raccolto un caso deterministico a offset `-0,20 s` e uno
stocastico nominale con seed 126.

| Round | Caso | Step | Penetrazione max | Interventi latch | Teacher-dependent |
|---:|---|---:|---:|---:|---:|
| 1 | deterministic | 500 | `24,308 mm` | 253 | 500/500 |
| 1 | stochastic | 500 | `22,675 mm` | 276 | 500/500 |
| 2 | deterministic | 500 | `24,296 mm` | 258 | 500/500 |
| 2 | stochastic | 500 | `22,493 mm` | 275 | 500/500 |
| 3 | deterministic | 500 | `23,919 mm` | 262 | 500/500 |
| 3 | stochastic | 500 | `22,462 mm` | 273 | 500/500 |

I sei episodi hanno chiuso PASS con lo shield. Il latch entrava oltre 15 mm,
impostava `effective_alpha=0` e serviva teacher puro fino al rilascio sotto
10 mm in swing. Frequenza e durata degli interventi dimostrano che i PASS di
raccolta non attestavano autonomia.

Il round 3 valutava P2. P3 veniva creato solo dopo quella raccolta: il primo
test fisico puro di P3 e' stato quindi il rollout finale.

## 4. Failure finale

Caso:

```text
final__deterministic_offset_minus_0p20
```

Esito:

- step: `259/500`;
- tempo: `16,3368709838 s`;
- end reason: `grf_penetration`;
- penetrazione step 258: `24,6584025 mm`;
- penetrazione step 259: `25,5699117 mm`;
- superamento: `0,5699117 mm`;
- cicli validi: 1;
- azione terminale finita e non clippata:
  `[0,7175263166, 0,3900616467]`;
- zero teacher, blending, safety latch e rumore nel caso deterministico.

La crescita da `18,786 mm` al passo 248 a `25,570 mm` al passo 259 e' continua
e non e' un glitch numerico. Gli altri gate falliti su step, campioni, query e
cicli sono conseguenze dell'arresto anticipato.

V26 ha prodotto tre eventi ordinati:

```text
HS 13.935870983805 s
TO 15.089870983804 s
HS 15.601870983804 s
```

Duplicati, out-of-order, fallback, sorgenti non V26, invalidi e timeout FSM
sono tutti zero. Il failure e' quindi una instabilita' della policy pura per
covariate shift fra stati teacher-shielded e stati autonomi.

Il ledger terminale SHA e'
`130b64f3e653e0d0afa8602c6ee65e0c20204fb3cace53365cdb9e0240bb4efc`.

Stato terminale:

```text
FAIL_H0_PRIMARY_SPLIT_V11_SAFE_DAGGER_PIPELINE
STOP_V11_TERMINAL_NO_RETRY_SWEEP_OR_RESCUE
```

## 5. File introdotti

Codice e runner:

- `validation/h0_primary_split_v11_weighted_full_mean_contract.py`;
- `validation/h0_primary_split_v11_weighted_fit.py`;
- `validation/run_h0_primary_split_v11_design_audit.py`;
- `validation/run_h0_primary_split_v11_weighted_full_mean.py`.

Test:

- `validation/test_h0_primary_split_v11_weighted_full_mean_contract.py`;
- `validation/test_h0_primary_split_v11_weighted_fit.py`;
- `validation/test_run_h0_primary_split_v11_design_audit.py`;
- `validation/test_run_h0_primary_split_v11_weighted_full_mean.py`.

Receipt e artifact:

- `validation/h0_primary_split_v11_design_audit_receipt.json`;
- `validation/h0_primary_split_v11_weighted_full_mean_preflight_receipt.json`;
- `validation/h0_primary_split_v11_weighted_full_mean_execution_lock.json`;
- `validation/h0_primary_grf_split_adaptation_runs/`
  `2026-08-09_h0_primary_split_v11_v26_weighted_full_mean_safe_dagger/`.

Non sono stati modificati GRF primaria, plugin C++, semantica SEA, geometria
V25, detector/FSM V26 o checkpoint H0 storici.

## 6. Test e verifiche

Prima del freeze:

- suite mirata V11: 43/43 PASS;
- `py_compile`: PASS;
- Ruff: PASS;
- `git diff --check`: PASS;
- strict JSON, finitezza e hash: PASS;
- audit indipendente: GO.

Dopo l'esecuzione:

- `py_compile`, Ruff e `git diff --check`: PASS;
- ledger strict JSON senza NaN/Inf: PASS;
- audit forense indipendente: conferma failure closed-loop e V26 regolare;
- suite mirata: 42 PASS, 1 FAIL documentale.

Il test post-run fallito presume che il receipt one-shot del design audit non
esista. Il receipt esiste legittimamente dopo l'esecuzione. Il test non e'
idempotente nello stato post-run e non ha causato il failure fisico; il codice
execution-locked non e' stato riscritto retroattivamente.

## 7. Scope rimasto chiuso

- altri cinque rollout finali V11;
- V10Q/qualification;
- trial protetti 05/06;
- trial reserve 03/07;
- zero-update save/reload e `checkpoint_zero`;
- comando warm-start;
- morphology reward positivo;
- corridor training;
- PPO;
- claim numerico Windows.

## TODO completati o superseduti

- [x] Separare detector e GRF primaria: V25/V26 sono detector-only e la GRF
  primaria e' rimasta intoccata.
- [x] Correggere routing dual-stream e rendere V26 la sorgente attiva nei test
  H0 autorizzati, senza fallback legacy sinistro.
- [x] Risolvere la coppia spuria TO/HS della FSM precedente: V26 heel-qualified
  ha superato development e replay.
- [x] Congelare una nuova lineage H0 con teacher coerente, sorgenti, receipt e
  write path no-clobber.
- [x] Correggere nella lineage V11 la contabilita' terminale di actor update e
  completed receipt: il ledger registra 4 actor update e tutti gli stage
  completati.
- [x] Eseguire P0 e raccolta DAgger development senza aprire trial protetti.
- [x] Conservare V9/V20, V25/V26, H0 e i failure precedenti come artifact
  storici, senza reinterpretarli.

## TODO aperti e propagati

### Nuova lineage H0

- [ ] Richiedere autorizzazione esplicita per una nuova lineage V12. V11 e'
  terminale e non puo' essere ritentato, salvato o reinterpretato.
- [ ] Inserire un gate fisico puro e unblended dopo ogni candidato
  P0/P1/P2/P3, prima di usarlo nello stage successivo.
- [ ] Rendere frequenza e durata del safety latch un gate esplicito di mancata
  autonomia.
- [ ] Raccogliere e pesare gli stati delle regioni di recovery senza lasciare
  che il teacher nasconda la divergenza dello student.
- [ ] Correggere nella nuova lineage il test one-shot non idempotente,
  mantenendo immutato il receipt V11.
- [ ] Non allentare il gate di penetrazione da 25 mm e non modificare V26, GRF
  primaria o SEA per forzare un PASS.

### Qualification e training readiness

- [ ] Ottenere sei rollout autonomi development PASS con almeno due cicli,
  zero safety stop e non-regressione SEA/reserve.
- [ ] Solo dopo, eseguire V10Q/qualification indipendente.
- [ ] Richiedere autorizzazione separata prima di aprire trial 05 e, soltanto
  dopo PASS, trial 06; mantenere 03/07 chiusi come reserve.
- [ ] Eseguire zero-update save/reload e produrre `checkpoint_zero` prima di
  pubblicare il comando warm-start.
- [ ] Mantenere morphology reward positivo, corridor training e PPO chiusi
  fino a `H0_TRAINING_READY` reale.

### Corridor, robustezza e deployment

- [ ] Completare il preflight causale del corridor con reward a peso zero,
  buffer morphology e confronto offline; non usare il replay come sostituto
  di un rollout live.
- [ ] Definire bordo causale, gestione di segmenti incompleti, timeout,
  bootstrap e `WAIT_HS`, aggiungere logging per evento e testare pesi piccoli
  soltanto su start/seed held-out e almeno un profilo esterno ad AB06.
- [ ] Documentare l'esito terminale del pilot V13/corridor storico e mantenere
  le reserve come gate fisico, non come reward.
- [ ] Validare ogni candidato futuro con multistart, seed held-out, recovery,
  cicli, penetrazione, SEA, reserve e clipping, non soltanto con RMSE o return.
- [ ] Conservare exact-start, compaction, interleaving e gate reserve
  condition-matched; non riutilizzare trace/seed per tuning post-hoc e
  differire memoria ricorrente finché non emerge un limite sequenziale.
- [ ] Eseguire prove su trial, velocita' e soggetti differenti prima di
  dichiarare generalizzazione.
- [ ] Chiarire il TO precoce storico e confrontare H0/logical 24 con protocollo
  identico; eseguire ablation progressive del plant e training A/B cambiando
  un solo fattore per volta.
- [ ] Congelare ordine e scaling delle feature, frequenza, unita', filtri, FSM,
  reset, governor, fallback e watchdog del contratto hardware; validare export
  actor-only, equivalenza host-target, latenza worst-case e HIL.
- [ ] Attestare DLL e parita' Windows x86_64; il claim numerico corrente resta
  macOS arm64.
- [ ] Validare detector, latenza, rumore e delay su segnali hardware/HIL e
  ground truth localizzata; mantenere separata la generazione della GRF e
  portarla a stato production-ready prima del deployment.
- [ ] Conservare come TODO storico una deflessione SEA iniziale coerente con
  la coppia richiesta.
