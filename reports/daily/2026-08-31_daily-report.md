# Daily Report - 2026-08-31

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Progetto della rete unificata imitativa/ex-novo e dell'actor predittivo](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md)
- [Separazione morphology knee/ankle e critic-only rewarm J20](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md)
- [Smoke test e analisi del sigma di esplorazione V26C/J20](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md)

## Sintesi

La giornata ha trasformato l'audit di latenza del 30 agosto in una specifica
architetturale per la nuova pipeline e ha completato il primo intervento
infrastrutturale: separazione dei contributi morphology knee/ankle con
retrocompatibilità e critic-only rewarm verificato sullo stesso actor J20.

La rete proposta è un unico actor deployable con lo stesso contratto di
osservazione e azione durante warm start ed ex-novo. Il warm start attiva la
reward imitativa e spegne quella ex-novo; il passaggio successivo spegne
completamente l'imitazione, crea critic e optimizer nuovi, riscalda il critic a
actor congelato e poi abilita PPO ex-novo. Le safety restano sempre attive.

L'actor non deve osservare target prescribed o segnali controlaterali non
disponibili sulla protesi. Tali informazioni possono essere usate dal teacher,
dalla reward e dal critic privilegiato. Il contratto iniziale resta l'MLP J20
`35 -> 256 -> 256 -> 2`; non è stato proposto un widening.

L'actor J20 non è ancora diventato predittivo. È stata progettata la strategia:
un teacher inverse-governor deve produrre label raw che, attraversando la
stessa catena slew+C2 del deployment, generino la served desiderata circa
`90 ms` dopo per il knee e `100 ms` dopo per l'ankle. Il successivo DAgger deve
raccogliere gli stati realmente visitati, incluso lo stato del governor.

È stato inoltre completato lo sweep frozen-policy del sigma sullo stesso actor
J20. Il valore `0.010` passa sei seed nominali e i due start `+/-0.20 s`; è la
raccomandazione per un pilot PPO breve con log-std congelata. `0.015` passa come
ceiling ma arriva a `27.5579 mm`, mentre `0.020` e `0.030` falliscono il limite
hard di `28 mm` sul seed 127.

## Architettura unificata proposta

Il contratto della reward è:

\[
R = R_{safety} + \alpha_I R_{imitation} + \alpha_E R_{exnovo}
\]

| Modalità | `alpha_I` | `alpha_E` | Safety |
|---|---:|---:|---|
| warm start | `1` | `0` | attiva |
| ex-novo | `0` | `1` | attiva |

I termini spenti possono essere calcolati in shadow logging, ma devono avere
effetto matematico nullo. Al cambio di modalità si preservano actor, ordine
delle osservazioni e semantica dell'azione; si sostituiscono critic e optimizer
perché il value function imitativo non rappresenta il nuovo ritorno ex-novo.

Le due colonne storiche `gait_phase_sin/cos` restano bit-zero per compatibilità
tensoriale; le altre 33 feature causali includono cinematica, SEA, GRF online,
FSM v3, durata ciclo, endpoint precedente e stato `q/v/a` della served.

## Actor predittivo

Un semplice phase shift fisso è stato escluso come soluzione definitiva:

- la fase di un filtro low-pass dipende dalla frequenza;
- lo slew limiter è non lineare e dipende dal precedente endpoint;
- knee e ankle hanno limiti e ritardi differenti;
- la cadenza modifica il profilo ankle;
- il raw actor possiede già una propria quota di sfasamento.

La strategia raccomandata conserva la testa a due azioni assolute. Su ogni
finestra, il teacher inverse-governor ottimizza una sequenza raw attraverso il
governor deterministico minimizzando errore served-vs-desired e regolarità del
comando, nel rispetto di slew, bounds e stato iniziale del filtro. Il primo
comando della sequenza ottima diventa la label dello student.

DAgger deve poi:

1. eseguire lo student in closed loop;
2. raccogliere gli stati realmente visitati;
3. interrogare il teacher da quegli stati;
4. aggiungere nominali e recovery al dataset;
5. aggiornare lo stesso actor 35D;
6. validare ogni round esclusivamente closed-loop.

## Separazione del morphology corridor

Il profilo AB06 resta un singolo artefatto di provenance, ma contiene già due
geometrie indipendenti, ciascuna su 202 punti event-warped:

```text
pros_knee_angle:  mean_rad + std_rad
pros_ankle_angle: mean_rad + std_rad
```

Nel contratto corrente:

| Parametro | Knee | Ankle |
|---|---:|---:|
| moltiplicatore standard deviation | `1.6` | `0.6` |
| margine | `7.5 deg` | `7.5 deg` |
| hard bound diagnostico | `[-1.4, 0.0] rad` | `[-0.6, +0.6] rad` |

La served di ciascun giunto viene confrontata esclusivamente con la propria
banda. Le loss, le escursioni, i bound correnti e gli hard excursion sono
calcolati e loggati separatamente. Restano condivisi il profilo sorgente, la
griglia di fase, la FSM v3 e il delay morphology da `40 ms`.

## Implementazione dei pesi separati

`RewardConfig` espone ora:

```yaml
morphology_knee_weight: null
morphology_ankle_weight: null
```

Un valore omesso eredita `morphology_weight`. La nuova composizione è:

\[
T_{morphology} =
\frac{w_{knee}L_{knee} + w_{ankle}L_{ankle}}{2}
\]

Con pesi uguali il codice mantiene esplicitamente la vecchia aritmetica,
ottenendo equivalenza esatta con la baseline. Un giunto a peso zero non valuta
la propria loss, prevenendo `0 * NaN`; pesi negativi o non finiti falliscono
closed.

Poiché non era stato ancora scelto un rapporto asimmetrico, è stata usata la
baseline neutra:

```text
morphology_weight       = 1.05
morphology_knee_weight  = 1.05
morphology_ankle_weight = 1.05
```

Questa configurazione verifica il nuovo plumbing senza cambiare l'obiettivo
numerico e non costituisce ancora un esperimento ankle-specific.

## Critic-only rewarm

È stato caricato l'actor J20 e creato un critic fresco con:

- actor congelato;
- `logstd` congelata;
- optimizer nuovo;
- una iterazione e `4096` step;
- learning rate `1e-4`;
- `10` epoche;
- pesi morphology espliciti `1.05/1.05`;
- FSM v3 e contratto causale invariati.

Run valido:

```text
Trajectory Generator/runs/training/
MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2
```

| Verifica | Risultato |
|---|---:|
| actor, dieci tensori | byte-identico |
| max actor diff runtime | `0.0` |
| `logstd` | byte-identica |
| sigma knee/ankle | `0.00499999967` |
| mean KL | `0.0` |
| critic digest | cambiato |
| VF loss | `0.00440714` |
| VF explained variance | `0.791205` |
| `phase_timeout_stance` | `0` |
| `morphology_causal_contract_failure` | `0` |
| `resync_count_max` | `0` |
| `hs_cancelled_count_max` | `0` |
| telemetria mancante | `0/4096` |

Il primo tentativo nella directory priva di suffisso `_r2` si è fermato prima
di Ray e prima di qualsiasi update per una restrizione macOS
`psutil/sysctl`. È un artefatto incompleto e non è un checkpoint candidato.

Il checkpoint `_r2` dimostra che il critic può essere adattato senza muovere
l'actor. Poiché `1.05/1.05` è reward-equivalente al peso globale, non dimostra
ancora l'efficacia di una maggiore pressione ankle e non contiene un actor
predittivo.

## Sweep sigma frozen-policy

Sono stati generati moduli isolati constant-sigma mantenendo la mean-network
byte-identica e senza update PPO/critic. La batteria finale comprende `35`
rollout da `500` step o fino a terminazione, common random numbers, sei seed e
tre start.

| sigma | nominale | multi-start | max penetrazione | decisione |
|---:|---:|---:|---:|---|
| `0.010` | `6/6` | `2/2` | `26.8901 mm` | pilot breve raccomandato |
| `0.015` | `6/6` | `2/2` | `27.5579 mm` | ceiling, non primo pilot |
| `0.020` | `5/6` | non avanzato | `28.2061 mm` | hard FAIL |
| `0.030` | `4/5` | non avanzato | `28.1840 mm` | hard FAIL |

Sul seed avverso 127 la penetrazione cresce da `25.6843 mm` a sigma `0.005`,
a `26.8901`, `27.5579` e `28.2061 mm` per sigma `0.010`, `0.015` e `0.020`.
La causa dei failure superiori è l'evoluzione closed-loop accumulata: clipping,
FSM, morphology contract e saturazione SEA restano puliti.

Il raddoppio `0.005 -> 0.010` quadruplica la varianza. Dopo lo slew restano in
media `0.00606/0.00604 rad` RMS su knee/ankle, cioè l'`81.1/86.0%` del rumore
fisico raw. Il pilot deve mantenere `freeze_logstd=true` e durare inizialmente
soltanto `5-10` iterazioni.

## File modificati o introdotti

- `Trajectory Generator/baseline_MLP/reward_function.py`;
- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_joint_split_equal_1p05_2026-08-31.json`;
- `validation/test_morphology_joint_weights.py`;
- `validation/audit_morphology_split_critic_rewarm.py`;
- `Trajectory Generator/baseline_MLP/validation/analyze_v26c_sigma_exploration_smoke.py`;
- `Trajectory Generator/baseline_MLP/validation/v26c_sigma_exploration_smoke_2026-08-31/`;
- `reports/user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md`;
- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/morphology_joint_split_critic_rewarm_audit.json`;
- i tre report utente consolidati sopra.

## Test e verifiche

- `py_compile` sui file Python interessati: PASS;
- 14 test aggregati morphology/split: PASS;
- round-trip dei campi opzionali: PASS;
- equivalenza esatta legacy/split uguale: PASS;
- pesi asimmetrici e guardie non-finite: PASS;
- audit checkpoint: tutti i gate `true`;
- actor e `logstd` immutati, critic aggiornato;
- telemetria e contatori FSM puliti;
- 35 rollout sigma frozen-policy analizzati: PASS completo a `0.010/0.015`,
  hard FAIL riprodotto a `0.020/0.030`;
- sei seed nominali e due start perturbati a `0.010`: `8/8` PASS;
- mean-network exact e save/reload exact su `8/8` moduli sigma;
- riproduzione exact dei baseline J19B deterministico e `sigma=0.005`.

Due suite legacy complete incontrano assertion su valori della configurazione
attiva già diversi dalle aspettative storiche (`morphology_weight` e swing
timeout). I test specifici della modifica passano; il drift preesistente delle
configurazioni resta un debito separato.

## TODO completati

- [x] Formalizzare la rete unica con modalità imitativa ed ex-novo.
- [x] Distinguere feature actor deployable da segnali teacher/reward/critic.
- [x] Progettare il meccanismo generale dell'actor predittivo.
- [x] Separare loss e pesi morphology knee/ankle con retrocompatibilità.
- [x] Eseguire un critic-only rewarm neutro mantenendo J20 byte-identico.
- [x] Auditare critic, actor, sigma e salute FSM del checkpoint risultante.
- [x] Eseguire lo sweep frozen-policy del sigma e misurare l'esplorazione
  effettiva dopo lo slew/governor.
- [x] Identificare `sigma=0.010` come candidato per un pilot breve e
  `sigma=0.015` come ceiling non raccomandato.

## TODO aperti e propagati

- [ ] Implementare il teacher inverse-governor e il generatore delle label raw.
- [ ] Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`
  ankle su più cadenze e stati del governor.
- [ ] Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e
  recovery.
- [ ] Implementare e verificare il cambio integrale di modalità reward,
  mantenendo safety sempre attiva e termini spenti in shadow logging.
- [ ] Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.
- [ ] Ripetere il critic-only rewarm quando la reward diventerà realmente
  asimmetrica.
- [ ] Valutare una fascia soft interna e una corretta assegnazione causale del
  credito, senza spostare il corridor finale.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con
  `sigma=0.010` isotropico e `freeze_logstd=true`, senza modificare J20 o il
  canonico lungo.
- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i
  rollout deterministici e i gate del pilot.
- [ ] Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM
  v3 in tutti i nuovi candidati.
- [ ] Rivalutare sigma dopo l'actor predittivo o una modifica della logica
  causale del morphology corridor.
- [ ] Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,
  dopo il recupero della qualità AB06.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione
  esplicita.
- [ ] Conservare sidecar J8 e artefatti storici senza modifiche retroattive.
- [ ] Riallineare in manutenzione separata le assertion legacy soggette a drift
  e gli altri debiti tecnici/amministrativi già propagati: osservabilità RLlib,
  `cycle_valid`, swing timeout, probe schema, pin V20, `best == last`,
  `fit_p2` V12R3, marker/audit storici e deflessione SEA iniziale.
