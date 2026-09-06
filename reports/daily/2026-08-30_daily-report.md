# Daily Report - 2026-08-30

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- [Audit del corridor morfologico rispetto alle cinematiche prescribed e V26](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md)

## Sintesi

La giornata è stata dedicata a stabilire se le cinematiche prodotte dai
training ex-novo V26 fossero una conseguenza inevitabile del modello corrente
e se il morphology corridor AB06 fosse stato costruito in modo errato.

Il verdetto è che il profilo non presenta errori di soggetto, laterality,
segno, unità o mapping degli eventi. Il modello e i SEA possono seguire
cinematiche prescribed prevalentemente interne al corridor. Le curve V26
mantengono una forma generale prescribed-like, ma la realizzano troppo tardi
nel ciclo; l'errore dominante è nella catena che trasforma il target raw
dell'actor nella served reference, non nel tracking fisico served→actual.

L'aumento del peso morphology da `0.0025` a `1.05`, pari a un fattore 420, non
ha cambiato materialmente le curve. La sola moltiplicazione della penalità non
corregge sfasamento, latenza, reward delayed o scarsa esplorazione dell'actor.
Il corridor resta valido come envelope diagnostico AB06, ma la sua formulazione
come reward online deve essere resa sensibile a cadenza e attribuzione causale.

## Evidenze principali

### Fattibilità del tracking

| Esperimento | Knee RMSE | Ankle RMSE |
|---|---:|---:|
| CMC-like actual vs prescribed | `0,00310 rad` | `0,02020 rad` |
| prescribed controller recente | `0,00439 rad` | `0,00546 rad` |
| V26 peso `1.05`, actual vs served | `0,00995 rad` | `0,00881 rad` |

La simulazione prescribed completa è risultata fuori corridor nello `0%` dei
campioni knee e nel `1,99%` dei campioni ankle. Questo esclude un limite
meccanico generale come spiegazione della mancata plantarflessione V26.

### Sfasamento rispetto al prescribed

Anticipando offline le curve V26 si ottiene una correlazione di circa `0,91`
con il prescribed per entrambi i giunti:

| Giunto | Anticipo ottimo | Equivalente temporale |
|---|---:|---:|
| Knee | `0,106` cicli | circa `160 ms` |
| Ankle | `0,089` cicli | circa `135 ms` |

Vicino al TO l'ankle prescribed è circa `-0,124 rad`, mentre la served V26 è
circa `+0,128 rad`: oltre a essere tardiva, ha segno funzionale opposto nella
finestra biomeccanica di interesse.

### Effetto del peso morphology

Tra peso `0.0025` e `1.05`:

| Giunto | RMSE fra le served | Correlazione |
|---|---:|---:|
| Knee | `0,000477 rad` | `0,9999989` |
| Ankle | `0,000654 rad` | `0,9999891` |

Nel run `1.05`, il knee è fuori corridor in `10/372` campioni e l'ankle in
`62/372`. Il test ha quindi falsificato l'ipotesi che il problema dipendesse
solamente da un peso globale insufficiente.

### Dipendenza dalla cadenza

Il profilo unico aggrega 123 cicli con periodi da `0,879` a `1,669 s`. I 75
cicli veloci hanno ankle vicino al TO pari a circa `-0,318 rad`; i 22 cicli
lenti, più simili al rollout corrente, circa `-0,082 rad`. Il profilo globale è
quindi influenzato maggiormente dai cicli veloci.

La cadenza spiega parte del mismatch con la media globale, ma non assolve il
V26: la sua served a `+0,128 rad` resta distante anche dalla media lenta e con
segno funzionale errato.

## Addendum frozen-policy sulla latenza

Senza modificare policy o avviare training, sono stati confrontati quattro
stadi della catena su tutti e quattro i rollout V26:

```text
actor raw -> slew limiter -> filtro C2 -> served -> cinematica fisica
```

| Tratto | Knee | Ankle |
|---|---:|---:|
| actor raw → dopo slew | `10 ms` | `20 ms` |
| dopo slew → served | `80 ms` | `80 ms` |
| served → actual | `0 ms` | circa `0 ms` |
| actor raw → served | `90 ms` | `100 ms` |

I limiti hard di velocità, accelerazione e jerk del governor non sono
intervenuti. Gli `80 ms` principali derivano quindi dalla dinamica low-pass del
filtro C2 a `4 Hz`.

Al TO del run `1.05`:

| Segnale ankle | Valore |
|---|---:|
| prescribed | `-0,1242 rad` |
| actor raw | `-0,1276 rad` |
| dopo slew | `-0,0222 rad` |
| served | `+0,1284 rad` |
| actual | `+0,1273 rad` |

Il raw actor è quindi già vicino al prescribed in quel campione; slew e filtro
consegnano il movimento in ritardo. A questa latenza fisica si aggiungono
`40 ms` di ritardo nella consegna del campione morphology alla reward PPO. Il
delay reward non rallenta il comando, ma assegna la penalità a una transizione
successiva rispetto a quella che ha originato l'errore.

## Strategia risultante

- Conservare il corridor corrente come baseline diagnostica event-aligned.
- Non spostare il corridor nel tempo per accettare una traiettoria tardiva.
- Separare la metrica finale retrospettiva dal segnale causale apprendibile.
- Progettare un corridor condizionato su periodo, stance e swing.
- Correggere il credito della reward o valutare la served futura predetta dal
  governor.
- Separare i contributi knee/ankle prima di nuovi esperimenti sui pesi.
- Introdurre una fascia interna soft per fornire gradiente prima del bordo.
- Usare pilot preregistrati brevi da `5–10` iterazioni prima di altri run da
  50, sempre dallo stesso parent J20 per confronti controllati.

## File introdotti

- `reports/user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md`;
- `validation/audit_v26_frozen_policy_latency.py`;
- `validation/v26_reference_latency_audit/2026-08-30_frozen_policy_latency_decomposition.json`;
- `plot/08_30_2026_v26_reference_latency_audit/01_frozen_policy_latency_decomposition.png`.

Non sono stati modificati training, checkpoint, reward, configurazioni,
profilo morphology, plugin C++ o semantica SEA.

## Test e verifiche

- provenance del profilo, laterality, segni e unità verificati;
- confronto dei quattro rollout V26 con prescribed e corridor;
- event warping retrospettivo su cicli completi;
- sweep di phase shift con passo `0,001` cicli;
- stratificazione dei 123 cicli AB06 per periodo;
- bound del plot 8 ricostruiti entro `2,22e-16 rad`;
- scomposizione temporale stabile su `4/4` rollout;
- script diagnostico compilato con `py_compile`;
- JSON verificato con assertion strutturali;
- PNG `2160x1620` generato e ispezionato;
- nessun training eseguito e policy non mutata.

## TODO completati

- [x] Escludere errori di segno, unità, lato o provenance del corridor.
- [x] Dimostrare che il modello può seguire il prescribed.
- [x] Quantificare separatamente actor→slew, slew→served e served→actual.
- [x] Verificare che il solo aumento del peso globale non corregge il toe-off.
- [x] Identificare la dipendenza del profilo morphology dalla cadenza.

## TODO aperti e propagati

- [ ] Progettare un segnale predittivo o una riassegnazione causale della loss
  che compensi la catena actor→served senza spostare il gate finale.
- [ ] Separare i pesi morphology knee e ankle e conservare logging per giunto.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Valutare una fascia soft interna al corridor.
- [ ] Preregistrare ogni nuovo pilot con stop rule, rollback e confronto dallo
  stesso checkpoint J20.
- [ ] Rivalutare sigma: `0,005` non deve essere ereditata automaticamente.
- [ ] Monitorare sempre penetrazione con soglie `20 mm` soft, `25 mm`
  diagnostica e `28 mm` hard, insieme ai contatori FSM v3.
- [ ] Mantenere separate served reference e cinematica fisica nelle metriche.
- [ ] Generalizzazione multimodello con dataset EPIC e soggetto held-out.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri e richiedono approvazione
  esplicita prima dell'integrazione.
- [ ] Conservare il sidecar J8 stantio e gli artefatti storici senza modifiche
  retroattive.
- [ ] Debiti tecnici/amministrativi ancora aperti: osservazioni non gateate R3,
  marker `RESTORE_AUDIT_PENDING`, processi storici orfani, semantica
  `cycle_valid`, osservabilità RLlib, swing timeout, probe schema, pin V20,
  `best == last`, `fit_p2` V12R3 e deflessione SEA iniziale.
