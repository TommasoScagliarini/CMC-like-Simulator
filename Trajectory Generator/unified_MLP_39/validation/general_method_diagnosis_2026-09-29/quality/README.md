# Diagnosi generale della qualità — 29 settembre 2026

Questa analisi offline confronta **errore e forma cinematica**, **forma lenta delle
coppie**, oscillazioni, ampiezza e ripetibilità. V26 è un benchmark descrittivo:
le sue coppie non diventano un target da imporre alla reference LEFT. Nessuna
reward, policy, configurazione di produzione o plugin SEA è stata modificata.

## Esito delle due nuove sequenze fisiche

**La reference LEFT è eseguibile con imitazione precisa e coppie molto più
regolari attraverso il governor e il simulatore attuali.** La sequenza
anticipata costruita offline completa 500/500 segmenti; anche RIGHT completa.
Entrambe le nuove baseline sono state prima riprodotte esattamente: i 17 STO
per lato coincidono byte per byte con gli archivi, come verificato nella
[ricevuta dinamica](../dynamic/baseline_sto_equality.json).

Il confronto grafico essenziale è
[LEFT_focused_comparison.png](../../../../../plot/09_29_2026_general_method_diagnosis/quality/LEFT_focused_comparison.png):
in alto target LEFT, best80 e sequenza; in basso coppie LP10 e V26 descrittivo
sulla sua reference RIGHT. Non si adattano ritardi o fasi agli output. Le
bande della cinematica sono SD fra due cicli, non intervalli di incertezza.

| Target LEFT, finestra 13.15–17.95 s | best80LEFT | Sequenza anticipata LEFT |
|---|---:|---:|
| RMSE actual/target knee / ankle [deg] | 6.376 / 4.040 | **0.390 / 0.339** |
| RMSE served/target knee / ankle [deg] | 6.385 / 4.006 | **0.00470 / 0.00902** |
| Errore forma profilo actual knee / ankle | 0.362 / 0.250 | **0.0208 / 0.0369** |
| Escursione profilo actual / target knee / ankle | 0.750 / 0.928 | **1.001 / 1.011** |
| Residuo HF20 knee [Nm RMS] | 3.783 | **0.121** |
| TV del profilo torque LP10 / range, knee / ankle | 7.685 / 3.906 | **2.734 / 2.334** |
| Coppia ankle min / max [Nm] | −12.62 / +29.17 | **−28.39 / +1.22** |
| Penetrazione massima STO [mm] | 14.15 | **19.03** |

Il miglioramento LEFT è significativo anche nella forma cinematica, senza
compressione dell'escursione. La forma delle coppie cambia e diventa più
regolare. Il knee ha un picco **assoluto** di 68.90 Nm: nella convenzione dei
canonici è un minimo di −68.90 Nm, con massimo +16.16 Nm. Non confondere il
segno nativo OpenSim della colonna STO con il segno grafico del knee.

L'[analisi fisica complementare](../physical/README.md), sulla stessa finestra,
esclude la semplice spiegazione di un piede scaricato: impulso GRF LEFT
354→1166 N·s e reserve traslazionali pelvis 419→222 N RMS; reserve protesiche
zero. Non tutte le reserve diminuiscono: quelle rotazionali biologiche salgono
49.9→51.1 Nm. Aumenta anche la penetrazione, che resta sotto la soglia soft
esistente; ciò non certifica da solo realismo del contatto.

**V26 resta il benchmark buono.** Le coppie della sequenza LEFT non sono
sovrapponibili a V26 e il loro picco/RMS knee è maggiore. La correlazione dei
profili LP10 alle rispettive fasi prescribed, circa zero, descrive questa
differenza: non boccia la qualità LEFT. Reference, carico e eventi fisici sono
diversi, e una stessa percentuale delle due reference non impone identico
contatto. Il miglioramento LEFT è evidente; la parità qualitativa completa
della forma delle coppie con V26 non è ancora dimostrata. I grafici temporali
e per fase consentono una valutazione distinta dalla sola regolarità.

La sequenza RIGHT ottiene anch'essa RMSE actual/target 0.631/0.522 deg e
served/target 0.0230/0.0136 deg, ma raggiunge 25.07 mm di penetrazione e una
coppia ankle fino a +46.93 Nm. Il buon inseguimento cinematico non esaurisce
quindi il problema di contatto/coppia, neppure con il target del benchmark.

### Preferenza della reward, a target invariato

| Contratto comune best80 pen2, 500 step | Baseline del proprio target | Sequenza |
|---|---:|---:|
| LEFT: ritorno | 305.060 | **478.658** |
| LEFT: score actual + served | 385.769 | 499.622 |
| LEFT: costo comandi | 75.577 | 20.189 |
| LEFT: costi penetrazione / momento GRF | 0.000 / 4.694 | 0.000 / 0.000 |
| RIGHT: ritorno | 324.151 | **341.134** |
| RIGHT: score actual + served | 396.911 | 498.697 |
| RIGHT: costi penetrazione / momento GRF | 0.000 / 6.041 | 15.554 / 120.049 |

Il ritorno runtime della sequenza RIGHT, 348.911, usa penetrazione peso 1:
**non viene confuso** con il valore 341.134 a peso comune 2. Entro RIGHT, V26
conserva un ritorno comune superiore, 363.786, perché evita gran parte dei costi
fisici malgrado l'imitazione meno precisa. Non si ordinano LEFT e RIGHT fra loro.

Per LEFT esiste dunque, da questo reset e con questo modello, una soluzione
dinamicamente eseguita con qualità imitativa molto migliore e favorita dalla
reward invariata. Non è necessario postulare l'inseguibilità insufficiente
del governor o una reward che preferisca inevitabilmente il best80. Rimane da
spiegare come farla apprendere causalmente alla policy, preservando la qualità
fisica e la robustezza. La sequenza usa informazione futura e non è una nuova
rete addestrata, né una dimostrazione dell'ottimo globale.

Integrazione eseguita: sette trace complete nel comparatore, zero righe parziali;
ritorni originali verificati entro 5.12e-13, STO/trace entro 6.38e-8 rad e input
invariati. Ricevute in `dynamic_integration_verification.json` e
`verification.json`. Nessuna simulazione è stata avviata dagli script quality.

## Esito sui cinque episodi storici

La **capacità della 39D di eseguire V26 con reference RIGHT è già dimostrata**:
il [controllo dell'ordine aritmetico del 23 settembre](../../../../../reports/user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md)
ha riprodotto gli STO cinematici, di coppia e GRF byte per byte, mantenendo
l'ordine originale delle operazioni V26. V26 resta il benchmark buono accettato.
RIGHT60 è un checkpoint di apprendimento distinto, non quel trasferimento
esatto: la sua qualità inferiore non confuta la capacità dimostrata della 39D.

Il confronto RIGHT60/V26 mostra invece quanto RMSE e correlazione possano nascondere
una differenza di forma importante: RIGHT60 ha una buona correlazione del knee
con il target ma riproduce solo circa il **68% dell'escursione** del profilo medio,
contro il **98% di V26**. Anche la forma delle coppie alla caviglia è diversa.
Il risultato distingue la capacità rappresentativa/esecutiva, già verificata,
dalla soluzione ritrovata da questo training RIGHT60. La domanda rimane come
apprendere una qualità equivalente usando il target LEFT.

| Episodio | Target proprio | RMSE actual knee / ankle [deg] | Escursione knee / target, profilo medio | Errore di forma knee / ankle, profilo medio |
|---|---|---:|---:|---:|
| best80LEFT | LEFT | 6.38 / 4.04 | 0.750 | 0.362 / 0.250 |
| ORACLE_LEFT | LEFT | 6.36 / 4.04 | 1.007 | 0.436 / 0.502 |
| RIGHT60 | RIGHT | 7.14 / 2.45 | 0.681 | 0.218 / 0.296 |
| V26 | RIGHT | 6.91 / 1.43 | 0.985 | 0.290 / 0.208 |
| ORACLE_RIGHT | RIGHT | 7.93 / 3.04 | 1.004 | 0.464 / 0.501 |

L'errore di forma è l'RMSE tra curve centrate e divise per la propria deviazione
standard: zero indica stessa forma a fase prescritta, indipendentemente da offset
e scala. **Va letto insieme all'escursione**: il knee RIGHT60 è l'esempio di un
errore di forma piccolo con ampiezza insufficiente. Non viene cercato uno shift
temporale che massimizzi la correlazione o nasconda il ritardo.

La catena SEA segue bene la reference servita: RMSE actual/served knee–ankle di
best80LEFT 0.281/0.342 deg, contro actual/target 6.38/4.04 deg. Su queste trace il
divario cinematico principale è quindi già presente nella reference servita;
questa è una localizzazione del problema, non una prova causale sul training.

Gli oracles non sono teacher già risolti: restituiscono quasi tutta l'escursione
e coppie molto più lisce, ma conservano ritardo/errore di forma, coppie lente
diverse e penetrazione massima nella finestra di 22.79 mm LEFT e 21.67 mm RIGHT.
L'oracle usa preview 10 ms e non rappresenta una policy appresa equivalente.

## Coppie: separare forma e alta frequenza

| Episodio | Residuo HF20 knee [Nm RMS] | HF20 / RMS dinamica knee | TV profilo torque knee LP10 / range | TV profilo torque ankle LP10 / range |
|---|---:|---:|---:|---:|
| best80LEFT | 3.783 | 0.267 | 7.685 | 3.906 |
| ORACLE_LEFT | 0.147 | 0.006 | 2.575 | 2.317 |
| RIGHT60 | 3.278 | 0.292 | 3.633 | 3.084 |
| V26 | 3.276 | 0.370 | 5.001 | 2.435 |
| ORACLE_RIGHT | 0.143 | 0.005 | 2.600 | 2.566 |

Una penalità generica dell'alta frequenza non rappresenta da sola l'obiettivo
qualitativo: V26 contiene HF knee comparabile a RIGHT60 e non inferiore dopo
normalizzazione. best80LEFT presenta inoltre più variazione della componente
lenta e minore ripetibilità fra cicli. Gli oracles sono molto più lisci ma hanno
ampiezze e profili di coppia diversi da V26. Non sono state imposte regole sul
segno della coppia o fasi specifiche di AB06.

La correlazione del profilo medio ankle LP10 RIGHT60/V26 è circa -0.48. Non proviene da
un disallineamento dei target: tempi, fase prescribed di entrambi i giunti,
target q e qdot sono identici all'interno di RIGHT (verificato numericamente).
La fase ankle RIGHT è +0.013 rispetto al knee in tutte e tre le trace: viene
usata la fase propria del giunto, verificata uguale fra episodi. La correlazione
ankle LP10 sulla stessa finestra temporale è -0.555; sui medesimi due cicli
completi è -0.081, senza adattamento dello shift. La differenza rispetto alla
correlazione dei profili medi evidenzia il peso della variabilità tra cicli:
non va interpretata come inversione di segno costante. Il plot
`torque_RIGHT_V26_same_time.png` mostra esplicitamente questo controllo.

## Che cosa preferisce la stessa reward su queste trace

Si verifica prima la ricostruzione di ogni ritorno originale con la sua
configurazione storica. Si ricostruiscono poi tutti i punteggi con il **medesimo
contratto best80 pen2**. Le loss elementari di imitazione restano riferite al
target attivo dell'episodio e vengono ripesate con i quattro pesi comuni.
Le tolleranze delle loss di contatto vengono controllate. Nessun confronto di
ritorno viene effettuato fra LEFT e RIGHT.

| Stesso target LEFT, stesso contratto pen2 | best80LEFT | ORACLE_LEFT |
|---|---:|---:|
| Ritorno, 500 step | 305.06 | 363.43 |
| Score imitazione actual | 84.18 | 85.48 |
| Score imitazione served | 301.59 | 305.81 |
| Costo variazione comandi | 75.58 | 20.03 |
| Costo penetrazione | 0.00 | 6.69 |
| Costo momento GRF | 4.69 | 0.00 |

La reward osservata preferisce l'oracle LEFT soprattutto per il minore costo dei
comandi. Non certifica una cinematica di forma migliore: il suo errore di forma
a fase fissa è peggiore del best80, nonostante l'escursione sia più fedele. Su
RIGHT il medesimo contratto ordina V26 (363.79), RIGHT60 (324.15), ORACLE_RIGHT
(317.81), coerentemente con l'errore cinematico globale. Sono **ordinamenti di
trace già disponibili**, non ottimi della reward, stime di gradienti PPO o prove
che una modifica dei pesi porti il training a quei comportamenti.

Per l'indagine successiva è utile tenere distinti: inseguimento temporale della
reference, escursione/offset, forma lenta delle coppie, HF e fattibilità del
contatto. Una sola metrica, inclusa la reward totale, non li rappresenta tutti.
La scelta dei criteri qui è generale; la loro verifica disponibile resta su un
solo modello/soggetto e non giustifica conclusioni di generalizzazione.

## Metodo e dati

- Input: percorsi provenienti da `left_reward_audit_2026-09-24/sources.json` e
  `best80_torque_audit_2026-09-24/analyze.py`; manifest locale `sources.json`.
- Cinematica: trace a 100 Hz, q/served/target e velocità; perdita elementare
  verificata contro q/qdot e range registrati; confronto STO/trace solo dove
  lo STO copre il timestamp. Il recorder STO omette l'ultimo endpoint e non
  viene extrapolato per la verifica.
- Finestra cinematica e coppie: 13.15–17.95 s, estremi inclusi con tolleranza
  numerica. Reward: tutti i 500 step registrati. Perciò metriche di forma e
  ritorno non hanno esattamente la stessa finestra e non vengono trattate come
  misure matematicamente equivalenti.
- Profili: solo cicli prescribed completi del **singolo giunto** nella finestra,
  201 campioni per ciclo, interpolazione della fase unwrapped e media dopo il ricampionamento.
  Si usano due cicli per riferimento; SD descrive variabilità fra quei cicli,
  non incertezza statistica. I gruppi FSM fisici sono conservati separatamente
  e non realineano la reference o la coppia. I budget reward per fase usano
  esplicitamente la fase prescribed knee, non la fase FSM.
- Cicli RIGHT knee: 13.915252–15.525235 s e 15.525235–17.112862 s; RIGHT ankle:
  13.894243–15.504397 s e 15.504397–17.092452 s. Sono identici per V26,
  RIGHT60 e ORACLE_RIGHT all'interno del singolo giunto. Cicli LEFT, entrambi i giunti: 13.946871–15.615778 s e
  15.615778–17.163976 s.
- Segni: JSON/CSV e griglie diagnostiche generali conservano i segni nativi
  OpenSim; il knee è flessione-negativo. Il plot focalizzato segue i canonici:
  knee q e coppia moltiplicati per −1, ankle invariato. Le metriche aggiuntive
  `canonical_plot_min_nm`, `canonical_plot_max_nm`, `absolute_peak_nm` rendono
  esplicita la conversione; errori RMS, correlazioni e TV non cambiano.
- Coppie: `SEA_*_tau_spring` a 1 kHz, finestra comune. Filtro Butterworth ordine
  4 a fase zero (offline) con cutoff 10 e 20 Hz; HF definita come raw meno LP.
  Il rapporto HF normalizzato usa la RMS del segnale raw centrato. Non è una
  proiezione spettrale ortogonale e il rapporto di energie non è una frazione
  esatta della potenza PSD. Nessun filtro viene applicato al controllo.
- TV è somma delle differenze assolute. Per il segnale temporale: divisa per
  durata e range robusto P95–P05; per il profilo medio: divisa per il suo range.
  Una TV piccola può corrispondere a una forma sbagliata: resta diagnostica.
- Confronti V26/LEFT: stessa fase normalizzata della **propria reference**,
  meramente descrittivi. Il target LEFT ha tempi e traiettorie diversi.

## Riproduzione e verifica

Dal root, usando un ambiente con numpy/scipy/matplotlib/gymnasium:

```sh
python "Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/quality/analyze.py"
```

Eseguito con `/opt/anaconda3/envs/envCMC-rllib/bin/python`. Non richiede OpenSim o
plugin C++ e il codice usa `pathlib`, quindi non incorpora dipendenze macOS.

Per aggiungere rollouts della stessa diagnosi, senza sostituire i cinque input:

```json
{"nome": {"path": "percorso/relativo/al/repository", "target": "LEFT"}}
```

Passare tale file con `--extra-sources`. L'esecuzione rifiuta confronti nello
stesso gruppo con tempi, fase, q target o qdot target diversi. Aggiorna solo
questi nuovi artefatti di diagnosi e i relativi plot, mai gli input storici.

Verifiche eseguite: separazione sintetica 1 Hz + 60 Hz, invarianza affine della
metrica di forma, ricostruzione di cinque cicli sintetici a fase nota,
ricostruzione reward (errore massimo 5.12e-13), perdita elementare (3.95e-8),
confronto STO/trace (6.04e-8 rad), identità dei riferimenti nello stesso gruppo,
hash SHA256 ricontrollati a fine esecuzione. Risultati in `verification.json`.

File generati: `metrics.json`, `summary.csv`, `phase_profiles.csv`,
`reward_steps_*.csv`, `sources.json`, `input_hashes.json`, `verification.json` e
`execution.log`. Dopo l'integrazione sono presenti sette plot diagnostici nel root
`plot/09_29_2026_general_method_diagnosis/quality/`; non sostituiscono i sette
plot canonici dei rollouts. Nessuna simulazione/training avviata da questo script.

## Integrazione delle sequenze fisiche della stessa indagine

`dynamic_sources.json` identifica le nuove baseline LEFT/RIGHT e le due sequenze
anticipate costruite con lo stesso algoritmo nel ramo `governor/`.
`integrate_dynamic.py` legge un caso soltanto dopo la sua ricevuta finale,
verifica il ritorno originale e controlla l'identità temporale dei riferimenti.
Non avvia simulatori e non esegue polling.

- Solo **500 segmenti completi**, conclusi per `episode_time_limit`, possono
  entrare in `dynamic_full_sources.json` e nel confronto completo di qualità.
- Un fallimento anticipato resta tale. Componenti reward e cinematica vengono
  confrontate soltanto sul prefisso di segmenti fisici completati comune alla
  nuova baseline e alla sequenza. Il terminale parziale resta nella verifica
  del ritorno originale ma non nelle metriche cinematiche o di prefisso.
- Per i casi parziali i filtri usano lo stesso supporto temporale in entrambi
  i dati e scartano 150 ms prima dell'estremo finale. Nessun futuro della
  baseline viene prestato alla candidata. Se manca un ciclo prescribed intero,
  il profilo di ciclo viene dichiarato non disponibile; non viene completato
  artificialmente. Le metriche temporali richiedono una finestra di almeno
  200 ms e restano descrittive di quel tratto.
- Un vantaggio di reward sul prefisso non dimostra vantaggio di ritorno completo,
  robustezza o successo; non include il costo terminale e il futuro mancante.
  Anche una sequenza completa è una soluzione open-loop con informazione sulla
  traiettoria futura, non la prova che PPO o una policy causale la apprenda.

```sh
python "Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/quality/integrate_dynamic.py"
```

Una verifica tramite troncamento sintetico a 100 step della baseline finalizzata
controlla l'assenza di ranking full-return e di cicli inventati:
`dynamic_partial_handling_check.json`. Le ricevute fisiche appartengono al ramo
`dynamic/`; i risultati appaiati vengono salvati in `dynamic_pair_metrics.json`.

## Collegamento con casi storici e fonti primarie

I precedenti aiutano a distinguere correzioni già disponibili da ipotesi ancora
da verificare. Non vengono riapplicati automaticamente pesi o impostazioni
storiche a LEFT.

| Evidenza storica | Lezione per questa indagine |
|---|---|
| [Trasferimento V26 del 23 settembre](../../../../../reports/user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md): mantenendo l'ordine aritmetico originale, cinematica/coppie/GRF identiche byte per byte nella 39D. | La capacità di eseguire il buon V26 è verificata; la sensibilità numerica del primo trasferimento è un caso risolto. Non attribuire automaticamente a quella causa il divario di una policy LEFT con altri pesi e target. |
| [Diagnosi COP del 22 giugno](../../../../../reports/user/2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md) e [guardia introdotta](../../../../../reports/user/2026-06-22_reward_guard_momento_grf_caviglia.md): il SEA compensava un momento GRF sfavorevole. | Buon tracking della reference può coesistere con coppie indesiderate. Controllare contatto e braccio del momento; non correggere per supposizione il segno del SEA. La guardia storica non è un target completo di forma della coppia. |
| [Ablazione GRF soft del 24 giugno](../../../../../reports/user/2026-06-24_analisi_rollout_grf_soft.md): il pacchetto forte correggeva la caviglia trasferendo costo su knee e carico; il soft recuperò un compromesso migliore, conservando chattering. | Un costo più severo non garantisce simultaneamente imitazione, carico e coppie. Serve valutazione multidimensionale; il caso storico non autorizza un tuning specifico AB06 in questa indagine. |
| [Audit ritardo del 22 settembre](../../../../../reports/user/2026-09-22_ritardo_filtro_coppia_ankle_e_proseguimento_unified39.md): circa 80 ms negli oracles diretti; la policy compensava già parte del ritardo, conservando forma/offset/escursione errati. | Un oracle diretto non esaurisce la capacità del governor. Una sequenza anticipata accurata nel replay deve comunque superare la dinamica fisica e non equivale a una policy causale appresa. |
| [Audit reward del 24 settembre](../../../../../reports/user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md): ritorni ricostruiti correttamente e oracle favorito su trace; criticità di penetrazione prima dei fallimenti. | L'ordinamento di poche trace non certifica l'ottimo, la trainabilità o un comportamento desiderato su tutto l'episodio. Valutare i fallimenti senza confrontare durate sbilanciate. |

La ricerca web è stata svolta prima di questi esperimenti; le seguenti sono le
fonti primarie già consultate, senza nuove ricerche durante l'integrazione.

- [MIMOC, Miller et al.](https://arxiv.org/html/2305.10989v1), §§III–V:
  reference dinamicamente coerenti e comprensive di coppie; nell'ablazione,
  omettere il tracking delle coppie produce picchi e contatti più aggressivi.
  Supporta la distinzione cinematica/cinetica. Non valida le coppie V26 come
  target LEFT né dimostra che quella reward sia la soluzione del nostro caso.
- [Pavlichenko e Behnke, ICRA 2022](https://www.ais.uni-bonn.de/papers/ICRA_2022_Pavlichenko.pdf),
  §§IV–V: correzione esterna della reference su giunti flessibili; tracking di
  velocità utile contro bang-bang, filtro con ritardo da apprendere. Un costo
  azione mal tarato induceva correzioni intermittenti ampie. Supporta l'analisi
  di q/qdot e del governor, senza trasferire parametri dal manipolatore.
- [CAPS, Mysore et al.](https://cs-people.bu.edu/rmancuso/files/papers/ICRA21_1616_FI.pdf),
  §§III–IV: regolarizzazione temporale/spaziale della policy e possibili effetti
  avversi dei filtri aggiunti dopo il training. Supporta una futura ablation di
  regolarità, se la dinamica ne conferma il ruolo; non identifica forma lenta
  corretta né rende il chattering l'unico criterio di qualità.
- [OpenSim Moco, Dembia et al.](https://journals.plos.org/ploscompbiol/article?id=10.1371/journal.pcbi.1008493),
  esempio walking/Fig. 8: tracking cinematico, contatti e vincoli dinamici
  devono essere considerati insieme; anche con buon tracking resta errore nel
  picco verticale GRF. Supporta la verifica di fattibilità fisica, non certifica
  compatibilità del nostro plugin SEA con Moco o una soluzione già pronta.

L'inferenza generale che guida le prove è quindi verificare separatamente
**capacità esecutiva, apprendibilità, compatibilità cinematica del governor e
fattibilità dinamica della reference**. Le conclusioni causali restano limitate
agli interventi fisici effettivamente eseguiti e alle rispettive ricevute.
