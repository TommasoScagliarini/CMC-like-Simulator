# V26 nella unified39: ordine dei calcoli e robustezza al rumore

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 23 settembre 2026. Autorizzazione: «svolgi i primi 2 e poi valutiamo».
Stato: entrambi gli esperimenti conclusi e verificati. La batteria stocastica
è terminata alle 11:43 CEST. Nessun nuovo training avviato.

**Risultato principale:** conservando l'ordine dei calcoli V26 si riproducono
esattamente cinematica, coppie e GRF storiche. L'export V26 trasferito completa
inoltre 5/5 episodi con rumore originale e 5/5 con rumore dimezzato, contro
3/5 e 4/5 della best72. È un'indicazione favorevole alla soluzione storica
nel contesto attuale, non ancora una dimostrazione statistica generale.

## Problema e strategia

Il primo trasferimento dei pesi V26 nella unified39 aveva recuperato buona
parte della qualità storica, ma conservava differenze nelle traiettorie,
nelle coppie e negli eventi FSM. Restava inoltre da confrontare la robustezza
di questa soluzione con la best72 sinistra scelta come baseline operativa.

Sono stati eseguiti due esperimenti di sola valutazione:

1. Ripetere il rollout deterministico mantenendo l'ordine originale delle
   operazioni della rete V26, invertendo la permutazione degli ingressi prima
   dell'inferenza anziché permutare le colonne del primo layer.
2. Valutare l'export V26 trasferito con cinque sequenze di rumore e due
   ampiezze, usando lo stesso protocollo già applicato alla best72.

Nessun apprendimento o aggiornamento dei checkpoint. FSM v3 mantenuta, senza
modifiche al simulatore, ai plugin o ai controllori SEA. Il percorso RIGHT è
una sottoclasse diagnostica isolata: il normale percorso LEFT resta distinto.

## Controllo deterministico: riproduzione del risultato storico

**I file STO di cinematica, coppie SEA e GRF sono identici byte per byte a
quelli del rollout V26 di agosto.** Ciascuno contiene 5.000 campioni. Le
500 azioni e le 500 osservazioni dell'actor coincidono esattamente, dopo la
rimappatura degli ingressi. Il controllo termina tutti i 500 passi.

| Verifica | Esito |
|---|---:|
| Azioni uguali al rollout storico | 500/500 |
| Osservazioni actor uguali, a parità di ordine degli ingressi | 500/500 |
| Cinematica STO, 22 colonne e 5.000 righe | Stesso SHA256 |
| Coppie SEA STO, 5 colonne e 5.000 righe | Stesso SHA256 |
| GRF STO, 27 colonne e 5.000 righe | Stesso SHA256 |
| Passi dell'episodio | 500/500 |
| Cicli validati | 2 |
| Ritorno | 363,785609 |
| RMSE giunto ginocchio / caviglia rispetto al target RIGHT | 6,906° / 1,459° |
| Picco positivo coppia caviglia | 10,188 Nm |
| RMS componente coppia ginocchio sopra 20 Hz, dopo 200 ms | 3,280 Nm |
| Massima penetrazione ai campioni della reward / nello STO | 14,797 / 16,238 mm |

La differenza di ritorno rispetto allo storico è circa 4×10⁻¹³ e dipende
dall'arrotondamento nella somma. I piccoli scarti dello stato cinematico nei
JSON sono esattamente spiegati dalla rappresentazione float32 attuale rispetto
al float64 storico; non compaiono nei file STO. Non tutti i file del rollout
sono identici: metadata, diagnostica e formato di alcuni campi JSON differiscono.

La penetrazione resta quella storica, non è zero. Il suo massimo nello STO
include il transitorio iniziale e usa campioni a 1 ms, mentre la reward è
registrata ogni 10 ms.

### Perché il primo trasferimento dava una traiettoria diversa

Riordinare le colonne dei pesi conserva la funzione matematica, ma cambia
l'ordine delle somme in virgola mobile. Nel primo trasferimento lo scarto
nella prima azione era circa 5,96×10⁻⁸; il circuito rete–filtro–simulatore
amplificava progressivamente questa perturbazione. La cinematica superava
0,1° di differenza alla caviglia al passo 37 e al ginocchio al 41, prima della
prima differenza negli ingressi FSM, osservata al passo 95.

Il nuovo controllo conserva invece le operazioni della rete originale e
recupera il risultato storico, pur mantenendo il medesimo ambiente attuale e
la FSM v3. Questo riconduce le differenze residue del **primo trasferimento**
alla sensibilità all'ordine dei calcoli. Non dimostra che il divario della
**best72 addestrata sul sinistro** abbia la stessa causa: quella policy ha
pesi, target e storia di apprendimento differenti.

Il risultato esclude che, per questo episodio e questo target RIGHT, il
contenitore attuale o la FSM v3 impediscano necessariamente l'esecuzione della
soluzione storica. Non dimostra equivalenza fra tutte le transizioni delle
versioni FSM, né stabilità generale su altri stati iniziali.
Non esercita neppure i percorsi di raccolta dei fallimenti, critic, PPO e
ripresa dell'optimizer che possono incidere diversamente sull'apprendimento.

## Protocollo stocastico

Sorgente: best87 del training V26 del 20 agosto. Cinque seed del rumore
123–127, ciascuno valutato con scala 1 e 0,5. Start fisso a 12,99 s, seed
ambiente 123, 500 passi da 10 ms, sei canali prescribed/clock sempre visibili.
Target RIGHT storico con shift ginocchio 0,465 e caviglia 0,452; clock actor
della fase destra non sfasata; velocità prescribed senza divisione per 10.

**Per le dieci prove si usa l'export con colonne dei pesi permutate**, come
nel primo trasferimento. Questa scelta è registrata prima di conoscere
l'esito del controllo deterministico. Il controllo con ordine originale e
la batteria stocastica sono quindi due interventi distinti; non sono state
eseguite dieci prove rumorose con l'adattatore dell'ordine originale.

Il campionatore è lo stesso `completion_noise_rollout.paired_selector` usato
per la best72: generatore Torch CPU indipendente, un nuovo campione gaussiano
per ogni passo, azione = media + deviazione standard appresa × campione ×
scala. Il clipping a [-1,1] avviene nell'ambiente.

Per lo stesso seed si verifica l'identità dei campioni gaussiani fra policy
e scale sui prefissi comuni. La deviazione standard appresa dipende dalla
policy e dallo stato: l'ampiezza assoluta del rumore non è imposta uguale.
Ridurre il rumore cambia anche la media dell'azione dopo il clipping.

Un episodio è completo soltanto se termina per `episode_time_limit` a 500
passi. Gli errori tecnici sono separati dai fallimenti fisici. Un terminale
hard può lasciare un ultimo stato parziale con timestamp ripetuto: quella
riga resta nel conteggio delle transizioni e nella ricostruzione del ritorno,
ma viene esclusa dalle metriche cinematiche e SEA. Il suo valore diagnostico
di penetrazione, talvolta zero, non indica assenza di penetrazione.

Al massimo quattro simulatori contemporanei, ciascuno con un thread numerico,
come nella valutazione best72. Watchdog per ogni caso. Nei nuovi rollout si
registrano anche gli STO; negli stocastici best72 questi file non erano stati
salvati. Per confrontare le coppie si usano quindi i contatori SEA comuni,
senza ricostruire picchi con segno o chattering da dati che non li contengono.

## Risultati stocastici

| Condizione | V26 trasferita RIGHT | Best72 LEFT | Passi medi V26 / best72 |
|---|---:|---:|---:|
| Rumore originale, scala 1 | 5/5 completi | 3/5 completi | 500 / 453,8 |
| Rumore dimezzato, scala 0,5 | 5/5 completi | 4/5 completi | 500 / 493,8 |

Tutti i nuovi casi finiscono per limite temporale. Nessun errore tecnico,
timeout del watchdog o terminale parziale. I casi best72 provengono dalla
valutazione già archiviata, non sono stati rilanciati.

| Seed | Passi V26, scala 1 | Passi best72, scala 1 | Passi V26, scala 0,5 | Passi best72, scala 0,5 |
|---|---:|---:|---:|---:|
| 123 | 500 | 500 | 500 | 500 |
| 124 | 500 | 500 | 500 | 500 |
| 125 | 500 | 303, penetrazione | 500 | 500 |
| 126 | 500 | 500 | 500 | 500 |
| 127 | 500 | 466, limite hard | 500 | 469, penetrazione |

Il caso best72 127 a scala 1 ha 466 transizioni registrate, di cui 465 passi
fisici completi: l'ultimo è parziale. Il massimo salvato di 27,398 mm non
include il sottopasso che supera il limite hard di 30 mm. Le altre due
interruzioni best72 superano la soglia ordinaria di 28 mm.

Nella V26 il massimo ai campioni della reward, considerando tutti i cinque
seed, è **25,084 mm** a scala 1 e **17,857 mm** a scala 0,5. La riduzione del
rumore abbassa quindi anche la penetrazione osservata; non va interpretata
come assenza di contatto penetrante nel modello.

### Rumore effettivo: una differenza da indagare

Per evitare confronti su durate diverse, le metriche seguenti usano i primi
**303 passi, 3,03 s**, comuni a tutti i venti episodi V26 e best72. Le
statistiche delle azioni aggregano cinque sequenze, ossia 1.515 passi per
condizione. Le unità sono quelle dell'azione normalizzata, non radianti.

| Metrica, prefisso comune | V26 scala 1 | Best72 scala 1 | V26 scala 0,5 | Best72 scala 0,5 |
|---|---:|---:|---:|---:|
| Std effettiva media ginocchio | 0,593 | 0,692 | 0,300 | 0,332 |
| Std effettiva media caviglia | 0,515 | 0,766 | 0,258 | 0,373 |
| RMS perturbazione prima del clipping, ginocchio | 0,650 | 0,772 | 0,342 | 0,361 |
| RMS perturbazione prima del clipping, caviglia | 0,685 | 1,069 | 0,326 | 0,521 |
| Componenti d'azione clippate | 24,09% | 33,47% | 13,83% | 21,45% |

A scala 1 la std media della best72 è circa **17% maggiore al ginocchio e
49% maggiore alla caviglia**. Anche la frequenza del clipping è maggiore.
Questo introduce un candidato concreto per spiegare parte del divario: la
distribuzione esplorativa appresa dalla best72 perturba maggiormente le azioni.

È però un'osservazione sulle rispettive traiettorie, non un esperimento che
isola la varianza. Le medie delle azioni, gli stati visitati e i target sono
diversi. Inoltre la best72 con rumore dimezzato conserva un fallimento pur
avendo std effettiva media inferiore alla V26 con rumore originale. Il solo
valore medio della std non riassume tutta la robustezza.

### Cinematica e coppie sullo stesso intervallo

| Metrica, primi 303 passi | V26 scala 1 | Best72 scala 1 | V26 scala 0,5 | Best72 scala 0,5 |
|---|---:|---:|---:|---:|
| RMSE ginocchio rispetto al proprio target | 6,41° | 6,68° | 5,88° | 5,74° |
| RMSE caviglia rispetto al proprio target | 2,88° | 4,03° | 1,85° | 2,71° |
| RMS coppia SEA ginocchio | 17,56 Nm | 20,93 Nm | 13,15 Nm | 15,42 Nm |
| RMS coppia SEA caviglia | 7,99 Nm | 10,25 Nm | 4,98 Nm | 6,04 Nm |
| RMS variazione temporale coppia ginocchio | 1.228 Nm/s | 1.260 Nm/s | 1.148 Nm/s | 1.225 Nm/s |
| RMS variazione temporale coppia caviglia | 153 Nm/s | 204 Nm/s | 138 Nm/s | 169 Nm/s |

Sono medie dei cinque valori per episodio. La caviglia V26 ha minore errore
rispetto al proprio target nelle due condizioni; al ginocchio non c'è un
vantaggio uniforme. Target RIGHT e LEFT rimangono differenti: la tabella è
descrittiva e non misura l'effetto causale del solo cambio di prescribed.

Le metriche SEA quantificano intensità e variazione delle coppie. La derivata
RMS non è la stessa misura del chattering sopra 20 Hz e non identifica il
segno dei picchi della caviglia. Per questi aspetti il confronto esatto del
controllo deterministico è la verifica più forte disponibile qui.

Il file dei risultati conserva anche confronti per ciascuna coppia di seed e
scala, usando la durata comune specifica della coppia: 500 passi quando
entrambi completano, 303/465/469 nei casi con terminazione anticipata. Il
prefisso globale evita durate sbilanciate, ma esclude i tratti finali; per
questo non è l'unica vista conservata.

## Interpretazione e decisione successiva

1. **La soluzione V26 è eseguibile nella 39D attuale.** La riproduzione esatta
   del controllo e i dieci episodi rumorosi completi rendono poco plausibile
   un'incapacità inevitabile dell'architettura o del contenitore di eseguire
   una buona soluzione. Questo non prova che il learner attuale la ritrovi.
2. **La sensibilità numerica spiega il residuo del primo trasferimento.** Non
   va estesa automaticamente alla best72: invertire la permutazione degli
   ingressi non è una correzione dimostrata per la policy sinistra appresa.
3. **Il divario best72 rimane da separare fra compito e apprendimento.** Fra
   le differenze osservabili c'è ora anche una maggiore esplorazione,
   soprattutto alla caviglia, oltre a target e storia del training.
4. **Non considererei l'oscuramento una soluzione già giustificata.** Toglie
   informazione alla policy e questi esperimenti non mostrano che corregga
   la robustezza. Il confronto RIGHT/LEFT con lo stesso learner e la stessa
   esperienza resta informativo; un confronto con rumore assoluto comune
   potrebbe prima isolare meglio il contributo dell'esplorazione.

Cinque seed allo stesso start sono uno screening. Gli intervalli Wilson 95%
del completamento sono circa 56,6–100% per 5/5, 23,1–88,2% per 3/5 e
37,6–96,4% per 4/5: molto ampi. Le due scale condividono i seed e non sono
dieci repliche indipendenti dello stesso esperimento. Non abbiamo dimostrato
equivalenza statistica fra training, né una differenza generale fra le
probabilità di completamento. Le percentuali qui misurate non sono la
percentuale di episodi completati durante il training PPO.

## Verifiche conclusive

- Controlli di rumore, azioni e reward passati per tutti i venti episodi,
  inclusi i dieci archiviati best72. Ricostruzione azioni esatta; massimo
  errore nella somma delle reward 4,55×10⁻¹³.
- Replay del generatore Torch e identità dei campioni gaussiani appaiati
  verificati. Provenienza e hash delle sequenze controllati.
- Trenta file di input registrati conservano lo stesso SHA256, inclusi
  checkpoint, sorgenti, asset e runner congelati.
- Dieci watchdog conclusi con codice 0; suite completata senza processi
  attivi nel suo registro. Nessun nuovo training o aggiornamento dei pesi.
- `analyze.py --all` concluso con codice 0. Grafici canonici e comparativi
  salvati in `plot/`, con ricevute; confronto visivo verificato.

## Artefatti

- [Sette grafici canonici del controllo deterministico](../../plot/09_23_2026_2/).
- [Confronto grafico dell'ordine dei calcoli](../../plot/09_23_2026_v26_robustness_comparison/deterministic_arithmetic_comparison.png).
- [Confronto della penetrazione nei venti episodi](../../plot/09_23_2026_v26_robustness_comparison/paired_noise_penetration.png).
- [Metriche complete, compresi i confronti per coppia — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-752c905a6e23f095>).
- [Verifica finale degli input e del completamento — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-7eef8d8dd18b5124>).
- [Verifica byte-per-byte degli STO e precisione dei JSON — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-1c907e94ec1cde22>).
- [Protocollo ed esecuzione riproducibile — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-c0fe5f8648e7cd34>).

## File introdotti e modificati

Dentro `Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/`:

- `case_run.py`: adattatore dell'ordine originale, valutazione rumorosa,
  cattura del terminale e protezione degli input storici.
- `run_suite.py`: dieci casi, massimo quattro processi, stato e watchdog.
- `analyze.py`: controlli offline, confronto deterministico e stocastico,
  confronto su durate comuni e grafici nel root `plot/`.
- `README.md`, `protocol.json`, hash e ricevute: metodi e provenienza.

Aggiornato il precedente report sul trasferimento con una nota che rimanda
al controllo esatto. Nessun sorgente di produzione modificato in questi due
esperimenti; il codice riusa la diagnostica precedente senza riscriverla.

## TODO da discutere, non avviati

- Decidere se isolare prima l'esplorazione con rumore assoluto comune alle
  policy, mantenendo distinti target e politica media.
- Preparare il confronto di apprendimento RIGHT/LEFT con FSM v3, stesso
  ordine degli ingressi, inizializzazione, raccolta dati e budget di
  transizioni; preservare lo stato dell'optimizer nelle eventuali riprese.
- Valutare l'oscuramento soltanto come passaggio successivo distinto, senza
  assumere che il training ex-novo risolva automaticamente la robustezza.
