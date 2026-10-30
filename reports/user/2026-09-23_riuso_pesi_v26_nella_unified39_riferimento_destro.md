# Riuso della policy V26 nella unified39 con riferimento destro storico

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 23 settembre 2026. Esperimento diagnostico autorizzato dall'utente, senza
training. Checkpoint sorgente: best87 del training imitativo del 20 agosto.
FSM mantenuta alla versione v3. Baseline operativa dell'utente: unified39 best72.

**Aggiornamento successivo del 23 settembre:** il controllo che conserva
l'ordine aritmetico originale, riordinando gli ingressi, ha riprodotto esattamente
le 500 azioni storiche e i file di cinematica, coppie SEA e GRF. Le differenze
residue descritte in questo report riguardano il primo percorso con colonne
dei pesi permutate: sono state ricondotte all'amplificazione delle differenze
floating point, mantenendo la stessa FSM v3. Vedi la
[verifica del controllo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-1c907e94ec1cde22>)
e i [nuovi grafici canonici](../../plot/09_23_2026_2/).

## Risultato e significato

**La policy V26 trasferita nella struttura unified39 completa il rollout e
recupera una qualità cinematica vicina alla baseline storica.** La 39D può
quindi rappresentare ed eseguire la soluzione storica quando riceve gli stessi
tipi di segnali prescritti. Il divario della best72 sinistra non è spiegabile
con una semplice incompatibilità dimensionale o incapacità dell'architettura.

Il recupero è parziale sul piano delle coppie e degli eventi: il picco positivo
alla caviglia resta superiore allo storico e interviene una risincronizzazione
della FSM. Non abbiamo riprodotto la stessa traiettoria campione per campione,
né verificato la robustezza stocastica. Un episodio deterministico completato
non equivale al recupero della percentuale storica di completamento in training.

## Problema e strategia

Il dubbio era se il contenitore attuale impedisse anche a una policy già buona
di funzionare, oppure se il divario nascesse soprattutto dal nuovo problema
imitativo e dal percorso di apprendimento. È stata riutilizzata la policy
storica senza eseguire aggiornamenti dei pesi.

- Actor: 39 ingressi, due layer da 256 neuroni con tanh, due azioni assolute.
- I quattro target passano dagli indici storici 2–5 agli indici 35–38.
  Permutazione delle colonne del primo layer, nuovo → storico:
  `[0,1,6,…,38,2,3,4,5]`. Copiati entrambi gli alias serializzati del layer,
  invariati bias e layer successivi.
- Template prescribed **destro** storico, guidato dalla fase destra; shift
  ginocchio **0,465**, caviglia **0,452**. Il clock sin/cos dell'actor rappresenta
  la fase destra **non sfasata**, come nel V26.
- Sei canali prescritti sempre visibili; velocità in rad/s senza divisione
  per 10. Reward e normalizzazione costruite sul target destro storico.
- Reset a **12,99 s**, offset 1 s, durata 5 s, 500 passi da 10 ms, seed 123,
  azioni deterministiche. Governor, SEA e FSM v3 della pipeline attuale.
- Export di sola inferenza: il critic non è presente nel checkpoint esportato
  storico e non è stato trasferito. Nessun optimizer o stato Adam utilizzato.

La sottoclasse diagnostica e l'export hanno un contratto RIGHT dedicato. I
validatori ordinari del percorso LEFT restano invariati e rifiutano questo
export. Non è un checkpoint pronto per continuare il training.

## Controlli effettuati

1. Equivalenza di media e logstd su **500 osservazioni storiche e 4.096 ingressi
   sintetici**: differenza massima **7,15×10⁻⁷**. Riprodotte le azioni archiviate
   e verificato il ricaricamento dell'export convertito.
2. Controllo negativo: omettendo la rimappatura degli ingressi l'errore massimo
   sulle azioni supera 4,7. La verifica è sensibile a una permutazione errata.
3. Controllo durante tutti i 500 passi: rete storica e rete convertita sui
   medesimi segnali correnti coincidono entro **2,38×10⁻⁷** nei logits.
4. Target q/qdot, sei ingressi prescritti effettivi e quattro escursioni di
   normalizzazione: differenza **zero** rispetto ai campioni storici.
5. Stato fisico, target e ingressi iniziali coincidono con il reset storico.
6. Ricostruzione offline della reward completata; verificati gli hash di 24
   file di input, inclusi checkpoint originali, sorgenti e asset registrati.
7. Watchdog concluso con codice 0: **482,8 secondi** di tempo reale per il
   rollout. Nessun nuovo training. Sette grafici generati, senza canali mancanti.

Nel reset JSON grezzo il clock ereditato disabilitato riporta fase zero. Il
wrapper presenta correttamente all'actor la fase destra, inizialmente circa
0,892481. Il reset grezzo è preservato; la distinzione è documentata nella
ricevuta di verifica. Non è un errore negli ingressi della policy.

## Risultati quantitativi

| Metrica | V26 originale best87 | Pesi V26 nella 39D, RIGHT | Unified39 best72, LEFT |
|---|---:|---:|---:|
| Passi completati | 500/500 | 500/500 | 500/500 |
| Ritorno | 363,79 | 370,62 | 200,90 |
| RMSE ginocchio rispetto al proprio target | 6,91° | 6,68° | 6,56° |
| RMSE caviglia rispetto al proprio target | 1,46° | 1,25° | 5,98° |
| Picco positivo coppia SEA caviglia | 10,19 Nm | 22,16 Nm | 47,44 Nm |
| RMS componente coppia ginocchio >20 Hz, dopo 200 ms | 3,280 Nm | 3,251 Nm | 4,108 Nm |
| Penetrazione massima ai campioni della reward | 14,797 mm | 14,797 mm | 21,833 mm |
| Penetrazione massima nello STO a 1 ms | 16,238 mm | 16,238 mm | 21,838 mm |
| Cicli validati dalla FSM | 2 | 1 | 2 |
| Eventi invalidi registrati | 0 | 1 | 0 |

Le colonne RIGHT condividono lo stesso target; la colonna LEFT è descrittiva,
poiché usa un target diverso. Il ritorno maggiore del trasferimento non basta
a dichiararlo migliore dello storico: peggiorano coppia positiva e conteggio
dei cicli validi. Le normalizzazioni non nascondono una differenza tra i due
bracci RIGHT: sono state verificate identiche.

La penetrazione non è nulla. La differenza fra 14,8 e 16,2 mm dipende dalla
frequenza di campionamento delle due misure, e si presenta allo stesso modo
nello storico e nel trasferimento. Il massimo comune coincide con la fase
iniziale; non implica che tutti i contatti successivi siano identici.

La componente sopra 20 Hz è un indicatore quantitativo omogeneo del chattering,
ottenuto con lo stesso filtro Butterworth per tutti i rollout. Non definisce
da sola la qualità complessiva della coppia: il ginocchio trasferito presenta
anche un minimo più negativo, −38,59 Nm contro −29,97 Nm storici, dopo 200 ms.

Anche la forma cinematica conserva una differenza rilevante: il picco di
flessione del ginocchio scende da **63,78° a 55,63°**. L'RMSE globale vicino o
inferiore non significa che tutti i tratti del passo siano stati recuperati.
Il ritorno non penalizza esplicitamente gli eventi FSM in questa configurazione;
può quindi crescere anche mentre diminuisce il numero di cicli validati.

## Differenze ancora presenti e loro origine osservabile

L'episodio completo contiene un HS rifiutato come troppo precoce al passo
**159**, una risincronizzazione al **167** e un ciclo scartato come degradato
al **262**. Per questo il conteggio finale è di un solo ciclo validato. È una
distinzione importante rispetto al numero di oscillazioni visibili e rispetto
al completamento temporale dell'episodio.

La divergenza cinematica inizia prima della differenza nei segnali FSM:

- Ingressi iniziali identici; differenza nella prima azione circa **5,96×10⁻⁸**.
- Ai primi passi le differenze cinematiche sono dell'ordine di 10⁻⁶ gradi.
- L'errore fra le due traiettorie supera 0,1° alla caviglia al passo **37** e
  al ginocchio al **41**. Le accelerazioni della reference, presenti nel
  feedback dell'actor, mostrano già scarti crescenti.
- Le prime differenze osservate nei canali FSM arrivano all'ingresso del
  passo **95**, quando le traiettorie sono già differenti.

Questo è compatibile con amplificazione di piccole differenze numeriche nel
circuito policy–governor–simulatore. La permutazione modifica l'ordine delle
somme floating point pur conservando la funzione matematica. Non abbiamo
isolato sperimentalmente tale effetto da tutte le differenze di esecuzione.
**Non è corretto attribuire tutta la divergenza alla FSM v3**, perché gli
scarti cinematici materiali la precedono; la FSM modifica poi effettivamente
la sequenza degli eventi osservata dalla policy.

Le traiettorie risultanti hanno RMSE simili rispetto al target ma non sono
identiche fra loro: RMS dello scarto tra i due rollout **3,84° al ginocchio** e
**1,21° alla caviglia**, con scarto massimo al ginocchio **10,66°**.

## Indicazioni per proseguire

Il risultato restringe la diagnosi: la struttura 39D e l'esecuzione attuale
consentono una buona imitazione del riferimento storico senza riapprenderla.
Rimane da separare il contributo del target sinistro dal processo con cui la
best72 ha appreso a inseguirlo. Questo test cambia contemporaneamente policy
e target rispetto alla best72, quindi non dimostra che il solo prescribed
sinistro causi il divario.

Il training di controllo con target RIGHT resta informativo, ma ora la sua
domanda precisa è **se il learner attuale riesca a ritrovare una soluzione che
sappiamo essere eseguibile**, non se l'architettura abbia capacità sufficiente.
Un confronto RIGHT/LEFT nella stessa pipeline e a parità di esperienza
permetterebbe di studiare il cambiamento del compito. Il passaggio a oscuramento
ed ex-novo resta una strategia possibile, non una correzione già dimostrata
per i problemi di robustezza.

TODO ancora aperti, nessuno avviato da questo test:

- Quantificare la robustezza stocastica della policy trasferita e confrontarla
  con un riferimento storico valutato con lo stesso protocollo.
- Se prioritario, separare sensibilità numerica e differenze di runtime con
  un controllo che mantenga l'ordine aritmetico storico o riproduca azioni
  registrate. Non richiede cambiare la FSM v3.
- Concordare se procedere al training di controllo RIGHT o al percorso di
  oscuramento della best72; nessun nuovo training è partito.

## File e grafici

Sorgenti nuovi, tutti nel perimetro della rete:

- [probe.py — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-560d4a9edc315066>): conversione, ambiente diagnostico, rollout e watchdog.
- [verify_artifacts.py — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-aa85364c2ac49f66>): ingressi, normalizzazioni, reset e provenienza.
- [analyze.py — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-07d03d07c3dda63f>): metriche confrontabili e grafici.
- [README e protocollo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-8394c096ec7dd6b7>).

Nessuna modifica ai sorgenti di produzione, FSM o plugin SEA; checkpoint
storici e best72 preservati. Il file YAML nel rollout conserva la configurazione
di partenza; il contratto diagnostico e `diagnostic_env_config.json` identificano
esplicitamente l'intervento RIGHT.

- [Sette plot canonici](../../plot/09_23_2026_1/).
- [Confronto V26 / trasferimento / best72](../../plot/09_23_2026_v26_transfer_comparison/comparison.png).
- [Metriche complete — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-1ebcbda9eabdecfc>).
- [Verifiche sugli artefatti — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-15b6a9fdfde0c2f4>).
- [Rollout — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-a922b5bf17052f05>).

![Confronto diagnostico](../../plot/09_23_2026_v26_transfer_comparison/comparison.png)
