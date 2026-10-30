# Audit della reward LEFT: fasi, contatto e clipping

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 24 settembre 2026. Analisi richiesta dopo il confronto RIGHT39/V26, per
verificare se la reward storica produca incentivi adeguati al riferimento
sinistro. Tutte le prove di questo audit usano dati già registrati: **nessun
training o rollout fisico nuovo, nessuna modifica alla configurazione attiva**.

## Esito e decisione operativa

La reward è calcolata correttamente e distingue diverse carenze della policy
LEFT. Non abbiamo trovato un'inversione generale dell'obiettivo: il comando
diretto del riferimento LEFT riceve più reward della policy LEFT72 sotto la
reward attuale e sotto tutte le varianti offline esaminate.

È però emerso un compromesso pertinente alla robustezza: nei due fallimenti
del seed 127, la penetrazione cresce con momento GRF positivo; la penalità
anti-flip non intercetta la salita finale e la penalità di penetrazione si
attiva a ridosso del collasso. Nei dieci passi completati prima dell'arresto,
la reward corrente separa pochissimo quelle traiettorie dal deterministico
che sopravvive. Raddoppiare **soltanto** il peso di penetrazione da 1 a 2 rende
più netta quella separazione sui dati registrati.

Questo motiva un candidato per un confronto controllato LEFT, non certifica
una soluzione. La fase iniziale di apprendimento, la cinematica incompleta e
la fragilità residua non sono ricondotte a una sola causa. Non sarebbe
corretto concludere che occorra semplicemente aumentare tutti i pesi fisici,
raddoppiare quello della caviglia oppure ridurre il costo dei comandi.

## Dati e metodo

Sono stati analizzati **20 episodi distinti, 9.738 righe di trace**:

- dieci episodi deterministici/diagnostici: LEFT25/40/60/72/80/100, comando
  diretto LEFT e RIGHT, RIGHT60 e V26 best 87 storico;
- dieci episodi LEFT72 con cinque seed 123–127 e rumore originale/dimezzato;
- TensorBoard e contatori delle prime 25 iterazioni LEFT e RIGHT, a parità
  di 4096 transizioni per iterazione, prima del problema di ripristino LEFT.

I componenti dello scalar sono ricalcolati con `compute_reward` e la
`reward_config` effettivamente salvata nel rispettivo rollout. **I campi
`reward_terms.penalty` e alcuni score del trace appartengono al calcolo
preliminare dell'ambiente**, non al wrapper che determina il ritorno usato
nel training. Non sono stati confusi con i contributi finali.

La singola riga terminale hard del seed 127 ha tempo duplicato e diagnostiche
di contatto azzerate: entra nella ricostruzione del ritorno, ma viene esclusa
dalle statistiche della traiettoria fisica. I confronti fra episodi di durata
diversa usano un prefisso comune di 303 passi o finestre temporali appaiate.
Gli stessi seed riproducono le estrazioni gaussiane standard; non impongono
perturbazioni assolute uguali quando cambia la deviazione standard della policy.

Per i deterministici LEFT e il comando diretto LEFT sono verificati gli stessi
500 target articolari. Le fasi sono dieci intervalli della **fase prescribed
sinistra**, distinti dallo stato della FSM fisica. RIGHT/V26 sono contesto
descrittivo: il loro ritorno riguarda un altro riferimento.

## Formula effettiva e normalizzazione

Per gli esperimenti analizzati:

```text
L = errore_pos_knee² / range_knee²
    + 2 errore_pos_ankle² / range_ankle²
    + .02 errore_vel_knee² / range_vel_knee²
    + .04 errore_vel_ankle² / range_vel_ankle²

S = .8 / (1 + 8 L_served) + .2 / (1 + 4 L_actual)
reward = clip(S − penalità_base, 0, 1)
         − 2 safety_loss − 1 penetration_loss
         − .1 ankle_moment_flip_loss − 2 out_of_band_loss
```

Ogni errore quadratico normalizzato ha inoltre un cap 25. La normalizzazione
concordata è presente; il suo cambiamento fra target non è classificato come
bug. `served` indica la reference filtrata fornita al controllore, `actual`
il movimento effettivo. I premi ex novo di avanzamento/completamento dei
cicli sono disattivati; restano penalità terminali e perdita delle reward future.

I deficit dei due score non lineari vengono ripartiti proporzionalmente fra
le quattro componenti di loss. Questa è una contabilità esatta prima del
clipping: **non è il guadagno causalmente ottenibile correggendo un solo
giunto**, né un gradiente PPO.

## Quanto pesa ogni problema nel deterministico LEFT72

| Contributo su 500 passi | LEFT72 | Comando diretto LEFT |
|---|---:|---:|
| Deficit score attribuito alla posizione knee | 38,273 | 36,485 |
| Deficit score attribuito alla posizione ankle | 137,289 | 67,984 |
| Deficit attribuito alle due velocità | 1,865 | 4,239 |
| Costo command-rate | 77,831 | 20,026 |
| Altri costi della base | 0,339 | 1,149 |
| Penalità momento GRF | 43,108 | 0 |
| Penalità penetrazione | 0,139 | 3,344 |
| Altri costi dopo clipping | 0,260 | 0 |
| **Ritorno** | **200,895** | **366,774** |

Il deficit maggiore riguarda la caviglia. Il movimento richiesto non è
ignorato dalla reward: ridurre ipoteticamente del 10% soltanto l'errore di
posizione served ankle, congelando tutto il resto, aggiunge 11,429 punti;
la stessa variazione relativa sul ginocchio aggiunge 3,517 punti. È una
sensibilità dello scalar, non un intervento fisico realizzato.

La fase rivela problemi differenti:

- **20–30%:** reward media LEFT72 −0,058; il costo medio anti-flip è 0,545
  per passo. Il corrispondente comando diretto ha reward 0,900 e anti-flip 0.
- **30–40%:** errore ankle LEFT72 di 9,10° RMSE, deficit ankle medio 0,515
  e reward 0,275. È il bin con maggiore beneficio dalla correzione locale
  ipotetica della caviglia.
- **70–80%:** errore knee LEFT72 di 12,41° RMSE; qui si concentra il maggiore
  beneficio locale ipotetico della correzione del ginocchio.

Anche il comando diretto ha difetti: nella fase 60–70% la reward media scende
a 0,167, con errore ankle 10,01° e knee 10,98°. Ha il target dell'endpoint 10ms
successivo, attraversa il filtro e raggiunge circa 998 N e 22,79 mm nel rollout.
La sua maggiore reward non ne fa una policy pronta, robusta o perfettamente
imitativa. LEFT72 è il best selezionato nel training, non il massimo ritorno
del rollout deterministico fra tutti i checkpoint.

[Grafico della scomposizione](../../plot/09_24_2026_left_reward_audit/reward_decomposition.png)
e [contributi per fase](../../plot/09_24_2026_left_reward_audit/phase_contributions.png).

## Clipping: quasi assente nei checkpoint maturi, presente all'inizio

Il clipping esterno interviene in 0/500 passi per ciascuno dei deterministici
LEFT40/60/72/80/100. In LEFT25 interviene in 6 passi, con una correzione totale
di +0,119863 punti. Nei dieci episodi rumorosi LEFT72 interviene solo 2 volte
su 4738 righe, con correzione totale +0,004996. Le penalità fisiche applicate
dopo il clipping restano comunque attive.

Non abbiamo esteso questo risultato al training iniziale senza controllarlo.
I TensorBoard mostrano effettivamente un clipping importante nelle prime
iterazioni, ma **maggiore in RIGHT**, che poi impara meglio:

| Iterazioni | Correzione media clipping LEFT | RIGHT |
|---|---:|---:|
| 1–5 | 0,04911 | 0,08063 |
| 6–10 | 0,02868 | 0,04946 |
| 11–15 | 0,01253 | 0,02592 |
| 16–20 | 0,00332 | 0,01122 |
| 21–25 | 0,00114 | 0,00441 |

Sono medie di cinque scalari registrati, ottenuti da medie mobili di 100 passi
per worker, non medie esatte delle 4096 transizioni né percentuali di passi
clippati. La ricostruzione lineare preserva l'operatore di media; non applica
erroneamente il clipping alla loss media. L'identità fra componenti e reward
è verificata entro precisione float32.

Nelle iterazioni 21–25 LEFT ha uno score imitativo aggregato maggiore
(0,5716 contro 0,4860), ma completa 19/51 episodi contro 32/42. Score normalizzati
su riferimenti diversi non costituiscono una metrica comune di robustezza.
Questi dati non sostengono il clipping come causa selettiva del divario LEFT.

## Il costo dei comandi: importante, ma non è una prova per ridurlo

In LEFT72 il 94,2% del costo command-rate deriva da `segment_delta_loss`:
73,280 dei 77,831 punti. Questa componente misura la distanza fra **endpoint
grezzo della policy e reference filtrata corrente all'inizio del segmento**.
Non misura direttamente la differenza tra due azioni grezze consecutive.

Il costo raggiunge un cap oltre 0,25 rad knee e 0,20 rad ankle. In LEFT72 accade
nel 43,4%/64,0% dei passi; in RIGHT60 nel 49,4%/34,0%; nel comando diretto LEFT
nel 5,4%/5,6%. È una saturazione interna diversa dal clipping esterno della
reward. Oltre la soglia questo termine non distingue ulteriori aumenti
della distanza; gli altri termini possono ancora distinguerli.

Può esistere un compromesso con l'anticipo necessario per pilotare il filtro,
ma il costo elevato può anche essere una conseguenza della policy appresa.
Nei primi cinque aggiornamenti il costo command-rate è quasi identico:
0,18707 LEFT e 0,18697 RIGHT, circa 98,4%/98,5% della penalità base.

Dimezzarne il peso sui trace congelati aggiunge 38,916 punti a LEFT72 e solo
10,013 al comando diretto LEFT. Il divario favorevole al comando diretto si
riduce da 165,879 a 136,977. Non è un'evidenza che abbassare il peso produca
traiettorie migliori; manca la risposta della policy al nuovo training.

## I fallimenti non sono tutti inversioni del momento GRF

**Seed 125, rumore originale.** Al passo 298 la penetrazione è 16,82 mm ma il
momento GRF è circa −60 Nm: anti-flip già al cap 2,5 e reward −2,264. Al passo 303
la penetrazione arriva a 28,41 mm e la reward a −5,389. Il caso non supporta
una mancata penalizzazione locale: nella finestra finale il segnale è già
fortemente negativo.

**Seed 127, entrambe le scale.** La salita finale della penetrazione avviene
con momento GRF positivo, quindi senza anti-flip. L'ultima riga fisica con
rumore originale registra 27,40 mm,1028 N,+55,9 Nm e reward −0,746; segue il
terminale hard. Con rumore dimezzato termina a 28,24 mm e reward −2,789.

Le ultime osservazioni sotto 20 mm precedono l'ultima osservazione fisica di
40 ms nel seed 125,30 ms nel seed 127 originale,70 ms nel seed 127 dimezzato.
La penalità di penetrazione è zero sotto 20 mm e cresce quadraticamente oltre
la soglia: a 21 mm costa 0,015625, a 24mm 0,25, a 28mm 1, prima del costo terminale.

Esistono stati oltre 20 mm con reward positiva, anche in episodi che poi
recuperano. Questo non prova che il ritorno globale favorisca un fallimento:
contano le reward successive e il valore perso alla terminazione. Per esempio,
negli ultimi 20 passi fisici del seed 127 originale la reward media è +0,485
contro −0,349 del deterministico che sopravvive: quest'ultimo imita peggio e
paga molto più anti-flip. Nei passi finali del collasso la reward diventa
negativa. Gli stati e le storie delle azioni sono differenti.

[Grafico dei due meccanismi](../../plot/09_24_2026_left_reward_audit/physics/paired_failure_reward_contact.png).

## Una variante mirata valutata offline

Sono state ricalcolate otto varianti oltre al controllo: command-rate
dimezzato/azzerato, blend served/actual 50/50, peso posizione ankle raddoppiato,
sharpness served dimezzata, limite clipping inferiore −1, anti-flip raddoppiato,
penetrazione raddoppiata. Nessuna modifica è stata applicata al training.

La variante più direttamente collegata ai due fallimenti 127 è
`grf_penetration_weight: 1.0 → 2.0`. Sui dieci passi completati prima del
terminale, confrontati agli stessi indici del deterministico:

| Caso | Differenza ritorno fallito−nominale attuale | Con penetrazione×2 | Con anti-flip×2 |
|---|---:|---:|---:|
| Seed 125 originale, passi 293–302 | −5,141 | −6,344 | −5,916 |
| Seed 127 originale, passi 456–465 | −0,030 | −1,219 | +0,030 |
| Seed 127 dimezzato, passi 459–468 | −0,062 | −2,525 | +0,044 |

Sono somme locali su 100 ms, escludendo il terminale. Nei casi 127 il segnale
attuale è quasi equivalente fra finestra pericolosa e nominale; aumentare
il peso della penetrazione separa meglio quelle finestre. Aumentare solo
anti-flip non affronta il meccanismo e può peggiorare la separazione.

Questo è un **candidato diagnostico**, non un peso ottimizzato: non anticipa
l'attivazione sotto 20 mm, non corregge da solo la cinematica e potrebbe
favorire una strategia che scarica il piede. Un eventuale training deve
valutare anche contatto, cicli, escursioni e coppie. La variante riduce il
ritorno nominale LEFT72 di 0,139 punti e quello del comando diretto LEFT
di 3,344; anche quest'ultimo ha penetrazione e viene correttamente penalizzato.

L'override isolato è conservato in
[candidate_penetration_weight_2.json — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-3b5e3afe30d1a721>).
È un frammento da fondere con la configurazione LEFT, non una configurazione
completa e non è stato attivato.

## Terminale hard e assegnazione temporale del merito

Nel terminale hard del seed 127 il tempo non avanza nella trace, il segmento
è interrotto e i termini di contatto vengono riportati a zero. La reward
è comunque **−1,900799**, con penalità safety 2; non è un fallimento privo
di penalizzazione. Le loss imitative conservano valori non nulli.

Alla sola soglia hard 30mm, la formula ordinaria della penetrazione darebbe
già 1,5625 punti. Questo è un calcolo alla frontiera, non il recupero del
campione interrotto. Raddoppiare il peso lascia zero un termine registrato
zero: la coerenza quantitativa del terminale hard merita un controllo
separato, senza inventare momento GRF o stato fisico mancanti. Non spiega
da sola il divario LEFT, che comprende molte terminazioni ordinarie.

Gamma 0,99 con passo 10 ms pesa un costo distante 1s con fattore 0,366. La
propagazione geometrica diretta dei residui GAE con lambda 0,9 si dimezza in
circa 60 ms, ma il value bootstrap può trasmettere conseguenze più lontane:
non è un orizzonte rigido. I parametri coincidono con V26 e RIGHT. Ritorno
episodico, ritorno scontato e advantage PPO non vanno identificati fra loro.

## Strategia successiva e limiti

Per studiare la robustezza LEFT il confronto più circoscritto emerso è una
coppia di continuazioni dallo **stesso checkpoint LEFT72**: controllo con
peso 1 e variante con peso 2. Stessi pesi iniziali, protocollo critic/optimizer,
budget di transizioni, seed e valutazione; prescribed e clock visibili,
riferimento sinistro senza shift, FSM v 3 e resto della ricetta conservati.
La gestione del terminale hard deve essere esplicita e comune ai bracci.

Gli esiti vanno misurati con completamento, margine di penetrazione, carico,
cicli, RMSE per fase ed escursioni/coppie. I ritorni definiti da reward
diverse non sono una misura comparativa comune. Un eventuale beneficio
dimostrerebbe il contributo di quel peso alla continuazione, non spiegherebbe
automaticamente l'intero apprendimento LEFT da zero.

L'audit conserva un solo seed di training, un solo start fisico e pochi
seed di rumore. I campioni temporali sono correlati. Il comando diretto
non è una policy appresa e le modifiche offline non generano nuove azioni.
Non è stata dimostrata una causa unica né equivalenza statistica a V26.

## File prodotti e verifiche

Tutto il codice nuovo è in
[left_reward_audit_2026-09-24 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-3fb9b5d3ac66bb4f>):

- `analyze.py`, `metrics.json`, `summary.csv`, `phase_summary.csv`, dieci
  `steps_*.csv`, `sources.json`, `input_hashes.json`, `verification.json`;
- `physics/`: analisi degli 11 episodi LEFT72, finestre appaiate e
  `training_initial_probe.py/json/md` per i primi 25 aggiornamenti;
- `sensitivity/`: ricostruzione dello scalar, otto varianti, cap interni,
  sensibilità per fase e finestre pre-fallimento;
- quattro figure in [plot/09_24_2026_left_reward_audit](../../plot/09_24_2026_left_reward_audit/).

Le somme dei ritorni coincidono con gli originali entro 1e−8; la verifica
osservata è dell'ordine di 10⁻¹³. Le identità per passo e le aggregazioni per
fase sono controllate numericamente; le sei componenti del command-rate e
il segment delta sono ricostruiti dalle reference salvate. Gli input hanno
manifest SHA-256 e verifica di immutabilità. Le figure sono state ispezionate.
Una revisione indipendente ha verificato contabilità, limiti dei confronti,
trattamento del terminale e distinzione fra clipping esterno e cap interni.

Non sono stati modificati codice di produzione, checkpoint, preset, FSM,
governor, controllori SEA o plugin C++. Non sono stati eseguiti training,
rollout nuovi o forward della policy.

## TODO

- [ ] Definire ed eseguire il confronto LEFT72 con peso di penetrazione 1 contro 2, con budget e criteri fisici comuni, se si procede con questa ipotesi.
- [ ] Verificare il trattamento quantitativo della penetrazione nel terminale hard prima di attribuire gli esiti alla sola taratura dei pesi.
- [ ] Distinguere un eventuale beneficio nella continuazione LEFT da una spiegazione del divario nell'apprendimento da zero; quest'ultima richiede un controllo dedicato.
