# Screening RIGHT39 e continuazione dalla 40 alla 60

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 23 settembre 2026. Screening concluso alle 19:46 CEST; continuazione
avviata alle 19:47 CEST. Al momento di questo resoconto la prima iterazione
aggiuntiva, la 41, è in esecuzione: il training alla 60 non è ancora concluso.

## Problema e decisione

Il training RIGHT39 fino alla 40 aveva migliorato nettamente il completamento
degli episodi rispetto alla variante con riferimento sinistro, lasciando un
divario cinematico rispetto a V26 e irregolarità nei cicli FSM. L'utente ha
autorizzato uno screening stocastico seguito dalla continuazione, specificando
di fermarsi alla **60 assoluta**, senza proseguire automaticamente alla 100.

Lo screening sostiene la prosecuzione concordata. La ripresa usa il checkpoint
completo della 40; il checkpoint best35 resta conservato. Il risultato sui
cinque seed non dimostra una regressione tale da giustificare un cambio della
sorgente di ripresa, né dimostra equivalenza con V26.

## Strategia e risultati dello screening

Eseguiti dieci nuovi rollout: best35 e last40, ciascuno con rumore originale,
seed gaussiani 123–127, seed ambiente 123, start a 12,99 s e limite 500 passi.
Massimo quattro simulazioni contemporanee, senza sovrapposizione con il
training. Il confronto aggiunge dieci rollout archiviati: V26 best87 e
LEFT39 best72 sugli stessi seed.

| Policy | Episodi completi | Almeno due cicli validi | Completi, almeno due cicli e zero eventi invalidi | Eventi invalidi totali |
|---|---:|---:|---:|---:|
| RIGHT39 best35 | 5/5 | 5/5 | 1/5 | 10 |
| RIGHT39 last40 | 4/5 | 3/5 | 1/5 | 6 |
| V26 best87 nel contenitore attuale | 5/5 | 3/5 | 2/5 | 8 |
| LEFT39 best72 | 3/5 | 3/5 | 1/5 | 3 |

L'unico fallimento dei nuovi rollout è **last40, seed125**: termina al passo
301 per penetrazione, con massimo registrato di 28,815 mm, un ciclo valido e
zero eventi invalidi. Non è un errore software né una terminazione della FSM.
Non si sono verificati interventi del limite di penetrazione hard nei dieci
nuovi casi. Best35 raggiunge un massimo registrato di 23,757 mm.

Anche V26 best87, valutata nel contenitore attuale con FSM v3, completa due
episodi senza cicli FSM validi. Questi non sono rollout stocastici originali
d'agosto. Arrivare a fine episodio e avere cicli regolari sono misure distinte;
anche raggiungere due cicli validi non implica assenza di eventi invalidi.

La qualità cinematica V26 non è ancora recuperata. Sul prefisso fisico comune
di 301 passi, le RMSE medie rispetto al riferimento sono:

| Policy | Ginocchio | Caviglia |
|---|---:|---:|
| RIGHT39 best35 | 10,98° | 4,55° |
| RIGHT39 last40 | 9,77° | 4,60° |
| V26 best87 | 6,38° | 2,88° |

I campioni gaussiani standard coincidono per seed, ma ogni policy conserva
la propria deviazione standard appresa e dipendente dallo stato. Non è dunque
un confronto a perturbazione assoluta identica. Cinque seed a start fisso
sono uno screening; non repliche indipendenti di training o una prova di
equivalenza statistica. V26 best87 è inoltre più matura dei checkpoint 35/40.

## Continuazione avviata

- Ripresa da `milestone_iteration_000040/checkpoint_last`, inclusi actor,
  critic, optimizer Adam e stato PPO.
- Unica variazione di configurazione: budget finale da 40 a 60. Riferimento
  destro V26, FSM v3, clock e prescribed visibili al 100%, velocità non divise
  per 10, reward e governor restano quelli del run precedente.
- 13 worker, 14 CPU Ray, un thread numerico per processo; batch da 4096.
- 20 iterazioni aggiuntive, 81.920 nuove transizioni nominali, totale 245.760.
- Run conservato in `MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23`:
  il nome della cartella mantiene `40iter`, lo storico viene esteso fino a 60.
- Monitor PID 56530; supervisore trainer PID 56531. Arresto previsto alla 60,
  senza rollout o continuazioni ulteriori automatiche.
- ETA indicativa alle 22:30–22:50 CEST, basata sul tempo del run precedente.

Il checkpoint preserva lo stato di apprendimento, ma gli episodi fisici e
lo stato casuale dei processi ripartono: la continuazione non è identica bit
per bit a un processo mai interrotto. Il monitor conserva lo storico 1–40,
controlla le nuove iterazioni e alla 41 completa la verifica del primo update.

## File aggiunti e verifiche

Nessuna modifica a codice di produzione, plugin SEA, simulatore root o FSM.
Gli script e la documentazione nuovi sono nei seguenti percorsi:

- [Screening: protocollo e script — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-e168bf9a96219724>)
- [Metriche complete dello screening — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-df4df5c4c496d999>)
- [Preparazione e monitor della continuazione — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-894f4299d2bc5af0>)
- [Configurazione 40–60 — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-bab82e1793b045de>)
- [Stato aggiornato della continuazione — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-03d11e7be18281db>)
- [Grafico comparativo della penetrazione](../../plot/09_23_2026_right_screening_comparison/paired_penetration.png)

Verifiche concluse: dieci rollout senza errori tecnici; integrità dei 62 input
dello screening; ricostruzione del rumore, delle azioni e della reward;
corrispondenza di target e clock con il riferimento V26; prescribed visibili;
analisi comparativa dei venti episodi e ispezione della figura salvata in `plot/`.

Il preflight della continuazione non ha eseguito training o simulazioni.
Ha verificato pesi esatti, 12 stati Adam associati ai parametri vivi con
contatore 3240 e coefficiente KL 0,20000000298. Congelati 145 input e 36 file
del milestone40; preservati anche metadati e storico precedenti.

Gli audit prodotti dal processo di training alle 19:47:45 confermano il
ripristino effettivo dalla 40, senza reinizializzazione dell'actor, con optimizer
e stato PPO verificati. Il controllo del primo update e dell'arresto esatto
alla 60 resta a carico del monitor durante l'esecuzione.

## Da verificare al termine

- Confermare completamento esatto alla 60 e audit del monitor superati.
- Analizzare completamento, penetrazione, apprendimento e irregolarità FSM
  delle iterazioni 41–60, confrontandole con 35/40 e con V26.
- Decidere la valutazione successiva sui checkpoint salvati. Nessuna ulteriore
  simulazione, continuazione a 80/100 o oscuramento è accodata da questo piano.
