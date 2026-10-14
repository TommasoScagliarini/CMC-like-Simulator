# Risposta dinamica a un impulso di azione

Diagnostica su policy congelate LEFT best80 e RIGHT last60, stesso modello AB06.
La baseline di ciascuna policy è un rollout nominale nuovo da 500 passi, verificato
esattamente contro il rispettivo archivio per azioni, osservazioni e segmenti.
`run_case.py` esegue i rollout; `analyze.py` analizza soltanto artefatti completati.
Non si svolgono aggiornamenti PPO o modifiche di reward, governor, SEA o FSM.

## Protocollo

Un impulso normalizzato +0,005 viene sommato a una sola azione, knee oppure
ankle, per un segmento di 10 ms. L'indice zero-based 101 è il primo indice >=100
con entrambi i comandi delle due policy lontani dal clipping di almeno l'ampiezza
dell'impulso. L'indice 100 non è valido: il ginocchio RIGHT è già saturo.
La regola non usa l'esito della risposta perturbata. Il protocollo effettivo
di ciascun caso è letto dal suo `diagnostic_protocol.json`.

Le prove di impulso possono terminare intenzionalmente dopo **250 passi**:
con impulso alla decisione 101, ciò osserva circa **1,49 s di risposta fisica**
e **1,48 s di decisioni successive**. Il raggiungimento di questo budget è
distinto dal completamento dell'episodio intero di 500 passi. Non è un fallimento.
Le sequenze offline eventualmente eseguite dallo stesso runner non vengono
analizzate come impulsi da questo script.

## Verifiche e tempi

- Il prefisso fino all'impulso deve essere identico alla nuova baseline; anche
  l'osservazione alla decisione perturbata e la media prodotta sono identiche.
- Viene verificato un solo intervento, sul giunto previsto, con ampiezza corretta
  dopo il clipping. Comandi registrati, azioni grezze nel trace e azioni applicate
  devono coincidere secondo la trasformazione documentata.
- Medie e comandi sono indicizzati al **tempo di decisione**, inizio segmento.
  Cinematica, reference e FSM sono indicizzate alla fine del segmento. La prima
  decisione influenzabile è +10 ms, senza aggiungere artificialmente un segmento.
- I segnali di contatto provengono dal campionamento/caching della diagnostica
  reward, non da una misura continua simultanea del wrench. Le coppie SEA sono
  ricavate dagli STO a frequenza maggiore.
- Righe terminali parziali o senza avanzamento temporale sono escluse. Solo il
  prefisso fisicamente completo e comune delle due prove entra nelle statistiche.
  Le coppie vengono interpolate nel solo intervallo temporale comune, mai
  extrapolate oltre la baseline.
- Le finestre 0,05/0,1/0,2/0,5/1/2/4 s vengono pubblicate solo se sono
  interamente osservate sia per la fisica sia per le decisioni. Con 250 passi
  le finestre 2 e 4 s sono omesse; non vengono riempite con dati più brevi.

## Significato delle metriche

`requested_horizon_reached` certifica il budget richiesto senza terminazione
fisica; `full_episode_completed_500` certifica l'intero episodio nominale.
`failed_during_requested_horizon`, `early_failure` e `evaluation_status`
rendono espliciti gli arresti fisici o un'incompletezza della registrazione.
Il risultato riporta separatamente i massimi tempi effettivamente osservati
per risposta fisica, decisioni e coppie.

Il rapporto tra differenza della media e impulso applicato è una **risposta
a perturbazione finita**; il corrispondente rapporto sul comando applicato
include il clipping. Non sono automaticamente Jacobiani, fattori di crescita
asintotica o certificati d'instabilità. Le differenze sono riportate anche
in gradi, rad/s, rad/s², N, Nm e millimetri.

L'appaiamento isola l'intervento entro ciascuna policy. LEFT e RIGHT possono
trovarsi in stati fisici diversi allo stesso istante, nonostante il modello
sia comune. Un singolo tempo e una sola direzione/ampiezza dell'impulso non
dimostrano robustezza generale. La prima divergenza FSM è indicata nei grafici:
le risposte successive comprendono anche tale cambiamento della dinamica ibrida.

`first_response_explanation.json` ricostruisce inoltre il segmento perturbato
con il governor esatto e con lo stesso stato interno iniziale della relativa
baseline. Riporta q/qdot/qddot, limiti, jerk richiesto prima del clamp e numero
di sottopassi limitati per giunto. Le diagnostiche aggregate e gli endpoint
serviti sono verificati contro il rollout fisico. Questo controllo è necessario:
un impulso non clippato all'ingresso può essere assorbito dal jerk limiter,
quindi la perturbazione meccanica iniziale può essere diversa fra LEFT e RIGHT.

L'analisi knee osserva nel segmento 101 una variazione di qddot knee
di circa **0,4681 rad/s² in LEFT** e **zero in RIGHT**. Il jerk limiter knee è
inattivo in LEFT e attivo su tutti i sottopassi in RIGHT. La rapida propagazione
iniziale LEFT e l'attenuazione iniziale RIGHT non sono quindi un confronto a
parità di disturbo della reference, pur usando lo stesso impulso normalizzato.

Il confronto ankle non presenta questo assorbimento iniziale: il jerk ankle
non è limitato in nessuno dei due casi e l'impulso produce pressoché lo stesso
incremento della reference: q ≈8,3407e−6 rad, qdot ≈0,00239448 rad/s,
qddot ≈0,436907 rad/s². Lo scarto fra LEFT e RIGHT nella variazione qddot è
inferiore a 6,6e−7 rad/s². Gli altri stati restano differenti e il ginocchio
RIGHT è comunque nel regime di jerk limitato: non è un confronto dello stesso
stato completo.

## Risultati dei quattro impulsi

Tutti e quattro raggiungono i **250 passi richiesti**, senza terminazione
fisica; le prove osservano 1,49 s fisici e 1,48 s di decisioni dopo l'impulso.
Le finestre pubblicate arrivano a 1 s. Nessun caso pending rimane.

| Impulso | Prima risposta applicata > impulso | Prima differenza q >0,1° | Prima differenza forza >1 N | Prima divergenza FSM |
|---|---:|---:|---:|---:|
| LEFT knee | 10 ms | 60 ms | 70 ms | 470 ms |
| LEFT ankle | 10 ms | 70 ms | 70 ms | 470 ms |
| RIGHT knee | 220 ms | 340 ms | 300 ms | 680 ms |
| RIGHT ankle | 10 ms | 140 ms | 120 ms | 630 ms |

Le soglie sui comandi usano la norma dei due comandi normalizzati; quelle
sugli angoli si attivano quando almeno un giunto supera la differenza indicata.
Le differenze iniziali di policy, cinematica e contatto precedono la prima
divergenza FSM in tutte le prove.

| Impulso | Massimo rapporto risposta applicata / impulso entro 50 ms | Entro 200 ms |
|---|---:|---:|
| LEFT knee | 115,93 | 365,31 |
| LEFT ankle | 106,80 | 331,67 |
| RIGHT knee | 0,0174 | 0,5827 |
| RIGHT ankle | 4,218 | 108,59 |

Il forte aumento osservato in LEFT coinvolge anche l'impulso ankle, che
produce un disturbo iniziale della reference comparabile a RIGHT. È evidenza
locale di maggiore sensibilità della traiettoria LEFT in questa condizione,
senza identificare una causa esclusiva o una proprietà universale della rete.
Le risposte più lunghe comprendono crescita nonlineare, clipping, contatto e
poi cambiamenti FSM; i grandi rapporti finiti non vanno interpretati come
guadagni costanti da moltiplicare ad ogni passo.

Le metriche comprendono RMS e massimo delle differenze di coppia SEA per
giunto e finestra; queste sono variazioni rispetto alla baseline della stessa
policy, non misure assolute di plausibilità fisiologica della curva.

## Artefatti

L'analisi, dopo la conclusione degli impulsi, produce `metrics.json`,
`first_response_explanation.json`, `input_hashes.json`, `analysis_verification.json` e grafici diagnostici sotto
root `plot/09_29_2026_general_method_diagnosis/dynamic/`. I casi privi di ricevuta
finale sono registrati come pending, non classificati come fallimenti.

```sh
python "Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/dynamic/analyze.py"
```

Lo script usa percorsi relativi al repository. L'esecuzione live Windows resta
da verificare. I casi analizzati e quelli ancora pending sono elencati nella
ricevuta `analysis_verification.json`.
