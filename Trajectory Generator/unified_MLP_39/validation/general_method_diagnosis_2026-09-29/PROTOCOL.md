# Indagine del metodo generale — 29 settembre 2026

Instruction check token: `CMC_AGENT_OK_2026`.

Richiesta: qualità cinematica = errore di imitazione e forma della traiettoria;
qualità di coppia = principalmente forma della curva. V26 è il benchmark
accettato. Disponibile soltanto AB06: LEFT e RIGHT sono due reference dello
stesso modello, non una validazione di generalizzazione multimodello.

Nessun training, tuning dei pesi, modifica di reward, FSM, governor, controller
o plugin. Gli algoritmi diagnostici e le perturbazioni sono comuni ai due lati.
Le coppie V26 sono un confronto descrittivo e non un target imposto a LEFT.

## Bracci indipendenti

1. `quality/`: confronto di forma/errore e ritorni ricostruiti da trace esistenti.
   Distinguere fase prescribed, eventi FSM e istanti fisici; confrontare reward
   soltanto a target uguale e con configurazione esplicitata.
2. `governor/`: replay della classe di produzione su segmenti salvati; eventuali
   sequenze inverse con stessa procedura LEFT/RIGHT. Fattibilità offline non
   implica fattibilità dinamica. Fallimento solver non prova impossibilità.
3. `feedback/`: inferenza verificata sui checkpoint originali, Jacobiani locali
   scalati e dipendenza dai gruppi di feature. Derivate locali non certificano
   instabilità del circuito completo.
4. `dynamic/`: baseline deterministica e impulso positivo di 0.005 unità di
   azione normalizzata per un solo segmento, al decision index 101 (zero-based),
   separatamente knee e ankle. Stesso start 12.99 s, seed 123. Baseline di 500
   segmenti; impulsi di 250 segmenti (1.49 s di risposta dopo l'intervento).
   Azioni successivamente prodotte dalla policy congelata. LEFT best80
   e RIGHT60; confronto di ciascun perturbato con la propria nuova baseline.

L'impulso è piccolo rispetto al range azione [-1,1]; la conversione in radianti
e l'effetto del clipping vengono registrati. Il tempo comune è un intervento
controllato entro ciascuna policy, non un confronto a identico stato tra policy.
La riproducibilità della baseline e del prefisso prima dell'impulso è verificata.

Preflight prima di qualunque impulso: l'indice inizialmente proposto 100 ha
raw knee RIGHT=1.1843, quindi l'impulso positivo sarebbe interamente clippato.
Si sceglie il primo indice >=100 con tutti e quattro i comandi LEFT/RIGHT
strettamente dentro [-1+epsilon,1-epsilon]: indice101, tempo decisione14.00s.
La selezione dipende dagli input archiviati, non dall'esito di perturbazioni.
In quel punto le forze LEFT/RIGHT sono diverse: la prova è appaiata entro
ciascuna policy e non isola causalmente una differenza fra le due policy.

Ogni rollout fisico ha watchdog (startup 180 s, stall 120 s). Le due baseline
iniziali hanno limite totale 900 s; per le prove successive il limite è 1800 s,
adeguato al throughput osservato di circa 2 s per segmento in due processi.
Questa modifica riguarda soltanto il limite di tempo di calcolo, non durata
simulata, dati o criteri scientifici.
Primo insieme: due baseline e quattro impulsi. Eventuali due prove delle
sequenze inverse hanno il watchdog da 1800 s e restano diagnostiche, senza PPO.
I sette plot canonici vengono salvati sotto root `plot/`, in directory per caso.

Aggiornamento di solo costo computazionale prima di avviare gli impulsi: le
prove d'impulso terminano al segmento250, sufficiente per la risposta locale
fino a1s, e non costituiscono test di completamento episodio/robustezza. Le
baseline e le sequenze inverse conservano l'orizzonte completo500. Il confronto
usa soltanto i prefissi temporalmente comuni. Le baseline iniziali sono state
trasferite a supervisione1800s senza interrompere i worker e mantenendo guardie
anti-stallo; ricevute operative nei rispettivi folder.

## Esiti discriminanti

- Reference liscia e precisa nel replay ma coppie/contatto scadenti nella
  dinamica: il solo problema di imitazione della reference non esaurisce il caso.
- Sequenza fisicamente migliore e favorita dalla reward: evidenza di una
  soluzione migliore disponibile, non prova dell'ottimo globale né dei gradienti PPO.
- Amplificazione degli impulsi: localizzare temporalmente policy, reference,
  giunti e contatti; distinguere risposta iniziale da divergenza degli eventi.
- Nessun criterio o soglia di promozione modificato dopo aver visto i risultati.
  Si riportano misure e limiti, senza certificare robustezza da un singolo start.
