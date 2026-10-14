# Audit offline del learner PPO — 2026-09-29

Confronto delle policy e dei critic sullo stesso batch on-policy, con advantages e target GAE congelati. Nessun training o rollout è eseguito da questa analisi.

Il batch contiene **4096 transizioni reali** e 14 righe mascherate. I frammenti originali sono 14: 2 terminati, 0 troncati e 12 ancora aperti. Episodi terminali osservati integralmente: 2.

## Riproduzione della policy prima dell'update

Sul batch intero, l'errore massimo fra logits archiviati e inferenza prima dell'update è 2.1457672e-06; con inferenza una riga per volta è 0. Lo scarto massimo del ratio da 1 è 0 con inferenza per riga. Il confronto tra le due modalità quantifica l'effetto numerico float32 della dimensione del batch; non equivale automaticamente a una differenza di policy.

La penalità KL del confronto usa il coefficiente realmente ripristinato (0.30000001), letto dallo snapshot prima dell'update. Il valore `kl_coeff` nel JSON di configurazione appartiene alla costruzione dell'algoritmo, prima del ripristino dello stato ausiliario.

## Obiettivo congelato

| Snapshot | KL media | Policy loss | VF MSE raw | VF loss capped | VF oltre cap | EV |
|---|---:|---:|---:|---:|---:|---:|
| before | 1.0611e-13 | -0.020205 | 33.965 | 2.1969 | 8.40% | 0.5591 |
| minibatch_000008 | 0.00619372 | -0.02309 | 33.432 | 2.1091 | 8.28% | 0.5683 |
| first_pass | 0.00600577 | -0.023661 | 33.433 | 2.1083 | 8.28% | 0.5696 |
| final | 0.011478 | -0.038008 | 28.961 | 1.8498 | 6.98% | 0.6218 |

La loss di policy passa da -0.020205 a -0.0380078; la MSE critic sullo stesso target passa da 33.9655 a 28.9611. Questi sono risultati di adattamento sul batch utilizzato per l'update, non validazione indipendente.

La VF loss locale satura ogni errore quadratico al valore di cap: sopra soglia il termine capped è localmente piatto. La sua media può quindi nascondere una coda di errori grandi. Quota e distribuzione sono misurate, ma non identificano da sole la causa delle coppie o della qualità imitativa.

## Confini, bootstrap e interpretazione

L'allineamento esatto tra osservazioni degli episodi e righe del batch è `True`. La ricostruzione indipendente TD/GAE usa bootstrap zero soltanto sulle vere terminazioni, conservando il valore della prossima osservazione ai limiti del frammento.
Nel CSV `independent_bootstrap_value_used` appartiene alla ricostruzione corretta; `stock_bootstrap_value_used` è il valore dopo l'azzeramento del helper stock (prima dell'eventuale correzione isolata). `bootstrap_value_predicted` è la predizione originale del critic alla riga artificiale.
**Discordanza verificata nei target:** errore massimo 56.3589177698346. Nel primo frammento aperto il target dell'ultima azione equivale alla sola reward, senza il valore della successiva osservazione. La composizione dei connector installati marca terminale la riga artificiale e il calcolo GAE annulla quel valore. Il riproduttore sintetico end-to-end, salvato in `bootstrap_connector_reproducer.json`, distingue frammento aperto, truncation e terminale vero.

Questo è un errore verificabile nella costruzione del segnale di apprendimento della versione locale. Non dimostra, da solo, che spieghi tutta la differenza LEFT/RIGHT o le curve di coppia. L'A/B sul medesimo batch deve distinguere il suo effetto dall'ottimizzazione e dalla risposta fisica.

La normalizzazione stock delle advantages comprende anche le righe artificiali, poi escluse dalla loss. Le metriche distinguono la sola variazione della normalizzazione dall'effetto combinato della correzione dei target; nessuna di queste modifiche è applicata dall'analizzatore.

`minibatch_000008` indica otto aggiornamenti. `first_pass` indica il primo aggiornamento che supera o raggiunge il numero di righe del batch: consultare `exact_epoch_boundary` nei metadati prima di chiamarlo una singola epoca esatta.

Sono disponibili 0 dataset fisici indipendenti con osservazioni complete del critic. Per ciascuno, tutti i critic sono valutati sulle stesse osservazioni. Ritorni a orizzonte finito e ritorni con bootstrap V_before sono riportati separatamente: senza vera terminazione nessuno dei due è una misura osservata del ritorno infinito.

## Decisioni supportate e limiti

- Un mismatch iniziale oltre l'errore numerico richiede verificare pesi, versione del sampler e semantica raw/logprob prima di interpretare l'update.
- Un miglioramento della surrogate sul batch seguito da peggioramento dei rollout sposta l'indagine verso ampiezza dell'update, distribuzione visitata e obiettivo, senza provare una causa unica.
- Il confronto critic prima/dopo sul target fisso distingue adattamento numerico da validazione; un critic con errore residuo non è automaticamente il responsabile del fallimento RL.
- Un solo batch e un solo seed non dimostrano convergenza, impossibilità di PPO, stabilità globale o superiorità statistica.

Dati: `metrics.json`, `snapshot_metrics.csv`, `episode_boundaries.csv`, `predictions.npz`, `minibatch_metrics.csv`. Figure nella cartella root `plot/09_29_2026_rl_learning_audit/learner/`. Provenienza e integrità in `input_hashes.json` e `verification.json`.
