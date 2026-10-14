# LEFT: esiti dei pilot reward e diagnosi del coefficiente KL

Data del report: **2026-09-29**. Attività sperimentali rendicontate: 25–28 settembre 2026.
Instruction check token: `CMC_AGENT_OK_2026`.

## Esito e stato corrente

**Best80 LEFT, peso penetrazione 2, resta il riferimento.** Ankle 10% riduce l'area positiva nominale ma viola i vincoli fisici e fallisce una prova rumorosa. Ankle 5% migliora il tracking nominale medio, ma non riduce l'area rispetto a best80 e perde un ciclo FSM valido. Nessun checkpoint intermedio esaminato supera lo screening.

Sono conclusi entrambi i pilot, l'analisi delle fasi e del learner e le sei valutazioni intermedie. L'ultima esecuzione del 28 settembre ha svolto esclusivamente valutazioni dei checkpoint esistenti: **nessun nuovo aggiornamento PPO**. Il presente aggiornamento del 29 settembre è documentale. La conservazione del coefficiente KL durante il critic-only è una prossima ipotesi sperimentale, ancora da implementare e preparare.

## Problema, soluzione e strategia

Il problema iniziale era modificare la reward senza attendere ore per ogni tentativo. Il banco sotto `Trajectory Generator/unified_MLP_39/` separa il ricalcolo offline sui rollout salvati dalla misura dell'effetto appreso tramite brevi continuazioni riproducibili.

Il corpus comprende 17 rollout; V26 e RIGHT restano descrittivi perché usano target diversi. Le nove intensità offline modificano una famiglia alla volta e destinano il 5%, 10% o 20% dello score imitativo positivo del best80 al nuovo costo. Questa calibrazione quantifica un effetto numerico sui trace, non prevede il miglioramento della policy.

Dopo i vincoli di tracking e contatto, le priorità restano area positiva della coppia SEA ankle, regolarità della coppia knee e regolarità cinematica. I termini sono opt-in, con pesi predefiniti zero e sottrazione dopo il clipping della reward esistente, soltanto in imitazione. Scale esplicite: 30 Nm, 1000 Nm/s e 1000 rad/s³; non sono soglie fisiologiche. La reference servita rimane diagnostica per la cinematica.

I pilot ripartono dallo stesso checkpoint completo best80: un aggiornamento critic-only e tre PPO, batch 4096, episodi di 5 s, seed training 123, simulatore invariato e prescribed/clock visibili. Valutazioni nominali e rumorose 123/125/127 usano gli stessi campioni gaussiani e sigma congelata di best80. Ciò non implica traiettorie di training identiche nei worker.

I vincoli confrontano ciascuna candidata sia con best80 sia con il controllo: tracking actual/served globale e per fase entro incremento `min(10%, 0,5°)`, escursione e impulso almeno al 90%, completamento e cicli non ridotti, penetrazione nominale sotto 20 mm e incremento massimo 2 mm negli scenari comparabili, senza nuove terminazioni hard. Il beneficio ankle richiede almeno −10% nominale, nessun aumento mediano rumoroso e nessun peggioramento di regolarità oltre il 10%. I ritorni di obiettivi diversi non vengono usati per ordinare le policy.

## Esecuzioni e costi

| Esecuzione | Data effettiva | Lavoro | Tempo cumulativo |
| --- | --- | --- | ---: |
| Pilot ankle 10% | 25 settembre | Controllo e candidata; 12 valutazioni | 105,16 min |
| Pilot ankle 5% | 26 settembre | Una candidata; 4 valutazioni nuove, riferimenti riutilizzati | 39,79 min |
| Checkpoint intermedi | 28 settembre | Sei nominali delle iterazioni 82–83, senza training | 17,39 min |

Ogni esecuzione è rimasta entro il proprio budget di 120 minuti. Il percorso ankle 5% conserva la data di preparazione del 25 settembre, distinta dalla data effettiva di lancio. Il riuso verificato del controllo e degli otto rollout di riferimento evita duplicazioni ma non aggiunge repliche indipendenti.

## Risultati dei pilot

Metriche nominali sull'intero episodio 12,99–17,99 s; le tabelle della finestra storica 13,15–17,95 s rimangono nei report di dettaglio.

| Policy finale | Area ankle+ [Nm·s] | RMSE knee [°] | RMSE ankle [°] | Cicli FSM validi | Penetrazione [mm] | Impulso [N·s] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Best80 | 8,650 | 6,344 | 3,997 | 2 | 14,795 | 389,545 |
| Controllo, iter. 84 | 25,611 | 6,208 | 6,009 | 2 | 19,045 | 465,945 |
| Ankle 5%, iter. 84 | 9,029 | 5,591 | 3,293 | 1 | 16,783 | 468,214 |
| Ankle 10%, iter. 84 | 3,036 | 7,347 | 4,021 | 2 | 14,795 | 345,357 |

**Ankle 10%**, peso 1,3379186063656872: area nominale −64,9% rispetto a best80, ma tracking knee e impulso non rispettano i vincoli. Il seed rumoroso 127 termina dopo 0,97 s con `invalid_binary_event`; l'area del rollout incompleto non è un miglioramento confrontabile. Esito: `rejected_constraints`.

**Ankle 5%**, peso 0,6689593031828436: completa tutti e quattro gli scenari, ma l'area nominale cresce del 4,4% rispetto a best80. La mediana delle differenze appaiate di area nei tre scenari rumorosi è +1,931 Nm·s rispetto a best80 e +3,361 Nm·s rispetto al controllo. Falliscono inoltre vincoli per fase e sui cicli. Esito: `rejected_constraints`.

La forte deriva del controllo impedisce di attribuire tutti i cambiamenti alla sola nuova reward.

## Checkpoint intermedi: nessuna soluzione recuperabile nello screening

Area positiva sull'intero episodio, in Nm·s:

| Ramo | Iter. 82: un PPO | Iter. 83: due PPO | Iter. 84: tre PPO |
| --- | ---: | ---: | ---: |
| Controllo | 5,537 | 14,992 | 25,611 |
| Ankle 5% | 15,860 | 17,203 | 9,029 |
| Ankle 10% | 12,285 | 10,332 | 3,036 |

Ogni candidata è confrontata con best80 e con il controllo alla stessa iterazione. Il controllo 82 riduce l'area, ma l'impulso 343,247 N·s è inferiore al minimo 350,590 N·s e falliscono anche tracking per fase e regolarità. Gli intermedi ankle 5% peggiorano area e tracking ankle. Ankle 10% a83 mantiene area +19,45% rispetto a best80, RMSE ankle 4,460° oltre il limite 4,397° e un ciclo valido anziché due.

Tutti e sei i nuovi rollout completano 500 step/5 s e producono sette plot canonici ciascuno: 42 figure sotto root `plot/`. Lo screening è completo, senza casi mancanti e senza candidate ammissibili. Per protocollo non sono state eseguite ulteriori prove rumorose. Non emerge una soluzione valida fermando prima questi training; ciò non dimostra che altre durate siano inutili.

## Diagnosi fisica e del learner

**Coefficiente KL.** Il critic-only mantiene actor, logstd e relativi stati Adam, ma l'adattamento automatico RLlib dimezza il coefficiente KL da 0,30 a 0,15 durante l'iterazione 81, in risposta alla KL quasi nulla dell'actor congelato. Il primo PPO parte quindi da 0,15 in tutti e tre i rami. Il comportamento è verificato nei checkpoint e nel codice RLlib installato; non è una causa dimostrata delle regressioni e non è stato corretto.

L'esperimento successivo isolabile sarebbe conservare 0,30 durante critic-only e riattivare l'adattamento con il PPO dell'actor, mantenendo invariati gli altri parametri. Occorre un controllo con lo stesso trattamento: il controllo storico con coefficiente 0,15 avrebbe soltanto funzione descrittiva.

**Critic.** Le explained variance dopo il rewarm sono 0,671/0,663/0,658 per controllo/ankle 5%/ankle 10%. Il loss VF limitato a 10 nasconde errori grezzi più grandi; le metriche registrate non costituiscono una validazione indipendente. Non è dimostrata la convergenza dopo un solo aggiornamento.

**Cicli ankle 5%.** Il conteggio inferiore deriva dalla qualificazione FSM degli eventi: un HS troppo precoce dopo il TO a 14,538 s porta al resync e allo scarto del ciclo degradato successivo. Best80 conta anche un ciclo molto breve di 0,769 s prima di un proprio resync. Il vincolo 2 contro 1 resta valido nel pilot, ma non equivale a dimostrare un'oscillazione fisica persa.

**Distribuzione del beneficio ankle 10%.** L'82,4% della riduzione totale di area proviene dall'ultima finestra prescribed parziale 17,17–17,99 s: area 5,756 → 1,131 Nm·s e impulso 120,013 → 65,963 N·s. Nei due cicli prescribed completi precedenti l'impulso invece aumenta. La riduzione non è uniforme e il segno della coppia non è usato come equivalente del segno della forza verticale.

## File interessati

L'implementazione iniziale del banco è documentata nel [report del 25 settembre](2026-09-25_banco_tuning_rapido_reward_left.md). Le successive estensioni riguardano, sotto `Trajectory Generator/unified_MLP_39/`:

- `reward_tuning/pilot.py`, `compare.py`, `__main__.py`, `references.py`, `report.py`: gestione della candidata esplicita, riuso verificato dei riferimenti e report del pilot.
- `configs/reward_tuning_left_ankle5.yaml`: configurazione del ramo ankle 5%.
- `validation/evaluate_pilot_intermediates_2026_09_28.py`: preparazione, budget e ripresa delle sole valutazioni.
- `validation/compare_pilot_intermediates_2026_09_28.py`: screening con controllo alla stessa iterazione e output CSV/JSON/figure.
- `validation/diagnose_reward_phases_2026_09_28.py`: analisi di fasi, campioni SEA, contatto ed eventi.
- Test del riuso e del report in `tests/test_tuning_references.py`, `tests/test_tuning_report.py`; test diagnostici in `tests/test_pilot_intermediate_diagnosis.py`.

Le aggiunte iniziali a `simulation_runner.py` sono diagnostiche dei campioni SEA; controller e plugin non cambiano. Nessun fix KL o modifica FSM viene introdotto dalla diagnosi. In questa richiesta vengono scritti soltanto il presente report e il daily parziale del 29 settembre; gli artefatti sperimentali congelati restano intatti.

## Test e verifiche già eseguiti

- Replay dei 17 ritorni entro 4,55e-13; parità della reward storica con pesi nuovi zero. Smoke OpenSim di 2×5 segmenti senza aggiornamenti PPO, con stati fisici identici e parità della reward entro 1e-12.
- Suite 39D al preflight ankle 5%: 208 test superati, con un warning grafico `tight_layout` della fixture vuota. Otto test mirati della diagnosi intermedi superati.
- Audit del learner: 12 checkpoint e 403 hash; actor/logstd/Adam conservati durante rewarm, critic aggiornato, prescribed visibili su 4096/4096 transizioni e reward corretta nei 13 worker.
- Verifica di rumore e sigma congelati nelle valutazioni; fallimenti e dati mancanti non promuovono una candidata.
- Manifest intermedi: 631 input congelati, sei nominali completi e 42 plot verificati con SHA-256. Analisi per fase: 2000 segmenti e 20000 campioni SEA, riconciliazione delle somme positive entro 2,79e-8 Nm.
- Filtri applicati prima del ritaglio. La diagnosi per fase usa intervalli semiaperti; lo screening storico include il campione superiore a 17,95 s. Gli integrali alternativi sono espliciti, senza interpretare un campione di differenza come effetto fisico.

Questi sono risultati delle esecuzioni precedenti, non test rilanciati per scrivere il report. La verifica live Windows resta aperta. Un seed di training e tre prove rumorose sono uno screening operativo, non un'attestazione di robustezza statistica. Oscuramento, training ex-novo da 100 iterazioni e tuning simultaneo delle famiglie non sono stati avviati.

## Artefatti di riferimento

- [Pilot ankle 10%: confronto](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/comparison.md>)
- [Pilot ankle 5%: report completo](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_ankle5_2026-09-25/report.md>)
- [Diagnosi e valutazioni intermedie concluse](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/report.md>)
- [Diagnosi del learner e KL](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/training_analysis/report.md>)
- [Fasi, eventi e contatto](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/phase_analysis/report.md>)
- [Screening nominale degli intermedi](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/analysis/report.md>)
- [Verifica finale dei rollout e plot](<../../Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/verification.json>)

## TODO da propagare

- [ ] Isolare in un nuovo esperimento la conservazione del coefficiente KL durante il critic-only, con controllo sottoposto allo stesso trattamento; definire configurazioni e budget prima dell'esecuzione. Il fix non è implementato né il training predisposto.
- [ ] Verificare la sufficienza del riadattamento del critic con diagnostica degli errori e validazione indipendente; un aggiornamento non dimostra convergenza.
- [ ] Sperimentare separatamente regolarità della coppia knee e cinematica, mantenendo tracking e contatto come vincoli e senza modificare contemporaneamente le famiglie.
- [ ] Eseguire la verifica live Windows della pipeline e, prima di conclusioni sulla robustezza, repliche di training e condizioni di valutazione più ampie.

