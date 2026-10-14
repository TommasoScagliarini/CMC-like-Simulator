# A/B sullo stesso batch: target stock e correzione boundary

Il controllo riproduce esattamente l'update originale: pesi actor/critic, Adam, scalari e input/ordine/forward di tutti gli 81 minibatch. Il ramo corretto riparte dallo stesso stato e dagli stessi episodi, senza raccogliere nuovi dati. Cambiano soltanto i target TD-lambda reali e le advantages che ne derivano; target artificiali e convenzione di normalizzazione sono preservati.

La correzione cambia il segno delle advantages normalizzate nel 23.999% delle 4096 transizioni. Non significa che ogni target corrispondente sia gravemente errato: la centratura e la scala cambiano anche sulle righe lontane dal confine. La quota dei target stock distanti oltre 1 dal target corretto è 9,5215% nel batch originale.

## Griglia a target fisso

Ogni coppia target/advantages è congelata. Confrontare **before, control e corrected dentro la medesima coppia**; le loss di righe appartenenti a target diversi non misurano la stessa funzione obiettivo.

| Target | Policy/critic | VF MSE raw | VF capped | EV | Policy loss | KL |
|---|---|---:|---:|---:|---:|---:|
| stock | before | 33.96547 | 2.19694 | 0.5591 | -0.020205 | 0.000000 |
| stock | control | 28.96106 | 1.84984 | 0.6218 | -0.038008 | 0.011478 |
| stock | corrected | 33.18335 | 2.02038 | 0.5734 | -0.027933 | 0.009844 |
| corrected_real_boundary | before | 7.36790 | 1.70023 | 0.7632 | -0.036341 | 0.000000 |
| corrected_real_boundary | control | 6.83727 | 1.76125 | 0.7799 | -0.043537 | 0.011478 |
| corrected_real_boundary | corrected | 7.08265 | 1.42731 | 0.7736 | -0.048638 | 0.009844 |

La tabella verifica adattamento al segnale di training e mostra se l'update stock o corretto ottimizza meglio ciascun segnale. Non misura ancora la qualità delle traiettorie né attribuisce tutta la difficoltà LEFT a questo errore.

## Critic su traiettorie separate dal batch

Casi con osservazioni complete disponibili: 12. Casi ancora in attesa di receipt: 0. Casi completati senza osservazioni complete del critic: 1.

Tutti e tre i critic sono valutati sulle stesse osservazioni all'interno di ciascun rollout. Solo una vera terminazione rende il ritorno osservato fino alla fine un campione Monte Carlo dell'episodio della policy generatrice. Per truncation o fine della registrazione sono distinti il ritorno finito senza coda e il ritorno con bootstrap del critic before: sono convenzioni esplicite, non verità indipendenti sul valore continuativo. Il target TD a un passo usa anch'esso V_before fisso, mentre la residual TD con il valore di ciascun critic è soltanto diagnostica.

**Limite fuori-policy:** azioni, reward e ritorni appartengono al comportamento che ha generato il rollout (before, control, corrected o first_pass). Il nominale usa azioni deterministiche: anche quando il checkpoint coincide, non rappresenta il rendimento atteso della policy stocastica appresa dal critic. I campi `critic_checkpoint_matches_generator_checkpoint` e `own_stochastic_behavior` distinguono le due condizioni; il secondo vale soltanto per un rollout noisy del medesimo checkpoint. I confronti degli altri critic sullo stesso esito sono fuori-policy: neppure un ritorno terminale è una verità comune per tre policy diverse. Questi punteggi descrivono gli scarti sul medesimo dato, senza certificare l'accuratezza di ogni V rispetto al rendimento atteso della propria policy o un ranking assoluto dei critic. Le traiettorie sono separate dal batch di training; le transizioni successive sono correlate e il loro numero non equivale al numero di repliche statisticamente indipendenti.

| Rollout | Termine | Critic | RMSE su ritorno osservato | RMSE con bootstrap before |
|---|---|---|---:|---:|
| before_noisy_123 | orizzonte finito | before | 13.0150 | 5.3739 |
| before_noisy_123 | orizzonte finito | control | 12.4415 | 5.4601 |
| before_noisy_123 | orizzonte finito | corrected | 13.0506 | 5.1950 |
| before_noisy_125 | orizzonte finito | before | 21.2899 | 10.2161 |
| before_noisy_125 | orizzonte finito | control | 20.7424 | 9.8262 |
| before_noisy_125 | orizzonte finito | corrected | 21.4845 | 10.3784 |
| before_noisy_127 | terminale / MC | before | 16.3068 | 16.3068 |
| before_noisy_127 | terminale / MC | control | 15.9687 | 15.9687 |
| before_noisy_127 | terminale / MC | corrected | 16.3774 | 16.3774 |
| after_nominal | orizzonte finito | before | 19.3209 | 7.5254 |
| after_nominal | orizzonte finito | control | 18.7931 | 7.2452 |
| after_nominal | orizzonte finito | corrected | 19.5485 | 7.7310 |
| after_noisy_123 | orizzonte finito | before | 22.9733 | 12.5114 |
| after_noisy_123 | orizzonte finito | control | 22.4349 | 12.1319 |
| after_noisy_123 | orizzonte finito | corrected | 23.0565 | 12.5872 |
| after_noisy_125 | orizzonte finito | before | 19.1107 | 6.9201 |
| after_noisy_125 | orizzonte finito | control | 18.6155 | 6.5962 |
| after_noisy_125 | orizzonte finito | corrected | 19.3363 | 7.1285 |
| after_noisy_127 | orizzonte finito | before | 13.1004 | 6.5535 |
| after_noisy_127 | orizzonte finito | control | 12.7636 | 6.8957 |
| after_noisy_127 | orizzonte finito | corrected | 13.0505 | 6.3290 |
| first_pass_nominal | orizzonte finito | before | 17.0851 | 6.2565 |
| first_pass_nominal | orizzonte finito | control | 16.5883 | 6.1517 |
| first_pass_nominal | orizzonte finito | corrected | 17.2395 | 6.2777 |
| corrected_nominal | orizzonte finito | before | 15.9946 | 5.6369 |
| corrected_nominal | orizzonte finito | control | 15.5481 | 5.6673 |
| corrected_nominal | orizzonte finito | corrected | 16.1761 | 5.6532 |
| corrected_noisy_123 | orizzonte finito | before | 13.1532 | 7.1139 |
| corrected_noisy_123 | orizzonte finito | control | 12.6631 | 7.1999 |
| corrected_noisy_123 | orizzonte finito | corrected | 13.1748 | 6.9512 |
| corrected_noisy_125 | orizzonte finito | before | 14.7892 | 5.8424 |
| corrected_noisy_125 | orizzonte finito | control | 14.4194 | 6.1190 |
| corrected_noisy_125 | orizzonte finito | corrected | 14.9375 | 5.7349 |
| corrected_noisy_127 | orizzonte finito | before | 15.8805 | 7.4416 |
| corrected_noisy_127 | orizzonte finito | control | 15.4817 | 7.5189 |
| corrected_noisy_127 | orizzonte finito | corrected | 15.9360 | 7.3351 |

Osservazioni critic assenti: before_nominal.

Sono confronti su un batch e pochi rollout, non una dimostrazione di convergenza, stabilità globale o sufficienza della correzione. Nessun codice di produzione o ambiente Python globale è stato modificato; questa correzione isolata non è un backport completo della modifica upstream.

Provenienza: `input_hashes.json`; verifiche: `verification.json`; dati: `metrics.json`, `cross_target_metrics.csv`, `heldout_critic_metrics.csv`. Figura nella cartella root dei plot del learner.
