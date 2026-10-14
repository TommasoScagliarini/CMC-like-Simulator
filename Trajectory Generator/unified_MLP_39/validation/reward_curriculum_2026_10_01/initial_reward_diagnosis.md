# Diagnosi iniziale prima degli aggiornamenti dell'actor

Le quattro valutazioni usano i pesi delle medie di CP90, dopo trasferimento e warm-up del solo critic. Nei confronti visibile/visibile e hidden/hidden fra obiettivi, azioni e osservazioni coincidono sull'intero episodio entro1e-7. Tutti i rollout completano500passi.

| Misura | CP90 visibile nel runtime comune | CP90 hidden nel runtime comune |
|---|---:|---:|
| ROM fisica knee/ankle, finestra13.19–17.94s | 41.71°/30.67° | 12.02°/10.26° |
| Cicli validi | 2 | 1 |
| Eventi invalidi | 0 | 3 |
| Return ex novo | -680.32 | -342.42 |
| Reward base ex novo, totale | 9.38 | 9.99 |
| Penalità morphology, totale | 682.26 | 133.87 |
| Penalità temporale, totale | 5.20 | 169.77 |
| Step con morphology disponibile | 484/500 | 258/500 |
| Campioni risolti morphology | 488/500 | 258/500 |
| Campioni scartati morphology | 12/500 | 242/500 |
| Knee fuori corridoio, frazione degli step disponibili | 48.14% | 42.64% |
| Ankle fuori corridoio, frazione degli step disponibili | 37.40% | 41.09% |

I numeri morphology di questa tabella riguardano **served_ref** e la fase causale della reward, non la q fisica al medesimo tempo di consegna. I totali della penalità comprendono tutti i campioni risolti, mentre il grafico del corridoio visualizza l'ultimo campione per step disponibile:488campioni e484step sono denominatori diversi.

Il valore assoluto dei return non è comparabile tra reward diverse. Il confronto visibile/hidden *dentro* la reward ex novo mostra che quest'ultima preferisce inizialmente il rollout cinematicamente compresso, nel complesso dei suoi termini. La differenza non si spiega soltanto con il numero di campioni valutati: cambiano anche fase, violazioni del corridoio, eventi e timeout. Non è una prova che il training convergerà verso la quiete; è una ragione per non scegliere i checkpoint soltanto con il return.

La figura morphology visibile mostra soprattutto disallineamento temporale: alla fase in cui il corridoio richiede plantarflessione, la reference ankle è ancora positiva; il picco knee arriva dopo quello del profilo. I triangoli TO del plot sono annotazioni al tempo di consegna e non vanno interpretati come una misura precisa del ritardo fra fase e evento. Il corridoio viene calcolato con journal/eventi e40ms di ritardo, includendo una stima delle durate durante il bootstrap.

Questi sono dati iniziali, non risultati del training. Il preset J20 resta invariato nel ramo ex novo e le soglie di progressione cinematiche restano quelle preregistrate. Il costo morphology elevato segnala inoltre una differenza di scala dei target di valore: nel report finale vanno riportati loss VF clippata/non clippata ed explained variance, senza dare per dimostrata la calibrazione del critic dopo una sola iterazione.

## Differenze nei riferimenti e limiti di ottimizzazione

Il target imitativo (`PhaseBasedImitationTarget`) interpola il segnale cinematico filtrato, seleziona periodi entro il25% dalla mediana e media cicli uniformemente normalizzati HS→HS. Il corridoio ex novo è costruito invece su123cicli completi HS→TO→HS con allineamento separato di appoggio e oscillazione, da interpolazione lineare della IK originale. Nel profilo l'alpha medio di TO è0.62233. Queste costruzioni non definiscono necessariamente lo stesso riferimento, neppure con una fase perfetta. Sui500campioni del rollout visibile, l'ankle prescribed target varia da−6.61° a+22.05°; la media del profilo morphology da−17.90° a+14.81°. Sono estremi di segnali diversi (target campionato e media del corridoio), non una misura diretta dell'errore né una dimostrazione di incompatibilità con la banda di tolleranza.

Nella versione locale di RLlib (`ray/rllib/algorithms/ppo/torch/ppo_torch_learner.py`), la VF loss è `clamp((V-target)**2,0,vf_clip_param)`. Con `vf_clip_param=10`, la derivata del termine VF è nulla sui campioni con errore assoluto maggiore di√10≈3.16. I primi aggiornamenti mostrano loss non clippata molto superiore alla clippata ed explained variance prossima a zero. Non disponiamo della frazione esatta di campioni saturati: il meccanismo è verificato nel codice, la sua rilevanza quantitativa resta da misurare. Il warm-up resta quello preregistrato per entrambi i rami; non è corretto interpretarne la sola esecuzione come prova di critic calibrato.
