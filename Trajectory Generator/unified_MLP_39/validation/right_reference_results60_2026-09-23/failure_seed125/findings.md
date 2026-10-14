# Seed125: failure residuo della RIGHT58

Audit offline di quattro rollout già esistenti, senza nuove simulazioni. Script riproducibile `analyze.py`, dati e hash in `metrics.json` e `input_hashes.json`. Confronto indipendente dello sviluppo temporale, complementare al confronto puntuale in `noise_screening/seed125_failure_comparison.json`.

**58 e 40 falliscono nello stesso tratto della seconda stance; 35 e V26 superano quel tratto.** Alla 58 la reference ankle già si allontana verso l'alto dal prescribed e l'articolazione la insegue sempre più velocemente; il fallimento non è preceduto da invalidazioni o resync della FSM. Questa è una descrizione osservabile della traiettoria verso la penetrazione, non l'isolamento di una sola causa meccanica.

## Allineamento e limite della riga hard terminale

I target dei primi299 step sono identici nelle quattro condizioni; i primi300 draw Gaussiani standard sono identici (assert verificati). Le deviazioni standard apprese e gli stati visitati differiscono, quindi le perturbazioni assolute non sono identiche.

La 58 termina alla chiamata300 per `grf_penetration_hard`, ma la riga300 ripete il tempo della299 (15,98s assoluti, 2,99s dall'inizio). Il suo stato articolare può essere avanzato parzialmente mentre il tempo logico e il riferimento sono rimasti quelli precedenti; il suo diagnostico penetrazione è zero. **Questa riga è esclusa dal confronto degli stati.** Il punto comune è il segmento299 regolarmente concluso e la finestra delle statistiche è2,70–2,99s.

Il massimo della trace100Hz prima del fallimento58 è27,933mm, ma il file fisico1kHz contiene ancora avanzamenti parziali fino15,988s e raggiunge29,9465mm prima della guardia interna30mm. Non è contraddittorio che il massimo registrato resti inferiore alla soglia che interrompe l'integrazione. Il massimo STO e quello diagnostico non sono lo stesso campionamento: per esempio includendo il campioneSTO a15,980s il picco comune è28,170mm, mentre il diagnostico del segmento299 è27,933mm. Le tabelle seguenti usano la trace100Hz, le durate sopra soglia usano loSTO1kHz.

## Sviluppo durante gli ultimi0,19s comuni

| Tempo dall'inizio | Quantità | RIGHT35 | RIGHT40 | RIGHT58 | V26 |
|---|---|---:|---:|---:|---:|
| 2,80s | Penetrazione[mm] | 11,63 | 10,26 | 10,56 | 12,61 |
| 2,90s | Penetrazione[mm] | 10,63 | 9,83 | 10,44 | 14,34 |
| 2,95s | Penetrazione[mm] | 13,16 | 15,37 | 17,69 | 18,15 |
| 2,99s | Penetrazione[mm] | 15,75 | 24,91 | 27,93 | 22,93 |
| 2,90s | Reference ankle servita[°] | 7,79 | 10,29 | 12,86 | 13,48 |
| 2,95s | Reference ankle servita[°] | 9,56 | 14,42 | 17,10 | 16,15 |
| 2,99s | Reference ankle servita[°] | 13,60 | 18,74 | 21,92 | 18,60 |
| 2,99s | Ankle actual[°] | 9,30 | 14,58 | 17,64 | 17,07 |
| 2,99s | Velocità ankle actual[°/s] | 68,80 | 115,00 | 145,95 | 60,87 |

Il prescribed ankle è comune:12,91° a2,90s,14,10° a2,95s e15,11° a2,99s. Quindi, tra2,90 e2,99s la reference58 aumenta di9,06° mentre il prescribed aumenta di2,20°; il V26 aumenta di5,12°. Il distacco non nasce esclusivamente tra reference servita e articolazione: è già presente nella reference generata, e il tracking aggiunge un proprio scostamento durante l'accelerazione.

Alla2,99s: errore served−prescribed ankle58 **+6,80°**, actual−served **−4,28°**; V26 rispettivamente **+3,48° e−1,52°**. Nell'ultima finestra2,90–2,99s, gli RMSE servito-prescribed / actual-servito sono58 **3,78°/3,30°** eV26 **2,22°/1,42°**. L'RMSE actual-prescribed58 è però soltanto1,06° in questa breve finestra: i due errori di segno opposto si compensano parzialmente. Un basso RMSE di posizione locale non assicura sicurezza del contatto o controllo della velocità.

Anche il ginocchio differisce: al punto comune58 actual−10,80°, served−10,06°; V26actual−13,27°, served−12,64°, contro target−12,03°. Non si può attribuire la penetrazione al solo ankle senza separare gli effetti di ginocchio, dinamica, geometria del piede e carico.

| Contatto fisico da2,70s | RIGHT35 | RIGHT40 | RIGHT58 | V26 |
|---|---:|---:|---:|---:|
| Prima attraversata20mm, tempo dall'inizio[s] | 3,011 | 2,969 | 2,959 | 2,964 |
| Tempo sopra20mm nella finestra comune2,80–2,99s[ms] | 0 | 22 | 32 | 27 |
| Forza normale massima nella finestra2,70–2,99s[N] | 697,1 | 878,8 | 980,0 | 693,9 |
| Massimo penetrazione salvato nell'intero rollout[mm] | 22,42 | 28,82 | 29,95 | 25,11 |

Il V26 supera anch'esso20mm, ma la penetrazione raggiunge il massimo25,11mm e recupera; la58 cresce più rapidamente fino alla guardia. Non basta quindi contare il tempo sopra20mm per distinguere successo e fallimento. I numeri di tempo sopra soglia sono occupazione dei campioni1ms, non una stima continua sub-millisecondo.

## FSM, azioni e rumore

Tutte e quattro sono in `STANCE_AFTER_HS` al punto comune. L'HS precedente è a2,595s per58 e2,605s per40/V26 (35:2,603s), quindi non vi è un ampio sfasamento di quel contatto tra condizioni. Prima del fallimento58 e40 hanno1ciclo valido,0eventi invalidi e0resync;35 ha1ciclo,2invalidi e0resync;V26 ha0cicli,1invalido e1resync, pur completando poi l'episodio. Questo caso mostra che il fallimento fisico residuo può verificarsi anche con FSM formalmente regolare.

Le sigma ankle medie degli ultimi10segmenti completati sono35:0,935,40:0,994,58:0,911,V26:0,961. Una sigma media maggiore della58 non spiega il caso. Le azioni già tagliate non bastano a ricostruire la dinamica: l'ultima azione ankle del segmento299 è tagliata a−1 sia58 siaV26, ma stato e memoria del filtro/riferimento sono differenti. La300 avrebbe lo stesso draw standard ankle+1,2627 in tutte le condizioni; la58 interrompe quel segmento per la guardia, quindi non si confronta il suo esito parziale con il normale segmento300 delle altre policy.

## Interpretazione

Il caso125 evidenzia una vulnerabilità residua specifica della traiettoria rumorosa: rapidità dell'aumento della reference servita, scostamento di inseguimento e crescita del carico/penetrazione compaiono insieme, mentre il target prescribed e i draw standard sono uguali. Non dimostra un bug della FSM, un target errato o che basti ridurre la sigma. La35 sopravvive pur imitando peggio l'ankle in questa finestra; quindi ottimizzare il solo errore cinematico non ordina automaticamente le policy per sicurezza fisica.

Questo singolo seed non prova una regressione generale58vs40 e non autorizza modifiche di reward. Il passo interpretativo corretto è integrare il caso con lo screening completo e usare le trace esistenti per verificare quando divergenza della reference, velocità e contatto iniziano a differire, prima di scegliere un esperimento causale.
