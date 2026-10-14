# Telemetria degli episodi — preflight offline, 22 settembre 2026

## Attivazione e perimetro

Impostare `UNIFIED39_EPISODE_TELEMETRY_DIR` a una cartella **assoluta**, propagata anche agli EnvRunner Ray. Variabile assente/vuota: osservatore disattivato. Nessun parametro PPO, reward, azione, FSM o dinamica cambia. I soli file runtime modificati/aggiunti sono:

- `unified_MLP_39/episode_telemetry.py`: osservatore e serializzazione.
- `unified_MLP_39/tb_logging.py`: chiamate da `on_episode_step`/`on_episode_end` e metriche diagnostiche TensorBoard.

Non vengono invocati policy forward, estrazioni casuali o funzioni del simulatore. L'osservatore legge infos, azioni, osservazioni e output della policy già registrati da RLlib.

## Output

Un file `episodes_w{worker}_p{pid}_{timestamp_ns}_part{n}.jsonl` per writer/worker, creato al primo episodio concluso. Il timestamp identifica il writer senza usare RNG. Una riga JSON per **ogni esito**, inclusi gli episodi completi. Una ripartenza del processo ha un writer diverso; ogni `episode_uid` include writer, worker, indice ambiente e `SingleAgentEpisode.id_`.

Schema `unified39_episode_telemetry_v1`:

- esito, flag terminated/truncated, numero di step e ritorno dell'intero episodio;
- copertura: `observed_steps`, `missing_steps`, `rows_with_missing_fields`, `partial_episode`, `policy_distribution_rows`;
- tempo iniziale, finale e durata, quando la copertura include lo start;
- picchi di penetrazione/carico/penalità e minimo momento, con tempo e fase FSM;
- contatori di resync, eventi invalidi, clipping e penalità; ultimo resync/HS cancellato, anche fuori dalla coda;
- accumulatori degli errori dei giunti in rad² per calcolare il RMSE;
- `policy_versions`: prima/ultima/minima/massima versione dei pesi, numero di cambi e copertura sull'intero episodio; rimane disponibile quando il cambio è uscito dalla coda degli ultimi 50 step;
- `tail`: ultimi **50 step**, 0,5 s nominali a dt=0,01, con fase, joint/reference/target, azione grezza, actor observation precedente, contatto, termini reward e frazioni dei limiti governor;
- `policy_outputs`: `action_dist_inputs`, `action_logp`, `weights_seq_no` già presenti nell'episodio. Sono assenti se il chiamante non li ha registrati; non vengono ricalcolati.

L'azione grezza è il campione del modulo prima del clipping module-to-env; `policy_segment_values` documenta il comando realmente costruito. Il trace non inventa una seconda azione applicata a partire dal solo campione grezzo. Il passo riporta sia tempo precedente sia tempo corrente: le guardie fisiche possono interrompere l'avanzamento prima dell'endpoint.

## Memoria, frammenti ed errori

L'API locale RLlib 2.55.1 è verificata direttamente: `cut()` mantiene `id_` e `t` cumulativo. La coda risiede sul callback, non dentro `episode.custom_data` o nei batch del learner. Attraversa i frammenti senza ricopiare il lookback; duplicati con stesso timestep vengono ignorati.

Una coda per slot worker/env, massimo 128 slot. Il riepilogo usa contatori/accumulatori fissi. Le code vengono liberate anche in caso di errore di scrittura. Reset inattesi sostituiscono la vecchia coda e segnalano l'abbandono. La memoria degli ultimi episodi chiusi è anch'essa limitata a 128 slot.

Metriche diagnostiche `episode_telemetry/*`: `observed_steps`, `policy_distribution_rows`, `episodes_written`; in caso di problemi anche `missing_rows`, `step_errors`, `write_errors`, `episodes_dropped`, `abandoned_episodes`, `evicted_episodes`, `callback_errors`. I contatori di errore non emessi non provano da soli che il logging funzioni: controllare sempre copertura positiva e file prodotti. Eventuali problemi producono warning limitati per categoria, senza propagare l'errore al training. Dopo un errore di scrittura il writer cambia suffisso `part`, isolando un'eventuale ultima riga incompleta.

Un worker fatalmente interrotto può perdere l'episodio ancora in memoria: questa telemetria non è un journal continuo persistente. Gli episodi conclusi sono salvati e chiusi immediatamente. La diagnosi causale resta riferita a quello che è stato effettivamente osservato.

## Verifiche

Eseguire `test_episode_telemetry.py` con l'ambiente Python del training, senza avviare Ray. Risultato: **14 test superati**, zero errori/fallimenti; ricevuta in `test_receipt.json`.

Copertura: frammenti reali `SingleAgentEpisode` con lookback, duplicazione callback, sei cause di fine episodio, reset inatteso, limiti di memoria, multi-env/multi-worker, errore I/O con recupero, path relativo rifiutato senza interrompere il training, step mancanti, valori non finiti, ripresa parziale, logger diagnostico guasto e cambi di versione della policy attraverso i frammenti, inclusi cambi precedenti alla coda. Verificati invariati stato RNG Python/NumPy/Torch e l'intero stato dell'episodio, inclusi infos/output già presenti.

Benchmark sintetico 2.500 step, callback completo off/on: incremento CPU **47,23 µs/step**, circa **76 kB per episodio** con coda da 50 step. Il logger di questo microbenchmark è uno stub; la verifica seguente usa anche il logger RLlib reale.

`benchmark_real_metrics.py` riproduce i callback con **MetricsLogger reale**, 334 termini reward registrati e 147 componenti ricostruite dal rollout fisico: quattro prove alternate off/on, 500 step ciascuna. Incremento CPU medio misurato **57,78 µs/step**, comprensivo delle chiamate `log_value` e di una riduzione finale. Nessuna simulazione o inferenza aggiuntiva. Copertura 500/500 e nessun errore; dettagli in `real_metrics_benchmark.json`. Gait e parte privilegiata dell'osservazione sono ricostruzioni dell'API per il solo benchmark di costo, non una nuova prova fisica.

`verify_physical_ab.py` confronta due rollout fisici già eseguiti dall'orchestratore, stessa policy40, ambiente seed123 e rumore seed123 a scala1. **500 step, trace integrali e summary con hash identici**, campioni di rumore e deviazioni standard identici sui campi comuni. La differenza nel formato dei noise records è soltanto l'aggiunta dei logits nel braccio ON, verificati separatamente. Il sink registra tutti i 500 step e una coda finale di 50 coerente con il trace, senza dati mancanti o errori. Ricevuta `physical_ab_verification.json`: `passed=true`.

Costo cronometrato del sink fisico: **46,08 ms totali**, **92,17 µs/step**, pari allo **0,00849%** del tempo del braccio ON; record JSON di circa 194 kB. Il timer comprende observe/finish e scrittura JSON ma esclude l'adattatore SingleAgentEpisode creato apposta per la prova. Le durate complessive OFF/ON non vanno sottratte come misura di overhead: i processi condividevano risorse con altre diagnostiche. L'aggregazione delle versioni della policy è stata aggiunta dopo l'importazione del modulo nel processo fisico ON; è coperta dai successivi test d'immutabilità e dal benchmark con MetricsLogger reale.

Queste sono misure locali di costo ridotto, **non una garanzia di rallentamento nullo** su qualsiasi carico Ray/disco. La propagazione ai worker e l'integrazione nel callback di training richiedono ancora il controllo delle prime iterazioni reali dell'orchestratore.
