# Ripresa del controllo sperimentale

Il training ex novo del secondo blocco ha completato correttamente le cinque nuove iterazioni (7–11). La verifica esterna in `execute.py` confrontava erroneamente `iterations_run`, cumulativo, con il conteggio delle sole iterazioni aggiunte. Il summary riporta11cumulative e5in`iterations_completed_this_process`; nessuna iterazione saltata, exit code0del training.

Lo script originale e il suo hash sono conservati. `continue_execution.py` usa il campo corretto, riutilizza il blocco ex novo già completato e riprende dal blocco imitativo mancante. La configurazione ex novo già eseguita è stata verificata identica; nessuna modifica a codice di training, reward, maschere o parametri PPO. L'errore originale resta in`execution.log`; la prosecuzione è in`continuation.log` e ha un manifest separato. È stato inoltre reso coerente il nome della decisione nell'eventuale gate finale fallito, senza modificare le condizioni di avanzamento.
