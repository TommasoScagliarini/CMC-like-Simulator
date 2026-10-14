# Confronto degli audit di sorgente prima dei rollout

13 ottobre 2026. CMC_AGENT_OK_2026. Nessun esito dinamico nuovo letto.
Le conclusioni Codex sono sigillate in `codex_source_seal.json` prima della
lettura di `claude/source_response.md`. Claude ha usato `claude-opus-5-5`,
effort `max`, senza fallback, strumenti Read/Grep/Glob e nessuna esecuzione.

I due audit concordano sulla necessità di inizializzare coerentemente giunti,
rotori, reference, governor ed endpoint. Il solo cambio delle posizioni fisiche
non realizzerebbe il test: la prima anchor potrebbe riportare la IK protesica.
Concordano inoltre su controllo nominale coerente, censura dell'ultimo segmento
non restituito, inclusione dei failure fisici e limiti di N20.

Claude ha letto una versione intermedia del harness mentre la review Codex
continuava. Le sue osservazioni su count8, assenza di selezione automatica,
interruzione del batch e assenza di trace incrementale descrivono tale versione;
il codice finale viene verificato separatamente e congelato prima del preflight.
La forma dell'azione nel probe è invece un difetto concreto da correggere e
verificare nativamente. Anche la separazione fra postprocessing e risultato
fisico viene corretta; il timeout di calcolo è censura tecnica.

Si mantengono i criteri preregistrati di cinque secondi e almeno due cicli
qualificati: i rollout nominali precedenti hanno misurato due cicli in questa
finestra, quindi il periodo GRF di 1,669 s non dimostra che il criterio sia
impossibile. Il nuovo nominale rimane necessario; completion, sequenza FSM e
qualificazione ciclica vengono comunque riportate separatamente. Non si passa
a sette secondi o a soglie relative dopo aver visto esiti.

Il criterio actual-served misura tracking del controller. Le escursioni sui
cicli completi e gli eventi aggiungono una verifica di gait nel banco, senza
provare convergenza all'unico ciclo nominale o equilibrio autonomo. Un replay
open-loop delle azioni nominali sarebbe un controllo utile per attribuire il
recupero al feedback neurale; non è necessario per misurare il comportamento
del sistema con la policy congelata e non viene dichiarato eseguito.

La biologia è dinamicamente integrata e inseguita da muscoli/reserve: non è
cinematica prescritta. L'affermazione di Claude «il bacino non può cadere» è
troppo forte. Il supporto biologico resta un confondente esplicito, e le guardie
di altezza/divergenza continuano a valere.

Non si accetta la regola generale «qualsiasi eccezione dopo uno step è fisica»:
per esempio un errore del wrapper o un timeout rimangono tecnici. I failure di
guardia fisica conosciuti sono distinti; le eccezioni ambigue restano visibili
nel denominatore selezionato e nei bound di incertezza.

Il gate iniziale di 27 mm e 5 BW definisce un insieme ammesso dal modello,
senza certificare realismo umano, equilibrio statico o self collision. Lo
stato AIR del detector e il contatto primario sono riportati separatamente.
