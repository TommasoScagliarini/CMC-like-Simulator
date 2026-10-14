# Diagnostica offline governor — 2026-09-29

La reference LEFT e seguibile con precisione dal governor attuale mediante
endpoint anticipati regolari, senza cambiare bounds, cutoff, jerk, accelerazione,
velocita o reset. Questo risultato riguarda il riferimento servito: non dimostra
che contatto/GRF/SEA possano realizzarlo fisicamente, ne che una policy causale
abbia gia imparato a produrre questi endpoint.

Nessun modello OpenSim e stato costruito e nessun rollout fisico o training e
stato eseguito da questi script. Tutti i file prodotti sono in questa cartella.

## Metodo e verifica

`analyze_governor.py` carica mediante AST la classe **di produzione**
`ProstheticSegmentKinematics` da `osim_trj_cmc_like.py`, senza importare lo stack
OpenSim. Ricostruisce tutti i 500 segmenti di best80 LEFT, RIGHT60, V26 e dei due
oracle diretti del 21 settembre. Verifica anche la conversione azione applicata
normalizzata -> bounds assoluti -> target slew -> endpoint registrato.

- Errore massimo rispetto ai file reference STO: circa **5.1e-9** in q/qdot,
  compatibile con le 8 cifre decimali dei file.
- Differenze rispetto ai trace LEFT/RIGHT: circa 3e-8 rad, 1.2e-7 rad/s,
  1.9e-6 rad/s2, compatibili con stati float32 nei trace. V26 registra double.
- Reset comune: q `[-0.2257699263227882, 0.3420621490728574]`, qdot
  `[-0.025858258121854143, 0.4299230359007116]`, accelerazione interna iniziale
  del governor **zero**. L'accelerazione reference osservata prima del primo
  step appartiene al prescribed e non deve inizializzare il governor.
- Bounds endpoint knee `[-1.5, 0]` rad; ankle `[-0.7, 0.7]` rad.
- Target slew disabilitato in tutti e cinque i run; lo script verifica questo
  caso e rifiuta l'ottimizzazione se lo slew cambia.
- Cutoff 4 Hz; limiti velocita `[6, 3.5]` rad/s, accelerazione `[60, 55]`
  rad/s2, jerk `[3000, 2750]` rad/s3.
- Config archiviate LEFT/RIGHT e config caricate dagli ancestors dei checkpoint
  differiscono soltanto nella sezione logging (`config_identity.json`).

L'ottimizzazione offline applica lo stesso algoritmo e le stesse formule a
entrambi i target e a entrambi i giunti. Nessun parametro e stato selezionato
in funzione dei risultati LEFT o RIGHT. L'inizializzazione analitica e:

`endpoint = q + 2*qdot/w + 2*qddot/w^2 + qjerk/w^3`, `w = 2*pi*cutoff`.

Le derivate superiori sono calcolate con `numpy.gradient` sul target registrato.
Si minimizza poi la media di errore q normalizzato al quadrato piu 0.1 volte
l'errore qdot normalizzato al quadrato, con penalita 1e-4 sul jerk normalizzato
e 1e-4 sul rate endpoint normalizzato. I range q/qdot del target attivo
definiscono le unita (floor 0.05 rad/0.1 rad/s); jerk e rate sono normalizzati
con i limiti fisici J e V. La scelta dei pesi e dichiarata nel codice e comune
a tutte le prove, non costituisce una proposta di reward di produzione.

L-BFGS-B usa bounds originali, maxiter 100, ftol 1e-10, gtol 1e-7, maxls 30,
budget 100 secondi/giunto. Tutti e quattro i solve hanno dichiarato convergenza
in circa 5-7 secondi/giunto. L'integrazione ottimizzata riproduce esattamente
clipping e limiti del governor. La validazione finale usa nuovamente la classe
di produzione, dopo quantizzazione in azioni **float32**.

I gradienti analitici sono verificati con differenze finite sia
sull'inizializzazione sia sui comandi storici che attivano spesso i limiter:
errore massimo circa 2.1e-10. L'implementazione rapida e quella produzione
differiscono meno di 5.3e-9 nello stato q/v/a di fine segmento.

## Risultati

RMSE served-target, dopo i primi 0.2 secondi; coppie in tabella = knee / ankle.
Queste sono metriche cinematiche, non metriche di coppia SEA.

| Sequenza | q RMSE rad | qdot RMSE rad/s | jerk served RMS rad/s3, episodio intero |
|---|---:|---:|---:|
| LEFT best80 | .11206 / .07038 | .76226 / .51405 | 2469 / 2371 |
| RIGHT60 | .12583 / .04308 | .90641 / .42707 | 2559 / 2046 |
| V26 | .12273 / .02442 | .57185 / .30088 | 2383 / 1951 |
| Oracle LEFT diretto | .11038 / .07399 | .87893 / .85734 | 135 / 183 |
| Oracle RIGHT diretto | .13810 / .05460 | 1.20756 / .71095 | 194 / 156 |
| LEFT formula analitica senza fit | .00815 / .00547 | .06620 / .07171 | 185 / 325 |
| RIGHT formula analitica senza fit | .01301 / .00405 | .18684 / .08572 | 513 / 416 |
| LEFT ottimizzato | .000078 / .000155 | .01730 / .03616 | 165 / 241 |
| RIGHT ottimizzato | .000402 / .000234 | .10767 / .07190 | 226 / 278 |

La formula analitica senza fit migliora gia molto entrambi i lati. Il limite
evidenziato dall'oracle diretto e quindi il ritardo del governor quando riceve
il target stesso, non una dimostrazione che LEFT sia fuori dalla sua capacita.
L'ottimizzazione aggiunge la compensazione della discretizzazione e tratta il
transitorio iniziale sotto i vincoli attuali.

Le candidate ottimizzate rispettano i bounds azione: massimo assoluto LEFT
`[.7405,.5880]`, RIGHT `[.7490,.8191]`. Frazione jerk limiter media LEFT zero,
RIGHT .000273. Non si e ridotta l'ampiezza del target per ottenere questi valori.

## Artefatti per rollout fisico del coordinatore

- `candidates_LEFT.npz`, `candidates_RIGHT.npz`: sequenze ottimizzate.
- `candidates_LEFT.json`, `candidates_RIGHT.json`: stessi endpoint/azioni,
  leggibili senza NumPy.
- `analytic_LEFT.npz`, `analytic_RIGHT.npz`: formula feedforward senza solve,
  per confronto indipendente dall'ottimizzazione.
- `metrics.json`: protocollo, replay, risultati solver e candidate.
- `additional_checks.json`: verifica gradienti su policy saturanti e formula.
- `input_hashes.json`: hash dei principali input dell'esecuzione.
- `replay_metrics.json`, `run.log`, `additional_checks.log`: risultati intermedi.
- `target_derivative_check.json`: controllo descrittivo q/qdot target.

Schema NPZ candidate: `normalized_action` shape `(500,2)` float32 nell'ordine
knee/ankle; `time_start`, `time_end`; `raw_endpoint_rad`; `served_q`,
`served_qdot`, `served_qddot`; `target_q`, `target_qdot`; `initial_q`,
`initial_qdot`; e campioni fine-step `dense_time`, `dense_q`, `dense_qdot`,
`dense_qddot`. Finestra 12.99-17.99 s. La variante analitica omette i dense.

Per un rollout usare la config/checkpoint corrispondente, sostituire le azioni
con `normalized_action[i].reshape(1,2)` e preservare la normale pipeline.
Controllare tempi, reset e target del trace risultante contro gli array NPZ;
in caso di terminazione fisica anticipata confrontare soltanto il prefisso.

## Limiti di interpretazione

- Accesso all'intera traiettoria futura: e un test diagnostico offline, non un
  controllore deployable e non una proposta di tabella per AB06.
- Nessun vincolo di contatto, penetrazione, GRF, COP, reserve o coppia entra
  nell'ottimizzazione: verificare questi aspetti nel rollout fisico separato.
- L'ottimizzazione non certifica un ottimo globale. Un eventuale fallimento
  del solver non sarebbe prova di impossibilita; qui si conservano candidati
  effettivamente verificati nel governor.
- qdot target e derivata numerica del q campionato a 100 Hz non coincidono
  perfettamente, soprattutto RIGHT. Il controllo e solo descrittivo e non
  esclude artefatti di campionamento o confini di fase; non prova un bug.
- I risultati su un target registrato non dimostrano generalizzazione della
  rete o del contatto a soggetti, andature o perturbazioni differenti.

Riproduzione: eseguire `analyze_governor.py`, poi `verify_and_analytic.py`, con
NumPy, SciPy e PyYAML. Il codice e Python portabile; runtime usato qui:
`/opt/anaconda3/envs/envCMC-rllib/bin/python`. Non richiede librerie native SEA.
