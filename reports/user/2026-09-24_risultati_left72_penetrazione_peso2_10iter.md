# Risultati LEFT72: dieci iterazioni con peso penetrazione 2

## Problema e strategia
Verificare se raddoppiare grf_penetration_weight da 1 a 2 migliori la robustezza del training imitativo LEFT della unified39. Resume completo da best72, dieci aggiornamenti 73–82, 4096 transizioni per aggiornamento, 13 worker. FSM v3, prescribed e clock visibili al 100%. Attiva anche la correzione del contatto terminale hard. Il controllo contemporaneo peso1 non è stato eseguito.

## Esecuzione e verifiche
Conclusione regolare 24 settembre 2026 alle 13:02:14 CEST, durata 85 minuti e 40 secondi. Dieci aggiornamenti riusciti, 40960 nuove transizioni; tutti i batch hanno 4096 campioni visibili. Best del nuovo ramo: iter80, ritorno medio di training 261.74765; last82: 242.38823. Ritorni non confrontabili direttamente con reward peso1.

Telemetria: 86 episodi conclusi, tutti completi nella registrazione, nessun duplicato o record invalido, tutti i 13 worker osservati. Conteggi coerenti con le differenze dei contatori cumulativi rispetto all’iterazione72: 54 completi, 28 terminazioni ordinarie per penetrazione e 4 hard. I quattro terminali hard conservano misura fisica (30.145–30.226 mm) e loss di penetrazione non nulla (1.608–1.634 prima del peso), oltre alla safety. La correzione software opera anche in produzione.

## Confronto descrittivo con lo storico

| Finestra | Peso1 storico | Peso2 nuovo |
|---|---:|---:|
| Iter73–82, 40960 transizioni | 49/90 = 54.4% | 54/86 = 62.8% |
| Iter78–82, 20480 transizioni | 24/41 = 58.5% | 27/47 = 57.4% |

Lo storico 63–72 raggiungeva già 66/88 = 75.0%. Nel nuovo ramo le prime cinque iterazioni danno 27/39 = 69.2%, le ultime cinque 27/47 = 57.4%. Le ultime tre danno 21/28 = 75.0%, dopo un calo nelle iter78–79. Non emerge una crescita monotona né una soluzione consolidata. Le percentuali sono rapporti tra episodi conclusi aggregati, non medie delle percentuali per iterazione.

Il vantaggio globale di 8.35 punti percentuali è descrittivo: una sola traiettoria di apprendimento, episodi dipendenti, reset dei simulatori al resume e correzione hard assente nello storico. Non è un esperimento controllato sul solo peso. Gli episodi conclusi coprono 39131 step registrati, mentre 40960 è l’esperienza complessiva raccolta: la telemetria qui analizzata scrive gli episodi alla conclusione, quindi esclude le code ancora aperte all’arresto.

## Diagnosi fisica
Tutti i 32 fallimenti terminano nello stato STANCE_AFTER_HS; nessuna terminazione per timeout swing, evento invalido o divergenza ginocchio. Picco medio di penetrazione: 23.36 mm nei completi, 28.99 mm nei falliti. Cinquanta dei 54 episodi completi oltrepassano almeno una volta 20 mm: terminare l’episodio non equivale ad avere contatto con ampio margine.

Nei 32 fallimenti il superamento di 20 mm è osservabile nella coda e precede il termine mediamente di 10.75 step, circa 0.108 s (range 3–40 step). Nei 50 step finali nessun fallimento ha resync o cancellazione HS; 2/32 hanno cambio policy entro 10 step. Questi dati concentrano l’indagine sul contatto durante l’appoggio e sulla rapidità di crescita della penetrazione; non dimostrano quale componente del controllo la causi.

RMSE fisico giunto-target, media dei RMSE per episodio: completi ginocchio7.55°, caviglia4.01°; falliti ginocchio6.53°, caviglia3.87°. I falliti coprono meno tempo: un RMSE inferiore non significa maggior qualità o robustezza. Non sono RMSE della reference servita dal filtro e non vanno confrontati direttamente con altri tipi di errore nei grafici. Nessun clipping dell’azione osservato. Coppie e chattering non sono stati rivalutati con nuovi rollout.

## Interpretazione e seguito
Il peso2 non ha risolto la fragilità. Esiste un vantaggio aggregato rispetto alla prosecuzione storica, ma non nelle ultime cinque iterazioni. Non giustifica ancora promuovere il nuovo ramo o avviare l’oscuramento.

83/86 episodi attraversano cambi di policy: queste statistiche descrivono la raccolta durante PPO, non la robustezza del checkpoint80 o82 congelato. La policy usata per raccogliere dati precede inoltre l’aggiornamento salvato: il best per ritorno di training è un candidato alla valutazione, non un vincitore già verificato.

TODO: valutare best80 e last82 congelati contro best72, con rollout deterministico e stessi seed/rumore, misurando completamento, cicli validi, penetrazione, cinematica e coppie. Per isolare causalmente il peso serve il ramo di controllo peso1 con lo stesso codice corretto e protocollo di resume. Nessun nuovo training o rollout avviato in questa analisi.

## Artefatti e file
Nessuna modifica al codice di training o simulazione. Creati:
- Questo report.
- `Trajectory Generator/unified_MLP_39/validation/left72_weight2_results_2026-09-24/comparison.json`.
- Nella sottocartella `telemetry/`: `summary.json`, `summary.md`, `per_episode.jsonl`, prodotti dall’analizzatore esistente.

Fonti: summary e train_iterations.jsonl del run `MLP39_LEFT72_penetration_weight2_10iter_2026-09-24`, relative 13 sorgenti episode_telemetry; train_iterations.jsonl storico `MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21`; metadati checkpoint_best e audit startup. Verifica offline, senza simulatore né consumo CPU di rollout.


## Rollout canonici successivi, richiesti dall’utente
Prodotti rollout deterministici di best80 e last82 e sette plot canonici per ciascuno. Entrambi completano500 passi e2 cicli validi; penetrazione massima14.795 mm (best80),18.178 mm (last82). Cartelle root: `plot/09_24_2026_unified39_LEFT_pen2_best80` e `plot/09_24_2026_unified39_LEFT_pen2_last82`. Ricevute aggiornate, tutti14 PNG verificati; figura7 controllata visivamente per target sinistro. Nessun nuovo training. Lo screening rumoroso e il confronto controllato rimangono da fare.
