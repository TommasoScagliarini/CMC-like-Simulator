# Reset IK nella rete ex-novo: audit indipendenti e test

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 12 ottobre 2026, Europe/Rome. Instruction check token: CMC_AGENT_OK_2026.
Report richiesto con create_report nella stessa sessione.

**Stato:** indagine, test, audit indipendente e confronto finale con Claude conclusi.

## Problema e conclusione sostenuta dai test

L'utente ha chiesto di verificare se posizione e velocità IK al reset siano l'unico dato prescribed usato nella rete ex-novo e se ripetere la posa iniziale possa limitare l'apprendimento. Ha richiesto test, audit indipendente Claude da terminale con Opus 5.5 ed effort massimo e sintesi dei risultati.

**Il reset inizializza davvero q e qdot dall'IK. Il training esaminato usa però tre stati iniziali distinti, ripetuti per runner: la posa non è unica nell'intero training.** La copertura intenzionale dei reset resta ristretta.

L'actor non legge direttamente i riferimenti biologici riservati al critic. Il sistema conserva comunque dipendenze prescribed: tracking biologico continuo da IK, GRF destra sperimentale, critic privilegiato e corridor morfologico derivato dall'IK. Al reset l'actor riceve anche qddot IK nei campi dell'accelerazione della reference servita.

Sei rollout a policy congelata completano cinque secondi. Azzerare le sole velocità protesiche, mantenendo posa, biologia e bootstrap del governor, ha effetto piccolo nella condizione esaminata. Cambiare start modifica invece ritorno, transitorio e margine di penetrazione. **Questi test misurano sensibilità e copertura; non identificano causalmente l'effetto della distribuzione dei reset sull'apprendimento PPO.**

## Banco e strategia

Banco AB06, setup PI321/500 e config risolta del run MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01, sotto Trajectory Generator/runs/training/. Checkpoint rl_module_best, logical iteration 47: corrisponde a 46 nuovi aggiornamenti actor; i 50 nuovi update complessivi del run sono logical 2–51. Actor 35D, osservazione completa 84D, inferenza deterministica seed 123.

Il [protocollo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-dbe35b50ba2605e5>) prevedeva audit sorgenti e log storici, reset passivi, isolamento actor/critic, sei rollout, confronto numerico con lo storico e sintesi dei due audit. Le metriche distinguono startup 0–0,5 s e porzione successiva, come fissato nel harness prima dell'esecuzione; l'addendum corregge la formulazione «primo secondo» del protocollo.

Muscoli/SO, reserve, controllo biologico, governor, limiti, profili, plugin e checkpoint sono preservati. La sinistra usa contatto online applicato; la destra conserva GRF prescribed. Tre processi isolati hanno eseguito ciascuno due casi seriali.

Nel caso zero velocità, un hook del solo harness cambia qdot dei due giunti e motor_speed dei due SEA prima del priming detector. Il governor conserva la propria condizione IK originale: è una perturbazione della velocità fisica a parità di reference iniziale.

## Percorso del reset

reset() chiama _initialise_episode(), cancella segmento protesico e memoria governor. Con imitation_initialize_to_target=false, il primo kin.get(t0) restituisce la IK base filtrata a 6 Hz. Il runner scrive q/qdot delle 21 coordinate, inizializza muscoli e stati SEA, azzera controlli e memorie controller/SO e prepara il detector.

- Motor angle coincide con q del giunto: deflessione e coppia elastica iniziali nulle.
- Motor speed coincide con qdot; integrale PI e controlli sono azzerati.
- Previous endpoint e reference iniziale ereditano la IK.
- Il governor si inizializza al primo segmento con accelerazione interna zero.
- Prima del segmento i campi actor served_ref_accel contengono qddot IK: −14,6960 rad/s² knee e −15,9921 rad/s² ankle nello start nominale.
- La FSM dipende dalla geometria: lo start −0,2 s è WAIT_HS; gli altri casi partono in stance parziale. Il carico online osservato al reset vale ancora zero anche quando la FSM è già in stance.

Fonti: simulation_runner.py alle righe 335 e 1796; Trajectory Generator/osim_trj_cmc_like.py alle righe 1786, 2536 e 2939. Le sonde native confermano errore massimo q e qdot applicata **esattamente zero**. Tre reset nominali con seed 123/124/125 producono stati e osservazioni actor identici.

## Distribuzione realmente usata nel training

Config risolta: random_init=false, tre offset 1,756870984 / 1,956870984 / 2,156870984 s, dodici runner, quattro per offset. L'assegnazione è deterministica e ogni runner ripete il proprio start. exact_start_sampling bilancia le transizioni; non randomizza la posa a ogni reset.

I log confermano 1.536 transizioni per offset per update: **76.800 per offset, 230.400 complessive** sui 50 nuovi update. Best47 aveva ricevuto 70.656 nuove transizioni per offset. Sono registrati 152 episodi completati per offset in 38 gruppi sincronizzati; questi conteggi non costituiscono un journal di ogni reset.

| Offset rispetto al nominale | Return storico medio | Range delle medie per gruppo | Advantage medio positivo |
|---|---:|---:|---:|
| −0,2 s | 50,239 | 36,521–53,202 | 44/50 update |
| 0 s | 25,657 | 23,446–28,013 | 3/50 update |
| +0,2 s | 31,386 | 22,793–35,746 | 10/50 update |

Gli start non sono tre copie della stessa posa: q knee ricostruita dai file attuali è circa −36,41° / −9,98° / −13,64°, con qdot +4,011 / −0,0418 / −0,0921 rad/s. Queste ricostruzioni sono confermate dalle sonde odierne, ma non vanno presentate come nuove misure dei reset storici. Il run non conserva un journal completo dei reset o hash di tutte le sorgenti eseguite.

I trend dei return sono sostanzialmente piatti. Explained variance VF media 0,0647, range −0,0462–0,1291. Il parent critic-rewarm usava solo lo start nominale, 4096 step e morphology weight 1,05; il run usa peso 5,0. Gli advantage sono standardizzati globalmente: il segno per start non misura da solo l'errore grezzo del critic. Questi dati e il learning rate 5e−7 non identificano il motivo causale della curva piatta.

Evidenze: [audit della distribuzione — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-94d9a2c7a259ee1a>), [statistiche indipendenti — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-7f75654cb93080b6>).

## Prescribed: actor, critic e plant

L'actor legge soltanto 35 feature; la value function legge tutte le 84. Una prova runtime ha alterato i 49 ingressi critic con zeri, permutazione e valori finiti molto grandi: **24 controfattuali su otto osservazioni, distribuzioni e azioni actor esattamente invarianti**.

La separazione non elimina il supporto prescribed del plant. Le 19 coordinate biologiche sono guidate continuamente dall'IK, con muscoli e reserve; la GRF destra viene applicata da file. Il critic riceve riferimenti biologici _kin_ref a ogni passo. I pros_*_target del suffix usano invece la reference servita: dopo t0 non sono una waveform IK protesica continua.

In questa config blend_tracking, blend_reference e blend_bio sono tutti zero: gli score computati non contribuiscono direttamente al ritorno. Il corridor morphology rimane derivato dall'IK e indicizzato dagli eventi FSM. È un prior sulla forma, distinto dall'imposizione di q_IK(t) all'actor. La reference servita e la memoria controller diventano poi funzioni della policy/governor, pur ereditando il bootstrap iniziale.

Ricevuta: [isolamento actor/critic — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-c6091da15db50c0e>).

## Sei test dinamici a pesi congelati

Tutti i casi: 500 step da 10 ms, 5 s fisici, termine normale episode_time_limit, zero eventi invalidi e zero clipping delle azioni policy. RMSE contro la reference servita, distinta dall'errore contro la IK originale.

| Reset | Return | Cicli validi | Penetrazione max [mm] | RMSE knee / ankle [°] | Campioni morphology risolti / 500 |
|---|---:|---:|---:|---:|---:|
| Nominale addestrato | 27,424 | 2 | 24,649 | 0,571 / 0,506 | 376 |
| Addestrato −0,2 s | 53,939 | 3 | 23,508 | 0,651 / 0,450 | 481 |
| Addestrato +0,2 s | 38,062 | 2 | 22,445 | 0,682 / 0,530 | 403 |
| Nuova fase +0,4 s | 11,989 | 2 | 27,771 | 0,799 / 0,572 | 410 |
| HS sinistro successivo | −11,713 | 2 | 23,699 | 0,798 / 0,509 | 391 |
| Nominale, velocità protesiche zero | 27,451 | 2 | 24,673 | 0,570 / 0,506 | 377 |

Il replay nominale riproduce **byte per byte i 17 STO** del controllo AB06 storico del 6 ottobre. Verifica il banco nominale; non certifica automaticamente ogni runtime storico o ogni start.

Nella prova zero cambiano esattamente quattro state variable: speed knee/ankle e motor_speed dei due SEA. Posa, velocità biologiche e reference iniziale restano identiche. Return +0,02755, circa +0,100%; penetrazione +0,02386 mm. La differenza actual contro actual nominale è RMS 0,1025° knee e 0,0650° ankle sui cinque secondi. Il risultato non supporta una forte dipendenza dalle **sole velocità protesiche** iniziali nella condizione provata. Non è stato rimosso il bootstrap del governor o quello biologico.

Gli start nuovi completano l'episodio ma hanno return inferiori. A +0,4 s: RMSE knee nei primi 0,5 s 1,681°, poi 0,629°; ankle 1,043°, poi 0,493°. Margine dalla guardia di 28 mm: solo 0,229 mm. Sul successivo HS il return è negativo pur con due cicli e buon inseguimento della reference.

Il caso −0,2 s parte in WAIT_HS e accredita tre cicli; il nominale parte in stance parziale e ne accredita due. Cambiano anche finestra biologica/GRF e copertura morphology. La differenza di return non può essere assegnata alla sola posa o interpretata automaticamente come migliore qualità meccanica. Il successivo HS include anche una porzione temporale oltre le finestre usate nel training esaminato.

Evidenze: [risultati consolidati — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-0e0a324ef1132e0e>), [sonde passive — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-9ff584926faefe68>), [42 plot canonici](../../plot/reset_ik_audit_2026_10_12/).

## Audit indipendenti e confronto con Claude

Codex ha sigillato il proprio audit source prima di leggere l'output Claude. Claude ha svolto l'audit da terminale con claude-opus-5-5, effort max, strumenti Read/Grep/Glob e senza leggere i risultati Codex della nuova indagine. La ricevuta del provider conferma il modello effettivo claude-opus-5-5; nessun fallback. Non ha eseguito OpenSim.

Entrambi gli audit distinguono reset IK deterministico, tre start fissi, actor separato dal critic, supporto prescribed del plant e mancanza di un effetto causale dimostrato sul training. Claude ha segnalato anche qddot IK nell'osservazione iniziale e i confondenti del critic-rewarm; Codex li ha verificati nei log, nelle sorgenti e nelle sonde native.

Il primo audit Codex resta congelato. Un [addendum — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-bbf3dd9795cb439e>) chiarisce che il possibile contributo bio alla reward citato condizionalmente è inattivo nella config. Le espressioni del primo audit Claude su equilibrio «garantito» dalle reserve o curva piatta spiegata dal learning rate non vengono adottate come esiti causali: le reserve forniscono supporto, mentre critic e learning rate rimangono ipotesi concorrenti.

Fonti: [audit Codex sigillato — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-23395efe65ff8df8>), [audit Claude — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-32801c44eb48464a>), [ricevuta modello — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-9b0407d87a4f6e41>).

### Discussione finale e sintesi riconciliata

La seconda chiamata Claude, ancora Opus 5.5 con effort massimo e senza tools, ha analizzato entrambi gli audit e i risultati numerici. Modello effettivo confermato anche in questa fase. [Risposta completa — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-926d23f1034eb9fa>) e [sintesi riconciliata — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-d14b5a947e80d671>).

Claude conferma i fatti e la mancata identificazione del nesso causale reset–apprendimento. Accolte le precisazioni: una posa per runner, tre nell'intero training; forte dipendenza del return dallo start, più contenuta nelle metriche meccaniche osservate; il test zero velocità non rimuove la velocità IK dalla reference/governor; curva storica senza miglioramento rilevabile, senza attribuirne già la causa all'ottimizzatore.

Il successivo HS ha posa protesica vicina al nominale ma qddot knee iniziale +0,816 contro −14,696 rad/s², oltre a una diversa finestra biologica/GRF. La riduzione del return non isola quindi la posa o la sola copertura morphology. I return fra start diversi restano totali descrittivi, non un confronto causale di qualità a condizioni comuni.

Una precisazione alla risposta Claude: reserve e residui sono stati registrati nei file nativi e le loro massime forze generalizzate sono nelle metriche, pur escluse dalla capsula compatta della discussione. Per la reserve verticale, picco assoluto nominale 364,90 N, +0,4 s 531,09 N, successivo HS 470,34 N e zero velocità 363,17 N. Non ne deriva una qualifica scientifica dell'equilibrio. La [review indipendente dei dati salvati — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-36d321a4924231ed>) conferma i ricalcoli e l'ispezione visiva di 18 PNG.

La soluzione operativa è mantenere separati diagnosi e modifica: il reset attuale non è stato identificato come collo di bottiglia dell'apprendimento. Per una prova causale, definire prima un controllo positivo di apprendimento e una inizializzazione critic comune sulla reward effettiva; poi confrontare distribuzioni reset con unico fattore variato, budget/seed uguali e start held-out comuni. Aumentare gli start fissi per runner e ricampionare per episodio sono interventi diversi. Questa è una proposta futura RIK-T01, non un training già eseguito né una modifica già autorizzata del learner o del bootstrap PPO rinviato.

## File aggiunti e verifiche

Il nuovo codice è tutto in Trajectory Generator/validation/reset_ik_audit_2026_10_12/: frozen_reset_tests.py, test_frozen_reset_tests.py, privilege_invariance.py, consolidate_results.py, claude_audit.py e helper/statistiche distribution/. La cartella conserva protocollo, manifest/hash, log, watchdog, receipt e audit. Aggiunti questo report e il daily parziale del 12 ottobre per propagare i TODO; nessuna chiusura end_day.

Nessun sorgente operativo, plugin, checkpoint, modello o profilo è stato modificato dall'indagine. Verifiche: **10 test offline PASS; reset nativi e matching SEA PASS; 24 controfattuali critic PASS; sei rollout; 102 STO × 5.000 righe finite; replay nominale esatto; 73 input protetti invariati; 42 PNG, sette per caso, senza canali mancanti.**

Primi tentativi conservati: login Claude non disponibile nel sandbox, poi operativo fuori sandbox; due errori del solo harness prima dei rollout, config frozen e directory precreata dal watchdog, corretti con nuovi test. Il consolidatore ha inoltre corretto la lettura della frase «No missing channels.» come assenza effettiva di canali mancanti. Non sono state ripetute selettivamente traiettorie fallite per promuovere gli esiti.

Le prove native attestano macOS arm64. Il harness usa pathlib, subprocess e interprete selezionabile; la verifica nativa Windows della pipeline resta aperta. Nessun aggiornamento PPO eseguito in questa indagine.

## Limiti e TODO da propagare

Un solo checkpoint, un soggetto/condizione, inferenza deterministica e pochi start. I seed diversi verificano ripetibilità, non costituiscono repliche di training. Zero velocità protesiche in una posa IK di cammino non è una posa statica bilanciata: **V26-T04 resta aperto**. Restano validi i TODO precedenti di generalizzazione, qualifica fisica, Windows e bootstrap PPO.

- [ ] **RIK-T01** — Preregistrare ed eseguire un confronto PPO fra distribuzione reset attuale e ampliata, con stessi pesi iniziali, reward, learner, calibrazione iniziale del critic, budget e seed; valutare su start comuni held-out e distinguere copertura, accredito FSM e finestre biologiche/GRF. Non cambiare contemporaneamente learning rate o reward per attribuire l'eventuale beneficio ai reset.

- [ ] **RIK-T02** — Definire e qualificare un bootstrap iniziale causale e coerente di osservazioni/reference/governor/SEA/FSM, verificando in particolare qddot IK nell'actor a t0 e carico inizialmente zero; confrontare gli esiti con il contratto congelato senza riscrivere gli artefatti storici.

