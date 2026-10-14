# Confronto reward e oscuramento da CP90 — 2026-10-01

Autorizzazione: richiesta utente di eseguire il confronto imitazione/ex novo e separare reward e disponibilità prescribed. `CMC_AGENT_OK_2026`.

## Domanda e intervento

Stesso actor CP90 (best90 del run29set), critic e Adam nuovi, stessa sigma costante0.01 e congelata, stessi semi123, PPO conservativo J20 (LR5e-7, una epoca, clip0.05, KL1), batch4096 e13worker. Prima iterazione critic-only, LR1e-4. I pesi delle medie devono restare esatti durante trasferimento e warm-up.

Reward ex novo: preset J20 settembre, compreso morphology5/5 e penalità configurate. Controllo: stesso dizionario, cambiando soltanto reward_mode; i pesi imitativi inattivi nell'ex novo vengono impostati ai valori CP90 in entrambi i rami. Quindi il controllo ha l'obiettivo imitativo CP90 ma costi comuni e criteri FSM J20. Questo confronto **non** è una ripetizione del precedente pilot imitativo: penetration2 diventa0.05, oltre alle altre differenze documentate nei config.

Fisica, reset, start1s, governor e slew rimangono quelli CP90 (slew disattivato), identici nei due rami. I parametri della FSM provenienti dal reward config J20 sono applicati ad entrambi. Le terminazioni per contratto morphology sono identiche anche nel controllo; la penalità morphology resta inattiva solo nello scalare imitativo. Nessuna modifica al plugin SEA o al simulatore root.

## Curriculum e budget preregistrato

Episodi interi con visibilità dei sei canali75%,50%,25%,0%. Nessuna attenuazione delle ampiezze. Stesso livello in entrambi i rami e RNG inizialmente identici; numero effettivo di passi visibili può differire a causa delle durate.

Per ogni livello: cinque aggiornamenti actor, rollout deterministico visibile e completamente oscurato per entrambi. Si progredisce se il ramo ex novo completamente oscurato soddisfa tutti i criteri:500passi senza terminazione di protezione, almeno2cicli validi, escursione knee>=20gradi e ankle>=15gradi, RMSE actual-served<=2gradi per entrambi. Le escursioni e gli errori sono misurati nella finestra comune13.19–17.94s. Queste sono soglie minime esplorative, inferiori alle escursioni CP90visibile41.71/30.67gradi, non criteri di promozione finale.

Se non passa, entrambi ricevono altri cinque aggiornamenti allo stesso livello e si rivaluta. Se continua a non passare, si arresta la progressione e si documenta il fallimento del recupero entro il budget, senza togliere altre informazioni a una policy ancora incapace. Il controllo segue lo stesso calendario del ramo ex novo: questo mantiene accoppiata l'esposizione e non costituisce una selezione del miglior seed. Al livello finale si eseguono dieci aggiornamenti complessivi per consolidamento, anche se il primo controllo passa.

Massimo40aggiornamenti actor per ramo, più un warm-up. Primo gate dopo5, seconda decisione dopo10. Un seed: screening causale condizionato al protocollo, non dimostrazione di robustezza né test definitivo della reward.

## Verifiche e valutazione

- Test retrocompatibilità default sempre-hidden exnovo; maschere immutate nel batch learner; RNG e resume; export e import; critic-only; sigma costante; metadati e contratto input.
- Smoke nativo per ciascun obiettivo:128passi per iterazione, una critic-only e un update actor, directory separate. Pesi smoke mai parent operativo.
- Rollout iniziali a pesi CP90 trasferiti: visibile e hidden per ogni reward, per verificare uguaglianza fisica e effetto del runtime condiviso prima degli update.
- A ogni gate: completamento, cicli, ROM, actual-served, actual-target, fase, morphology e sua copertura, coppia ankle positiva(area e durata), knee chattering spring/motore. Non confrontare return numerici fra obiettivi diversi.
- Sette plot canonici di ogni rollout nella cartella root plot dedicata; summary comparativo nella presente directory.
- Hash degli input e sorgenti prima delle esecuzioni; resume senza modifiche alle sorgenti durante training; metriche e fallimenti conservati.

## Limiti dichiarati prima dei risultati

Il LR J20, nato per rifinire un actor già adattato, potrebbe essere insufficiente per trasferire CP90 senza prescribed. Un gate fallito entro10update non dimostra impossibilità del metodo. Morfologia reward su served_ref, non direttamente q fisica; valutare entrambe. All'avvio il corridoio può non essere disponibile: registrare copertura ed eventi. Rete indipendente dai sei ingressi non implica intero simulatore privo di prescribed biologici.

## Correzione infrastrutturale durante lo smoke

Il primo smoke nativo ha aggiornato soltanto il critic, poi il guard KL ha arrestato la prova perché il learner standard non emette telemetria per minibatch. Il campionamento esatto esistente richiede tre start distinti e non è applicabile allo start singolo scelto. È stato quindi aggiunto un learner che riusa soltanto la telemetria KL già esistente, senza il connettore di riordinamento per tre start: batch4096,13worker,start1s restano quelli preregistrati. Il fallimento è archiviato; nessun peso smoke viene riusato come parent di training.
