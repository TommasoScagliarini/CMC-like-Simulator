# Audit locale dell'apprendimento PPO — 29 settembre 2026

Instruction check token: `CMC_AGENT_OK_2026`.

Autorizzazione utente: «procedi» sul piano di audit di un batch on-policy e un
aggiornamento PPO completo. Nessun teacher nel training. Qualità: errore e forma
cinematica; principalmente forma delle coppie, con V26 come benchmark descrittivo.

## Esperimento preregistrato

- Sorgente: checkpoint completo LEFT best80, penalty penetrazione 2. Ripristino
  actor, critic, momenti Adam, learning rate, coefficiente KL ed entropy verificati.
- Reward, architettura, osservazioni, governor, contatto e configurazione PPO
  invariati. Nessun critic rewarm. Unico aggiornamento logico: iterazione 81.
- Nuova raccolta on-policy: 4096 transizioni, 13 worker, seed 123, minibatch 512,
  10 epoche, gamma 0.99, lambda 0.9. Il checkpoint non conserva lo stato fisico o
  RNG storico dei worker: non si pretende di ricostruire il batch storico81.
- Cluster Ray dedicato e priorità ridotta durante il training da zero già attivo.
  Un'eventuale terminazione dovuta al watchdog resta un errore operativo e non
  un esito scientifico. Nessuna ripetizione automatica dell'update.
- Strumentazione osservativa su istanze del learner: episodi prima della GAE,
  valori calcolati dalla pipeline originale, batch dopo GAE, minibatch effettivi,
  loss/ratio/KL/clipping e snapshot dei parametri. Non sostituire la loss.
- Le righe di bootstrap artificiali sono distinte dalle transizioni vere tramite
  maschere. Otto minibatch non sono necessariamente un'epoca completa.
- Verificare ratio iniziale e logprob usando l'azione raw campionata. Il clipping
  dell'azione applicata non costituisce da solo un errore del policy gradient.
- Verificare GAE, value target, terminazioni, truncation e bootstrap; misurare
  errore critic grezzo e quota limitata dal cap. Correlazioni con reward o
  cinematica non dimostrano causalità né il vero vantaggio di una singola azione.
- Confronto fisico best80/primo passaggio disponibile/finale: nominale e, per
  best80/finale, seed rumorosi 123/125/127, stream gaussiani appaiati e sigma
  propria di ciascuna policy. Stesso reset, orizzonte massimo500, fallimenti
  espliciti. Riutilizzare un riferimento solo dopo verifica di configurazione,
  sigma e input. I sette plot canonici sono salvati sotto root `plot/`.

Precisazione prima di avviare le valutazioni: il nominale usa la pipeline
deployment già adottata nei benchmark e riproduce bit-per-bit i comandi archiviati.
Gli episodi rumorosi replicano anche `NormalizeAndClipActions` del training
(`normalize_actions=True`, `clip_actions=False`, `unsquash_action` di RLlib),
prima dell'env: la mappa sul Box[-1,+1] può differire dal clip diretto di circa
una ULP in float32. Conservare raw action e action_for_env. I confronti before/after
usano lo stesso protocollo; il nominale non è una stima Monte Carlo del ritorno
della policy stocastica. Il termine reward del clipping azione ha peso zero.

Il primo avvio è stato interrotto in `PPOConfig.build_algo`, senza sampling o
aggiornamenti: il setup hook Ray non trovava il pacchetto del progetto. Tentativo
conservato in `attempts/01_startup_import`; launcher corretto con PYTHONPATH
esplicito e input ricongelati prima del nuovo avvio. Non è un esito RL.

## Interpretazione e passi condizionati

Separare: tentativi esplorati, convenienza secondo l'obiettivo scontato, segnale
del critic/GAE, aggiornamento sul batch, comportamento fisico successivo. Una
sequenza offline nominale favorita dalla reward non prova rappresentabilità
causale della policy39, rendimento stocastico superiore o gradiente PPO utile.

Una correzione viene scelta solo in presenza di un difetto riproducibile; eventuali
varianti dell'ottimizzazione riusano batch e stato iniziale congelati. Nessun tuning
specifico AB06 della reward. Il primo audit non certifica apprendimento da zero,
robustezza statistica o trasferimento ad altri modelli, attualmente indisponibili.

## Intervento condizionato: target ai confini dei frammenti

Il confronto indipendente con la ricorrenza TD/GAE ha rilevato un difetto nei
connector Ray2.55.1: il valore della prossima osservazione viene annullato ai
confini artificiali del campionamento. Un riproduttore con due transizioni e
valori noti conferma anche un errore nella propagazione precedente una vera
terminazione. I tre sorgenti installati coincidono byte per byte con il tag
ufficiale; la correzione upstream è il commit
`1927658ad6a97788bd86fc59310d1ed45a08e60e`, PR59958, 27 maggio2026.

Questo difetto attiva il passo condizionato già previsto. Il replay locale
riutilizza episodi, critic iniziale, actor, Adam, stato ausiliario e RNG del batch
congelato, senza ambiente o nuovo sampling. Prima di eseguire il ramo corretto,
il controllo deve riprodurre esattamente lo stato finale actor/critic/Adam e
l'ordine dei minibatch dell'aggiornamento osservato.

Il ramo corretto cambia soltanto target e advantages delle transizioni reali
secondo la ricorrenza standard. Conserva i target artificiali stock e la stessa
convenzione di normalizzazione su tutte le righe: l'effetto della normalizzazione
è una questione distinta. Questo è un intervento per isolare il difetto ai
confini, non un backport completo della patch upstream né una modifica alla
dipendenza installata. Reward, iperparametri PPO e governor restano invariati.

Il confronto fisico aggiunge quattro casi separati: nominale e rumore con seed
123/125/127, sigma propria, stesso reset e stessi flussi Gaussiani dei nove casi
originali. Il manifest aggiuntivo conserva distinti i due gruppi; si mantiene
un massimo complessivo di due simulazioni dell'audit simultanee.

Una correzione locale non ripara la storia delle prime80 iterazioni. Il suo esito
non certifica il training corretto da zero né la possibilità o impossibilità
del RL di raggiungere V26. V26 e LEFT best80 registrano la stessa versione Ray
e lo stesso percorso dei connector: il difetto è potenzialmente condiviso,
mentre la differenza del suo impatto storico non è stata misurata.
