## Confronto finale: audit Claude/Codex e reset da pose protesiche casuali (best47, AB06)

**Limiti.** Ragiono solo sulla capsula. Non ho visto STO, plot, journal, receipt, codice finale né il testo sigillato di Codex, e non posso verificare gli hash. Ho controllato la coerenza interna di conteggi, ID, CP e strati, non i dati grezzi. Non conosco la semantica esatta di «confirmed», le finestre dei diagnostici né la massa di AB06. Nessuna prova futura citata qui è stata eseguita.

**Verdetto.** Il risultato regge come prova condizionale. Nel box ammesso, 18/20 pose casuali portano il sistema chiuso a un gait valido secondo criteri congelati, senza esclusioni né sostituzioni. È evidenza di un ampio bacino di attrazione del sistema intero, a un solo istante del passo e con velocità protesica nulla. Non dimostra il merito della rete, l'equilibrio, la copertura del box intero né un'affidabilità ≥95%.

### 1. Il mio audit: cosa regge e cosa no

Codex ha sigillato le sue conclusioni prima di leggere il mio audit, quindi le convergenze sono indipendenti. Sono quattro: bootstrap coerente necessario, nessuna attribuzione al feedback senza replay, piano primario ≠ detector, CI condizionale a N=20. I dati le confermano tutte. Le mie sfumature sono in §3, §4 e §5.

**Confermato**
- **Bootstrap.** Senza adapter il governor ripartirebbe da anchor IK, quindi cambiare solo le q fisiche non basta. I probe nativi del primo segmento attenuano la tautologia del controllo al reset che avevo segnalato.
- **Harness.** Secondo la capsula, i rischi che avevo segnalato sono chiusi:
  - probe via wrapper originale (1,2)+clip, confermato dai probe nativi;
  - count 20 e hash dei 401 receipt;
  - journal incrementali e ritorni censurati;
  - batch che prosegue, post-processing separato, timeout trattato come tecnico.
  
  Che l'IndexError si sarebbe davvero manifestato resta una mia deduzione.
- **Gate e dominio.** random006, ammesso a 23.6 mm, supera la guardia a 30 mm: unisce il rischio del gate a 27 mm e la censura del segmento terminale. random025 parte a 0.045 rad dalla guardia del ginocchio e fallisce proprio lì.
- **Detector ≠ forza.** Il nominale è HEEL con penetrazione primaria 0. Altri cinque casi TOE/HEEL hanno 0 mm.
- **Asimmetria tra partenze AIR e in contatto.**
  - I 9 pass AIR hanno il primo HS a 0.13–0.43 s e 3 cicli. Il primo ciclo è presumibilmente accorciato, ma l'HS2 non è in capsula.
  - I 9 pass da contatto hanno il primo HS a 1.649–1.650 s (nominale 1.648) ed esattamente 2 cicli.
- **RMSE actual–served.** Vale 0.45–0.67°, con margine di circa 3× sulla soglia di 2°. Misura il controllore e non ha discriminato nessun caso.

**Corretto**
- **Biologia.** Non è prescritta: viene integrata liberamente e tracciata da muscoli e reserve. «Il bacino non può cadere» era troppo forte. I residui pelvici d'avvio restano però alti: 227 N nel nominale, circa 243 N in media nei pass da contatto, circa 310 N negli AIR. Il test quindi non prova l'equilibrio.
- **Due cicli in 5 s.** Avevo previsto che, partendo in contatto, il terzo HS cadesse oltre 5 s: era sbagliato, il nominale completa 2 cicli. Il margine però è sottile. Il periodo medio è al massimo (5−1.649)/2 ≈ 1.676 s; con eventi a circa 1.669 s resterebbero circa 13 ms. L'istante dell'HS3 non è in capsula.
- **Orizzonte 7 s e L4 come criterio primario.** Non erano nel protocollo congelato e, a esiti noti, non sono più adottabili. Il confronto late actual-vs-nominal resta descrittivo.
- **Conclusione ammissibile.** La mia presupponeva 20/20 e va riformulata (§7).

### 2. Denominatori e CP

- **Pool.** 400 proposte, 36 scarti (30 oltre 30 mm, 6 tra 27 e 30), 364 ammesse (91%). I conti tornano.
- **ID.** Gli ammessi vanno da 003 a 025; mancano 001, 002, 011, 012 e 024. Quindi 25 proposte consumate, 5 scarti, 20 ammessi in ordine, nessuna sostituzione, 0 unresolved.
- **CP.** Per 18/20 l'intervallo 95% bilaterale è [68.3%, 98.8%] e il limite unilaterale è 71.7%. Ho ricalcolato le code binomiali: i valori sono corretti.
- **Cosa stima il CP.** L'esito è deterministico data la posa. Il CP stima quindi la frazione della distribuzione ammessa che passa, non l'affidabilità tra ripetizioni.
- **Peso del denominatore.**
  - 90% sulle pose ammesse.
  - Circa 82% sul box intero (0.91×0.90) se gli scarti contassero come insuccessi.
  - 72% (18/25) sulle proposte consumate, con la stessa ipotesi.
  - Gli scarti non sono insuccessi, ma non sono nemmeno stati testati.
- **Copertura realizzata.**
  - Nessuna caviglia sotto −0.31 rad e solo 2/20 sotto −0.10, una fascia pari a circa il 41% dell'intervallo di caviglia.
  - 13/20 pose cadono nel range IK, che occupa circa il 38% del box.
  - 5 scarti su 25 proposte consumate, contro il 9% del pool.
  - Se dipenda dal gate, dal caso o da entrambi si può verificare sui receipt esistenti.

### 3. Cause dei failure

**random025** (AIR; ginocchio −0.0455 rad)
- 13 azioni raw del ginocchio su 14 sono oltre +1 e vengono clippate.
- Il riferimento servito sale a −0.0033 rad, cioè verso l'estensione.
- L'angolo attuale lo supera di circa 0.21° e attraversa il bordo operativo di 0 rad di +0.025° a 0.14 s, comunque entro il ROM di ±π/2.
- È l'intervento di una guardia operativa preregistrata: non è un crash né una caduta, ma non è neppure un artefatto. Fuori dal range IK del ginocchio, le uscite della policy erano saturate verso l'estensione.
- Cosa sarebbe successo senza la guardia non è osservabile.

**random006** (HEEL; ginocchio −0.168, caviglia +0.375)
- La penetrazione sale da 23.56 a 30.13 mm in 0.319 s. Il superamento avviene su un campione fisico successivo all'ultimo boundary (0.310 s): 31 finestre positive più un ritorno a durata zero.
- Viola sia la guardia nativa sia il criterio a 28 mm, quindi la classificazione regge con qualunque lettura.
- La posa è dentro il range IK di entrambi i giunti.
- Ipotesi compatibili ma non separabili: contatto iniziale profondo a cui si somma il loading response, e carico osservato pari a 0 fino al primo campione.

**Pattern osservato (a posteriori, non inferenziale)**
- I due failure sono le due pose con il ginocchio più esteso, entrambe vicine a una guardia, ed entrambi avvengono entro 0.32 s.
- Tutti i pass hanno ginocchio ≤ −0.349 rad.
- Fuori dal range IK passano 6 pose su 7, dentro 12 su 13. Il fattore comune osservato è la vicinanza alle guardie, non l'uscita dal dominio IK in sé.
- Il 90% è quindi una media su regioni forse diverse e dipende dal disegno del box.

### 4. Completion vs recovery

- **Discriminazione dei criteri.** Nessun caso completato ha fallito i criteri stretti (18 completati, 18 sicuri, 18 pass). Qui i criteri di gait non discriminano oltre la completion.
  - Escursioni per ciclo: ginocchio 0.755–0.826 contro soglia 0.60; caviglia ≥0.43 contro 0.30.
  - I soli criteri non distinguono tra robustezza dopo il transitorio e soglie troppo larghe.
- **Confronto late actual-vs-nominal (descrittivo).** È più informativo.
  - Pass da contatto: ≤0.024°, cioè convergenza all'orbita nominale.
  - Pass AIR: ginocchio 0.47–2.89°, caviglia 0.19–0.99°. L'escursione del ginocchio nell'ultimo ciclo è 0.755–0.777 rad contro 0.803 del nominale: a 5 s la convergenza è solo parziale.
- **Penetrazione.** Il picco è 22.6–24.8 mm in tutti i pass e nel nominale: 3.2–5.4 mm sotto la soglia di 28 mm e già oltre la penalità di 20 mm. «Sicuro» significa entro soglia, non con ampio margine.
- **Fragilità correlata.** I casi da contatto ereditano il margine temporale del nominale (HS1 entro ±1 ms). Uno spostamento sistematico di quell'ordine, anche solo numerico, li farebbe fallire tutti insieme sul criterio dei 2 cicli. È un rischio di falsi negativi, non di falsi positivi.
- **Diagnostiche.**
  - 5 pass AIR hanno return sotto −70, e il return AIR cala al crescere della flessione iniziale: il return non misura il recupero.
  - Il clipping è nullo in tutti i pass.
  - Le diagnostiche d'avvio di random025 coprono solo 0.14 s: sono censurate e non confrontabili.
  - Da questi dati non traggo alcuna inferenza sul learning PPO.

### 5. Reader V2

È una correzione di lettura legittima, ma successiva al sigillo di V1, quindi va documentata con il diff V1/V2. È accettabile per tre ragioni:
- V1 resta sigillato.
- I 18 pass hanno 500 finestre positive e nessun ritorno a durata zero, quindi le regole V2 non li toccano.
- Nessuna lettura poteva promuovere random006 a pass; l'unico rischio era escluderlo (18/19). V2 sceglie l'opzione conservativa e corregge solo la causa attribuita.

### 6. Cosa è dimostrato

- **Gait recuperato nel plant: sì, 18/20.** Vale per l'intero sistema chiuso (policy, slew, governor, SEA/PI, contatto, FSM, biologia con reserve), secondo criteri congelati.
- **Merito esclusivo della rete: no.** HS1 entro ±1 ms e residuo ≤0.024° indicano una forte contrazione. Lo stesso quadro però è compatibile con governor e slew che farebbero convergere anche una sequenza open-loop, e con il trascinamento da parte della biologia. Senza replay le due spiegazioni non si separano.
- **Standing o avvio da fermo: no.** Il reset avviene in pieno cammino.
- **Box intero: no.** Il 9% scartato non è testato, le caviglie negative sono sottorappresentate, e la fascia a ginocchio esteso ha solo due campioni, entrambi failure.
- **Affidabilità del 95%: no.** Il 95% è compatibile con l'intervallo, ma il limite inferiore è 71.7%. Senza rumore né ripetizioni, l'affidabilità operativa non è misurata. Servirebbero almeno 59 successi su 59, oppure circa 124 casi con al più 2 failure, su un nuovo campione preregistrato.

### 7. Sintesi comune proposta

> **Condizioni.** AB06, best47 congelata e deterministica, reset all'HS sinistro (t0=13.947 s) con q̇=0, SEA scarica e bootstrap coerente. Biologia muscle/reserve-driven. Pose uniformi nel box, ristrette al gate (91%).
>
> **Esito.** Il sistema protesico chiuso ha completato 5 s con gait valido, secondo criteri congelati, in 18/20 casi (90%; CP95 68.3–98.8%; ≥71.7% unilaterale). I casi partiti in contatto riconvergono all'orbita nominale; quelli partiti AIR raggiungono un gait valido con convergenza parziale. I due failure sono fisici, precoci (≤0.32 s), sulle pose più estese e vicini a guardie preregistrate. Nessun caso completato ha fallito i criteri di gait.
>
> **Non dimostrato.** Merito del feedback, equilibrio, box intero, affidabilità ≥95%, altre fasi, velocità, soggetti o piattaforme.

### 8. Prove future (non eseguite, non retroattive)

Queste prove estendono la portata del risultato, non lo invalidano:
1. Replay open-loop delle azioni nominali dalle stesse pose; ablazioni di governor e slew.
2. Campagna stratificata e preregistrata su:
   - ginocchio esteso (per esempio > −0.30);
   - caviglia negativa;
   - partenze AIR contro partenze in contatto, su entrambi i piani;
   - penetrazione iniziale ≤12 mm contro 12–27 mm.
3. Orizzonte più lungo, con criteri relativi al nominale preregistrati e istante dell'HS3 riportato.
4. Numero di casi dimensionato sul claim desiderato.
5. Altre fasi del passo, q̇ e precarico SEA non nulli, rumore o policy stocastica, altri soggetti.
6. Bilancio delle reserve; equilibrio e standing trattati come compiti distinti.
7. Riproducibilità cross-platform, Windows inclusa, tenendo conto del margine temporale condiviso.
8. Verifica separata del mascheramento dell'end_reason anche nei log storici di training.
