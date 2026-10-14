Verdetto: sui fatti i due audit convergono. Il nesso tra reset e apprendimento resta non identificato. La sintesi Codex regge, ma va precisata in alcuni punti, e l'A/B proposto rischia di non avere potenza statistica.

## 1. Veridicità delle premesse

Sui fatti il mio audit e C1–C6 concordano. C5 si allinea dopo l'addendum: bio, tracking e reference sono a 0. Le divergenze sono interpretative (correzioni sotto) o riguardano la sintesi (§4).

- **Vero:** al reset q e qdot di tutte le coordinate vengono dall'IK, con errore 0 e invarianti ai seed. Dall'IK derivano anche motori SEA, `served_ref` q/qdot/qddot, `previous_endpoint` e FSM. 16 feature dell'actor su 35 sono valori IK.
- **Falso che ci sia una sola posa globale:** gli start sono 3, uno fisso per runner, 4 runner ciascuno, con completamenti sincronizzati.
- **Falso che l'IK al reset sia l'unica informazione prescritta:**
  - il plant insegue l'IK biologica con la GRF destra presa da file;
  - il critic riceve `_kin_ref` a ogni passo;
  - il corridoio morfologico deriva dall'IK.
- **Non dimostrato:** che la ripetizione del reset limiti l'apprendimento.

**Correzioni al mio primo audit**
- "L'equilibrio lo garantiscono i residui" e "la curva piatta si spiega più semplicemente con lr 5e-7 e il critic" sono interpretazioni, non esiti causali. Nessun test ha misurato residui o reserve, né variato l'ottimizzatore.
- Lo stesso vale per "regime quasi fermo". Restano i fatti: lr, σ, clip, epoche e KL ~2–3·10⁻⁴, quest'ultimo solo sulle iterazioni che avevo controllato.
- "La cartella `reset_ik_audit_2026_10_12` non esiste" era inesatto e irrilevante. La cartella esisteva; andava scritto "non letta, per indipendenza".
- "Reset confermato dagli artefatti del 02/09" vale solo per il nominale, come il replay esatto attuale dei 17 STO. Il codice storico non è pinned, quindi non c'è equivalenza verificata né per gli altri start né per il training.

## 2. Impatto osservato dei nuovi test

Ambito: un checkpoint (iterazione 47), condizione AB06, inferenza deterministica, zero update, solo macOS arm64.

**Privilegi:** 24 controfattuali sul suffisso di 49 feature lasciano distribuzione e azione identiche bit a bit. L'actor non dipende direttamente dal suffisso in inferenza; resta la dipendenza indiretta tramite gli advantage in training.

| caso | return | HS/cicli | campioni morfologia *settled* | RMSE ginocchio 0–0.5 s |
|---|---|---|---|---|
| nominale 1.957 s | 27.4 | 3/2 | 376 | 0.22° |
| −0.20 s (training) | 53.9 | 4/3 | 481 | 0.37° |
| +0.20 s (training) | 38.1 | 3/2 | 403 | 0.43° |
| +0.40 s (held-out) | 12.0 | 3/2 | 410 | 1.68° |
| HS ciclo successivo (held-out) | −11.7 | 3/2 | 391 | 0.28° |
| qdot protesiche = 0 | 27.5 | 3/2 | 377 | 0.22° |

In tutti i casi: 5 s completati, 0 eventi invalidi, 0 azioni clippate, penetrazione 22–28 mm.

1. **Ordine degli start:** quello dei tre start di training riproduce lo storico. In due casi su tre, però, il return deterministico supera il massimo osservato in training (38.1 contro 35.7 a 2.157 s). Il confronto 1:1 non è lecito, anche perché il codice non è pinned.
2. **Accredito FSM:** −0.20 s è l'unico start in WAIT_HS. Ha un ciclo in più e scarta 19 campioni, contro 90–124 negli altri casi. È un indizio che parte della dipendenza dallo start sia contabilità della reward.
3. **Finestra più che posa:** "HS ciclo successivo" parte quasi uguale al nominale per q/qdot protesici e FSM. Differisce soprattutto l'accelerazione servita al ginocchio (+0.8 contro −14.7 rad/s²). Cicli, copertura e transitorio sono simili, ma il return scende a −11.7.
   - Lo scarto è più compatibile con la finestra (IK biologica, GRF, aderenza al corridoio) che con posa o copertura.
   - È un solo confronto, e senza decomposizione della reward resta non attribuibile.
4. **Transitorio:** solo +0.40 s ne mostra uno marcato (caviglia 1.04° contro 0.46°), rientrato dopo 0.5 s. Registra anche la penetrazione e la forza massime del set (27.8 mm, 1257 N).
5. **qdot protesiche = 0:** la prima azione cambia (ginocchio da 0.737 a 0.760), ma le traiettorie restano entro 0.0057 rad e il return varia di +0.03.
   - Però `served_ref` qdot/qddot e le velocità biologiche restano quelle IK.
   - Quindi il test non rimuove l'informazione di velocità IK.
6. **Cosa misura il tracking:** l'RMSE actual−served è ≤0.84° dopo 0.5 s in tutti i casi. Dice che il SEA segue il riferimento della policy, non che il cammino sia buono.

## 3. Apprendimento causale ancora non identificato

- Nessun esperimento ha variato la distribuzione di reset in training. I test frozen misurano dipendenza e generalizzazione di un checkpoint, non apprendimento.
- Nei dati storici nessuno start mostra un trend rilevabile in 50 update. Ultimi 10 meno primi 10, a 1.757/1.957/2.157 s: −0.66, +0.18, −0.02.
- Il best47 è il massimo di una serie piatta e rumorosa, compatibile con una selezione sul rumore.
- Spiegazioni concorrenti, nessuna testata:
  - copertura ristretta: 3 start fissi, runner sincronizzati;
  - regime dell'ottimizzatore: non è accertato che l'actor si sia mosso abbastanza da rendere osservabile un effetto del reset;
  - critic calibrato su un'altra configurazione: rewarm di 1 update, solo sul nominale, con pesi morfologia 1.05 contro 5.0 nel PPO. EV media 0.065 e advantage ordinati come i return sono compatibili con un critic che distingue gli start meno di quanto differiscano i return. Il dato è relativo, non un errore raw;
  - contabilità FSM e finestra biologica/GRF.

## 4. Critica della sintesi candidata

Corretti: reset IK, privilegi non diretti, scaffolding, esiti frozen e zero-qdot, causalità non dimostrata, segno relativo dell'advantage.

Da correggere:
- **"3 pose fisse per runner":** è una posa per runner, tre in totale. Lo start fissa anche la finestra IK/GRF e lo stato FSM.
- **"Senza collasso":** significa solo nessuna terminazione e nessun evento invalido. Residui e reserve non sono misurati, e la penetrazione è già 24.6 mm nel nominale.
- **"Forte start dependence":** è forte nel return, moderata nelle metriche meccaniche registrate. Il caso "HS ciclo successivo" suggerisce che conti più la finestra della posa.
- **"Sensibile alla copertura morphology":** regge per −0.20 s, non per "HS ciclo successivo" e +0.40 s. Il return è confrontabile tra checkpoint a parità di start e finestra, non tra start diversi.
- **"Learning plateau":** in questo run è più esatto "nessuna variazione rilevabile". Un plateau presuppone un apprendimento già avvenuto.
- **Confondenti da esplicitare:** regime dell'ottimizzatore, cambio dei pesi reward tra rewarm e PPO, possibile selezione del best sul rumore.
- **"A/B senza tuning simultaneo dell'ottimizzatore":** è giusto non variarlo tra i bracci. Ma nel regime attuale entrambi i bracci rischiano un risultato nullo e non informativo. Prima servono un critic comune ricalibrato e un controllo positivo.
- **Finestra 0.5 s predefinita:** accolto. Però lo scarto morfologico, verosimilmente di startup, arriva a 124 campioni (~1.24 s), oltre la finestra.

## 5. Raccomandazione operativa

1. **Produzione:** nessuna modifica al reset finché l'A/B non decide, perché non è dimostrato che sia il collo di bottiglia. Riportare i risultati per start e finestra: il solo 1.957 s delle valutazioni storiche non è rappresentativo.
2. **Strumentazione:**
   - commit e hash dei sorgenti nel summary;
   - log per start di componenti della reward, advantage raw, target e predizioni del value;
   - residui/reserve e saturazione SEA nei rollout.
3. **Test frozen economici (non causali):**
   - estendere il caso qdot = 0 azzerando anche `served_ref` vel/accel e la velocità iniziale del governor;
   - ripetere −0.20 e +0.20 s alla stessa fase sul ciclo successivo.
4. **Rewarm del critic:** comune ai due bracci, sull'unione dei loro start, con la reward del PPO (pesi morfologia 5.0) e actor congelato. Entrambi i bracci partono dallo stesso checkpoint.
5. **Controllo positivo:**
   - sul braccio A cercare un regime di ottimizzazione in cui l'apprendimento sia misurabile secondo un criterio prefissato, poi congelarlo per entrambi i bracci. Scegliere il regime su A introduce un bias conservativo contro B;
   - se nessun regime fa apprendere A, provare B con gli stessi regimi. Se neanche B apprende, il problema è altrove.
6. **A/B:**
   - braccio A: 3 start fissi per runner;
   - braccio B: 12 start, uno per runner, inclusi i 3 di A, su fasi e cicli disgiunti dagli held-out;
   - il ricampionamento per episodio va in un terzo braccio;
   - variante del contratto exact-start in tutti i bracci, stesso budget, almeno 3 seed (meglio 5).
7. **Valutazione:**
   - su held-out comuni ai bracci: return per ciclo accreditato, componenti della reward, guardrail fisici, effetti per start con intervalli tra seed;
   - adottare B solo se migliora gli held-out senza peggiorare gli start di training oltre il rumore.
8. **Windows:** verifica nativa prima di allenare su Windows.
