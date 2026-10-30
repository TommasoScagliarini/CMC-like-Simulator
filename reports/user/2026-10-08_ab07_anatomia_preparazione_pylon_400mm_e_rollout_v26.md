# AB07: anatomia, preparazione, pylon da 400 mm e rollout v26 — 2026-10-08

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


**Data del report:** 8 ottobre 2026, Europe/Rome. **Esperimenti documentati:** sessione del 7 ottobre, successiva al [report dei test preliminari del 6 ottobre](2026-10-06_test_preliminari_v26_ab07_e_confronto_controllato_ab06.md). I nomi e i timestamp originali degli audit sono conservati. **Instruction check token:** `CMC_AGENT_OK_2026`.

## Problema, risultato e decisione operativa

La differenza di carico tra AB06 e AB07 doveva essere attribuita distinguendo anatomia del soggetto, preparazione della protesi e altre cause del banco. L'indagine ha identificato **un effetto causale della registrazione del montaggio**, soprattutto dell'allungamento distale di 17,579 mm: questo passaggio inverte la differenza relativa di altezza della caviglia e aumenta il contatto virtuale. Non è stato dimostrato che l'esatto allungamento sia necessario per l'anatomia di AB07, né che sia stato introdotto intenzionalmente per alterare i risultati.

Lo screening mostra che **400 mm riducono il sovraccarico e le reserve rispetto ai 403,478 mm registrati**, nel banco attuale. La costruzione alternativa richiesta dall'utente, con ginocchio all'altezza sana e piede appoggiato geometricamente, porta invece a circa 434 mm: soddisfa i vincoli statici dichiarati, ma il suo test dinamico si interrompe dopo 0,16 s. La compatibilità fra montaggio, nuova IK, contatto virtuale e controllo rimane da isolare.

L'utente ha scelto di **mantenere provvisoriamente 400 mm**. Il successivo rollout della rete **v26C best47** su questa preparazione completa tutti i 6,11 s richiesti. Insegue bene il proprio riferimento, ma l'impulso verticale sinistro è **19,226% inferiore alla misura sana di AB07**. La calibrazione del detector è rimasta invariata in tutti questi esperimenti.

## Strategia e vincoli conservati

L'indagine è stata condotta in cartelle isolate: ricostruzione comune AB06/AB07, controfattuali di montaggio sullo stesso AB07, screening completo, nuova calibrazione statica e infine rollout neurale sul riferimento da 400 mm. Ogni montaggio sperimentale ha la propria calibrazione dei 12 marker sinistri e la propria IK completa coerente; una IK di un altro montaggio non viene presentata come preparazione equivalente.

Sono stati conservati anatomia originale, parametri e semantica dei SEA, profili di forza e detector, soglie e controllori di produzione. Nei controfattuali del pylon, masse e inerzie restano quelle del modello: non è stata ricostruita la massa di un tubo reale di lunghezza diversa. Gli offset statici del piede servono a calibrazione marker e mapping della IK; non sono modifiche dello zero articolare o della calibrazione del detector.

Nei test **CMC-like guidati dalla IK**, le pose protesiche non sono imposte: il riferimento passa attraverso governor, controllori Python e SEA dinamico non-impedance, con coppia effettiva della molla. Nel **rollout neurale**, le azioni provengono dal checkpoint congelato. In entrambi i casi il lato biologico resta muscle-driven con reserve; la forza sinistra è generata e applicata dal contatto online, mentre a destra si usano le GRF sperimentali.

## Anatomia e preparazione: che cosa è stato isolato

La ricostruzione uniforme AB06/AB07 riproduce le IK protesiche precedenti byte per byte e le forze del replay cinematico. La diversa provenienza storica delle preparazioni non spiega il divario: esso esiste già sulle pose preparate, prima di introdurre la rete. Due nuove IK sui modelli sani originali, con lo stesso solver, controllano anche la provenienza delle IK del dataset.

| Contrasto AB07 − AB06 del centro caviglia in appoggio | Valore |
|---|---:|
| Modelli sani, solver uniforme | +9,273 mm |
| Modelli protesici prima della registrazione | +9,276 mm |
| Modelli protesici dopo la registrazione corrente | −8,050 mm |
| Variazione introdotta dalla registrazione | −17,326 mm |

La differenza sana comprende anatomia, movimento e posa, quindi non misura la sola anatomia. Il fitting iniziale mantiene quel contrasto; è la registrazione successiva a invertirlo. AB07 rimane comunque più alto del proprio centro caviglia sano: l'inversione riguarda il confronto relativo con AB06.

Quattro preparazioni coerenti di AB07 separano lunghezza distale e offset statico di caviglia. Nel replay della IK filtrata, senza nuova dinamica e senza rete:

| Pylon | Offset statico | Forza verticale sinistra media |
|---|---:|---:|
| 385,899 mm | 0° | 22,240 N |
| 385,899 mm | +4,731° | 0,019 N |
| 403,478 mm | 0° | 444,728 N |
| 403,478 mm | +4,731° | 356,561 N |

L'allungamento aumenta il contatto in entrambi i confronti; l'offset statico lo attenua. L'interazione è non lineare e non consente percentuali causali additive. Queste forze sono **replay cinematico**, distinti dai valori della simulazione integrata riportati sotto.

I 17,579 mm sono calcolati per riprodurre pitch e clearance statici della mesh del donatore AB06. Sono necessari per quei bersagli, non dimostrati necessari in assoluto per AB07. Variare il bersaglio pitch di ±2° modifica l'extra richiesto a 14,215–20,994 mm; variare la quota di ±5 mm lo porta a 12,552–22,607 mm. I file esaminati non attestano una misura indipendente del pavimento rispetto ai marker. Ciò non dimostra che il piano codificato sia errato.

Fonti: [audit indipendente](../../validation/ab06_ab07_independent_attribution_2026_10_07/sintesi_finale.txt) e [verifica del montaggio](../../validation/ab07_mount_registration_verification_2026_10_07/sintesi_finale.txt).

## Screening del pylon e origine del valore 400 mm

Il controfattuale proporzionale richiesto è:

`430 mm × (417,741016 / 465,480599) = 385,899299 mm`.

430 mm è il montaggio AB06; numeratore e denominatore derivano dai segmenti sani AB07 e AB06. La proporzione è un criterio sperimentale da verificare, non una misura indipendente della protesi reale. **400 mm era uno dei punti della griglia fissata prima dei risultati**, non il risultato di questa proporzione o una lunghezza anatomica identificata.

Sono state preparate e integrate **18 condizioni**: sette lunghezze, ciascuna con offset 0° e +4,731°, tre controlli proporzionali a −2°/−5°/−10° e AB06. Quindici coprono i tre cicli primari; tre si interrompono per la soglia di penetrazione invariata di 28 mm. Per i casi incompleti non vengono inventate medie su tre cicli.

| CMC-like AB07, tre cicli primari | Forza sinistra media | Reserve verticale media | Reserve verticale RMS | Lavoro assoluto della reserve verticale |
|---|---:|---:|---:|---:|
| Corrente: 403,478 mm / +4,731° | 417,56 N | −131,76 N | 224,18 N | 33,95 J |
| 400 mm / +4,731° | 304,86 N | −19,06 N | 152,66 N | 19,49 J |
| Proporzionale: 385,899 mm / +4,731° | 13,58 N | +272,22 N | 364,93 N | 55,51 J |
| Proporzionale: 385,899 mm / −10° | 291,59 N | −5,80 N | 137,69 N | 18,05 J |

Il bilancio del moto richiede circa 285,8 N medi dal lato sinistro. Una reserve verticale positiva sostiene il bacino verso l'alto; una negativa lo tira verso il basso. A 400 mm diminuiscono forza e assistenza residua rispetto al corrente; entrambi i casi hanno tre cicli FSM validi. Rimuovere tutto l'extra mantenendo +4,731° lascia invece pochissimo carico, pur conservando tre cicli riconosciuti.

Il proporzionale/−10° sostiene il modello senza l'extra, ma ha **zero HS, TO e cicli nell'actor FSM**. Gli HS della forza arrivano circa 139/145/166 ms dopo quelli misurati, oltre il criterio originario di 120 ms. Quindi non è pronto per la rete. Questo è il caso in cui reserve inferiori e fallimento della percezione coesistono; non descrive ogni accorciamento del pylon.

Fonti: [tabella completa](../../validation/ab07_pylon_necessity_screening_2026_10_07/screening_overview.csv), [sintesi dello screening](../../validation/ab07_pylon_necessity_screening_2026_10_07/final_synthesis.txt) e [figure aggregate](../../plot/AB07_pylon_necessity_screening_2026_10_07/).

## Calibrazione con ginocchio sano e piede appoggiato

È stato applicato il criterio proposto dall'utente mantenendo le coordinate biologiche della posa statica sana originale, ricavata dalle 436 pose registrate. Il ginocchio sano è a **474,956 mm**. Il piede comune è la stessa mesh `AM_foot_l.STL`; il piano scelto è quello codificato **Y=0**. La verifica è geometrica e non dimostra equilibrio statico o pressione plantare sotto carico.

La soluzione finale a quattro regioni plantari mantiene la quota del ginocchio e ottiene tre punti non allineati a terra, nessuna penetrazione della mesh e quarto gap di 0,000845 mm, entro la tolleranza di 0,01 mm. Le dimensioni cinematiche risultano:

- `tibia_pylon`: **433,966724 mm**;
- traslazione Y dell'attacco `osseo_parent`: **−185,770409 mm**;
- orientazione X dello stesso attacco: **−3,318315°**;
- collegamento interno `osseo_pylon`→ginocchio: **205 mm**, conservato.

La traslazione dell'attacco non è la lunghezza del corpo osseo_pylon. Mesh strutturali, masse e inerzie non sono state rifabbricate o riscalate. La registrazione automatica donor è stata esclusa per preservare questi vincoli, poi sono state eseguite calibrazione marker e IK complete proprie della variante.

**La geometria passa, la dinamica si interrompe:** la variante finale supera il limite superiore del ginocchio dopo 0,16 s, a +0,144°. La soluzione intermedia a due regioni, 432,915 mm, si interrompe dopo 0,17 s. Non è un arresto per penetrazione né un timeout. La nuova variante non raggiunge i tre cicli primari: reserve e detector su quella finestra sono indisponibili, non nulli e non dichiarati falliti.

Nel prefisso comune osservato di **159 ms**, il confronto diretto è:

| Caso | Reserve verticale media | RMS | Forza sinistra media |
|---|---:|---:|---:|
| 400 mm | +194,07 N | 205,00 N | 0,98 N |
| Due regioni | −663,83 N | 690,99 N | 862,72 N |
| Quattro regioni | −712,48 N | 741,13 N | 913,78 N |

Nella variante finale il comando del ginocchio è clippato in 117/160 campioni; il clamp motore a 500 N·m compare soltanto in due. `F_opt=100 N·m` limita il riferimento `u·F_opt`, non rigidamente la coppia della molla.

Le sfere del contatto e la suola STL sono geometrie diverse: nella posa statica a quattro regioni le sfere risultano geometricamente penetrate fino a **47,38 mm**, rispetto a 7,43 mm nel riferimento 400. È una concreta incompatibilità da investigare, non la causa dinamica già isolata dell'arresto; il reset avviene su una posa di cammino. Il 400, pur completando il test, lascia nella posa statica scelta tallone e avampiede a **16,02 e 44,33 mm sopra Y=0**. Non è quindi qualificato come lunghezza anatomica dal criterio di appoggio richiesto.

Fonte: [sintesi della calibrazione](../../validation/ab07_standing_geometry_fit_2026_10_07/final_synthesis.txt), [risultati numerici](../../validation/ab07_standing_geometry_fit_2026_10_07/analysis/lean_numeric_summary.json) e [plot](../../plot/AB07_standing_geometry_fit_2026_10_07/).

## Rollout v26 best47 sul modello da 400 mm

È stato riusato il modello appena preparato da 400 mm, byte-identico al riferimento dello screening, senza una nuova IK. Checkpoint `MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best`, iterazione logica 47; SHA del `module_state.pkl`: `2d1d9b30f2b043f4b3b7b4347c06e9b3aace1c7d089b33e2893930554d84bd7c`. Inferenza deterministica, seed 123, nessun aggiornamento dei pesi. Actor a 35 feature, osservazione completa a 84.

Un solo tentativo nativo completa **611 segmenti e 6,11 s**, da 12,591740520 a 18,701740520 s, con arresto normale `episode_time_limit`. È l'intero episodio richiesto, non l'intera acquisizione di circa 143 s. I file STO registrano prima di ogni sottopasso: 6110 righe, supporto di 6,109 s; lo stato finale raggiunge il termine richiesto.

La finestra primaria prefissata **14,087704688–18,690780190 s**, durata **4,603075502 s**, comprende tre cicli sinistri misurati, da HS al successivo HS dello stesso piede. La porzione iniziale è esclusa dalle statistiche seguenti. I cicli nominali di analisi sono distinti dai cicli accettati internamente dalla FSM.

| Tre cicli primari, stesso modello e finestra | Rete v26 | CMC-like da IK |
|---|---:|---:|
| Reserve verticale media | +44,873 N | −19,063 N |
| Reserve verticale RMS | 155,131 N | 152,662 N |
| Impulso verticale della reserve, firmato | +206,554 N·s | −87,746 N·s |
| Lavoro assoluto della reserve verticale | 21,293 J | 19,495 J |
| Forza verticale sinistra media | 240,932 N | 304,858 N |
| Impulso verticale sinistro | 1109,030 N·s | 1403,285 N·s |
| RMS actual/riferimento servito: ginocchio | 0,451° | 0,300° |
| RMS actual/riferimento servito: caviglia | 0,426° | 0,393° |

La misura sana AB07 sugli stessi tempi è **1373,010 N·s**. La percentuale usa `100 × (I_online / I_misurato − 1)` e vale **−19,226%** per la rete; non è una percentuale rispetto ad AB06 o al CMC-like. La reserve RMS è vicina al controllo, ma la media cambia segno, quindi il sostegno medio non è uguale.

La differenza RMS dalla IK originale non filtrata di AB07 è **9,664° al ginocchio e 6,156° alla caviglia**. È distinta dall'errore di inseguimento del riferimento della rete. Una policy ex-novo non ha automaticamente la fedeltà alla IK come obiettivo di imitazione.

Il confronto conserva modello, IK, profili e configurazione fisica, ma mantiene il contratto proprio del checkpoint: sorgente/mapping delle azioni, slew, clock e criteri dell'actor FSM differiscono dal controllo da IK. **Non è un esperimento che isola soltanto i pesi della rete.** Dettagli nel [confronto dei contratti — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-7970d6a0b5ca9305>).

### Detector e limiti effettivamente osservati

Nell'intero episodio neurale: **4 HS, 4 TO, 3 cicli validi**, zero eventi invalidi, risincronizzazioni, cancellazioni o revoche. Calibrazione invariata. Sulla finestra primaria:

| Clock/evento del detector geometrico | HS associati | TO associati | Tolleranze HS / TO |
|---|---:|---:|---:|
| Onset | 4/4 | 3/3 | 120 / 180 ms |
| Confermato | 2/4 | 2/3 | 50 / 80 ms |
| Consegnato alla rete | 4/4 | 2/3 | 60 / 90 ms |

Per gli onset, bias medio HS **−27,214 ms**, TO **+14,463 ms**; errore medio assoluto **33,249 e 64,171 ms**. Il bias può cancellare errori di segno opposto. L'HS finale è osservato, ma il suo margine temporale successivo completo è censurato: un mancato abbinamento lì non è automaticamente un evento mancato. Tre cicli validi non certificano tutti i clock.

Gli attraversamenti della forza online a 20 N sono un altro canale: tre HS associati su quattro, un extra e HS finale censurato; tre TO associati, un extra. Non vanno confusi con gli eventi del detector geometrico.

Nessun clipping dell'azione della rete, del comando SEA o clamp della coppia motore a 500 N·m. L'integratore della cascata al ginocchio raggiunge invece il proprio limite per **388 campioni, 0,388 s, 6,35%**: è un limite distinto. Penetrazione massima sinistra **22,356 mm**, sotto la guardia di 28 mm; nei tre cicli primari **17,833 mm**.

Fonti: [sintesi del rollout — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-cef5f7cde5b9030c>), [metriche — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-7ca6be26832954af>), [eventi — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-5ddc06c9ce190796>).

## Collaborazione indipendente con Claude

Gli audit richiesti sono stati svolti tramite comando `claude` da terminale, modello effettivo **`claude-fable-5-1`**, effort **max**, chat isolate e strumenti disabilitati. Sono stati inviati esclusivamente capsule numeriche e risultati aggregati nel consenso limitato dell'utente, senza sorgenti, credenziali o accesso ai file. Negli audit indipendenti richiesti, le conclusioni locali Codex sono state fissate prima di leggere quelle esterne; successivamente sono stati confrontati i risultati e inviati supplementi numerici.

Claude ha verificato gli aggregati, **non ha rieseguito OpenSim**. Il consenso riguarda origine procedurale dell'extra, miglioramento a 400 mm, limiti della soluzione proporzionale e arresto precoce della variante statica. Non sono state adottate come dimostrate le ipotesi di ulteriori 9 mm anatomicamente necessari, pavimento necessariamente errato, pitch raw obbligatorio o limite SEA già identificato come causa. La forza statica delle sfere e la telemetria SEA hanno corretto inferenze iniziali; i testi visibili e le divergenze rimangono conservati negli audit. L'ultimo rollout neurale è stato revisionato da agenti interni; non è attribuito a una nuova esecuzione di Claude.

## File aggiunti e risultati conservati

| Perimetro | Contenuto principale |
|---|---|
| `validation/ab06_ab07_causal_investigation_2026_10_07/` e `ab06_ab07_preparation_vs_model_2026_10_07/` | Diagnosi iniziali, controfattuali e capsule numeriche |
| `validation/ab06_ab07_independent_attribution_2026_10_07/` | Rebuild comuni, IK sane, confronto indipendente, `sintesi_finale.txt` |
| `validation/ab07_mount_registration_verification_2026_10_07/` | Quattro preparazioni, attribuzione delle quote, sensibilità dei bersagli e integrità |
| `validation/ab07_pylon_necessity_screening_2026_10_07/` | Protocollo, helper di esecuzione, 18 preparazioni/run, metriche e fase aggregate |
| `validation/ab07_standing_geometry_fit_2026_10_07/` | Fit statico, modelli/IK coerenti, due varianti nuove, controllo 400 e analisi |
| `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/` | `run_rollout.py`, `prepare_execution.py`, `postprocess_rollout.py`, `setup.xml`, manifest, metriche e review |
| [Run neurale](<../../Trajectory Generator/runs/rollout/MLP_V26C_best47_AB07_pylon400_2026_10_07_6p11s/>) | Output nativi, policy trace, eventi e diagnostiche |
| [Sette plot del rollout](../../plot/MLP_V26C_best47_AB07_pylon400_2026_10_07_6p11s/) | Figure canoniche nel root, tutte ispezionate |

Lo screening conserva **126 plot canonici**, il confronto standing **21** e il rollout neurale **7**, nelle rispettive cartelle `plot/`. Gli audit mantengono anche figure aggregate e ricevute delle fasi tecniche precedenti. Nessun sorgente di produzione, plugin C++, checkpoint o profilo operativo è stato modificato da questi esperimenti. La scelta dei 400 mm si riferisce alla variante isolata; non è dichiarata una sovrascrittura del bundle canonico AB07.

## Test e verifiche concluse

- Ricostruzione dei modelli/IK, coerenza marker–modello–cinematica, confronti FK e preservazione degli input negli audit di attribuzione.
- Screening dinamico di 18 condizioni, finitezza dei dati, bilancio verticale, identità SEA e trattamento esplicito dei tre run interrotti. Il riferimento AB07 corrente riproduce esattamente il prefisso comune dei 17 STO storici a 9 s, di 6110 campioni; i file completi hanno durata e hash diversi. Il controllo AB06 dello screening a 6,21 s mantiene un piccolo scostamento terminale già documentato, senza effetto sul contrasto di montaggio.
- Standing: verifica geometrica delle quattro regioni, comparazione sul solo prefisso comune osservato e riproduzione byte-identica del controllo 400, senza promuovere le varianti interrotte a completamenti.
- Rollout: 130 input protetti invariati, 17 STO × 6110 righe finite, 26 artefatti nativi invariati. Calcolo indipendente di 10.945 scalari su 231 combinazioni coordinata/finestra con corrispondenza esatta sulla quadratura comune; controllo diretto di media e RMS della reserve verticale.
- Sette PNG neurali ispezionati, peer review numerica PASS e [ricevuta finale — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-a9b821f2320e46a6>).

La finestra primaria era fissata prima dell'esecuzione. Il protocollo analitico neurale dettagliato è stato sigillato dopo l'avvio del worker, prima di leggere gli esiti: la [qualifica temporale — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-866744d29e66bfb1>) conserva questa distinzione. Correzioni dei reader, preflight e review sono tracciate e non equivalgono a retry dinamici selettivi. Le prove native documentate sono macOS arm64; non è stata svolta una nuova qualifica live Windows.

Il precedente paired AB06 a 9 s conserva la receipt `failed` per l'ultimo callback del detector, anche se stato, runner e ambiente raggiungono l'horizon fisico. Il callback finale non produce tutti gli output di trace/reward/osservazione/eventi attesi. I tre cicli precedenti usati nell'audit restano osservati; il run non viene promosso a completato. Questo debito è distinto dallo scostamento terminale del controllo AB06 dello screening a 6,21 s e rimane in V26-T01.

## Visualizer in loop

È stato fornito e verificato il comando seguente; `--mlp` seleziona il rollout valido più recente e applica il suo setup. Durante la verifica selezionava il run v26/400 mm indicato sopra. Nessuna GUI è stata avviata dall'agente.

```bash
cd "/Users/tommy/Documents/CMC-like-Simulator - Claude"
/opt/anaconda3/envs/envCMC-rllib/bin/python -B visualize.py --mlp --loop
```

## TODO e limiti ancora aperti

Le voci seguenti non autorizzano nuovi lanci. I 400 mm restano una scelta provvisoria del banco, non una dimensione fisica identificata per AB07. L'anatomia originale deve essere conservata; il metodo di preparazione deve gestire soggetti diversi con criteri comuni e verificabili.

- [ ] AB07-P01 — Isolare l'interazione montaggio–mapping marker/IK–contatto–controllo che interrompe la variante con appoggio statico, distinguendo il confronto meccanico a cinematica comune dalla preparazione coerente di ogni modello; mantenere fissa la calibrazione del detector e non aumentare i limiti SEA come sostituto della diagnosi.
- [ ] AB07-P02 — Definire vincoli indipendenti e compatibili su dispositivo, allineamento, appoggio e piano di acquisizione per dimensionare la protesi con lo stesso criterio sui soggetti; verificare coerenza fra suola STL, sfere di forza e punti del detector prima di qualificare una lunghezza fisica.
- [ ] V26-T01 — Correggere e validare separatamente la guardia dei confini campionati del detector binario esposta dall'ultimo callback AB06, mantenendo il rifiuto dei batch realmente non validi e verificando casi di rounding/horizon su macOS e Windows; conservare receipt e trace failed del test originale.
- [ ] V26-T02 — Predisporre ed eseguire, quando richiesto, un confronto learned AB06/AB07 a checkpoint e contratto policy congelati, stessi criteri HS/startup/tre cicli nominali e durate comuni; la coppia IK-driven corrente non è quel test della rete.
- [ ] V26-T03 — Estendere la qualifica del trasferimento con start/condizioni e repliche dichiarati, misurando separatamente carico, HS/TO, knee clipping, margine di penetrazione, tracking, MTP e reserve; preservare best47 e tenere distinta la qualifica dai futuri training.

AB07-Q01/Q02 e G09/G10 hanno nuovi avanzamenti, ma la qualifica scientifica complessiva di contatto, carico e tempi resta aperta. AB07-Q03 conserva il debito MTP; AB07-Q04/G25 quello Windows. G12/G13 avanzano nell'ambito esplorativo, senza completare la campagna multi-modello. BW/preprocessing, AB08 e i debiti 39D/PPO restano invariati; il bug bootstrap PPO resta rinviato dall'utente.

Il [daily parziale dell'8 ottobre](../daily/2026-10-08_daily-report.md) conserva le **270 voci aperte** del [daily del 6 ottobre](../daily/2026-10-06_daily-report.md) e aggiunge AB07-P01/P02, senza dichiarare conclusi i TODO precedenti. La scrittura di questo report comporta soltanto consolidamento documentale e propagazione del backlog, non nuove simulazioni, preparazioni o training.
