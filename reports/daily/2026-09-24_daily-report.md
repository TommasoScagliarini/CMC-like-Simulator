# Daily Report — 2026-09-24

Chiusura della giornata su richiesta `end_day`. Questo documento consolida
gli otto report utente del 24 settembre e sostituisce gli aggiornamenti
parziali con lo stato finale. Gli artefatti e i report di origine restano
conservati; gli stati «in esecuzione» riportati nei documenti di avvio sono
storici e sono risolti dagli esiti qui sotto.

## Stato alla chiusura e punto di ripartenza

**Best80 LEFT con peso penetrazione 2 resta il candidato preferibile nel
confronto nominale.** La prosecuzione autorizzata da best80 a 100 è conclusa;
il nuovo best82 e last100 non migliorano né il tracking della caviglia né
l'integrale della sua coppia positiva. La robustezza non è risolta.

I due training LEFT eseguiti oggi sono entrambi conclusi. Sono stati prodotti
quattro rollout deterministici e 28 plot canonici LEFT in root `plot/`.
Non è stato avviato un ulteriore training, l'oscuramento o l'ex-novo.

La priorità discussa resta capire e migliorare LEFT. RIGHT ha fornito
evidenza che l'architettura 39D può apprendere un'imitazione qualitativamente
vicina a V26; questo non dimostra equivalenza statistica o identifica una
causa unica del divario LEFT. Il training RIGHT rimane fermo alla 60.
Lo screening aggiuntivo RIGHT last60 è pendente e subordinato a un eventuale
riuso di quel ramo, non è un prerequisito per proseguire l'indagine LEFT.

La prossima decisione deve distinguere robustezza con policy congelata e
qualità delle coppie. Sono ancora aperti lo screening rumoroso best80,
il controllo contemporaneo peso 1 e un eventuale intervento mirato alla
coppia knee/contatto ankle. La conclusione della giornata non autorizza
l'esecuzione automatica di queste attività.

## Report del giorno

| Report | Contenuto e stato finale |
|---|---|
| [RIGHT39 fino60 e robustezza](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md) | Consolidamento del training concluso il 23: best58, last60 e cinque seed sul best58; niente continuazione oltre 60 |
| [Audit reward LEFT](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md) | Scomposizione per fase, clipping, due meccanismi di fallimento, candidato peso penetrazione 2 |
| [Fix del contatto terminale hard](../user/2026-09-24_fix_contatto_terminale_hard_unified39.md) | Misura fisica conservata nell'eccezione, reward e telemetria coerenti; test e smoke superati |
| [Avvio LEFT72 peso2](../user/2026-09-24_avvio_left72_penetrazione_peso2_10iter.md) | Override esplicito del solo peso e restore completo; training 73–82 poi concluso |
| [Risultati LEFT72 peso2](../user/2026-09-24_risultati_left72_penetrazione_peso2_10iter.md) | Best80/last82, completamento 62,8%, quattro hard contabilizzati correttamente, primi 14 plot |
| [Cause delle coppie best80](../user/2026-09-24_best80_cause_coppie_ankle_knee.md) | Contatto/COP e compensazione ankle, componente rapida del comando knee, prova statica |
| [Avvio best80 fino100](../user/2026-09-24_avvio_left_best80_peso2_fino100.md) | Ripresa 81–100 senza cambiare l'obiettivo; restore e hash verificati |
| [Risultati best80 fino100](../user/2026-09-24_risultati_left_best80_fino100_e_plot_canonici.md) | Nuovo best82 e last100 inferiori al best80 nominale; altri 14 plot, analisi e verifiche finali |

## RIGHT39: conclusioni consolidate

Il report di oggi raccoglie prove concluse il 23 settembre, non nuovi
training del 24. La continuazione 41–60 ha raccolto 81.920 passi, per 245.760
complessivi;153/157 episodi completi (97,5%),37/40 nelle ultime 5 iterazioni
(92,5%). I quattro fallimenti sono penetrazioni ordinarie. A esperienza
simile V26 era alla 53, con 245.132 transizioni. Il confronto degli storici
è distinto dal confronto qualitativo con V26 best87, più maturo.

Best di training 58, ritorno 251,974. Last60 è migliore nel deterministico:
RMSE knee 7,066° e ankle 2,414°, contro 7,223°/3,118° della 58 e 6,906°/1,459°
di V26 sul protocollo di quel report. Tutti completano 500 passi e 2 cicli.
La flessione massima knee della 60 è 51,44°, inferiore a 64,32° del target:
RMSE vicino non significa forma equivalente. La coppia positiva ankle
raggiunge 25,89 Nm nella 60 e 31,85 Nm nella 58, contro 10,19 Nm V26.

Best58 completa 4/5 seed 123–127;3/5 hanno almeno due cicli validi. Fallisce
il seed 125 al passo 300 per penetrazione hard, mentre best35 e V26 lo
completano. Last40 falliva lo stesso seed al passo 301. Confronti cinematici
sul prefisso comune di 299 passi, escludendo il terminale incompleto;
stesse estrazioni gaussiane standard, ma sigma appresa propria di ciascuna
policy. Cinque prove non stimano con precisione una probabilità generale
né dimostrano equivalenza statistica.

Nel transitorio critico la reference ankle best58 cresce troppo rapidamente;
il successivo errore d'inseguimento può compensarla parzialmente nel solo
RMSE actual/target, nascondendo il ridotto margine di contatto. Non emergono
resync o eventi FSM invalidi che spieghino quel fallimento. Il ritardo
ottimale descrittivo 20/30 ms spiega soltanto parte dell'errore nominale e
non misura causalmente il ritardo del filtro.

Verificati restore actor/critic/Adam/KL, target RIGHT coincidenti con V26 sui
500 campioni, sei ingressi visibili, reward e contatori. Il report conserva
561 voci di manifest verificate e 14 PNG canonici del 23. Nessuna scelta di
RIGHT last60 per l'oscuramento è stata effettuata.

## Audit LEFT: incentivi, normalizzazione e limiti

Analizzati 20 episodi archiviati,9.738 righe, e i primi 25 aggiornamenti
LEFT/RIGHT a 4096 transizioni per iterazione. La reward effettiva del wrapper
è stata ricostruita con la configurazione di ciascun rollout: non sono
stati confusi gli score preliminari dell'ambiente con quelli usati da PPO.

La normalizzazione concordata per escursioni è presente. Lo score combina
80% reference servita e 20% movimento effettivo; posizione ankle con peso
relativo 2, velocità knee/ankle 0,02/0,04. Nel regime storico peso 1:

```text
S = 0.8 / (1 + 8 L_served) + 0.2 / (1 + 4 L_actual)
r = clip(S - costi_base, 0, 1)
    - 2 safety - 1 penetration - 0.1 ankle_moment_flip - 2 out_of_band
```

Il clipping è assente nei deterministici maturi LEFT40/60/72/80/100 esaminati;
compare nelle prime iterazioni ma è maggiore in RIGHT, che apprende meglio.
Non è quindi una spiegazione selettiva sufficiente del divario LEFT.
In LEFT72 il 94,2% del costo command-rate deriva dalla distanza fra endpoint
grezzo e reference filtrata all'inizio del segmento, non dalla differenza
tra due azioni grezze consecutive. I cap interni sono distinti dal clipping
esterno. Ridurre offline questo peso favorisce anche la policy imperfetta:
non dimostra che il successivo training migliori.

Due meccanismi fisici distinti: seed 125 con momento GRF invertito già molto
penalizzato; seed 127 con penetrazione crescente e momento positivo, quindi
senza anti-flip nella salita finale. La penalità penetrazione è nulla sotto
20 mm e cresce tardi rispetto ad alcuni collassi. Otto modifiche candidate
sono state valutate soltanto sui trace congelati. Il peso 1→2 separa meglio
le finestre pre-fallimento del seed 127 dal nominale, motivando il successivo
esperimento; non anticipa la soglia e non prova una soluzione.

Il comando diretto LEFT riceve più reward della policy LEFT72, ma conserva
ritardo e penetrazione 22,79 mm: non è una policy robusta pronta all'uso.
Gli score su target diversi e i ritorni con reward diverse non sono criteri
comuni di confronto. Gamma/GAE coincidenti con RIGHT/V26 non escludono
problemi di assegnazione del merito; non definiscono un orizzonte rigido.

## Correzioni software completate

### Contatto nella terminazione hard

Prima del fix l'eccezione conservava solo il messaggio: la safety restava
attiva, ma i termini di contatto erano azzerati. Il runner conserva ora
misura reale, lato, soglia, tempo fisico e copia del contatto. L'ambiente
recupera la penetrazione e il momento quando la geometria ha un timestamp
coerente; non inventa il contatto mancante.

La terminazione resta hard con safety una sola volta e senza bootstrap.
`segment_completed=False` impedisce di presentare alla FSM un segmento
interrotto come completo; tempo logico e contatore non avanzano. Trace e
telemetria conservano il tempo fisico separato ed escludono dagli RMSE il
campione cinematico incompleto. Il comportamento legacy senza payload resta
esplicitamente indisponibile. Soglie 20/28/30 mm, FSMv 3, SEA e formule dei
preset non sono cambiati.

Con 31 mm il test dà loss 1,890625; i due pesi producono costi 1,890625 e 3,78125
sullo stesso contatto. Passati 24 test mirati e 15 preflight RLlib. Smoke
OpenSim nominale prima/dopo esattamente uguale su cinque segmenti per braccio
con istanze nuove; smoke hard con soglie abbassate soltanto nel test su
contatto reale, reset e tre segmenti successivi senza residui terminali.

Rilevato separatamente un problema preesistente: `ProstheticPhaseFSM.reset()`
può conservare gli ultimi angoli nei diagnostici di escursione. Nel campione
osservato reward, osservazioni e contatti coincidevano; differivano i
diagnostici. Nessuna modifica FSM eseguita, impatto generale da quantificare.

### Override controllato del peso durante il resume

Aggiunta `--resume-penetration-weight-override`: modifica soltanto
`grf_penetration_weight` in imitazione, in una cartella distinta, persistendo
configurazione e provenienza. Actor, critic, Adam e KL sono conservati;
nessun rewarm automatico del critic. Il critic inizialmente stima ancora il
vecchio obiettivo e si adatta durante PPO. Il resume ordinario mantiene il
controllo della reward; il ramo modificato si riapre senza ripetere l'override.

Verificati i wrapper effettivi dei 13 worker; eliminata dal logger l'eredità
delle statistiche di ritorno del vecchio obiettivo, mantenendo i contatori
di esperienza. Passati 25 test e uno smoke PPO reale da 128 transizioni,
13 worker, minibatch 64, una epoca. Riapertura in processo nuovo con zero
aggiornamenti riuscita. Audit del training effettivo PASS; Windows live
resta da verificare. Il ramo di controllo peso 1 è configurato, ma non eseguito.

## Training LEFT eseguiti oggi

| Ramo | Esecuzione CEST | Aggiornamenti | Nuove transizioni | Checkpoint |
|---|---|---:|---:|---|
| LEFT72 →82, peso 2 | 11:36:33–13:02:14 | 10, iter 73–82 | 40.960 | best80, last82 |
| Best80 →100, peso 2 invariato | 17:23:31–19:57:15 | 20, iter 81–100 | 81.920 | nuovo best82, last100 |

Sono 122.880 transizioni nuove complessivamente raccolte nei due run, esclusi
gli smoke; parte appartiene a rami diversi dopo la 80. Non vanno presentate
come esperienza di un'unica sequenza continua fino 100. Il nuovo best82
proviene dalla ripresa di best80 ed è distinto dal precedente last82.

Configurazione conservata: prescribed LEFT guidato dalla fase LEFT senza
shift, target e clock visibili 100%, divisore velocità 1, slew disattivato,
governor 4 Hz, FSMv 3, start offset 1 s (tempo fisico 12,99 s), episodio 5 s,
batch 4096, minibatch 512,10 epoche, LR 1e-4,13 worker. La seconda continuazione
non cambia nuovamente la reward: penetrazione 2, anti-flip 0,1.

Nel primo restore Adam aveva 12 stati e step 3807; nella ripresa best80
step 4455, KL 0,3000000119. Ripristino esatto verificato, checkpoint sorgenti
conservati. I batch sono completi e tutti visibili; nel secondo ramo actor
medio e critic cambiano in tutti 20 gli aggiornamenti.

### Completamento e telemetria

| Finestra | Completi / terminati | Percentuale | Fallimenti |
|---|---:|---:|---|
| Primo ramo 73–82 | 54/86 | 62,8% | 28 penetrazioni ordinarie,4 hard |
| Primo ramo, ultime 5 | 27/47 | 57,4% | 20 penetrazioni |
| Secondo ramo 81–100 | 105/179 | 58,7% | 69 penetrazioni ordinarie,2 hard,2 divergenze knee,1 evento invalido |
| Secondo ramo, ultime 5 | 26/49 | 53,1% | 21 penetrazioni ordinarie,2 hard |

Nel primo ramo tutti 32 fallimenti sono in stance. I quattro hard conservano
misure 30,145–30,226 mm e loss 1,608–1,634 prima del peso: fix funzionante anche
in training. Il vantaggio su peso 1 storico (54,4% globale,58,5% nelle ultime 5)
è descrittivo; quello storico non aveva lo stesso fix e lo stesso resume.
Cinquanta dei 54 completi superano almeno una volta 20 mm. La soglia soft
precede mediamente di circa 0,108 s la terminazione nei fallimenti osservati.

Nel secondo ramo 71/74 fallimenti riguardano la penetrazione. Completamento
per blocchi di cinque iterazioni:66,7%,58,3%,58,1%,53,1%. Dei 105 completi,
91 superano 20 mm;10 hanno zero cicli validi,36 uno,42 due,16 tre,1 quattro.
Completamento temporale, qualità del contatto e cicli validi restano criteri
distinti. La soglia soft 20 mm non coincide con la soglia di arresto.

83/86 episodi del primo ramo e 174/179 del secondo attraversano aggiornamenti
della policy. Queste percentuali non misurano la robustezza dei checkpoint
congelati. I 39.131/79.731 passi negli episodi conclusi non esauriscono i
40.960/81.920 passi dei batch: i frammenti ai confini della finestra di
raccolta rendono diversi i denominatori. Nessun record invalido o duplicato;
contatori telemetrici e trainer coerenti.

## Rollout finali e scelta del candidato

Best80 e precedente last82 completano 500 passi e 2 cicli validi, con picchi
penetrazione 14,795/18,178 mm. Anche nuovo best82 e last100 completano 500 passi
e 2 cicli. Il confronto finale usa stessa reward peso 2 e stessa finestra
fisica 13,15–17,95 s per RMSE e coppie; ritorni e picchi di penetrazione sono
sull'intero episodio. RMSE riferito al giunto fisico rispetto al target LEFT.

| Metrica | Best80 sorgente | Nuovo best82 | Last100 |
|---|---:|---:|---:|
| RMSE knee [°] | **6,38** | 6,63 | 6,74 |
| RMSE ankle [°] | **4,04** | 5,51 | 5,73 |
| Picco positivo ankle [Nm] | 29,17 | 30,12 | 29,16 |
| Integrale positivo ankle [Nm·s] | **8,61** | 16,74 | 16,15 |
| RMS knee, passa-alto 10 Hz [Nm] | 8,20 | **7,81** | 9,20 |
| RMS knee, passa-alto 20 Hz [Nm] | 3,78 | 3,45 | **3,40** |
| Penetrazione massima [mm] | 14,80 | 15,94 | 14,80 |
| Ritorno deterministico | **305,06** | 256,36 | 259,15 |

Il nuovo best82 è selezionato soltanto fra gli aggiornamenti del nuovo ramo;
il suo massimo ritorno medio di training 255,13 è inferiore a 261,75 riportato
per best80, su popolazioni di training differenti. Non è un vincitore
verificato sul checkpoint sorgente. L'errore ankle cresce 36,3%/41,8%,
l'integrale positivo 94,5%/87,6%. Il picco quasi costante nasconde coppie
positive più persistenti. Il miglioramento sopra 20 Hz non elimina il
peggioramento knee sopra 10 Hz nella 100.

L'indicazione è conservare best80 per la qualità nominale. Il controllo
rumoroso con checkpoint congelati resta aperto: non è stata dimostrata una
superiorità statistica nella robustezza. Non estendere automaticamente lo
stesso training invariato e non assumere che l'ex-novo corregga le coppie.

## Diagnosi delle coppie e implicazioni

### Caviglia

Best80: a 17,474 s coppia SEA+29,17 Nm, comando +29,21, P+2,59 e I+26,62;
momento GRF vicino−28,44 Nm, carico 460,28 N, penetrazione 12,66 mm.
L'integrale sostiene la coppia necessaria a compensare il momento esterno;
non è al clamp. Correlazione sotto carico tra coppia e opposto del momento
0,9984, residuo RMS 0,91 Nm. Nessun nuovo errore di segno o saturazione è
emerso; un integrale grande da solo non dimostra windup.

La ricostruzione statica a 17,47 s riproduce il contatto registrato entro
0,000023 N sulla forza. Con altre coordinate e velocità generalizzate fisse:

| Intervento locale | Momento GRF ankle [Nm] | Penetrazione [mm] |
|---|---:|---:|
| Posa registrata | −28,84 | 12,22 |
| Solo ankle al target LEFT | −6,09 | 11,27 |
| Solo knee al target LEFT | −21,26 | 10,24 |
| Entrambi al target LEFT | +3,08 | 12,47 |

Quindi imitazione e momento favorevole possono migliorare insieme localmente;
non è dimostrato un conflitto necessario del target LEFT con il contatto.
La prova non è una traiettoria dinamicamente eseguibile e non certifica la
validità biomeccanica dell'ambiente. Il fenomeno è coerente con i report
storici su posa, COP e braccio del contatto; compare anche in RIGHT.

Il nuovo best82 conserva questo meccanismo: picco+30,12 Nm, comando +30,33,
P+1,54/I+28,79, momento GRF vicino−29,16; correlazione 0,9987. Il SEA segue
la reference: RMSE joint/served ankle 0,34°/0,44°/0,45° per 80/82/100,
molto meno dell'errore rispetto al prescribed. Questo localizza gran parte
dello scostamento nella reference servita, senza isolare policy e filtro.

### Ginocchio e reward

La cascata usa `qdot_cmd = qdot_ref + Kp_outer * errore_posizione` e
`tau_ref = Kp_inner * errore_velocità + I`. Con Kp_inner 29,2, l'errore rapido
best80 di 0,242 rad/s RMS sopra 10 Hz produce circa 7,07 Nm sul termine P.
La posizione servita ha soltanto 0,118° RMS rapido: un grafico cinematico
apparentemente liscio può nascondere oscillazioni notevoli di velocità/coppia.
Il comando è già oscillante prima della molla; feedback e contatto restano
accoppiati, senza isolamento completo della perturbazione iniziale.

Best80 ha 8,20 Nm RMS sopra 10 Hz contro 4,60 V26, ma 3,78 sopra 20 Hz contro 3,28.
Il comando diretto LEFT storico produce 0,48/0,15 Nm, pur avendo altri difetti.
I passa-alto sono strumenti di analisi, non filtri introdotti nel controller.

Bilancio best80: score imitativo 385,77, command-rate 75,58 (70,99 da
segment_delta, circa 94%), effort 0,258, errore coppia SEA 0,00162, altri termini
motore 0,00438, anti-flip 4,69 e penetrazione 0. I pesi dedicati al rateo e
all'intensità della coppia elastica sono zero; la variazione u resta però
presente dentro command-rate (0,435 punti). Non è corretto dire che manca
ogni penalità di regolarità. Non si richiede una curva fisiologica di coppia.

Penetrazione nominale sotto 20 mm in 80/nuovo 82/100: costo nullo, mentre
anti-flip cresce 4,69→15,15/10,14. Aumentare ancora il solo peso penetrazione
non colpisce direttamente questi burst. La reward rileva il momento
sfavorevole ma non impone un vincolo assoluto. Le somme non sono gradienti
PPO né provano che la policy sia l'ottimo dell'obiettivo.

Le correzioni storiche e l'ex-novo non vanno trasferiti come garanzie:
jerk knee 2000 ridusse il chattering ma peggiorò tracking/contatto; la
regolarità ex-novo storica dipendeva dall'intera catena e dal prior.
Un futuro esperimento deve separare regolarità knee e momento ankle sotto
carico, controllando anche eventuali strategie di scarico del piede.

## File modificati, artefatti e verifiche

| Perimetro | Modifiche/artefatti del giorno |
|---|---|
| `simulation_runner.py` | Payload diagnostico dell'eccezione hard; nessuna modifica alla semantica SEA |
| `Trajectory Generator/osim_trj_cmc_like.py` | Recupero del contatto, tempo fisico, segmento incompleto e priorità hard |
| `Trajectory Generator/unified_MLP_39/rollout_eval.py`, `episode_telemetry.py`, `validation/analyze_episode_telemetry.py` | Trace e contatori coerenti con segmenti incompleti |
| `unified_MLP_39/reward_resume.py`, `training_runtime.py`, `train_ppo_mlp.py`, `training_config.py` | Override circoscritto e persistente con audit del resume |
| `unified_MLP_39/tests/` | Test payload hard, contatto/reward, telemetria e `test_reward_resume.py` |
| `unified_MLP_39/configs/` | `left72_penetration_weight1_10iter_2026-09-24.yaml`, variante peso 2 e `left80_penetration_weight2_to100_2026-09-24.yaml` |
| `unified_MLP_39/runs/plans/` | Piani dei due training e valutazioni, launcher, manifest, startup audit, log |
| `reports/user/` e questo daily | Otto resoconti del giorno, collegamenti e TODO riconciliati |

Le abbreviazioni `unified_MLP_39/` indicano sempre la sottocartella di
`Trajectory Generator/`. Il plugin C++, la FSM e i controller non sono
stati modificati. Il cambiamento root riguarda il trasporto diagnostico del
contatto, non logica di rete. Gli artefatti storici rimangono conservati.

Analisi e verifiche sotto `Trajectory Generator/unified_MLP_39/validation/`:

- `left_reward_audit_2026-09-24/`: score per fase, contatto, clipping, otto
  varianti offline e manifest degli input.
- `hard_guard_contact_2026-09-24/`: test, smoke live, copie prima del fix,
  `nominal_mismatch.json` e revisione del reset FSM.
- `penetration_resume_2026-09-24/`: test del resume e log.
- `left72_weight2_results_2026-09-24/`: contatori e telemetria del primo ramo.
- `best80_torque_audit_2026-09-24/`: analisi di sei rollout, probe statico,
  metriche, hash e figura diagnostica.
- `left80_to100_results_2026-09-24/`: analisi training, cicli, telemetria,
  confronto rollout, `metrics.json`, `input_hashes.json`, `verification.json`.
- `right_reference_results60_2026-09-23/`: fonti del resoconto RIGHT consolidato.

Gli audit offline verificano ritorni ricostruiti (entro 1e-8 nell'audit reward,
entro 1e-5 nel confronto coppie), costo comandi entro 1e-8, identità P+I entro
1e-6 Nm, uniformità dei campioni e integrità degli input. Gli ultimi 14 PNG
sono leggibili, senza canali mancanti, con ricevute coerenti; controllate
visivamente le figure 1 e 7 di entrambi i checkpoint. I 28 input dell'ultima
analisi e i 28 file sorgente best80 mantengono gli hash. Queste verifiche
sono state eseguite nelle rispettive attività; la chiusura `end_day` esegue
soltanto consolidamento documentale e controllo dei collegamenti.

## Plot da ritrovare

| Checkpoint / analisi | Cartella in root |
|---|---|
| LEFT best80 del ramo da 72 | [09_24_2026_unified39_LEFT_pen2_best80](../../plot/09_24_2026_unified39_LEFT_pen2_best80/) |
| LEFT precedente last82 del ramo da 72 | [09_24_2026_unified39_LEFT_pen2_last82](../../plot/09_24_2026_unified39_LEFT_pen2_last82/) |
| LEFT nuovo best82 del ramo da 80 | [09_24_2026_unified39_LEFT_pen2_from80_best82](../../plot/09_24_2026_unified39_LEFT_pen2_from80_best82/) |
| LEFT last100 del ramo da 80 | [09_24_2026_unified39_LEFT_pen2_from80_last100](../../plot/09_24_2026_unified39_LEFT_pen2_from80_last100/) |
| Reward LEFT | `plot/09_24_2026_left_reward_audit/`: percorso citato nel report, assente al controllo di chiusura |
| Coppie best80 | `plot/09_24_2026_best80_torque_diagnosis/`: percorso citato nel report, assente al controllo di chiusura |
| Contatto/controller nuovo best82 | [09_24_2026_LEFT_from80_to100_diagnosis](../../plot/09_24_2026_LEFT_from80_to100_diagnosis/) |
| RIGHT best58, già prodotto il 23 | [09_23_2026_39D_right_best58](../../plot/09_23_2026_39D_right_best58/) |
| RIGHT last60, già prodotto il 23 | [09_23_2026_39D_right_last60](../../plot/09_23_2026_39D_right_last60/) |

Ogni set canonico contiene sette PNG. La figura 7 LEFT usa il target di fase
sinistro registrato per confronto/errore; la gamba destra nella riga centrale
è soltanto contesto. I nomi `from80` distinguono i due checkpoint 82.

Il controllo di chiusura trova tutti i set canonici collegati sopra e la
diagnostica del nuovo best82. Le due cartelle diagnostiche indicate come
assenti non sono state trovate nemmeno cercando i relativi PNG sotto `plot/`.
La causa non è stata determinata; script, metriche e manifest delle due
analisi rimangono nelle rispettive directory di validazione. Il daily
conserva i percorsi storici senza presentarli come collegamenti disponibili
e senza rigenerare file che potrebbero essere stati rimossi intenzionalmente.

## TODO riconciliati dai report del giorno

### Completati

- [x] Recuperare e verificare la misura hard con percorso comune ai pesi 1/2;
  verificata anche nei quattro fallimenti hard del primo training.
- [x] Implementare e verificare la biforcazione controllata del solo peso.
- [x] Avviare e concludere LEFT72 peso 2 fino 82, con 10 aggiornamenti,
  40.960 nuove transizioni e telemetria integra.
- [x] Valutare nominalmente best80 e precedente last82, anche rispetto ai
  trace storici; produrre 14 plot canonici.
- [x] Eseguire l'analisi delle coppie best80 e il probe statico del contatto.
- [x] Risolvere la decisione sulla prosecuzione: autorizzata e conclusa da
  best80 fino 100,20 aggiornamenti e 81.920 nuove transizioni.
- [x] Analizzare nominalmente nuovo best82 e last100, produrre 14 plot e
  confrontare completamento, cinematica, contatto e coppie con best80.
- [x] Consolidare tutti gli otto report del 24 e conservare i TODO precedenti.

### Aperti: LEFT e verifiche comuni

- [ ] Riconciliare la collocazione delle figure diagnostiche reward LEFT e
  coppie best80, assenti nei percorsi storici alla chiusura; se servono ancora,
  localizzarle o rigenerarle dagli script conservati. I plot canonici non
  sono coinvolti in questa mancanza.
- [ ] Eseguire lo screening con perturbazioni comuni a policy congelata di
  best80 e precedente last82 contro best72; includere eventuali nuovi
  candidati se ancora utili. Misurare completamento, cicli, penetrazione,
  carico, cinematica per fase e coppie. La parte deterministica è conclusa,
  quella rumorosa no; best80 non è ancora certificato come più robusto.
- [ ] Eseguire il ramo controllo LEFT72 peso 1, con lo stesso fix hard,
  protocollo di resume, inizializzazione e budget del peso 2, se si continua
  il confronto causale. Non usare ritorni di reward diverse come criterio
  comune e non attribuire al peso 2 l'effetto dell'intero insieme di cambiamenti.
- [ ] Distinguere il miglioramento in continuazione dalle cause del divario
  nell'apprendimento LEFT da zero; serve un controllo dedicato.
- [ ] Se si interviene sulle coppie, progettare un confronto dal medesimo
  checkpoint e con budget identico: regolarità della coppia knee e momento
  ankle sotto carico, con scale esplicite e verifica di tracking/contatto.
- [ ] Fissare criteri distinti per completamento, cicli validi, forma delle
  traiettorie e coppie prima della selezione e dell'oscuramento; il solo
  ritorno o completamento temporale non basta.
- [ ] Quantificare separatamente il trascinamento degli ultimi angoli nel
  reset dei diagnostici di escursione FSM e l'incidenza nelle configurazioni
  che li utilizzano; nessun fix FSM applicato oggi.
- [ ] Eseguire la verifica live Windows del fix hard, della ripresa e della
  pipeline scelta. Le prove odierne attestano macOS arm 64.
- [ ] Pianificare repliche di training e condizioni di valutazione sufficienti
  per comparabilità statistica/generalizzazione, distinguendo esperienza,
  target e reward. Nessuna equivalenza statistica V26 è già dimostrata.

### Aperti: RIGHT, rinviati rispetto alla priorità LEFT

- [ ] Se si considera RIGHT last60 per riuso/oscuramento, valutarla sugli
  stessi cinque seed già usati per best58 prima della scelta.
- [ ] Approfondire il transitorio seed 125 confrontando best35, last40, best58,
  last60 e V26 su reference, velocità, contatto e margine di penetrazione.
- [ ] Isolare su RIGHT l'origine del momento GRF invertito nell'ultimo
  appoggio prima di modificarne reference, governor o reward. La diagnosi
  locale LEFT non chiude automaticamente questo controllo.

Le attività sopra sono pendenti, non nuovi comandi di esecuzione. Il
trasferimento RIGHT→LEFT non è stato avviato; oscuramento e training ex-novo
restano da pianificare. Restano validi i vincoli utente: FSMv 3, prescribed e
clock visibili nelle prove attuali, plot canonici nella root e nessuna
prosecuzione RIGHT oltre 60.

## Backlog ereditato da propagare

Le voci seguenti sono conservate dal precedente daily e già riportate
nell'aggiornamento parziale odierno, a sua volta basato sul
[daily del15 settembre](2026-09-15_daily-report.md). Non sono state
riesaminate integralmente in questa chiusura. Le formulazioni storiche,
anche ripetute o superate in parte, non autorizzano automaticamente nuove
esecuzioni: prevalgono lo stato finale e le decisioni riportate sopra.
La creazione della39D è completata; i riferimenti a V1, A/B/C, teacher,
DAgger e morphology appartengono ai rispettivi piani storici e non sono
nuove modifiche del training imitativo qui analizzato.

- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale confrontando i risultati a 5, 50 e 100.
- [x] Creazione della variante parallela a 39 feature: esaurita; la variante è presente ed è il soggetto del training e delle valutazioni qui riportati.
- [ ] Definire e verificare schema 39, maschera persistente e indipendenza ex-novo dalle copie dei target; fissare curriculum, budget, seed e criteri prima del confronto A/B/C.
- [ ] Eseguire diagnostica dell'osservabilità, confronto A/B/C completamente mascherato in valutazione e test di trasferimento ex-novo B/C, distinguendo avvio, regime e variabilità tra seed.
- [ ] Risolvere e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start.
- [ ] Costruire dati diagnostici temporalmente allineati e valutare osservabilità delle 35 feature su episodi indipendenti.
- [ ] Scegliere fra curriculum con target mascherati, avvio supervisionato e possibili soluzioni causali di fase/memoria sulla base di evidenze.
- [ ] Se si procede con la maschera, definire il nuovo schema, la normalizzazione, l'eventuale indicatore di disponibilità e lo stato persistente del curriculum, conservando la V1.
- [ ] Implementare e verificare isolamento dei target, coerenza raccolta/PPO e indipendenza degli aggiornamenti ex-novo dalle copie dei target.
- [ ] Definire budget, seed, progressione e criteri di accettazione prima del confronto A/B/C; eseguire poi il pilot e le repliche necessarie.
- [ ] Validare i candidati completamente mascherati ai tre start, con metriche separate di avvio, tracking, cicli validi e protezioni.
- [ ] Verificare trasferimento, warm-up e resume della variante scelta e confrontarne il beneficio nel successivo ex-novo.
- [ ] Misurare in un nuovo run imitativo l'effetto del peso di penetrazione 1.0, senza attribuirgli gli esiti dello smoke storico a 0.05.
- [ ] Completare il benchmark e il verdetto di parità qualitativa secondo il report del 6 settembre, includendo costo di apprendimento e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows della pipeline scelta; riprendere predittività e compensazione del ritardo soltanto in una versione successiva.
- [ ] Fissare checkpoint baseline, manifest degli input e protocollo comune
  di confronto, mantenendo intatti gli artefatti storici.
- [ ] Implementare il valutatore comune sotto `Trajectory Generator/unified_MLP/`,
  con metriche per start e per giunto, copertura dei cicli e gestione esplicita
  dei fallimenti e dei dati mancanti.
- [ ] Completare e riprodurre il benchmark baseline deterministico sui tre start.
- [ ] Definire prima della selezione finale margini di regressione, condizioni
  riservate alla valutazione, numero di repliche e budget di addestramento.
- [ ] Eseguire un vero pretraining e training ex-novo della rete unificata,
  con valutazioni periodiche; pilot imitativo a cinque iterazioni concluso,
  prosecuzione e training ex-novo completo ancora da eseguire.
- [ ] Eseguire il confronto deterministico e stocastico finale, includendo
  variabilità tra seed di training e incertezza delle differenze.
- [ ] Produrre il verdetto quantitativo di parità, regressione o risultato
  inconclusivo, insieme al costo necessario per raggiungere la qualità ottenuta.
- [ ] Eseguire le verifiche live su Windows per attestare anche quella piattaforma.
- [ ] Riprendere actor predittivo e compensazione del ritardo in una versione
  successiva; restano esclusi dal recupero della parità della V1 attuale.
- [ ] Implementare il teacher inverse-governor e il generatore delle label raw.
- [ ] Validare frozen-policy gli orizzonti attorno a `90 ms` knee e `100 ms`
  ankle su più cadenze e stati del governor.
- [ ] Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e
  recovery.
- [ ] Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.
- [ ] Ripetere il critic-only rewarm quando la reward diventerà realmente
  asimmetrica.
- [ ] Valutare una fascia soft interna e una corretta assegnazione causale del
  credito, senza spostare il corridor finale.
- [ ] Progettare e validare un corridor cadence-conditioned su dati held-out.
- [ ] Preregistrare ed eseguire un pilot PPO separato di `5-10` iterazioni con
  `sigma=0.010` isotropico e `freeze_logstd=true`, senza modificare J20 o il
  canonico lungo.
- [ ] Decidere un'eventuale estensione oltre 10 iterazioni soltanto dopo i
  rollout deterministici e i gate del pilot.
- [ ] Monitorare penetrazione `20/25/28 mm`, served-vs-actual e contatori FSM
  v3 in tutti i nuovi candidati.
- [ ] Rivalutare sigma dopo l'actor predittivo o una modifica della logica
  causale del morphology corridor.
- [ ] Generalizzazione oltre AB06 tramite dataset EPIC e soggetto held-out,
  dopo il recupero della qualità AB06.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri soggetti ad approvazione
  esplicita.
- [ ] Conservare sidecar J8 e artefatti storici senza modifiche retroattive.
- [ ] Riallineare in manutenzione separata le assertion legacy soggette a drift
  e gli altri debiti tecnici/amministrativi già propagati: osservabilità RLlib,
  `cycle_valid`, swing timeout, probe schema, pin V20, `best == last`,
  `fit_p2` V12R3, marker/audit storici e deflessione SEA iniziale.
- [ ] Completare V1 fino a100 e confrontare5/50/100.
- [ ] Creare la variante39 dopo valutazione100; superare isolamento e smoke.
- [ ] Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.
- [ ] Rendicontare tutti gli esiti e costi, distinguendo fattibilità e parità.
- [ ] Eseguire la verifica live Windows separata.
- [ ] Proseguire la stessa V1 da 50 a 100 iterazioni totali e svolgere la valutazione finale 5/50/100.
- [ ] Svolgere la fase 5: variante 39, isolamento, smoke, confronto A/B/C e trasferimento ex-novo B/C secondo il protocollo.
- [ ] Completare la diagnostica dell'osservabilità e verificare la sincronizzazione senza clock prescribed, distinguendo avvio e regime.
- [ ] Completare il benchmark di parità con la baseline, includendo costi e variabilità tra seed.
- [ ] Eseguire le verifiche live Windows; mantenere aperti i debiti storici non risolti.

<!-- reconciled-report-todos-2026-10-05:start -->
## TODO riconciliati e propagati — integrazione del 5 ottobre 2026

Ricostruzione documentale dello stato a questa data. Le voci sotto precisano i TODO storici ripetuti nelle sezioni precedenti: una nuova esecuzione o un PASS su un altro ramo non chiude il debito originario. Le attività condizionate restano proposte; questa integrazione non avvia esperimenti. Le chiusure riportano una fonte esplicita.

### Attività aperte

- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto hardware prima di qualsiasi claim di deployment. — [fonte 2026-08-09, riga 342](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery, soggetti/profili esterni, rumore e delay. — [fonte 2026-08-09, riga 340](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA coerente con la coppia richiesta. — [fonte 2026-08-09, riga 344](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Compilare/distribuire e verificare su Windows x86_64 le DLL SEA `_ff` e OnlineGRF corrette, con provenienza e ABI OpenSim/Simbody/Python; claim numerico separato dalla prova macOS. — [fonte 2026-08-09, riga 337](../user/2026-08-09_h0_v12r2_training_readiness.md).
- [ ] Rieseguire (opzionale, read-only) l'eval offline sul `corpus.npz` + modulo salvati in `v12r3/…/fit/p2/` per stabilire quale metrica bocciò il `fit_p2` e di quanto: unico esito della sessione senza numeri persistiti. — [fonte 2026-08-15, riga 181](../user/2026-08-15_retrospettiva_forense_sessione_09-10_agosto_v12r2_v12r4.md).
- [ ] Riconciliare il pin V20/V26 in train_ppo_mlp.py. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Chiarire l’anomalia teacher retry best==last==warm_start. — [fonte 2026-08-16, riga 192](../user/2026-08-16_v12r12_fix_pre_freeze_applicati_e_verificati.md).
- [ ] Valutare un override esplicito dello schema in rollout_eval.py per il checkpoint39/43. — [fonte 2026-08-18, riga 149](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] Promuovere il driver di qualifica da scratchpad a script versionato alla prossima esecuzione della catena. — [fonte 2026-08-20, riga 95](../user/2026-08-20_risultato_training_ready_exnovo_B0820.md).
- [ ] Follow-up v3 fuori scope: riportare lo swing timeout da 2,6 a 1,3 s. — [fonte 2026-08-21, riga 95](../user/2026-08-21_fsm_v3_resync_validazione_e_bug_tolleranza_corridoio.md).
- [ ] Qualificare generalizzazione oltre AB06 su almeno un soggetto/modello EPIC completamente held-out da teacher, DAgger/PPO, iperparametri e corridor; più trial dello stesso soggetto non bastano. — [fonte 2026-08-22, riga 15](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md).
- [ ] **Contratto FSM phase-aligned:** correggere o rendere univoca la semantica di `cycle_valid` dopo una cancellazione HS; fino ad allora i prodotti phase-aligned restano fail-closed. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] **Osservabilità RLlib:** nel prossimo eventuale run registrare environment configurati/sani, batch corrente/lifetime e generation/replacement dei runner in campi distinti. — [fonte 2026-08-23, riga 109](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md).
- [ ] Chiarire perché B3/target_phase vale zero sotto runtime v 3 anche con cicli validi. — [fonte 2026-08-24, riga 212](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] Decidere se archiviare il bundle J10 rifiutato, preservandolo byte-identico come evidenza. — [fonte 2026-08-27, riga 243](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] Decidere documentalmente sui due gcs_server orfani segnalati il 18 agosto; verificare stato e necessità prima di eventuali interventi. — [fonte 2026-08-27, riga 273](../user/2026-08-27_v26c_j20_restore_audit_r2_preparazione.md).
- [ ] Mantenere esplicita l’assenza storica di commit_verification in J2/J8 e di actor_digest in J2; evitare attestazioni retroattive inventate. — [fonte 2026-08-27, riga 233](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Consolidare additivamente la correzione rad/secondi nei record V26C congelati, senza modificarne gli artefatti numerici. — [fonte 2026-08-27, riga 353](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Chiarire/correggere in manutenzione separata policy_std=null nelle vecchie telemetrie J12–J19, senza inventare valori o riscrivere evidenza congelata. — [fonte 2026-08-27, riga 219](../user/2026-08-27_v26c_j16_j15r1_closed_loop_qualification_readiness.md).
- [ ] Chiarire path di staging e valid_cycle_count dei receipt J9R1 e i falsi heel strike del canale legacy 20 N; preservare i record e spiegare che non alterarono FSM/reward/verdetto. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] Documentare la provenance mancante dei runner nei vecchi leaf e i due runner J15/J15R1 verso la stessa leaf; i pin introdotti in J18 non colmano i precedenti. — [fonte 2026-08-27, riga 231](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Conservare il limite di riproducibilità J11: artefatti senza versione Torch/argomenti Adam sufficienti; il J15R1 riproducibile non colma retroattivamente J11. — [fonte 2026-08-27, riga 232](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Mantenere espliciti nei riepiloghi DIAGNOSTIC_NOT_BINDING contraddittorio e assenza di post-commit storico J9R1; le correzioni dei successori non riscrivono quel leaf. — [fonte 2026-08-27, riga 341](../user/2026-08-27_v26c_j12_closed_loop_qualification_readiness.md).
- [ ] Conservare e spiegare additivamente le discrepanze July su label raw/mean, gate ok:false e numeri di riga, senza riscrivere J0–J10R1. — [fonte 2026-08-27, riga 337](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] Decidere e verificare la correzione del campo nominal_mean_shift dichiarato ma non calcolato dal runner J15R1; misura esterna conclusa, debito di codice distinto. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j15r1_fresh_refit_execution.md).
- [ ] Se si vuole attribuire causalmente la deviazione pre-breccia, svolgere un confronto appaiato; una sola trace non isola i fattori confondenti. — [fonte 2026-08-27, riga 235](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Indagare il clock swing non azzerato su hs_bounce_cancelled, distinguendolo dalla semantica cycle_valid; nessun fix è attestato da J19/R3/J21. — [fonte 2026-08-27, riga 429](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] Separare o reinterpretare best_validation_mse contaminato dalla ripetizione dei blocchi nei fit storici; non usarlo come validazione indipendente. — [fonte 2026-08-27, riga 234](../user/2026-08-27_v26c_j17_architect_audit_addendum.md).
- [ ] Decidere come spiegare/gestire il marker sorgente RESTORE_AUDIT_PENDING immutabile dopo la chiusura esterna G9, senza mutare retroattivamente il checkpoint. — [fonte 2026-08-28, riga 250](../user/2026-08-28_v26c_j20_restore_audit_r3_esecuzione.md).
- [ ] Valutare le tre osservazioni non vincolanti R3 rev 7 (conversione downcast/None, mismatch raw R2, tipi optimizer registrati), separandole dai gate 13/13 passati. — [fonte 2026-08-28, riga 299](../user/2026-08-28_v26c_j20_restore_audit_r3_correzione_rev7_gate_learning_rate.md).
- [ ] Progettare e validare corridor cadence-conditioned su dati held-out. — [fonte 2026-08-30, riga 346](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Valutare fascia soft interna e assegnazione causale del credito senza spostare il corridor finale. — [fonte 2026-08-30, riga 360](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Verificare uno sweep 0–180 ms di anticipo della sola loss e il rapporto con il ritardo phase-normalized 135–160 ms; distinto dalla decomposizione già conclusa. — [fonte 2026-08-30, riga 376](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] Preregistrare il primo rapporto morphology knee/ankle asimmetrico. — [fonte 2026-08-31, riga 606](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Validare frozen-policy gli orizzonti circa 90 ms knee / 100 ms ankle su più cadenze e stati del governor. — [fonte 2026-08-31, riga 603](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Definire la loss imitativa raw-space/served-predicted-space per il nuovo teacher, prima di costruire la rete predittiva. — [fonte 2026-08-31, riga 604](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] Riprodurre il benchmark deterministico della stessa baseline sui tre start: la sola prova centrale storica non basta. — [fonte 2026-09-06, riga 247](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Confrontare nominale e stocastico con seed indipendenti di training e incertezza delle differenze. — [fonte 2026-09-06, riga 252](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Preregistrare margini, condizioni held-out, repliche e budget prima della selezione finale. — [fonte 2026-09-06, riga 248](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Fissare baseline, manifest e protocollo comune di parità; conservare gli artefatti storici. — [fonte 2026-09-06, riga 242](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Completare il confronto imitazione/ex-novo con valutazioni periodiche secondo il benchmark: il pretraining V1 è concluso, ma smoke e pilot non esauriscono il confronto finale. — [fonte 2026-09-06, riga 250](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Completare il valutatore comune con metriche per start/giunto, cicli, fallimenti e dati mancanti; i valutatori di singole campagne non attestano l’intero benchmark. — [fonte 2026-09-06, riga 244](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Rendere il verdetto quantitativo di parità/regressione/inconclusivo e il costo per raggiungere la qualità, senza confondere fattibilità tecnica con qualità biomeccanica. — [fonte 2026-09-06, riga 254](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Eseguire e attestare separatamente le verifiche native Windows della pipeline scelta. — [fonte 2026-09-06, riga 256](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] Completare la verifica dell’indipendenza degli aggiornamenti ex-novo dalle copie dei target. L’assenza di nuove copie nel suffisso privilegiato e la conservazione della maschera raccolta/PPO sono documentate; non equivalgono da sole al test specifico degli aggiornamenti. — [fonte 2026-09-14, riga 295](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Validare candidati completamente mascherati ai tre start, con avvio/tracking/cicli/protezioni distinti; il primo pilot del 1 ottobre non esaurisce questa validazione. — [fonte 2026-09-14, riga 297](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Costruire dati allineati e misurare osservabilità delle 35 feature su episodi indipendenti. — [fonte 2026-09-14, riga 292](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Preregistrare ed eseguire A/B/C di osservabilità, trasferimento B/C e repliche: non confondere questo protocollo con le successive ablazioni clock/scala/slew. — [fonte 2026-09-14, riga 97](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).
- [ ] Confrontare curriculum, avvio supervisionato e soluzioni causali di fase/memoria con evidenze; l’adozione del masking non risolve da sola l’osservabilità. — [fonte 2026-09-14, riga 293](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Verificare sincronizzazione senza clock prescribed, distinguendo avvio e regime ai tre start. — [fonte 2026-09-14, riga 291](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Verificare trasferimento, warm-up e resume della variante scelta e misurare il beneficio nel successivo ex-novo; correttezza tecnica e vantaggio di apprendimento sono distinti. — [fonte 2026-09-14, riga 298](../user/2026-09-14_osservabilita_prescribed_mascheramento_e_test_fattibilita.md).
- [ ] Completare il confronto V1 5/50/100 sui tre start. Il training fino a100 è superato dalla continuazione fino a300; il confronto specifico 5/50/100 non risulta documentato e resta aperto. — [fonte 2026-09-14, riga 95](../user/2026-09-14_avvio_piano_pretraining_50_100_fattibilita.md).
- [ ] Manutenzione separata delle assertion legacy e debiti RLlib, cycle_valid, swing timeout, probe schema, pin V20, best==last, fit_p2 V12R3, marker/audit e deflessione SEA iniziale; chiudere solo con prova specifica. — [fonte 2026-09-15, riga 161](../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).
- [ ] **W02:** sostituire il controllo `os.kill(pid, 0)` del percorso attach con una verifica di vitalità non distruttiva e validarla su Windows. — [fonte 2026-09-15, riga 353](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W03:** predisporre un percorso Windows per i runner J20 warm-up e restore R3, conservando la provenance storica e rendendo esplicito l'interprete della nuova esecuzione. — [fonte 2026-09-15, riga 354](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W04:** canonicalizzare le chiavi dei percorsi nell'attestazione J21 e verificare su Windows i gate T12/T18 con artefatti integri e casi negativi. — [fonte 2026-09-15, riga 355](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W05:** definire una politica dei fine-riga coerente con gli hash dei sorgenti unified e verificarla su un checkout Windows reale senza riscrivere i pin storici per nascondere differenze. — [fonte 2026-09-15, riga 356](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W06:** rendere il generatore differito 39 compatibile con LF e CRLF e verificarne la materializzazione rispettando la separazione dalla V1. — [fonte 2026-09-15, riga 357](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W07:** definire e verificare la ricollocazione di run, checkpoint e trace per il riuso delle valutazioni CP50/CP100 mantenendo le verifiche di identità del contenuto. — [fonte 2026-09-15, riga 358](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **W08:** distinguere nell'analisi sigma un report mancante da un sigma realmente nullo e verificare la rianalisi degli artefatti trasferiti. — [fonte 2026-09-15, riga 359](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Fissare un ambiente Windows riproducibile con versioni Python/Ray/Torch/NumPy/OpenSim e dipendenze native, compatibile con i checkpoint scelti. — [fonte 2026-09-15, riga 361](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Documentare e verificare root di checkout, supporto long-path Windows/Git e percorsi generati dal workflow selezionato. — [fonte 2026-09-15, riga 360](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Verificare su Windows orchestrazione, valutazioni/report e, quando prevista, materializzazione della variante 39; attestare soltanto i percorsi realmente provati. — [fonte 2026-09-15, riga 363](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Eseguire una verifica nativa Windows di caricamento plugin, reset/step, training breve con worker Ray, rollout, salvataggio/restore e warm-up, registrando gli esiti separatamente dalle evidenze macOS. — [fonte 2026-09-15, riga 362](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] Validare oracle LEFT nel loop fisico sui tre start; il controllo successivo su un solo start non chiude la richiesta. — [fonte 2026-09-20, riga 199](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] Valutazioni dedicate dei checkpoint nominali e stocastiche, con durata/tracking/contatto e un insieme held-out distinto dalla selezione. — [fonte 2026-09-20, riga 200](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] Ampliare la valutazione best72 oltre cinque seed per condizione: l’evaluation iniziale è conclusa, l’estensione resta aperta. — [fonte 2026-09-22, riga 156](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Isolare con prove controllate gli effetti di traiettoria, risposta dinamica e distribuzione delle azioni; confrontare in rollout comando atteso dopo clipping e media clippata. — [fonte 2026-09-22, riga 157](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Estendere confronti fail/success alla stessa fase e prima della soglia nei fallimenti di training, oltre ai due casi appaiati della 100. — [fonte 2026-09-22, riga 158](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Esplicitare nei riepiloghi automatici clipping raw a monte di RLlib e clipping interno all’ambiente; la distinzione nel testo non attesta l’implementazione automatica. — [fonte 2026-09-22, riga 159](../user/2026-09-22_risultati_finali_unified39_100iter.md).
- [ ] Separare nel software reward, visibilità dei sei canali ed esplorazione, consentendo ex-novo visibile e curriculum. — [fonte 2026-09-23, riga 124](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).
- [ ] Preparare confronto causale RIGHT/LEFT da zero con FSM v3, ordine ingressi, inizializzazione, raccolta, Adam/resume e budget comuni; due storie separate non provano il solo effetto del target. — [fonte 2026-09-23, riga 123](../user/2026-09-23_strategia_best72_confronto_v26_e_oscuramento.md).
- [ ] Verificare separatamente reset dei diagnostici di escursione FSM e incidenza nelle configurazioni che li usano. — [fonte 2026-09-24, riga 140](../user/2026-09-24_fix_contatto_terminale_hard_unified39.md).
- [ ] Valutare best80/last82 e successivi candidati contro best72 con perturbazioni comuni e misure fisiche; il piccolo pilot reward non esaurisce questo confronto. — [fonte 2026-09-24, riga 102](../user/2026-09-24_avvio_left72_penetrazione_peso2_10iter.md).
- [ ] Valutare RIGHT last60 sugli stessi cinque seed prima di scegliere un checkpoint per masking. — [fonte 2026-09-24, riga 266](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] Isolare origine del momento GRF invertito nell’ultimo appoggio prima di cambiare reference/governor/reward. — [fonte 2026-09-24, riga 270](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] Approfondire transitorio seed125 su best35/40/58/60 e V26 con reference, velocità, contatto e penetrazione. — [fonte 2026-09-24, riga 268](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).

### Attività condizionate e vincoli

- [ ] **Vincolo:** Per ogni candidata, eseguire qualification scientifica distinta dall’integrità tecnica del training; questa regola non è un’attestazione di robustezza generale. — [fonte 2026-08-14, riga 215](../user/2026-08-14_h0_v12r9_training_readiness_preflight_monitor.md).
- [ ] **Condizionato:** Probe `train_ppo` (39/43): correggere solo con migrazione di schema dichiarata (lineage futura), non per i training comparabili con giugno. — [fonte 2026-08-18, riga 146](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md).
- [ ] **Condizionato:** Arbitrare la tensione fra interpolazione, quota on-policy e copertura piena prima di un nuovo round interpolato. — [fonte 2026-08-24, riga 214](../user/2026-08-24_v26b_audit_luglio_r1r2_e_progetto_rev4d.md).
- [ ] **Condizionato:** Riesaminare l'effetto della lunghezza del prefisso distinguendo interpolazione zero e ricette interpolate. — [fonte 2026-08-24, riga 180](../user/2026-08-24_v26b_rev4d_repeat_esperimento_decorrelato.md).
- [ ] **Condizionato:** Affrontare il margine di contatto del candidato storico Rev 4E, oltre 28 mm di 0,22 mm, se il ramo viene riaperto. — [fonte 2026-08-24, riga 203](../user/2026-08-24_v26b_rev4e_r2_replay.md).
- [ ] **Condizionato:** Conservare il limite interpretativo: il braccio A non converge al proprio budget e il suo NO-GO non identifica un limite del contratto. — [fonte 2026-08-25, riga 112](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md).
- [ ] **Condizionato:** Riesaminare il peggioramento del fold 9 dopo il minimo a epoca 100 se il ramo B1 viene riaperto. — [fonte 2026-08-25, riga 154](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md).
- [ ] **Condizionato:** Identificare la causa dell'errore delle teste rispetto a ciclo 1 senza concludere non-identificabilità da vicinanza. — [fonte 2026-08-25, riga 167](../user/2026-08-25_v26b_b1r2_diagnostica.md).
- [ ] **Condizionato:** Trattare l'errore localizzato nelle 308 righe di testa, distinto dalla vecchia assenza di copertura LOCO. — [fonte 2026-08-25, riga 185](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md).
- [ ] **Condizionato:** Nei futuri confronti non usare come generalizzazione il best val MSE luglio da split casuale su righe ripetute 32×. — [fonte 2026-08-25, riga 199](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Rivedere l'igiene della selection/patience nel ramo B1, senza confonderla con convergenza o causa del NO-GO. — [fonte 2026-08-25, riga 124](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md).
- [ ] **Condizionato:** Riesaminare eseguibilità delle label swing ammesse per raggiungibilità,13,5% oltre il criterio 10%. — [fonte 2026-08-25, riga 139](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md).
- [ ] **Condizionato:** Risolvere max_abs WAIT oltre 0,15 nel ramo B1, se riaperto. — [fonte 2026-08-25, riga 198](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md).
- [ ] **Condizionato:** Conservare LOTO3 fold, LOCO6 fold,11 fold B1R2, braccio budgetA e braccio lrB come rami futuri non operativi; integrarli solo con approvazione esplicita. — [fonte 2026-08-26, riga 184](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Verificare retrospettivamente l’equivalenza semantica del teacher prescritto sotto v3 nell’audit della collection storica. — [fonte 2026-08-26, riga 178](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Attestare il gate della collection teacher storica sotto v3 e corridoio corrente con la relativa evidenza. — [fonte 2026-08-26, riga 179](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md).
- [ ] **Condizionato:** Conservare i limiti J14: label open-loop su stati student-visited, phase-invalid senza soglia e 183 righe pre-mismatch mantenute senza contributo informativo; chiudere la qualità solo con closed-loop. — [fonte 2026-08-27, riga 256](../user/2026-08-27_v26c_j14_post_mismatch_dagger_dataset_readiness.md).
- [ ] **Condizionato:** Eventuale integrazione/riapertura dei rami LOTO, LOCO, B1R1/B1R2 con protocollo e autorizzazione; conservarne i verdetti storici, senza considerarli già operativi. — [fonte 2026-08-27, riga 343](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Usare i ceiling empirici J8 C/D dichiarati dopo ricalibrazione, distinguendoli dalla soglia ritirata; nessun rilassamento retroattivo del FAIL J18. — [fonte 2026-08-27, riga 265](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Conservare la priorità A>B>C>D e la rimozione di 4 righe su 4225 come parte esplicita del design/pesi, non una scelta variabile dopo il risultato. — [fonte 2026-08-27, riga 376](../user/2026-08-27_v26c_j18_prereg_update_b_only.md).
- [ ] **Condizionato:** Mantenere max-drift come tie-break diagnostico nella selezione preregistrata; un futuro cambio del ranking richiede una nuova decisione. — [fonte 2026-08-27, riga 267](../user/2026-08-27_v26c_j18_ricalibrazione_eligibilita_e_selezione.md).
- [ ] **Condizionato:** Documentare ENV_MUTATION_POLICY come superset conservativa in J19C: autorizzava variazioni start non esercitate dalla prova nominale. — [fonte 2026-08-27, riga 227](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Conservare e spiegare le assertion readiness diventate false dopo emissione GO/esecuzione (E54/S47 e successive), senza trattarle come regressioni scientifiche o riscrivere i file pinnati. — [fonte 2026-08-27, riga 192](../user/2026-08-27_v26c_j19a_esecuzione_riproduzione_singola.md).
- [ ] **Condizionato:** Valutare in un futuro protocollo se lo shift nominale debba essere un gate anticipato: J11=0.0264 rispetto al riferimento July 0.005 era deliberatamente non binding. — [fonte 2026-08-27, riga 332](../user/2026-08-27_v26c_j11_multistart_fit_execution.md).
- [ ] **Condizionato:** Monitorare bande 20/25/28 mm e margini di F/H (minimo storico circa 2.316 mm), senza trasformare diagnostici soft in nuovi hard gate a posteriori. — [fonte 2026-08-27, riga 247](../user/2026-08-27_v26c_j19b_esecuzione_qualificazione_closed_loop.md).
- [ ] **Condizionato:** Conservare la fragilità storica seed 125/F e monitorare i margini nelle qualifiche; il dato precedente non certifica nuove policy. — [fonte 2026-08-27, riga 431](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Una qualifica non autorizza implicitamente warm-up/PPO: ogni nuovo stadio richiede la sua successiva autorizzazione pertinente. — [fonte 2026-08-27, riga 212](../user/2026-08-27_v26c_j19c_esecuzione_heldout_g_i.md).
- [ ] **Condizionato:** Ogni nuova fase deve avere report dedicato/audit e autorizzazione pertinente prima della successiva; i GO monouso storici non vengono riutilizzati. — [fonte 2026-08-27, riga 205](../user/2026-08-27_v26c_j15_fresh_refit_execution.md).
- [ ] **Vincolo:** Conservare immutati sidecar J8 e artefatti storici. — [fonte 2026-08-27, riga 356](../user/2026-08-27_qualifica_closed_loop_j9r1.md).
- [ ] **Condizionato:** Se si riusa questo design, giustificare repeat 8 anziché ereditarlo dalla metodologia July; le nuove righe J14 usano repeat 1, ogni diversa ponderazione richiede preregistrazione. — [fonte 2026-08-27, riga 184](../user/2026-08-27_v26c_j10_multistart_teacher_readiness.md).
- [ ] **Condizionato:** La forza vincolante del protocollo 23 agosto non autorizzato richiede decisione esplicita prima di usarlo in un nuovo esperimento. — [fonte 2026-08-27, riga 435](../user/2026-08-27_v26c_j13_j12_failure_diagnosis_and_recovery_plan.md).
- [ ] **Condizionato:** Sorvegliare la copertura WAIT_HS: 20 righe uniche solo B, 160 dopo tiling; non inventare una soglia di sufficienza o confondere dato ripetuto con indipendente. — [fonte 2026-08-27, riga 241](../user/2026-08-27_v26c_j10r1_multistart_teacher_collection.md).
- [ ] **Condizionato:** Conservare e spiegare i check R1 B06/I07 e R2 C29/E01/H02/H04 diventati falsi per destinazione presente dopo esecuzione, senza riscrivere gli artefatti pinnati. — [fonte 2026-08-28, riga 276](../user/2026-08-28_v26c_j20_restore_audit_r2_esecuzione.md).
- [ ] **Condizionato:** Per ogni nuovo pilot fissare config/comando/budget/stop/monitor/rollback e ottenere GO nuovo; la proposta storica 5–10 sigma 0.010 non viene considerata eseguita solo perché esistono run successivi con altro budget. — [fonte 2026-08-28, riga 1059](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Per una futura ricostruzione applicare l’intera checklist: parent agosto verificato, input July esclusi, actor 35/critic 84 e mapping per nome, sigma misurata, dataset tracciati, preservation funzionale, qualifica 6/6+3/3, FSM/penetrazione sane, actor congelato nel warm-up, critic/Adam/restore esatti e attestazione separata senza autorizzazione implicita al training. — [fonte 2026-08-28, riga 1029](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Rivalutare sigma dopo actor predittivo o modifica causale del morphology corridor. — [fonte 2026-08-28, riga 1055](../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md).
- [ ] **Condizionato:** Mantenere la served contro corridor retrospettivo HS→TO→HS come metrica finale event-aligned, senza alterare reward passata o spostare il gate per accettare ritardi. — [fonte 2026-08-30, riga 358](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Se si interviene sul governor, confrontare cutoff/slew ankle in un’ablation controllata a policy congelata; la misura dei lag correnti non prova il risultato di tale modifica. — [fonte 2026-08-30, riga 374](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Se si riprende questa strategia, implementare teacher inverse-governor e label raw da stato effettivamente visitato; validare il beneficio closed-loop. Pista distinta dal RL senza teacher. — [fonte 2026-08-30, riga 392](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Preregistrare gate quantitativi per toe-off, saturazione, FSM, tracking e condizioni held-out/multi-start prima di selezionare l’actor predittivo. — [fonte 2026-08-30, riga 394](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Preregistrare pilot PPO separato 5–10 update, sigma 0,010 isotropico e logstd congelato, conservando J20/canonico; non sostituirlo con pilot di altra ricetta. — [fonte 2026-08-30, riga 388](../user/2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md).
- [ ] **Condizionato:** Ripetere il critic-only rewarm quando la reward diventa realmente asimmetrica. — [fonte 2026-08-31, riga 346](../user/2026-08-31_separazione_morphology_knee_ankle_e_critic_rewarm.md).
- [ ] **Condizionato:** Ogni nuovo run usa config/namespace separati senza alterare J20, canonico lungo o GO consumati; è un invariante, non una creazione da rifare ora. — [fonte 2026-08-31, riga 415](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Decidere eventuale estensione oltre 10 soltanto dopo rollout deterministici e gate del pilot J20. — [fonte 2026-08-31, riga 419](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Nel pilot preregistrato verificare dopo ogni iterazione penetrazione, FSM, loss, clipping/saturazione e deriva actor; valutare deterministicamente i checkpoint prodotti. — [fonte 2026-08-31, riga 417](../user/2026-08-31_smoke_test_sigma_esplorazione_v26c_j20.md).
- [ ] **Condizionato:** Eseguire DAgger closed-loop sullo stesso actor 35D con nominali e recovery, se si riprende questa strategia. — [fonte 2026-08-31, riga 610](../user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md).
- [ ] **Condizionato:** Riprendere actor predittivo e compensazione del ritardo soltanto in una versione successiva; attività differita. — [fonte 2026-09-06, riga 257](../user/2026-09-06_rete_unificata_v1_e_protocollo_parita_qualitativa.md).
- [ ] **Vincolo:** Monitorare penetrazione 20/25/28 mm, served-vs-actual e FSM v3 in ogni nuovo candidato. — [fonte 2026-09-15, riga 152](../user/2026-09-15_pretraining_unificata_valutazione_intermedia_50.md).
- [ ] **Vincolo:** Preservare gli artefatti storici e distinguere portabilità dei pesi, migrazione dei risultati e riproduzione dell’esperimento. — [fonte 2026-09-15, riga 364](../user/2026-09-15_audit_compatibilita_windows_da_v26_a_unified_mlp.md).
- [ ] **Condizionato:** Eventuali ulteriori prove da B devono isolare un fattore per volta. I successivi rami allineati modificano più impostazioni insieme; non attestano la chiusura delle ablazioni proposte né autorizzano il loro avvio. — [fonte 2026-09-20, riga 201](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] **Condizionato:** Valutare il significato della guardia knee a 0°, distinguendo sicurezza, durata e fedeltà; eventuale modifica richiede prova separata. — [fonte 2026-09-20, riga 202](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] **Vincolo:** Separare masking dalla costruzione di una baseline affidabile e fissare seed/criteri prima di rivendicare parità. — [fonte 2026-09-20, riga 203](../user/2026-09-20_risultati_ablazioni_unified39_clock_scala_slew.md).
- [ ] **Condizionato:** Eventuale screening appaiato dei checkpoint LEFT25/29/40 e geometria di contatto nei tratti problematici; i soli rollout CP40 non completano tutte le celle. — [fonte 2026-09-21, riga 151](../user/2026-09-21_risultati_unified39_v26_aligned_40iter.md).
- [ ] **Vincolo:** Valutare masking come passaggio distinto senza presumere che ex-novo risolva automaticamente la robustezza. — [fonte 2026-09-23, riga 278](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).
- [ ] **Vincolo:** Distinguere RIGHT da zero, adattamento LEFT, trasferimento V26 e masking/ex-novo; non attribuire risultati di un braccio agli altri. — [fonte 2026-09-23, riga 125](../user/2026-09-23_avvio_training_39d_riferimento_destro.md).
- [ ] **Condizionato:** Se prioritario, confrontare V26 e best72 con rumore assoluto comune, distinguendo target e policy media: sigma proprie con gaussiani comuni non isolano la sola esplorazione. — [fonte 2026-09-23, riga 273](../user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md).
- [ ] **Vincolo:** Fissare criteri distinti per completamento, cicli, forma e coppie prima della selezione e del masking; non usare il solo return. — [fonte 2026-09-24, riga 272](../user/2026-09-24_resoconto_39d_right_60iter_e_robustezza.md).
- [ ] **Vincolo:** Distinguere beneficio della continuazione da spiegazione del divario nell’apprendimento da zero, che richiede un controllo dedicato. — [fonte 2026-09-24, riga 309](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).
- [ ] **Condizionato:** Se si procede, eseguire il controllo LEFT72 peso 1 contro peso 2, con terminale hard corretto, stesso resume, budget e criteri fisici; il solo ramo peso 2 non isola causalità. — [fonte 2026-09-24, riga 307](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).
- [ ] **Vincolo:** Eventuali modifiche reward per le coppie devono usare stesso checkpoint/budget, tracking e contatto come vincoli e confronti separati knee/ankle. — [fonte 2026-09-24, riga 121](../user/2026-09-24_best80_cause_coppie_ankle_knee.md).

### Chiusure documentate e consolidamento retrospettivo

- [x] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`. — [fonte 2026-08-17, riga 121](../user/2026-08-17_diagnosi_divergenza_luglio_v26_e_proposta_retraining_imitativo_nativo.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report della giornata). — [fonte 2026-08-17, riga 95](../user/2026-08-17_h0_v12r17_verdetto_e_probe_capacita.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-17_daily-report.md)).
- [x] Daily 2026-08-16/17/18 da consolidare al prossimo `end_day`. — [fonte 2026-08-18, riga 154](../user/2026-08-18_rollout_nativo_v26_bug_probe_schema_7plot_e_avvio_opzione_a.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-18_daily-report.md)).
- [x] Daily 2026-08-18/19 da consolidare al prossimo `end_day`. — [fonte 2026-08-19, riga 159](../user/2026-08-19_indagine_iterazioni_lente_crash_storm_reject_continue_fix.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-19_daily-report.md)).
- [x] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`. — [fonte 2026-08-20, riga 104](../user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md). **Chiuso il 5 ottobre 2026, con questa integrazione:** Daily corrispondente esistente e consolidato; copertura documentale attestata il5ottobre2026. ([evidenza](2026-08-20_daily-report.md)).
- [x] Verificare il trattamento quantitativo della penetrazione nel terminale hard e renderlo comune ai pesi 1 e 2. — [fonte 2026-09-24, riga 308](../user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md). **Chiuso:** Fix del recupero diagnostica terminale dichiarato verificato e concluso. ([evidenza](../user/2026-09-24_fix_contatto_terminale_hard_unified39.md)).
- [x] Verificare biforcazione, dieci update/40.960 transizioni e salute telemetria del ramo peso 2 alla 82. — [fonte 2026-09-24, riga 97](../user/2026-09-24_avvio_left72_penetrazione_peso2_10iter.md). **Chiuso:** Dieci aggiornamenti conclusi; robustezza a policy congelata è separata. ([evidenza](../user/2026-09-24_risultati_left72_penetrazione_peso2_10iter.md)).
- [x] Completare ramo peso 2 alla 100 (20 update/81.920 nuove transizioni), valutare best/last nominali e produrre 14 plot canonici. — [fonte 2026-09-24, riga 22](../user/2026-09-24_avvio_left_best80_peso2_fino100.md). **Chiuso:** Checkbox conclusione e plot chiusi. ([evidenza](../user/2026-09-24_risultati_left_best80_fino100_e_plot_canonici.md)).

<!-- reconciled-report-todos-2026-10-05:end -->
