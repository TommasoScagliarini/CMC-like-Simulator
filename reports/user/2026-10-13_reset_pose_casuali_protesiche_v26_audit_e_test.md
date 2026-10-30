# Reset da pose protesiche casuali — V26 best47

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 13 ottobre 2026. Instruction check token: CMC_AGENT_OK_2026.
Stato: test, verifiche, audit source e confronto finale Claude conclusi.

La policy congelata recupera la sequenza di gait preregistrata in **18 delle
20 pose casuali ammesse**. Due casi terminano ai limiti operativi: penetrazione
del piede e piccolo overshoot del ginocchio vicino all’estensione. Il controllo
nominale con lo stesso bootstrap supera tutti i criteri. Esiste quindi evidenza
positiva di recupero da numerose pose iniziali diverse, insieme a due
controesempi alla capacità universale di gestire il reset casuale.

Il **90% osservato** ha intervallo binomiale esatto Clopper–Pearson al 95%
**68,30–98,77%**; limite inferiore unilaterale al 95% **71,74%**. La probabilità
è condizionata al dominio ammesso, al modello AB06, al tempo biologico fisso,
alle velocità protesiche iniziali nulle e al banco CMC-like attuale. Non è una
certificazione di affidabilità del 90% o del 95% su tutte le pose.

## Problema e perimetro

L’[indagine del 12 ottobre](2026-10-12_reset_ik_exnovo_audit_indipendente_e_test.md)
aveva verificato start IK diversi e velocità protesiche azzerate nella posa
nominale. Non aveva provato posizioni casuali. L’utente ha confermato di variare
solo ginocchio e caviglia protesici, mantenendo la biologia inizializzata dall’IK.

Si valuta best47 V26C, actor35/critic84, sul modello AB06. Tutte le 19 coordinate
biologiche e le loro velocità iniziali restano IK; successivamente sono integrate
dinamicamente e controllate da muscoli/reserve sulle stesse reference IK. La GRF
destra rimane prescribed e quella sinistra online. Le velocità del resto del
modello non sono nulle: **questa prova non è avvio autonomo da fermo**.

Checkpoint: `MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best`.
Tempo iniziale 13,946870983805102 s; offset 1,956870983805102 s; durata richiesta
5 s, 500 decisioni da 10 ms e campionamento/integratore a 1 ms. Inferenza
deterministica seed123, zero aggiornamenti PPO/critic.

## Strategia e bootstrap coerente

Cambiare solo q fisiche dopo il reset sarebbe insufficiente: endpoint,
reference e governor potrebbero riportare la prima anchor sulla IK. Il nuovo
adapter di sola validazione inizializza con la stessa posa estratta giunti,
rotori, endpoint e governor C2. Le velocità dei due giunti e rotori sono zero,
la deflessione SEA e gli integrali PI sono zero; reference/governor hanno
q=q0, v=a=0. Dopo il primo segmento si usa il governor originale, con continuità
C2 e senza fallback alla IK protesica fuori dal segmento.

Detector e FSM vengono inizializzati dopo la posa effettiva, senza inventare
HS, TO o cicli. Il carico osservato dall’actor a t0 conserva il contratto
originale zero fino alla prima sample; il gate geometrico usa il carico fisico
effettivo. Questo limite osservativo resta visibile, specialmente nei reset
inizialmente caricati, e non viene corretto implicitamente nel test.

Il controllo **coherent_nominal** usa q nominali IK e lo stesso bootstrap
v=a=0 dei random. Il controllo legacy del 12 ottobre è un riferimento storico
riusato, non una nuova esecuzione o una replica aggiunta al denominatore.
Non sono modificati pesi, plugin C++, controllori, reward, profili o guardie.

## Campionamento e criteri prima degli esiti

Pool congelato di 400 coppie IID uniformi, `random.Random(20261013)`:

| Coordinata | Dominio iniziale [rad] | Dominio iniziale [°] |
|---|---:|---:|
| Knee | −1,20 … −0,03 | −68,75 … −1,72 |
| Ankle | −0,55 … +0,55 | −31,51 … +31,51 |

Lo screening senza integrazione o inferenza policy conserva tutti i candidati,
ordine e ricevute. Richiede ROM/stato/geometria finiti, penetrazione primaria
LEFT ≤27 mm e norma della forza primaria ≤5 BW. Il limite episodio di 28 mm
e la guardia fisica a 30 mm restano quelli originali. Non si usa il piano
sensor-only del detector come autorità del contatto di forza.

**364/400** pose sono ammesse; **36/400** respinte per penetrazione iniziale,
di cui 30 già oltre 30 mm. Nessun rifiuto per forza. Si eseguono le **prime
20 ammesse nell’ordine sigillato**, senza sostituire i failure dinamici. Non si
esclude AIR per un limite arbitrario di clearance. Ammissibilità non certifica
equilibrio statico, self collision o realismo umano delle penetrazioni.

Tra le 20 pose, 15 hanno tutte le sfere primarie sollevate e 10 hanno entrambi
i punti detector OFF. Cinque pose senza contatto primario hanno invece heel
o toe detector ON: piani e geometrie dei due stream sono diversi. Non sono
heel strike sintetici; tutti i contatori reset HS/TO/cicli sono zero.

Un recupero qualificato deve rispettare contemporaneamente:

- 500 step / 5 s, normale `episode_time_limit`, nessuna terminazione;
- almeno due cicli actor validi HS→TO→HS e primo HS confermato entro 3 s;
- zero eventi invalidi, cicli rifiutati/degraded, timeout, resync e HS cancellati;
- penetrazione primaria strettamente <28 mm su tutte le sample native, incluse
  le evidenze terminali, e guardie/range operativi rispettati;
- RMSE actual–served ≤2° per giunto dopo i primi 0,5 s;
- almeno due distinti cicli completi, ciascuno con escursione actual knee
  ≥0,60 rad e ankle ≥0,30 rad. Il transitorio iniziale non vale come ciclo.

Return, clipping e reserve sono diagnostiche separate. Criteri, codice di
classificazione e denominatori vengono sigillati prima dei nuovi esiti.

## Risultati

| Esito random | Numero |
|---|---:|
| Durata completa e guardie rispettate | 18/20 |
| Recupero gait secondo tutti i criteri | 18/20 |
| Terminazioni ai limiti operativi | 2/20 |
| Errori tecnici o misure irrisolte | 0/20 |

Il controllo coerente completa 5 s, due cicli qualificati, zero eventi invalidi,
primo HS confermato a 1,648 s, penetrazione massima 24,690 mm e return 27,317.
RMSE actual–served dopo 0,5 s: knee 0,595°, ankle 0,509°.

Nei 18 recuperi: nove casi completano due cicli e nove tre cicli; primo HS
confermato fra 0,130 e 1,650 s. RMSE actual–served dopo 0,5 s: knee
0,599–0,670°, ankle 0,454–0,586°. Anche pose con ginocchio distante oltre
50° dal nominale recuperano la sequenza. Nessun clipping policy nei 18 PASS.

Nell’ultimo secondo, actual rispetto al controllo sul medesimo clock:
knee RMSE 0,012–2,886° e ankle 0,008–0,993°. Il recupero dei cicli non
implica convergenza esatta alla traiettoria nominale.

**random_006** parte da knee −0,167941 rad e ankle +0,375195 rad, con
penetrazione iniziale 23,556 mm, ammessa dal gate. La guardia fisica misura
**30,129935 mm a 0,319 s** dal reset. Il boundary logico rimane a 0,310 s:
32 reward realmente restituiti, 31 segmenti completati e nove sample native
del segmento interrotto. Il summary riporta `morphology_causal_contract_failure`,
mentre `failure` e `terminal_contact` nel journal testimoniano direttamente
`OnlineGRFPenetrationLimitExceeded`. La causa fisica non viene eliminata dal N20.

**random_025** parte da knee −0,045479 rad e ankle −0,053361 rad. Termina
a **0,140 s** per `joint_divergence:pros_knee_angle`: knee terminale
**+0,00043687 rad, cioè +0,02503°**, oltre la guardia operativa superiore 0.
La reference servita è −0,00331233 rad; l’errore di tracking è circa 0,215°.
Il valore resta dentro il ROM dichiarato del modello ±π/2. È un piccolo
overshoot operativo, non prova di instabilità catastrofica o caduta. Il raw
knee supera +1 in 13/14 decisioni, massimo 1,05793, ed è clippato dal wrapper
originale. Il failure preregistrato resta tale; non si prova un rescue o un
allentamento della guardia.

Sette dei 18 PASS hanno return negativo; intervallo −109,819 … +61,381,
mediana 26,214. Questi sette partono con detector AIR e completano tre cicli;
hanno maggiori morphology loss registrate rispetto al controllo. È una
associazione descrittiva fra startup, eventi e loss, senza ricostruire reward
controfattuali o attribuire un effetto causale sul learning.

Il supporto biologico pesa sul recupero. Nei PASS, la norma della forza
residuale pelvis allo startup arriva a RMS 386,88 N contro 227,28 N del
controllo (1,70×) e picco 729,24 N contro 369,71 N (1,97×). Nell’ultimo secondo
il massimo RMS è 118,17 N contro 105,39 N (1,12×). Le coppie delle reserve
rotazionali sono analizzate separatamente in Nm. La vicinanza della risposta
biologica al controllo non significa tracking IK perfetto: il controllo
stesso conserva errori, fra cui MTP destro circa 13,97° RMS dopo 0,5 s.

![Pool delle pose ed esiti](../../plot/random_pose_reset_2026_10_13/summary/pose_pool_and_recovery.png)

Tabella individuale e [CSV completo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-d9909e593723c658>) accompagnano gli
[esiti sigillati — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-610d04754a4291c7>).

| Caso | q0 knee / ankle [°] | Durata logica [s] | Cicli validi | Picco pen. [mm] | Esito |
|---|---:|---:|---:|---:|---|
| random_003 | -46.77 / -1.66 | 5.000 | 2 | 24.761 | PASS |
| random_004 | -40.94 / 7.45 | 5.000 | 2 | 24.762 | PASS |
| random_005 | -28.91 / 20.65 | 5.000 | 2 | 24.676 | PASS |
| random_006 | -9.62 / 21.50 | 0.310 | 0 | 30.130 | Penetrazione |
| random_007 | -63.82 / -17.57 | 5.000 | 3 | 23.485 | PASS |
| random_008 | -20.04 / 20.41 | 5.000 | 2 | 24.691 | PASS |
| random_009 | -61.75 / -15.76 | 5.000 | 3 | 22.981 | PASS |
| random_010 | -60.11 / -3.04 | 5.000 | 3 | 22.644 | PASS |
| random_013 | -29.49 / 6.84 | 5.000 | 2 | 24.687 | PASS |
| random_014 | -46.93 / -5.21 | 5.000 | 2 | 24.774 | PASS |
| random_015 | -23.66 / 14.31 | 5.000 | 2 | 24.687 | PASS |
| random_016 | -37.25 / 27.02 | 5.000 | 3 | 23.053 | PASS |
| random_017 | -31.15 / -1.56 | 5.000 | 2 | 24.678 | PASS |
| random_018 | -38.05 / 24.70 | 5.000 | 3 | 22.985 | PASS |
| random_019 | -52.39 / 14.64 | 5.000 | 3 | 22.595 | PASS |
| random_020 | -48.24 / 21.38 | 5.000 | 3 | 22.703 | PASS |
| random_021 | -22.66 / 20.08 | 5.000 | 2 | 24.688 | PASS |
| random_022 | -61.89 / 9.07 | 5.000 | 3 | 23.380 | PASS |
| random_023 | -65.67 / 30.64 | 5.000 | 3 | 23.572 | PASS |
| random_025 | -2.61 / -3.06 | 0.140 | 0 | 0.000 | Limite knee |

La durata della riga 006 è il boundary logico; il contatto fisico terminale è a 0,319 s.

## Audit indipendenti e confronto

Codex ha sigillato le conclusioni di sorgente prima di leggere Claude; anche
il piano statistico e il classificatore sono stati preparati senza nuovi
risultati dinamici. Claude è stato invocato da terminale con modello esatto
`claude-opus-5-5`, `--effort max`, senza fallback. L’audit source usa soltanto
Read/Grep/Glob; il confronto numerico finale non dispone di tool. Input,
risposte, modello effettivo e ricevute sono conservati.

Entrambi gli audit source concordano sul bootstrap coerente e sul controllo
nominale necessario. Claude identifica anche il problema della forma flat
dell’azione nel probe; la correzione usa il wrapper originale, con reshape
(1,2) e clipping, e viene verificata prima del freeze e nativamente. Le sue
altre osservazioni su count8, selezione assente e batch interrotto riguardano
una versione intermedia; il codice finale fissa pool400/first20, ricevute
hashate, trace incrementale, continuità, batch completo e postprocessing
separato dall’esito fisico.

L’ipotesi che due cicli siano impossibili in 5 s a causa del periodo GRF
1,669 s non è confermata: il controllo e tutti i 18 PASS ne completano almeno
due. Il periodo misurato della GRF non impone quello degli eventi actor.
Il bacino è dinamicamente integrato con tracking, non cinematicamente fissato;
«non può cadere» sarebbe quindi una conclusione troppo forte. Ogni eccezione
non diventa automaticamente un failure fisico: timeout e difetti del harness
sono tecnici, mentre guardie fisiche documentate restano failure del test.

Il confronto finale è concluso con lo stesso modello effettivo Opus 5.5 ed
effort massimo. Claude conferma conteggi, denominatori, CI e validità
condizionale dei 18/20 recuperi. Considera legittima la correzione reader V2:
nessun PASS dipende dal terminale zero-time e il failure006 non può diventare
un successo. Distingue anch’esso il piccolo overshoot025 da una caduta o da
una violazione del ROM del modello.

I due failure sono le due pose con ginocchio più esteso nel campione; tutti
i PASS hanno q0 knee≤−0,349 rad. È una descrizione post hoc di soli due punti,
con ankle e contatto iniziale diversi, non un confine di sicurezza identificato.
Claude sottolinea anche la scarsa copertura realizzata della caviglia negativa
e la differenza fra startup detector AIR e detector ON. Non si promuove un
sottodominio selezionato dopo gli esiti.

Una sua ipotesi finale di margine temporale circa 13 ms, con possibile failure
comune per uno shift di 1 ms, viene verificata sui journal e **non confermata**:
nel controllo gli HS sono confermati a 1,648, 3,187 e 4,735 s; il secondo ciclo
è quindi chiuso **265 ms** prima del limite 5 s. I nove PASS detector ON hanno
lo stesso HS3 a 4,735 s. Non si usa il periodo GRF 1,669 s come se fosse il periodo
actor. L’[adjudication con tutti i timestamp — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-142ba8cce3baa885>) conserva questa correzione.

La sintesi condivisa è il recupero della sequenza operativa nel sistema
chiuso per 18/20 pose ammesse, senza sostituzioni, a un solo timing biologico.
L’espressione «ampio bacino di attrazione» viene limitata a evidenza positiva
su pose distribuite nel dominio: venti punti non mappano un bacino completo.
La convergenza è molto vicina al controllo nei PASS detector ON, parziale
nei PASS detector AIR. Non sono dimostrati contributo esclusivo del feedback,
equilibrio, box intero, affidabilità ≥95% o un effetto causale sul learning.

Il 90% resta una stima **condizionata agli ammessi**. Le moltiplicazioni
illustrative con il 91% di ammissibilità non costituiscono un test della capacità
di recuperare le 36 pose escluse. Il confronto con Claude è una critica della
capsula numerica; Claude non ha rieseguito i rollout né ispezionato i raw
nel confronto finale. Le verifiche raw sono eseguite separatamente da Codex.

Risposte complete: [audit source Claude — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-0acffef99124b01f>),
[confronto finale Claude — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-dc025c313c648b99>),
[audit Codex — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-40c691e84a135428>) e
[verifica indipendente degli output — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-18f5331ee5261ab7>).

## File e verifiche

Tutto il nuovo codice di validazione vive in
`Trajectory Generator/validation/random_pose_reset_2026_10_13/`:

- `random_pose_reset.py`, `bootstrap_probe.py`, `run_selected_campaign.py`:
  reset coerente, pool/manifest, selezione e processi nativi isolati;
- `test_bootstrap.py`: 11 test offline PASS, incluso governor originale,
  continuità fra segmenti, divieto di fallback IK e forma/clipping del probe;
- `aggregate_results_v2.py`, test e seal: 20 test offline PASS, criteri
  invariati, CI esatti e contabilità corretta del terminale;
- `compact_diagnostics.py`, test e summary: cinque test offline PASS,
  eventi/clipping/reserve e confronto sul medesimo clock;
- `verify_preflight.py`, `review_bootstrap_preflight.py`, `verify_saved_outputs.py`:
  letture indipendenti di geometria, bootstrap e artefatti numerici;
- `claude_audit.py`, `prepare_claude_comparison.py`, `claude/`: invocazioni e
  audit/confronto; `plot_campaign.py`: overview scientifiche esportabili;
- protocollo, audit, seal, pool400, 401 receipt statici, quattro probe nativi,
  21 rollout, CSV, tabelle e risultati JSON.

Il reader V1 e il relativo seal/test restano immutabili, anche in archivio.
V2 corregge soltanto la lettura del guard mascherato e del return terminale
senza avanzamento; non cambia soglie, reward, selezione, policy o esiti nativi.
La revisione del verificatore usata nella capsula Claude è conservata con
SHA identico all’input, ricostruita da sole aggiunte documentate; la versione
finale aggiunge prove e riepiloghi senza cambiare i risultati preesistenti.

Verifiche effettive: **36 test offline** (11 bootstrap +20 reader V2 +5
diagnostiche), **401 reset statici**, **quattro first-segment probe nativi
senza integrazione**, **21 rollout**. Tutti i **357 STO** sono finiti e con
header/griglie coerenti; reward, journal, reference e bootstrap coincidono.
Le reference biologiche sono identiche byte per byte sui prefissi comuni;
le risposte biologiche actual differiscono fra i casi, coerentemente con
l’integrazione libera. Tutti i **76 input congelati**, compresi i 73 della
campagna precedente, restano invariati. Zero clamp motore ±500 nei canali
plugin effettivamente registrati, distinto dal clipping dell’azione policy.

I **147 plot canonici** sono sotto root
`plot/random_pose_reset_2026_10_13/<case>/`; ulteriori overview PNG/PDF sono
in `summary/`. Sette immagini sono ispezionate visivamente. Nei failure, le
figure di ciclo senza cicli validi mantengono esplicita l’assenza di dati.
I plot mostrano il knee con flessione positiva; i calcoli usano le coordinate
STO originali in radianti. Le prove native attestano macOS arm64; gli script
usano path e subprocess portabili, senza certificare una prova live Windows.

## Interpretazione e attività aperte

Il risultato risponde positivamente alla capacità di recuperare **molte**
pose casuali protesiche nel banco congelato, e negativamente alla sua capacità
universale. Non identifica un effetto causale del reset IK sull’apprendimento:
non si sono confrontati training PPO matched con distribuzioni iniziali diverse.
L’evidenza riguarda policy, governor, controller SEA e supporto biologico
insieme; il contributo esclusivo del feedback neurale richiede un replay
open-loop delle azioni nominali sulle stesse pose, ancora non eseguito.

Restano aperti RIK-T01 (confronto PPO matched), RIK-T02 (bootstrap causale
completo e carico iniziale) e V26-T04 (avvio da fermo dell’intero modello).
Altre fasi biologiche, velocità/precarichi casuali, altri soggetti, orizzonti
lunghi, equilibrio e hardware richiedono protocolli separati.

- [ ] **RPR-T01** — Conservare la causa fisica prioritaria nel wrapper reward
  V26 quando il contratto morphology fallisce dopo un segmento interrotto;
  qualificare separatamente la correzione di `end_reason`, senza cambiare
  reward, guardie o gli artefatti congelati di questa campagna.
