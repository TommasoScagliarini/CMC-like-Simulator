# Analisi preregistrata random-pose AB06 — best47

Data: 2026-10-13. Instruction check token: `CMC_AGENT_OK_2026`.
Questo documento prepara la valutazione; non attesta esecuzioni o risultati nuovi.
Produzione, plugin C++, profili GRF/detector, FSM, reward e pesi restano invariati.
Sono stati letti direttamente harness, receipt e metriche del 12 ottobre; nessun
contenuto Claude o conclusione consolidata è stato usato.

## Domanda e contratto

Stimare la frazione di **pose protesiche geometricamente ammesse** da cui il
checkpoint AB06 best47 recupera una sequenza di gait valida in cinque secondi,
con il resto del plant inizializzato e guidato dalla stessa IK biologica.
È una prova di sensibilità al reset all'interno di un dominio angolare dichiarato,
non una prova di avvio da stazione eretta, equilibrio statico, generalizzazione
multimodello o robustezza a velocità iniziali casuali.

- Checkpoint congelato: `Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best`.
- Best logical iteration 47 = 46 nuovi update actor dopo il parent logical1.
- Tempo fisico iniziale sempre `13.946870983805102 s`; offset sempre
  `1.956870983805102 s`; durata `5.0 s`, 500 policy step da `0.01 s`.
- Tutte le coordinate biologiche, pelvis inclusa, e le loro velocità iniziali
  sono quelle della IK nominale; i riferimenti biologici per i cinque secondi
  sono identici fra i casi. La dinamica biologica effettiva può rispondere
  diversamente alle pose protesiche e non deve essere artificialmente copiata
  dal controllo durante il rollout.
- Soltanto `pros_knee_angle` e `pros_ankle_angle` cambiano posizione iniziale.
  Le due velocità articolari protesiche sono sempre zero, senza fattori casuali.
- SEA: `motor_angle = joint_angle`, `motor_speed = joint_speed = 0`, quindi
  deflessione e velocità relativa iniziali della molla nulle.
- Governor e riferimento servito vengono inizializzati coerentemente alla posa
  scelta, con stato `q = q_random`, `v = 0`, `a = 0`; memoria filtro C2 e ogni
  stato necessario alla sua realizzazione devono rappresentare tale equilibrio.
  Endpoint precedente/anchor del limiter = `q_random`. Ultimo comando SEA,
  controller e SO seguono i reset standard del banco.
- La lettura protesica IK iniziale non deve reintrodurre il nominale nella
  served al primo step. Dopo il reset la policy congelata genera gli endpoint;
  biologia, GRF primaria e GRF destra prescribed restano sul percorso ordinario.
- FSM/detector si inizializzano dalla geometria della nuova posa dopo il reset
  completo. Nessun HS, TO, credito o ciclo viene inventato per avviare la prova.
  Inferenza deterministica, seed123 solo per riproducibilità; zero update.

Il test `zero_prosthetic_velocity` del 12 ottobre non realizza questo contratto:
modificava quattro speed state fisici ma conservava il governor/served iniziale
IK. Il controllo coerente qui sotto è quindi necessario.

## Campione e ammissibilità prima della dinamica

Campione principale: **20 pose ammesse**, ricavate in ordine da un pool sigillato
di massimo **400 proposte IID uniformi** sul rettangolo:

| Coordinata | Limite inferiore [rad] | Limite superiore [rad] |
|---|---:|---:|
| Knee | -1.20 | -0.03 |
| Ankle | -0.55 | +0.55 |

Congelare prima di qualsiasi outcome il generatore/algoritmo, seed20261013,
versione del runtime, intero pool, ordine e SHA256. I due angoli sono estratti
indipendentemente; nessuna estrazione casuale del tempo IK. Il collector di
geometria deve essere privo di inferenza policy e di integrazione nel tempo.
Accettare le prime 20 proposte che superano tutti i gate; non riselezionare
secondo prima azione, reward, raggiungibilità stimata, equilibrio o successo.
Valutare/salvare la geometria di tutte le 400 proposte per stimare la frazione
ammessa; i rollout riguardano soltanto le prime 20 ammesse. Se ce ne sono meno
di 20, usare tutte quelle ammesse e dichiarare campione inferiore al target,
senza cambiare dominio, soglie, seed o budget.

Gate iniziali congelati:

1. Angoli finiti nel rettangolo e nei range effettivi delle coordinate del
   modello, senza clamp silenziosi; coordinate non bloccate/non prescritte da
   vincoli incompatibili con l'intervento. Non alterare range o modello.
2. Stato e forward kinematics finiti; pose dei componenti e geometria contatto
   valutabili. Un'eccezione del collector è `TECHNICAL_ERROR`, non un'esclusione
   biomeccanica implicita.
3. Massima penetrazione iniziale delle **sfere GRF primarie sinistre**
   `<= 0.027 m`: margine operativo di1mm rispetto al limite dinamico di28mm.
   Il detector force-free non è autorità per questo gate.
4. Se il piano di esecuzione adotta il gate di forza iniziale, congelarlo prima
   del collector: norma della forza applicata primaria `<= 5 BW`, utilizzando
   esattamente massa e gravità del guard runtime. Salvare comunque forza e BW.
   Nessuna nuova soglia o diversa componente viene scelta dopo gli outcome.

La proposta è di adottare il punto4 quando il guard operativo effettivo è5BW,
così l'ammissibilità non autorizza una posa già oltre il guard del banco.

**Pose airborne ammesse:** non imporre un tetto arbitrario di clearance.
Se entrambe le guardie detector sono OFF e le sfere primarie non penetrano,
la posa resta ammissibile nei range del modello. Salvare signed clearance
delle sfere primarie, minima distanza al piano, punti heel/toe, altezza del
piede e flag di contatto/carico. La geometria consentita dal rettangolo e dalla
biologia fissa determina l'intervallo di altezza effettivamente campionato.
Non sostituire airborne con pose caricate. Distinguere contatto geometrico,
carico primario e due bit detector, senza dedurli l'uno dall'altro.

`Admissible` significa soltanto joint/model/finite/contact-geometry ammessi.
Non si effettuano gate di equilibrio statico, self-collision o stabilità futura.
Un cedimento dinamico di una posa ammessa è un FAIL del test e rimane nel
denominatore; non diventa un motivo di esclusione retrospettivo.

## Matrice efficiente: 21 rollout fisici

Prima eseguire un solo controllo `coherent_nominal`: q protesiche nominali,
qd protesiche0, SEA coerenti e governor/endpoint `(q_nominal,0,0)`, stesso
contratto dei 20 random. Poi eseguire tutte le20 pose accettate nell'ordine
sigillato. Il controllo non entra nella stima di successo random.
Le misure nominali del 12 ottobre sono un riferimento storico separato,
senza aggiungere quei casi al denominatore o considerarli controlli coerenti.

Ogni rollout vive in un processo nuovo e una destinazione no-clobber; si può
usare parallelismo limitato dal budget dell'host senza cambiare condizioni.
Un FAIL fisico termina il caso e conserva tutti gli output fino al terminale;
il campione prosegue. Un controllo coerente fisicamente fallito è riportato
separatamente e rende debole l'attribuzione degli eventuali failure alla sola
variazione di q; non giustifica la sostituzione delle pose random.
Mismatch di input congelati, contratto reset non realizzato o errore di unità
bloccano la campagna prima di conclusioni scientifiche. Un errore tecnico
resta tale; qualsiasi correzione richiede nuova versione/leaf, preservando
tentativo originale e identica posa, senza rescue o selezione di seed.

## Verifica reset e integrità

Salvare un receipt immediatamente dopo il reset e prima del primo comando:
q/qd di tutte le coordinate, SEA angle/speed, stato governor completo,
endpoint/anchor, controller/SO/FSM e prime guardie sensoriali. Registrare
richiesto vs realizzato, con errori assoluti massimi `<=1e-10` rad o rad/s
per q/qd/SEA, `<=1e-9 s` per il tempo, e memoria governor q/v/a equivalente
entro la tolleranza dichiarata del suo formato. Il receipt deve mostrare che
solo coordinate protesiche e dipendenze coerenti sono state modificate.

Riferimenti biologici: uguaglianza numerica delle q/qd biologiche iniziali
fra i casi e equality/SHA del riferimento IK generato sulla medesima griglia
temporale. Non esigere uguaglianza della cinematica biologica actual durante
l'integrazione, che è parte della risposta fisica del plant.

Proteggere SHA256 di pesi e checkpoint metadata, modello/setup/IK, plugin,
profili GRF/detector/morphology, configurazioni, sorgenti di produzione e
harness. Verificare prima e dopo ogni caso: zero update, layout35/84,
ordine feature e semantica SEA invariati. Conservare stdout, watchdog,
receipt, summary, trace500step oppure prefisso terminale, tutti i file `.sto`
finiti e i sette plot canonici sotto `plot/random_pose_reset_2026_10_13/<case>/`.

## Criteri quantitativi congelati

Un caso random è `GAIT_RECOVERY_PASS` soltanto se tutti i seguenti criteri
sono soddisfatti; diagnostiche e ritorno non sostituiscono questi gate:

| Gate | Criterio |
|---|---|
| Durata | 500 policy step, tempo finale minus iniziale5s, `episode_time_limit`; nessuna terminazione fisica o timeout watchdog |
| Sequenza gait | almeno2 cicli completi HS→TO→HS validi dalla FSM autorevole, senza credito del bootstrap parziale |
| Recovery temporale | primo HS valido reale accettato/confermato entro3s; salvare onset e conferma separatamente |
| Eventi | invalid_event_count0 e nessun `invalid_event_this_step`, timeout, ciclo rifiutato o degraded; eventuali reject/drop sensoriali espliciti contabilizzati, senza affidarsi al solo summary |
| Contratti | zero morphology causal contract failure e zero fallback non accettati; ogni resync/cancellazione HS viene salvato e riferito separatamente |
| Penetrazione | picco primario sinistro **strettamente<28mm** su tutte le integrazioni1ms, includendo il reset; threshold soft20mm soltanto diagnostico |
| Range dinamici | tutti i q actual rispettano i range effettivi del modello; dominio di campionamento iniziale non diventa un nuovo range dinamico |
| Tracking | RMSE actual-minus-served **<=2° per ciascun giunto** sul segmento `[t0+0.5,t0+5)`; finestra startup `[t0,t0+0.5)` e full-horizon riportate separatamente |
| Forma ciclica | sui cicli completi, knee excursion>=0.60rad e ankle excursion>=0.30rad in almeno due cicli; riportare ogni ciclo, non il solo range globale del transitorio |
| Numeri/output | nessun NaN/Inf in stato/comandi/output e griglie temporali consistenti |

Le soglie di escursione riprendono criteri V26C precedenti; qui sono esplicitamente
applicate ai cicli completi per evitare che la grande escursione di un solo
transitorio di recupero valga come gait. Questa è una nuova qualificazione e
non una reinterpretazione degli esiti del 12 ottobre o J19.

La soglia2° equivale a `0.03490658504 rad`. `.sto` actual/reference sono in radianti
con `inDegrees=no`: calcolare prima RMSE in rad e convertirlo una sola volta.
Allineare per timestamp originali, senza time-shift ottimizzato o resampling
che nasconda la latenza. Per5s gli output correnti registrano gli endpoint
sinistri `[t0,t0+4.999]`:5000 campioni; il runtime receipt verifica5s completi.
Per l'esclusione della prima0.5s usare `elapsed>=0.5` con tolleranza temporale
esplicita, indipendente dagli arrotondamenti dei timestamp salvati.

Salvare anche tempo al primo contatto primario, primoTO, primoHS, primo/secondo
ciclo completo; distanza iniziale dal nominale; RMSE e p95 errore tracking;
range actual/served, clipping, SEA peak/energia/saturazione, reserve perDOF,
carico/impulso primario, return e componenti, margin28mm e tutti i repair FSM.
Separare `RUN_COMPLETED`, `TRACKING_PASS`, `GAIT_PASS` e il composito finale:
un governor ben seguito non dimostra da solo recupero di camminata.

## Statistiche e capacità: denominatori dichiarati

- Pool: `N_proposed=400`, `N_admissible`, `N_excluded`, `N_geometry_error`.
  Ogni riga conserva q proposte, misura geometrica, tutti i gate e ragioni
  multiple; riportare sia conteggi per singolo motivo sia una categoria
  primaria mutuamente esclusiva con priorità congelata.
- Campagna: `N_selected=min(20,N_admissible)`, `N_attempted`,
  `N_technically_valid`, `N_technical_error`, `N_physical_fail`, `N_success`.
  Primo denominatore intenzionale `N_selected`; nessun failure ammesso è
  rimpiazzato. Se esistono errori tecnici irrisolti, non fornire un'unica
  stima scientifica: riportare intervallo pessimistico/ottimistico
  `[success/N_selected,(success+technical_error)/N_selected]`, oltre ai
  risultati descrittivi degli eseguiti validi.
- Con campagna integra, stima primaria `p_hat=success/N_selected`: probabilità
  di successo sotto la distribuzione uniforme del rettangolo **condizionata
  ai gate iniziali**, per questo checkpoint, biologia/timing nominale e qd0.
  L'intero box non è validato dalla sola stima condizionale; riportare a parte
  la copertura `N_admissible/400` e la distribuzione delle pose escluse.
- CI95% binomiale esatto Clopper–Pearson: limite inferiore
  `BetaPPF(0.025; success, N-success+1)` per success>0; superiore
  `BetaPPF(0.975; success+1, N-success)` per success<N, estremi0/1 appropriati.
  Riportare anche il lower bound one-sided95%, con alpha0.05.
- Anche20/20 offre soltanto lower95% two-sided≈83.2% / one-sided≈86.1%:
  non certifica95% di affidabilità o tutte le pose del dominio. Per un lower
  one-sided95%>=95% servirebbero almeno59 successi senza failure in una nuova
  campagna appropriata; non ampliare questa campagna dopo averne visto l'esito.
- Il dominio è **angolare limitato ma continuo**; la lista400 è uno strumento
  riproducibile IID, non una popolazione fisica esaustiva. I CI descrivono
  incertezza Monte Carlo sotto la distribuzione preregistrata. Se si desidera
  stimare invece una popolazione discreta finita, serve un protocollo diverso
  con campionamento senza rimpiazzo e CI ipergeometrico.
- Ripetere una posa deterministica non aumentaN indipendente. Non contare
  policy step, cicli o sample1ms come repliche indipendenti.
- Tutti i20 casi: elenco individuale, scatter2D qknee/qankle success/fail,
  distribuzione penetrazione/height/carico al reset e mappe firstHS/recovery.
  Contact/airborne e quartili angolari sono analisi descrittive predefinite,
  con denominatori e CI per gruppo soltanto dove sensati; nessuna selezione
  post-hoc di un sottodominio promuovibile o test multipli di significatività.
- Tempo di recovery per casi falliti/censurati: riportare `not_reached` e
  ultimo tempo osservato, senza imputare5s o calcolare medie solo sui successi.

## Evidenze iniziali e limiti d'interpretazione

Lettura diretta `rollouts/nominal/metrics.json` del12/10:500step,2cicli,
0invalid, return27.42392079, RMSEknee/ankle0.570515°/0.505732°, picco
penetrazione1ms24.64912mm. Il caso `zero_prosthetic_velocity` riportava
500step,2cicli,0invalid, return27.45147288, RMSE0.570091°/0.506210° e
24.67298mm. Questi numeri sono solo riferimenti del vecchio reset; il nuovo
controllo coerente deve essere misurato nuovamente.

Una coorte20 non misura un bacino di attrazione completo, robustezza hardware,
effetto di un reset diverso sul training PPO o dipendenza dal movimento biologico.
I riferimenti biologici fixed-IK sono parte della condizione sperimentale,
quindi un eventuale PASS resta limitato a questo plant CMC-like ibrido.
Nessun adattamento policy/critic, fine-tuning o modifica SEA è implicito nel piano.

## Fonti verificabili

- `Trajectory Generator/validation/reset_ik_audit_2026_10_12/PROTOCOL.md`: vecchio contratto zero-qdot e distinzione dal gait onset.
- `.../frozen_reset_tests.py:204–300`: validazione input/hash, getter al reset e receipt q/qd/SEA.
- `.../rollouts/nominal/{receipt.json,metrics.json,rollout_summary.json}`: baseline numerica del12/10.
- `.../rollouts/zero_prosthetic_velocity/{receipt.json,metrics.json}`: perturbazione fisica con served/governor IK invariati.
- `simulation_runner.py:337–390`, `:1796–1843`: inizializzazione IK, reset controller/SO e SEA coerenti.
- `Trajectory Generator/osim_trj_cmc_like.py:2476–2609`: diagnostics, tempo/reset, memoria endpoint e prime FSM.
- `output.py:437–447`, `:813–839`: unità actual/reference e salvataggio `.sto`.
- `Trajectory Generator/validation/reset_ik_audit_2026_10_12/distribution/distribution_summary.json`: campionamento storico tre offset, diverso dalla randomizzazione pose proposta.
- `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j19b_runs/j19b_closed_loop_v26c_2026-08-27_r1/v26c_j19b_closed_loop_receipt.json`: precedenti gate kinematic quality, non qualifica odierna best47.

Gli hash delle fonti lette per questo piano sono raccolti in
`analysis_plan_sources.json`. Gli input operativi completi andranno sigillati
dal harness prima dell'esecuzione, senza confondere questa lettura con un freeze
dei futuri artifact numerici.
