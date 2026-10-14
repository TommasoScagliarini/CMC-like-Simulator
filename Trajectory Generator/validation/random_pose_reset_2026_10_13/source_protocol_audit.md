# Audit indipendente Codex del protocollo random-pose reset — 2026-10-13

Instruction check token: CMC_AGENT_OK_2026

Audit di sorgenti e metodo, senza test nativi, training o modifiche alla produzione. Non sono stati letti gli esiti Claude o root del 13/10. Sono stati consultati soltanto il protocollo e il sorgente del harness congelato del 12/10 come riferimento metodologico. Nessun esito del 12/10 entra nelle conclusioni qui riportate.

## Ambito e limite

AB06, checkpoint congelato `MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best` (identificazione best47 ricevuta dal task, non deserializzata indipendentemente). Tempo iniziale fisso `13.946870983805102 s`, orizzonte deterministico 5 s, resto dei parametri/pesi/plugin invariato. Varia soltanto la posizione iniziale di `pros_knee_angle` e `pros_ankle_angle`, nel ROM preliminare `[-1.20,-0.03]` e `[-0.55,+0.55] rad`. Velocita fisiche protesiche nulle; biologia q/qdot dalla IK allo stesso tempo iniziale. Il riferimento protesico parte con q casuale e v/a nulle. L'accelerazione **fisica** resta un risultato della dinamica: non va imposta a zero.

La prima lettura non disponeva ancora di un harness random-pose del 13/10. Una successiva revisione source della prima versione di `random_pose_reset.py`, `bootstrap_probe.py` e `test_bootstrap.py` e riportata in fondo. Nessun esito esecutivo e stato letto e il presente audit non attesta un GO nativo.

## Bootstrap senza ritorno alla IK

Patchare soltanto q/qdot fisiche dopo il reset ordinario non basta. `clear_segment()` elimina spline e stati del reference model; `get(t)` senza spline ritorna la IK. Anche il primo `_action_to_segment()` legge `kin.get(t)` come anchor di continuita e usa `_last_policy_endpoint` come anchor dello slew limiter, inizializzato durante il reset da `kin.get(t)`.

Evidenza nel sorgente `Trajectory Generator/osim_trj_cmc_like.py`: `clear_segment` L532–539; `get` L826–843; reset L2565–2600; primo anchor/slew L2683–2705. Il reference model ordine 3 inizializza q/v dalla prima spline e a a zero (L707–728), poi mantiene continuita q/v/a tramite `BPoly.from_derivatives` (L809–814).

La sequenza corretta per un adapter di validazione e:

1. Congelare tempo, config, checkpoint e manifest delle pose; eliminare memoria dell'episodio precedente.
2. Prima di `runner.reset_to_time(t0)`, installare un bootstrap protesico che restituisca q casuale, qdot=0, qddot=0 per i due giunti, lasciando ogni coordinata biologica e derivata delegata alla IK. Il bootstrap deve rimanere valido durante tutte le letture senza spline fino all'installazione del primo segmento policy. Una hold spline coerente puo svolgere questa funzione; non basta una sostituzione temporanea del solo `get` usato dal runner.
3. Lasciare al reset esistente l'inizializzazione coordinata: q/qdot effettive, `motor_angle=q`, `motor_speed=0`, stati PI/integratori e controlli azzerati. La molla parte senza precarico. Un precarico costituirebbe un secondo fattore sperimentale.
4. Allineare governor q=q casuale, v=a=0 e `_last_policy_endpoint=q casuale` **prima** della prima osservazione e del primo action mapping. Se un segmento hold termina prima dell'ultima lettura del reset, `get` puo nuovamente cadere sulla IK: deve coprire il necessario oppure usare un override persistente.
5. Campionare e inizializzare baseline detector e FSM dalla posa definitiva, dopo che il bootstrap fisico e SEA e coerente; azzerare anche gli event/morphology ledgers tramite il reset ordinario dei wrapper.
6. Registrare osservazione actor, prima raw action, anchor della continuita, anchor dello slew e riferimento servito a t0 e t0+1 ms. La prima azione puo cambiare il target; l'uscita del governor deve partire dalla posa casuale con continuita, senza salto alla IK.

Evidenza `simulation_runner.py`: reset legge `self._kin.get` e applica q/qdot L335–357; azzera controlli/memorie SO/controller L359–381; campiona t0 L382–396; `_init_muscle_states` azzera plugin states e allinea motore L1762–1845; `_set_state` L2645–2674.

Non-protesici delegati alla IK: `osim_trj_cmc_like.py` L453–458 e L826–843. Il controllo biologico continua a leggere la reference ad ogni finestra: `simulation_runner.py` L864–870. E quindi un test di posa protesica iniziale casuale dentro una simulazione biologica gia in cammino; non valida avvio da fermo dell'intero corpo.

## Pose ammissibili e geometrie distinte

Usare un gate **prima dei rollout**, indipendente dalla policy: valori finiti, ROM di proposta esplicito, bounds/model constraints coerenti, stato realizzabile, e penetrazione t0 massima delle sfere **primarie APPLIED** sinistre `<=0.028 m`. Registrare anche la distanza dalla soglia per i casi borderline. Non abbassare o alzare questa soglia dopo gli esiti. Il guard nativo a 30 mm rimane distinto dal controllo env a 28 mm.

Il ROM preliminare e interno agli action bounds knee `[-1.5,0]`, ankle `[-0.7,+0.7]` e ai guard articolari knee `[-2,0]`, ankle `[-0.9,+0.9]`: `osim_trj_cmc_like.py` L205–215 e L309–318. Il modello `.osim` definisce range circa ±pi/2 e knee `clamped=false`: il range dichiarato non sostituisce un gate fisico.

Per ogni sfera primaria, nella posa candidata:

`penetration = max(0, radius - dot(world_center - plane_origin, unit_normal))`.

Il profilo primario `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json` ha otto sfere sinistre, alcune con coordinate duplicate, e ground y=`-0.009503954760384832 m`. Preservare tutti gli elementi originali e usare i transform OpenSim dello stato candidato, non approssimazioni della sola caviglia. Evidenza C++ `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp` L40–59 e L87–106; la penetrazione registrata e quella fisica non smussata, mentre la forza usa smoothing.

Il detector V25 e **force-free**: due station heel/toe, ground y=`0.0148208231 m`, bit `clearance<=0`. La sua plane e 24.324777860384833 mm sopra quella primaria. Un bit detector e un carico fisico possono quindi discordare. Il detector non serve a stabilire penetrazione ammissibile delle sfere applicate. Evidenza `binary_phase_detector.py` L1–6, L297–313 e L361–402; profilo `validation/binary_phase_detector_v25_geometry_runs/2026-08-04_local_reach_sweep_dev02_04_08/selected_candidate_profile.json`.

STL visuali e detector analogico rimangono diagnostiche separate, senza trasformarli retroattivamente in gate. Le STL non costituiscono la geometria delle forze applicate in questo runtime. Se si vuole un criterio di intersezione mesh, bisogna congelare prima modello, transform e soglia, presentandolo come vincolo aggiuntivo. Le pose AIR sono ammissibili per il test di recupero contatto: escluderle renderebbe il test condizionato a supporto iniziale e ne cambierebbe l'obiettivo.

Gate env: `osim_trj_cmc_like.py` L4406–4420 e L4592–4624. Il max concerne soltanto i lati online applicati. Guard 30 mm: `config.py` L63 e trasporto terminale `osim_trj_cmc_like.py` L1924–1929.

## FSM: niente HS sintetico al reset

La baseline va campionata dalla posa random definitiva. AIR lascia `WAIT_HS`; un qualunque contact word avvia una partial stance senza valid-HS, last-valid-HS sintetico o credito ciclo. Il primo TO di una partial stance e reale, ma non crea da solo una sequenza completa HS→TO→HS.

Evidenza: reset/prime `osim_trj_cmc_like.py` L2592–2611 e L3203–3267; `binary_phase_adapter_v26.py` L40–78; `prosthetic_phase_fsm.py` L243–279 e L901–920. Riutilizzare il t0 IK detector dopo aver sostituito lo stato fisico sarebbe un errore di bootstrap.

Distinguere: t0 word e carico; startup partial stance; primo HS reale; primo TO reale; prima sequenza completa con HS reale iniziale; resync/degraded events; cicli validi e periodo/frazioni stance-swing. Nessun conteggio basato soltanto su t0 contact o TO→HS da bootstrap vale come nuovo ciclo completo.

## Dipendenze prescribed che restano

Ex-novo non espone healthy imitation target all'actor; sound gait clock disabilitato. q fisiche e memoria del riferimento servito rimangono ovviamente influenzate dal reset scelto. Il critic conserva i kin_ref biologici. Il controllore biologico e la GRF destra seguono dati prescribed; la GRF sinistra e online applied e il loader disabilita automaticamente la prescribed sinistra. La morphology e ancora un prior cinematico derivato da AB06.

Evidenza: config risolta del run; actor/critic `osim_trj_cmc_like.py` L2992–3007 e L3054–3108; slicing della rete `baseline_MLP/asymmetric_rl_module.py` L143–181; hybrid `model_loader.py` L926–945 e L1020–1084. Il random reset elimina il bootstrap protesico IK, non ogni dipendenza prescribed della simulazione o del training precedente.

## Proposta di campionamento preregistrato

Questa e una proposta da riconciliare con il protocollo esecutivo 13/10, non un risultato o obbligo di numero casi.

- PCG64 seed `20261013`, campionamento uniforme del rettangolo ROM, pool fisso di 256 proposte; scegliere in ordine le prime 24 geometricamente ammissibili. Manifest completo delle proposte con q, quantities del gate, motivo di esclusione e word detector. Se meno di 24 eleggibili, dichiarare insufficienza senza modificare ROM/gate dopo risultati.
- Congelare il manifest prima di chiamare la policy. Non filtrare per raw action, reward, HS/TO, survival, tracking o reserve future. Non sostituire fallimenti con nuove pose favorevoli.
- Riportare proposta e distribuzione condizionata all'ammissibilita: rejection fraction, bin 4×4, coverage e word/contact strata. L'eventuale regione eliminata dal gate geometrico limita esplicitamente il dominio delle conclusioni.
- Controlli paired: (A) IK q/qdot originarie e reset congelato; (B) stessa q IK con velocita protesica nulla e governor/FSM allineati; (C) pose casuali con medesimo bootstrap di B. Il confronto B/C separa effetto posa dall'effetto velocita/bootstrap.
- Policy deterministica, stesso tempo, biologia/GRF/config/checkpoint e guard. I repeat verificano riproducibilita tecnica; non rimuovono dal denominatore il caso fallito.

## Assi di valutazione separati

- **Completamento:** 500 step policy/5 s e `episode_time_limit`; distinguere task failure, numerical failure, native crash, watchdog e reset failure. Durata completata non equivale a cammino valido.
- **Eventi/cicli:** transizioni reali con onset/confirmation/delivery, bootstrap partial stance distinto, tempo al primo ciclo completo, count validi/resync/degraded, periodi e stance/swing. AIR: tempo al primo contatto e al primo HS verificato.
- **Tracking:** actual-versus-served RMSE/max per giunto, startup 0–1 s e segmento 1–5 s dove disponibile; jump iniziale, v/a/jerk e limiter fraction. La IK imitation RMSE non e il target di successo ex-novo.
- **Contatto:** primaria applicata a 1 ms, picco guard/penetrazione, Fy/BW, impulsi, slip e COP. Detector word e contatto fisico distinti.
- **Reserve:** root versus muscle-capable reserve, RMS/p95/max e tracking biologico. Reserve elevate qualificano validita biomeccanica indipendentemente dal completamento.
- **SEA:** u saturation, clamp interno tau_input ±500 Nm, tau_spring, motor speed/accel/power e integrali. Finitezza, stress e tracking sono giudizi separati.

Denominatore: tutti i casi ammessi congelati e tentati. Conservare failure receipts e dati parziali anche per eccezione al reset/primo step, crash e hard guard. Nessun confronto aggregato deve equiparare traiettorie brevi fallite a quelle complete o limitarsi ai survivors senza dichiararlo. Presentare matrice per caso, t/reason di failure e statistiche post-startup condizionate quando mancano 5 s. Flush e watchdog devono conservare ultimo timestamp fisico/segmento/log, non sostituire il campione terminale con un valore ipotetico.

## Test causali minimi e revisione implementation

1. Invarianti passive, senza integrazione: readback random q, served q/v/a, motor e biologici, t0 detector/FSM, poi costruzione primo segmento con continuita e nessun rebound IK.
2. Controlli A/B/C sopra, policy congelata, manifest held-out e regime iniziale/post-startup separati. Misura sensibilita del checkpoint a nuove pose; non attribuisce un miglioramento del training alla randomizzazione.
3. Prima del GO dell'implementation 13/10: verifica ordine transaction reset, first action, gate primary geometry, manifest congelato e failure capture. Non e consentito aggirare un fallimento aumentando gain/limiti o modificando plugin/policy.

Un confronto di training random-reset versus IK-reset richiederebbe PPO paired, inizializzazione comune, budget/seed/reward uguali e valutazione held-out. I soli rollout di best47 non risolvono tale domanda.

## Addendum: prima implementation 13/10, sola lettura

Versione sorgente letta prima di esiti, riferimenti a `random_pose_reset.py`:

- L41–101: adapter coerente. Il bootstrap restituisce q casuale e v/a zero prima del primo segmento; inizializza gli stati del governor, preserva i biologici da base IK e rifiuta l'uscita da un segmento servito senza tornare a protesi IK. L94–101 verifica q/v/a del primo segmento. Nessun plugin modificato.
- L171–226: adapter installato su env e runner prima del reset esplicito. Reset ordinario allinea motori/controller e campiona FSM dalla nuova posa. Asserzioni verificano biologici invariati, pose/rotori/served/endpoint e contatori evento iniziali nulli. Lo schema actor/full e confrontato prima del rollout (L287–292).
- L136–164: gate statico usa primaria APPLIED, forza massima normativa esistente, penetrazione nativa30mm/env28mm e altezza pelvis esistente. La norma forza coincide con il guard runtime `simulation_runner.py` L1637–1655. Non filtra AIR o per risultato policy. Nessuna mesh/detector confusa con supporto applicato.
- L114–133: candidati uniformi e seed espliciti congelati prima di integrazione; le rejections restano in ordine. Il numero default8 e una scelta CLI, non prova di adeguata potenza; la versione effettiva del manifest dovra esplicitare N e popolazione.
- `test_bootstrap.py` costruisce la classe governor originale dal suo AST e verifica biologia/continuita/fallback senza import nativo. Questi test non sono stati eseguiti dall'auditor; il loro sorgente copre correttamente il rischio identificato.

Gap da correggere/verificare prima di congelare l'esecuzione:

1. Il manifest L120 ha soltanto il controllo IK q/qdot originale. Manca il controllo q IK con velocita protesica zero e bootstrap governor/FSM coerente. Senza tale B, una differenza random-versus-legacy mescola posa, velocita e memoria del riferimento. La posa di controllo B puo derivare da IK allo stesso tempo senza richiedere appartenenza al ROM dei candidati, dato che e un controllo e non una proposta casuale.
2. `main` L427–428 ritorna subito al primo worker crash/watchdog. Ogni caso preregistrato va tentato o almeno esplicitamente marcato non tentato per una ragione tecnica globale; il fallimento locale non deve impedire la raccolta degli altri independent cases. Raccogliere exit/status per tutti e restituire un exit aggregato alla fine.
3. Il dict diagnostico iniziale e assegnato a `receipt` solo L221, dopo le asserzioni L205–220. Un assertion failure perde i valori appena calcolati. Salvare il snapshot prima delle asserzioni (con stato `bootstrap_unvalidated`) e conservare check values/motivo su failure. Analogamente, il native crash al reset puo non raggiungere tale snapshot: supervisor deve conservarne stage/log/timeout reason.
4. Il postprocessing L345–350 puo propagare un `RuntimeError` plotter e diventare `harness_or_native_failure` L354–357 pur dopo un rollout fisico concluso. Separare status fisico dal postprocessing; il check sette plot deve restare verificabile, senza ri-etichettare una conclusione fisica come fallimento nativo.
5. Il capture L299–310 registra i returned rewards dopo ogni step e il supervisore L421–426 conserva crash/watchdog. Per hard penetration deve conservare anche timestamp/campione fisico nel payload `terminal_contact`, senza affidarsi soltanto al boundary policy. Questo e una verifica di completezza degli artifact, non un'affermazione sugli esiti.

Le revisioni richieste sono del solo harness di validazione. Possono essere controllate con nuova lettura/hash prima dei rollout senza alterare produzione, plugin o checkpoint.

## Review finale source, sigillata prima dei nuovi esiti nativi

Hash riletti direttamente dall'auditor:

| File nella presente cartella | SHA256 |
|---|---|
| `random_pose_reset.py` | `7a218968860ce58977f6b112fae6e105faff787f04c2a89d5340f9cda4d6aee5` |
| `bootstrap_probe.py` | `792e6647d5222e421acda2bd01e67321bc6e2ff45424aac059412109002c62e8` |
| `test_bootstrap.py` | `799a91e64ab445a2042205f02a53c5f70c5b53bfc68f8796e1c6e7b6843a08b2` |

Nessun receipt/outcome preflight o rollout del 13/10 e stato letto. L'auditor non ha eseguito i test, ne import nativi. I finding su bootstrap, snapshot, prosecuzione batch, postprocessing, timeout e binding della selection sono stati derivati indipendentemente dai sorgenti prima della revisione finale. Il difetto di shape della probe e stato comunicato dal root come finding Claude dopo il sigillo precedente; qui si verifica soltanto il fix nel sorgente, senza rivendicarne l'origine indipendente e senza leggere output Claude.

Nella versione finale:

- **Gate reale:** pool400 e primi20 eleggibili, seed fisso; penetrazione statica massima27mm e force guard5BW, AIR senza cap. La margin27mm e una scelta preregistrata del protocollo finale, piu conservativa della proposta iniziale28mm di questo audit. Runtime28mm/nativo30mm restano distinti. Evidenza `random_pose_reset.py` L115–170. La proposta24/256/PCG64 precedente in questo documento e superseduta dal protocollo esecutivo20/400/`random.Random`; rimane traccia del suggerimento indipendente iniziale.
- **Wrapper/reset/FSM:** adapter persistente q random/v=a0 L42–102; reset L218–288 e wrapper `RewardShapingWrapper.reset` in `baseline_MLP/reward_function.py` L1472–1491. Tutta la memoria reward/contact/morphology si resetta attraverso il wrapper originale, poi il runner campiona e prime la FSM dalla posa random. Snapshot e tempo verificato prima delle assertions L258–264; motori, ausiliari SEA e PI verificati L268–282. Non risultano scorciatoie alla IK o eventi sintetici nel source.
- **Controlli:** `coherent_nominal` deriva q dalla IK e usa lo stesso bootstrap zero-v dei random (L121 e L222–231). E quindi presente il controllo B necessario. Il controllo A legacy viene documentato come riuso degli artifact congelati del 12/10, non come nuovo run in questo manifest. Qualunque confronto A/B deve riportare tale provenance e verificare l'identita degli input congelati; questo audit non legge o confronta i relativi esiti.
- **Prima azione probe:** `apply_wrapped_action` L319–327 usa esattamente `FlattenClipAction.action` della produzione, con reshape/clip prima della pianificazione diretta L479–487. Non modifica il percorso reale `env.step` dei rollout.
- **Selection integrity:** L340–370 pretende il set completo dei receipt, verifica hash e case contro il pool, status e gate concordi, poi ricostruisce ordine/outcomes e primi20 dal pool. Non si affida piu alla lista outcomes modificabile. Identita della selection trasportata ai worker L628–631 e L655. Possono essere eseguiti subset operativi tramite `--cases`, ma il report finale deve coprire tutti i21 casi selezionati o indicare esplicitamente i non tentati.
- **Failure fisico/tecnico:** timeout wall e `segment_wall_clock_timeout_censored` L181–182; forza, penetrazione, numerical e harness failure restano categorie distinte L177–187. Le summary censurate L190–215 includono soltanto reward/transizioni realmente restituite; nessun credito per la transizione interrotta. `rollout_complete` significa ritorno normale dall'evaluator: il flag `completed_requested_duration` L495–497 e l'end_reason sono necessari per il giudizio temporale.
- **Postprocessing:** L330–337 cattura errori plot/I/O come campo separato e non riscrive status/completion della simulazione, nei percorsi normali e censurati L499/L512. Restano separati check dei sette plot e giudizio fisico.
- **Batch/failure capture:** L645–668 assegna receipt batch con digest dei case-id e mode, prosegue dopo failure locali e restituisce exit aggregato alla fine. Receipt iniziale salvato prima assertions; JSONL L451–460 conservano transizioni completate anche se un crash nativo impedisce except/finally. In tale caso `integration_steps` nel receipt puo essere stale: ricostruire durata dai journal/watchdog/log disponibili; non inventare reward o campioni della transizione interrotta. Il primo segmento e salvato anche sul percorso censurato Python L517–518.
- **Test source:** `test_bootstrap.py` copre mapping della action originale, timeout tecnico, postprocessing distinto, binding receipt/outcomes e continuita governor/fallback; `bootstrap_probe.py` attiva soltanto il relativo mode. Nessuna esecuzione e stata effettuata dall'auditor.

**Verdetto source:** nessun difetto bloccante rilevato nella versione sigillata per il protocollo fisso. E appropriato eseguire il preflight statico e le probe native con gli input congelati; l'esecuzione dinamica dei casi selezionati resta subordinata al successo delle invarianti/probe native, senza cambiare guard, policy o plugin. Questo e un GO del source e del metodo, non una validazione di stabilita, di risultati futuri o di biomeccanica/generalizzazione.

Persistono i limiti interpretativi originali: dominio condizionato al gate geometrico, biologia/time/GRF fissi, accelerazione fisica non imposta, startup in walking-state, denominatori senza survivors bias e assi completion/cicli/tracking/contact/reserve/SEA separati.
