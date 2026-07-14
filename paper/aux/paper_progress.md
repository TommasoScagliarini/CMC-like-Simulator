# Paper Writing Progress

Questo documento e' il registro operativo della scrittura del paper. Serve come
baseline per un LLM che si approccia al manoscritto senza dover ricostruire ogni
volta il contesto dai report giornalieri.

## Stato generale

Il paper riguarda il progetto di RL trajectory generation per una protesi
transfemorale knee+ankle con attuatori SEA, valutata in un simulatore OpenSim
CMC-like muscle-driven.

Stato scientifico attuale:

- il `Trajectory Generator` non e' ancora nella fase finale ex-novo;
- la linea attuale e' ancora pretraining imitativo / validazione architetturale;
- l'obiettivo finale resta la generazione di traiettorie cinematiche protesiche
  ex-novo;
- evitare overclaim del tipo "we demonstrate ex-novo trajectory generation" finche'
  non sono completi baseline ladder, rollout finali e multi-seed.

Formula prudente consigliata:

```text
We develop and validate the architecture for reinforcement-learned prosthetic
reference generation, and currently use imitation pretraining as the first stage
toward ex-novo trajectory generation.
```

## File principali

- `paper/aux/paper_outline.md`: outline annotata del paper.
- `paper/aux/literature_related_work_positioning_DRAFT.md`: mappa della
  letteratura e posizionamento del contributo.
- `paper/sections/section_3_DRAFT.md`: prima bozza Markdown della Section III,
  preservata come traccia di ragionamento.
- `paper/sections/section_3.tex`: bozza LaTeX attiva della Section III
  (`Simulation Testbed`).
- `paper/main.tex`: contenitore LaTeX modulare in stile IEEEtran, con inclusione
  delle sezioni tramite file separati.
- `paper/aux/How to Write an Academic Research Paper_ Complete Guide 2025.md`:
  guida generale alla scrittura scientifica, usata come supporto metodologico.

## Progressi completati

### 0. Revisione struttura paper dopo confronto con professori

File:

```text
paper/aux/paper_outline_v2_generator_methods.md
```

Decisione strutturale aggiornata:

- la sezione `Methods` non deve contenere simulazione, OpenSim testbed,
  inverse dynamics, static optimization, GRF, reserve, SEA/protesi SEA o
  validazioni del plant;
- `Methods` deve descrivere il `Trajectory Generator` in senso stretto e la sua
  control chain modulare:
  - formulazione RL;
  - output/action contract;
  - generazione della traiettoria tramite gradini/knot prodotti dalla policy;
  - filtro C2 come passaggio da comandi discreti a reference liscia;
  - descrizione generale di `Cascade Control`;
  - descrizione generale di `Motor Driver`;
  - ricostruzione della fase del gait cycle come prerequisito per la
    generazione ex-novo;
  - observation design e actor/critic split;
  - reward design;
- la training strategy, inclusi PPO, rete, iterazioni, seed, checkpoint e
  rollout protocol, va invece in `Experimental Setup`;
- tutto cio' che riguarda il testbed/simulazione va spostato in
  `Experimental Setup` e `Results`.

Conseguenza:

- la vecchia Section III `Simulation Testbed` resta tecnicamente utile, ma non
  deve piu' essere trattata come sezione Methods;
- la nuova outline V2 separa esplicitamente:
  - metodo = generatore di traiettorie + cascade control + motor driver;
  - apparato sperimentale = OpenSim/CMC-like/SEA/GRF/reserve;
  - risultati = validazione apparato + valutazione generator.

### 1. Redazione dell'outline

File:

```text
paper/aux/paper_outline.md
```

Contenuto prodotto:

- struttura del paper in formato IEEE conference, 10 pagine, doppia colonna;
- tesi principale:
  - RL genera la reference cinematica protesica;
  - il governor/reference interface la rende fisicamente realizzabile;
  - SEA, cascade/PI e simulatore CMC-like sono il testbed validato;
- distinzione tra contributo principale e infrastruttura:
  - protagonista: `Trajectory Generator`;
  - testbed: SEA + cascade/PI + CMC-like simulator;
- status per sezione:
  - `[READY]`, `[PARTIAL]`, `[BLOCKED]`;
- scaletta delle sezioni:
  - Introduction;
  - Related Work;
  - Simulation Testbed;
  - RL Trajectory Generator;
  - Experimental Setup;
  - Results;
  - Discussion;
  - Limitations & Future Work;
  - Conclusion;
- registro delle equazioni vive;
- lista figure;
- lista tabelle;
- checklist integrita' dati.

Decisioni importanti:

- mantenere le bozze concettuali in Markdown, ma lavorare sul manoscritto attivo
  nel main LaTeX modulare;
- non usare `return/reward` come metrica di successo finale;
- distinguere sempre:
  - actual-vs-IK RMSE;
  - served-vs-target RMSE;
  - target imitation vs reference servita;
- mantenere placeholder espliciti per risultati non ancora disponibili;
- non presentare il pretraining imitativo come risultato ex-novo conclusivo.

### 2. Redazione della bozza literature/related-work

File:

```text
paper/aux/literature_related_work_positioning_DRAFT.md
```

Contenuto prodotto:

- analisi della cartella `Trajectory Generator/literature`;
- classificazione dei paper per ruolo:
  - protesi RL: P01-P05;
  - OpenSim + DRL muscoloscheletrico: P06-P11, D02;
  - imitation/pretraining: P08, P10, P11;
  - action space e gerarchia high-level/low-level: P04, P12, P13, S02;
  - SEA e motivazione biomeccanica: P20;
  - reward energia/metabolico/fisica: P19, P21;
  - POMDP/memoria/partial observability: P22;
  - safety/future work: P14-P18;
- identificazione del nucleo piu' forte:
  - P04 + P11 + P13 + P20 + P22;
- gap statement consigliato:

```text
Prior prosthesis RL mainly tunes impedance parameters or target features;
we train an RL policy to generate the continuous kinematic reference trajectory
for a knee-ankle series-elastic transfemoral prosthesis.
```

Posizionamento emerso:

- P01-P05 ottimizzano impedenza, target feature o simmetria, ma non generano una
  reference continua knee+ankle per SEA;
- P11 e' il precedente OpenSim/transfemoral piu' vicino, ma usa muscle-like
  prosthetic actuators, non SEA + reference generator;
- P13/S02 supportano la scelta di un action space tipo position/reference invece
  di torque diretto;
- P22 supporta il framing POMDP e la futura estensione con memoria, ma la policy
  attuale e' una MLP feed-forward senza memoria esplicita;
- P20 giustifica la scelta SEA e il focus sul lavoro/coppia ankle.

### 3. Bozza Section III

File:

```text
paper/sections/section_3_DRAFT.md
paper/sections/section_3.tex
```

Ordine scelto:

```text
SEA -> CMC-like simulator -> cascade/PI controller
```

Motivo:

- partire dal SEA mette subito in primo piano il vincolo fisico centrale;
- il SEA giustifica perche' la reference generata dalla rete deve essere
  band-limited e tracciabile;
- il simulatore CMC-like mostra come la reference venga valutata in un contesto
  muscle-driven;
- il cascade/PI chiude il collegamento tra reference cinematica e comando SEA.

Contenuto della bozza:

- descrizione dei due `SeriesElasticActuator` su:
  - `pros_knee_angle`;
  - `pros_ankle_angle`;
- stati interni:
  - `motor_angle`;
  - `motor_speed`;
  - `torque_error_integral`;
- legge SEA viva del plugin PI:
  - spring torque;
  - torque reference;
  - motor torque PI;
  - clamp a 500 N m;
  - motor dynamics;
  - anti-windup;
  - spring torque come generalized force;
- parametri attuali del modello `AB06_SEASEA_stiff321_500_pi`;
- descrizione CMC-like:
  - IK filtering a 6 Hz;
  - finestra 11.99-21.0 s;
  - timestep 1 ms;
  - biological outer-loop accelerations;
  - zero-actuator inverse dynamics con mass-matrix projection;
  - static optimization muscle-first;
  - reserves come fidelity gauge;
- descrizione cascade/PI:
  - outer position-to-velocity loop;
  - inner velocity PI;
  - normalizzazione del torque command in `u`;
  - gain attuali knee/ankle;
  - ruolo del controller come interfaccia non appresa.

Stato LaTeX attuale:

- `section_3.tex` e' la bozza attiva, inclusa da `paper/main.tex`;
- la sezione e' ancora marcata con `\draftnote{TO BE REVIEWED}`;
- struttura confermata:
  - introduzione al testbed;
  - modello SEA fisico;
  - simulatore CMC-like muscle-driven;
  - tracking protesico cascade/PI;
- scelta narrativa consolidata:
  - nella sottosezione SEA si descrive il plant fisico;
  - la spiegazione del PI/motor drive resta nella sottosezione controllori;
  - il simulatore viene presentato come ambiente di valutazione che tratta
    diversamente lato protesico e lato biologico, mantenendoli pero' accoppiati
    nella forward dynamics.

Dati verificati contro modello/config/plugin:

- modello attivo: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`;
- `config.py` punta a `AB06_SEASEA_stiff321_500_pi.osim`;
- setup recente: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`;
- parametri plant SEA confermati:
  - Knee: `K=321`, `J_m=0.01`, `B_m=0.1`, `F_opt=100`;
  - Ankle: `K=500`, `J_m=0.01`, `B_m=0.1`, `F_opt=250`;
- guadagni drive plugin confermati:
  - Knee: `(Kp,Kd,Ki)=(18,11,190)`, `integral_torque_limit=100`;
  - Ankle: `(Kp,Kd,Ki)=(11.3,11,123)`, `integral_torque_limit=100`;
  - modo `Impedence=false`;
  - clamp motore a `+/-500` N m nel sorgente C++;
- equazioni plugin confermate:
  - `tau_s = K(theta_m - theta_j)`;
  - `tau_m_raw = tau_ref + Kp(tau_ref - tau_s) + sat(Ki xi, +/-L_i) - Kd omega_m`;
  - `dot(theta_m)=omega_m`;
  - `J_m dot(omega_m)=tau_m - tau_s - B_m omega_m`;
  - in non-impedance mode la forza generalizzata applicata al giunto e'
    `tau_s`, non `tau_m`;
- impostazioni simulatore confermate:
  - finestra `11.99`-`21.0` s;
  - `dt=0.001` s;
  - `integration_scheme="rk4_bypass"`;
  - IK preprocessing abilitato, Butterworth zero-phase, cutoff `6 Hz`,
    ordine `4`, resampling `0.001` s;
  - biological tracking gains `Kp=100`, `Kd=20`, con pelvis translation
    `25/10`;
  - static optimization muscle-first con `reserve_weight=1e6`;
- guadagni cascade Python confermati:
  - Knee: `Kp_outer=18.85`, `Kp_inner=29.2`, `Ki_inner=1377`,
    inner integral torque limit `50` N m;
  - Ankle: `Kp_outer=47.125`, `Kp_inner=2.8275`, `Ki_inner=213`,
    inner integral torque limit `200` N m.

Analisi corrente di `section_3.tex`:

- il contenuto tecnico e' sostanzialmente coerente con modello, config e plugin;
- il paragrafo introduttivo e' ancora in revisione:
  - deve dire che il testbed serve sia a valutare le reference apprese sia a
    validare/valutare la control chain protesica;
  - evitare di far sembrare che il lato biologico sia cinematicamente bloccato:
    meglio parlare di experimental kinematics as tracking references;
  - specificare che il diverso trattamento tra lato protesico e lato biologico e'
    un disaccoppiamento di controllo/recruitment, non un disaccoppiamento
    dinamico;
- la frase "tracked through the controllers" e' volutamente generale ma va
  rifinita: possibile wording consigliato:

```text
Instead, it provides a prosthetic kinematic reference that is converted into
physically constrained actuator commands by the prosthetic control interface.
```

- nella sottosezione SEA c'e' una duplicazione da rimuovere:
  - il paragrafo che inizia con "Thus, the prosthetic joint is actuated..." appare
    sia subito dopo le equazioni SEA sia dopo la tabella;
  - tenerlo una sola volta, preferibilmente dopo le equazioni e prima della
    tabella, oppure usarne una versione breve come ponte verso il simulatore;
- la caption della tabella SEA e' tecnicamente corretta ma puo' essere piu'
  precisa:
  - preferibile `SEA plant parameters adopted.` per distinguere plant parameters
    da actuator-drive gains;
- la sottosezione CMC-like e' coerente, ma richiede revisione stilistica finale
  e possibili citazioni;
- la sottosezione controller e' tecnicamente allineata al plugin, ma manca ancora
  la tabella finale dei guadagni/limiti;
- sono ancora presenti placeholder per figure e tabelle.

Note per revisione futura:

- verificare tutti i numeri contro lo snapshot finale del modello/config usato negli
  esperimenti; i numeri correnti sono stati verificati contro lo snapshot attuale,
  ma dovranno essere ricontrollati se cambiano modello/config;
- decidere naming finale coerente tra "series-elastic actuator" e "SEA";
- rifinire il paragrafo introduttivo sugli SEA. Versione consigliata:

```text
Series-elastic actuators (SEAs) consist of an electric motor connected to the
joint through a torsional spring in series. Thus, the joint is not driven rigidly
by the motor; instead, torque is transmitted through spring deflection. SEAs were
selected to actuate the transfemoral prosthesis for two main reasons.

First, the series spring can store and release energy during the gait cycle. This
can improve energetic behavior and provides intrinsic compliance, allowing the
actuator to attenuate impacts during gait. Second, SEAs are mechanically simpler
than variable-stiffness actuators while still providing compliant actuation.

Each SEA was implemented as a custom OpenSim plugin. This choice preserves
compatibility with OpenSim's generalized-force framework while allowing the
actuator to expose internal motor states and series-spring dynamics.
```

- la conversione LaTeX della Section III e' iniziata in `section_3.tex`;
- aggiungere citazioni in Section III solo dove servono davvero:
  - P20 per SEA/metabolic/ankle work;
  - P06/D02 per OpenSim-RL/Gym-style environment se richiamato;
  - P13/S02 solo come ponte verso Section IV, non per saturare la sezione testbed.

## Prossimi passi consigliati

1. Scrivere la Section II Related Work partendo da
   `literature_related_work_positioning_DRAFT.md`.
2. Scrivere la Section IV RL Trajectory Generator, ma con wording prudente:
   "imitation pretraining toward ex-novo generation".
3. Ripulire `section_3.tex`:
   - rimuovere la duplicazione del paragrafo conclusivo SEA;
   - rifinire introduzione e wording sul diverso trattamento protesico/biologico;
   - trasformare i placeholder di Table III/Table IV in tabelle LaTeX reali;
   - decidere caption definitiva della tabella SEA.
4. Preparare figure schematiche per Section III:
   - schema SEA;
   - flow testbed CMC-like/OpenSim;
   - eventuale schema control interface.
5. Prima dei risultati finali, completare baseline ladder e multi-seed; non chiudere
   Abstract/Conclusion prima di quei numeri.
