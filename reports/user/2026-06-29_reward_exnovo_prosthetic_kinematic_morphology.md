# Reward ex-novo: prosthetic kinematic morphology - 2026-06-29

## Problema

La reward `ex_novo` attuale contiene gia' termini task-based per contatto,
unloading, validita' del contatto, regolarita' degli eventi, range articolare
globale, uso SEA e sicurezza. Resta pero' aperto il punto 7:

```text
prosthetic kinematic morphology
```

L'obiettivo del termine e' guidare la policy verso traiettorie protesiche
biomeccanicamente plausibili senza reintrodurre una reward imitativa. Il termine
non deve premiare la vicinanza alla traiettoria IK media o alla sound leg, ma
deve funzionare come guardrail: la traiettoria generata puo' essere libera, ma
non deve diventare cinematicamente incompatibile con un passo protesico
ragionevole.

La motivazione pratica e' creare un "paracarro" per la rete: evitare che, nella
generazione ex-novo, la policy sia completamente cieca sulla forma temporale
plausibile di knee e ankle durante il gait cycle.

## Principio di progetto

Il punto centrale emerso e':

```text
non premiare il centro della traiettoria media;
premiare/stabilizzare lo stare dentro un corridoio operativo phase-dependent.
```

Quindi la traiettoria media AB06 puo' essere usata per costruire il corridoio,
ma non deve diventare un attrattore di reward.

Forma concettuale corretta:

```text
dentro il corridoio:
  nessuna penalita' o penalita' nulla

vicino ai limiti:
  penalita' morbida crescente

molto fuori dal corridoio:
  penalita' forte
```

Per ora la terminazione esplicita e' da evitare o da rimandare. Il morphology
term dovrebbe essere soft; le terminazioni restano dedicate a safety fisica,
divergenze e limiti articolari hard gia' presenti.

## Range cinematico phase-dependent

La proposta principale e' costruire offline un corridoio cinematico da:

```text
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
```

Il dataset AB06 utile va circa da `t = 11.9 s` a `t = 150 s`. La segmentazione
dei cicli deve essere fatta usando le GRF/eventi per definire sequenze:

```text
HS -> TO -> HS
```

Pipeline proposta:

```text
IK_results_AB06_SEASEA.mot
+ GRF/eventi HS-TO-HS
-> segmentazione in gait cycles completi
-> normalizzazione di ogni ciclo su percentuale GC
-> media(phi) e std(phi) di knee/ankle
-> corridoio operativo:
   q_min(phi) = mean(phi) - K * std(phi) - margin
   q_max(phi) = mean(phi) + K * std(phi) + margin
```

Durante il training, per la fase corrente `phi`:

```text
if q in [q_min(phi), q_max(phi)]:
    loss = 0
else:
    loss = excursion fuori banda, normalizzata dalla larghezza del corridoio
```

Una variante utile e' introdurre due bande:

```text
inner corridor:
  nessuna penalita'

outer corridor:
  penalita' progressiva

hard safety corridor:
  eventuale truncation futura, non nel primo pass
```

## Differenza rispetto al range articolare gia' presente

Il config attuale contiene gia':

```yaml
prosthetic_joint_range_weight
prosthetic_joint_q_min
prosthetic_joint_q_max
```

Questi campi descrivono un range articolare globale, non dipendente dalla fase.
Servono a evitare configurazioni fisicamente assurde in senso ampio.

Il morphology corridor sarebbe diverso:

```text
prosthetic_joint_range:
  q dentro limiti globali ampi

prosthetic_kinematic_morphology:
  q dentro limiti plausibili per quella fase del gait cycle
```

Esempio: una flessione del ginocchio puo' essere accettabile in swing, ma non
in loading response sotto carico. Il range globale non distingue questi casi;
il morphology term si'.

## Velocita' articolari

Per ora non si vuole costruire un corridoio phase-dependent sulle velocita' di
knee e ankle.

Motivi:

- `qdot` dipende molto dalla velocita' del passo;
- la durata del ciclo puo' variare;
- il reference governor e il SEA introducono filtraggio e ritardi;
- il segnale puo' diventare troppo vincolante e meno robusto;
- una envelope su `qdot(t)` rischia di diventare piu' imitativa di quella su
  `q(phi)`.

Nel primo pass si propone quindi:

```text
includere q(phi) corridor;
non includere qdot(phi) corridor;
mantenere smoothness / jerk / high-frequency penalty come guardrail separati.
```

Se in futuro servisse un vincolo sulle velocita', una forma piu' robusta sarebbe
studiare `dq/dphi` invece di `dq/dt`, per ridurre la dipendenza dalla durata
assoluta del passo.

## Sincronizzazione con il gait cycle

Il tema critico e' la sincronizzazione tra il corridoio cinematico e la fase del
gait cycle.

La fase deve essere dedotta dal detector `online_grf`, non da un clock
prescribed sound-side. La sfida e' far si' che la sequenza:

```text
HS -> TO -> HS
```

avvenga correttamente e non venga manipolata dalla policy.

Una possibile costruzione e':

```text
HS rilevato:
  phi = 0
  stato discreto = stance

tra HS e TO:
  phi cresce dentro la porzione stance del ciclo

TO rilevato:
  stato discreto = swing

tra TO e next HS:
  phi cresce dentro la porzione swing del ciclo

next HS:
  chiusura ciclo e reset a phi = 0
```

Per trasformare gli eventi in una fase continua su `%GC`, una soluzione
pragmatica e' usare una fase ibrida:

```text
stance:
  phi = phi_TO_nominal * elapsed_since_HS / T_stance_expected

swing:
  phi = phi_TO_nominal
        + (1 - phi_TO_nominal) * elapsed_since_TO / T_swing_expected
```

con clamp:

```text
during stance: phi in [0, phi_TO_nominal]
during swing: phi in [phi_TO_nominal, 1]
```

Il detector GRF decide la fase discreta stance/swing; i tempi nominali/filtrati
costruiscono una fase continua. Questo evita di usare direttamente, in modo
autoreferenziale, il periodo HS->HS appena prodotto dalla policy come target
temporale corrente.

## Rischio di gaming degli eventi

Il rischio principale e' che la policy impari a manipolare gli eventi invece di
produrre un passo valido. Esempi:

- anticipare o ritardare il TO per trovarsi in una configurazione che riceve meno
  penalita';
- generare micro-contatti che resettano la fase;
- tenere il piede in una condizione borderline per ottenere una fase favorevole;
- far dipendere troppo la reward da un singolo istante/evento.

Mitigazioni proposte:

- mantenere separato il termine di event-order/regolarita';
- usare timeout di stance e swing;
- applicare morphology solo quando la sequenza eventi e' credibile;
- usare finestre temporali/fase, non reward spike istantanei;
- evitare grandi bonus al singolo evento;
- penalizzare configurazioni incompatibili distribuite sulla fase, non imporre
  target puntuali rigidi.

## Azioni specifiche legate ai checkpoint del gait cycle

La seconda idea e' aggiungere vincoli biomeccanici specifici per checkpoint o
finestre del gait cycle.

La formulazione deve essere a inequality / hinge loss, non a tracking puntuale.
Esempio corretto:

```text
a HS, penalizza se il ginocchio e' troppo flesso
```

Esempio da evitare:

```text
a HS, forza il ginocchio esattamente a 0 deg
```

Finestre candidate:

```text
HS / initial contact:
  phi in [0.00, 0.08]

loading / early stance:
  phi in [0.08, 0.25]

mid/late stance:
  phi in [0.25, phi_TO]

TO window:
  phi in [phi_TO - delta, phi_TO + delta]

early swing:
  phi in [phi_TO, 0.75]

late swing / pre-HS:
  phi in [0.85, 1.00]
```

Esempi di inequality:

```text
HS / pre-HS:
  penalizza solo se knee e' troppo flesso

loading:
  penalizza se knee collassa in flessione

swing:
  penalizza se knee non raggiunge una flessione minima

late swing:
  penalizza se knee non ritorna verso estensione

stance:
  penalizza ankle in configurazione di collasso

swing:
  penalizza toe clearance insufficiente, se la geometria e' disponibile
```

Nel modello corrente la flessione del ginocchio protesico e' negativa. Quindi
un esempio tipo "knee circa 40 deg" va interpretato come circa:

```text
-0.70 rad
```

## Interfasi HS -> TO -> HS

La definizione delle interfasi e' ancora da progettare con attenzione.

Le interfasi non dovrebbero essere solo istanti puntuali, perche':

- gli eventi GRF possono avere rumore;
- l'istante di TO/HS puo' variare;
- una policy puo' spostare gli eventi per massimizzare reward;
- vincoli troppo puntuali producono reward sparse e instabile.

Meglio definire finestre morbide:

```text
HS window
loading window
stance window
TO window
swing window
pre-HS window
```

Ogni finestra deve avere:

- una condizione di attivazione basata su fase/eventi online;
- una o piu' inequality biomeccaniche;
- tolleranze larghe;
- perdita normalizzata;
- peso iniziale basso.

## Accoppiamento eventi-configurazioni

La terza idea e' premiare/penalizzare accoppiamenti tra eventi del gait cycle e
configurazioni di knee/ankle.

Esempi discussi:

```text
TO avviene quando il knee e' circa 40 deg di flessione
TO avviene quando l'ankle e' circa 20 deg
dopo TO la gamba non deve essere in carico, quindi GRF protesica ~ 0
```

Questa idea e' utile, ma e' la piu' delicata. Se il reward dice "TO e' buono
quando q e' vicino a un valore", la policy puo' imparare a generare TO nel
momento cinematicamente conveniente, non necessariamente nel momento gait
corretto.

Forma consigliata:

```text
non premiare "TO esatto con q esatto";
penalizzare invece configurazioni incompatibili in una finestra attorno a TO.
```

Esempio:

```text
TO window:
  knee deve stare dentro una banda larga;
  ankle deve stare dentro una banda larga;
  nessun bonus al valore centrale.
```

Il vincolo "dopo TO, GRF = 0" e' gia' concettualmente coperto dal termine
`swing_unloading`. Conviene non duplicarlo dentro morphology, ma assicurarsi che
`swing_unloading` sia phase-gated correttamente.

## Forma candidata del termine

Una forma ad alto livello:

```text
morphology_loss =
    w_corridor * corridor_excursion_loss
  + w_hs * hs_knee_flexion_violation
  + w_loading * loading_knee_collapse_violation
  + w_swing * swing_knee_underflexion_violation
  + w_prehs * pre_hs_knee_underextension_violation
  + w_to * to_window_configuration_violation
  + w_clearance * toe_clearance_violation
  + w_osc * high_frequency_oscillation_loss
```

Nel primo pass, il set minimo consigliato e':

```text
1. corridor_excursion_loss su knee+ankle, phase-dependent;
2. knee stance collapse inequality;
3. knee swing underflexion inequality;
4. pre-HS knee underextension inequality;
5. ankle stance collapse inequality.
```

Toe clearance puo' essere aggiunto solo se la geometria del piede protesico e'
disponibile in modo robusto nell'env.

## Integrazione nella reward

La reward ex-novo potrebbe integrare il termine come penalita':

```text
reward -= morphology_weight * morphology_loss
```

oppure come score positivo:

```text
morphology_score = exp(-morphology_weight * morphology_loss)
reward += blend_morphology * morphology_score
```

Per coerenza con altri guardrail, la forma come penalita' e' piu' semplice da
diagnosticare. Se invece si vuole renderla parte della base positiva, serve
evitare che saturi o domini `contact_load_score` e `phase_regular_score`.

Una configurazione futura potrebbe contenere campi del tipo:

```yaml
morphology_weight: 0.0
morphology_corridor_k_std: 3.0
morphology_corridor_margin_rad: [0.10, 0.08]
morphology_corridor_soft_fraction: 0.80
morphology_knee_stance_collapse_weight: 0.0
morphology_knee_swing_underflexion_weight: 0.0
morphology_pre_hs_extension_weight: 0.0
morphology_ankle_stance_collapse_weight: 0.0
```

I pesi dovrebbero partire a zero o molto bassi, con logging diagnostico prima
dell'attivazione forte.

## Criticita'

Criticita' principali:

1. Dipendenza dalla fase online.
   Se la fase `phi` e' sbagliata, il corridoio giusto viene applicato nel punto
   sbagliato del passo.

2. Dataset AB06 singolo.
   Un corridoio da un solo soggetto/condizione puo' essere troppo specifico. Va
   allargato con `K` alto, margini minimi e smoothing.

3. Rischio di imitazione mascherata.
   Il centro medio non deve ricevere reward positiva. Solo l'uscita dal corridoio
   deve essere penalizzata.

4. Transizioni HS/TO.
   Intorno agli eventi la deviazione standard puo' essere distorta da piccoli
   errori di allineamento. Serve smoothing e `std_floor`.

5. Reward hacking sugli eventi.
   La policy puo' manipolare HS/TO se la reward attorno agli eventi e' troppo
   puntuale o troppo remunerativa.

6. Eccesso di shaping.
   Troppi termini morfologici possono soffocare la parte task-based della reward
   e ridurre la capacita' ex-novo.

7. Terminazione prematura.
   Terminare per uscita dal corridoio morfologico puo' rendere il training
   fragile. Meglio iniziare con penalita' soft.

## Strategia proposta

Strategia incrementale:

1. Costruire offline il corridoio `q_min(phi), q_max(phi)` da AB06.
2. Validare il corridoio su cicli AB06 lasciati fuori e su rollout esistenti.
3. Aggiungere logging diagnostico senza peso in reward.
4. Verificare distribuzione di:
   - corridor excursion;
   - fase online;
   - event order;
   - stance/swing timeout;
   - knee/ankle violations.
5. Attivare solo `corridor_excursion_loss` con peso basso.
6. Aggiungere progressivamente inequality di checkpoint.
7. Solo dopo valutare toe clearance e penalita' anti-oscillazione dedicate.

## Verifiche raccomandate prima del training

Prima di usare il termine in training lungo:

- plottare `mean`, `std`, `q_min`, `q_max` su `%GC`;
- verificare che il corridoio contenga i cicli AB06 originali;
- controllare che `std_floor` eviti strozzature artificiali;
- testare il corridoio su rollout esistenti;
- misurare quante violazioni sono dovute a fase errata e quante a morfologia
  realmente non plausibile;
- confrontare morphology loss contro `phase_regular_score` e `contact_load_score`;
- assicurarsi che nessun termine premi esplicitamente la distanza minima dalla
  traiettoria media.

## Decisioni prese

- Il morphology term deve essere progettato come guardrail, non come tracking.
- Il corridoio phase-dependent puo' essere costruito da
  `IK_results_AB06_SEASEA.mot`.
- La segmentazione deve usare eventi GRF `HS -> TO -> HS`.
- Per ora non si usa una envelope sulle velocita' articolari.
- La fase deve venire dal detector `online_grf`.
- La sequenza HS->TO->HS deve restare oggetto di reward/diagnostica separata.
- Le azioni biomeccaniche ai checkpoint devono essere inequality morbide.
- Il termine deve partire come logging/diagnostica o con peso molto basso.

## Domande aperte

- Quale profilo GRF usare esattamente per costruire offline gli eventi del
  dataset AB06: detector HS-TO gia' validato o GRF prescribed/oracle?
- Quale valore iniziale usare per `K` nel corridoio: `2.5`, `3.0` o piu' alto?
- Serve un corridoio separato per stance e swing oppure basta `q_min/q_max(phi)`
  continuo su `%GC`?
- Quanto deve essere largo lo `std_floor` minimo per knee e ankle?
- Toe clearance e' disponibile in modo affidabile nell'env corrente o richiede
  nuova diagnostica geometrica del piede?
- Il morphology term deve essere sottratto dopo il clip della base reward, come
  safety/contact feasibility, oppure entrare dentro la base ex-novo?

## File modificati

- Aggiunto questo report:
  `reports/user/2026-06-29_reward_exnovo_prosthetic_kinematic_morphology.md`.

Non sono stati modificati:

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`;
- `Trajectory Generator/baseline_MLP/reward_function.py`;
- `Trajectory Generator/osim_trj_cmc_like.py`.

## Test e verifiche eseguite

Verifiche svolte in questa fase:

- consultati i report precedenti in cui il punto 7 era stato rimandato;
- confermato che `prosthetic kinematic morphology` non e' ancora implementato;
- ricondotta la proposta ai tre blocchi discussi:
  - corridoio cinematico phase-dependent;
  - inequality biomeccaniche ai checkpoint;
  - accoppiamento eventi-configurazioni;
- nessun test eseguito, perche' il lavoro e' di progettazione/report e non ha
  introdotto modifiche al codice.

## TODO

- [ ] Costruire offline i cicli AB06 da `IK_results_AB06_SEASEA.mot` e GRF/eventi
      `HS -> TO -> HS`.
- [ ] Generare `q_min(phi), q_max(phi)` per `pros_knee_angle` e
      `pros_ankle_angle`, con smoothing e `std_floor`.
- [ ] Validare il corridoio su dataset AB06 e rollout esistenti.
- [ ] Aggiungere logging diagnostico del morphology corridor senza peso in
      reward.
- [ ] Definire le finestre HS/loading/TO/swing/pre-HS e le relative inequality.
- [ ] Integrare il termine in `training_exnovo_cfg.yaml` solo dopo validazione
      diagnostica.
