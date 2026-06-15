# Progettazione reward ex-novo task-based

**Data**: 2026-06-11  
**Contesto**: ridiscussione completa della reward `ex_novo` del Trajectory
Generator, tenendo conto dell'architettura asymmetric actor-critic e della
letteratura locale in `Trajectory Generator/literature/`.

## Problema

La modalita' `ex_novo` corrente non e' realmente task-based. Combina infatti:

- tracking della traiettoria comandata dalla policy;
- vicinanza alla IK protesica sperimentale;
- vicinanza delle coordinate biologiche alla IK;
- penalita' su effort, smoothness, saturazione, sicurezza e riferimenti fuori
  banda.

Il peso dominante della IK rende l'obiettivo sostanzialmente imitativo. Inoltre,
tracking biologico e reserve possono mantenere il modello stabile anche quando
la protesi fornisce un supporto insufficiente, producendo reward elevate senza
un contributo protesico funzionale.

Era quindi necessario definire una reward ex-novo che:

1. non prescriva una traiettoria articolare sperimentale;
2. valuti risultati biomeccanici influenzabili dalla policy;
3. distingua l'obiettivo funzionale dai vincoli di fattibilita' e sicurezza;
4. sfrutti correttamente il critic privilegiato senza rendere l'actor dipendente
   da segnali non disponibili in deployment;
5. preservi la modalita' `imitation` per il pretraining.

## Decisioni di progetto

- Va modificata l'attuale modalita' `ex_novo`; non va creata una terza modalita'.
- `imitation` continua a usare i riferimenti cinematici per il pretraining.
- In `ex_novo`, target IK protesici e `kin_ref` devono essere mascherati o
  disattivati per il critic, mantenendoli disponibili nell'architettura per
  `imitation`.
- L'energia SEA potra' essere registrata per uso futuro, ma inizialmente avra'
  peso zero.
- La saturazione SEA deve entrare esplicitamente nella reward.
- Il tracking tra comando della policy e moto ottenuto, inizialmente chiamato
  `command_realizability`, non deve essere un obiettivo positivo dominante.
  Verra' mantenuto come diagnostica `execution_error`, inizialmente a peso zero.
- La nuova reward deve separare:
  - obiettivo primario del task;
  - fattibilita' fisica;
  - regolarizzazione;
  - sicurezza hard.

## Analisi della letteratura locale

Sono stati consultati gli indici, le note strutturate e alcuni passaggi dei PDF
originali presenti nel corpus locale.

### P03 - Wearer-Prosthesis Interaction for Symmetrical Gait

Il paper mostra che:

- gli impulsi antero-posteriori di braking e propulsion sono influenzati dal
  controllo protesico;
- il net inter-limb AP impulse e' il correlato piu' consistente della simmetria
  dello stance time;
- bilanciare braking dell'arto protesico e propulsion dell'arto sano migliora
  la coordinazione durante la transizione tra arti;
- la sola regolazione della protesi non garantisce necessariamente una perfetta
  simmetria, perche' il comportamento dell'arto sano contribuisce al risultato.

### P04 - Hierarchical Optimization for Robotic Knee Prostheses

E' il riferimento locale piu' vicino all'obiettivo del progetto:

- il livello high-level ottimizza direttamente la simmetria dell'impulso
  propulsivo;
- il livello low-level realizza il target cinematico tramite controllo
  d'impedenza;
- le cinematiche che ottimizzano la simmetria non coincidono con la cinematica
  normativa;
- ripristinare la traiettoria articolare normativa non equivale necessariamente
  a ottimizzare la prestazione del sistema persona-protesi.

### P12 - GLiDE

GLiDE mostra che una reward funzionale semplice funziona quando l'architettura
impone la fattibilita' tramite:

- gait pattern strutturato;
- QP per GRF compatibili con il cono d'attrito;
- foot placement vincolato;
- controllo low-level per realizzare le azioni high-level.

La lezione trasferibile e' che i comportamenti non fisici non devono essere
contrastati esclusivamente sommando molte penalita' alla reward.

### P19 - ECO

ECO supporta:

- asymmetric actor-critic con actor deployable e critic privilegiato;
- separazione tra reward del task e costi/vincoli;
- uso di fase, contatti e stato globale come informazioni privilegiate;
- difficolta' di ottenere un buon compromesso multi-obiettivo regolando
  solamente pesi additivi.

### P16 - Safe Robot Learning in Assistive Devices

Il paper motiva vincoli espliciti su:

- magnitudine del comando;
- command-rate;
- velocita';
- angoli articolari.

Questi segnali sono direttamente pertinenti a saturazione SEA, smoothness e
limiti articolari del Trajectory Generator.

## Obiettivo primario proposto

La letteratura locale ha portato a correggere la prima ipotesi: la forza
verticale di supporto non dovrebbe essere il driver principale della reward.

L'obiettivo primario proposto e':

> Generare un contributo protesico funzionale alla locomozione, misurato tramite
> impulsi GRF antero-posteriori e coordinazione temporale bilaterale.

### Impulso propulsivo

Durante la parte propulsiva della stance protesica:

```text
I_prop_pros = integral(max(F_AP_pros, 0) dt)
```

La convenzione del segno deve essere verificata sul modello AB06.

Una prima misura funzionale puo' confrontare gli impulsi propulsivi dei due arti:

```text
SI_prop =
    (I_prop_sound - I_prop_pros)
    / (0.5 * (I_prop_sound + I_prop_pros) + epsilon)

propulsion_loss = SI_prop^2
```

Questo obiettivo non prescrive gli angoli di ginocchio e caviglia necessari per
generare l'impulso.

### Net inter-limb impulse

Durante la transizione tra arti:

```text
I_transition = I_brake_pros + I_prop_sound
transition_loss = normalized(I_transition)^2
```

Questa misura valuta la coordinazione dinamica tra arto protesico e arto sano.

### Limite del confronto bilaterale

Nel simulatore corrente:

- la GRF protesica e' calcolata online;
- la GRF sana resta prescribed.

Usare la GRF sana come riferimento funzionale privilegiato e' meno restrittivo
dell'imitazione IK, ma non e' completamente indipendente dai dati
sperimentali. E' considerato un compromesso iniziale ragionevole, da rendere in
futuro meno prescrittivo usando bande funzionali ampie.

## Fattibilita' del contatto

### Supporto verticale

La GRF verticale deve essere usata come condizione di fattibilita', non come
obiettivo da massimizzare o curva da imitare:

```text
missing_support_loss =
    stance_mask * max(0, Fz_min - Fz_pros)^2

swing_load_loss =
    swing_mask * max(0, Fz_pros - Fz_swing_max)^2

overload_loss =
    max(0, Fz_pros - Fz_max)^2
```

Le maschere stance/swing devono essere continue e ricavate inizialmente dal gait
clock dell'arto sano con offset anti-fase.

Non va usato inizialmente il flag online-GRF `in_contact`, perche' nei rollout
precedenti e' risultato sempre attivo.

### Foot slip

Il plugin online-GRF calcola la velocita' tangenziale relativa al terreno per
ogni sfera di contatto. Il segnale aggregato corrente usa pero' il massimo tra
le sfere, incluse quelle non caricate.

Per la reward serve uno slip pesato dal carico:

```text
effective_slip =
    sum(Fnormal_i * slip_i)
    / (sum(Fnormal_i) + epsilon)

slip_loss =
    stance_mask * contact_weight
    * max(0, effective_slip - slip_tolerance)^2
```

Lo slip deve essere penalizzato solamente durante un contatto realmente
portante.

### Penetrazione

La penetrazione rimane:

- penalita' morbida oltre una soglia operativa;
- terminazione hard oltre una soglia critica.

## Saturazione e regolarizzazione

La saturazione SEA deve essere misurata su tutti i campioni interni del segmento,
non solamente sull'ultimo comando disponibile:

```text
saturation_loss =
    mean(max(0, abs(u) - u_soft)^2)
```

Vanno inoltre registrati:

- percentuale temporale in saturazione;
- massimo `abs(u)`;
- durata consecutiva massima della saturazione;
- valori separati per ginocchio e caviglia.

Una saturazione persistente deve essere penalizzata molto piu' di un breve
transitorio.

Restano inoltre:

- penalita' smoothness/command-rate;
- out-of-band sui riferimenti comandati;
- limiti articolari hard di anti-divergenza.

## Asymmetric actor-critic

### Modalita' imitation

Il critic puo' osservare:

- target IK protesici;
- errori rispetto alla IK;
- `kin_ref` biologico;
- stato globale e GRF bilaterali.

Questi segnali facilitano il pretraining imitativo.

### Modalita' ex-novo

Il critic deve osservare:

- stato fisico globale;
- GRF bilaterali;
- fase e contatti;
- metriche di impulso;
- slip, penetrazione e saturazione;
- eventuali statistiche temporali necessarie alla reward.

Target IK protesici e `kin_ref` devono essere mascherati o disattivati. In caso
contrario il critic potrebbe usare il riferimento sperimentale come scorciatoia
temporale anche se la reward non lo premia.

L'actor resta limitato ai segnali realisticamente disponibili sulla protesi,
oltre al gait clock contralaterale mantenuto temporaneamente come pacemaker.

## Formula concettuale della reward ex-novo V1

```text
task_score =
    w_propulsion   * propulsion_score
  + w_transition   * transition_score
  + w_coordination * coordination_score

feasibility_loss =
    w_missing_support * missing_support_loss
  + w_swing_load      * swing_load_loss
  + w_overload        * overload_loss
  + w_slip            * slip_loss
  + w_penetration     * penetration_loss

regularization_loss =
    w_saturation * saturation_loss
  + w_smoothness * smoothness_loss
  + w_oob        * out_of_band_loss

reward_ex_novo =
    task_score
  - feasibility_loss
  - regularization_loss
  - safety_penalty
```

Pesi inizialmente a zero:

```text
sea_energy_weight = 0
tracking_loss_weight = 0
reference_loss_weight = 0
bio_loss_weight = 0
auto_periodicity_weight = 0
```

L'auto-periodicita' andra' introdotta solamente dopo aver ottenuto un contatto
funzionale, per evitare di consolidare un comportamento periodico ma scorretto.

## Strategia di implementazione futura

1. Aggiungere le nuove metriche come diagnostica senza modificare subito la
   reward.
2. Verificare assi e segni della GRF AP sul modello AB06.
3. Calibrare bande, soglie e normalizzazioni sui rollout noti.
4. Correggere lo slip aggregato rendendolo contact-weighted.
5. Esporre statistiche di saturazione lungo l'intero segmento.
6. Mascherare condizionalmente IK e `kin_ref` nel critic `ex_novo`.
7. Attivare la nuova reward modificando l'attuale branch `ex_novo`.
8. Eseguire test comportamentali controllati prima di un training prolungato.

## Test comportamentali richiesti

La nuova reward dovra' verificare che:

1. supporto distribuito durante stance batta assenza di supporto;
2. supporto distribuito batta un singolo picco verticale;
3. piede senza contatto non ottenga vantaggio perche' ha slip zero;
4. supporto con slip sia peggiore di supporto stabile;
5. una traiettoria statica facile da tracciare non ottenga reward elevata;
6. saturazione persistente sia peggiore di un breve transitorio;
7. cambiare solamente IK e `kin_ref` non modifichi la reward `ex_novo`;
8. un migliore impulso AP protesico migliori il task score senza richiedere
   vicinanza alla IK.

## File analizzati

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/asymmetric_rl_module.py`
- `online_grf.py`
- `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp`
- `Trajectory Generator/literature/INDEX.md`
- `Trajectory Generator/literature/TOPICS.md`
- `Trajectory Generator/literature/README_FOR_LLM.md`
- `Trajectory Generator/literature/notes/P03_wen2020_wearer_prosthesis_symmetry.md`
- `Trajectory Generator/literature/notes/P04_li2023_hierarchical_knee_symmetry.md`
- `Trajectory Generator/literature/notes/P12_glide_quadrupedal_centroidal.md`
- `Trajectory Generator/literature/notes/P16_majd_nn_repair_assistive.md`
- `Trajectory Generator/literature/notes/P19_eco_energy_constrained.md`
- `Trajectory Generator/literature/notes/P22_lswm_world_model_bipedal.md`
- PDF originali prioritari P03, P04, P05, P11, P12, P19 e P22.

## File modificati

- `reports/user/2026-06-11_progettazione_reward_ex_novo_task_based.md`

Nessun file di codice, configurazione o plugin e' stato modificato.

## Verifiche eseguite

- inventariata la knowledge base locale della letteratura;
- consultati indici tematici e note strutturate;
- verificati direttamente nei PDF originali i passaggi principali relativi a:
  - impulsi AP e simmetria;
  - obiettivi funzionali protesici;
  - critic privilegiato;
  - reward, costi e vincoli di locomozione;
  - contatto del piede e sicurezza;
- confrontati i segnali richiesti con quelli attualmente disponibili nel
  simulatore e nel plugin online-GRF.

Non sono stati eseguiti training o rollout, poiche' questa fase e' stata
esplicitamente limitata alla progettazione.

## TODO aperti

- [ ] Confermare formalmente che l'obiettivo primario V1 sia la combinazione di
  impulso propulsivo protesico e coordinazione inter-limb.
- [ ] Verificare assi, segni e normalizzazione delle GRF AP sul modello AB06.
- [ ] Definire le finestre di braking/propulsion rispetto al gait clock.
- [ ] Stabilire se il confronto iniziale deve usare la GRF sana prescribed
  direttamente oppure bande funzionali derivate.
- [ ] Calibrare `Fz_min`, `Fz_swing_max`, `Fz_max`, `slip_tolerance` e `u_soft`.
- [ ] Rendere lo slip aggregato contact-weighted.
- [ ] Esporre saturazione e relativi eventi su tutti i campioni del segmento.
- [ ] Definire il masking condizionale di target IK e `kin_ref` nel critic
  `ex_novo`, preservando compatibilita' con il warm-start imitativo.
- [ ] Implementare prima le metriche in modalita' diagnostica.
- [ ] Definire e automatizzare i test comportamentali della reward.
- [ ] Introdurre auto-periodicita' solamente dopo la validazione del task V1.
- [ ] Mantenere `sea_energy_weight = 0` nella prima implementazione.
