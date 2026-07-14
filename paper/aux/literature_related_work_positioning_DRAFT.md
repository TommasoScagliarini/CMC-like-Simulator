# Literature Map and Related-Work Positioning

Questo documento sintetizza i paper presenti in `Trajectory Generator/literature`
utili per scrivere la sezione di related work e per posizionare il contributo del
progetto. Va letto insieme a:

- `paper/paper_outline.md`
- `Trajectory Generator/literature/INDEX.md`
- `Trajectory Generator/literature/TOPICS.md`
- `Trajectory Generator/literature/notes/Pxx_*.md`

## Contesto del progetto

Il lavoro sul `Trajectory Generator` non e' ancora nella fase finale ex-novo. Lo
stato attuale e' una fase di pretraining imitativo e validazione dell'architettura.
L'obiettivo scientifico finale resta:

```text
generare traiettorie cinematiche protesiche ex-novo tramite RL
```

ma nel paper, finche' i risultati ex-novo non sono completi, conviene formulare il
contributo in modo prudente:

```text
We develop and validate the architecture for reinforcement-learned prosthetic
reference generation, and currently use imitation pretraining as the first stage
toward ex-novo trajectory generation.
```

## Paper da citare subito

| Ruolo nel paper | Paper |
|---|---|
| Precedenti diretti su protesi RL | P01, P02, P03, P04, P05 |
| OpenSim + DRL muscoloscheletrico | P06, P07, P08, P09, P11, D02 |
| Imitation / pretraining | P08, P10, P11 |
| Action space e gerarchia high-level/low-level | P04, P12, P13, S02 |
| SEA / motivazione biomeccanica | P20 |
| Reward energia/metabolico/fisica | P19, P21 |
| POMDP / memoria / partial observability | P22 |
| Safety / future work | P14, P15, P16, P17, P18 |

## Ispirazione diretta

### 1. Protesi RL con controllo gerarchico

P01, P02 e P04 usano RL o BO+RL per regolare parametri di impedenza o target
cinematici. Da qui deriva l'idea generale:

```text
high-level learner -> target/parametri -> low-level prosthesis controller
```

Il paper piu' vicino concettualmente e' P04, perche' sceglie target cinematici
per migliorare la simmetria propulsiva. Tuttavia P04 lavora su feature/target di
ginocchio e impedenza, non su una traiettoria continua knee+ankle per una protesi
SEA.

### 2. OpenSim + DRL muscoloscheletrico

P06 e D02 fondano l'idea di OpenSim come ambiente Gym/RL. P08, P10 e P11 sono
importanti per PPO + imitation learning. P11 e' il precedente piu' vicino al
dominio del progetto: transfemoral prosthesis + OpenSim + PPO/imitation.

La differenza chiave e' che P11 usa prosthetic muscle-like actuators, mentre questo
progetto usa una protesi knee+ankle con attuatori SEA e un generatore high-level di
reference cinematiche.

### 3. Action-space design

P13 e' utile per discutere torque vs position/reference action. S02 evidenzia che,
per una protesi SEA, un high-level AI che produce traiettorie/reference lisce e'
piu' sensato di un torque end-to-end.

Questa linea supporta:

- il reference governor;
- il fatto che la rete non controlli direttamente `u`, torque o impedenza;
- la separazione tra trajectory generation high-level e tracking low-level.

## Dove il lavoro si discosta

Frase centrale consigliata:

```text
Prior prosthesis RL mainly tunes impedance parameters or target features;
we train an RL policy to generate the continuous kinematic reference trajectory
for a knee-ankle series-elastic transfemoral prosthesis.
```

Differenze principali:

- P01/P02/P03/P04/P05: protesi knee, impedance tuning, feature-level target,
  human-in-the-loop o OpenSim, ma non trajectory generator continuo knee+ankle.
- P11: OpenSim + transfemoral + DRL, ma la policy controlla muscle-like/prosthetic
  actuator forces, non reference cinematiche per SEA.
- P06/P08/P10: musculoskeletal motion imitation, ma non protesi SEA e non reference
  generator per low-level cascade.
- P12/P13/P19/P22: ottimi per motivare action space, gerarchia, critic privilegiato
  e partial observability, ma non sono protesi/OpenSim.
- P20: supporta la scelta SEA e il bisogno di ankle work, ma non e' RL.

## Innovazione da rivendicare con cautela

Da evitare, finche' il trajectory generator ex-novo non e' completo:

```text
we demonstrate ex-novo RL trajectory generation
```

Formula piu' corretta nello stato attuale:

```text
We develop a reinforcement-learning architecture for prosthetic reference
generation and validate its action interface, low-level feasibility, and
imitation-pretraining stage toward ex-novo trajectory generation.
```

Innovazioni difendibili gia' ora:

- action space come absolute kinematic reference, non torque e non impedance
  increments;
- reference governor band-limited tra policy e cascade, necessario per non eccitare
  la dinamica SEA;
- protesi transfemorale knee+ankle SEA in OpenSim/CMC-like;
- lato biologico mantenuto muscle-driven, con reserve come misura di fedelta'
  fisica;
- actor-critic asimmetrico: actor deployable, critic privilegiato;
- impostazione futura ex-novo, non semplice tracking IK.

## Related work consigliata

### 1. RL for robotic prostheses

Citare P01-P05.

Conclusione da far emergere:

```text
Questi lavori ottimizzano impedenza, target feature o simmetria, ma non generano
una traiettoria reference continua per una protesi SEA knee+ankle.
```

### 2. DRL in musculoskeletal/OpenSim simulation

Citare P06-P11 e D02.

Conclusione da far emergere:

```text
OpenSim+RL e imitation learning sono consolidati, ma i precedenti controllano
muscoli, eccitazioni o actuator-like forces, non un high-level prosthetic
trajectory generator.
```

### 3. Action spaces and hierarchical locomotion control

Citare P04, P12, P13 e usare S02 come guida interna o supporto secondario.

Conclusione da far emergere:

```text
La letteratura supporta action space astratti e low-level tracker; il contributo
del progetto e' specializzare questa idea su una protesi SEA con governor
fisicamente vincolato.
```

### 4. Safety, energy and partial observability

Citare P14-P22 soprattutto in motivazione, discussion, limitations e future work.

P22 e' particolarmente utile per dire che il problema e' parzialmente osservabile,
ma che la policy attuale e' una MLP feed-forward senza memoria esplicita. Questo
puo' diventare un limite dichiarato e una direzione futura: history stacking,
GRU/LSTM o world-model/belief estimator.

## Paper meno centrali

P23/P24, legati a RLVR e language models, sono periferici per il paper corrente.
P25/D01 sono utili solo se si vuole discutere software deployment o integrazione
C++/Python, ma non dovrebbero stare nella related work principale.

S02 e' utile come sintesi progettuale, ma nella bibliografia conviene appoggiarsi
primariamente ai paper originali che S02 riassume.

## Nucleo forte per la narrativa

Il nucleo minimo piu' forte e':

```text
P04 + P11 + P13 + P20 + P22
```

Da questo si costruisce il contrasto:

```text
target/impedance tuning
vs
reference trajectory generation for a physical SEA prosthesis
```

## Take-home message

La tesi da portare nel paper non e' "RL controlla la protesi", ma:

```text
RL genera una reference cinematica protesica;
il reference governor la rende compatibile con la banda dell'attuatore;
il cascade/SEA/OpenSim muscle-driven testbed verifica se quella reference e'
fisicamente onesta.
```

