# Durata degli episode, phase-clock del gait e traiettorie ex-novo (analisi di design)

Data: 2026-06-09

## Problema

Oggi la durata degli episode di training è **fissa**. Si voleva valutare se
adottare una **durata variabile basata sul gait cycle**: far durare un episode
da Heel Strike (HS) al successivo. La discussione, partita da questa idea, è
evoluta in un'analisi di design più ampia su come strutturare gli episode, la
fase del cammino e la funzione obiettivo, in coerenza con l'obiettivo finale del
progetto: far **generare alla rete traiettorie cinematiche protesiche ex-novo**
compatibili col modello (non imitare il dato sperimentale).

Questo report è un'**analisi progettuale**: nessuna modifica al codice.

## Punti di partenza e riformulazione

### Il reference è time-indexed, non phase-indexed

La cinematica di riferimento è indicizzata nel **tempo**, non nella fase. Quindi
"HS → HS" può essere realizzato in due modi con costi molto diversi:

- **stop-condition** (basso rischio): si usa l'HS solo per decidere quando
  terminare l'episode, lasciando il reference time-indexed;
- **re-fasatura / time-warp** (alto costo): si reindicizza il reference sulla
  fase, toccando interpolatore cinematico, outer loop e GRF prescribed.

### Stato del rilevamento HS (bug aperto, NON validato per questo uso)

Va corretto un equivoco emerso: ciò che è stato validato l'8 giugno è la **GRF
online (wrench) su finestra gait-scale 2 s con cinematica oracolo/prescribed**,
con esito *strutturalmente negativo* (la online pura non rifasa il wrench → da
qui l'architettura ibrida). Restano invece **aperti** due bug del rilevamento
eventi, esplicitati nei TODO del daily 2026-06-08 e non chiusi:

- timing **heel-strike del lato sinistro (protesico)** sfasato nei rollout con
  policy viva;
- flag onlineGRF `in_contact` **sempre attivo** nel rollout.

Conseguenza: l'HS del **lato sano (destro)** è affidabile perché prescribed e
deterministico; l'HS del **lato protesico (sinistro)** rilevato **online** è il
segnale ancora rotto. La distinzione è netta: i bug colpiscono solo la
rilevazione **online con policy viva**, non la conoscenza degli eventi nel
**reference** (che restano deterministici e leggibili offline dai dati).

## Analisi: sfasatura dei due cicli e regime ex-novo

In regime **prescribed** la sfasatura tra i due lati è esatta (`~T/2`) perché
entrambe le gambe leggono lo stesso orologio (i dati IK). Il periodo `T` è lo
stesso per i due lati: una finestra HS-sano → HS-sano contiene quindi **un ciclo
protesico completo** (tutte le fasi `[0,1]` percorse una volta), solo che parte
dalla fase protesica `~0.5` invece che da `0`.

**Ma in regime ex-novo questa garanzia cade.** Tolta l'imitazione sul lato
protesico (che è l'obiettivo), si ha:

- lato sano = **clock fisso** (prescribed, periodo `T` imposto dai dati);
- lato protesico = **libero** (lo disegna la policy).

Nulla, strutturalmente, lega periodo e fase della protesi a quelli del lato
sano. Fissare il bordo dell'episode sull'HS sano garantisce un ciclo *sano*
completo, **non** che la protesi compia il suo ciclo dentro quella finestra. La
coordinazione bilaterale, in ex-novo, va **assicurata** (per costruzione) o
**selezionata** (premiata), non data per scontata.

### Tre livelli per assicurare la coordinazione

1. **Garanzia per costruzione — reference phase-indexed (phase-clock):** una
   variabile di fase `φ ∈ [0,1)` guidata dal lato sano; tutto ciò che riguarda
   la protesi è funzione di `φ` e non di `t`. "Un ciclo sano = un ciclo
   protesico" per costruzione. Più invasivo (tocca l'indicizzazione), ma è
   l'unico che *garantisce* la coordinazione lasciando libera la forma.
2. **Selezione via reward + criterio terminale:** si resta time-indexed e si
   *incentiva* la coordinazione (premio se l'evento protesico cade alla fase
   giusta, bonus terminale se il ciclo si chiude in finestra, penalty/terminazione
   se perde periodicità). Non garantisce, ma "seleziona per".
3. **Entrainment meccanico (supporto):** la protesi è attaccata a un bacino che
   si muove periodicamente a `T` → accoppiamento dinamico che spinge la protesi
   nel ritmo. Reale ma debole: alleato di fondo, non garanzia.

## Architettura convergente

La direzione scelta è **opzione 2 + opzione 3 a sostegno**, con una **variabile
di fase `φ`** come infrastruttura. Questo abbandona la durata-variabile-HS-a-HS
in favore di una soluzione più solida:

- **Episode = orizzonte lungo fisso** (es. 10 s, ~8-10 cicli). Tutti gli episode
  hanno la stessa lunghezza → si **elimina** il coupling durata↔return e la
  varianza degli orizzonti variabili in PPO (problemi che la durata variabile
  avrebbe introdotto). Più cicli per episode aiutano la rete a imparare la
  periodicità.
- **Clock `φ`** dal lato **sano** (prescribed, deterministico): onda a dente di
  sega che sale `0 → 1` lungo un ciclo sano e torna a `0` all'HS sano
  successivo. È il pacemaker e l'informazione "dove sono nel ciclo" che la policy
  deve avere. Nota: nell'env esiste già un'impalcatura di gait phase
  (`_online_gait_info`/`gait_phase`) ma cablata sul detector **online/protesico**
  (rotto); per il pacemaker serve la fase del lato **sano**, che è deterministica
  e non dipende dal bug.
- **`φ` in osservazione come `(sin φ, cos φ)`** (evita la discontinuità `1→0`).
- **"Reset" = bookkeeping, non stato fisico.** Chiarito un equivoco: durante
  l'episode la simulazione gira in continuo e lo stato fisico della protesi
  (angoli, stati motore SEA) **non si teletrasporta mai**. Ciò che "resetta" a
  `φ=1` è solo il wrap del contatore di fase / l'indice su cui il reward valuta
  il ciclo. "La protesi resetta ogni ciclo sano" significa: il suo
  comportamento atteso è **periodico e agganciato al clock sano**.
- **Offset del punto di reset = parametro tunabile** (default = HS sano; si
  potranno provare toe-off, mid-stance per cercare il punto che dà il training
  più stabile). Non deciso a priori.

### Imitazione vs ex-novo: cosa misura il reward

Distinzione chiave emersa: **il clock `φ` è agnostico** rispetto all'obiettivo,
ma la **forma** è dove imitazione ed ex-novo divergono.

- **Imitazione (NON è l'obiettivo del progetto):** reward = tracking di un
  template `φ`-indicizzato costruito dalle colonne `pros_knee_angle` /
  `pros_ankle_angle` del `.mot`. Shape + timing (anti-fase) insieme,
  **detector-free**.
- **Ex-novo (obiettivo del progetto):** **non** esiste un template da inseguire;
  la forma la deve scoprire la rete. Il wrap di `φ` non serve più a rileggere un
  riferimento, ma diventa il confine su cui si misura:
  - **auto-periodicità** (template-free): confronto della protesi **con sé stessa
    al ciclo precedente** alla stessa `φ` → premia un cammino ripetibile senza
    prescriverne la forma (serve un buffer dell'ultimo ciclo, economico);
  - **coordinazione/anti-fase** col lato sano;
  - **obiettivi di task** (stabilità, fattibilità GRF/penetrazione, effort/energia
    SEA, ROM/no divergenze, smoothness) — da cui la **forma emerge**.

### Curriculum imitazione → task (P.S. dell'utente)

L'imitazione resta utile come **pre-training**, ed è elegante perché la macchina
di fase è **identica** nei due regimi:

1. pre-train imitativo (tracking del template `φ`-indicizzato dalle colonne
   `pros_*`) → la rete parte da una falcata sensata;
2. switch a task-based: stessa `φ`, stessa osservazione, stesso reset; cambia
   **solo** il reward (periodicità + coordinazione + obiettivi di task).

Coerente con i TODO già a roadmap "curriculum imitativa→task-based" e "separare
fattibilità da imitazione".

## Verifiche fattuali eseguite (read-only)

- **Finestra dati IK corretta:** `models/AB06_SEASEA_Threadmill/data/
  IK_results_AB06_SEASEA.mot` va da **11.99 s a 155.045 s** (~143 s, 28 612
  righe a 200 Hz, `inDegrees=yes`). Smentita l'ipotesi iniziale di una finestra
  ~6.8 s (che era del `config`, riferita a un altro file di cinematica). Episode
  da 10 s sono ampiamente disponibili **senza stitching**; con così tanti dati
  diventa sensato e sicuro anche `random_init=True` (il phase-clock calcola `φ`
  dall'HS sano più vicino qualunque sia l'istante di partenza).
- Il `.mot` contiene già le colonne `pros_knee_angle` e `pros_ankle_angle`
  (sorgente naturale del template imitativo per il pre-training).
- Riletti `reward_function.py` (meccanica del reward: score, penalty, termini
  `safety`/`oob` sottratti dopo il clip per mantenere gradiente) e i report
  daily 2026-06-08 / utente 2026-06-09 per ricostruire lo stato dei bug HS.

## File modificati

Nessuno: discussione puramente progettuale, nessuna modifica al codice del
simulatore, della pipeline RL, dei SEA o del plugin C++.

## TODO

### Decisioni di design da chiudere prima dell'implementazione
- Definire la **gerarchia degli obiettivi** del regime task-based ex-novo:
  quali sono **vincoli duri** (terminazione: caduta, `joint_divergence`,
  penetrazione GRF, ROM modello), quale l'**obiettivo primario** (efficienza
  energetica/energia SEA, stabilità/robustezza, simmetria col lato sano, o
  combinazione) e cosa è contorno. È il prerequisito per scrivere il reward
  task-based.
- Confermare che **periodicità + coordinazione** restano termini di reward
  sempre attivi ("deve restare un cammino ciclico e in fase").
- Tarare empiricamente l'**offset del punto di reset** di `φ` (HS sano vs
  toe-off vs mid-stance).
- **Approfondire la strategia per il pre-training imitativo** (come e quando
  agganciarlo al template `φ`-indicizzato, criterio di passaggio a task-based).
- **Discutere la costruzione della reward** task-based (forma dei termini di
  auto-periodicità, coordinazione e task; pesi e blend; gradiente residuo).

### Implementazione (dopo le decisioni, tutto dentro `Trajectory Generator/`)
- Estrazione **HS sano + periodo `T`** dai dati sperimentali → definizione del
  clock `φ`.
- **`φ` in osservazione** come `(sin φ, cos φ)` (ricablare la gait phase sul lato
  sano/prescribed invece che sul detector online).
- **Episode a orizzonte lungo fisso** (~10 s) + offset di reset tunabile.
- Reward **task-based** (auto-periodicità template-free + coordinazione + termini
  di task) e, opzionale, **pre-train imitativo** col template `φ`-indicizzato
  dalle colonne `pros_*` del `.mot`.

### Prerequisito per gli sviluppi "evento discreto" / full task-based
- Correggere i due bug HS aperti (timing heel-strike **protesico** online; flag
  `in_contact` sempre attivo): necessari solo se si vorrà un bonus al contatto
  esatto o un trigger basato sull'evento protesico online. Il percorso
  phase-indexed/auto-periodicità qui descritto **non** ne dipende.

### TODO ereditati e propagati (dai report precedenti, ancora aperti)
- Far sì che il **timeout di iterazione/sampling** uccida davvero un'iterazione
  in stallo (oggi solo il run-timeout totale ha effetto); troncare gli episodi
  degeneri che bloccano il sampling.
- Ridurre alla radice le divergenze `joint_divergence_pros_knee_angle`, la
  saturazione del knee SEA e i fallback bounded least-squares della Static
  Optimization.
- Eseguire un rollout deterministico di `rl_module_best` (2 s, record) del run
  ibrido notturno per misurare cosa ha imparato in ~40 iter.
- Allineamento macOS: `setuptools<81` nell'env `envCMC-rllib`; ricompilare/
  riconfermare il plugin onlineGRF `.dylib` su arm64.
- Migliorare il critic (explained variance negativa); tarare `oob_weight` con
  `reward/oob_term`; rivalidare il filtro 6 Hz su rollout lungo.
- Metriche di suitability per le traiettorie ex-novo (velocità, stabilità,
  simmetria, effort, energia SEA, fattibilità GRF).
- Housekeeping repo, knowledge base letteratura, controllo SEA storici (come da
  daily 2026-06-08).
