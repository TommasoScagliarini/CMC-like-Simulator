# Progetto della rete unificata imitativa/ex-novo e dell'actor predittivo

**Data:** 2026-08-31  
**Stato:** specifica architetturale iniziale; separazione morphology knee/ankle
implementata; critic-only rewarm di controllo completato  
**Baseline operativa:** actor V26C/J20 derivato dal warm start imitativo V26 di
agosto  
**Scopo:** costituire il punto di partenza verificabile per progettare la nuova
rete, evitando di reintrodurre dipendenze da segnali non disponibili sulla
protesi e separando nettamente warm start imitativo e ottimizzazione ex-novo

## 1. Decisione architetturale

La struttura proposta è:

> Un solo actor deployable, con un unico contratto di osservazione e di azione,
> addestrato in due modalità successive. Nel warm start sono attivi i termini
> imitativi e spenti quelli puramente ex-novo; nella seconda fase i termini
> imitativi vengono spenti e l'actor viene ottimizzato con la sola reward
> ex-novo, mantenendo sempre attive le protezioni di sicurezza.

In forma schematica:

```text
sensori e stato causale deployable
              |
              v
       actor unico 35D
              |
              v
      target raw knee/ankle
              |
              v
  slew limiter -> filtro C2 -> served reference -> modello/SEA

warm start:  reward imitativa ON,  reward ex-novo OFF, safety ON
ex-novo:     reward imitativa OFF, reward ex-novo ON,  safety ON
```

Questa struttura può funzionare. Non rende però automaticamente l'actor
predittivo e non elimina da sola il domain shift tra teacher e student. Sono
necessari un teacher coerente con la catena `slew + filtro`, raccolta DAgger
closed-loop e un passaggio controllato tra le due reward.

## 2. Campi della rete e segnali della reward sono concetti diversi

La rete non deve ricevere come osservazione tutto ciò che è disponibile nel
dataset o nel simulatore. Il criterio corretto è:

| Componente | Contratto |
|---|---|
| Actor | Solo feature causali realmente disponibili sulla protesi |
| Azione actor | Stessa semantica nelle due fasi |
| Teacher imitativo | Può usare prescribed e dati offline privilegiati |
| Reward imitativa | Può confrontare l'azione con target offline |
| Critic | Può usare osservazioni privilegiate durante il training |
| Reward ex-novo | Può usare grandezze del simulatore non osservate dall'actor |
| Safety | Sempre attiva nel warm start e nell'ex-novo |

I segnali controlaterali o prescribed possono quindi essere usati per costruire
label, target e reward. Non devono essere aggiunti all'actor se non saranno
disponibili nel deployment. In caso contrario l'actor imparerebbe una
scorciatoia destinata a sparire nell'ex-novo o sulla protesi reale.

### 2.1 Contratto J20 da conservare nella prima iterazione progettuale

Il checkpoint J20 usa un actor MLP `35 -> 256 -> 256 -> 2`. Le 35 colonne sono
già presenti nello schema usato nel runtime ex-novo:

- cinematica protesica knee/ankle;
- stati motore SEA;
- GRF online ed eventi causali;
- fase e stato della FSM v3;
- durata online del ciclo;
- precedente endpoint dell'actor;
- stato `q/v/a` della served reference;
- comando SEA precedente.

Le due colonne storiche `gait_phase_sin` e `gait_phase_cos` sono mantenute per
compatibilità tensoriale ma intenzionalmente azzerate; la fase utilizzabile è
quella online derivata dal detector causale. Il transplant audit ha copiato 33
feature operative e mantenuto a zero queste due colonne.

La prima versione dell'actor predittivo non richiede quindi di allargare la
rete né di cambiare l'output. La predittività può essere introdotta cambiando
la semantica del target appreso, non necessariamente l'architettura.

## 3. Reward a due modalità

Il contratto generale è:

\[
R = R_{safety} + \alpha_I R_{imitation} + \alpha_E R_{exnovo}
\]

Durante il warm start:

\[
\alpha_I=1,\qquad \alpha_E=0
\]

Durante l'ex-novo:

\[
\alpha_I=0,\qquad \alpha_E=1
\]

I termini disattivati possono essere calcolati in `shadow mode` per il logging,
ma devono contribuire esattamente zero all'obiettivo. La composizione deve
usare rami espliciti e non prodotti del tipo `0 * NaN`.

### 3.1 Cosa resta sempre attivo

Restano indipendenti dalla modalità:

- hard guard di penetrazione GRF a `28 mm`;
- soglia soft a `20 mm`;
- soglia diagnostica storica di luglio a `25 mm`;
- limiti articolari e saturazioni critiche;
- validità causale degli eventi e della FSM;
- protezioni che impediscono traiettorie non deployable.

### 3.2 Passaggio dal best imitativo all'ex-novo

Al cambio completo della reward:

1. si conserva l'actor del best imitativo;
2. si conserva lo stesso ordine delle osservazioni e la stessa semantica
   dell'azione;
3. si azzerano matematicamente i termini imitativi;
4. si crea un critic nuovo, oppure si resetta completamente il critic
   precedente;
5. si resetta lo stato dell'optimizer, per non ereditare i momenti Adam
   associati al precedente obiettivo;
6. si svolge un breve warm-up critic-only con actor e `logstd` congelati;
7. si calibra l'esplorazione con uno sweep controllato di sigma;
8. si sblocca l'actor e si avvia PPO ex-novo.

Il checkpoint iniziale resta matematicamente un warm start dei parametri, ma
non è più vincolato da una reward imitativa. La capacità di allontanarsi dalla
baseline dipenderà da esplorazione, segnale di credito e landscape della reward,
non dalla presenza residua del termine imitativo.

## 4. Come rendere predittivo l'actor

### 4.1 Problema misurato

L'audit frozen-policy del 2026-08-30 ha scomposto la catena temporale:

| Tratto | Knee | Ankle |
|---|---:|---:|
| actor raw -> dopo slew | `10 ms` | `20 ms` |
| dopo slew -> served C2 | `80 ms` | `80 ms` |
| served -> cinematica fisica | `0 ms` | `0 ms` |
| actor raw -> served complessivo | `90 ms` | `100 ms` |

Il problema non è principalmente il tracking SEA della served reference. È il
fatto che il comando generato ora raggiunge la served reference circa
`0.09-0.10 s` dopo. La reward morphology causale aggiunge inoltre `40 ms` di
ritardo nell'assegnazione del credito; quel ritardo non modifica fisicamente il
comando, ma rende più difficile attribuire la penalità alla decisione che ha
causato l'uscita dal corridor.

### 4.2 Definizione operativa di actor predittivo

L'actor deve imparare una politica:

\[
u_t = \pi(o_t)
\]

tale che, dopo il reference governor deterministico:

\[
q^{served}_{t+h} \approx q^{desired}_{t+h}
\]

con un orizzonte iniziale vicino al ritardo misurato per ciascun giunto. In
altre parole, l'output raw deve iniziare il movimento prima affinché l'uscita
del filtro sia corretta al momento biomeccanico richiesto.

La predizione non richiede informazioni future in input. L'actor può inferire
il futuro prossimo da segnali causali già disponibili: fase online, stato FSM,
durata del ciclo, cinematica e velocità, stato corrente del filtro e precedente
target. È la stessa logica con cui un controllore model-predictive anticipa la
dinamica pur osservando solo lo stato presente.

### 4.3 Perché non basta traslare sempre il target di 100 ms

Un anticipo fisso è utile come probe, ma non è la soluzione finale:

- il ritardo di un low-pass dipende dalla frequenza del segnale;
- lo slew limiter è non lineare e interviene in modo diverso in base alla
  distanza dal precedente endpoint;
- knee e ankle hanno limiti diversi;
- il periodo del passo varia;
- il profilo ankle AB06 cambia sensibilmente con la cadenza;
- l'errore di fase già presente nel raw actor non coincide con il solo ritardo
  del governor.

Nel luglio 2026 furono provati anticipi globali `0.15/0.20/0.25/0.40 s` e
anticipi separati. L'ankle anticipato di `0.20 s` migliorava leggermente la
sopravvivenza, ma nessuna prova arrivava al TO. L'indagine trovò poi un bug nel
limiter: il clamp era ancorato alla reference filtrata anziché al precedente
target accettato. Corretto quel bug, il teacher senza lookahead completò
`500/500` step. Questa storia dimostra che un lookahead statico non deve essere
usato per mascherare errori nella catena di controllo; oggi la catena corretta
va invece invertita esplicitamente.

### 4.4 Strategia raccomandata: teacher inverse-governor

La prima implementazione consigliata mantiene l'MLP e le due azioni assolute
attuali. Si costruisce offline un teacher che conosce esattamente:

- slew rate knee/ankle;
- stato iniziale del governor;
- filtro C2 `butterworth3_jerk_limited` a `4 Hz`;
- bounds articolari e di azione;
- traiettoria served desiderata.

Su ogni finestra, il teacher cerca una sequenza raw `u[t:t+H]` che minimizzi:

\[
\sum_{k=0}^{H}
\left\|q^{served}_{t+k}(u)-q^{desired}_{t+k}\right\|^2_W
+ \lambda_{\Delta u}\|\Delta u\|^2
+ \lambda_{\Delta^2 u}\|\Delta^2 u\|^2
\]

sottoposta alla stessa catena deterministica presente nel deployment. L'label
per l'actor è il primo comando raw della sequenza ottima, non la posizione
prescribed dello stesso timestamp.

Questo approccio:

- compensa congiuntamente slew e filtro;
- produce label diverse in base allo stato reale del governor;
- può usare orizzonti distinti per knee e ankle;
- mantiene invariata l'interfaccia deployable dell'actor;
- evita di spostare artificialmente il corridor finale.

### 4.5 DAgger predittivo

Il fit nominale del teacher inverse-governor non è sufficiente. Dopo la prima
distillazione:

1. si esegue l'actor in closed loop;
2. si raccolgono gli stati realmente visitati dall'actor, incluso lo stato del
   governor;
3. il teacher ricalcola l'azione corretta da quegli stati;
4. si aggiungono le coppie osservazione/azione al dataset;
5. si riaddestra lo stesso actor 35D;
6. si interrompono le recovery al primo mismatch FSM non recuperabile, come nel
   protocollo July-faithful già ricostruito;
7. si valida esclusivamente in closed loop.

La raccolta DAgger deve includere variazioni di fase, stato del filtro e
cadenza. Se il dataset contiene soltanto la traiettoria nominale, l'actor non
impara come anticipare dopo una deviazione.

### 4.6 Alternative successive

Se l'inverse-governor offline non fosse sufficiente, le alternative in ordine
di complessità sono:

1. unroll differenziabile del governor nella loss supervisionata;
2. piccolo forward model appreso `actor raw -> served futura`, validato contro
   il governor esatto;
3. output a breve orizzonte, di cui il runtime consuma il primo comando;
4. decoder deterministico C2 con parametri prodotti dall'actor.

Non è consigliato iniziare allargando la rete: i tentativi storici con capacità
maggiore non avevano risolto il comportamento closed-loop. Prima va corretto il
target di apprendimento e misurata la sufficienza dell'MLP 256x256.

## 5. Separazione dei termini morphology knee e ankle

### 5.1 Implementazione

`RewardConfig` accetta ora:

```yaml
morphology_weight: 1.05
morphology_knee_weight: 1.05
morphology_ankle_weight: 1.05
```

I due campi nuovi sono opzionali. Se omessi, ereditano
`morphology_weight`, preservando i vecchi YAML.

Poiché la loss storica è:

\[
L_{morph}=\frac{L_{knee}+L_{ankle}}{2}
\]

il nuovo termine è:

\[
T_{morph}=\frac{w_{knee}L_{knee}+w_{ankle}L_{ankle}}{2}
\]

Con `w_knee = w_ankle = morphology_weight` il risultato è esattamente quello
storico. Il codice mantiene espressamente la vecchia aritmetica quando i pesi
espliciti sono uguali, così la baseline non cambia neppure per roundoff.

Sono esposti separatamente nel logging:

- `morphology_knee_weight_effective`;
- `morphology_ankle_weight_effective`;
- `morphology_knee_term`;
- `morphology_ankle_term`;
- flag di non finitezza per ciascun giunto.

Un giunto con peso zero non valuta la propria loss. Questo impedisce che un
valore non finito disattivato contamini la reward tramite `0 * NaN`.

### 5.2 Baseline neutra scelta

Non essendo stato ancora specificato un nuovo rapporto knee/ankle, è stata
creata una configurazione esplicitamente separata ma neutra:

```json
{
  "morphology_weight": 1.05,
  "morphology_knee_weight": 1.05,
  "morphology_ankle_weight": 1.05
}
```

Questa configurazione verifica plumbing, serializzazione e critic rewarm senza
confondere l'esperimento con un nuovo peso ankle. Non costituisce ancora il
pilot ankle-specific.

### 5.3 Implicazione per i futuri esperimenti

Per aumentare soltanto la pressione sull'ankle si varierà
`morphology_ankle_weight`, lasciando fisso quello knee. Poiché questo cambia la
reward ex-novo, ogni candidato asimmetrico deve partire dallo stesso actor di
controllo e da un critic nuovamente riscaldato sul relativo obiettivo.

La separazione dei pesi non corregge però la latenza: rende il segnale
selettivo, ma una penalità ankle che arriva tardi può continuare a non produrre
il comportamento desiderato. Il pilot asimmetrico va quindi eseguito dopo, o
insieme a, una correzione esplicita del credito/predizione.

## 6. Critic-only rewarm eseguito

### 6.1 Sorgente

Actor sorgente:

```text
Trajectory Generator/baseline_MLP/validation/
v26c_july_replica_2026-08-26/j20_runs/
j20_critic_warmup_v26c_2026-08-27_r1/rl_module_last
```

Manifest delle feature:

```text
Trajectory Generator/baseline_MLP/validation/
v26c_july_replica_2026-08-26/
v26c_j20_actor_feature_manifest_overlay.json
```

### 6.2 Contratto del run

- una iterazione;
- `4096` step nominali;
- actor congelato;
- `logstd` congelata;
- critic inizializzato fresco;
- optimizer del nuovo run;
- learning rate `1e-4`;
- `10` epoche;
- reward morphology esplicitamente separata `1.05/1.05`;
- stessa FSM v3 e stesso contratto causale J20.

Run valido:

```text
Trajectory Generator/runs/training/
MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2
```

Il primo tentativo, senza permessi sufficienti al processo macOS per le letture
`psutil/sysctl`, si è fermato prima di inizializzare Ray e prima di qualsiasi
update:

```text
Trajectory Generator/runs/training/
MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31
```

Quella directory è un artefatto incompleto e non è un checkpoint candidato.

### 6.3 Risultati verificati

| Verifica | Esito |
|---|---:|
| step campionati | `4096` |
| actor, 10 tensori | byte-identico alla sorgente |
| actor max absolute diff runtime | `0.0` |
| `logstd` | byte-identica |
| sigma knee/ankle | `0.00499999967` |
| mean KL | `0.0` |
| critic digest iniziale | `5ce4ef41...` |
| critic digest finale | `b8874489...` |
| critic digest cambiato | sì |
| VF loss | `0.00440714` |
| VF explained variance | `0.791205` |
| `phase_timeout_stance` | `0` |
| `morphology_causal_contract_failure` | `0` |
| `resync_count_max` | `0` |
| `hs_cancelled_count_max` | `0` |
| telemetria mancante | `0/4096` |

L'audit automatico ha passato tutti i gate. Questo prova che è possibile
adattare il critic al contratto split mantenendo invariato l'actor. Poiché i
due pesi sono uguali, la reward numerica è ancora la stessa del caso globale:
il run è una baseline infrastrutturale, non evidenza che un peso ankle maggiore
risolva il toe-off.

## 7. Pipeline proposta per la nuova rete

### U0 — Congelamento dei contratti

Definire e versionare:

- schema actor deployable;
- schema critic privilegiato;
- semantica delle due azioni;
- dinamica esatta di slew e filtro;
- modalità reward `imitation` ed `exnovo`;
- provenance dei target e del corridor.

Gate: nessuna feature osservata dall'actor deve essere assente sulla protesi.

### U1 — Generazione del teacher predittivo

Costruire label raw tramite inverse-governor e verificare offline che, passate
nella catena congelata, ricostruiscano la served desiderata. Eseguire uno sweep
dell'orizzonte attorno a `90 ms` knee e `100 ms` ankle, includendo valori
inferiori e superiori.

Gate: errore served-vs-desired ridotto rispetto al teacher timestamp-aligned,
senza aumento delle violazioni di slew/bounds.

### U2 — Warm start imitativo unificato

Usare lo stesso actor e gli stessi campi dell'ex-novo. Tutte le feature causali
del governor e della FSM devono essere popolate con valori runtime reali anche
nel warm start; non devono comparire colonne fittizie che cambiano distribuzione
al passaggio di modalità.

Reward:

```text
imitation = ON
exnovo    = OFF
safety    = ON
```

### U3 — DAgger closed-loop

Raccogliere nominali, recovery e start perturbati usando lo student; interrogare
il teacher inverse-governor sugli stati realmente visitati; aggiornare lo stesso
actor. Validare closed-loop dopo ogni round.

### U4 — Selezione del best imitativo

La selezione non deve basarsi solo sulla MSE offline. Deve includere:

- `500/500` step;
- eventi HS/TO e cicli completi;
- penetrazione sotto hard guard;
- qualità served e cinematica al toe-off;
- stabilità della FSM v3;
- confronto multi-start e stocastico.

### U5 — Cambio completo della reward

Caricare solo l'actor del best U4; creare critic e optimizer nuovi.

```text
imitation = OFF
exnovo    = ON
safety    = ON
actor     = frozen
```

Eseguire il critic-only warm-up fino a una explained variance stabile, non per
un numero prefissato di iterazioni se i gate non sono ancora soddisfatti.

### U6 — Calibrazione dell'esplorazione

Misurare l'effetto reale di sigma sul delta delle azioni e sulla served dopo il
governor. Lo sweep deve precedere il training lungo; `sigma=0.005` è una
baseline storica, non un valore da assumere corretto senza verifica.

### U7 — PPO puramente ex-novo

Sbloccare actor e mantenere `imitation=0`. I termini imitativi restano soltanto
in shadow logging. Se vengono usati ancora come reward, la fase non è più
puramente ex-novo.

### U8 — Validazione finale

Valutare separatamente:

- raw actor;
- output dopo slew;
- served C2;
- cinematica fisica;
- corridor retrospettivo event-aligned;
- corridor causale usato nel training;
- return e decomposizione completa della reward.

## 8. Gate per dichiarare riuscito l'actor predittivo

Un candidato non è promosso soltanto perché completa l'episodio. Deve:

1. mantenere lo schema actor privo di feature privilegiate;
2. conservare la salute FSM: `phase_timeout_stance=0`,
   `morphology_causal_contract_failure=0`, `resync_count=0` e
   `hs_cancelled_count=0` nel nominale;
3. ridurre l'errore actor-to-served attribuibile a ritardo;
4. portare l'ankle served verso plantarflessione nella finestra del TO;
5. non ottenere il risultato spostando il gate diagnostico nel tempo;
6. rispettare `20/25/28 mm` come soft/diagnostic/hard;
7. funzionare su più start e con rollout stocastici;
8. non aumentare saturazioni, jerk o interventi del limiter oltre soglie
   preregistrate;
9. mostrare un cambiamento reale dell'actor durante l'ex-novo;
10. mantenere il tracking served-to-actual entro la qualità già osservata.

## 9. Rischi principali e mitigazioni

| Rischio | Mitigazione |
|---|---|
| Actor impara feature che spariscono nel deployment | Separare rigorosamente actor, critic e reward |
| Anticipo fisso valido solo a una cadenza | Teacher condizionato sullo stato e sweep multi-cadenza |
| Reward morphology arriva dopo l'errore | Credito alla decisione originaria e fascia soft interna |
| Peso ankle domina e degrada il knee | Pesi separati e gate per giunto |
| Critic eredita il vecchio obiettivo | Critic e optimizer freschi al cambio modalità |
| Sigma troppo piccola produce solo fine tuning | Sweep dell'esplorazione effettiva dopo il governor |
| Rete più grande destabilizza il closed loop | Conservare prima 256x256 e modificare target/DAgger |
| Corridor unico impone una cadenza sbagliata | Corridoio condizionato su periodo/stance/swing |
| Miglioramento del return nasconde toe-off errato | Selezione checkpoint con metriche ankle e di fase |

## 10. File introdotti o modificati

### Codice

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - pesi morphology separati;
  - fallback retrocompatibile;
  - logging per giunto;
  - guardie `NaN` per giunto;
  - attivazione del contratto causale se almeno un peso è positivo.

### Configurazione

- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_joint_split_equal_1p05_2026-08-31.json`

### Test e audit

- `validation/test_morphology_joint_weights.py`
- `validation/audit_morphology_split_critic_rewarm.py`
- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/morphology_joint_split_critic_rewarm_audit.json`

### Artefatto critic rewarm valido

- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/checkpoint_last`
- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/rl_module_last`

## 11. Test eseguiti

La nuova composizione ha superato:

- round-trip della configurazione opzionale;
- equivalenza byte-level della reward con pesi split uguali;
- composizione matematica con pesi asimmetrici;
- disattivazione sicura di un giunto con loss `NaN`;
- fail-closed per loss non finita su un giunto attivo;
- fail-closed per pesi negativi o non finiti;
- test readiness morphology esistenti;
- test event-anchored, retrospective e recorded replay pertinenti;
- audit del checkpoint critic-only con actor immutato.

Due suite legacy complete hanno incontrato assertion su valori di configurazione
attiva già diversi dalle aspettative storiche (`morphology_weight` e timeout
swing). I test specifici della nuova logica passano; quelle due assertion non
indicano una regressione prodotta dalla separazione knee/ankle, ma drift di
configurazioni preesistente da riallineare in una manutenzione separata.

## 12. Decisioni ancora da approvare

Prima di avviare la costruzione della nuova rete restano da fissare:

1. metodo di generazione delle label inverse-governor;
2. griglia dello sweep temporale e criteri di selezione;
3. definizione della loss imitativa in raw space e served-predicted space;
4. fascia soft interna al corridor e assegnazione causale del credito;
5. pesi iniziali separati knee/ankle per il primo pilot;
6. gate quantitativi di toe-off e saturazione;
7. sweep di sigma e criterio per dichiarare esplorazione sufficiente;
8. eventuale corridor cadence-conditioned;
9. numero di round DAgger e composizione nominal/recovery;
10. protocollo held-out e multi-start.

La generalizzazione oltre AB06 resta un TODO successivo: dovrà usare la
conversazione progettuale precedente e il dataset EPIC per verificare che
normalizzazione, phase detector, teacher e corridor non siano model-dependent.
Non va mescolata alla prima milestone, che resta recuperare la qualità AB06.

## 13. Conclusione operativa

La nuova rete non deve essere progettata come due actor differenti né come un
actor che osserva dati imitativi privilegiati. Deve essere un unico actor
deployable, addestrato prima da un teacher predittivo compatibile con il
governor e poi con una reward ex-novo completamente sostitutiva.

La separazione morphology knee/ankle e il critic-only rewarm sono già pronti.
Il prossimo vero sviluppo architetturale è il generatore di label
inverse-governor e la sua validazione frozen-policy. Solo dopo quel gate ha
senso scegliere un peso ankle maggiore ed eseguire un nuovo training.

## 14. Riferimenti storici

- [Transizione ex-novo da warm start imitativo](2026-06-25_transizione_ex_novo_warmstart_imitativo.md)
- [Migrazione imitation/ex-novo e transplant](2026-06-25_piano_migrazione_imitation_exnovo_cfg_transplant.md)
- [Prima integrazione del morphology corridor](2026-06-29_integrazione_corridor_morphology_reward_exnovo.md)
- [Slew limiter nella reward ex-novo](2026-07-07_target_slew_limiter_reward_exnovo.md)
- [Riadattamento imitativo, lookahead e correzione del limiter](2026-07-11_riadattamento_imitativo_target_domain.md)
- [Guida completa V26/J20 training-ready](2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md)
- [Rollout base V26/J20 e plot corridor](2026-08-28_rollout_best_exnovo_v26c_j20_plot_canonici_e_corridor_morfologico.md)
- [Audit corridor, prescribed e latenza frozen-policy](2026-08-30_audit_corridor_morfologico_vs_prescribed_e_v26.md)
- [Progettazione storica del reference model C2](2026-06-13_progettazione_training_imitativo_v4_frequenze_reference_model_c2.md)
