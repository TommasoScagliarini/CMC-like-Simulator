# Validazione della rete unificata V1 — 6 settembre 2026

Instruction check token: `CMC_AGENT_OK_2026`.

Lo smoke ha completato la sequenza **pretraining imitativo → critic-only → PPO
ex-novo** con checkpoint separati e caricamento diretto dell'actor. Questo
verifica il funzionamento della pipeline; una sola iterazione imitativa non
attesta la qualità del cammino, la convergenza del training o la compensazione
del ritardo, che resta fuori dalla V1.

**Nota del 14 settembre 2026:** il peso di penetrazione dei preset imitativi
è stato portato da 0.05 a 1.0 su richiesta dell'utente, mentre l'ex-novo resta
a 0.05. Le prove live descritte qui risalgono al 6 settembre e usavano 0.05
anche nell'imitazione. I risultati e gli snapshot originali sono conservati;
nessuna nuova esecuzione dinamica con peso 1.0 è attestata da questo documento.
Per questa modifica la suite aggiornata ha superato **82 test in 5.82 s** su
macOS; sono stati verificati i quattro preset, le soglie 20/28 mm, l'hash della
configurazione V26 sorgente dell'override e l'assenza di modifiche alla baseline.

## Piattaforma e isolamento

Esecuzioni dinamiche su **macOS arm64**, Darwin 25.5.0, Python **3.10.20**, Ray
**2.55.1**, PyTorch **2.10.0**, nell'ambiente `envCMC-rllib` con OpenSim e plugin
SEA della baseline. Nessuna esecuzione dinamica Windows è stata effettuata:
path portabili e shim Windows sono predisposti, ma il risultato live qui
riportato riguarda macOS.

Codice e risultati della nuova pipeline vivono in `unified_MLP/`. Il controllo
Git finale non mostra modifiche ai file esterni a questa directory; baseline,
plugin C++, runtime condiviso e registro storico restano intatti. I run di
questa verifica non sovrascrivono checkpoint della baseline. Le sorgenti dei
preset e i loro SHA256 sono conservati in
[configs/provenance.json](configs/provenance.json).

## Contratto dell'ambiente e target

L'[audit dinamico dell'ambiente](runs/validation/environment_contract.json)
ha verificato tutti e tre gli offset `1.756870983805102`, `1.956870983805102`,
`2.156870983805102` s:

- **35 feature actor e 84 osservazioni totali** in entrambe le modalità.
- Osservazioni esattamente uguali al reset e dopo lo stesso primo comando.
- Le due colonne del clock prescribed sono zero.
- Target imitativi esattamente uguali alle coordinate prescribed sinistre
  interpolate al tempo corrente.

I target imitativi sono `pros_knee_angle` e `pros_ankle_angle` da `base_kin`;
il prescribed controlaterale non è la sorgente della loss. Lo stack GRF,
heel/toe, FSM V26/v3, governor e reset rimane quello dei preset comuni.
La verifica dinamica a un passo completa i test unitari di coordinate, unità,
indipendenza dai target destri e isolamento matematico dell'obiettivo inattivo.

## Smoke completo: 576 transizioni

Ogni iterazione ha raccolto **192 transizioni reali**, con **3 EnvRunner**, un
batch di **64 transizioni per ciascuno start**, minibatch **64** e **un'epoca**.
I controlli di bilanciamento della raccolta e di conteggio del learner sono
passati in tutte e tre le iterazioni. Il padding introdotto per GAE viene
rimosso prima del training: non viene conteggiato come transizione reale.

| Run e iterazione | Fase | Actor | Critic | Risultato |
|---|---|---|---|---|
| [Imitazione r3, iter. 1](runs/training/smoke_v1_imitation_r3/summary.json) | `imitation` | Medie aggiornate | Aggiornato | Completata, inizializzazione nuova |
| [Ex-novo r1, iter. 1](runs/training/smoke_v1_ex_novo_r1/summary.json) | `critic_warmup` | Esattamente invariato | Aggiornato | Completata |
| [Ex-novo r1, iter. 2](runs/training/smoke_v1_ex_novo_r1/summary.json) | `ex_novo` | Medie aggiornate | Aggiornato | Completata |

L'[audit di inizializzazione](runs/training/smoke_v1_ex_novo_r1/actor_initialization_audit.json)
conferma la copia esatta dell'actor preaddestrato, la mancata importazione di
critic e optimizer e l'identità dei pesi dopo la sincronizzazione. La successiva
configurazione di sigma **[0.010, 0.010]**, con logstd congelata, conserva
esattamente la rete delle medie. Il controllo comprende learner e quattro
moduli EnvRunner, cioè tre worker di raccolta più il runner locale.

Gli audit `unified_phase_audit` registrano digest separati di actor, rete delle
medie e critic prima e dopo ogni iterazione. Nel warm-up il digest dell'actor
resta identico e il digest del critic cambia. Nell'iterazione PPO ex-novo cambia
anche la rete delle medie; il massimo KL medio per minibatch è
**0.0004950799**, entro il guard **0.01**, con zero KL non finiti.

Durante la raccolta ex-novo sono intervenute terminazioni per
`grf_penetration`: tre episodi nell'iterazione critic-only e un ulteriore episodio
nell'iterazione PPO. Le protezioni hanno funzionato; i restart degli episodi
non hanno alterato il numero di transizioni raccolte per start. Questo è un
limite della policy appena inizializzata, non una prova di cammino stabile.

I risultati, gli snapshot risolti, TensorBoard e i checkpoint per iterazione
sono nelle cartelle dei run collegati. Il tempo di training registrato è circa
**75 s** per lo smoke imitativo e **138 s** per le due iterazioni ex-novo;
questi tempi descrivono la macchina usata, non costituiscono un benchmark.

## Reload e pretraining con dieci epoche

Il [resume in un processo nuovo](runs/training/smoke_v1_resume_check/summary.json)
ha ripristinato il checkpoint ex-novo all'iterazione **2**, con
`critic_warmup_completed=true`, `last_completed_phase=ex_novo` e prossima
iterazione **3**. Il totale richiesto era già raggiunto: **zero nuovi
aggiornamenti**, nessuna seconda inizializzazione dell'actor e nessuna ripetizione
del warm-up. L'[audit di resume](runs/training/smoke_v1_resume_check/resume_audit.json)
conserva i digest di actor, medie e critic, risultati uguali a quelli dopo
l'ultima iterazione ex-novo. Questa prova verifica il ripristino senza eseguire
un altro aggiornamento PPO.

Una [verifica imitativa aggiuntiva](runs/training/smoke_v1_multiepoch/summary.json)
ha completato un'iterazione con **10 epoche**, mantenendo batch 192 e tre worker:
**192 transizioni reali**, **1920 righe elaborate dal learner**, **30 minibatch**,
64 transizioni raccolte per start e controlli di bilanciamento passati. Actor e
critic si aggiornano, con zero KL non finiti. Il massimo KL medio per minibatch
è **0.0293760**; il guard stretto del pilot ex-novo è intenzionalmente assente
nel pretraining. Questa prova copre il percorso multi-epoca senza avviare un
training lungo o modificare il batch 4608 dei preset completi.

## Rollout diagnostici ai tre start

I tre rollout hanno ricaricato il modulo ex-novo in processi nuovi, con azioni
deterministiche, registrazione della traccia policy e output `.sto` attivi.
Tutti i summary riportano `ok=true`, schema 35/84 e nessun clipping delle azioni.
La tabella riporta l'esito fisico effettivo, distinto dal completamento del
comando di valutazione.

| Start | Step / durata simulata | Termine | Penetrazione massima | HS / TO / cicli validi |
|---|---:|---|---:|---:|
| [Early, 1.756871 s](runs/rollout/smoke_v1_early/rollout_summary.json) | 500 / 5.00 s | `episode_time_limit`, troncato al limite previsto | 15.469 mm | 0 / 0 / 0 |
| [Center, 1.956871 s](runs/rollout/smoke_v1_center/rollout_summary.json) | 294 / 2.94 s | `phase_timeout:swing`, terminato dalla protezione | 21.484 mm | 0 / 1 / 0 |
| [Late, 2.156871 s](runs/rollout/smoke_v1_late/rollout_summary.json) | 295 / 2.95 s | `phase_timeout:swing`, terminato dalla protezione | 27.943 mm | 0 / 1 / 0 |

Le tracce [early](runs/rollout/smoke_v1_early/rollout_policy_trace.json),
[center](runs/rollout/smoke_v1_center/rollout_policy_trace.json) e
[late](runs/rollout/smoke_v1_late/rollout_policy_trace.json) registrano azione
raw/applicata, osservazioni actor, target imitativi, stato protesico,
diagnostica della reference servita e del governor, heel/toe, eventi e FSM,
termini di reward e contabilità morphology. I rispettivi `sim_outputs/`
contengono stati, riferimenti, SEA, GRF e diagnostica biologica del simulatore.

**Nessun rollout ha completato un ciclo valido.** Il raggiungimento dei cinque
secondi nello start early, con ritorno totale zero e senza eventi validi,
non prova la qualità della policy. Center e late hanno prodotto un toe-off e
poi raggiunto il timeout di swing. Il test dimostra caricamento, integrazione,
telemetria e applicazione delle protezioni; rimane necessario un vero training
per valutare la qualità del cammino.

## Tentativi iniziali e correzioni

Due tentativi precedenti sono conservati per rendere verificabile la diagnosi:

- [Imitazione r1](runs/training/smoke_v1_imitation_r1/summary.json): **zero
  iterazioni**, errore `PermissionError` nel `sysctl()` usato da `psutil` durante
  `ray.init()`, causato dalle restrizioni della sandbox. Il successivo avvio
  autorizzato ha consentito a Ray di inizializzarsi.
- [Imitazione r2](runs/training/smoke_v1_imitation_r2/summary.json): **zero
  iterazioni**, `TypeError` durante la sincronizzazione della fase perché il
  runner locale esponeva direttamente un `RLModule`, non un contenitore
  indicizzabile per `default_policy`. La gestione dei due formati è stata
  corretta nella sola pipeline unificata; r3 ha completato il percorso reale.

## Test automatici

**82 test superati in 3.95 s** nella verifica conclusiva su macOS, usando
il comando documentato senza impostare `PYTHONPATH`. La suite copre contratto osservazioni/azioni, target sinistro, reward esclusive,
trasferimento actor, warm-up e stato di fase, preset e provenienza, compatibilità
del runtime e isolamento degli output. Sono inoltre verificati i parser CLI e
il rifiuto degli override fisici incompatibili, compreso il tentativo di
aggirarli con `--no-auto-config` e un YAML esplicito.

Comando per ripetere l'intera suite, dalla root del repository:

```console
python -m pytest "Trajectory Generator/unified_MLP/tests" -q
```
