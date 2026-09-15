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
quelle prove non attestano il comportamento dinamico con peso 1.0. Il primo
avvio del preset aggiornato è documentato nella sezione successiva.
Per questa modifica la suite aggiornata ha superato **82 test in 5.82 s** su
macOS; sono stati verificati i quattro preset, le soglie 20/28 mm, l'hash della
configurazione V26 sorgente dell'override e l'assenza di modifiche alla baseline.

## Avvio completo e isolamento delle terminazioni — 14 settembre 2026

Il primo lancio utente di `pretraining_v1_pen1_pilot_2026-09-14` è fallito
prima del training con `ModuleNotFoundError: No module named 'ray'`: usava
Python 3.13 dell'ambiente base. Il rilancio usa esplicitamente Python 3.10
di `envCMC-rllib`, con gli stessi argomenti CLI: `pretraining.yaml`, cinque
iterazioni e directory dedicata.

Una prima iterazione completa ha verificato 4608 transizioni, 1536 per start,
12 EnvRunner, 10 epoche/90 minibatch, loss finite e aggiornamento di actor e
critic. Ha però rilevato una terminazione `morphology_causal_contract_failure`
in imitazione: il gating della reward scalare non impediva al wrapper di
impostare il flag terminale usando i coefficienti morphology originali.
L'episodio interrotto poteva quindi influenzare ritorni e GAE. Il tentativo
è stato fermato durante l'iterazione 2 e conservato integralmente, incluso
il checkpoint dell'iterazione 1, in
[diagnostica del tentativo interrotto](runs/diagnostics/pretraining_v1_pen1_pilot_2026-09-14_morphology_termination_20260914T111005Z/interruption_diagnosis.json).
Non è un crash spontaneo e non costituisce il risultato del pilot corretto.

La correzione in `unified_MLP/reward_function.py` limita a `ex_novo` le due
terminazioni morphology: errore del contratto causale e violazione del
corridoio hard. Ledger, diagnostica e protezioni comuni rimangono attivi.
Sono state aggiunte 18 regressioni comportamentali del wrapper: prima della
correzione 11 fallivano; dopo, **100 test dell'intera suite superati in 4.20 s**.
I test verificano anche che ex-novo mantenga gli effetti morphology e che
imitazione conservi penetrazione, SEA, limiti, timeout e troncamenti.

Il pilot corretto è ripartito da pesi nuovi, seed 123, senza importare actor
o optimizer dal tentativo interrotto. Il
[manifest di lancio](runs/training/pretraining_v1_pen1_pilot_2026-09-14/launch_metadata.json)
registra interprete, comando, hash del file reward e archivio precedente.
Alle 13:22 CEST la prima iterazione corretta è completata e la seconda è in
corso; il processo viene lasciato attivo verso le cinque iterazioni richieste.
L'[audit di avvio](runs/training/pretraining_v1_pen1_pilot_2026-09-14/startup_verification.json)
verifica 4608 transizioni reali, 1536 per start, 90 minibatch, loss finite,
actor e critic aggiornati, checkpoint e milestone dell'iterazione 1 salvati.
L'export actor è stato ricaricato in un processo nuovo con digest identici,
schema valido e output di inference finiti. La suite e questo avvio sono
verifiche su macOS; il pilot non è ancora concluso al momento dell'handoff.

Nel batch corretto restano 18 terminazioni `grf_penetration`, una
`grf_penetration_hard` e sei timeout swing, senza terminazioni morphology.
La diagnostica morphology segnala ancora una riga affetta: resta registrata
senza sostituire la causa fisica di fine episodio. Il confronto ripetuto
con lo stesso seed trova identico anche il primo aggiornamento actor: in
questo batch la guardia precedente aveva sovrascritto l'etichetta di una
terminazione fisica già presente. Non è quindi dimostrata una variazione
dei ritorni di quel batch; i test di regressione coprono anche il caso in
cui la morphology sarebbe stata l'unica causa di terminazione.

Questa prova certifica l'avvio e il progresso tecnico, non convergenza o
qualità del cammino. La causa interna della segnalazione del ledger non è
identificabile dai contatori aggregati e resta da valutare per l'ex-novo.

## Pilot concluso e confronto checkpoint 3/5 — 14 settembre 2026

Il [pilot corretto](runs/training/pretraining_v1_pen1_pilot_2026-09-14/summary.json)
ha terminato **5/5 iterazioni**, 23 040 transizioni reali, alle 13:57:56 CEST,
in circa 46 minuti, senza crash, restart o iterazioni saltate. Il checkpoint
best è l'iterazione 3 secondo il ritorno aggregato del training; l'ultimo è
l'iterazione 5. Tale classifica non equivale a una valutazione deterministica
bilanciata per start: la metrica RLlib dipende dagli episodi completati e
dalla sua finestra di aggregazione.

Sono stati eseguiti **sei rollout deterministici**, uno per checkpoint/start,
con seed 123, limite 500 step/5 s, trace e output `.sto`. Tutti terminano con
`ok=true` e processo child con exit code 0. I due checkpoint hanno configurazioni
risolte identiche; gli hash degli export sono registrati nel
[manifest](runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/launch_manifest.json).
Fisica, detector, filtro, slew limiter e reward imitativa con penetrazione 1.0
provengono dagli snapshot originali. Non è stato eseguito ulteriore training.

| Start | Durata CP3 / CP5 [s] | RMSE ginocchio CP3 → CP5 [°] | RMSE caviglia CP3 → CP5 [°] | Penetrazione max CP3 → CP5 [mm] |
|---|---:|---:|---:|---:|
| Early | 5.00 / 5.00 | 26.33 → 24.97 | 12.96 → 12.27 | 20.26 → 23.98 |
| Center | 2.80 / 2.80 | 25.65 → 25.27 | 13.03 → 11.30 | 20.36 → 24.86 |
| Late | 2.83 / 2.83 | 24.61 → 23.63 | 12.62 → 10.75 | 18.71 → 18.77 |

RMSE actual–prescribed sinistro, in gradi, con target e stato allineati allo
stesso timestamp post-step e ponderazione per durata del passo. Ogni coppia
ha durata uguale: la finestra comune coincide con i suoi episodi completi.
Early raggiunge il limite di durata; center e late terminano per timeout swing.
**Zero heel strike e zero cicli validi in tutti i casi**, un toe-off in ciascun
center/late. Il periodo online osservato dall'actor resta zero per tutta la
traccia; i canali online sin/cos restano costanti. Non è quindi entrato in uso
un clock basato su un periodo misurato.

Il checkpoint 5 migliora moderatamente il tracking angolare in ogni start,
ma aumenta la penetrazione e non migliora durata o cicli. Il tracking delle
velocità non migliora uniformemente. La RMSE actual–served è invece
**0.25–0.29° al ginocchio e 0.16–0.40° alla caviglia**: il movimento segue
il riferimento dopo filtro, mentre è quel riferimento a restare lontano dal
prescribed. Nessun clipping delle azioni o saturazione interna `tau_input`.

Questi risultati non mostrano un deterioramento uniforme del checkpoint 5
rispetto al 3, né un cammino valido per uno dei due. Confermano la mancata
attivazione del periodo online in questi rollout, senza dimostrare che
l'assenza dei target nell'actor sia la causa o che altro training non possa
aiutare. Non costituiscono una verifica della parità con la vecchia baseline.

Artefatti: [tabelle e protocollo completo](runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/comparison.md),
[metriche JSON](runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/comparison.json),
[grafici PNG](runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/tracking_angles.png)
e [PDF](runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/tracking_angles.pdf).
Le curve actual e served sono quasi sovrapposte; le curve si fermano alla
terminazione, senza estrapolare il movimento mancante.

L'analizzatore offline ha superato **6 nuovi test**, inclusi allineamento,
conversione delle unità, pesi temporali, finestre di durata differente,
aggregazione dei campioni SEA e rifiuto di dati misurati non finiti. I grafici
sono stati generati e controllati visivamente. Comandi portabili di analisi
documentati nel README; esecuzione live effettuata solo su macOS arm64.

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
