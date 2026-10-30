# Training 39D da zero sul riferimento destro V26: implementazione, smoke e avvio

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Data: 23 settembre 2026. Richiesta: addestrare la 39D sul riferimento destro,
eseguendo smoke test prima del training completo fino a iterazione 40.

**Smoke conclusi con esito positivo. Training completo avviato alle 12:50:29
CEST**, da inizializzazione casuale, con monitor persistente e watchdog del
trainer. Alla verifica di avvio la prima iterazione è in corso; questo report
non contiene ancora risultati di apprendimento del run completo.

## Problema e soluzione

Il trasferimento V26 precedente era riservato alla valutazione. Il percorso
ordinario 39D accettava soltanto il riferimento sinistro. È stata introdotta
la selezione esplicita `target_reference: right_v26`, propagata da YAML/CLI ad
ambiente, trainer, rollout e contratto dei checkpoint. Un checkpoint RIGHT
viene rifiutato se interpretato come LEFT, e viceversa.

Il nuovo esperimento usa pesi casuali. Non carica pesi V26 né prosegue dai
pesi dello smoke. Actor e critic iniziali dello smoke coincidono con quelli
del precedente training 39D LEFT con seed 123; l'actor iniziale del run
completo coincide a sua volta esattamente con quello dello smoke.

La ricetta deriva dalla configurazione LEFT riallineata. Si cambia il target
prescribed, il clock associato e la normalizzazione derivata dal target.
L'ordine degli ingressi resta quello della 39D. FSM v3 e gestione attuale dei
fallimenti restano attive: non si ripristinano i bug storici.

## Configurazione effettiva

| Elemento | Impostazione |
|---|---|
| Actor / osservazione completa | 39 / 88; due layer 256 tanh |
| Inizializzazione | Casuale, seed 123 |
| Target | Template periodico RIGHT storico V26 |
| Shift target ginocchio / caviglia | 0,465 / 0,452 cicli |
| Clock actor | Sin/cos della fase RIGHT non sfasata |
| Disponibilità prescribed e clock | 100%, curriculum vuoto |
| Divisore velocità prescribed | 1 |
| Start / durata episodio | 12,99 s / 5 s, senza start differenziati |
| FSM | v3 |
| Slew limiter | Disattivato |
| Governor | Butterworth3 jerk-limited, 4 Hz, limiti concordati |
| Reward | Ricetta imitativa V26, verificata numericamente |
| PPO | Batch 4096; minibatch 512; 10 epoche; LR 1e-4 |
| Parallelismo | 13 worker, 14 CPU Ray; un thread numerico per processo |
| Budget | 40 iterazioni, nominalmente 163.840 transizioni |
| Checkpoint | Completi e moduli di inferenza conservati a ogni iterazione |

[Configurazione completa — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-9f6a9111962438c5>).

## Verifiche prima dell'avvio

1. **107 test automatici superati.** Inclusi target e clock RIGHT, maschera
   congiunta dei sei canali, contratti, parsing, incompatibilità RIGHT/LEFT e
   regressione del percorso sinistro. Il contratto LEFT archiviato viene
   ricostruito identico.
2. **Preflight OpenSim contro V26.** Su 500 campioni storici, errore massimo
   q/qdot del target zero; escursioni di normalizzazione coincidenti. Scarto
   del clock prima del cast float32 2,98×10⁻⁸. Stessa configurazione reward;
   ritorno storico ricostruito entro 3,98×10⁻¹³.
3. **Dinamica e maschera.** Riprodotti 20 passi con azioni storiche registrate,
   prima con canali visibili e poi nascosti. Osservazioni actor coincidenti
   con lo storico dopo riordino; reward, loss e stati fisici identici fra le
   due condizioni. Il riferimento interno della reward resta disponibile.
4. **Smoke di training reale.** Due iterazioni, poi ripresa dal checkpoint
   completo della seconda fino alla terza: 576 transizioni complessive,
   batch 192, minibatch 64, un'epoca, tre worker. Aggiornati actor, critic e
   logstd, con loss finite e visibilità 100% nei batch.
5. **Ripresa completa verificata.** I 12 stati Adam ripristinati coincidono
   esattamente con il checkpoint e sono collegati ai parametri vivi. Il
   contatore passa da 8 a 12 aggiornamenti; stato PPO e coefficiente KL
   conservati. Nessuna reinizializzazione di actor o critic alla ripresa.
6. **Ricaricamento per inferenza.** Rollout di 20 passi dall'export della
   seconda iterazione, senza specificare manualmente target/configurazione.
   Il riferimento RIGHT viene recuperato automaticamente; target coincidenti
   con quelli storici e tutti i canali visibili.

Gli smoke verificano implementazione, aggiornamenti e serializzazione. Il
loro budget ridotto non è una prova di convergenza, robustezza o buona
camminata e non viene incluso nelle 40 iterazioni del run completo.

## Stato e supervisione

Run:
`Trajectory Generator/unified_MLP_39/runs/training/MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23`.

- Monitor persistente PID **31255**, supervisore training PID **31256**.
- Verificati 13 `SingleAgentEnvRunner` attivi e inizializzazione fresca.
- Il monitor registra completamento o errore e non avvia ulteriori training.
- Su macOS il blocco dell'inattività termina con il processo di training.
- Registrati 48 hash e copie degli input/sorgenti; 14 input storici esterni
  alla variante risultano invariati, inclusi asset/checkpoint verificati.

[Stato aggiornato — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-6df43ed8be9c8cbd>) ·
[Log training — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-671e67c426343bc5>) ·
[Protocollo — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-73fd8e950029c7ea>).

## File modificati e artefatti

Nel solo perimetro `Trajectory Generator/unified_MLP_39/`:

- `phase_target.py`, `env.py`, `env_factory.py`: template RIGHT e selezione
  esplicita, mantenendo invariato il default LEFT.
- `target_availability.py`, `actor_transition.py`: contratto dei sei canali
  e validazione della provenienza del riferimento.
- `training_config.py`, `runtime_contract.py`, `train_ppo_mlp.py`,
  `rollout_eval.py`: propagazione e conservazione della scelta nel training,
  nel resume e nell'inferenza.
- Due configurazioni dedicate e due nuovi file di test; README aggiornato.
- `validation/right_reference_training_2026-09-23/`: preflight, audit dello
  smoke, launcher, log e ricevute delle verifiche.
- `runs/plans/right_reference_training_2026-09-23/`: protocollo, snapshot,
  manifest, stato persistente e verifica dell'inizializzazione del run.

Nessuna modifica alla versione FSM, alla formula reward o ai controllori SEA.
Implementazione Python portabile; esecuzione live verificata su macOS arm64,
non ancora su Windows.

## TODO

- Analizzare il run a 40 iterazioni a parità di transizioni con il precedente
  LEFT e con le finestre V26 disponibili; includere completamento, rumore,
  cinematica e coppie, distinguendo training ed evaluation.
- Conservare la distinzione fra questo test da zero e l'eventuale futuro
  oscuramento della V26 trasferita. Nessun oscuramento o ex novo è avviato.
- Un singolo seed non dimostra equivalenza statistica; eventuali repliche
  dipenderanno dall'esito del confronto.
