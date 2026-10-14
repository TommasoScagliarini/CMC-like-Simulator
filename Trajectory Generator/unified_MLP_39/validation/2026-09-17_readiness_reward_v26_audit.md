# Verifica 39D e confronto con il V26 imitativo del 20 agosto

Instruction check token: CMC_AGENT_OK_2026

## Richiesta e conclusione

L'utente ha confermato il riferimento del 20 agosto e la sequenza: prima
verificare risultati comparabili con i quattro target sempre visibili, poi
valutare l'oscuramento progressivo.

**La 39D attuale non è ancora pronta per il training e la sua reward non è
formalmente equivalente alla reward storica V26.** Ambiente e rete funzionano
nella prova diretta, ma un errore nell'accesso agli ambienti vettoriali blocca
l'applicazione della maschera prima del primo aggiornamento PPO. Il confronto
scientifico richiede inoltre un contratto comune di supervisione.

Questa verifica non modifica il runtime, i preset, i checkpoint o il plugin.
Non è stato avviato alcun training. I file prodotti sono evidenze dell'audit.

## Riferimento storico

- Run: `Trajectory Generator/runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/`.
- Fonti: `training_cfg.resolved.yaml`, `summary.json`, `checkpoint_best_meta.json`,
  `actor_feature_manifest.json` nella directory del run.
- Best: iterazione 87; training completato a 100.
- Report: `reports/user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md`
  e `reports/user/2026-08-20_verdetto_fresh_run_june_equivalent_nativo_v26.md`.

I report e i parametri congelati sono coerenti con le formule ricostruite.
Il confronto con il commit pre-run `68a62cd8` non mostra modifiche alle formule
dei target e delle loss imitative; non si rivendica una riproduzione bit per bit
dell'intero runtime storico, di cui manca uno snapshot sorgente completo.

## Readiness tecnica

1. Nell'ambiente `envCMC-rllib`, il codice 39D effettivamente presente nel
   workspace costruisce ambiente e rete, esegue reset e un passo OpenSim da
   10 ms: actor 39, osservazione completa 88, valori e uscita policy finiti,
   quattro target visibili, due slot clock a zero, nessuna terminazione.
   È una prova di esecuzione minima, non una verifica del cammino.
2. Le suite esistenti `test_feasibility39_bundle.py` e
   `test_absolute_actions.py`: **24 test superati in 4.96 s**. La prima suite
   materializza una variante temporanea: il PASS non certifica il coordinamento
   dei worker reali della variante installata.
3. Riprodotto il bug corrente con i veri wrapper Gymnasium installati:
   `DictInfoToList(SyncVectorEnv(...))` non espone direttamente `envs`.
   `_runner_environments()` in `target_availability.py:171` restituisce il
   vettore al posto dei singoli ambienti. `_runner_apply()` produce
   `AttributeError: 'SyncVectorEnv' object has no attribute '_target_availability'`.
   La correzione deve scoprire gli ambienti interni dopo aver rimosso il wrapper
   vettoriale e conservare i controlli sulla copertura dei worker.
4. Il vecchio smoke del 15 settembre conserva `iterations_completed=0` e
   l'errore `target availability was not applied to any sampling environment`.
   Il suo modulo iniziale non è un checkpoint addestrato. Update PPO,
   salvataggio e ripresa 39D non risultano validati da quel run.
5. I preset non specificano `target_availability`; il default CLI è **p=0**.
   Il primo confronto richiesto necessita di **p=1 e curriculum vuoto**.
6. Windows live resta da verificare; nel workspace mancano le DLL SEA `_ff`
   e OnlineGRF richieste. Sono presenti i corrispondenti plugin macOS.

La prova diretta è stata eseguita senza costruire un algoritmo Ray né chiamare
`train()`. Il primo tentativo del nostro harness aveva omesso il caricamento
lazy dello stack del trainer; è stato corretto chiamando il loader previsto.
Questo errore del harness non è classificato come difetto dell'entry point.

## Confronto formale della reward

La parte positiva ha la stessa forma:

```text
r = clip(0.8 / (1 + 8 * L_served)
       + 0.2 / (1 + 4 * L_actual) - P, 0, 1) - Q
```

`L_served` confronta il riferimento effettivamente servito con il target;
`L_actual` confronta la cinematica fisica con il target. Le loss usano errori
quadratici normalizzati e limitati a 25 per componente: pesi posizione
ginocchio/caviglia 1/2, pesi velocità 0.02/0.04. I campioni sono valutati al
tempo post-step. Formula comune non significa uguaglianza delle funzioni:
cambiano target, scale, P e Q.

| Elemento | V26 del 20 agosto | 39D corrente |
|---|---|---|
| Target | Template periodico medio della gamba sana destra, shift di fase knee 0.465 e ankle 0.452 | Prescribed sinistro al tempo corrente |
| Scale delle loss | Escursioni del target periodico | Escursioni del prescribed sinistro |
| Peso action clipping | 0 | 0.25 |
| Peso eventi invalidi | 0 | 0.1 |
| Peso validità contatto | 0 | 0.1 |
| Peso timeout soft | 0 | 0.5 |
| Peso range articolare fisico | 0 | 2 |
| Peso ankle moment flip | 0.1 | 0.05 |
| Limite inferiore ginocchio reference OOB | -1.4 rad | -1.25 rad |

Action clipping, invalid event e contact validity entrano prima del clipping
della reward. Timeout, range fisico, flip e OOB entrano dopo. Il timeout soft
swing è inoltre 0.90 s nel V26 e 0.75 s nella 39D.

Restano identici i pesi effort, smoothness, saturation, command rate, le
penalità SEA attive, safety=2, penetrazione=1 e il peso OOB=2. I termini
morphology, swing unloading, overload, slip, reserve/pelvis e clawback sono
disattivati nel ramo imitativo unificato: la loro presenza nel YAML non è una
differenza attiva dell'obiettivo. Le guardie morphology sono limitate a ex-novo.

Controesempi algebrici eseguiti caricando le due configurazioni e chiamando
`compute_reward` sugli stessi input sintetici, con tutte le altre loss nulle:

| Unica loss pari a 1 | V26 | 39D |
|---|---:|---:|
| Action clipping | 1.00 | 0.75 |
| Invalid event | 1.00 | 0.90 |
| Contact validity | 1.00 | 0.90 |
| Timeout | 1.00 | 0.50 |
| Range articolare | 1.00 | -1.00 |
| Ankle flip | 0.90 | 0.95 |

Questi casi confutano l'uguaglianza delle funzioni; non rappresentano rollout
osservati e non stimano la frequenza delle penalità durante il training.

Fonti nel codice: `unified_MLP_39/env_v1.py:68`,
`unified_MLP_39/reward_function.py:934` e `:1075`,
`osim_trj_cmc_like.py:941`, `:2348`, `:4259`.

## Differenza numerica fra i target

Nella prova diretta abbiamo ricostruito il teacher periodico con gli asset
correnti, heel strike destri e parametri storici, campionando la stessa griglia
usata per le scale. Questo confronta i due algoritmi di supervisione sugli
asset locali; non è un replay del checkpoint storico.

| Quantità | Ginocchio | Caviglia |
|---|---:|---:|
| RMSE fra i due target di posizione | 7.57° | 7.28° |
| RMSE fra i due target di velocità | 1.066 rad/s | 0.793 rad/s |
| Range posizione teacher periodico | 0.9326 rad | 0.3548 rad |
| Range posizione prescribed sinistro | 0.9278 rad | 0.5481 rad |
| Range velocità teacher periodico | 8.1111 rad/s | 3.3759 rad/s |
| Range velocità prescribed sinistro | 7.2409 rad/s | 5.2641 rad/s |

I dettagli della prova sono in `2026-09-17_readiness_direct_probe.json`.

## Altre differenze da controllare nell'esperimento

- V26 aveva già 39 ingressi actor, con clock attivo e quattro target nei
  canali 2–5. La nuova 39D azzera il clock e aggiunge i target ai canali 35–38.
- FSM v2 nel V26, v3 nella variante: cambia la semantica degli eventi.
- Start V26 fisso a offset 1.0 s; unificata su tre start
  1.75687/1.95687/2.15687 s.
- Slew limiter disabilitato nel V26, limiti 2.5/2.0 rad/s nella variante.
- Batch V26 4096 non esatto, variante 4608 bilanciato esatto.
- Gamma=0.99, lambda=0.9, durata 5 s, step 10 ms e soglie di penetrazione
  20/28 mm sono uguali; questo non annulla le differenze precedenti.

## Sequenza sperimentale conseguente

Prima di un training lungo occorre correggere il callback e verificare un
piccolo aggiornamento PPO con worker reali, p=1, batch visibile al 100%,
checkpoint e ripresa. Il profilo sperimentale deve dichiarare p=1 senza
curriculum, lasciando invariati i preset e gli artefatti storici.

Per riprodurre l'obiettivo V26 occorre un profilo separato che ripristini
teacher, normalizzazioni e penalità storiche, con target osservati coerenti
con quelli premiati. Se si conserva invece il prescribed sinistro corrente,
serve una baseline riaddestrata sotto lo stesso nuovo obiettivo: il return
storico V26 non è direttamente confrontabile. La scelta non è stata
implementata durante questo audit.

La comparabilità dei risultati va misurata su un protocollo comune, con
tracking, durata, cicli/eventi e sicurezza, margini di equivalenza fissati
prima della selezione e repliche con seed di training indipendenti. Il
report storico del 20 agosto usa l'espressione equivalenza statistica ma
documenta un confronto entro la variabilità di un singolo seed: non basta
come test formale di equivalenza fra procedure di training.

Solo dopo il confronto a target visibili ha senso attribuire al curriculum
eventuali differenze rispetto a quella stessa 39D. La valutazione conclusiva
del curriculum deve usare target completamente oscurati.

## Chiarimento successivo dell'utente — 17 settembre

L'utente considera accettabili il target prescribed sinistro corrente e le
penalità aggiuntive di eventi, timeout e range fisico. La proposta precedente
di ripristinare il teacher storico non rappresenta quindi la direzione scelta:
il confronto richiesto riguarda la qualità ottenibile, accettando queste
differenze dell'obiettivo. È stata chiesta una spiegazione della normalizzazione,
senza richiederne una modifica.

Confermata disponibilità 100% dei quattro target per il primo esperimento.
Creato `configs/pretraining_visible_targets.yaml`, copia del preset corrente
con p=1 e curriculum vuoto. Tutte le altre impostazioni sono identiche,
comprese reward e normalizzazioni. Verificati caricamento dalla CLI esistente,
visibilità costante in imitazione e valutazione e chiusura forzata in ex-novo.
Il budget ereditato non costituisce un protocollo statistico preregistrato.
Nessun training avviato; il blocco del callback resta da correggere.
