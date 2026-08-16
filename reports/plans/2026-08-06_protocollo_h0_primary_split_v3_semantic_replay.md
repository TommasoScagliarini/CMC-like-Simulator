# Protocollo H0 primary-split V3 — semantic replay canonico

Data: 2026-08-06

## Scopo

Adattare l'actor H0 alla semantica osservativa `primary_grf_split_v1` senza
usare H0 o un teacher prescritto per generare nuove dinamiche fuori dal dominio
in cui H0 e stato validato. Il risultato atteso e un candidato H0 compatibile
con la GRF primaria sinistra online, con detector separato dagli ingressi di
carico continuo e `morphology_weight=0`.

Il protocollo non modifica la GRF primaria, il plugin SEA, la FSM legacy o il
detector V25. Non esegue PPO, non aggiorna il critic e non apre trial protetti.

## Motivazione e separazione dai rami precedenti

- V1 e terminato fail-closed perche H0 live, alimentato con la nuova semantica,
  non e un teacher stabile sui trial 02/04/08.
- V2 e terminato fail-closed perche anche il teacher cinematico prescritto
  supera 25 mm di penetrazione nei medesimi segmenti cross-speed prima di
  completare un ciclo.
- V3 e un nuovo ramo no-clobber. Non riapre, modifica o ritenta V1/V2.
- I trial 02/04/08 restano autorevoli per il detector, ma non sostengono un
  claim di robustezza dinamica H0 cross-speed.

## Input congelati

- H0: `validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/rl_module_last`.
- Configurazione H0 risolta e manifest 35/84 associati.
- Due trace train canonici H0 completi da 500 step, seed 123 e 124, e un trace
  holdout procedurale separato, seed 125:
  `validation/controller_memory_ablation/2026-07-13_markov35_corrected_full_sigma0005_seed*/rollout_policy_trace.json`.
- I corrispondenti summary storici.
- Profilo GRF primaria e profilo analogico detector gia congelati dai lock V1.

## Fase 1 — replay fisico train fail-closed

Per i seed 123–124:

1. ricostruire il runtime canonico H0 sul trial 01, offset
   `1.956870983805102 s`, con GRF primaria sinistra applicata e detector
   analogico solo per gli eventi legacy;
2. mantenere V25 disabilitato e morphology reward a zero;
3. riprodurre esattamente le 500 `raw_policy_action` del trace storico; H0 non
   seleziona alcuna azione nel loop;
4. registrare l'osservazione actor primary-split prima di ogni azione;
5. ricostruire offline la mean H0 sull'osservazione analogica storica;
6. verificare la parita bit-exact delle 33 feature non interessate dallo split.

Gate replay: 500/500 step, fine per time limit, almeno due cicli, penetrazione
`<25 mm`, zero clipping, timeout, safety stop, fallback, hard invalid e
non-finiti. Il contatore diagnostico legacy `invalid_event_count` deve essere
uguale al riferimento storico condition-matched; non viene reinterpretato
come errore nuovo del detector V25.

Il seed 125 non viene materializzato nel corpus di fit e non puo essere aperto
dal replay diagnostico o dai test pre-freeze. Prima del freeze ne sono ammessi
soltanto path, dimensione e hash gia noti come provenienza storica; il contenuto
semantico viene aperto soltanto dopo freeze del candidato e claim one-shot. E
quindi un holdout procedurale dell'adattamento, non un dato globalmente mai
osservato.

## Fase 2 — corpus train-only e adattamento singolo

Ogni stato fisico produce una coppia con la stessa label H0:

- view storica analogica;
- view runtime primary-split.

Il corpus di fit contiene soltanto seed 123–124: 1.000 stati fisici e 2.000
record paired. Il seed 125 e assente dall'NPZ passato al trainer.

E consentito un solo candidato. Si parte da H0 e si aggiornano esclusivamente
le colonne del primo layer corrispondenti a:

- `online_left_normal_grf_bw`;
- `online_left_in_contact`.

Tutto il resto dell'actor, `logstd` e ogni tensore non-actor resta bit-identico.
Iperparametri e soglie sono congelati prima dell'esecuzione nel lock V3.

Iperparametri preregistrati: seed 123, esattamente 400 epoche, batch 128,
learning rate `5e-5`, clip weight 1, logstd weight 0 e anchor weight `1e-2`.
La selezione e `fixed_final_epoch`: non esistono early stopping, patience,
validation split o best checkpoint; il candidato e incondizionatamente lo
stato dell'epoca 400.

Il gate di fit verifica integrita, finitezza, limiti azione, mutazioni limitate
esattamente alle due colonne ammesse e save/reload actor esatto. Come condizione
necessaria, ma non sostitutiva del holdout, applica inoltre sul solo train le
stesse soglie preregistrate del gate offline: riduzione RMSE primary-split
almeno 50%, RMSE `<=0.01`, massimo errore `<=0.10`, RMSE view storica
`<=0.005` e massimo errore `<=0.05`. Il candidato viene quindi congelato con
hash prima di accedere semanticamente al seed 125.

## Fase 2b — holdout finale separato

Dopo il freeze no-clobber del candidato, un worker separato esegue il replay
seed 125, produce 500 stati/1.000 record paired e valuta il candidato senza
alcun update. Gate holdout: errore primary-split ridotto almeno del 50%, RMSE
`<=0.01` e massimo errore assoluto `<=0.10`; view storica con RMSE `<=0.005` e
massimo errore `<=0.05`. Un FAIL e terminale: sono vietati retry, retuning o un
secondo candidato.

## Fase 3 — qualificazione autonoma

Il candidato viene eseguito autonomamente, senza teacher o action replay, nelle
sei condizioni canoniche:

- deterministico offset nominale e nominale ±0,20 s;
- stocastico nominale con seed 126–128 e sigma H0 `0.005`, usando noise tape
  congelati prima del fit.

Ogni caso deve completare 500 step, almeno due cicli, penetrazione `<25 mm`,
zero clipping/fallback/timeout/safety/hard-invalid/non-finiti e nessuna
regressione SEA/reserve rispetto alla baseline ammessa. La sorgente continua
resta la GRF primaria e gli eventi restano legacy in questa fase.

Prima del lock di qualificazione devono essere congelati esplicitamente il
riferimento condition-matched e i cap numerici SEA/reserve. Questi valori non
si deducono retroattivamente dai sei casi e richiedono una decision receipt
separata; lo scaffold puo essere preparato e testato prima, ma non puo eseguire
la qualifica in assenza di tale receipt.

## Fase 4 — port zero-update e avanzamento

Solo dopo la qualifica:

1. creare un checkpoint trainer con actor candidato;
2. ricreare critic e optimizer senza restore del loro stato;
3. eseguire zero update PPO;
4. verificare actor identico su learner, env-runner ed export;
5. verificare optimizer fresco, critic nuovo, save/reload deterministico.

Poi eseguire un nuovo preflight A/B/C con V25:

- A: legacy events, V25 disabilitato;
- B: V25 shadow, bit-identico ad A;
- C: V25 active, morphology reward ancora a zero.

Il lavoro autonomo si ferma prima dell'apertura one-shot del trial protetto 05.
Nessun PPO con reward positivo e nessuna promozione runtime sono autorizzati da
questo protocollo.

## Esito del preflight development del 2026-08-06

Due replay completi e due repliche indipendenti sui seed train 123–124 hanno
confermato, in modo bit-exact fra replica 1 e replica 2:

- 500/500 step, 3 cicli e fine per time limit per entrambi;
- penetrazione massima `0.023268426267771973 m` sul seed 123 e
  `0.023883867413594984 m` sul seed 124;
- zero mismatch temporali, zero mismatch sulle 33 feature non-GRF e zero
  mismatch della view analogica H0 ricostruita;
- zero clipping, timeout, safety stop, hard invalid, non-finiti, fallback SEA,
  riuso della soluzione precedente, violazioni bounds, infeasibilita selezionate
  e mismatch fra soluzione solver e controlli serviti;
- `invalid_event_count=1`, identico al rispettivo storico.

La nuova telemetria copre tutte le 5.000 finestre SO per replay, non soltanto
l'ultima finestra dei 500 policy step. Sul seed 123 SLSQP non converge 596
volte: `lsq_linear` termina con successo 244 volte e raggiunge il limite di
1.000 iterazioni (`status=0`) 352 volte. Sul seed 124 i conteggi sono 593, 287
e 306. Le soluzioni `status=0` sono finite, entro i bounds e accettate dal
runner; il massimo residuo osservato e circa `8.32e-14`, ma SciPy non ne
certifica il successo e l'optimality arriva a circa `2.85e-9`. Il vecchio
conteggio 61/59 descriveva soltanto i policy step la cui ultima finestra aveva
usato il ramo alternativo; il conteggio completo autorevole e 596/593.

Il gate preregistrato resta quindi FAIL per quattro ragioni correlate:
`fallback_count>0`, bounded-LS non tutti certificati, bounded-LS unsuccessful e
hard SO fallback secondo la definizione congelata. OSQP, una tolleranza dello
status o un aumento delle iterazioni cambierebbero il contratto e non vengono
applicati in questo ramo.

Prima del freeze serve una decisione esplicita fra:

1. mantenere il vincolo assoluto e chiudere il percorso H0 canonico corrente;
2. autorizzare un addendum che ammetta esplicitamente anche `lsq_linear`
   `status=0/max_iter` quando input e output sono finiti, bounds e soglie di
   feasibility sono verificati indipendentemente, nessuna soluzione precedente
   e riusata, residui/optimality rispettano limiti congelati e le repliche sono
   bit-exact. Questa opzione e una modifica scientifica del gate, non una
   correzione tecnica automatica.

Fino a tale decisione non vengono creati lock V3, corpus, candidato o update
actor. Il seed 125 non viene aperto.

## Addendum autorizzato — policy `verified_status0_max_iter_v1`

L'autorizzazione esplicita dell'utente successiva al preflight seleziona la
seconda opzione senza modificare SciPy, le chiamate della static optimization,
i controlli serviti o la dinamica. I due receipt diagnostici FAIL e i 24
artifact delle quattro repliche restano storia immutabile: il flag grezzo
`success=false`, i conteggi raw e i gate originari non vengono riscritti.

Per la futura esecuzione V3 un caso `lsq_linear` senza successo e accettabile
esclusivamente se, nello stesso tentativo selezionato:

- `status=0`, `nit=1000` e messaggio canonico di massimo numero di iterazioni;
- `optimality <= 1e-8`;
- input, output e residui sono finiti e la forma dell'output e corretta;
- la violazione massima dei bounds e `<=1e-9`;
- la feasibility indipendente soddisfa residuo norm o massimo assoluto
  `<=1e-6`, oppure residuo relativo `<=1e-3`;
- la telemetria dei tre residui coincide entro `1e-12` con quella ricalcolata;
- soluzione bounded-LS, soluzione selezionata e controlli effettivamente
  serviti hanno lo stesso SHA-256;
- non vi sono riuso della soluzione precedente, fallback SEA, non-finiti,
  infeasibilita, violazioni dei bounds o mismatch;
- le due repliche per ciascun seed train sono bit-exact.

Il `hard_fallback` e il `bounded_lsq_success=false` raw restano visibili, ma
vengono conteggiati separatamente come verificati dalla policy. Ogni altra
firma rimane non accettata e chiude il gate fail-closed. Il gate operativo
richiede quindi zero fallback **non accettati**, non la cancellazione dei
fallback raw storici. Policy, soglie e receipt di decisione devono essere
congelati nell'execution lock prima di corpus, actor update o accesso semantico
al seed 125.
