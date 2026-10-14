# Audit ripresa e campionamento — 22 settembre 2026

## Bug Adam dimostrato e correzione

Nel checkpoint della 39D il contatore Adam passa da **2025 a iterazione 25**
a **81 a iterazione 26**, poi 162 a 27, 324 a 29 e 1215 a 40. La ripresa
25→40 ha mantenuto i pesi ma ha effettivamente riavviato Adam. Questo è un
fattore confondente dimostrato della continuazione; non dimostra che sia la
causa del divario V26, già osservato prima di 25.

Il codice installato RLlib 2.55, `TorchLearner._set_optimizer_state`, converte
l'intero stato con `convert_to_torch_tensor`, inclusi gli ID interi dentro
`param_groups[...]['params']`. Le chiavi del dizionario dei momenti restano
intere. `torch.optim.Optimizer.load_state_dict` non associa gli ID-tensore
agli ID-interi e lascia i momenti sotto chiavi orfane. Il primo update crea
quindi nuovi momenti per i parametri vivi. La sola presenza di momenti nel
checkpoint, o un controllo di `get_state`, non rileva l'errore.

`unified_MLP_39/optimizer_resume.py` ripristina direttamente lo stato originale
del checkpoint convertendo soltanto i valori dei momenti e lo step in tensori.
Gli ID dei parametri rimangono interi. Subito dopo `algo.restore_from_path`,
prima di qualsiasi update, il trainer verifica:

- associazione di tutti i momenti ai parametri vivi;
- identità di gruppi, ID, step e contenuto byte per byte dei momenti;
- pesi actor e critic invariati;
- corrispondenza fra gli optimizer del checkpoint e quelli del learner.

In caso di differenza solleva errore. Salva `resume_optimizer_audit.json`
nell'output della ripresa. La riapplicazione preesistente del learning rate
avviene successivamente; la ricetta concordata usa lo stesso LR 1e-4.

### Verifiche eseguite

`tests/test_optimizer_resume.py`: **3 test passati**. Il test principale
riproduce il difetto RLlib e dimostra che, dopo la correzione, il successivo
update Adam con gradienti identici coincide esattamente con un optimizer
senza interruzione. Altri test controllano optimizer mancante e ripristino
idempotente quando la mappatura è già corretta.

`audit_optimizer_resume.py` ricostruisce il learner reale dai checkpoint
storici 25 e 40, senza Ray.init, ambiente o simulazione, e senza update.
`optimizer_resume_audit.json` contiene hash e risultati: 12/12 momenti
correttamente associati, digest originali/ripristinati identici, step 2025 e
1215 rispettivamente, pesi invariati; invariati anche `weights_seq_no`,
`should_module_be_updated` e `metrics_logger`. Nessun checkpoint modificato.

La ripresa autorizzata da 40 deve conservare **1215**, non sintetizzare 3240
o tentare di ricostruire una storia controfattuale. A batch invariato il primo
checkpoint 41 dovrebbe contenere step **1296**.

### Coefficiente KL adattivo e scheduler

Il `PPOLearner` installato non serializza direttamente
`curr_kl_coeffs_per_module`. Il metrics logger del checkpoint contiene la
chiave `curr_kl_coeff` ma con `values=[]`, quindi non permette di ricostruire
il valore finale. La history40 documenta **0.20000000298023224**.

`learner_state_resume.py` salva in ogni nuovo checkpoint completo un sidecar
`unified_learner_state.json` con KL, entropy e learning rate, legato allo
SHA256 del relativo `learner_group/learner/state.pkl`. Il salvataggio è
agganciato sia alle milestone immutabili sia ai checkpoint last/best; nella
milestone avviene prima della pubblicazione atomica. Il ripristino verifica
hash, nomi e valori, poi ripristina KL esattamente. L'audit compare nel campo
`ppo_training_state` di `resume_optimizer_audit.json`.

Solo per il checkpoint40 già esistente il fallback è vincolato al suo SHA256
verificato e al valore di history sopra; non si assume il valore iniziale
per altri vecchi checkpoint. Legacy sconosciuti senza sidecar vengono rifiutati.
La ricetta usa LR ed entropy costanti: sono confrontati esattamente; scheduler
variabili estranei a questa ricetta vengono rifiutati anziché azzerati.

`tests/test_learner_state_resume.py`: **4 test passati**, incluso ripristino
di KL artificialmente adattato a 0.675, legame hash del sidecar, rifiuto del
legacy ambiguo e di scheduler non gestiti. **7 test totali**. La verifica
offline del learner reale40 conferma anche KL esattamente preservato; il JSON
probatorio è stato aggiornato. Non è stata modificata la ricetta PPO.

## Ripresa 40→60→80→100

`--iterations` è il limite assoluto. Ogni tappa deve usare il medesimo output
e il checkpoint completo dell'ultima milestone validata. Non usare
`rl_module_last` per addestrare: contiene il modulo di valutazione, non lo
stato completo di training. Lanciare un solo trainer per volta, attenderne
l'uscita e la chiusura dei processi di sua proprietà prima dei rollout e
della tappa successiva. La configurazione resta 13 worker, batch 4096.

Il supervisor può saltare un'iterazione dopo un timeout: `status=completed`
da solo non certifica una tappa valida. Richiedere storia contigua 1…N,
assenza di skip, checkpoint e metadata a N, prefisso precedente immutato,
audit optimizer riuscito e uguaglianza dei digest iniziali actor/critic.
Archiviare gli audit per tappa, perché quelli nell'output principale vengono
sovrascritti dalla ripresa successiva.

Esiste una finestra di crash fra pubblicazione della milestone immutabile,
append della history e salvataggio di `checkpoint_last`: un riavvio cieco
può incontrare una milestone già esistente o una history più avanti del
checkpoint. In questo caso fermare la sequenza e verificare coerenza di
checkpoint/history, senza cancellare artefatti o duplicare iterazioni.

Aggiornare il source freeze con la patch autorizzata e la telemetria, mantenendo
la provenienza degli hash precedenti. Verificare gli hash prima di ogni tappa.
Le valutazioni fisiche devono usare l'ultima milestone 60/80/100 richiesta,
senza selezionare implicitamente il best.

### Limite RNG

La maschera prescribed ha un proprio stato serializzato, ma i checkpoint
RLlib esaminati non contengono gli stati globali Torch per l'esplorazione o
NumPy per i minibatch. La ripresa non è quindi equivalente bit per bit a un
processo ininterrotto e gli episodi dei worker ripartono. Questo limite non
autorizza a dichiarare un ripristino RNG completo; i vecchi stati non sono
ricostruibili dai checkpoint. La patch Adam non cambia questi meccanismi.

## Distribuzione delle azioni per diagnosi del rumore

La distribuzione effettiva è `TorchDiagGaussian`: divide i logits in media
e log-deviazione standard, usa sigma=`exp(log_std)` e campiona una normale
non trasformata. La modalità stocastica del rollout richiama la medesima
distribuzione. Il wrapper dell'ambiente taglia l'azione grezza a [-1,1], poi
la mappa sui limiti angolari e la passa al governor.

Per il confronto controllato usare `a_raw = mu + alpha * sigma * z`,
con `alpha` 1 o 0.5, `z` normale standard indipendente per giunto e decisione
di 10 ms. `hold_steps=1` è necessario: mantenere z per più step modifica la
correlazione temporale dell'esplorazione. Usare gli stessi z nei due bracci
riduce la variabilità del confronto, pur producendo stati successivi diversi.
NumPy e Torch generano la stessa legge, non la stessa sequenza da un seed.
Inviare l'azione grezza all'ambiente, mantenendo il clipping nel wrapper
come durante il training; non scalare media o intera azione e non modificare
il checkpoint. Distinguere sigma nativa, sigma applicata e saturazione.
