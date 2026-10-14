# Verifica completa masking e sigma — 2026-10-01

Autorizzazione: «procedi con la verifica completa con anche i training».
Instruction check token: CMC_AGENT_OK_2026.

## Domande e confronti fissati prima degli esiti

1. Confrontare fase online e prescribed, separando avvio senza periodo valido,
   regime, HS/TO e resync. Usare le tracce esistenti e le nuove condizioni.
2. CP90/112/184 a pesi congelati, quattro condizioni clock/target visibili o
   oscurati. Sei condizioni intere già archiviate; sei nuove condizioni con
   maschere parziali. Zero PPO in questa matrice. q/qdot contengono a loro volta
   fase: l'ablation misura dipendenza dai canali, non prova non-osservabilità.
3. Due continuazioni CP90, 10 update 91–100 per ramo, probabilità episodica
   hidden 25% costante. Controllo con sigma appresa e variante con funzione
   sigma di CP90 immutabile. Mean e critic apprendibili in entrambi. Valutare
   i checkpoint 95 e 100 sia visible sia hidden, senza scegliere a posteriori
   il best. CP90 costituisce la valutazione a zero update già disponibile.

## Invarianti

Start offset1s, seed123, durata5s, segmenti10ms, 13 worker, batch4096,
minibatch512, 10 epoche, lr1e-4. Ripristino completo actor/critic/Adam/KL,
stesso RNG delle maschere CP90. Le durate diverse possono produrre diversi
numeri di episodi e di transizioni oscurate: non si assume identità delle
traiettorie raccolte. Nessun critic rewarm, nuova reward o correzione bootstrap.

La variante usa sei buffer di pesi copiati da CP90 e un forward funzionale per
la sigma. Non aggiunge parametri all'ottimizzatore. Sampler, learner e inferenza
usano gli stessi logits. La funzione sigma è congelata; i suoi valori possono
variare perché cambiano gli stati visitati. Il controllo usa lo stesso modulo
diagnostico con ramo standard. Entrambi partono dalla stessa distribuzione.

Implementazione soltanto in questa directory: wrapper process-local per la
maschera e selezione del modulo. I moduli esportati serializzano classe,
trattamento e buffer; nessuna dipendenza da una patch runtime per l'inferenza
successiva. Il resolved YAML mantiene la configurazione scientifica originale;
il trattamento è esplicito in provenance, metadata RLModule e ricevute audit.

## Verifiche e criteri di interpretazione

Preflight con restore di 12 stati Adam e KL esatti, 1000 osservazioni visibili
e oscurate, logits iniziali identici, gradienti mean attivi, sigma invariata
dopo perturbazione encoder nel solo ramo frozen, export/reload identico e
restore completo del learner custom. Nessun PPO durante preflight.

Misure: RMSE actual-target e served-target, actual-served, escursione,
allineamento picchi/fase, contatto, penetrazione, eventi/cicli, completamento,
clipping azioni, sigma su stessi stati e su campioni effettivi, reward per
visibilità. Coppie ankle positive e bande knee restano distinte dalle misure
cinematiche. Finestra principale13.19–17.94s; fallimenti riportati e confrontati
soltanto su prefissi comuni, senza estensioni o imputazioni.

Un beneficio della variante richiede miglioramento dell'imitazione hidden
senza perdita del completamento e senza peggioramento sostanziale visible;
singolo seed e breve budget sono diagnostici, non prova di superiorità generale.
Un risultato nullo non dimostra innocuità della sigma su tempi più lunghi.

Sette plot canonici per ogni nuovo rollout sotto root plot/ in directory
dedicate. Tutti gli input e i checkpoint sorgente sono conservati; output in
runs/training/MLP39_masking_sigma_verification_2026-10-01 e
runs/rollout/masking_verification_2026-10-01. Verifica live macOS; Windows resta
da eseguire, script con pathlib/subprocess e senza comandi shell specifici OS.
