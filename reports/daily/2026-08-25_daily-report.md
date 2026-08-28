# Daily Report - 2026-08-25

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Ricostruzione storica delle architetture e dei tentativi di widening](../user/2026-08-25_ricostruzione_storica_architetture_actor_e_tentativi_rete_larga.md)
- [A0/A1 — contratto 25D e preflight](../user/2026-08-25_v26b_a0a1_protocollo_contratto25d_preflight.md)
- [B0/B1 — esecuzione e NO-GO](../user/2026-08-25_v26b_b0_b1_esecuzione_esito_nogo.md)
- [B1R1 LOCO — fit e NO-GO](../user/2026-08-25_v26b_b1r1_loco_fit_esito_nogo.md)
- [B1R1 LOCO — studio corretto](../user/2026-08-25_v26b_b1r1_loco_studio_corretto.md)
- [B1R1 — studio read-only degli split](../user/2026-08-25_v26b_b1r1_studio_split_readonly.md)
- [B1R2 — diagnostica](../user/2026-08-25_v26b_b1r2_diagnostica.md)
- [B1R2 — diagnostica rev1](../user/2026-08-25_v26b_b1r2_diagnostica_rev1.md)
- [B1R2-A — revisione indipendente e proposta](../user/2026-08-25_v26b_b1r2a_revisione_indipendente_e_proposta.md)
- [B1R2-B — addendum correttivo e screening](../user/2026-08-25_v26b_b1r2b_addendum_correttivo_screening.md)
- [B1R2-B — NO-GO, ipotesi falsificata](../user/2026-08-25_v26b_b1r2b_esito_nogo_ipotesi_falsificata.md)
- [Fase A — diagnostica read-only](../user/2026-08-25_v26b_fasea_diagnostica_readonly.md)
- [Fase A — rettifica e protocollo 25D](../user/2026-08-25_v26b_fasea_supplemento_rettifica_e_protocollo_25d.md)
- [Ramo 35D masked — protocollo e preflight](../user/2026-08-25_v26b_ramo_35d_masked_protocollo_e_preflight.md)

## Sintesi

La ricostruzione storica ha corretto il percorso operativo. Le reti riuscite tra giugno e agosto mantenevano la stessa capacità — **due hidden layer tanh da 256**, con larghezze d'ingresso cambiate 31→17→39→35 — e non usavano un adapter appreso. I tentativi di widening a 512/1024 avevano superato verifiche offline ma erano falliti closed-loop per penetrazione; non vi è quindi evidenza a favore di una rete più grande.

È stato inoltre chiarito che il riferimento «training del 15 luglio» non identifica un checkpoint ex-novo promosso: quel giorno fu eseguito un pilot PPO sul lineage luglio, mentre la qualità utile proveniva dal bridge supervisionato precedente. Luglio deve essere usato come **fonte metodologica**, non come parent operativo. Il parent corrente resta esclusivamente il migliore training imitativo V26 di agosto.

I rami A0/A1, B0/B1, LOTO/LOCO e B1R1/B1R2 hanno prodotto studi, fit e revisioni, ma nessun candidato promuovibile. Il passaggio a un actor 25D separato è stato ritirato: la decisione corretta è un **solo actor 35D** con le colonne non osservabili mascherate/azzerate nello stadio base e riabilitate nello stadio Markov. Le ipotesi su split, patience, budget e learning rate esplorate nei rami B1 sono state insufficienti o falsificate.

## Lavoro svolto

- Ricostruite da codice e artefatti le architetture storiche e i fallimenti dei tentativi di allargamento.
- Progettati ed eseguiti i rami diagnostici 25D/35D masked e gli studi LOTO/LOCO/B1R1/B1R2.
- Corrette conclusioni intermedie su split, teste/coda, leakage e limite informativo.
- Stabilito che il prossimo percorso deve replicare l'ordine BASE→MARKOV di luglio sul parent agosto, senza pesi, dataset o trace luglio.
- Nessun PPO, critic warm-up o training ex-novo lanciato.

## Test e verifiche

- Verifica degli shape e delle architetture storiche direttamente nei checkpoint/artefatti.
- Controlli read-only e fit preregistrati dei rami B0/B1; tutti i NO-GO conservati senza promozione.
- Verifiche di immutabilità/no-clobber sui receipt e sui module state dei tentativi esplorativi.

## TODO completati

- [x] Escludere il widening come rimedio supportato dall'evidenza.
- [x] Ritirare l'actor intermedio 25D come artefatto separato.
- [x] Stabilire il parent operativo: imitation V26 agosto; luglio solo metodo.
- [x] Conservare LOTO, LOCO, B1R1 e B1R2 come rami futuri, non integrarli nella baseline senza approvazione dell'utente.

## TODO aperti e propagati

- [ ] Preregistrare una replica fedele BASE→MARKOV di luglio usando un unico actor 35D e il parent V26 agosto.
- [ ] Misurare sigma nel caso corrente prima delle recovery; 0,005 non va assunto.
- [ ] Risolvere il gate di forma caviglia/plantarflessione e la copertura delle fasi closed-loop senza sacrificare la preservazione nominale.
- [ ] Monitorare la penetrazione rispetto alle bande 20/25/28 mm e la copertura swing/WAIT_HS.
- [ ] LOTO, LOCO, B1R1 e B1R2 restano TODO futuri e richiedono approvazione esplicita dell'utente prima di ogni integrazione.
- [ ] Generalizzazione multimodello EPIC, differita.
- [ ] Semantica `cycle_valid` dopo HS cancellato, osservabilità RLlib e TODO tecnici/amministrativi storici ancora aperti.

