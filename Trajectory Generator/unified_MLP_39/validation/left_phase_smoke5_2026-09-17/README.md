# Smoke 39D sulla fase sinistra — 5 iterazioni

**PASS operativo su macOS arm64.** Cinque aggiornamenti PPO completati da pesi nuovi, senza restart del processo o iterazioni saltate. Nessuna modifica al runtime durante il run.

Configurazione: 3 worker, batch 192 (64 transizioni per start), minibatch 64, 1 epoca PPO, seed 123. Target sulla fase prescribed sinistra con shift zero; quattro target visibili al 100%, curriculum vuoto. Totale: 960 transizioni, 320 per start.

Durata del run: 337.9 s. I cinque checkpoint esportati sono stati ricostruiti in un processo di verifica separato, con digest actor identici a quelli registrati dopo ciascun aggiornamento.

| Iterazione | Return medio riportato | Lunghezza media riportata (step) | KL medio | Visibilità |
|---:|---:|---:|---:|---:|
| 1 | 14.7449 | 32.72 | 0.019540 | 100% |
| 2 | 15.8276 | 26.50 | 0.006868 | 100% |
| 3 | 4.5308 | 78.00 | 0.005886 | 100% |
| 4 | N/D | N/D | 0.003470 | 100% |
| 5 | 0.4037 | 81.00 | 0.004038 | 100% |

Le medie sono quelle riportate da RLlib e non costituiscono repliche indipendenti. N/D indica una metrica episodica non riportata in quel batch; non viene sostituita con zero. Sono presenti terminazioni degli episodi per penetrazione: il PASS riguarda il funzionamento software, non il raggiungimento di un cammino stabile o della parità con V26.

Verificati in ogni iterazione: bilanciamento dei tre start, aggiornamento di actor e critic, continuità dei digest fra aggiornamenti, loss/KL finiti, telemetria completa, maschera interamente visibile, contatori cumulativi per worker e schema checkpoint v2. Verificata l'invarianza degli hash delle sorgenti e della configurazione fotografate prima del run.

Questo smoke mantiene p=1 per tutte le iterazioni; il cambio di maschera e la ripresa PPO sono stati verificati separatamente nell'implementazione precedente. Windows non è stato eseguito.

- [Verifica dettagliata](verification.json)
- [Configurazione esatta](smoke5.yaml)
- [Log del training](training.log)
- [Manifest iniziale delle sorgenti](source_hashes_before.json)
- [Summary del run](../../runs/training/left_phase_visible_smoke5_2026-09-17/summary.json)
- [Checkpoint finale](../../runs/training/left_phase_visible_smoke5_2026-09-17/milestone_iteration_000005)
- [Script di verifica](audit_smoke.py)
