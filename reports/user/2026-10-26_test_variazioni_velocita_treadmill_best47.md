# 2026-10-26 — Best47: variazioni di velocità sul treadmill

**Esito: la struttura attuale non completa questa sequenza di velocità.** Il rallentamento viene attraversato, con un allungamento del ciclo protesico, ma durante la successiva accelerazione interviene la terminazione per penetrazione del contatto. Il tratto costante a +20% e il ritorno alla velocità nominale non sono stati raggiunti. Questo risultato riguarda il protocollo sintetico qui descritto, non tutte le possibili variazioni di velocità.

La prova è stata avviata dopo il completamento e la scrittura del [report camminata–fermo–ripartenza](2026-10-26_test_camminata_fermo_ripartenza_best47.md). Usa lo stesso checkpoint congelato best47 su AB06 treadmill, preset ex novo corrente, actor a 35 feature, FSM binaria V26 e FSM actor v3, inferenza deterministica, seed 123. Nessun reset tra le fasi, nessun aggiornamento dei pesi o modifica delle guardie.

Il problema verificato è la capacità della catena completa di reagire a cambi di ritmo in linea retta. La soluzione sperimentale è il banco di prova isolato `run_transition.py`: una mappa temporale regolare modifica la cinematica biologica, con derivate coerenti; GRF prescritte destre, COP e momenti seguono la stessa mappa; la velocità della superficie online viene scalata agli stadi RK4 senza cambiare direzione. La protesi resta controllata dalla rete. I fattori sono relativi al caso calibrato, non velocità di cammino validate in m/s. Il contratto actor non riceve un nuovo comando esplicito di velocità.

La durata prevista era 11,50 s, dall'istante assoluto 13,946871 s. I plateau lento e veloce durano 3 s per consentire l'osservazione di un ciclo completo; i cicli che attraversano una rampa vengono analizzati separatamente.

| Fase | Tempo dall'avvio | Fattore | Copertura effettiva |
|---|---:|---:|---|
| Nominale | 0–2,0 s | 1,0 | Completa |
| Rallentamento | 2,0–2,5 s | 1,0 → 0,8 | Completa |
| Lento | 2,5–5,5 s | 0,8 | Completa |
| Accelerazione | 5,5–6,0 s | 0,8 → 1,2 | Fino a 5,93 s |
| Veloce | 6,0–9,0 s | 1,2 | Non raggiunta |
| Ritorno | 9,0–9,5 s | 1,2 → 1,0 | Non raggiunta |
| Nominale finale | 9,5–11,5 s | 1,0 | Non raggiunta |

La terminazione avviene dopo **593 step, a 5,930 s**, con `end_reason=grf_penetration`, fattore **1,19120** e penetrazione **28,18722 mm**, oltre la soglia di episodio di **28 mm**. È l'attivazione di una guardia della simulazione; non è un crash numerico e non dimostra da sola una caduta. La FSM è in `STANCE_AFTER_HS`, con 3 HS, 3 TO, 2 cicli validi, zero eventi invalidi, zero risincronizzazioni e nessun timeout. Non si osserva clipping delle azioni. Il bacino rimane sopra 0,95247 m; la massima norma delle reserve è 431,80 N·m.

Il controllo nominale completa 9,50 s e cinque cicli, con penetrazione massima 24,62730 mm. Nei primi 2 s comuni, prima della perturbazione, la massima differenza tra posizioni protesiche è 3,00×10⁻⁶ rad e tra azioni è 2,02×10⁻⁵. Non emerge una divergenza sostanziale introdotta dalla preparazione del caso. Il controllo copre il tempo dell'intera perturbazione effettivamente eseguita; i 2 s finali di recupero previsti avrebbero avuto un orizzonte non coperto dal controllo.

**La cadenza cambia, ma il rallentamento non è qualificato come regime stabile.** I periodi nominali misurati sono 1,535–1,592 s. Nella prova di velocità:

| Ciclo protesico valido | Periodo | Interpretazione |
|---|---:|---|
| 1,643–3,570 s | 1,927 s | Attraversa il rallentamento |
| 3,570–5,674 s | 2,104 s | Attraversa 174 ms della successiva accelerazione |

Il secondo ciclo ha fattore medio 0,802199, minimo 0,8 e massimo 0,892829. Non è interamente contenuto nel plateau lento: **la misura di regime a 0,8 rimane inconclusiva secondo il criterio scelto**. Il ciclo corrispondente nelle GRF sorgente deformate temporalmente dura 1,94110 s, da 3,458991 a 5,400091 s. Il ritardo dell'HS reale passa da 111,01 a 273,91 ms. È evidenza di ritardo rispetto alla temporizzazione biologica sintetica, non una misura autonoma della velocità di avanzamento del corpo.

La sonda separata delle FSM, con contatti ideali, accetta invece le cadenze 1 → 0,8 → 1,2 → 1 senza timeout o eventi invalidi. Il risultato dinamico non può quindi essere attribuito semplicemente al rifiuto di quegli intervalli temporali da parte della FSM. Una singola prova non separa le responsabilità di generatore, tracking, contatto e supporto biologico.

Sono stati aggiunti o aggiornati soltanto gli strumenti e artefatti sperimentali in `Trajectory Generator/validation/speed_transitions_2026_10_26/`, i report e i grafici. Il protocollo di velocità in `run_transition.py` è stato esteso ai plateau di 3 s prima dell'esecuzione; `analyze_results.py` misura copertura, cicli, confronto iniziale e integrità numerica. Verificati tutti i 17 STO e gli stream diagnostici: finiti; aggiornamento nativo della superficie e derivate cinematiche: superati; 41 input protetti: invariati. Prodotti i sette grafici canonici, senza canali mancanti. Nessuna modifica a checkpoint, preset operativo, plugin o sorgenti produttivi.

Limiti: singolo checkpoint/soggetto/seed, GRF sorgente con ampiezze conservate, transizioni sintetiche di 0,5 s e supporto biologico prescritto. Non sono stati testati il regime a +20%, il recupero finale o rampe più lente. Esecuzione nativa macOS arm64; Windows non eseguito. Il risultato è negativo per questa sequenza, senza estenderlo a un'incapacità generale di camminare a velocità diverse.

Evidenze: [rollout e protocollo](<../../Trajectory Generator/validation/speed_transitions_2026_10_26/speed_changes/>), [analisi numerica](<../../Trajectory Generator/validation/speed_transitions_2026_10_26/analysis.json>), [sette plot canonici](../../plot/speed_transitions_2026_10_26/speed_changes/), [grafico comparativo](../../plot/speed_transitions_2026_10_26/transition_overview.png).

Riproduzione dalla radice, con `envCMC-rllib` attivo e un suffisso nuovo:

```sh
python "Trajectory Generator/validation/speed_transitions_2026_10_26/run_transition.py" --case speed_changes --suffix _repeat
```

**TODO aperto:** distinguere il ritardo del generatore dalla risposta del contatto con prove mirate, usando plateau più lunghi e rampe più lente per qualificare separatamente rallentamento, regime veloce e recupero. Questa campagna non implementa correzioni funzionali.
