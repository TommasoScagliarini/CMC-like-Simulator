# Verifica della calibrazione AB08 e della qualità della IK

Data: 2026-10-06. Instruction check token: `CMC_AGENT_OK_2026`.

## Problema ed esito

La preparazione AB08 ha prodotto una IK con errore RMS medio sui marker di 11,998 mm e un massimo singolo di 114,931 mm. La richiesta era verificare se questi valori indicassero una calibrazione errata della pipeline.

La verifica indipendente conferma che **la calibrazione statica e geometrica AB08 è stata eseguita correttamente secondo il riferimento comune AB06**. Il lato protesico durante il cammino presenta un RMS medio praticamente uguale ad AB06. L'RMS complessivo maggiore riguarda soprattutto il lato biologico destro.

Il picco di 114,931 mm coincide con un salto del marker destro `R_Thigh_Upper` già presente nel MAT originale del dataset. La conversione non lo ha introdotto. La pipeline usa anche quel campione nella IK e produce un impulso nelle coordinate articolari. La gestione dei marker anomali prima della IK rimane un intervento da realizzare; in questa indagine non è stata modificata la cinematica adottata.

## Preparazione eseguita

Il comando autorizzato è terminato con codice 0:

```sh
/opt/anaconda3/envs/envCMC-like/bin/python tools/build_abx_seasea_pipeline.py \
  --subject AB08 \
  --bundle-dir models/AB08_SEASEA_Threadmill \
  --model-stem AB08_SEASEA_stiff321_500_pi \
  --matlab-nojvm \
  --skip-smoke-test
```

La preparazione ha completato innesto SEA, fitting anatomico, conversione dei dati statici e treadmill, registrazione geometrica sul piano comune, calibrazione dei marker sinistri, nuova IK OpenSim dai marker e creazione del bundle con GRF e overlay sano.

- Bundle: [AB08_SEASEA_Threadmill](../../models/AB08_SEASEA_Threadmill/).
- Modello: [AB08_SEASEA_stiff321_500_pi.osim](../../models/AB08_SEASEA_Threadmill/AB08_SEASEA_stiff321_500_pi.osim).
- Cinematica: [IK_results_AB08_SEASEA.mot](../../models/AB08_SEASEA_Threadmill/data/IK_results_AB08_SEASEA.mot), 28.580 campioni a 200 Hz, da 8,630 a 151,525 s.
- Report della build: [pipeline_report.json](../../models/AB08_SEASEA_Threadmill/pipeline_report.json).
- Modello nativo: 21 coordinate, 76 muscoli, 11 corpi; massa 72,412369 kg, preservata rispetto al modello sano AB08.

I parametri SEA corrispondono al donor AB06 PI. Il controllo degli hash ha confermato la conservazione della catena di controllo, dei riferimenti AB06 e del bundle AB07. Il comando comprendeva `--skip-smoke-test`: non sono stati eseguiti rollout dinamici, valutazioni della rete o training. Gli avvisi sulla mancanza di forze nel trial statico non hanno impedito la calibrazione; le mesh sono risolte dal loader runtime nell'ambiente macOS utilizzato.

## Strategia di verifica

Sono state separate quattro verifiche, utilizzando i file effettivamente adottati:

1. Ricalcolo in OpenSim delle posizioni dei marker sulle 322 pose statiche AB08, confronto dei centroidi con CSV e modello salvato, controllo di unità e parent body.
2. Confronto della geometria biologica, dei marker conservati, dei parametri SEA e del setup IK con i modelli sorgenti e il donor AB06.
3. Ricalcolo tramite cinematica diretta degli errori per marker sulle IK complete salvate di AB06, AB07 e AB08, confrontandoli con i diagnostici nativi disponibili.
4. Confronto numerico fra il MAT originale AB08 e il TRC esportato, più valutazione del marker problematico sul modello sano AB08 usando la IK del dataset.

Le verifiche hanno valutato pose già salvate: non hanno rilanciato il solver IK né integrato la dinamica. La lettura finale del MAT è stata effettuata con il decoder MCOS/SciPy disponibile, senza nuova conversione MATLAB.

## Calibrazione statica e fitting

| Verifica statica | Risultato |
|---|---:|
| Pose AB08 | 322 |
| Marker verificati | 28 |
| RMS complessivo | 0,223 mm |
| Errore massimo | 0,616 mm |
| RMS marker sinistri calibrati | 0,213 mm |
| RMS marker destri | 0,235 mm |

I 12 marker sinistri ricalibrati coincidono esattamente con i centroidi ricalcolati e con le località registrate nel CSV. I 16 marker biologici e pelvici sono preservati dal modello sano AB08. Anche `R_Thigh_Upper` è invariato e nella statica ha RMS 0,373 mm, massimo 0,615 mm.

La registrazione geometrica ha allungato il pylon di 19,477 mm e applicato un offset di +2,509137° alla caviglia **soltanto nella motion statica utilizzata per calibrare i marker**. Zero articolare, range, assi e comandi runtime sono preservati. La verifica nativa della clearance mediana della mesh e dell'inclinazione rispetto al riferimento AB06 restituisce residui di 3,8×10⁻¹⁷ m e 2,8×10⁻¹⁷ rad.

Il file statico utilizzato proviene realmente da AB08. Il suffisso `_ik_dataset_ab06_seasea.mot` è una convenzione del converter, non indica riuso della cinematica AB06.

Questo fit è verificato sui campioni statici impiegati per la calibrazione: non è una misura indipendente della geometria fisica della protesi né una certificazione della IK durante il cammino. Il terreno segue la convenzione comune relativa ad AB06.

## Errori durante il cammino

La tabella riporta la media temporale dell'RMS per frame; ogni gruppo comprende gli stessi marker nei tre soggetti.

| Misura | AB06 | AB07 corretto | AB08 |
|---|---:|---:|---:|
| RMS medio complessivo | 10,521 mm | 10,380 mm | 11,998 mm |
| RMS medio lato protesico | 11,836 mm | 9,928 mm | 11,885 mm |
| RMS medio lato destro biologico | 7,028 mm | 9,884 mm | 11,568 mm |
| RMS medio pelvi | 13,549 mm | 11,801 mm | 12,687 mm |
| P95 del massimo per-marker per frame | 28,313 mm | 28,172 mm | 26,487 mm |
| Massimo singolo marker | 146,574 mm | 120,598 mm | 114,931 mm |
| Frame con qualche errore >50 mm | 0,381% | 0,063% | 0,238% |

Il lato protesico AB08 differisce da AB06 di circa **+0,049 mm** nell'RMS medio. L'aumento sul lato destro biologico è circa **+4,540 mm**. Questo confronto non sostiene l'ipotesi di un errore specifico nel fitting protesico AB08.

Per AB06 manca un companion nativo degli errori relativo alla IK completa adottata: il confronto usa direttamente modello, IK e TRC adottati, con cinematica diretta nativa. Il vecchio file degli errori da 101 frame non è stato usato come riferimento completo. Le ricostruzioni AB07 e AB08 coincidono con i rispettivi companion; AB08 coincide inoltre con il log nativo completo entro la precisione numerica/di stampa.

Gli errori persistenti principali AB08 includono `R_Thigh_Front` (RMS 16,691 mm), `L_Shank_Upper` (16,113 mm), `R_Thigh_Upper` (14,737 mm), `R_ASIS` (14,468 mm) e `L_Thigh_Front` (14,245 mm). La verifica non identifica una causa unica per tutti questi residui durante il cammino.

## Origine del picco e limite della pipeline

A 56,635 s il marker `R_Thigh_Upper` si sposta di **123,546 mm in 5 ms**, seguito da un ritorno di **125,281 mm** nel campione successivo. Nello stesso intervallo i marker `R_Thigh_Front` e `R_Thigh_Rear` si spostano soltanto di circa 1,8–1,9 mm.

Il confronto completo delle 28 serie marker, ciascuna da 28.580 campioni, conferma che il TRC riproduce il MAT originale entro 5×10⁻¹¹ mm; i tempi coincidono. Il salto appartiene quindi al dataset MAT disponibile. Non è stata ispezionata l'acquisizione grezza delle telecamere, pertanto non viene attribuita una causa fisica specifica all'anomalia.

Sul modello sano AB08, usando la sua IK dataset, lo stesso campione produce un errore `R_Thigh_Upper` di **144,816 mm**. La distanza Upper–Front varia di circa 114,735 mm rispetto alla mediana statica, mentre Front–Rear cambia di circa −1,099 mm. Il problema è indipendente dal fitting protesico.

La IK usa 28 marker attivi con peso unitario e nessun vincolo cinematico del dataset sano. Il campione anomalo influenza la soluzione: `hip_adduction_r` passa −3,537° → −0,988° → −3,829° e le traslazioni della pelvi presentano un impulso di circa 5 mm.

Gli errori AB08 >50 mm riguardano 68 frame, pari a 0,340 s complessivi. Il marker dominante è destro/pelvico in tutti questi frame: 55 `R_ASIS`, 8 `R_Thigh_Upper`, 5 `R_Thigh_Front`.

Gli outlier spiegano i picchi ma **non spiegano l'RMS medio di 12 mm**: escludendo soltanto a scopo diagnostico i 167 frame con errore massimo >30 mm, l'RMS medio è ancora 11,966 mm. Nessun frame è stato rimosso dalla cinematica adottata. Le soglie 30/50/100 mm sono descrittive, non nuovi criteri di accettazione.

## Soluzione verificata e lavoro rimanente

Non è emerso un errore di unità, marker mapping, posa statica, scelta dei dati AB08 o registrazione geometrica da correggere. La calibrazione attuale è coerente con la pipeline e con il benchmark AB06.

Il limite individuato è la gestione della qualità dei dati marker prima della IK. Un intervento successivo dovrà definire un metodo generale e tracciabile per rilevare i campioni anomali, decidere come trattarli e verificare la nuova soluzione. La semplice eliminazione del massimo dal report non correggerebbe gli impulsi nella cinematica. L'eventuale riduzione dei residui persistenti richiede un'analisi distinta.

## File prodotti, modificati e verifiche

La build ha generato il bundle `models/AB08_SEASEA_Threadmill`, gli intermedi `models/AB08_SEASEA-raw`, i risultati e log `results/ab08_seasea_treadmill_treadmill_01_01*` e lo snapshot comune `results/abx_reference_rrmf81lr`.

L'indagine ha aggiunto soltanto script diagnostici, JSON e matrici NPZ in `validation/ab08_calibration_investigation_2026_10_06/`. Non ha modificato codice operativo, plugin, parametri SEA, modelli o IK adottate. La verifica finale degli hash conferma 48 file operativi/riferimenti invariati, comprendendo AB08, AB07 e controlli comuni.

Evidenze principali:

- [Verifica consolidata](../../validation/ab08_calibration_investigation_2026_10_06/calibration_verification.json).
- [Ricalcolo statico](../../validation/ab08_calibration_investigation_2026_10_06/static_audit.json) e [script](../../validation/ab08_calibration_investigation_2026_10_06/static_audit.py).
- [Contratti di fitting e setup IK](../../validation/ab08_calibration_investigation_2026_10_06/fitting_contract_audit.json).
- [Confronto IK completo e residui per marker](../../validation/ab08_calibration_investigation_2026_10_06/ik_error_analysis.json) e [script](../../validation/ab08_calibration_investigation_2026_10_06/ik_error_analysis.py).
- [Conferma sul MAT originale](../../validation/ab08_calibration_investigation_2026_10_06/source_marker_confirmation.json).
- [Verifica sul modello sano](../../validation/ab08_calibration_investigation_2026_10_06/healthy_right_marker_audit.json).
- [Verifica della preparazione](../../validation/ab08_preparation_2026_10_06/final_verification.json).

Questo report e l'aggiornamento del daily sono gli unici file modificati per il comando `create_report`. Non sono stati rilanciati test numerici già completati. L'esecuzione e le verifiche native sono state effettuate su macOS arm64; non costituiscono una prova live Windows. Non sono stati prodotti plot di rollout perché non è stato effettuato un rollout.

## TODO aperti

- [ ] **AB08-Q01:** Definire e implementare un controllo generale dei marker prima della IK, con politica esplicita per outlier/missing data, conservazione delle sorgenti e tracciabilità dei campioni trattati; ricalcolare e confrontare la IK senza cambiare parametri SEA/control chain.
- [ ] **AB08-Q02:** Analizzare separatamente i residui persistenti durante il cammino, soprattutto marker della coscia destra e `L_Shank_Upper`; verificare l'eventuale beneficio senza attribuire agli outlier la media RMS di 12 mm o introdurre correzioni specifiche AB08 prive di giustificazione.
- [ ] **AB08-Q03:** Qualificare AB08 su contatto/eventi V26, GRF online e tracking dinamico dopo la preparazione e gli eventuali interventi sulla qualità della IK. La build senza smoke test e gli audit statici non chiudono questa qualifica.
