# Test preliminari V26 su AB07 e confronto controllato con AB06 — 2026-10-06

**Giornata di riferimento:** 6 ottobre 2026, come richiesto dall'utente con `create_report` ed `end_day`. **Instruction check token:** `CMC_AGENT_OK_2026`.

La parte finale del lavoro è stata completata dopo mezzanotte del 7 ottobre, ora Europe/Rome. I percorsi `2026_10_07` e i timestamp delle receipt restano invariati: il report attribuisce il lavoro alla giornata richiesta, senza retrodatare le evidenze.

## Problema ed esito

Il bundle AB07 corretto nella registrazione piede–terreno doveva essere provato nel runtime V26 prima di giudicare il trasferimento della rete ex-novo. Servivano un controllo guidato dalla IK, la riproduzione del checkpoint sul banco AB06 e un primo rollout della stessa rete su AB07. Il confronto iniziale comprendeva contratti di riferimento diversi; non isolava il solo effetto della policy.

Sono stati eseguiti i tre test preliminari e, dopo il chiarimento delle metriche, due nuovi controlli IK-driven con impostazioni uguali, reset a HS misurato e tre cicli nominali dopo startup. **Nel confronto controllato AB06 sottostima l'impulso verticale del 13,80%, mentre AB07 lo sovrastima del 39,99%.** Anche l'anticipo medio degli HS è maggiore in AB07. La differenza persiste sotto lo stesso controllo, ma modello anatomico, cinematica e carichi sperimentali cambiano insieme: non è stata isolata una causa fisica unica.

La rete best47 riproduce esattamente gli output nativi storici su AB06. Il primo trasferimento su AB07 completa 5 s, ma mostra clipping del comando knee, una cancellazione HS e maggiore errore cinematico. Non costituisce una qualifica di robustezza o di generalizzazione multi-modello.

## Strategia e significato delle misure

In questo report **prescribed/IK-driven** indica una simulazione dinamica CMC-like: i riferimenti protesici sono campionati dalla IK, passano attraverso il governor e sono inseguiti dai controllori Python/SEA. Le pose effettive non sono imposte e non viene caricata una rete come sorgente delle azioni.

Nel **rollout learned**, invece, le azioni provengono dal checkpoint deterministico. Nei test qui descritti la GRF sinistra è generata e applicata dal contatto online; la destra usa le GRF sperimentali. “Differenza nel contratto di controllo” riguardava sorgente e mapping delle azioni, clock, limiti di slew e impostazioni FSM, non una sostituzione della GRF sinistra online con quella misurata.

Le misure confrontano:

- Impulso verticale sinistro simulato e misurato del medesimo soggetto, integrati sugli stessi tempi: `errore % = 100 × (I_sim / I_mis − 1)`. Nel confronto primario finale si usa il rapporto delle somme degli impulsi sui tre cicli, non la media delle percentuali per ciclo.
- Onset HS/TO del detector geometrico binario e tempi nominali ricavati dalla GRF sperimentale sinistra a 20 N. Onset, conferma, consegna e accettazione dell'actor FSM sono canali distinti. Gli attraversamenti a 20 N della forza simulata sono un ulteriore diagnostico, distinto dal detector geometrico.
- Forze normalizzate al BW del rispettivo modello; RMS e correlazione della waveform senza traslazioni temporali per far coincidere i contatti.
- Actual contro riferimento effettivamente servito dal governor, distinto da actual contro IK originale. Un piccolo errore sul riferimento servito non implica un piccolo errore sulla IK; per una policy ex-novo la fedeltà alla IK resta una misura descrittiva.

## Test 1: controllo AB07 dopo la registrazione geometrica

Run: [ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1](../../results/ab07_v26_ground_registered_prescribed_9p01s_2026_10_06_r1/). Audit: [result_summary.json](<../../Trajectory Generator/validation/AB07/prescribed_ground_registered_2026_10_06/result_summary.json>).

Il nuovo modello ground-registered è stato integrato da 10,360 a 19,370 s: 901 segmenti, 9.010 campioni a 1 ms, completamento normale. Nessuna rete. La destinazione inizialmente non vuota è stata rifiutata prima dell'integrazione; `_r1` identifica l'unica esecuzione effettiva di questo test.

| Misura | Risultato |
|---|---:|
| Impulso online / misurato, confini salvati a 10 ms | 3.909,707 / 2.749,462 N·s |
| Errore relativo di impulso | +42,199% |
| Bias medio onset HS | −89,311 ms |
| Bias medio onset TO | +118,474 ms |
| Eventi/cicli interni | 5 HS, 5 TO, 4 cicli validi |
| Invalid/resync/cancellazioni/timeout | 0 |
| RMS actual/served, knee / ankle | 0,551° / 0,520° |
| RMS actual/IK originale, knee / ankle | 7,566° / 3,350° |

I timing sono confrontati con 5 HS e 4 TO nominali; il TO iniziale non ancorato rimane un extra esplicito. Le 50 identità SEA passano. La tolleranza della derivata dell'angolo motore è ricondotta alla precisione effettiva di stampa del plugin/STO; il primo audit con soglia assoluta troppo stretta è conservato come provenance. I 42 ingressi protetti rimangono invariati.

Questo test sostituisce, come evidenza sul modello corrente, il vecchio diagnostico AB07 prima della correzione geometrica. Quel run archiviato aveva circa il 24,71% dell'impulso misurato e un solo ciclo interno; i due bundle non sono una singola ablation causale.

## Test 2: controllo del checkpoint ex-novo su AB06

Run: [MLP_V26C_best47_AB06_control_2026_10_06_5s](<../../Trajectory Generator/runs/rollout/MLP_V26C_best47_AB06_control_2026_10_06_5s/>). Audit: [result_summary.json](<../../Trajectory Generator/validation/AB06/v26c_best47_control_2026_10_06/result_summary.json>).

Checkpoint congelato: [rl_module_best](<../../Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best/>); SHA-256 di `module_state`: `2d1d9b30f2b043f4b3b7b4347c06e9b3aace1c7d089b33e2893930554d84bd7c`. Actor 35 feature, osservazione completa 84, policy deterministica, seed 123, azione absolute, un knot ogni 10 ms.

Il controllo dura 5 s, da 13,946870984 a 18,946870984 s, con 500 segmenti e 5.000 campioni. Tutti i 17 STO e i due CSV sono identici byte per byte al rollout storico best47 di AB06; coincidono anche le 500 osservazioni actor e le azioni. Le minime differenze in alcuni momenti diagnostici, al massimo 7×10⁻¹⁵ N·m, non cambiano gli output o l'esito. La riproduzione attesta il comportamento nello stesso contratto di quel controllo, non una qualifica su altri start.

Risultati: 3 HS, 3 TO, 2 cicli validi, nessun invalid/resync; penetrazione sinistra massima campionata a 1 ms 24,649 mm. Le 50 verifiche SEA passano. Sono preservati 91 ingressi protetti. Nessun training o aggiornamento dei pesi.

## Test 3: primo trasferimento best47 su AB07

Run: [MLP_V26C_best47_AB07_transfer_2026_10_06_5s](<../../Trajectory Generator/runs/rollout/MLP_V26C_best47_AB07_transfer_2026_10_06_5s/>). Audit: [result_summary.json](<../../Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/result_summary.json>).

Stesso checkpoint congelato, inferenza deterministica e contratto learned del controllo AB06; cambia il bundle AB07. Finestra 10,360–15,360 s, 500 segmenti e 5.000 campioni, completamento normale con tre clock fisici/logici concordi. Questa finestra è accoppiata all'inizio del controllo AB07, ma non alla stessa fase del reset AB06.

| Misura | Risultato learned AB07 |
|---|---:|
| Impulso rispetto al proprio misurato, griglia nativa 1 ms | +13,861% |
| Correlazione GRF | 0,454 |
| RMS errore GRF | 268,900 N |
| RMS actual/served, knee / ankle | 2,195° / 0,646° |
| RMS actual/IK originale, knee / ankle | 15,685° / 8,584° |
| Penetrazione sinistra massima a 1 ms | 27,929 mm |
| Cicli actor validi | 1 |

Un HS accettato a 12,169 s è revocato a 12,302 s per bounce; rimangono 2 HS e 3 TO, con un invalid/cancellazione e nessun resync o timeout. Il comando SEA knee è esattamente `u=+1` in 199 campioni, da 11,942 a 12,140 s; il comando prima del clamp arriva a 2,279 volte `F_opt`. Non risultano clamp della coppia motore o dell'integrale del plugin. I flag del limite dell'integrale cascade Python e il clipping di due azioni della policy sono diagnostici separati: non si può scrivere genericamente “nessuna saturazione”.

Le 50 identità SEA, i 6 controlli non tautologici e i controlli dell'actor clock passano; gli STO sono finiti. I 119 ingressi protetti e i 26 artefatti nativi rimangono invariati dopo il postprocessing. Il return negativo usa ancora la reward/corridor di AB06 e non è un nuovo gate di accettazione AB07.

Sui medesimi 5 s il controllo IK-driven AB07 ha impulso +48,657% e correlazione 0,864, usando i campioni preintegrazione a 1 ms. Il +48,725% nella tabella storica sotto usa invece i confini post-step a 10 ms: le basi temporali sono diverse. Il learned riduce l'eccesso di impulso ma peggiora la waveform; la coppia di run include sorgente delle azioni e limiti/clock/FSM differenti. Non isola l'effetto dei soli pesi.

## Ricostruzione della divergenza storica AB06

Audit offline: [result_summary.json](<../../Trajectory Generator/validation/AB06/measured_divergence_2026_10_06/result_summary.json>). Non sono state eseguite nuove integrazioni o inferenze per questa ricostruzione.

| Evidenza salvata | Errore impulso | Bias onset HS | Bias onset TO |
|---|---:|---:|---:|
| AB06 teacher prescribed J1 storico, 5 s / confini 10 ms | −17,268% | −26,753 ms | +73,440 ms |
| AB07 test 1, primi 5 s / confini 10 ms | +48,725% | −65,723 ms | +106,707 ms |
| AB07 test 1, interi 9,01 s / confini 10 ms | +42,199% | −89,311 ms | +118,474 ms |

J1 abbina 3 HS successivi al reset su 4 nominali e tutti i 3 TO; l'HS al reset è inizializzazione e non viene trasportato come nuovo evento. Il bias −26,753 ms descrive quei tre HS. J1 usava teacher IK normalizzato in azioni absolute, clock disattivato, slew 2,5/2 rad/s e FSM più restrittiva; test 1 usa azioni raw IK, clock attivo, slew disabilitato e FSM imitativa. I path dei tre profili coincidono, ma J1 non attesta individualmente tutti i loro hash storici. La tabella risponde alla domanda sull'errore storico, senza farne un confronto omogeneo.

Separatamente, il learned AB06 del controllo ha errore di impulso −18,440% sulla propria griglia salvata a 1 ms, lunga 4,999 s tra primo e ultimo campione; bias HS −33,753 ms e TO +48,440 ms. Gli eventi disponibili sono quelli actor accettati e il sottoinsieme trasportato dall'adapter: il journal binario completo e i campioni booleani a 1 ms non sono registrati in quell'archivio. Non è il risultato del controllo IK, non è stato ricostruito il canale assente e non è stato inventato un campione terminale.

## Confronto omogeneo finale: due controlli IK-driven

Protocollo e riepilogo: [result_summary.json](<../../Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/result_summary.json>). Le finestre sono state congelate nella [selection.json](<../../Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/selection.json>) prima dei rollout.

Sono conservate le impostazioni del test 1, senza rete: SEA, controllori, tre profili congelati, governor, filtri, limiti del target, clock, FSM e integrazione 1 ms. Il primo HS misurato almeno 2 s dopo l'inizio del setup seleziona il reset; devono essere disponibili 9 s nei dati e un ciclo startup seguito da tre cicli completi. Il confine di misura è nominale sperimentale, non scelto dopo aver visto i contatti simulati.

| Finestra | AB06 | AB07 |
|---|---:|---:|
| Reset HS / inizio fisico [s] | 15,615960838 | 12,591740520 |
| Fine richiesta [s] | 24,615960838 | 21,591740520 |
| Inizio dei tre cicli primari [s] | 17,163967986 | 14,087704688 |
| Fine dei tre cicli primari [s] | 21,822001051 | 18,690780190 |
| Massa modello [kg] | 78,09885033 | 58,80765053 |

I setup originali erano troppo corti per reset+9 s: sono state create copie nel solo audit, cambiando `t_end` e usando l'offset di reset selezionato. I percorsi del modello, IK, GRF e reserve e i setup canonici sono preservati. I confronti automatici delle configurazioni effettive non trovano differenze inattese. `Kp=11.3` e `11.300000000000001` della caviglia sono diversi come testo XML, ma rappresentano esattamente lo stesso double; non è un diverso guadagno.

| Misura primaria, tre cicli | AB06 | AB07 |
|---|---:|---:|
| Impulso simulato [N·s] | 1.522,230 | 1.922,050 |
| Impulso misurato [N·s] | 1.765,971 | 1.373,006 |
| Errore relativo di impulso | **−13,802%** | **+39,988%** |
| Bias onset HS, 4 confini unici | −34,250 ms | −94,464 ms |
| Bias onset TO, 3 eventi | +98,653 ms | +122,130 ms |
| Correlazione GRF | 0,944 | 0,880 |
| RMS errore GRF / BW | 0,156 | 0,378 |

Tutti i 4 HS e i 3 TO sono abbinati in ciascun soggetto, senza missing/extra nel canale onset primario e con tolleranze di pairing dichiarate. Le conferme/consegne con tolleranze storiche più strette restano separate. I tre cicli AB07 hanno eccessi di impulso +45,062%, +43,302% e +31,695%; non è un solo transitorio iniziale. Startup e tratto restante dei 9 s sono diagnostici secondari.

### Completamento fisico e problema terminale AB06

Entrambi producono 17 STO × 9.000 campioni finiti a 1 ms e raggiungono il termine fisico dei 9 s nei clock ambiente, runner e stato OpenSim. AB07 restituisce normalmente 900 segmenti. **AB06 conserva `status=failed` e soltanto 899 callback restituiti.**

L'ultimo segmento AB06 è integrato; dopo la dinamica, il gestore binario degli eventi rifiuta il confine: `(boundary − previous) / 0.001 = 9.999999998868248`, con errore rispetto a 10 di `1.13175247e-9`, maggiore della guardia `1e-9`. Il residuo temporale è −1,13175 ps. Non è un arresto per contatto: fallisce la contabilizzazione/commit del callback finale prima di observation/reward/trace. Il payload mancante non è stato ricostruito e la receipt non è stata promossa a completed. I tre cicli primari sono anteriori e interamente osservati.

La diagnosi è nelle verifiche [AB06](<../../Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/AB06/numerical_verification.json>) e [AB07](<../../Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/AB07/numerical_verification.json>). Il runtime e le soglie non sono stati modificati per ottenere una chiusura regolare.

### Verifiche e limiti fisici

- Entrambi: copertura/griglie/finitudine PASS, 50 identità SEA e 6 verifiche non tautologiche PASS. BW verificato anche dai payload originali del runtime.
- Penetrazione sinistra massima campionata sui 9 s: 22,967 mm AB06, 21,620 mm AB07, senza campioni oltre 28 mm. Il lato destro sensor-only non è il contatto applicato che usa questa guardia; i massimi campionati non certificano i sottostadi RK4.
- Nessun clamp del comando `u`, della coppia motore o dell'integrale del plugin; il flag di limite dell'integrale cascade knee compare in 1.096/1.459 campioni. Sono diagnostici distinti, non prova di assenza globale di limiti.
- Il validatore generico conserva un FAIL MTP in entrambi: RMS 13,997° / 10,828°. La richiesta delle reserve biologiche e i fallback intermedi di equilibrio sono esplicitamente registrati; completion e identità SEA non certificano validità fisiologica.
- Tutti i 44 input congelati e i 42 file nativi dei due run rimangono invariati dopo il postprocessing. I due binari plugin sono stati attestati durante il calcolo e ricontrollati immutati alla fine; non è una fingerprint acquisita retroattivamente prima del lancio.
- Sette plot canonici per soggetto, 14 PNG revisionati, nessun canale mancante, stessa selezione dei tre cicli per actual e healthy. I titoli AB06 dichiarano il fallimento dell'adapter terminale.

## File aggiunti e artefatti

Questa fase ha aggiunto strumenti diagnostici e risultati; non ha modificato controller root, plugin C++, semantica SEA, profili, pesi o modello operativo. Le precedenti modifiche di preparazione/fitting sono documentate nei report AB07/AB08 del giorno.

| Area | Strumenti nuovi e funzione |
|---|---|
| Audit AB07 ground-registered | `verify_sto_numerics.py`, `compare_prescribed_runs.py`, receipt e verifiche; `qualify_prescribed_v26.py` preesistente riusato |
| Controllo learned AB06 | `run_control.py`, `compare_control.py`, `verify_learned_rollout_numerics.py`, provenance e confronto storico |
| Trasferimento learned AB07 | `run_transfer.py`, `compare_transfer.py`, `verify_transfer_rollout_numerics.py`, osservazioni passive e verifica dei clock |
| Divergenza AB06 salvata | `analyze_learned_vs_measured.py`, `prescribed_J1/analyze_saved_prescribed.py`, riepiloghi e review |
| Audit paired | `select_nominal_windows.py`, `run_pair.py`, `compare_paired_saved.py`, `verify_paired_numerics.py`, `render_prescribed_plots.py`, `consolidate_results.py`; setup derivati e CSV dei tre cicli |

Gli script sono confinati in `Trajectory Generator/validation/`, usano percorsi `pathlib` e il shim di caricamento nativo Windows dove necessario. Le esecuzioni attestano macOS arm64; non è stata eseguita la qualifica live Windows.

Output paired: [AB06](../../results/AB06_paired_prescribed_HS_9s_2026_10_07/) e [AB07](../../results/AB07_paired_prescribed_HS_9s_2026_10_07/). Plot finali: [AB06](../../plot/AB06_paired_prescribed_HS_9s_2026_10_07/10_07_2026_1/) e [AB07](../../plot/AB07_paired_prescribed_HS_9s_2026_10_07/10_07_2026_1/). I precedenti test conservano sette plot ciascuno sotto `plot/AB07_ground_registered_v26_prescribed_2026_10_06/`, `plot/MLP_V26C_best47_AB06_control_2026_10_06_5s/` e `plot/MLP_V26C_best47_AB07_transfer_2026_10_06_5s/`.

I test degli adattatori, le review di codice e i controlli documentali non richiedono nuove integrazioni. Per il confronto paired sono state svolte 21 verifiche pure di preflight e verifiche dei manifest/PNG. Gli errori iniziali di serializzazione del postprocessing e l'audit di precisione iniziale restano conservati nelle rispettive cartelle.

## Stato delle qualifiche e TODO

AB07-Q01/Q02 e G09/G10 restano aperti per la qualifica scientifica del contatto: la riesecuzione richiesta da Q02 è conclusa, ma il carico e il timing non sono qualificati dal solo completamento. AB07-Q03 mantiene il debito MTP; Q04 e G25 mantengono Windows. Il primo test learned soddisfa l'esecuzione esplorativa G12/G13 nel suo ambito, senza chiudere la campagna multi-modello. Il preprocessing BW, AB08 e i debiti 39D/PPO non sono risolti da questi test.

Le voci seguenti sono propagate nel daily; non avviano ulteriori lavori:

- [ ] V26-T01 — Correggere e validare separatamente la guardia dei confini campionati del detector binario esposta dall'ultimo callback AB06, mantenendo il rifiuto dei batch realmente non validi e verificando casi di rounding/horizon su macOS e Windows; conservare receipt e trace failed del test originale.
- [ ] V26-T02 — Predisporre ed eseguire, quando richiesto, un confronto learned AB06/AB07 a checkpoint e contratto policy congelati, stessi criteri HS/startup/tre cicli nominali e durate comuni; la coppia IK-driven corrente non è quel test della rete.
- [ ] V26-T03 — Estendere la qualifica del trasferimento con start/condizioni e repliche dichiarati, misurando separatamente carico, HS/TO, knee clipping, margine di penetrazione, tracking, MTP e reserve; preservare best47 e tenere distinta la qualifica dai futuri training.

I TODO AB07/AB08 e G01–G29 dei precedenti report sono conservati nel [daily della giornata](../daily/2026-10-06_daily-report.md). Il bug bootstrap PPO di Ray/RLlib resta rinviato su indicazione dell'utente; non coincide con il nuovo errore terminale del detector e non è stato modificato.

Verifica documentale della chiusura: collegamenti locali esistenti e dati confrontati con gli audit finali; le tre voci V26-T01–T03 sono identiche nel daily. Conservati i 267 TODO aperti precedenti, ora 270 con le tre nuove voci. Nessuna nuova integrazione o verifica live Windows per la scrittura dei report.
