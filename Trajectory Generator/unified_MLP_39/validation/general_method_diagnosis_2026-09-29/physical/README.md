# Carico fisico, reserve e momento esterno — 2026-09-29

Questa analisi confronta ogni sequenza offline con la propria baseline congelata
LEFT o RIGHT. Non esegue simulazioni, non introduce reward o criteri di successo
e non modifica i controllori. Cinematica e forma delle coppie sono analizzate
separatamente in `../quality/`.

Scopo: controllare se un eventuale miglioramento cinematico o di regolarita
si accompagna a scarico del piede protesico, assistenza maggiore delle reserve,
penetrazione diversa o cambiamento del momento GRF che il SEA deve compensare.

## Risultati dei due episodi completi

Entrambe le sequenze concludono 500 step con `episode_time_limit`. Il confronto
principale usa esattamente 13.15-17.95 s per tutti i segnali fisici. Valori
baseline -> sequenza; la reference propria e preservata entro ciascuna coppia.

| Metrica fisica | LEFT | RIGHT |
|---|---:|---:|
| Impulso GRF verticale sinistra [N*s] | 354.28 -> 1166.49 | 347.51 -> 932.23 |
| GRF verticale media sinistra [N] | 73.81 -> 243.02 | 72.40 -> 194.22 |
| Tempo in contatto loggato [%] | 40.44 -> 54.94 | 46.46 -> 53.42 |
| Penetrazione massima STO 1 kHz [mm] | 14.149 -> 19.028 | 14.105 -> 25.074 |
| Tempo oltre soglia soft esistente 20 mm [%] | 0 -> 0 | 0 -> 10.48 |
| Reserve traslazioni pelvis, norma RMS [N] | 418.95 -> 222.41 | 423.07 -> 271.65 |
| Reserve pelvis_ty RMS [N] | 413.07 -> 212.65 | 417.88 -> 263.63 |
| Reserve rotazioni pelvis, norma RMS [Nm] | 92.07 -> 89.84 | 87.75 -> 97.58 |
| Reserve altre rotazioni biologiche, norma RMS [Nm] | 49.92 -> 51.09 | 49.99 -> 51.71 |
| Reserve locali protesi, norma RMS [Nm] | 0 -> 0 | 0 -> 0 |
| Knee spring-command RMSE [Nm] | 2.399 -> 0.579 | 2.178 -> 0.665 |
| Ankle spring-command RMSE [Nm] | 0.545 -> 0.597 | 0.360 -> 0.496 |

Il caso LEFT mostra piu carico online e meno assistenza traslazionale del bacino,
non una strategia che alleggerisce il piede o si appoggia maggiormente a quelle
reserve. L'impulso aumenta di un fattore 3.29, mentre la norma RMS delle forze
reserve pelvis diminuisce del 46.9%. Le reserve rotazionali biologiche crescono
del 2.3%: non si afferma una diminuzione uniforme di ogni reserve. Il contatto
diventa anche piu profondo; la penetrazione resta sotto la soglia soft gia
configurata, ma questo non certifica automaticamente realismo del contatto.

RIGHT aumenta il carico e riduce anch'esso l'assistenza traslazionale, ma con
penetrazione maggiore, 10.48% del tempo sopra la soglia soft e reserve rotazionali
pelvis maggiori dell'11.2%. Nessuna delle due sequenze supera il limite hard
esistente di 28 mm nei campioni registrati. Questi risultati distinguono il
successo cinematico dalla qualita dell'interazione meccanica con il terreno.

### Ankle sotto carico

Si usa soltanto la soglia di carico gia esistente, 50 N. I campioni caricati
sono diversi tra i rollout: le medie condizionate non appaiano gli stessi
istanti o le stesse geometrie. La correlazione spring/momento GRF e invece
calcolata su coppie di segnali del medesimo stato entro ciascun rollout.

| Metrica caricata | LEFT baseline -> sequenza | RIGHT baseline -> sequenza |
|---|---:|---:|
| Coppia spring media [Nm] | +0.896 -> -11.556 | +2.711 -> +19.863 |
| Momento GRF generalizzato medio [Nm] | -0.186 -> +12.343 | -2.023 -> -19.090 |
| RMS spring + momento GRF [Nm] | 0.910 -> 0.799 | 0.736 -> 0.798 |
| Correlazione spring / opposto momento GRF | .99837 -> .99984 | .99964 -> .99995 |
| COP_z - ankle_z medio [m] | -.01780 -> -.04207 | -.00035 -> +.04014 |

Il momento esterno registrato spiega direttamente la diversa coppia ankle
richiesta nei due contesti. LEFT produce una coppia prevalentemente negativa
con COP medio dall'altro lato rispetto a RIGHT, mentre RIGHT compensa un
momento GRF negativo con coppia positiva. Non e necessario attribuire il segno
positivo a rumore casuale del SEA; questo non autorizza pero una regola generale
di segno o l'imposizione della forma V26 al target LEFT.

### Precisione dei massimi di penetrazione

Non mescolare il massimo della finestra STO a 1 kHz con quello del summary a
100 Hz. Inoltre il summary dei baseline non include il campione di reset:
riporta 14.795/14.789 mm, mentre lo STO intero include 16.23769 mm al reset
comune. I valori nella tabella principale si riferiscono tutti alla stessa
finestra 13.15-17.95 s e alla stessa frequenza del recorder. Entrambe le versioni
e l'intero prefisso supportato sono conservati in `metrics.json`.

## Metodo

`analyze.py` richiede `diagnostic_receipt.json` finale per baseline e sequence:
non legge come completi rollout ancora in esecuzione. Le coppie di input sono
`LEFT_baseline/LEFT_sequence` e `RIGHT_baseline/RIGHT_sequence`, sotto
`unified_MLP_39/runs/rollout/general_method_diagnosis_2026-09-29/`.

- Finestra principale richiesta: 13.15-17.95 s, limitata all'intersezione
  effettivamente registrata da entrambi i casi e da tutte le tabelle necessarie.
  Si riporta anche l'intero prefisso comune, incluso il transitorio di reset.
  Una terminazione anticipata non viene trattata come un episodio da 500 step.
- Carico: media/RMS e impulso della GRF verticale e della forza normale online
  applicate a sinistra, nei propri N e N*s. Non si integra la GRF online destra
  sensor-only come se fosse una forza applicata. Non si ricostruisce qui la GRF
  prescribed destra.
- Contatto: frazione temporale e durata del booleano gia registrato dal modello,
  senza nuovi detector o soglie. La condizione caricato usa la soglia preesistente
  `grf_ankle_moment_flip_force_threshold_n`, verificata nelle due configurazioni.
- Penetrazione: media/RMS/massimo e tempo oltre le soglie soft/hard gia presenti
  nel rollout. Si riporta separatamente il massimo summary dell'episodio.
- Integrali e RMS temporali: trapezi sui campioni STO a 1 kHz, con interpolazione
  ai soli bordi supportati. Nessuna estrapolazione dell'ultimo endpoint omesso
  dal recorder. Le frazioni ai cambi di booleano hanno precisione del recorder.

Le reserve sono separate per dimensione fisica. Le colonne del file chiamate
`pelvis_tx/ty/tz_reserve_torque` sono forze generalizzate **N**, non Nm. Si
riportano per coordinata e come norma vettoriale RMS delle tre traslazioni.
Le rotazioni del bacino, le altre coordinate biologiche e i due giunti protesici
sono gruppi distinti in **Nm**. Non si sommano N e Nm in una norma mista.

Per l'ankle, il trace registra a 100 Hz il momento GRF generalizzato gia
proiettato dal codice di produzione:

`tau_GRF = ankle_axis . (M_ground - ankle_center x F)`.

Sui campioni disponibili e caricati si confronta la molla con `-tau_GRF`,
riportando correlazione e RMS della somma. La somma **non e un residuo completo
di inverse dynamics**: mancano inerzia, gravita e altri accoppiamenti. Il suo
valore non misura l'errore interno SEA. Quest'ultimo e descritto separatamente
dal confronto tra `tau_spring` e `tau_ref` registrati. Si conserva la convenzione
di segno nativa OpenSim, senza i cambi di segno grafici usati per mostrare la
flessione del knee come positiva.

Non e registrato un momento esterno generalizzato equivalente per il knee.
Non viene ricostruito da un semplice prodotto forza-per-COP o dedotto dal dato
ankle: si riportano carico, coppia e comando knee senza fingere quel bilancio.

## Interpretazione e limiti

La presenza di piu impulso o minori reserve elimina solo alcune spiegazioni
banali del miglioramento. Non certifica da sola un contatto corretto, una
strategia biomeccanica plausibile o la qualita della forma di coppia.
Analogamente una maggiore penetrazione puo aumentare il carico nel modello:
carico e penetrazione devono essere letti insieme.

La sequenza e deterministica e usa il target futuro offline. Un esito positivo
resta una prova di realizzabilita da un reset specifico con quel simulatore;
non dimostra che la rete 39D abbia imparato quel comportamento, che sia robusto
o che generalizzi. Nessun rapporto baseline/sequence viene usato come soglia
di promozione.

## Esecuzione

Dal root, dopo la chiusura dei due worker sequence:

```sh
python "Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/physical/analyze.py"
```

`--sides LEFT` o `--sides RIGHT` permette una coppia sola. Ogni esecuzione
scrive `metrics.json`, `input_hashes.json`, `verification.json` in questa
cartella; usare entrambe le coppie per l'artefatto finale. Il codice usa NumPy
e pathlib, senza dipendenze OpenSim o specifiche dell'OS.

Esecuzione completata sui quattro rollout live dopo le receipt finali. Verificati:
parsing, unita separate, soglie/configurazioni di contatto uguali entro ciascuna
coppia, assenza di estrapolazione, checkpoint invariati nelle receipt e momento
ankle disponibile su tutti i 481 campioni di ciascuna finestra principale.
Gli SHA256 di tutti gli input sono rimasti invariati. Output:
`metrics.json`, `input_hashes.json`, `verification.json`, `execution.log`.
