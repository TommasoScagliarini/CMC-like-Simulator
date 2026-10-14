# Sensibilità locale della policy e feedback dello stato del controller

Analisi offline del 29 settembre 2026, su AB06. Non esegue OpenSim, training,
aggiornamenti dei pesi o cambiamenti della reward. Input: rollout deterministici
LEFT best80 (penetrazione peso2), RIGHT last60 e V26 best87. I percorsi e gli hash
dei checkpoint effettivamente caricati sono salvati negli artefatti.

## Risultato

Le **1.500 azioni originali sono riprodotte esattamente**, preservando chiamate
di inferenza singole e ordine degli ingressi. Il summary V26 dichiara 43 feature
actor, ma il checkpoint ne consuma 39: le quattro diagnostiche finali sono escluse.
Per tutte le policy, campione deterministico della distribuzione = media dei
logits = azione grezza registrata. Il clipping a [-1,1], la trasformazione in
endpoint e il riuso dell'endpoint precedente sono verificati sui trace.

La sensibilità alla memoria del controller è concentrata soprattutto nelle
**accelerazioni della reference servita**, molto meno nell'endpoint precedente.
La tabella usa lo stesso fattore di scala fisica per tutte le policy. Sono
sensibilità differenziali moltiplicate per una dispersione comune, non azioni
ottenute applicando una perturbazione di un'intera deviazione standard.

| Gruppo, risposta RMS knee / ankle in azione normalizzata | LEFT best80 | RIGHT last60 | V26 best87 |
|---|---:|---:|---:|
| Endpoint precedente | 0,023 / 0,047 | 0,032 / 0,015 | 0,028 / 0,038 |
| Reference qdot | 0,363 / 0,222 | 0,221 / 0,175 | 0,266 / 0,322 |
| Reference qddot | 0,639 / 0,742 | 0,625 / 0,467 | 0,630 / 0,554 |
| Tutta la memoria controller | 0,737 / 0,776 | 0,664 / 0,500 | 0,685 / 0,644 |
| Target q e qdot | 0,792 / 0,472 | 0,411 / 0,277 | 0,742 / 0,516 |

Le accelerazioni dei due giunti hanno influenza incrociata. Per LEFT la
sensibilità ankle alla qddot knee è 0,512 e alla qddot ankle 0,537. Quindi la
diagnosi non giustifica sondare solo il canale dello stesso giunto.

Con la covarianza comune delle feature, che tiene conto delle correlazioni
all'interno di ogni gruppo, la sensibilità aggregata della memoria rimane
LEFT 0,766/0,728; RIGHT 0,672/0,490; V26 0,699/0,576. La conclusione sulla
dipendenza dalla memoria non dipende solo dall'ipotesi di coordinate indipendenti.

Il contributo qddot al preattivatore del primo layer è molto maggiore degli
altri gruppi in tutte e tre le reti: RMS a dispersione comune 2,37/2,43/2,34,
contro 0,212/0,201/0,205 per i target. LEFT visita inoltre una dispersione qddot
maggiore: 24,95/21,51 rad/s² contro 16,83/14,27 RIGHT e 19,26/15,89 V26.
La frazione di preattivatori con valore assoluto oltre 3 è 25,64% LEFT,
14,10% RIGHT e 15,72% V26. È una pista sul condizionamento numerico e sugli stati
visitati; non prova che una normalizzazione risolva il problema. Cambiare scala
agli ingressi di un checkpoint congelato senza compensare i pesi cambierebbe la
policy; compensare esattamente i pesi conserverebbe la stessa funzione.

Il collegamento isolato `azione → endpoint accettato → previous_endpoint →
media successiva` **non mostra amplificazione locale**: massima norma singolare
0,194 LEFT, 0,111 RIGHT, 0,186 V26; nessun campione oltre 1. Questo risultato
esclude solo un forte guadagno di quel singolo collegamento nelle traiettorie
nominali osservate. Non include il feedback attraverso qdot/qddot, SEA, contatto
o dinamica e non dimostra stabilità del sistema completo.

## Addendum: percorso attraverso il governor

`governor_feedback.py` usa la stessa classe di governor del simulatore, estratta
senza avviare OpenSim dal helper `../governor/analyze_governor.py`. Riproduce
ogni segmento dal suo stato q/qdot/qddot e misura con differenze centrate
±0,0001 e ±0,00005 l'effetto dell'azione su endpoint accettato e reference
servita al passo successivo. Questo effetto viene composto con il Jacobiano
della media della policy sull'osservazione successiva registrata.

| Norma singolare del percorso azione→reference→media grezza successiva | LEFT | RIGHT | V26 |
|---|---:|---:|---:|
| Media | 2,055 | 2,058 | 2,488 |
| Mediana | 1,742 | 2,006 | 2,407 |
| Massimo | 9,022 | 7,027 | 8,317 |
| Frazione campioni >1 | 58,9% | 69,9% | 78,1% |

La media grezza può essere saturata prima di diventare comando. Per questo
sono salvate separatamente anche le matrici `Dclip(mean_next) @ M`, in unità
di azione normalizzata **applicata** al passo successivo. Il clipping del
comando iniziale è già incluso in entrambi i percorsi.

| Norma singolare con clipping anche sul comando successivo | LEFT | RIGHT | V26 |
|---|---:|---:|---:|
| Media | 1,914 | 2,011 | 2,418 |
| Mediana | 1,534 | 1,927 | 2,357 |
| Massimo | 9,022 | 7,027 | 8,317 |
| Frazione campioni >1 | 57,2% | 69,1% | 76,2% |

Il clipping non aumenta la norma in alcun campione, verificato numericamente.
Nessuna media nominale successiva è esattamente sulla frontiera ±1; fuori
dall'intervallo la derivata locale di quel comando è zero. La verifica nonlineare
con clipping delle medie perturbate concorda entro 1,1e−7 in azione normalizzata.
La conclusione rimane invariata dopo questa distinzione fra media e comando.

Il contributo dominante è qddot: norma media del relativo blocco
2,004/2,029/2,382. Esiste quindi un percorso capace di aumentare la perturbazione
della media dopo un segmento. **È presente anche nella baseline V26 buona**,
con guadagno medio maggiore: non è una prova che spieghi il divario LEFT.

Questa matrice tiene ferme le altre osservazioni successive: stato articolare,
SEA, comando del cascade, carico, contatto, clock e FSM. Non è la matrice di
transizione dell'intero stato e i suoi autovalori non certificano stabilità o
instabilità del sistema. Non va iterata come se fosse il sistema completo.
Serve a localizzare un percorso da misurare nel rollout perturbato, dove gli
altri feedback possono amplificarlo o compensarlo.

La ricostruzione del singolo segmento coincide con il replay entro 1,1e−8
nelle unità di q/qdot/qddot. La perturbazione nonlineare coerente dei soli
ingressi del governor nella policy successiva verifica la composizione entro
1,4e−7 in azione normalizzata. Dimezzando la perturbazione, le differenze RMS
delle matrici sono 0,00038/0,00214/0,00178; ai cambi di ramo dei limitatori la
derivata non è unica, con scarto massimo locale 0,0771. Il primo segmento è
escluso perché l'accelerazione iniziale osservata non coincide con il reset
interno del governor a zero.

## Metodo e limiti

- Si usa la finestra osservativa 13,15–17,95 s, riferita all'inizio del segmento
  (`policy_segment_times[0]`); il campo `time` del trace indica invece il termine.
- Per feature semanticamente corrispondenti si calcola la covarianza entro
  ciascuna policy e poi la media delle tre covarianze. Le scale sono le radici
  della diagonale. Non entrano gli scarti fra le medie delle diverse policy.
- Jacobiano autograd della media, in unità fisiche degli ingressi. La rete
  differenziabile `module.pi` viene verificata identica a `forward_inference`;
  nessun gradiente è applicato ai parametri.
- Differenze centrate allo 0,1% e 1% della dispersione comune verificano la
  linearizzazione su feature continue, con entrambi i valori dentro il range
  marginale registrato. L'errore RMS relativo massimo è inferiore allo 0,07%.
  Restare nel range marginale non certifica il supporto congiunto fisico.
- Non vengono azzerati gruppi di feature. Le derivate ambientali di one-hot,
  contatto ed eventi sono salvate solo come sensibilità formali della rete:
  non rappresentano transizioni FSM realizzabili. Per il clock è salvata anche
  la derivata tangente alla circonferenza sin/cos; non si effettuano sonde finite
  fuori dalla circonferenza.
- Sono riportate sia sensibilità della media grezza sia quelle dell'endpoint
  effettivo, includendo il gradiente locale del clipping. Il clipping è maggiore
  in LEFT: knee 12,8%, ankle 8,8%; RIGHT 7,6%/0,8%; V26 10,2%/0,4% sull'episodio.
- I gruppi aggregati si sovrappongono: non sommare le relative quote. Le
  covarianze e i contributi al primo layer non sono attribuzioni causali.
- Ogni rete è valutata sui propri stati. Target, waveform e traiettorie sono
  differenti; una differenza di Jacobiano non isola l'effetto dei pesi dagli
  stati visitati. Le fasi nei CSV sono quelle del rispettivo clock, e non
  rappresentano appaiamento della stessa condizione di contatto LEFT/RIGHT.
- La deviazione standard gaussiana riportata è ricostruita dai logits, ma qui
  non si campiona rumore. Un test dinamico con impulsi appaiati è necessario
  per misurare guadagno, smorzamento, compensazioni e interazioni della catena.

## Artefatti e riproduzione

- `analyze.py`: caricamento, audit inferenza, Jacobiani, differenze finite e
  proxy del solo endpoint, senza mutazioni di input.
- `metrics.json`: metodo, versioni runtime, contratti verificati e risultati.
- `group_sensitivity.csv`, `feature_sensitivity.csv`, `phase_sensitivity.csv`:
  misure per gruppo, feature e fase del clock.
- `jacobians.npz`: Jacobiani per stato e azione, osservazioni in ordine comune,
  scale, covarianza e matrici del proxy endpoint.
- `input_hashes.json`, `verification.json`: provenienza e verifica immutabilità.
- `analysis.log`: log dell'esecuzione; il warning RLlib è di deprecazione.
- `governor_feedback.py`, `governor_feedback_metrics.json`,
  `governor_feedback_matrices.npz`, relativo log, manifest input e verifica:
  addendum del percorso reference, senza dinamica OpenSim.

Dal repository, con l'ambiente Python OpenSim/RLlib disponibile:

```sh
python "Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/feedback/analyze.py"
python "Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/feedback/governor_feedback.py"
```

L'analisi usa `Path` e rilocalizza i percorsi assoluti storici tramite il
componente `Trajectory Generator`. Non è stata eseguita una verifica Windows.
