# AB06_SEASEA - GRF, gait cycle e plotter

Data: 2026-05-13

## Sintesi

E' stata analizzata la causa dei plot gait-cycle anomali generati per
`AB06_SEASEA_Threadmill`.

Il problema non era nel mapping force plate/piede e non era introdotto dal
converter `.mat -> .mot`: nel file sorgente EPIC delle force plate sono presenti
micro-contatti GRF sopra la soglia storica `20 N`. Il detector precedente
interpretava ogni crossing rising come heel-strike, generando gait cycle non
fisiologici da pochi millisecondi.

La soluzione implementata mantiene la soglia storica `20 N`, ma aggiunge due
filtri temporali robusti:

- durata minima del contatto GRF;
- durata minima del gait cycle.

Questo preserva la compatibilita con i dataset gia funzionanti e scarta solo
eventi fisicamente impossibili.

## Problema osservato

Nei plot:

- `03_gaitcycle_torque_angle_power.png`;
- `04_gaitcycle_joint_velocity_power.png`;

si osservavano:

- torque-angle plot molto schiacciati;
- bande di deviazione standard molto larghe;
- chattering tra `13 s` e `14 s` nei segnali temporali.

Il file iniziale `results/sim_output_gait_events.csv` conteneva per il lato
sinistro 14 cicli, molti dei quali erano troppo brevi:

```text
0.0298 s
0.0140 s
0.0140 s
0.0141 s
0.0290 s
0.0069 s
```

Questi segmenti venivano normalizzati a `0-100%` gait cycle come se fossero
passi completi, distorcendo media e deviazione standard.

## Verifica sul file GRF sorgente

E' stato ispezionato il file sorgente:

```text
models/AB06-raw/10_09_18/treadmill/fp/treadmill_01_01.mat
```

Il file contiene una table `data` da `143056 x 19`, con intervallo:

```text
11.990 - 155.045 s
```

Nel sorgente EPIC sono gia presenti gli stessi micro-crossing a `20 N` visti nel
`.mot` convertito.

Per `Treadmill_L_vy`:

```text
13.430-13.560 s:
min = -3.214 N
max = 44.837 N
mean = 6.424 N
```

Crossing rilevati nel sorgente:

```text
13.446967
13.476794
13.490773
13.504777
13.518852
13.547862
13.939984
13.946871
...
```

Quindi gli spike non sono un errore del writer `.mot`; sono artefatti a bassa
forza presenti nel dato raw.

## Eventi EPIC gcLeft/gcRight

Sono stati ispezionati anche:

```text
models/AB06-raw/10_09_18/treadmill/gcLeft/treadmill_01_01.mat
models/AB06-raw/10_09_18/treadmill/gcRight/treadmill_01_01.mat
```

I fronti puliti di `HeelStrike` nel range simulato sono:

```text
gcLeft  HS: 13.965  15.635  17.190  18.745  20.305
gcRight HS: 13.180  14.800  16.400  17.970  19.545
```

Questi eventi confermano che i micro-crossing a `20 N` non sono veri passi.

## Implementazione

File modificati:

- `config.py`
- `output.py`

Sono stati aggiunti in `SimulatorConfig`:

```python
grf_contact_threshold_n = 20.0
grf_min_contact_duration_s = 0.05
grf_min_cycle_duration_s = 0.30
```

Il detector in `output.py` ora:

1. rileva i contatti sopra soglia;
2. calcola la durata del contatto;
3. scarta i contatti piu brevi di `grf_min_contact_duration_s`;
4. costruisce i cicli heel-strike -> heel-strike;
5. scarta cicli piu brevi di `grf_min_cycle_duration_s`.

Il CSV eventi ora include anche:

```text
cycle_duration_s
contact_duration_s
min_contact_duration_s
min_cycle_duration_s
```

## Eventi rigenerati

E' stato rigenerato:

```text
results/sim_output_gait_events.csv
```

usando:

```text
models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot
```

Risultato:

```text
left:  4 cicli
right: 4 cicli
```

Durate left:

```text
1.669
1.548
1.553
1.571 s
```

Durate right:

```text
1.616
1.603
1.570
1.573 s
```

Il CSV rigenerato e coerente con i fronti puliti `gcLeft/gcRight` del dataset.

## Plot rigenerati

Comando eseguito:

```bash
python plot/plotter.py
```

Output:

```text
plot/13_05_2026 - 1
```

Il plotter ha caricato correttamente:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_setup.xml
models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot
```

e ha trovato:

```text
left: 4
right: 4
```

Nei nuovi plot:

- i torque-angle plot non sono piu schiacciati dai micro-cicli;
- la deviazione standard di angle/velocity/power e molto piu realistica;
- rimane solo il warning sull'overlay healthy assente, atteso per il bundle AB06.

## Nota sul chattering

La correzione attuale migliora il riconoscimento dei gait cycle e quindi i plot
normalizzati.

Il chattering temporale tra `13 s` e `14 s` resta nei risultati gia simulati,
perche le micro-GRF erano comunque presenti nelle `ExternalLoads` applicate alla
dinamica durante la simulazione.

Per rimuovere quel chattering dalla simulazione stessa servirebbe un secondo
intervento sui dati GRF usati come carichi esterni, ad esempio:

- azzerare le GRF sotto soglia quando il contatto non supera la durata minima;
- oppure filtrare/segmentare i contatti GRF prima della run CMC-like;
- poi rilanciare la simulazione.

## Stato finale

La soluzione implementata e backward-compatible:

- la soglia storica resta `20 N`;
- i dataset gia funzionanti non dovrebbero perdere eventi validi;
- AB06 non genera piu gait cycle spurii da pochi millisecondi;
- i plot prodotti il `2026-05-13` sono quelli da usare per valutare i cicli
  gait puliti della simulazione corrente.
