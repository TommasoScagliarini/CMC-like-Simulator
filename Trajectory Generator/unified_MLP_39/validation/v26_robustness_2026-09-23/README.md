# V26 nella 39D: ordine aritmetico e robustezza — 23 settembre 2026

Esperimenti autorizzati dall'utente: «svolgi i primi 2 e poi valutiamo».
Nessun training, modifica della FSM v3 o aggiornamento dei checkpoint.

## Esito verificato

Entrambi gli esperimenti conclusi. Il controllo dell'ordine originale
riproduce le 500 azioni storiche e i tre STO di cinematica, coppie SEA e GRF
byte per byte. I dieci rollout stocastici V26 completano tutti l'episodio:
5/5 con rumore originale e 5/5 dimezzato; best72 archiviata completa 3/5 e 4/5.
Lo screening usa cinque seed e uno start: non dimostra superiorità generale.

Sul prefisso comune di 303 passi la std media caviglia a scala 1 è 0,5154
per V26 e 0,7656 per best72. La differenza di rumore effettivo, osservata sulle
rispettive traiettorie, è un possibile contributo da isolare ulteriormente.

Risultati in `all_metrics.json`; verifica input in `final_verification.json`.
Il [report completo](<../../../../reports/user/2026-09-23_v26_ordine_aritmetico_e_robustezza.md>)
riporta metodo, risultati e limiti. Grafici nel root `plot/09_23_2026_2/` e
`plot/09_23_2026_v26_robustness_comparison/`.

## 1. Controllo deterministico dell'ordine dei calcoli

Stesso ambiente RIGHT del test precedente, stesso reset, 500 passi, stessa
reward, filtri e SEA. L'actor è quello storico originale: un adattatore inverte
la permutazione degli ingressi unified39 prima del forward. Restano quindi
invariati ordine delle colonne dei pesi e ordine delle somme floating point.

Il preflight confronta separatamente 500 inferenze con le azioni archiviate:
la riproduzione è esatta. Nel rollout si mantiene anche il controllo fra
logits originali e logits della rete con pesi permutati sugli stessi stati.
La differenza rispetto al precedente rollout con colonne permutate è l'ordine
aritmetico, non il target né la versione FSM.

## 2. Dieci rollout stocastici

- Policy: export V26 convertito del primo test, con colonne permutate. La scelta
  è fissata prima dell'esito del controllo deterministico e documentata nel
  protocollo. Non si seleziona retroattivamente il percorso più favorevole.
- Seed ambiente sempre 123; seed rumore 123–127; scale 1 e 0,5.
- Stessa funzione `completion_noise_rollout.paired_selector` usata per best72:
  generatore Torch CPU indipendente, nuovo campione gaussiano per ogni passo,
  `azione = media + std_appresa * z * scala`, clipping soltanto nell'ambiente.
- Stessa sequenza standard normale fra scale e fra policy, verificata sui
  prefissi effettivamente registrati. La std appresa rimane specifica della
  policy e dello stato: non si impone identica ampiezza assoluta del rumore.
- Quattro processi di simulazione al massimo, ciascuno limitato a un thread,
  come nella valutazione best72. Ogni caso ha un watchdog indipendente.
- Registrati anche gli STO nei nuovi casi; i vecchi stocastici best72 hanno
  trace e statistiche SEA, ma non STO. Le metriche confrontate usano soltanto
  segnali disponibili con la stessa definizione in entrambi gli insiemi.

Un episodio è completo se arriva a 500 passi con `episode_time_limit`.
`grf_penetration_hard` può lasciare un ultimo stato parzialmente integrato con
timestamp ripetuto e penetrazione diagnostica zero: quel valore non significa
assenza di penetrazione. La riga è conservata per reward e classificazione del
fallimento, esclusa dalle metriche cinematiche dei passi completi. Errori
tecnici/watchdog non sono trattati come normali terminazioni fisiche.

## Esecuzione e analisi

Il codice riusa `v26_weight_transfer_2026-09-23/probe.py` senza modificarlo:
solo adattamenti locali al processo per selettore, osservatore del terminale
e metadata della valutazione. Le verifiche del contratto e della funzione
restano attive. Il normale percorso LEFT di produzione non viene modificato.

```text
python "Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/case_run.py" --preflight
python "Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/case_run.py" --case deterministic_original_order --order original --scale 0 --seed 123
python "Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/analyze.py" --deterministic-only
python "Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/run_suite.py"
python "Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/analyze.py" --all
```

I runner rifiutano di sovrascrivere i risultati. Stato e log della batteria:
`unified_MLP_39/runs/plans/V26_robustness_2026-09-23/`. Rollout:
`unified_MLP_39/runs/rollout/V26_robustness_2026-09-23/`.

L'analisi produce `deterministic_metrics.json`, i risultati stocastici, ricevute
dei plot e hash degli input. Tutte le immagini vanno nel root `plot/`.

Cinque seed per condizione costituiscono uno screening. Il confronto con
best72 cambia insieme policy e compito imitativo; non isola l'effetto del solo
target sinistro e non dimostra equivalenza statistica fra training.
