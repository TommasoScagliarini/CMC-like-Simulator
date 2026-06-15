# Fix righe duplicate nella progress bar del training

## Problema

Durante il training, gli aggiornamenti live della progress bar comparivano
duplicati o concatenati nel log dell'IDE, soprattutto durante i cambi di fase
come `checkpoint_last`, `checkpoint_best` e `algo.train`.

Il problema era esclusivamente cosmetico: training, checkpoint e metriche non
venivano eseguiti due volte.

## Causa

Il percorso TTY di `LiveProgress._write_line()` emetteva due volte la stessa
riga a ogni refresh:

```text
\r<riga con padding>\r<stessa riga>
```

Un terminale tradizionale sovrascrive entrambi i frame nella stessa posizione,
ma alcuni PTY, IDE e raccoglitori di log conservano i carriage return come
frame distinti, mostrando quindi la riga due volte.

## Soluzione

Il refresh TTY ora emette un solo frame:

```text
\r<riga con padding>
```

Il padding resta presente per cancellare l'eventuale coda di una riga
precedente più lunga. Il comportamento delle righe permanenti, della frequenza
di refresh e del fallback non-TTY non è stato modificato.

## Strategia

1. Ispezionata l'integrazione tra monitor del training e `LiveProgress`.
2. Identificata la doppia scrittura nel percorso TTY.
3. Ridotta la scrittura a un solo frame per refresh.
4. Aggiunti test con un finto TTY per verificare il flusso raw prodotto.
5. Verificata anche la transizione da riga live a log permanente e ritorno alla
   riga live.

## File modificati

- `Trajectory Generator/baseline_MLP/progress_display.py`
  - eliminata la seconda emissione della stessa riga durante il refresh TTY;
  - aggiunto un commento sulla compatibilità con IDE, PTY e log collector.
- `validation/test_progress_display.py`
  - aggiunti test di regressione per refresh singolo, padding e log permanente.

## Test e verifiche

Eseguito:

```powershell
python -m unittest validation.test_progress_display -v
```

Risultato: `2` test superati.

La correzione sarà visibile dai processi di training avviati dopo la modifica;
un processo già attivo mantiene in memoria la versione precedente del modulo.
