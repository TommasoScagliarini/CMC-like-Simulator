# Ristrutturazione di commands.txt: comandi standard in evidenza

**Data**: 2026-06-10
**Contesto**: baseline_MLP — usabilità della documentazione operativa

---

## Problema

`Trajectory Generator/baseline_MLP/commands.txt` era diventato inutilizzabile:
cresciuto per stratificazione storica in 12 sezioni numerate in ordine
cronologico di introduzione (non per utilità), con:

- i comandi **standard** (training ibrido §11b/§11c e relativo rollout)
  sepolti in fondo al file, dopo setup, smoke e varianti superate;
- tre varianti di training **non-ibride superate** (§6, §7, §11) e un rollout
  non-ibrido (§8) duplicati dei comandi standard a meno di un flag;
- commenti ormai **falsi** (es. "Checkpoint ogni 5" accanto a comandi con
  `--checkpoint-every 1`) e note storiche ridondanti già presenti nei report;
- note critiche (es. `--no-capture-output`) sparse tra i blocchi;
- esempi con path non più allineati alle convenzioni correnti.

## Soluzione

Riscrittura completa del file in **8 sezioni ordinate per uso**, con i comandi
standard in testa e tag maiuscoli grep-abili:

- **Header "REGOLE D'ORO"** (5 punti): `conda run --no-capture-output`
  obbligatorio; convenzione path relativi `runs\...` → `Trajectory
  Generator\runs` (unificata oggi per `--output-dir`, `--checkpoint`,
  `--resume-from`); obbligo di coerenza flag inference↔training; default
  `action_mode=absolute` e incompatibilità dei checkpoint delta; comportamento
  barra live TTY/non-TTY. Rimando al README per i razionali.
- **A) TRAINING STANDARD (ibrido)**: `[TRAIN-STD-WIN]` (12 worker overnight),
  `[TRAIN-STD-MAC]` (5 worker arm64), `[TRAIN-VERIFY]` (verifica end-to-end
  ~2 min, la configurazione usata nella validazione di oggi); nota di una riga
  per la variante non-ibrida (rimuovere `--online-grf-applied-side left`);
  legenda degli output prodotti.
- **B) INFERENCE STANDARD**: `[ROLLOUT-STD]` (con `--record-outputs` per
  `visualize.py`, con avviso sulle righe diagnostiche `[Recruit]`) e
  `[ROLLOUT-FAST]` (solo metriche); riga per i checkpoint legacy
  (`--action-mode delta --grf-mode prescribed --no-online-grf-observation`).
- **C) RESUME**: regola ("stessi flag del training + `--resume-from` +
  `--iterations` al nuovo target") + esempio completo copy-paste che estende
  il run standard da 40 a 60 iterazioni.
- **D) TensorBoard** (incluso prerequisito `setuptools<81`), **E) Reward
  custom**, **F) Verifiche rapide** (smoke import/env, py_compile, tiny
  train/rollout, validazione 50 iter storica), **G) Diagnostica/gate
  onlineGRF** (watchdog self-test, gate sensor/active), **H) Setup ambiente**
  (una tantum, spostato in fondo).

## Strategia

1. **Ordinare per frequenza d'uso**, non per cronologia: standard → resume →
   strumenti → verifiche → diagnostica → setup.
2. **Tag grep-abili** (`[TRAIN-STD-WIN]`, `[ROLLOUT-STD]`, ...) per trovare un
   comando al volo.
3. **Comandi sempre su riga singola completa** (il file è una fonte
   copy-paste: la leggibilità viene dalla struttura, non dallo spezzare i
   comandi).
4. **Nessuna conoscenza persa**: ogni comando unico è stato conservato; le
   varianti duplicate sono coperte da una nota esplicita di delta (un flag);
   le note storiche restano nei report.
5. **Allineamento allo stato validato oggi**: forme `runs\...` ovunque (valide
   dopo il fix di risoluzione path), commenti obsoleti rimossi,
   `progress_display.py` aggiunto alla lista `[COMPILE]` (mancava).

## File modificati

```text
Trajectory Generator/baseline_MLP/commands.txt   (riscrittura completa)
```

Nessuna modifica al codice, al simulatore root o al plugin C++ SEA.

## Verifiche eseguite

- Il README non referenzia numeri di sezione del vecchio file (solo rimandi
  generici a `commands.txt`): nessun riferimento rotto.
- Inventario incrociato vecchio→nuovo: tutti i comandi conservati o coperti
  dalla nota di variante; nessun comando unico perso.
- I comandi standard riflettono configurazioni **eseguite e validate oggi**:
  `[TRAIN-VERIFY]` è il Run A della validazione training; `[ROLLOUT-STD]` con
  forma `runs\...` è il Run I-A post-fix della validazione inference; il
  comando `[RESUME]` usa la forma relativa verificata in regressione.

## TODO

Nessun TODO nuovo introdotto da questa attività.
