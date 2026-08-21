# Verdetto fresh run june-equivalent nativo V26: equivalenza statistica raggiunta

Data: 2026-08-20

## Esito

`MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`: 100/100
iterazioni in 14,4 h, **0 errori off-grid**. L'esperimento pulito chiesto
dall'utente — la ricetta imitativa di giugno con l'unica variabile del
detector binario V26 (più compensazioni dichiarate) — dopo l'audit di
equivalenza del mattino (report 2026-08-20) e lo smoke di decollo.

## Confronto formale coi criteri di giugno

| Metrica (ultime 20 iter) | Giugno | Fresh V26 | Rapporto |
|---|---:|---:|---:|
| Return medio | 268,6 | 257,4 | **96%** |
| Len episodi | 476 | 494 | **104%** |
| Return per step | 0,564 | 0,522 | 93% |
| Best assoluto | 293,6 (iter 94) | 271,0 (iter 87) | 92% |

Soglie fisse (return/len): iter 10: 26/295 vs 47/428; iter 25: **162/446 vs
127/389** (fresh avanti); iter 50: 226/481 vs 232/456; iter 75: 259/500 vs
270/501; iter 100: 247/451 vs 273/478. Le curve si incrociano più volte:
le differenze finali (−4% return, +4% len) sono dentro la variabilità di
un singolo seed (giugno stesso oscilla ±8% tra iter 94 e 100).

**Verdetto sul criterio «stesso risultato»: soddisfatto entro la
variabilità di run singolo.** L'ipotesi diagnostica è confermata: il gap
della vecchia lineage non era il detector V26 ma i bug (probe, crash
storm) e le derive di configurazione (soglie penetrazione 12/17,
storia sporca dei resume).

## Mix terminazioni (contatori onesti)

- penetrazione: **198 vs 422 di giugno** (metà!) — la policy nativa
  gestisce il contatto meglio della baseline storica;
- timeout swing: 113 vs 0 — il costo onesto dell'heel-qualification V26
  (giugno concedeva HS liberamente); ~1/iterazione, non strutturale;
- eventi invalidi terminali: 0 (assorbiti come drop, by design);
- 43 morti worker totali, 39 nelle prime 2,5 h: guardia dura 30 mm vs
  terminazione 28 mm (2 mm di margine) nella fase caotica iniziale +
  AIR-latch — diagnosi nel report tempi; fix differito su decisione utente.

## Tempi

14,4 h vs 11,8 di giugno: ~2 h di round-extra da morti worker (transitorio
iniziale), ~5% per-step del detector V26, resto fisiologico. A regime
7,7–8,3 min/iter contro 7,0.

## Artefatti

- run: `runs/training/MLP_imitation_native_v26_08-20-2026_june_equiv_100iter/`
  (rl_module_best = iter 87);
- rollout + 7 plot: in produzione al momento della scrittura (cartella
  datata `plot/08_20_2026_*`).

## Addendum — coppia SEA positiva alla caviglia (fig. 01, domanda utente)

Osservazione: componenti positive fino a +10 N·m (giugno: ~+1). Analisi:

1. **Config verificata identica anche su questo asse**: il penalty
   anti-flip (`grf_ankle_moment_flip_weight 0,1`, tolleranza 8 N·m, gate
   forza 50 N) è uguale nei due resolved yaml. Scoperta collaterale: a
   giugno il codice del penalty era **uncommitted** (committato solo il
   13/07 in `bdbf99c`) — il run di giugno girò su tree sporco; il resolved
   yaml resta la ground truth (ed è ciò che l'audit ha usato).
2. **Il penalty tollera per design fino a 8 N·m**: entrambe le policy
   vivono nella stessa banda ammessa (giugno sceglieva ~+1, il fresh
   +2–4 con un burst a +10 — penalizzato solo oltre 8, brevemente).
3. **Localizzazione empirica** (dai .sto del rollout): il **100%** dei
   campioni con τ>+3 N·m cade **entro 0,5 s da un heel strike sinistro**
   (mediana 0,25 s), sotto carico (93% in contatto, ~170 N) — è la
   finestra di loading response post-HS. Firma della heel-qualification
   V26: serve contatto di tallone stabile e confermato → la policy impara
   a premere il tallone in caricamento. Giugno (HS concessi liberamente)
   non aveva l'incentivo. **Adattamento indotto dalla variabile
   sperimentale dichiarata, non deriva di configurazione.** Nota: un
   momento dorsiflessorio moderato in loading response è anche il pattern
   fisiologico (controllo della discesa del piede).

Opzioni: (a) accettare come adattamento legittimo V26 (raccomandata);
(b) stringere `grf_ankle_moment_flip_tau_tol_nm` 8→2-3 e/o alzare il
peso — modifica deliberata della reward rispetto a giugno, in tensione
con la qualification. Decisione utente.

## TODO

- [ ] Rollout + 7 plot del best e confronto visivo con `06_24` (in corso).
- [ ] Decisione differita: guardie di processo → terminazioni pulite
  (30 mm e AIR-latch) per campionamento esatto 4096.
- [ ] Con la baseline nativa V26 stabilita: catena di luglio (critic
  warmup → qualifica → checkpoint-zero → morphology → ex-novo + corridor).
- [ ] Daily 2026-08-18/19/20 da consolidare al prossimo `end_day`.
