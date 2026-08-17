# Diagnosi: perché l'adattamento di luglio riuscì e quello V26 si blocca — proposta di retraining imitativo nativo

Data: 2026-08-17

## Problema

Dopo cinque lineage one-shot (V12R13→R17) il warm-start adattato a V26
supera stabilmente il caso storico `+0.20` ma resta fragile su altri start
(`−0.20` puro muore a step 71; seed 127 a step 202). La domanda posta
dall'utente: perché la *stessa procedura* — adattare la baseline imitativa
per usarla come warm start dell'architettura ex-novo — funzionò a luglio
(risultati in `plot/07_15_2026_1_exnovo_50`) e ora no? Quali sono le
differenze tra il vecchio adattamento e quello corrente?

## Analisi (la risposta alla domanda)

**Non è la stessa procedura, perché non è la stessa trasformazione.**
L'adattamento di luglio attraversava un cambio di *dominio GRF*
(prescribed → online/ibrida) con la semantica degli **eventi di fase
identica** su entrambi i lati. L'adattamento corrente attraversa un cambio
di **semantica delle osservazioni** (routing eventi legacy → detector
binario V26/heel-qualified): le feature 10:24 non significano più la stessa
cosa che nei dati con cui H0 fu addestrato — in particolare gli stati di
timeout del legacy non esistono in V26 (l'alias documentato nella forense
R9/R10).

Le tre condizioni favorevoli di luglio, tutte cadute:

| Condizione | Luglio (riuscito) | Ora (V12) |
|---|---|---|
| Semantica osservazioni teacher/runtime | identica (legacy su entrambi i lati) | diversa (legacy → V26) |
| Teacher nel dominio target | **vivo** (interrogabile sugli stati realmente visitati) | assente: H0 diretto su V26 cade (R11); il tape congelato etichetta solo il proprio supporto → label off-policy ai punti di divergenza |
| Margine fisico | ampio (pilot50 stabile col proprio rumore) | sub-millimetrico (teacher sul tape: picco 24,3/25 mm; ogni rollout 23–25 mm) |

Sintesi: a luglio il problema era «adattamento di dominio *con*
supervisione»; oggi è «recovery *fuori supporto* senza teacher» — un
problema qualitativamente più duro, amplificato dal margine: un residuo di
imitazione ~0,005 in spazio azione, innocuo a luglio, oggi produce
escursioni di penetrazione da 1–2 mm nei singoli eventi di caricamento —
quanto basta per varcare il limite terminale dei 25 mm.

Evidenza a supporto (tutta già persistita): probe R11 (H0 puro su V26 cade
a 209 step), run esplorativa 16/08 (H0 collassa in 0,4–1,7 s su tutti gli
start durante il sampling), verdetti R16/R17 (per-start, per-bacino), probe
di capacità (W512≈W256; fresh-init dimezza la coda → il bacino di
inizializzazione domina la parte riducibile dell'errore).

## Soluzione proposta (dall'intuizione dell'utente)

**Retraining imitativo nativo su V26**: rieseguire la ricetta imitativa di
giugno (PPO su *reward* imitativa di tracking — non behavior cloning) con
`binary_active` V26 attivo. Punto strutturale che la rende la via più
pulita: **una reward non ha bisogno di label** — supervisiona qualunque
stato la policy visiti, in qualunque impianto. Il tetto delle label (che
affligge la distillazione da tape) non si applica; la baseline risultante
condivide *per costruzione* la semantica V26 col runtime ex-novo — la vera
condizione di luglio, non un'approssimazione; e la policy impara da subito
dentro il vincolo dei 25 mm invece di ereditare una navigazione legacy a
24–25 mm.

Costi/prerequisiti dichiarati:

1. **Tolleranza operativa in training** (modifica runtime, richiede
   autorizzazione): oggi l'adapter V26 attivo solleva un'eccezione fatale
   sul worker per un evento invalido (fail-closed da qualification); una
   policy a inizio training produce gait degradati → il training
   collasserebbe (visto empiricamente nella run esplorativa). Serve la
   semantica «evento invalido = terminazione episodio con end_reason
   contato» in modalità training — analoga alla terminazione per
   penetrazione.
2. **Calcolo**: ricetta congelata di giugno (100 iterazioni, reward
   grfsoft/knee1/ankle2) → ~1 giorno di training + coda della catena di
   luglio (critic warmup → qualifica pura → checkpoint-zero → morphology →
   comando di training).
3. **Rischio residuo**: il nuovo impianto può spostare gli equilibri della
   reward (possibile un retry di tuning) — rischio da «ricetta nota in
   condizioni nuove», non da problema aperto.

## Alternative documentate (nel cassetto)

- **V12R18 fresh-init W256** (distillazione, matrice piena): leva rapida
  (~3 h) supportata dal probe di capacità, ma non dissolve il tetto delle
  label — attacca il sintomo.
- **Causal teacher v9 come teacher vivo**: il meccanismo che generò i tape
  sicuri (stage `causal_teacher` PASS nella lineage v9) potrebbe etichettare
  stati arbitrari su V26; da verificare con analisi read-only se sia
  invocabile fuori replay o vincolato alle traiettorie prescritte. Se
  invocabile: rompe il tetto delle label per la via della distillazione.

## Chiarimento terminologico (seconda domanda dell'utente)

«Teacher vivo nel dominio V26» = un meccanismo che, dato *qualunque* stato
visitato dallo studente (inclusi quelli di divergenza), restituisce
l'azione corretta lì — in contrapposizione al tape congelato, che risponde
solo per i propri 3.000 stati registrati. A luglio esisteva (H0
interrogabile same-state); su V26 non esiste in forma diretta.

## File modificati

Nessun file di produzione modificato in questa milestone: analisi e
decisione. Artefatti di riferimento: `v12r17/diagnostics/` (probe di
capacità, hash nel report del 17/08), run root R13–R17, run esplorativa
16/08.

## Test e verifiche

- diagnosi fondata esclusivamente su evidenze persistite e hash-bound
  (probe R11, run esplorativa, verdetti R16/R17, probe di capacità);
- nessuna one-shot consumata da questa milestone.

## TODO

- [ ] Decisione dell'utente sul retraining imitativo nativo V26
  (raccomandato), incluse: autorizzazione alla modifica runtime
  «invalid event → terminazione episodio» in modalità training.
- [ ] In caso di via libera: (i) modifica runtime + test, (ii) riesecuzione
  ricetta imitativa di giugno con V26 attivo (monitorata), (iii) catena di
  luglio sul risultato fino al comando di training ex-novo + corridor.
- [ ] Alternative R18-fresh e causal-teacher: mantenerle documentate, non
  eseguirle senza autorizzazione.
- [ ] Daily 2026-08-16 e 2026-08-17 da consolidare al prossimo `end_day`.
