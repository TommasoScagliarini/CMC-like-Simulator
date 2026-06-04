# Knowledge base della letteratura RL/protesi (framework per LLM)

Data: 2026-06-04

## Problema

Era disponibile, fuori dal repo, una cartella di ricerca curata
(`/Users/tommy/Desktop/report opensim+rl/`) con due deep-research paralleli
(`GPT/` e `Gemini/`) sul tema del progetto: sostituire il generatore di
riferimento cinematico sperimentale con un agente neurale di traiettorie per una
protesi transfemorale SEA in OpenSim, con controllo gerarchico.

Obiettivo richiesto: **estrarre le informazioni dai paper e costruire un
framework efficace perché un LLM acceda a quelle informazioni**, con due vincoli
espliciti dell'utente:

1. la knowledge base deve restare **disaccoppiata dal progetto**: non deve
   influenzare TODO, codice, report o memorie;
2. l'estrazione deve essere **neutra** (riassumere i paper), non "applicare al
   progetto".

## Soluzione

Creata una knowledge base in `literature/` (dentro il repo ma **reference-only**,
non referenziata da codice/TODO), strutturata come "second brain" ipertestuale a
caricamento selettivo:

```text
literature/
  README_FOR_LLM.md   # protocollo d'accesso per un LLM + ricetta ri-estrazione
  INDEX.md            # router: 28 voci con id/anno/topics/stato/nota/PDF
  TOPICS.md           # indice trasversale topic -> paper (64 topic)
  notes/Pxx_*.md      # una nota per documento (28 note)
```

Lingua note: inglese (scelta dell'utente). Ogni nota ha frontmatter
machine-readable (`topics`, `keywords`, `pdf`, `pages_read`,
`extraction_confidence`, `related`) e sezioni fisse: TL;DR, Problema/contributo,
Metodo/architettura, Setup, Risultati, **Code/data**, **Notable claims ancorati a
pagina**, **Related notes** (link markdown ad altre note), **Caveat**.

Sono stati estratti **28 documenti**:

- **P01–P25**: i 25 paper (protesi RL, DRL-OpenSim, controllo gerarchico, safe
  RL, reward/energia, POMDP/memoria, RLVR);
- **D01–D02**: documentazione (PyTorch C++ frontend/LibTorch, osim-rl);
- **S02**: la sintesi Gemini. La sintesi GPT (S01) e' stata letta come
  `GPT/deep-research-report.md` e linkata.

## Strategia

- **Gerarchia a caricamento selettivo**: un LLM carica `INDEX.md` (piccolo) ->
  filtra per topic in `TOPICS.md` o per `grep` -> legge solo le note necessarie.
  Cosi' non serve ingerire tutti i paper.
- **Pilot prima dello scale**: estratto un primo paper (P01), validato lo schema
  con l'utente, iterato (aggiunte sezioni *Related notes* e *Code/data*, equazioni
  semplici scritte e pesanti spiegate con puntatore al PDF, claim ancorati a
  pagina, lingua inglese), poi esteso a tutti.
- **Lavoro a lotti** con persistenza immediata su file (estrai testo -> scrivi
  nota -> avanti), per non saturare il contesto.
- **Onesta' sull'affidabilita'**: ogni nota dichiara `pages_read` e
  `extraction_confidence`; i paper lunghi/periferici sono stati letti
  selettivamente e marcati di conseguenza.
- **TOPICS.md rigenerato automaticamente** dai frontmatter delle note, per
  garantire mappa topic->paper accurata e completa.

## File creati

Tutti nuovi, dentro `literature/` (nessuna modifica a file di progetto):

```text
literature/README_FOR_LLM.md
literature/INDEX.md
literature/TOPICS.md
literature/notes/P01_..  ... P25_..  (25)
literature/notes/D01_pytorch_cpp_frontend.md
literature/notes/D02_osim_rl_musculoskeletal.md
literature/notes/S02_gemini_ai_prosthetic_trajectory.md
```

Nessuna modifica a codice, `AGENTS.md`, `TODO_integration.md`, report di progetto
o memorie. I PDF non sono copiati nel repo: restano sul Desktop, le note ne
tengono il percorso assoluto.

## Test / verifiche eseguite

- **Estrazione testo**: nessuna libreria PDF/poppler era installata; installato
  **PyMuPDF** isolato in `/tmp/pdflib` (zero impatto sugli env conda). Estratti 27
  PDF (con conteggio pagine) e 2 HTML (strip dei tag via `html.parser`).
- **Copertura**: 28 note scritte, una per documento; `INDEX.md` tutto marcato
  estratto (✅); `TOPICS.md` rigenerato dai frontmatter -> **64 topic** verificati;
  i link ipertestuali tra note risolvono (filename canonici pre-registrati).
- **Lettura selettiva tracciata**: confidence alta sulla maggior parte; `low` dove
  letto solo abstract+TOC (P17 tesi 205pp; P24 survey 36pp); `medium` per i parziali
  (P14, P19, P23).

## Anomalie / note di integrita' del corpus

- **File mislabeled (P25)**: il PDF "Wrapyfi A Python Wrapper..." contiene in
  realta' un paper diverso (iCub Humanoid Robot Simulator, Tikhanoff 2008). Il vero
  paper Wrapyfi **manca** dal corpus. Segnalato nella nota P25 e in TOPICS.
- **P17**: il file e' la **tesi di dottorato di Berkenkamp (205 pagine)**, non solo
  il paper NeurIPS 2017; letti solo abstract+TOC.
- **RLVR (P23, P24)**: tema "RLVR" e' dominio LLM/world-model video, **periferico**
  al controllo protesico (coerente con quanto nota la sintesi GPT/S01); incluso come
  background sul principio "reward verificabili" + algoritmo GRPO.
- **Lacuna**: nel corpus **non c'e' nessun paper sugli SNN** (la scelta di rete
  spiking del progetto non e' coperta dalla letteratura raccolta).

## Vincoli rispettati

- KB **disaccoppiata**: `README_FOR_LLM.md` istruisce esplicitamente un agente a
  NON modificare codice/TODO/report/memorie sulla base di questi contenuti, salvo
  richiesta separata.
- Estrazione **neutra**: le note riassumono i paper; eventuali collegamenti col
  progetto sono solo descrittivi nei *Caveat/Related*, mai prescrittivi.

## TODO aperti

- Recuperare il vero paper **Wrapyfi** (il file in `paper/7` e' il paper iCub) se
  serve la parte middleware ZeroMQ/YARP/ROS.
- Approfondire, se utile, i documenti letti solo parzialmente: **P17** (tesi
  Berkenkamp, capitolo Lyapunov/region-of-attraction), **P24** (survey RLVR, §3
  algoritmi/reward), e i parziali P14/P19/P23.
- (Solo su richiesta esplicita) produrre una **sintesi trasversale** che attraversi
  le note per tema (es. reward design, observation/POMDP, action space, safety) —
  resta separata dal progetto finche' l'utente non decide diversamente.
- (Solo su richiesta) decidere se/come collegare i finding della letteratura ai
  TODO del progetto: per ora intenzionalmente **non** fatto.
