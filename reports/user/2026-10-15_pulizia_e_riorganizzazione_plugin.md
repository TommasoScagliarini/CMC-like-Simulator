# 2026-10-15 — Pulizia e riorganizzazione dei plugin

Eliminate le sette librerie SEA precedenti e copie di backup non necessarie
al workflow corrente, per 2.153.336 byte. Le due librerie attive sono state
spostate, senza modificarne il contenuto, nella struttura richiesta:

```text
plugins/
├── SEA/
│   └── libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib
└── online_grf_contact/
    └── libOnlineGRFContact.dylib
```

## Strategia e file modificati

Prima delle modifiche sono stati acquisiti gli hash di 72.320 file e tre
simulazioni native di riferimento, AB06/AB07/AB08, cascade+PI per 30 ms ciascuno.
Tre subagenti hanno lavorato separatamente su resolver/caricatore, strumenti
di preparazione e inventario dei plugin nel pilot MLP; uno ha poi eseguito
anche una revisione indipendente del resolver e dei binari.

- `path_resolver.py` espone le directory SEA e GRF e rimappa i vecchi percorsi
  piatti dei soli due plugin correnti, anche salvati con separatori Windows o
  indirizzi di checkout riconosciuti. Il caricamento usa basename OpenSim senza
  prefisso `lib` e senza estensione; le funzioni che trattano file mantengono
  invece il nome fisico richiesto.
- `config.py`, `model_loader.py` e `visualize.py` usano il nuovo layout. Anche
  il caricatore centrale riconosce le vecchie chiamate dei due plugin correnti.
  Vecchie implementazioni SEA, percorsi esterni esistenti e copie archiviate
  annidate non vengono sostituiti con il PI corrente.
- `scripts/run_opensim_sea_pipeline.py`, `tools/build_abx_seasea_pipeline.py` e
  `tools/calibrate_ab06_seasea_markers.py` usano la nuova directory. Sono stati
  rimossi i fallback alle vecchie dinamiche SEA. La calibrazione sceglie ora
  l'estensione appropriata anche su Windows.
- Il pilot di `unified_MLP_39/reward_tuning/` inventaria ricorsivamente le
  librerie nelle sottocartelle e ne verifica ancora gli hash prima di eseguire
  un run. I checkpoint, i preset e lo schema runtime MLP non sono stati riscritti.
- Aggiornati i test interessati, `.gitignore`, `AGENTS.md`, `tools/README.md` e
  `tools/online_grf_contact/README.md`. Aggiunti README e segnaposto dei gruppi
  sotto `plugins/`; le librerie restano ignorate da Git.

Non sono stati modificati C++, leggi di controllo, parametri fisici, modelli,
dati numerici o profili GRF. I sette nomi eliminati, con byte e SHA originali,
sono nel [receipt dei binari](../cleanup/2026-10-15_plugins_cleanup/binary_migration_receipt.json).

## Verifiche

| Controllo | Esito |
| --- | --- |
| Due librerie conservate | SHA e dimensioni identici prima/dopo |
| Librerie eliminate | Esattamente le sette previste; nessun'altra libreria installata |
| Resolver, namespace e caricatore | 16/16 test PASS |
| Runtime MLP e pilot | 52/52 test PASS, inclusa la guardia sugli hash dei plugin raggruppati |
| Pipeline, default ABx e registrazione geometrica | 38/38 test PASS prima e dopo |
| Preflight reale AB06/07/08 | PASS, anche fornendo il vecchio percorso Windows |
| Revisione indipendente | 60 combinazioni di percorsi verificate, piu controllo dei binari |
| Caricamento nativo dai vecchi percorsi piatti | Registrati SEA e forza di contatto GRF dalle nuove directory |
| Cascade+PI prima/dopo, tre soggetti | 48 STO e 52.920 valori inclusi i tempi: identici byte per byte, differenza numerica zero |
| Checkpoint best47 reale dopo la migrazione | PASS: vecchio setup salvato, due passi OpenSim, checkpoint e configurazione invariati |
| Confronto esteso di 72.320 file della baseline | 72.300 identici, 13 modifiche previste e sette rimozioni autorizzate; zero perdite o modifiche inattese |
| `git diff --check` | PASS |

Il confronto esteso include i file tracciati presenti, tutti i file dei modelli
e plugin e i report/evidenze precedenti; non equivale a un inventario di ogni
file ignorato del repository. La documentazione `tools/README.md`, gia non
tracciata, e verificata separatamente nel controllo di consegna.

Le prove native sono macOS arm64 e gli smoke sono brevi; non attestano una
nuova qualificazione scientifica completa o una prova nativa Windows.

## Windows e riproducibilita

Le DLL correnti non erano presenti in questo checkout prima della pulizia.
Le posizioni previste ora sono:

- `plugins/SEA/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`
- `plugins/online_grf_contact/OnlineGRFContact.dll`

Non sono state create DLL rinominando vecchi binari. La vecchia DLL senza
`_ff` era una delle sette librerie eliminate. Le istruzioni di compilazione e
installazione sono aggiornate per entrambi i sistemi.

Le vecchie librerie non erano tracciate da Git: il recupero richiede la copia
esterna segnalata dall'utente o i rispettivi artefatti originali. Durante le
verifiche sono state tenute copie temporanee fuori dal repository, eliminate
dopo i controlli finali. Nessun archivio dei sette binari eliminati e stato
aggiunto al repository.

Manifest, report e script congelati conservano nomi e hash originali. I replay
che aprono direttamente le vecchie posizioni o inventariano tutte le varianti
richiedono la copia precedente; non sono stati riscritti per nascondere gli
effetti della pulizia. Il converter V14 congelato e invariato; per un nuovo
uso va fornito il percorso corrente del plugin.

Evidenze: [inventari, ricevute e log](../cleanup/2026-10-15_plugins_cleanup/README.md).
HEAD resta `dba42388c32a441a5543f20881612a52781d7a99`; nessun commit o push.
