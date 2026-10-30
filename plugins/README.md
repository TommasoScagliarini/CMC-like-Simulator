# Plugin compilati

```text
plugins/
├── SEA/
│   └── libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib
└── online_grf_contact/
    └── libOnlineGRFContact.dylib
```

Questa cartella resta nella radice condivisa del workspace. Il core Python
che carica le librerie è in
[`CMC-like Simulator/`](<../CMC-like Simulator/README.md>); i basename dei
plugin continuano a partire da `plugins/`.

Le due librerie macOS arm64 sono quelle correnti, conservate byte per byte
durante la pulizia del 2026-10-15. Le sette varianti SEA e copie di backup
precedenti sono state eliminate dalla working tree. Nessun symlink conserva
la vecchia struttura piatta.

| Componente | Basename del caricatore | File Windows x64 previsto |
| --- | --- | --- |
| SEA PI | `plugins/SEA/SEA_Plugin_BlackBox_mCMC_impedence_ff` | `SEA/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` |
| Contatto GRF | `plugins/online_grf_contact/OnlineGRFContact` | `online_grf_contact/OnlineGRFContact.dll` |

Il checkout Mac contiene i due `.dylib`; le DLL Windows correnti non erano
presenti prima della pulizia. Il loro percorso e predisposto, senza sostituirle
con vecchie versioni. Il loader usa basename senza `lib` e senza estensione:
OpenSim applica la convenzione del sistema operativo.

Il resolver centrale accetta anche i precedenti percorsi piatti dei due
plugin correnti, compresi gli indirizzi salvati nei setup/checkpoint e i
separatori Windows. Non rimappa le vecchie implementazioni SEA al PI corrente:
hanno una dinamica diversa. I percorsi esterni esistenti e le copie archiviate
annidate conservano il proprio significato.

I binari restano ignorati da Git; README e segnaposto permettono di versionare
la struttura. Per recuperare i vecchi binari servono la copia esterna o i
rispettivi artefatti originali. I record e gli hash storici non vengono
riscritti: i replay che aprono direttamente le vecchie posizioni richiedono
un checkout/copia precedente.

Sorgenti SEA e compilazione: [tools/README.md](../tools/README.md).
Sorgenti e compilazione GRF:
[online_GRF/sources/README.md](../online_GRF/sources/README.md).
