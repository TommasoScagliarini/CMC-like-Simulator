# 2026-10-15 — Pulizia dei profili GRF online

Eliminati 19 JSON da `online_grf_profiles/`: i 14 classificati come inutili al
workflow corrente e i cinque archiviabili esplicitamente indicati dall'utente.
Questi ultimi sono marker-inferred, physical-basis, row, rocker e rocker-soft.
Sono stati rimossi 257.372 byte di profili, senza creare un archivio duplicato
dei JSON nella working tree.

## Strategia e conservazione

Prima di ogni cancellazione e stato verificato lo SHA della classificazione e
confrontato il contenuto con il blob Git. Tutti i 19 file erano identici al
commit `dba42388c32a441a5543f20881612a52781d7a99`, dal quale sono recuperabili
esattamente. La cancellazione ha riguardato esclusivamente i 19 nomi previsti.
L'elenco completo, con byte, hash e commit di recupero, e nel
[receipt della rimozione](../cleanup/2026-10-15_grf_profiles_removal/removal_receipt.json).

Restano cinque profili, byte-identici alla baseline:

- `AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`:
  profilo principale dei preset recenti e di best47.
- `AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`: detector corrente.
- `AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json`: alias ancora
  richiesto da default baseline, oracle e validatori.
- `AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO_v4_mesh_experimental.json`:
  candidato di validazione, non richiesto dal workflow attuale.
- `AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json`: baseline di
  validazione legacy, non richiesta dal workflow attuale.

Aggiornati `online_grf_profiles/README.md` e
`Trajectory Generator/baseline_MLP/README.md`, rimuovendo i collegamenti ai
file cancellati dall'inventario corrente e correggendo il vecchio default
physical-basis e la descrizione del contatto ibrido. Nessun sorgente runtime,
preset, plugin, modello o checkpoint e stato modificato da questa pulizia.

## A cosa servono i due profili di validazione

**Nessuno dei due e fondamentale per la simulazione o il training attuali, ne
per best47.** Servono alle specifiche verifiche che li selezionano.

Il V4 mesh sperimentale conserva un candidato scartato per il detector:
viene usato in test geometrici, diagnostica e replay dei confronti storici.
Il builder `validation/build_two_sensor_mesh_profile_v4.py` e stato eseguito
su un output temporaneo: tutti i campi fuori dai metadati sono esattamente
identici all'originale. Differiscono soltanto quattro campi di provenienza
(percorsi di mesh/modello/setup e SHA del setup), gia legati alle precedenti
riorganizzazioni. Il profilo e quindi rigenerabile come geometria e parametri
runtime, ma il JSON completo rigenerato non conserva l'hash storico.

`online_sensor_basis` resta un riferimento per uno smoke test opzionale,
l'esempio di confronto Python/plugin e la vecchia calibrazione su IK. Il suo
uso come supporto fisico attivo era problematico, con penetrazione storica di
circa 102 mm. Lo smoke contiene gia un percorso di setup precedente alla
riorganizzazione dei modelli e richiede adattamento per il replay.

Il sensor-basis e ricalibrabile tramite
`validation/calibrate_online_grf_basis.py`, ma non e garantita la ricostruzione
dello stesso JSON: servono geometria iniziale, IK, GRF, finestra e opzioni
originali e un ambiente numerico coerente. Il calibratore corrente e anche
diverso da quello originario. Non e stata eseguita una nuova calibrazione.

Entrambi i profili conservati sono stati confrontati con Git e risultano
byte-identici al commit indicato sopra. Per un replay che richiede lo SHA
originale, il ripristino da Git e esatto; una nuova calibrazione non e un
sostituto equivalente.

Esempio di ripristino di un file eliminato, da eseguire solo se serve:

```shell
git restore --source=dba42388c32a441a5543f20881612a52781d7a99 -- online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_row.json
```

## Verifiche

Due subagenti hanno controllato separatamente integrita e rigenerabilita.

| Controllo | Esito |
| --- | --- |
| File eliminati | Esattamente 19; tutti recuperabili byte per byte da Git |
| Profili rimasti | Esattamente cinque; tutti caricati dal parser runtime |
| SHA di 56 file tra profili, codice, configurazioni e plugin | Invariati |
| 31 riferimenti GRF in 22 YAML, incluso best47 | Risolti e caricati |
| Contratti runtime MLP | 37/37 PASS |
| Checkpoint best47 reale, setup legacy e due passi OpenSim | PASS; checkpoint e configurazione salvata invariati |
| Rigenerazione V4 su file temporaneo | PASS per geometria e parametri runtime; quattro differenze nei soli metadati |
| Ricerca dei 19 nomi nel codice/config operativi | Nessun consumatore specifico trovato |
| `git diff --check` | PASS |

Le prove native sono macOS arm64 e lo smoke e breve. Non e stata eseguita una
nuova qualificazione scientifica completa o una sessione nativa Windows.

I manifest storici che fotografano l'intera cartella conservano i vecchi nomi
e hash e non sono stati riscritti. I relativi replay richiederanno di
ripristinare i profili eliminati. Nell'audit operativo restano 38 citazioni
dei nomi rimossi in due inventari storici; non sono selezioni di profilo dei
run attuali.

Evidenze: [ricevute e log](../cleanup/2026-10-15_grf_profiles_removal/README.md).
Nessun commit o push eseguito.
