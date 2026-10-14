# Diagnosi del completamento con rumore controllato

| Scala rumore | Episodi completi | Passi medi | Return medio |
|---|---:|---:|---:|
| 0 | 1/1 | 500.0 | 222.15 |
| 0.5 | 5/5 | 500.0 | 284.20 |
| 1 | 5/5 | 500.0 | 231.84 |

Cinque semi per condizione stocastica: risultato esplorativo, non prova di equivalenza statistica.
Il checkpoint resta fisso; durante il training la policy cambia e gli episodi possono attraversare aggiornamenti.
Confronto RMSE aggiuntivo sul prefisso comune di 500 passi.
Sono verificati gli stessi campioni gaussiani e le stesse osservazioni/medie iniziali per ogni coppia.
