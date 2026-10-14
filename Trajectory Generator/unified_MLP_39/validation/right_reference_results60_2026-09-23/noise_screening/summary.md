# Robustezza stocastica RIGHT39 best58

Cinque seed appaiati, rumore originale, start unico. Metriche fisiche sui primi2.99s comuni.

| Policy | Completi | ≥2cicli | Completi,2cicli,0invalidi | Return medio | RMSE knee | RMSE ankle |
|---|---:|---:|---:|---:|---:|---:|
|RIGHT39 best35|5/5|5/5|1/5|210.49|10.99°|4.55°|
|RIGHT39 iter40|4/5|3/5|1/5|204.18|9.78°|4.61°|
|RIGHT39 best58|4/5|3/5|2/5|208.98|8.82°|3.65°|
|V26 best87 (replay)|5/5|3/5|2/5|259.06|6.36°|2.88°|

Ogni policy conserva la propria deviazione standard: stessi numeri gaussiani non significano perturbazioni assolute identiche. Cinque episodi non provano equivalenza statistica. V26 è il replay dei pesi best87 nel percorso corrente, non una replica completa del software di agosto.

- [Grafico](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/09_23_2026_right60_comparison/noise_paired_penetration.png>)
- [Grafico](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/09_23_2026_right60_comparison/noise_completion_and_tracking.png>)
- [Grafico](</Users/tommy/Documents/CMC-like-Simulator - Claude/plot/09_23_2026_right60_comparison/noise_signed_ankle_torques.png>)
