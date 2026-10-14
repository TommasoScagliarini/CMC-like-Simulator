# Audit offline reward LEFT — 24 settembre 2026

Risultati e interpretazione nel
[report](../../../../reports/user/2026-09-24_audit_reward_left_fasi_contatto_e_clipping.md).
Nessun training, nuova simulazione o modifica alla configurazione attiva.

Riproduzione, dall'ambiente Python del progetto e dalla root:

```bash
python "Trajectory Generator/unified_MLP_39/validation/left_reward_audit_2026-09-24/analyze.py"
python "Trajectory Generator/unified_MLP_39/validation/left_reward_audit_2026-09-24/physics/analyze.py"
python "Trajectory Generator/unified_MLP_39/validation/left_reward_audit_2026-09-24/physics/training_initial_probe.py"
python "Trajectory Generator/unified_MLP_39/validation/left_reward_audit_2026-09-24/sensitivity/analyze.py"
```

Gli script leggono solo gli artefatti precedenti, limitano i thread numerici,
scrivono i risultati in questa directory e le figure in `plot/` della root.
Non istanziano simulatori, policy o learner. Le figure aggiuntive dell'audit
non sostituiscono i sette plot canonici dei rollout originali.

`candidate_penetration_weight_2.json` contiene una sola variazione candidata
da fondere con la configurazione LEFT completa; non è stato attivato. Il suo
effetto è stato misurato solo ricalcolando la reward sui movimenti salvati.

- [Contabilità e fasi](metrics.json)
- [Fallimenti e contatti](physics/findings.md)
- [Primi25 aggiornamenti LEFT/RIGHT](physics/training_initial_probe.md)
- [Varianti a traiettorie congelate](sensitivity/findings.md)
- [Verifica principale](verification.json)
