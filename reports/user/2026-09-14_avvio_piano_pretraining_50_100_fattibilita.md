# Avvio del piano pretraining 50→100 e fattibilità a 39 ingressi

Data: 2026-09-14. Instruction check token: `CMC_AGENT_OK_2026`.

## Richiesta e stato

L'utente ha autorizzato l'intera sequenza: ripresa dalla V1 a5 fino a50
iterazioni totali, valutazione, altre50 fino a100, valutazione finale e
test di fattibilità. Sono quindi95 nuovi aggiornamenti della V1.

**Il lavoro è in esecuzione; i traguardi50/100 e la campagna fisica39 non
sono ancora conclusi.** Lo stato aggiornato è in
[STATUS.md](<../../Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/STATUS.md>)
e nel relativo `execution_state.json`, che prevalgono su questo resoconto
di avvio per i conteggi correnti.

## Resume e strategia

Il primo avvio nel sandbox si è fermato prima di Ray per un errore
`psutil/sysctl` macOS, senza update. Il rilancio con i permessi runtime
ha ripristinato RLlib iteration5/logical5, proseguendo da6/50. Digest
actor, medie e critic sono identici al checkpoint5. L'unica differenza
semantica della configurazione è il totale5→50; reward, PPO, fisica,
detector e contratto35 restano quelli del pilot.

Il checkpoint5 immutabile, la configurazione e i risultati precedenti sono
conservati. Un manifest controlla anche gli hash delle sorgenti runtime e
dei preset. La continuazione usa lo stesso run
`unified_MLP/runs/training/pretraining_v1_pen1_pilot_2026-09-14`.

Alle19:47 CEST è stato avviato un coordinatore persistente, PID14612,
che attende il processo di training già attivo, esegue le due valutazioni
e prosegue automaticamente con la fase5. Su macOS `caffeinate -i -w14612`
impedisce lo sleep inattivo per la durata del coordinatore. Comandi, PID,
log e stato sono salvati nella directory del piano.

Il coordinatore verifica history senza buchi, checkpoint coerenti, fase
imitativa, bilanciamento dei tre start e report completi. Un errore tecnico
arresta i passaggi dipendenti; un esito negativo di cammino resta invece un
risultato da registrare. Non dichiara completato l'intero piano prima della
ricevuta finale della fase5.

## Valutazioni e fattibilità

Alle milestone50/100 si valutano ultimo checkpoint e best per return di
training, con tie a favore dell'iterazione più precoce. Best non implica
qualificazione. Si riusano CP5 e i candidati valutati a50 solo dopo controlli
di provenienza e compatibilità; altrimenti CP5 viene rivalutato. I report
separano tracking actual/served/prescribed, velocità, avvio/regime, cicli,
durata, terminazioni, penetrazione, SEA e finestre temporali comuni.

Il [protocollo39](../plans/2026-09-14_protocollo_fattibilita_unificata39.md)
fissa il pilot A/B/C a50 iterazioni per braccio, seed123, disponibilità
sempre1/sempre0/curriculum1,.75,.5,.25,0. B/C ricevono poi11 iterazioni
ex-novo: warm-up1 e PPO10. Repliche456/789 sono condizionate al segnale
esplorativo preregistrato. Questi budget si aggiungono alle100 della V1.

La directory runtime `unified_MLP_39` **non è ancora stata creata**: verrà
materializzata dal bundle dopo la valutazione100. Prima di A/B/C si eseguono
diagnostica osservabilità V1, controlli software e smoke fisico39. La V1,
la baseline, il runtime condiviso e il plugin SEA restano preservati.

## File introdotti e verifiche

Sotto `Trajectory Generator/unified_MLP/` sono stati aggiunti:

- `validation/run_authorized_plan.py`: coordinamento persistente e controlli
  dei passaggi, con stato aggiornato periodicamente;
- `validation/evaluate_training_milestone.py`: selezione, rollout, analisi,
  tabelle e figure delle milestone;
- `validation/run_feasibility39.py`: campagna differita, smoke, confronti,
  copie sigma per valutazione e controlli del rumore applicato;
- `validation/feasibility39_bundle/`: generatore della variante isolata,
  schema39/full88, normalizzazione fissa, maschera per episodio, curriculum,
  persistenza RNG e conteggi effettivi di disponibilità;
- `validation/analyze_observability.py`: ridge su episodi separati per start,
  target riallineati al pre-step, osservazioni correnti e storie causali;
- i relativi test in `tests/`.

Suite completa: **165 test superati in9.11s**. Dopo il monitoraggio periodico:
18 test dell'orchestrazione superati. Due ulteriori regressioni su copie sigma
e rumore accoppiato sono state aggiunte: tutti i10 test del relativo file
passano. Non si presenta questo come una nuova esecuzione complessiva167.

I test del bundle includono loss PPO reale, aggiornamenti Adam identici a
target nascosti differenti e reload in processo nuovo. Le materializzazioni
di test avvengono solo in directory temporanee. Una verifica offline
dell'analizzatore sulle tre tracce CP5 produce1051 campioni, tutti di avvio:
regime non valutabile. Questo non costituisce la campagna fisica39 né una
prova di impossibilità informativa.

## TODO in esecuzione o pendenti

- [ ] Completare V1 fino a50 e verificarne i risultati intermedi.
- [ ] Completare la stessa V1 fino a100 e confrontare5/50/100.
- [ ] Materializzare39 dopo valutazione100; superare smoke e isolamento.
- [ ] Completare A/B/C, trasferimento B/C ed eventuali repliche preregistrate.
- [ ] Interpretare e rendicontare tutti gli esiti e costi, senza confondere
  fattibilità, successo tecnico e parità con la baseline.
- [ ] Verifica live Windows, separata dalla campagna macOS in esecuzione.
