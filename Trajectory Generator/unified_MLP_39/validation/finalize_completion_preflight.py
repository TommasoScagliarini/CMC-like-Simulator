"""Validate completed physical diagnostics and publish the continuation gate.

No training is launched here. Missing/failed evidence prevents the gate file.
"""
from __future__ import annotations
import json
import hashlib
from pathlib import Path
import sys

TG=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(TG))
from unified_MLP_39.validation.analyze_completion_diagnosis import analyze

ROOT=TG.parent
PACKAGE=TG/'unified_MLP_39'
BASE=PACKAGE/'validation/completion_diagnostics_2026-09-22'
PLAN=PACKAGE/'runs/plans/completion_continuation_2026-09-22'


def read(path):
    return json.loads(path.read_text())


def main():
    diagnosis=read(BASE/'diagnosis_results.json')
    assert diagnosis['ok'] and len(diagnosis['rows'])==11
    assert {(r['noise_scale'],r['noise_seed']) for r in diagnosis['rows']} == {(0.,123)} | {(s,n) for s in (1.,.5) for n in range(123,128)}
    telemetry=read(BASE/'telemetry_preflight/physical_ab_verification.json')
    assert telemetry['passed']
    telemetry_unit=read(BASE/'telemetry_preflight/test_receipt.json')
    assert telemetry_unit['ok'] and telemetry_unit['failures']==telemetry_unit['errors']==0
    cost=read(BASE/'telemetry_preflight/real_metrics_benchmark.json')
    assert cost['passed'] and cost['uses_real_rllib_metrics_logger']
    assert hashlib.sha256((PACKAGE/'episode_telemetry.py').read_bytes()).hexdigest()==cost['observer_source_sha256']
    analyzer=read(BASE/'telemetry_analyzer_preflight/preflight_receipt.json')
    assert analyzer['passed']
    assert hashlib.sha256((PACKAGE/'validation/analyze_episode_telemetry.py').read_bytes()).hexdigest()==analyzer['analyzer_sha256']
    assert read(BASE/'deterministic_replay_verification.json')['passed']
    optimizer=read(BASE/'optimizer_resume_audit.json')
    assert optimizer['restore_probes']['40']['passed']
    assert optimizer['restore_probes']['40']['parameters_unchanged']
    # The real-learner Adam+KL combined preflight is produced by its audit script.
    combined=read(BASE/'combined_resume_preflight.json')
    assert combined['passed'] and combined['ppo_training_state']['passed']
    orchestration=read(PLAN/'orchestration_verification.json')
    assert orchestration.get('ok', orchestration.get('passed',False))
    assert hashlib.sha256((PACKAGE/'validation/run_completion_continuation.py').read_bytes()).hexdigest()==orchestration['runtime_script_sha256']
    analysis=analyze(PACKAGE/'runs/rollout/completion_diagnostics_2026-09-22_iter40',BASE)
    gate={
        'approved_to_start':True,
        'authorization':'User: ok procedi — approved diagnosis, telemetry verification and sequential continuation40→100.',
        'diagnostics_completed':True,'telemetry_preflight_passed':True,
        'optimizer_preflight_passed':True,'adaptive_kl_preflight_passed':True,
        'evidence':{'diagnosis':str(BASE/'diagnosis_results.json'),
                    'telemetry':str(BASE/'telemetry_preflight/physical_ab_verification.json'),
                    'optimizer':str(BASE/'combined_resume_preflight.json'),
                    'orchestration':str(PLAN/'orchestration_verification.json')},
        'training_started_by_this_script':False,
    }
    target=PLAN/'preflight_ready.json'
    assert not target.exists(), 'Gate already exists; inspect instead of overwriting'
    target.write_text(json.dumps(gate,indent=2)+'\n')
    lines=['# Diagnosi del completamento e ripresa della 39D — 22 settembre 2026','',
        'Obiettivo: indagare il 38,8% di episodi completi (19/49 nelle iterazioni 36–40) prima della prosecuzione fino a 100.',
        'La diagnosi usa il checkpoint 40 fisso, start invariato, prescribed/clock visibili, FSM v3 e identica dinamica.', '',
        '## Diagnosi del rumore','',
        '| Scala della deviazione standard | Episodi completi | Passi medi | Return medio |',
        '|---|---:|---:|---:|']
    for group in analysis['groups']:
        lines.append(f"|{group['noise_scale']:g}|{group['complete']}/{group['episodes']}|{group['mean_steps']:.1f}|{group['mean_return']:.2f}|")
    lines+=['',
        'Cinque semi per livello stocastico; gli stessi campioni gaussiani sono usati a scala 1 e 0,5, con identiche osservazioni iniziali. '
        'La scala modifica solo la valutazione. Questo è un controllo esplorativo, non una prova di equivalenza statistica né una stima precisa della probabilità di successo.',
        'Il rollout deterministico riproduce esattamente tutti i 500 record del precedente rollout 40. '
        'Il dato di training include policy che cambiano e alcuni episodi che attraversano aggiornamenti: non è direttamente equivalente ai cinque episodi di una policy fissa.', '',
        '## Correzioni della ripresa','',
        'È stato dimostrato un difetto nel ripristino Adam: step 2025 al checkpoint 25, poi 81 a 26 e 1215 a 40. '
        'RLlib aveva ripristinato i pesi, lasciando i momenti dell’ottimizzatore scollegati dai parametri. '
        'Il difetto può influire sulle iterazioni 26–40; non spiega da solo il divario già presente prima della 25. La verifica precedente dei pesi non aveva rilevato questo problema.',
        'La nuova ripresa conserva i momenti effettivamente salvati a 40, verifica associazione e digest, e non modifica i pesi. '
        'Non ricostruisce una storia ipotetica: a 41 il contatore Adam atteso è 1296.',
        'Anche il coefficiente KL adattivo viene ora salvato esplicitamente in un sidecar legato all’hash del checkpoint. '
        'Per il vecchio checkpoint 40 è verificato il valore storico 0,20000000298023224. '
        'Le riprese 60/80 useranno il sidecar. Learning rate ed entropy rimangono costanti e verificati.',
        'Gli RNG globali e gli episodi OpenSim in corso non sono recuperabili dai checkpoint storici; la prosecuzione non coincide bit per bit con un processo mai interrotto.', '',
        '## Telemetria e risorse','',
        'La registrazione osserva soltanto dati già prodotti dal training: esito per episodio, riepiloghi e ultimi 50 passi. '
        'La memoria è limitata e i file sono separati per worker/processo. Nessuna inferenza aggiuntiva o estrazione casuale.',
        'Il confronto fisico A/B con la stessa sequenza stocastica ha superato il controllo di identità delle transizioni. '
        'Il sink ha richiesto 46,08 ms su 542,76 s di simulazione (0,00849%); il benchmark con MetricsLogger RLlib reale misura circa 57,78 microsecondi CPU aggiuntivi per passo. '
        'I costi misurati e i limiti del benchmark sono riportati nelle ricevute physical_ab_verification.json e real_metrics_benchmark.json. '
        'Questo misura un costo piccolo, senza garantire assenza assoluta di rallentamenti su un sistema condiviso.', '',
        '## Strategia di esecuzione','',
        'Prosecuzione 40→60→80→100 con la ricetta concordata invariata. '
        'Il training viene chiuso prima di ogni valutazione: un rollout deterministico e dieci stocastici, più i sette grafici canonici salvati in plot/. '
        'Durante il training restano soltanto telemetria e controlli leggeri. '
        'Il processo si ferma se falliscono i controlli di ripristino, integrità dello storico, batch 4096, prescribed visibile o copertura della telemetria.',
        f'Stato dell’esecuzione: `{PLAN.relative_to(ROOT)}/status.json`. '
        'Questa verifica preliminare non avvia direttamente il training e non ne dichiara il completamento.', '',
        '## File e verifiche','',
        '- `unified_MLP_39/train_ppo_mlp.py`, `optimizer_resume.py`, `learner_state_resume.py`: ripristino verificato Adam/KL.',
        '- `unified_MLP_39/tb_logging.py`, `episode_telemetry.py`: telemetria opzionale.',
        '- `unified_MLP_39/validation/completion_noise_rollout.py`, `run_completion_diagnosis.py`, `analyze_completion_diagnosis.py`: valutazioni appaiate e analisi.',
        '- `unified_MLP_39/validation/run_completion_continuation.py`: sequenza durevole, isolamento delle fasi e audit.',
        '- Test mirati di optimizer/KL e campionamento: 9 passati; test della telemetria e orchestratore nelle rispettive ricevute.',
        '- Dodici rollout fisici preliminari: uno deterministico, dieci stocastici e una ripetizione per il confronto A/B.', '',
        'TODO: completare le tappe 60/80/100 e interpretare esiti, penetrazione e cambi di policy usando i nuovi dati per episodio.', '']
    report=ROOT/'reports/user/2026-09-22_diagnosi_completamento_e_ripresa_unified39.md'
    report.write_text('\n'.join(lines))
    print(json.dumps({'passed':True,'gate':str(target),'report':str(report),'groups':analysis['groups']},indent=2))


if __name__=='__main__': main()
