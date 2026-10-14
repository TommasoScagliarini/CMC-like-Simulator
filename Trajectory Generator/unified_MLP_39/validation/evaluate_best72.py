"""Evaluate the verified best72 with the completed 60/80/100 noise protocol.

Uses immutable milestone weights, four single-thread evaluators at most, and
the existing watchdog, plots and paired-noise analyzer. Never starts training.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import importlib.util
import os
from pathlib import Path
import signal
import sys

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('completion_pipeline', HERE/'run_completion_continuation.py')
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)
PACKAGE, REPO, RUN = pipeline.PACKAGE, pipeline.REPO, pipeline.RUN
PREVIOUS = PACKAGE/'runs/plans/completion_recovery42_2026-09-22'
PLAN = PACKAGE/'runs/plans/evaluation_best72_2026-09-22'
ROLLOUTS = PACKAGE/'runs/rollout/MLP39_completion_2026-09-22_best072'
MODULE = RUN/'milestone_iteration_000072/rl_module_last'


def prepare():
    pipeline.require(not PLAN.exists() and not ROLLOUTS.exists(), 'Evaluation already prepared or outputs exist')
    pipeline.require(pipeline.read(PREVIOUS/'status.json')['status']=='completed', 'Prior pipeline not completed')
    pipeline.source_guard(PREVIOUS)
    meta=pipeline.read(RUN/'checkpoint_best_meta.json')
    pipeline.require(meta['logical_iteration']==72, 'Best checkpoint no longer 72')
    for name in ('module_state.pkl','unified_contract.json','training_cfg.resolved.yaml'):
        pipeline.require(pipeline.digest(MODULE/name)==pipeline.digest(RUN/'rl_module_best'/name),
                         f'Best and milestone72 differ: {name}')
    PLAN.mkdir(parents=True)
    protocol={'checkpoint':str(MODULE),'logical_iteration':72,'environment_seed':123,
              'noise_seeds':list(range(123,128)),'noise_scales':[0.,1.,.5],
              'max_concurrent':4,'steps':500,'training_concurrent':False,
              'deterministic_outputs':True,'canonical_plot_root':str(REPO/'plot'),
              'rollout_root':str(ROLLOUTS),'prepared_at':datetime.now().astimezone().isoformat()}
    pipeline.write(PLAN/'protocol.json',protocol)
    manifest=pipeline.read(PREVIOUS/'source_hashes.json')
    paths=[Path(__file__),PLAN/'protocol.json',RUN/'checkpoint_best_meta.json',RUN/'train_iterations.jsonl',RUN/'summary.json']
    paths.extend(p for p in MODULE.rglob('*') if p.is_file())
    paths.extend(p for p in (RUN/'rl_module_best').rglob('*') if p.is_file())
    for p in paths:
        manifest[pipeline.relative(p)]=pipeline.digest(p)
    pipeline.write(PLAN/'source_hashes.json',manifest)
    pipeline.write(PLAN/'freeze_receipt.json',{'manifest_sha256':pipeline.digest(PLAN/'source_hashes.json'),
                   'files':len(manifest),'best_matches_milestone72':True})
    pipeline.event(PLAN,'prepared',checkpoint=str(MODULE),cases=11)


def check():
    pipeline.source_guard(PLAN)
    pipeline.require(not pipeline.active_ray_processes(include_rollouts=True),
                     'Other training/rollout processes are active')
    pipeline.require(not ROLLOUTS.exists(), 'Refusing existing rollout output')
    print('Verified immutable best72, unchanged runtime, and no concurrent training/evaluation.',flush=True)


def finish_report(result):
    analysis=pipeline.read(PLAN/'analysis/noise_analysis.json')
    table=[]
    for it in (60,72,80,100):
        data=analysis if it==72 else pipeline.read(PREVIOUS/f'analysis_iter{it:03d}/noise_analysis.json')
        det=next(e for e in data['episodes'] if e['noise_scale']==0)
        groups={g['noise_scale']:g for g in data['groups']}
        table.append({'iteration':it,'deterministic_return':det['episode_return'],
            'deterministic_rmse_deg':det['rmse_degrees'],'penetration_mm':1000*det['grf_penetration_max_m'],
            'cycles':det['phase_valid_cycle_count'],'full_noise_complete':groups[1.]['complete'],
            'half_noise_complete':groups[.5]['complete']})
    result['comparison']=table
    pipeline.write(PLAN/'results.json',result)
    lines=['# Valutazione fisica del best72 — 22 settembre 2026','',
           'Undici rollout: deterministico e cinque seed appaiati a rumore originale/dimezzato. Nessun nuovo training.',
           '', '| Checkpoint | Return deterministico | RMSE ginocchio | RMSE caviglia | Penetrazione | Cicli validi | Completi σ1 | Completi σ0,5 |',
           '|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in table:
        q=r['deterministic_rmse_deg']
        lines.append(f"| {r['iteration']} | {r['deterministic_return']:.2f} | {q['pros_knee_angle']:.2f}° | {q['pros_ankle_angle']:.2f}° | {r['penetration_mm']:.2f} mm | {r['cycles']} | {r['full_noise_complete']}/5 | {r['half_noise_complete']}/5 |")
    lines+=['','Il best72 è selezionato dal return di training; questo confronto misura separatamente la policy salvata.',
            'Cinque seed a uno start non dimostrano equivalenza statistica. RMSE su episodi interrotti coprono durate differenti; per il confronto delle scale usare anche i prefissi comuni in analysis/noise_analysis.json.',
            '', '## Grafici canonici', '']
    lines += [f'- [{Path(p).name}](<{p}>)' for p in result['canonical_plots']]
    lines += ['', '## Verifiche', '',
              '- Best e milestone72 coincidono per pesi, contratto e configurazione.',
              '- Checkpoint e sorgenti conservano gli hash iniziali.',
              '- Target visibili, trace complete, stessi seed e campioni gaussiani appaiati.',
              '- Sette figure salvate sotto plot/ del root.',
              '- Nessuna ripresa o modifica del training.']
    (PLAN/'comparison.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')


def run():
    check()
    with (PLAN/'execution.lock').open('x') as stream:
        stream.write(str(os.getpid())+'\n')
    env={**os.environ,**pipeline.THREAD_ENV,'PYTHONUNBUFFERED':'1','MPLCONFIGDIR':str(PLAN/'mpl_cache')}
    env.pop('UNIFIED39_EPISODE_TELEMETRY_DIR',None)
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    result={'checkpoint':str(MODULE),'iteration':72,'evaluations':[]}
    try:
        folder=ROLLOUTS/'deterministic'
        pipeline.run_command(pipeline.evaluation_command(MODULE,folder,0.,123,123),
            PLAN/'deterministic.log',PLAN,'deterministic_best72',env,timeout=3900)
        metric=pipeline.rollout_metrics(folder);metric.update(noise_scale=0.,noise_seed=123)
        result['evaluations'].append(metric);pipeline.write(PLAN/'results.json',result)
        pipeline.run_command([sys.executable,str(HERE/'render_canonical_rollout.py'),'--rollout',str(folder)],
            PLAN/'canonical_plots.log',PLAN,'plotting_best72',env,timeout=600)
        figures=pipeline.read(folder/'plots/plot_receipt.json')['figures']
        pipeline.require(len(figures)==7,'Missing canonical figures')
        for f in figures:
            Path(f).resolve().relative_to((REPO/'plot').resolve())
            pipeline.require(Path(f).is_file(),'Missing canonical plot')
        result['canonical_plots']=figures;pipeline.write(PLAN/'results.json',result)
        cases=[(scale,seed) for seed in range(123,128) for scale in (1.,.5)]
        jobs=[]
        for scale,seed in cases:
            name=f'noise_{scale:g}_seed{seed}'
            jobs.append((name,pipeline.evaluation_command(MODULE,ROLLOUTS/name,scale,seed,123),PLAN/f'{name}.log'))
        pipeline.run_parallel_evaluations(jobs,PLAN,72,env)
        for scale,seed in cases:
            folder=ROLLOUTS/f'noise_{scale:g}_seed{seed}'
            meta=pipeline.read(folder/'noise_diagnostic.json')
            pipeline.require((meta['noise_scale'],meta['noise_seed'],meta['environment_seed'])==(scale,seed,123),
                             'Noise provenance mismatch')
            metric=pipeline.rollout_metrics(folder);metric.update(noise_scale=scale,noise_seed=seed)
            result['evaluations'].append(metric)
        pipeline.run_command([sys.executable,str(HERE/'analyze_completion_diagnosis.py'),
            '--rollout-root',str(ROLLOUTS),'--output',str(PLAN/'analysis')],
            PLAN/'analysis.log',PLAN,'analyzing_best72',env,timeout=600)
        pipeline.source_guard(PLAN)
        finish_report(result)
        pipeline.event(PLAN,'completed',cases=11,comparison=str(PLAN/'comparison.md'),results=str(PLAN/'results.json'))
    except BaseException as exc:
        pipeline.stop_child()
        pipeline.event(PLAN,'failed',error=repr(exc))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--prepare',action='store_true')
    group.add_argument('--check',action='store_true')
    group.add_argument('--run',action='store_true')
    args=parser.parse_args()
    prepare() if args.prepare else check() if args.check else run()
