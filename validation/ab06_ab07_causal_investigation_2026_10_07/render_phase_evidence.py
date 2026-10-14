"""Render the two-panel phase/contact figure from frozen JSON only.

No NPZ, motion/model read, new FK, integration or source change. Uses a temporary
Matplotlib configuration cache on Windows/macOS. Existing outputs are preserved.
"""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile

os.environ.setdefault('MPLCONFIGDIR', tempfile.mkdtemp(prefix='cmc_phase_mpl_'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

AUDIT = Path(__file__).resolve().parent
ROOT = AUDIT.parents[1]
OUT = ROOT/'plot/AB06_AB07_causal_investigation_2026_10_07'
SOURCES = {'phase_budget':AUDIT/'alternative/phase_contact_budget.json',
           'foot_pose':AUDIT/'model/foot_pose_decomposition.json'}


def fp(p):
    p = Path(p).resolve()
    return {'path':p.relative_to(ROOT).as_posix(), 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}


def main():
    image_paths = {kind:OUT/f'phase_contact_evidence.{kind}' for kind in ('png','svg')}
    manifest = AUDIT/'phase_contact_evidence_manifest.json'
    if manifest.exists() or any(p.exists() for p in image_paths.values()):
        raise FileExistsError('Preserve existing figure and manifest; version separately.')
    before = {name:fp(p) for name,p in SOURCES.items()}
    data = {name:json.loads(p.read_text(),parse_constant=lambda value:(_ for _ in ()).throw(ValueError(value)))
            for name,p in SOURCES.items()}
    if data['phase_budget']['status'] != 'PASS' or not data['foot_pose']['status'].startswith('PASS_'):
        raise AssertionError('Plot requires the verified phase and foot-pose evidence.')
    series = {}
    for subject in ('AB06','AB07'):
        bins = data['phase_budget']['subjects'][subject]['filtered_IK']['pooled_stance_phase_bins']
        poses = data['foot_pose']['subjects'][subject]['pooled_stance_phase_bins']
        if len(bins) != 20 or len(poses) != 20:
            raise ValueError('Figure requires exactly20 frozen stance bins.')
        force, depth, centers = [], [], []
        for i,(b,p) in enumerate(zip(bins,poses)):
            if b['phase_start'] != i/20 or b['phase_end'] != (i+1)/20:
                raise AssertionError('Budget phase grid differs.')
            if (p['phase_start'],p['phase_end']) != (b['phase_start'],b['phase_end']):
                raise AssertionError('Foot decomposition phase grid differs.')
            if abs(p['positive_penetration_m']-b['basis07_positive_penetration_mean_m']) > 1e-14:
                raise AssertionError('Two independent saved positive-depth summaries disagree.')
            centers.append(50*(b['phase_start']+b['phase_end']))
            force.append(b['force_mean_N'])
            depth.append(1000*p['positive_penetration_m'])
        arrays = {name:np.array(values) for name,values in [('phase_centers_percent',centers),('force_N',force),('positive_penetration_mm',depth)]}
        if not all(np.all(np.isfinite(a)) for a in arrays.values()) or min(depth) < 0 or min(force) < 0:
            raise ValueError('Figure values are nonfinite or physically invalid.')
        series[subject] = {name:a.tolist() for name,a in arrays.items()}
    if series['AB06']['phase_centers_percent'] != series['AB07']['phase_centers_percent']:
        raise AssertionError('Subject phase centers differ.')
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,
                         'axes.spines.top':False,'axes.spines.right':False,
                         'axes.titlesize':13,'axes.labelsize':11,
                         'svg.fonttype':'none','savefig.facecolor':'white'})
    fig, axes = plt.subplots(2,1,figsize=(10,8),sharex=True)
    colors = {'AB06':'#0072B2','AB07':'#D55E00'}
    for subject,style in [('AB06','-'),('AB07','--')]:
        x = series[subject]['phase_centers_percent']
        for ax,key in zip(axes,('force_N','positive_penetration_mm')):
            ax.plot(x,series[subject][key],label=subject,color=colors[subject],
                    linestyle=style,linewidth=2.4,marker='o',markersize=3.8)
    titles = ['Forza verticale del contatto virtuale','Sfera 07: penetrazione positiva nel piano di contatto']
    labels = ['Forza verticale [N]','Penetrazione positiva [mm]']
    for ax,title,label in zip(axes,titles,labels):
        ax.set_title(title,loc='left',pad=11)
        ax.set_ylabel(label)
        ax.set_xlim(0,100)
        ax.set_ylim(bottom=0)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
        ax.grid(axis='y',alpha=.25,linewidth=.8)
        ax.set_axisbelow(True)
        ax.tick_params(axis='both',length=3,color='#777777')
    axes[0].legend(loc='upper right',frameon=False,ncol=2)
    axes[1].set_xlabel('Fase dello stance misurato [%]')
    axes[1].set_xticks(np.arange(0,101,20))
    fig.suptitle('Contatto durante lo stance: AB06 e AB07',x=.105,y=.972,ha='left',fontsize=17,fontweight='bold')
    fig.text(.105,.935,'Replay IK filtrato · tre cicli nominali per soggetto · medie di 20 intervalli di fase',fontsize=11,color='#555555')
    fig.text(.105,.137,'Cicli aggregati pesando la durata dello stance; i punti rappresentano le medie degli intervalli.',fontsize=10,color='#444444')
    fig.text(.105,.110,'Fase normalizzata solo per descrivere la forma. Impulsi calcolati ai tempi originali.',fontsize=10,color='#444444')
    fig.text(.105,.083,'Contatto: protesi virtuale. Stance: eventi della GRF sperimentale del soggetto healthy.',fontsize=10,color='#444444')
    fig.subplots_adjust(left=.105,right=.97,top=.865,bottom=.215,hspace=.42)
    OUT.mkdir(parents=True,exist_ok=True)
    title = 'Contatto durante lo stance: AB06 e AB07'
    for kind,path in image_paths.items():
        metadata = {'Title':title,'Description':'Forza verticale e penetrazione positiva della sfera07 su20bin stance; solo JSON congelati.'}
        if kind == 'svg':
            metadata['Creator'] = 'Matplotlib'
        fig.savefig(path,format=kind,dpi=220,metadata=metadata)
    plt.close(fig)
    after = {name:fp(p) for name,p in SOURCES.items()}
    if before != after:
        raise AssertionError('A frozen JSON source changed during rendering.')
    result = {'created_utc':datetime.now(timezone.utc).isoformat(),'instruction_check_token':'CMC_AGENT_OK_2026',
              'status':'PASS','helper':fp(Path(__file__)),'source_fingerprints_before':before,
              'source_fingerprints_after':after,'sources_unchanged':True,
              'output_files':{kind:fp(path) for kind,path in image_paths.items()},
              'figure':{'panels':2,'size_inches':[10,8],'PNG_dpi':220,'phase_bins':20,
                        'force_source':'phase_budget.subjects.*.filtered_IK.pooled_stance_phase_bins.force_mean_N',
                        'penetration_source':'foot_pose.subjects.*.pooled_stance_phase_bins.positive_penetration_m*1000',
                        'cross_source_positive_penetration_agreement_tolerance_m':1e-14,
                        'line_convention':'Lines join bin means for descriptive shape; no spline smoothing or physical waveform reconstruction.',
                        'scope':'Only saved JSON; no model/NPZ read, new FK, integration, inference, tuning, raw-depth panel or canonical-plot replacement.'},
              'plotted_series':series,
              'limits':['Phase normalization is descriptive and preserves native-time impulse calculations.',
                        'These virtual-contact traces are not experimental forces measured from this prosthetic hardware.',
                        'Positive penetration is used; a large difference between two negative raw depths is not depicted as contact-force-producing penetration.']}
    with manifest.open('x',encoding='utf-8') as f:
        json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print('PASS',fp(manifest))
    for path in image_paths.values():
        print(path)


if __name__ == '__main__':
    main()
