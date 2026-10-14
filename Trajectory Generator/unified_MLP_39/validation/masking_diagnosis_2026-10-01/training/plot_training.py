"""Plot existing diagnostics; these are analysis figures, not canonical rollout plots."""
from pathlib import Path
import json
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
data=json.loads((OUT/'summary.json').read_text())
sigma=json.loads((OUT/'tail_sigma_distribution.json').read_text())
keys=list(data)
x=[0,25,50,75,100]
fig, axes=plt.subplots(2,2,figsize=(11,7.5),layout='constrained')
for ax,joint,label in zip(axes[0],['pros_knee_angle','pros_ankle_angle'],['Knee','Ankle']):
    for vis,color in [('visible','#126aa3'),('hidden','#bd5429')]:
        y=[data[k]['groups'][vis][joint+'_rmse_deg'] for k in keys]
        ax.plot(x,[v if v is not None else math.nan for v in y],'-o',color=color,label=vis)
    ax.set(title=f'{label}: physical angle vs imitation target',ylabel='Pooled RMSE [deg]')
    ax.legend()
for vis,color in [('visible','#126aa3'),('hidden','#bd5429')]:
    y=[data[k]['groups'][vis]['reward_per_step'] for k in keys]
    axes[1,0].plot(x,[v if v is not None else math.nan for v in y],'-o',color=color,label=vis)
axes[1,0].set(title='Recorded completed episodes',ylabel='Total return / total steps')
axes[1,0].legend()
for group,label,style in [('all','All endings','-o'),('time_limit','Time-limit only','--s')]:
    y=[sigma[k][group]['quantiles_sigma']['0.99'][0] for k in keys]
    axes[1,1].plot(x,y,style,label=label)
axes[1,1].set(yscale='log',title='Knee Gaussian sigma: P99 in final 50 steps',ylabel='Sigma [raw normalized action units]')
axes[1,1].legend()
for ax in axes.flat:
    ax.set_xticks(x,['Base 81–90','25%','50%','75%','100%'])
    ax.grid(True,alpha=.25)
    ax.set_xlabel('Nominal episode masking probability')
fig.suptitle('Best90 masking continuation: hidden imitation stays poor; exploration tails grow',fontsize=14)
fig.savefig(OUT/'training_diagnosis.png',dpi=180)
print(OUT/'training_diagnosis.png')
