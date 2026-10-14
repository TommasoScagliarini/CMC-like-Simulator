"""Materialize the paired experiment from CP90 and the existing J20 reward."""
import copy, hashlib, json
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
PACKAGE=HERE.parents[1]
ROOT=PACKAGE.parents[1]
SOURCE=PACKAGE/'runs/training/MLP39_LEFT_fresh_best80config_100iter_2026-09-29'
TRAIN=PACKAGE/'runs/training/MLP39_reward_curriculum_2026-10-01'
base=yaml.safe_load((SOURCE/'training_cfg.resolved.yaml').read_text())
exo=yaml.safe_load((PACKAGE/'configs/ex_novo.yaml').read_text())
common=copy.deepcopy(base)
common['model'].update(actor_transfer_warmup=True,freeze_logstd=True,fixed_exploration_sigma=[.01])
common['ppo']=copy.deepcopy(exo['ppo'])
common['ppo']['train_batch_size']=4096
common['parallelism'].update(num_env_runners=13,ray_num_cpus=14,exact_start_sampling=False)
common['target_availability'].update(target_mask_mode='episode',target_availability_probability=.75,target_curriculum_probabilities=[],target_curriculum_iteration_offset=0)
common['simulation']['iterations']=6  # One critic-only iteration, then five actor updates.
common['supervision'].update(checkpoint_every=1,retain_iteration_checkpoints=True,max_minibatch_mean_kl_loss=.01)
common['reward']=copy.deepcopy(exo['reward'])
# These terms are ignored by the ex-novo scalar branch. Match CP90's imitation
# objective in the control instead of silently substituting older defaults.
keys=['imitation_weight','served_imitation_weight','imitation_knee_position_weight','imitation_ankle_position_weight','imitation_knee_velocity_weight','imitation_ankle_velocity_weight','blend_served_imitation','blend_imitation','blend_imitation_tracking']
for k in keys: common['reward'][k]=base['reward'][k]
common['reward']['morphology_termination_mode']='ex_novo'
for mode in ['imitation','ex_novo']:
 cfg=copy.deepcopy(common)
 cfg['reward']['reward_mode']=mode
 cfg['run']={'output_dir':str(TRAIN/mode/'stage01'), 'init_actor_from':str(SOURCE/'rl_module_best'),'resume_from':None}
 cfg['provenance']={'experiment':'paired_reward_curriculum_CP90','source_actor':str(SOURCE/'rl_module_best'),'source_sha256':hashlib.sha256((SOURCE/'rl_module_best/module_state.pkl').read_bytes()).hexdigest(),'common_runtime':'CP90 dynamics/start; J20 FSM gates; identical morphology contract termination','reward_source':str(PACKAGE/'configs/ex_novo.yaml'),'control_imitation_weights':'CP90; inactive in ex_novo','planned_actor_updates_per_stage':5,'max_stage_extension_updates':5,'max_actor_updates_per_arm':40,'seed':123}
 (HERE/f'{mode}.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
 smoke=copy.deepcopy(cfg)
 smoke['ppo'].update(train_batch_size=128,minibatch_size=64)
 smoke['parallelism'].update(num_env_runners=2,ray_num_cpus=3)
 smoke['simulation'].update(iterations=2)
 smoke['run']['output_dir']=str(TRAIN/'smoke'/mode)
 (HERE/f'smoke_{mode}.yaml').write_text(yaml.safe_dump(smoke,sort_keys=False))
files=list((HERE/'before').glob('*.py'))+list((SOURCE/'rl_module_best').rglob('*'))
manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()}
(HERE/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Prepared paired configs: only reward_mode/output identities differ; source remains read-only.')
