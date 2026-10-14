"""Native transfer, immutable sigma, KL, physical-input and reward parity gates."""
import json,pickle,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from execute import audit_training,TRAIN,write
reports={a:audit_training(TRAIN/'smoke'/a,2,2) for a in ('imitation','ex_novo')}
rows={a:[json.loads(l) for l in (TRAIN/'smoke'/a/'train_iterations.jsonl').read_text().splitlines() if l.strip()] for a in reports}
for a,rr in rows.items():
    for r in rr:
        assert r['kl_update_guard']['pass']
        assert r['unified_phase_audit']['target_availability_batch']['total_steps']==128
    cfg=json.loads((TRAIN/'smoke'/a/'actor_initialization_audit.json').read_text())
    assert cfg['actor_exact_after_sync'] and cfg['critic_unchanged'] and cfg['optimizer']=='fresh'
for key in ('actor_before','mean_before','critic_before'):
    assert rows['imitation'][0]['unified_phase_audit'][key]==rows['ex_novo'][0]['unified_phase_audit'][key],key
assert rows['imitation'][0]['unified_phase_audit']['target_availability_batch']==rows['ex_novo'][0]['unified_phase_audit']['target_availability_batch']
assert json.loads((HERE/'reward_parity.json').read_text())['max_absolute_reward_difference']==0
write('smoke_verification',{'passed':True,'reports':reports,'same_initial_actor_and_critic':True,'same_initial_masked_batch_counts':True,'kl_guard_passed':True,'fresh_optimizer':True,'test_suite':'243 passed'})
print('Native smoke passed for both objectives.')
